//! virtual_esp32 (node name sim_esp32): the virtual car's ESP32, running the Rust firmware's own logic.
//!
//! A drop-in for sim_esp32 (aav_sim_models, the 2025-26 C++ in Python), chosen by
//! sim.launch.py esp32_impl:=rust. The same TCP port for legacy_uart_bridge, the same
//! output topic, the same status lines on the same /ws: the bridge and the Orin's page
//! (aav_web --esp32 ws://127.0.0.1:8766/ws) can't tell which runs. Its logic is the
//! virtual-esp32 crate (no ROS): spine-board's SpineBoard, spine-core, legacy-link's
//! SteerCal, bench-core's controls and tests, status-lines. On Ravensim's clock
//! (use_sim_time), 10 ms a tick, as on the board.
//!
//!   legacy_uart_bridge ──TCP, its frames──> virtual_esp32 ──> sim_adapter ──> Ravensim
//!                      <──the uplink (uplink:=true)──┘
//!
//! Listens     TCP 127.0.0.1:port (47000 + the domain, from the launch file), one
//!             connection at a time: the bridge's bytes in; the uplink frames back, with
//!             uplink true
//! Subscribes  /vehicle/wheels       aav_interfaces/WheelSpeeds  the rear wheels' mean: the
//!                                   speed the spine's wheel counter counts
//!             /vehicle/steering     aav_interfaces/ActuatorState  measured: the wheels' angle
//!             /vehicle/brake        aav_interfaces/ActuatorState  measured: the brake's position
//!             /vehicle/sim/jacks    std_msgs/Bool (latched)  the car on stands: CREEP allowed
//!             (the topics are parameters: sim_adapter publishes these with esp32:=true)
//! Publishes   /vehicle/sim/esp32_outputs  aav_interfaces/SpineOutputs, 50 Hz: what the stand-in
//!                                   devices were told: throttle (DAC / 700), brake (the Jrk's
//!                                   travel), reverse, steering (rad; while limp, where the
//!                                   wheels are)
//! Serves      http://127.0.0.1:<status_port>/ (8766; 0: off), on 127.0.0.1 only:
//!               GET  /, /status.json, /health, /config, /scan, /ws   the board's page and its
//!                    status lines, as sim_esp32's
//!               GET  /bench, /bench.json, /trace.json   Controls, Measure, Tests
//!               POST /press?button=, /leave, /test/start?name=, /test/heartbeat, /test/stop
//!                    the bench's writes: only from this computer (Host, Origin) and only on
//!                    the simulator's clock; otherwise 403 (virtual_esp32::page)
//! Calls       nothing
//! Parameters  port (47000), status_port (8766), frame_version (1: today's 13-byte frames;
//!             2: versions 2-4), wheel_speed (true: the spine counts the wheels; false: as
//!             today's board, so the test mode's throttle is refused), uplink (false: as
//!             legacy.yaml), wheels_topic, steering_topic, brake_topic, jacks_topic

use std::{
    io::{ErrorKind, Read, Write},
    net::{TcpListener, TcpStream},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{sync_channel, Receiver, SyncSender, TrySendError},
        Arc, Mutex, MutexGuard,
    },
    thread,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use anyhow::{bail, Context as _, Result};
use rclrs::*;
use ros_env::aav_interfaces::msg::{ActuatorState, SpineOutputs, WheelSpeeds};
use ros_env::builtin_interfaces::msg::Time;
use ros_env::std_msgs::msg::Bool;
use virtual_esp32::page::{Latest, Page, Reply, Request};
use virtual_esp32::ws::text_frame;
use virtual_esp32::{FrameVersion, Measured, Settings, VirtualEsp32};

/// The board's tick (ms).
const TICK_MS: i64 = 10;
/// More than this many ticks behind is a stall or a reset, not catch-up.
const MAX_CATCH_UP: i64 = 10;
/// The uplink's bytes kept for a slow reader, at most.
const UPLINK_KEEP: usize = 4096;
/// Status lines queued for one slow WebSocket reader; more are dropped, as on the board.
const STREAM_QUEUE: usize = 64;

/// Everything the threads share.
struct Shared {
    esp: VirtualEsp32,
    measured: Measured,
    latest: Latest,
    now_ms: u32,
    uplink: Vec<u8>,
    streams: Vec<SyncSender<String>>,
}

type State = Arc<Mutex<Shared>>;

fn lock(state: &State) -> MutexGuard<'_, Shared> {
    // A thread that panicked holding it: carry on with what it left (the spine's own
    // state is whole between calls).
    state.lock().unwrap_or_else(|e| e.into_inner())
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("sim_esp32")?;

    let int = |name: &str, default: i64| -> Result<i64> {
        Ok(node
            .declare_parameter(name)
            .default(default)
            .read_only()?
            .get())
    };
    let flag = |name: &str, default: bool| -> Result<bool> {
        Ok(node
            .declare_parameter(name)
            .default(default)
            .read_only()?
            .get())
    };
    let text = |name: &str, default: &str| -> Result<String> {
        let v: Arc<str> = node
            .declare_parameter(name)
            .default(default.into())
            .read_only()?
            .get();
        Ok(v.to_string())
    };
    let port = u16::try_from(int("port", 47000)?).context("port: 1 to 65535")?;
    let status_port =
        u16::try_from(int("status_port", 8766)?).context("status_port: 0 to 65535")?;
    let frame_version = match int("frame_version", 1)? {
        1 => FrameVersion::V1,
        2..=4 => FrameVersion::V2,
        v => bail!("frame_version:={v}: 1, or 2 (2, 3 and 4 alike)"),
    };
    let settings = Settings {
        frame_version,
        wheel_speed: flag("wheel_speed", true)?,
    };
    let uplink = flag("uplink", false)?;
    let topics = [
        text("wheels_topic", "/vehicle/wheels")?,
        text("steering_topic", "/vehicle/steering")?,
        text("brake_topic", "/vehicle/brake")?,
        text("jacks_topic", "/vehicle/sim/jacks")?,
    ];

    let boot_id = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.subsec_nanos() ^ d.as_secs() as u32);
    let state: State = Arc::new(Mutex::new(Shared {
        esp: VirtualEsp32::new(settings, boot_id),
        measured: Measured {
            steering: f32::NAN,
            brake: f32::NAN,
            ..Measured::default()
        },
        latest: Latest::default(),
        now_ms: 0,
        uplink: Vec::new(),
        streams: Vec::new(),
    }));

    let running = Arc::new(AtomicBool::new(true));
    {
        let running = Arc::clone(&running);
        let commands = Arc::clone(executor.commands());
        ctrlc::set_handler(move || {
            running.store(false, Ordering::SeqCst);
            commands.halt_spinning();
        })?;
    }

    let _keep = measurements(&node, &state, &topics)?;
    let clock = node.get_clock();
    // The writes only on the simulator's clock (use_sim_time): never on a computer's own.
    let simulated = matches!(clock.clock_type(), ClockType::RosTime);

    let bridge = TcpListener::bind(("127.0.0.1", port))
        .with_context(|| format!("127.0.0.1:{port} for legacy_uart_bridge"))?;
    bridge.set_nonblocking(true)?;
    {
        let (state, running) = (Arc::clone(&state), Arc::clone(&running));
        thread::spawn(move || serve_bridge(&bridge, &state, &running, uplink));
    }
    log_info!(
        node.logger(),
        "The Rust ESP32 firmware, standing in: listening on 127.0.0.1:{port} for \
         legacy_uart_bridge (frame version {}, the spine {} the wheels)",
        if frame_version == FrameVersion::V1 {
            "1"
        } else {
            "2-4"
        },
        if settings.wheel_speed {
            "counts"
        } else {
            "doesn't count"
        }
    );
    if status_port != 0 {
        let web = TcpListener::bind(("127.0.0.1", status_port))
            .with_context(|| format!("127.0.0.1:{status_port} for the page"))?;
        let page = Page {
            port: status_port,
            writes: simulated,
        };
        let state = Arc::clone(&state);
        thread::spawn(move || serve_page(&web, &state, page));
        log_info!(
            node.logger(),
            "The board's page: http://127.0.0.1:{status_port}/ (status lines on /ws); the \
             bench: http://127.0.0.1:{status_port}/bench ({})",
            if simulated {
                "its controls on: the simulator's clock, this computer only"
            } else {
                "read only: not on the simulator's clock"
            }
        );
    }

    let outputs = node.create_publisher::<SpineOutputs>("/vehicle/sim/esp32_outputs")?;
    let ticker = {
        let (state, running) = (Arc::clone(&state), Arc::clone(&running));
        thread::spawn(move || -> Result<()> {
            let tick_ns = TICK_MS * 1_000_000;
            let mut last_ns: Option<i64> = None;
            let mut ticks: u64 = 0;
            let mut tick = |nsec: i64| -> Result<()> {
                ticks += 1;
                let now_ms = (nsec / 1_000_000) as u32; // wraps, as the board's clock
                let mut s = lock(&state);
                let measured = s.measured;
                let out = s.esp.tick(now_ms, measured);
                s.now_ms = now_ms;
                s.latest.keep(&out.lines);
                if uplink {
                    s.uplink.extend_from_slice(&out.uplink);
                    let extra = s.uplink.len().saturating_sub(UPLINK_KEEP);
                    s.uplink.drain(..extra);
                }
                s.streams.retain(|tx| {
                    out.lines.iter().all(|line| {
                        !matches!(
                            tx.try_send(line.clone()),
                            Err(TrySendError::Disconnected(_))
                        )
                    })
                });
                drop(s);
                if ticks.is_multiple_of(2) {
                    let mut msg = SpineOutputs::default();
                    msg.header.stamp = stamp_of(nsec);
                    msg.header.frame_id = "base_link".into();
                    msg.throttle = out.drive.throttle;
                    msg.brake = out.drive.brake;
                    msg.reverse = out.drive.reverse;
                    msg.steering = out.drive.steering;
                    outputs.publish(msg)?;
                }
                Ok(())
            };
            while running.load(Ordering::SeqCst) {
                let now = clock.now();
                if now.nsec > 0 {
                    let owed = last_ns.map_or(0, |t| (now.nsec - t) / tick_ns);
                    if last_ns.is_none() || !(0..=MAX_CATCH_UP).contains(&owed) {
                        last_ns = Some(now.nsec);
                        tick(now.nsec)?;
                    } else if let Some(t) = last_ns.as_mut() {
                        for _ in 0..owed {
                            *t += tick_ns;
                            tick(*t)?;
                        }
                    }
                }
                thread::sleep(Duration::from_millis(1));
            }
            Ok(())
        })
    };

    executor.spin(SpinOptions::default()).first_error()?;
    running.store(false, Ordering::SeqCst);
    match ticker.join() {
        Ok(result) => result,
        Err(_) => bail!("the tick loop panicked"),
    }
}

/// Subscriptions that must live as long as the node.
type Keep = Vec<Box<dyn std::any::Any>>;

/// What Ravensim's car measured, from sim_adapter.
fn measurements(node: &Node, state: &State, topics: &[String; 4]) -> Result<Keep> {
    let [wheels, steering, brake, jacks] = topics;
    let wheels = {
        let state = Arc::clone(state);
        node.create_subscription::<WheelSpeeds, _>(wheels.as_str(), move |w: WheelSpeeds| {
            // The rear axle's speed: what the motor's encoder sees.
            lock(&state).measured.speed = (w.rear_left + w.rear_right) / 2.0;
        })?
    };
    let steering = {
        let state = Arc::clone(state);
        node.create_subscription::<ActuatorState, _>(steering.as_str(), move |a: ActuatorState| {
            lock(&state).measured.steering = a.measured;
        })?
    };
    let brake = {
        let state = Arc::clone(state);
        node.create_subscription::<ActuatorState, _>(brake.as_str(), move |a: ActuatorState| {
            lock(&state).measured.brake = a.measured;
        })?
    };
    let jacks = {
        let state = Arc::clone(state);
        node.create_subscription::<Bool, _>(
            jacks.as_str().keep_last(1).reliable().transient_local(),
            move |b: Bool| {
                lock(&state).measured.on_stands = b.data;
            },
        )?
    };
    Ok(vec![
        Box::new(wheels),
        Box::new(steering),
        Box::new(brake),
        Box::new(jacks),
    ])
}

/// The bridge's TCP: one connection at a time, bytes in, the uplink out.
fn serve_bridge(listener: &TcpListener, state: &State, running: &AtomicBool, uplink: bool) {
    let mut buf = [0u8; 4096];
    while running.load(Ordering::SeqCst) {
        let mut client = match listener.accept() {
            Ok((c, _)) => c,
            Err(e) if e.kind() == ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(10));
                continue;
            }
            Err(_) => {
                thread::sleep(Duration::from_millis(100));
                continue;
            }
        };
        if client.set_nonblocking(true).is_err() {
            continue;
        }
        // Like the board: the bridge gone, the spine's own 100 ms watchdog stops the car.
        'connected: while running.load(Ordering::SeqCst) {
            let mut idle = true;
            match client.read(&mut buf) {
                Ok(0) => break 'connected,
                Ok(n) => {
                    lock(state).esp.feed(&buf[..n]);
                    idle = false;
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(_) => break 'connected,
            }
            if uplink {
                let pending = std::mem::take(&mut lock(state).uplink);
                if !pending.is_empty() {
                    match client.write(&pending) {
                        Ok(n) if n < pending.len() => {
                            let mut s = lock(state);
                            let rest = pending[n..].to_vec();
                            s.uplink.splice(0..0, rest);
                        }
                        Ok(_) => {}
                        Err(e) if e.kind() == ErrorKind::WouldBlock => {
                            lock(state).uplink.splice(0..0, pending);
                        }
                        Err(_) => break 'connected,
                    }
                }
            }
            if idle {
                thread::sleep(Duration::from_millis(2));
            }
        }
    }
}

/// The page: a thread a connection.
fn serve_page(listener: &TcpListener, state: &State, page: Page) {
    for stream in listener.incoming().flatten() {
        let state = Arc::clone(state);
        thread::spawn(move || {
            let _ = answer(stream, &state, page);
        });
    }
}

/// One HTTP request: a reply, or the status lines' WebSocket.
fn answer(mut stream: TcpStream, state: &State, page: Page) -> std::io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    let mut head = Vec::new();
    let mut byte = [0u8; 1];
    while !head.ends_with(b"\r\n\r\n") {
        if stream.read(&mut byte)? == 0 || head.len() > 8192 {
            return Ok(());
        }
        head.push(byte[0]);
    }
    let Some(req) = Request::parse(&String::from_utf8_lossy(&head)) else {
        return Ok(());
    };
    let reply = {
        let mut s = lock(state);
        let now_ms = s.now_ms;
        let Shared { esp, latest, .. } = &mut *s;
        page.route(&req, esp, latest, now_ms)
    };
    match reply {
        Reply::Stream(accept) => {
            stream.write_all(
                format!(
                    "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n\
                     Connection: Upgrade\r\nSec-WebSocket-Accept: {accept}\r\n\r\n"
                )
                .as_bytes(),
            )?;
            let (tx, rx) = sync_channel(STREAM_QUEUE);
            lock(state).streams.push(tx);
            stream_lines(stream, &rx)
        }
        body => match body.http() {
            Some(bytes) => stream.write_all(&bytes),
            None => Ok(()),
        },
    }
}

/// The status lines to one reader until it goes; what it sends is read and dropped.
fn stream_lines(mut stream: TcpStream, rx: &Receiver<String>) -> std::io::Result<()> {
    stream.set_nonblocking(true)?;
    let mut sink = [0u8; 512];
    loop {
        match rx.recv_timeout(Duration::from_millis(500)) {
            Ok(line) => {
                stream.set_nonblocking(false)?;
                stream.write_all(&text_frame(&line))?;
                stream.set_nonblocking(true)?;
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => return Ok(()),
        }
        match stream.read(&mut sink) {
            Ok(0) => return Ok(()),
            Ok(_) => {}
            Err(e) if e.kind() == ErrorKind::WouldBlock => {}
            Err(e) => return Err(e),
        }
    }
}

fn stamp_of(nsec: i64) -> Time {
    Time {
        sec: nsec.div_euclid(1_000_000_000) as i32,
        nanosec: nsec.rem_euclid(1_000_000_000) as u32,
    }
}
