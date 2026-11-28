//! The virtual car's ESP32: the Rust firmware's own logic on a laptop, with no ROS.
//!
//! The board program (`firmware/spine/boards/esp32-umod`) builds a [`SpineBoard`] from the
//! chip's parts; this builds the same one from a laptop's ([`parts`]): the simulator's
//! clock, a wheel counter turned by the speed the simulator measured, the healthy
//! stand-in devices ([`StandInBus`]) and the Motor Pi's serial line as byte buffers. The
//! page's bench is `bench-core`'s, unchanged, and the status lines `status-lines`'.
//!
//! ```text
//!  feed(bytes) ─► LegacyLink ─┐                 ┌─► Outputs: throttle, brake, reverse, steering
//!  press(button) ─► Controls ─┼─► SpineBoard ───┤   (what the stand-in devices were told)
//!  start_test ─► Run ─────────┘    (spine-core) ├─► the uplink's bytes, the status lines
//!                                               └─► Recorder ─► judge, step responses
//! ```
//!
//! **Nothing here decides what is safe.** A button becomes the spine's own request
//! (`SpineBoard::request_test`, `LegacyLink::request_calibration`), and the spine and the
//! link refuse what they refuse, as on the board.
//!
//! | Differs from today's ESP32 | Why |
//! |---|---|
//! | the spine measures the speed (`SpeedFeedback::Wheel`, by default) | today's board has no wheel speed; the test mode's throttle needs one |
//! | no RC receiver (`rc_fitted` false) | the simulator has no transmitter; a fitted, silent one is a lost RC, a hard stop |
//! | the devices always answer, and a Jrk is where it was told | `StandInBus`; the measured steering and brake come from the simulator |

pub mod parts;
pub mod report;

use bench_core::controls::{Button, Controls, Refused, Where};
use bench_core::hardware::{judge, BenchTest, Progress, Run, Verdict};
use bench_core::step::{respond, steps, Response};
use bench_core::trace::{Recorder, Sample};
use legacy_link::{CalRequest, CalState};
use spine_board::{
    Calibration, DeviceCheck, Devices, FrameVersion, LegacyLink, LinkHealth, SpineBoard, StandInBus,
};
use spine_core::{Outputs, SpeedFeedback, SpineConfig, TestRequest};
use std::sync::{Arc, Mutex};

use parts::{BusHandle, ByteQueue, ByteSink, LaptopBoard, NoReceiver};
use report::{Reporter, Snapshot};

/// The open loop's speed at full throttle (m/s): the board program's.
pub const FULL_THROTTLE_SPEED: f32 = 3.0;
/// The brake with nobody driving: the board program's (full on, as the 2025-26 C++).
pub const IDLE_BRAKE: f32 = 1.0;
/// The recording's length: the longest test (6 s) at 100 Hz, and room to spare.
pub const TRACE_SAMPLES: usize = 640;

type Spine = SpineBoard<LaptopBoard, BusHandle, NoReceiver, LegacyLink<ByteQueue, ByteSink>>;

/// How the virtual ESP32 is built.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// Which frames the bridge sends: 1 (13 bytes, today's car) or 2 (and 3, 4).
    pub frame_version: FrameVersion,
    /// The spine measures the speed from its wheel counter (true), or not, as today's
    /// board (false: the test mode's throttle is then refused, as on the board).
    pub wheel_speed: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            frame_version: FrameVersion::V1,
            wheel_speed: true,
        }
    }
}

/// What the simulator measured since the last tick.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Measured {
    /// The driven (rear) wheels' speed, m/s, + forward.
    pub speed: f32,
    /// The steering angle, rad, + left (NaN: not known).
    pub steering: f32,
    /// The brake's position, 0..1 (NaN: not known).
    pub brake: f32,
    /// The car is on stands (Ravensim's jacks).
    pub on_stands: bool,
}

/// What the devices do after a tick: what the simulator's actuators get.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Drive {
    /// 0..1: the DAC's code over its full code.
    pub throttle: f32,
    /// 0..1 of the brake Jrk's travel past released.
    pub brake: f32,
    /// The direction output: backwards.
    pub reverse: bool,
    /// rad, + left: the steering Jrk's target; while limp, where the wheels are.
    pub steering: f32,
    /// The steering actuator is limp (its motor off).
    pub limp: bool,
}

/// One tick's results.
#[derive(Debug, Clone)]
pub struct TickOut {
    /// What the spine decided.
    pub outputs: Outputs,
    /// What the devices do.
    pub drive: Drive,
    /// The uplink's bytes, for the bridge (frame version 2 and up reads them).
    pub uplink: Vec<u8>,
    /// The status lines this tick made (`firmware/status`), one JSON object each.
    pub lines: Vec<String>,
}

/// Why a test didn't start.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestRefused {
    /// The throttle test, not on stands.
    NotOnStands,
    /// One is running already.
    Busy,
}

/// How far a test run has got.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Into the spine's test mode: `Exit` sent, `Enter` next.
    Entering,
    /// `Enter` sent; the moves start next tick.
    Starting,
    /// Its moves.
    Running,
    /// Done or stopped: `Exit` next.
    Leaving,
}

#[derive(Debug, Clone)]
struct TestRun {
    test: BenchTest,
    phase: Phase,
    run: Option<Run>,
    heartbeat_ms: u32,
}

/// A test's latest result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Outcome {
    /// Which test.
    pub test: BenchTest,
    /// Finished, or stopped (the page let go).
    pub progress: Progress,
    /// The verdict from its recording.
    pub verdict: Verdict,
}

/// The virtual ESP32.
pub struct VirtualEsp32 {
    spine: Spine,
    rx: ByteQueue,
    tx: ByteSink,
    cal: Calibration,
    config: SpineConfig,
    health: LinkHealth,
    reporter: Reporter,
    controls: Controls,
    test: Option<TestRun>,
    outcome: Option<Outcome>,
    recorder: Recorder<TRACE_SAMPLES>,
    ticks: u32,
    last: Option<(Outputs, Drive, CalState)>,
    measured: Measured,
}

impl core::fmt::Debug for VirtualEsp32 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("VirtualEsp32")
            .field("ticks", &self.ticks)
            .field("test", &self.test)
            .field("outcome", &self.outcome)
            .finish_non_exhaustive()
    }
}

impl VirtualEsp32 {
    /// Boots it: the devices checked, the spine's `init()`. `boot_id` goes in the lines.
    #[must_use]
    pub fn new(settings: Settings, boot_id: u32) -> Self {
        // Today's numbers. (The board program also scales the RC's right turns; with no
        // receiver here, that changes nothing.)
        let cal = Calibration::TODAY;
        let config = SpineConfig {
            speed_feedback: if settings.wheel_speed {
                SpeedFeedback::Wheel
            } else {
                SpeedFeedback::None {
                    full_throttle_speed: FULL_THROTTLE_SPEED,
                }
            },
            idle_brake: IDLE_BRAKE,
            rc_fitted: false,
            ..SpineConfig::default()
        };
        let bus = Arc::new(Mutex::new(StandInBus::new(&cal)));
        let devices_seen = DeviceCheck::run(&mut BusHandle(Arc::clone(&bus)), &cal.addresses);
        let handle = || BusHandle(Arc::clone(&bus));
        let devices = Devices::new(handle(), handle(), handle(), handle(), &cal);
        let (rx, tx) = (ByteQueue::default(), ByteSink::default());
        let link = LegacyLink::with_uplink(rx.clone(), tx.clone(), cal, settings.frame_version);
        let board = LaptopBoard::new(cal.wheel.metres_per_count);
        let mut spine = SpineBoard::with_link(board, devices, NoReceiver, link, cal, config);
        spine.init();
        let version = match settings.frame_version {
            FrameVersion::V1 => 1,
            FrameVersion::V2 => 2,
        };
        let reporter = Reporter::new(
            boot_id,
            report::config(&cal, &config, version),
            &devices_seen,
        );
        VirtualEsp32 {
            spine,
            rx,
            tx,
            cal,
            config,
            health: LinkHealth::new(config.cmd_timeout_ms),
            reporter,
            controls: Controls::default(),
            test: None,
            outcome: None,
            recorder: Recorder::default(),
            ticks: 0,
            last: None,
            measured: Measured::default(),
        }
    }

    /// The bridge's bytes, as they came over the serial line (any split).
    pub fn feed(&mut self, bytes: &[u8]) {
        self.rx.push(bytes);
    }

    /// The page's heartbeat: `button` held at `now_ms` (every 50 ms while held). A press
    /// while a test runs stops the test instead.
    pub fn press(&mut self, button: Button, now_ms: u32) {
        if let Some(t) = &mut self.test {
            if let Some(run) = &mut t.run {
                run.stop();
            }
            return;
        }
        self.controls
            .press(button, now_ms, self.config.max_steering_angle);
    }

    /// The page let go (closed, or left the Controls tab): out of the test mode.
    pub fn leave(&mut self) {
        self.controls.leave();
    }

    /// Start a test at `now_ms`. Hold it with [`VirtualEsp32::test_heartbeat`].
    ///
    /// # Errors
    ///
    /// [`TestRefused`]: the throttle test off stands, or a test already running.
    pub fn start_test(&mut self, test: BenchTest, now_ms: u32) -> Result<(), TestRefused> {
        if self.test.is_some() {
            return Err(TestRefused::Busy);
        }
        if !test.may_start(self.place()) {
            return Err(TestRefused::NotOnStands);
        }
        self.controls.leave();
        self.outcome = None;
        self.test = Some(TestRun {
            test,
            phase: Phase::Entering,
            run: None,
            heartbeat_ms: now_ms,
        });
        Ok(())
    }

    /// The page still holds RUN.
    pub fn test_heartbeat(&mut self, now_ms: u32) {
        if let Some(t) = &mut self.test {
            t.heartbeat_ms = now_ms;
            if let Some(run) = &mut t.run {
                run.heartbeat(now_ms);
            }
        }
    }

    /// Stop the running test now.
    pub fn stop_test(&mut self) {
        if let Some(run) = self.test.as_mut().and_then(|t| t.run.as_mut()) {
            run.stop();
        }
    }

    /// One tick of the board's loop at `now_ms` (every 10 ms), with what the simulator
    /// measured.
    pub fn tick(&mut self, now_ms: u32, measured: Measured) -> TickOut {
        self.measured = measured;
        self.spine.board().measure(now_ms, measured.speed);
        let place = self.place();
        let (test, cal) = match self.test.as_mut() {
            Some(t) => (t.next(now_ms, &mut self.recorder), CalRequest::Off),
            None => {
                let cmd = self.controls.tick(now_ms, place);
                (cmd.test, cmd.cal)
            }
        };
        if let Some(request) = test {
            self.spine.request_test(request);
        }
        if cal != CalRequest::Off {
            self.spine.link().request_calibration(cal, now_ms);
        }
        let outputs = self.spine.tick();
        self.record(now_ms, &outputs, test);

        let report = self.spine.last_report().copied();
        let cal_state = report.map_or(CalState::Off, |r| {
            self.spine.link().uplink_frame(&r, 0).calibration
        });
        let act = self.spine.actuators();
        let (dac, reverse) = (act.throttle_code(), act.reverse());
        let steer_counts = act.steering_counts().unwrap_or(self.cal.steering.centre);
        let brake_counts = act.brake_counts().unwrap_or(self.cal.brake.full);
        let limp = cal_state == CalState::Limp;
        let drive = Drive {
            throttle: self.cal.throttle.fraction(dac),
            brake: self.cal.brake.fraction(brake_counts),
            reverse,
            steering: if limp && measured.steering.is_finite() {
                measured.steering
            } else {
                self.cal.steering.angle(steer_counts)
            },
            limp,
        };
        let stats = self.spine.link().stats();
        let last_frame = self.spine.link().last_frame();
        let health = self.health.update(now_ms, &stats);
        let positions = status_lines::Positions {
            steer: measured
                .steering
                .is_finite()
                .then(|| self.cal.steering.target(measured.steering)),
            brake: measured
                .brake
                .is_finite()
                .then(|| self.cal.brake.target(measured.brake)),
        };
        let snapshot = Snapshot {
            now_ms,
            out: &outputs,
            out_levels: status_lines::Out {
                steer_counts,
                brake_counts,
                dac,
                reverse,
            },
            positions,
            link: (stats, last_frame),
            health,
        };
        let lines = self.reporter.tick(self.ticks, &snapshot);
        self.ticks = self.ticks.wrapping_add(1);
        self.last = Some((outputs, drive, cal_state));
        TickOut {
            outputs,
            drive,
            uplink: self.tx.take(),
            lines,
        }
    }

    /// The tick's sample. What was asked is the page's request (a step, as a person asks
    /// it), so the response includes the spine's own steering rate; otherwise the spine's.
    fn record(&mut self, now_ms: u32, out: &Outputs, request: Option<TestRequest>) {
        let m = self.measured;
        let (steer_asked, brake_asked) = match request {
            Some(TestRequest::Steering(a)) => (a, out.brake),
            Some(TestRequest::Brake(b)) => (out.steering_target, b),
            _ => (out.steering_target, out.brake),
        };
        self.recorder.push(
            now_ms,
            Sample {
                at_ms: 0,
                steer_asked,
                steer: m.steering,
                brake_asked,
                brake: m.brake,
                throttle: out.throttle,
                speed: m.speed,
            },
        );
        // A run that has ended: its verdict, from its recording.
        if let Some(t) = &mut self.test {
            if let Some(run) = t.run.as_ref().filter(|_| t.phase == Phase::Leaving) {
                if self.outcome.is_none() {
                    self.recorder.stop();
                    self.outcome = Some(Outcome {
                        test: t.test,
                        progress: run.progress(),
                        verdict: judge(t.test, self.recorder.samples()),
                    });
                }
            }
            if t.phase == Phase::Leaving && t.run.is_none() {
                self.test = None;
            }
        }
    }

    /// Where the page's own rule says the car is.
    fn place(&self) -> Where {
        if self.measured.on_stands {
            Where::OnStands
        } else {
            Where::OnTheGround
        }
    }

    /// The button held now, if its heartbeat is fresh.
    #[must_use]
    pub fn held(&self, now_ms: u32) -> Option<Button> {
        self.controls.held(now_ms)
    }

    /// Why the page's own rule refused the last tick's button.
    #[must_use]
    pub fn refused(&self) -> Option<Refused> {
        self.controls.refused()
    }

    /// The running test and how far it has got.
    #[must_use]
    pub fn running(&self) -> Option<(BenchTest, Phase)> {
        self.test.as_ref().map(|t| (t.test, t.phase))
    }

    /// The latest test's result.
    #[must_use]
    pub fn outcome(&self) -> Option<Outcome> {
        self.outcome
    }

    /// The latest tick: what the spine decided, what the devices do, the steering
    /// calibration's state.
    #[must_use]
    pub fn last(&self) -> Option<(Outputs, Drive, CalState)> {
        self.last
    }

    /// The recording so far (the latest test's).
    #[must_use]
    pub fn trace(&self) -> &[Sample] {
        self.recorder.samples()
    }

    /// How an actuator followed each step in the recording: the delay, the 10-90 % rise,
    /// where it ended (`bench-core`'s `step`). Empty with no position from it.
    #[must_use]
    pub fn step_responses(&self, actuator: Actuator) -> Vec<Response> {
        let samples = self.recorder.samples();
        let least = actuator.least();
        if samples.iter().any(|s| actuator.pick(s).1.is_nan()) {
            return Vec::new();
        }
        let t: Vec<u32> = samples.iter().map(|s| s.at_ms).collect();
        let asked: Vec<f32> = samples.iter().map(|s| actuator.pick(s).0).collect();
        let measured: Vec<f32> = samples.iter().map(|s| actuator.pick(s).1).collect();
        let (found, _) = steps::<8>(&asked, least);
        found
            .iter()
            .flatten()
            .map(|&step| respond(&t, &measured, step, least))
            .collect()
    }

    /// The spine's steering limit (rad).
    #[must_use]
    pub fn max_steering(&self) -> f32 {
        self.config.max_steering_angle
    }
}

/// Which actuator a step response is of.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Actuator {
    /// The steering: rad.
    Steering,
    /// The brake: 0..1.
    Brake,
}

impl Actuator {
    /// What was asked, and what it did.
    fn pick(self, s: &Sample) -> (f32, f32) {
        match self {
            Actuator::Steering => (s.steer_asked, s.steer),
            Actuator::Brake => (s.brake_asked, s.brake),
        }
    }

    /// The smallest step timed, and the pass band: the hardware tests' own.
    fn least(self) -> f32 {
        match self {
            Actuator::Steering => 0.03,
            Actuator::Brake => 0.1,
        }
    }
}

impl TestRun {
    /// This tick's request, moving the run along; the recording starts with the moves.
    fn next(&mut self, now_ms: u32, recorder: &mut Recorder<TRACE_SAMPLES>) -> Option<TestRequest> {
        match self.phase {
            Phase::Entering => {
                self.phase = Phase::Starting;
                Some(TestRequest::Exit)
            }
            Phase::Starting => {
                if now_ms.wrapping_sub(self.heartbeat_ms) > bench_core::controls::HOLD_MS {
                    self.phase = Phase::Leaving;
                    return Some(TestRequest::Exit);
                }
                self.phase = Phase::Running;
                // `Run::start` only refuses the throttle off stands, checked at the start.
                self.run = Run::start(self.test, now_ms.wrapping_add(10), Where::OnStands);
                recorder.start(now_ms, self.test.length_ms() + 200);
                Some(TestRequest::Enter)
            }
            Phase::Running => {
                let Some(run) = self.run.as_mut() else {
                    self.phase = Phase::Leaving;
                    return Some(TestRequest::Exit);
                };
                let request = run.tick(now_ms);
                if run.progress() != Progress::Running {
                    self.phase = Phase::Leaving;
                }
                Some(request)
            }
            Phase::Leaving => {
                // The verdict is taken this tick (`record`); then the run is gone.
                self.run = None;
                Some(TestRequest::Exit)
            }
        }
    }
}

#[cfg(test)]
mod tests;
