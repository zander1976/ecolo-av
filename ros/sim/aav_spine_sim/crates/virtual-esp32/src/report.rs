//! The status lines (`firmware/status`), from what the tick already has: the ESP32
//! board program's `report.rs`, on a laptop (a board program is no library, so its
//! cadence is copied here; the lines themselves are `status-lines`', unchanged).
//!
//! | Line | When |
//! |---|---|
//! | `hb` | once a second |
//! | `status` | every 10 ticks (10 a second, as with `wifi-status`) |
//! | `event` | the tick the mode, arming, e-stop or faults change |
//! | `health` | once a second: the Pi link's verdict, the devices, the faults |
//! | `config`, `devices` | 250 and 350 ms after boot, then every 10 s |

use spine_board::{
    feedback_name, Calibration, DeviceCheck, Found, LastFrame, LinkReport, LinkStats, LinkVerdict,
};
use spine_core::protocol::Mode;
use spine_core::{EstopSteering, Outputs, SpineConfig};
use status_lines::{
    Body, Config, DeviceList, DeviceSeen, Event, Health, Heartbeat, Levels, Line, LinkCounts, Out,
    Positions, Rc, Rx, Spread, State, Status, Switch, Tick, Value, Verdict,
};

/// The build, as the page and the Orin's page show it.
pub const BUILD: &str = "virtual car";
/// The board program's features this one stands for.
pub const FEATURES: &str = "legacy-uart,wifi-status,wifi-control";
/// Why the chip last reset: it is a program.
const LAST_RESET: &str = "power_on";

const STATUS_EVERY_TICKS: u32 = 10;
const HEARTBEAT_EVERY_TICKS: u32 = 100;
const HEARTBEAT_AT_TICK: u32 = 5;
const HEALTH_EVERY_TICKS: u32 = 100;
const HEALTH_AT_TICK: u32 = 55;
const CONFIG_EVERY_TICKS: u32 = 1000;
const CONFIG_AT_TICK: u32 = 25;
const DEVICES_AT_TICK: u32 = 35;

/// What one tick had, for its lines.
#[derive(Debug, Clone, Copy)]
pub struct Snapshot<'a> {
    /// The tick's clock, ms.
    pub now_ms: u32,
    /// What the spine decided.
    pub out: &'a Outputs,
    /// The last target sent to each Jrk, the DAC's code, the direction.
    pub out_levels: Out,
    /// Where the actuators are (counts): the simulator's car, not the stand-in's echo.
    pub positions: Positions,
    /// The Pi link: its counts and its last good frame.
    pub link: (LinkStats, Option<LastFrame>),
    /// The Pi link's verdict this tick.
    pub health: LinkReport,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Watched {
    mode: status_lines::Mode,
    armed: bool,
    estop: bool,
    faults: u16,
}

/// Numbers the lines, and decides which lines a tick makes.
#[derive(Debug)]
pub struct Reporter {
    seq: u32,
    boot_id: u32,
    config: Config<'static>,
    devices: [DeviceSeen<'static>; 4],
    missing: [&'static str; 4],
    missing_len: usize,
    spread: Spread,
    watched: Option<Watched>,
}

impl Reporter {
    /// `config`: the build and its numbers. `devices`: what answered at boot.
    #[must_use]
    pub fn new(boot_id: u32, config: Config<'static>, devices: &DeviceCheck) -> Self {
        let mut seen = [DeviceSeen {
            name: "",
            address: 0,
            answered: false,
            feedback: None,
        }; 4];
        let mut missing = [""; 4];
        let mut missing_len = 0;
        for (slot, found) in seen.iter_mut().zip(devices.iter()) {
            *slot = device_seen(found);
            if !found.answered {
                if let Some(m) = missing.get_mut(missing_len) {
                    *m = found.device.name();
                    missing_len += 1;
                }
            }
        }
        Reporter {
            seq: 0,
            boot_id,
            config,
            devices: seen,
            missing,
            missing_len,
            spread: Spread::new(),
            watched: None,
        }
    }

    /// One tick: the lines it makes, in order.
    pub fn tick(&mut self, ticks: u32, s: &Snapshot<'_>) -> Vec<String> {
        let mut lines = Vec::new();
        let out = s.out_levels;
        self.spread.add(Levels::from(&out));
        let state = State {
            mode: mode(s.out.mode),
            armed: s.out.armed,
            estop: s.out.estop_active,
            faults: s.out.faults,
        };
        let now = Watched {
            mode: state.mode,
            armed: state.armed,
            estop: state.estop,
            faults: state.faults,
        };
        if let Some(was) = self.watched.replace(now) {
            for (what, from, to) in changes(&was, &now) {
                let body = Body::Event(Event { what, from, to });
                lines.push(self.line(s.now_ms, body));
            }
        }
        if ticks % HEARTBEAT_EVERY_TICKS == HEARTBEAT_AT_TICK {
            let body = Body::Heartbeat(Heartbeat {
                boot_id: self.boot_id,
                build: BUILD,
                features: FEATURES,
                last_reset: LAST_RESET,
            });
            lines.push(self.line(s.now_ms, body));
        }
        if ticks % CONFIG_EVERY_TICKS == CONFIG_AT_TICK {
            let body = Body::Config(self.config);
            lines.push(self.line(s.now_ms, body));
        }
        if ticks % CONFIG_EVERY_TICKS == DEVICES_AT_TICK {
            let devices = self.devices;
            let body = Body::Devices(DeviceList { devices: &devices });
            lines.push(self.line(s.now_ms, body));
        }
        if ticks % HEALTH_EVERY_TICKS == HEALTH_AT_TICK {
            let answered = self.devices.iter().filter(|d| d.answered).count();
            let h = s.health;
            let missing = self.missing;
            let body = Body::Health(Health {
                link: Some(verdict(h.verdict)),
                age_ms: h.age_ms,
                counter_moving: h.counter_moving,
                counts: counts(&h.stats, Some(&h)),
                devices_answered: u8::try_from(answered).unwrap_or(u8::MAX),
                devices_expected: u8::try_from(self.devices.len()).unwrap_or(u8::MAX),
                missing: &missing[..self.missing_len],
                faults: s.out.faults,
                mode: state.mode,
                last_reset: LAST_RESET,
                build: BUILD,
                boot_id: self.boot_id,
            });
            lines.push(self.line(s.now_ms, body));
        }
        if ticks.is_multiple_of(STATUS_EVERY_TICKS) {
            let (low, high) = self.spread.take(Levels::from(&out));
            let (stats, frame) = s.link;
            let rx = frame.map(|f| Rx {
                version: f.version,
                counter: f.counter,
                check: f.check,
                steer: f.values.steering,
                speed: f.values.speed,
                accel: f.values.accel,
                age_ms: f.age_ms(s.now_ms),
            });
            let status = Status {
                rx,
                out,
                low,
                high,
                pos: s.positions,
                link: counts(&stats, Some(&s.health)),
                // No receiver on the virtual car.
                rc: Rc {
                    switch: Switch::NoSignal,
                    channels: &[],
                },
                state,
                // The laptop's ticks run on the simulator's clock: no gaps to report.
                tick: Tick {
                    worst_gap_us: 0,
                    overruns: 0,
                },
            };
            lines.push(self.line(s.now_ms, Body::Status(status)));
        }
        lines
    }

    fn line(&mut self, up_ms: u32, body: Body<'_>) -> String {
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1);
        Line { seq, up_ms, body }.to_string()
    }
}

/// The build and its numbers, as the board program's `board_config` gives them.
#[must_use]
pub fn config(cal: &Calibration, config: &SpineConfig, frame_version: u8) -> Config<'static> {
    let (a, s, b) = (&cal.addresses, &cal.steering, &cal.brake);
    let steer_cal = frame_version >= 2;
    status_lines::Config {
        build: BUILD,
        features: FEATURES,
        version: env!("CARGO_PKG_VERSION"),
        link: "uart",
        frame_version: Some(frame_version),
        tick_ms: 10,
        watchdog_ms: 100,
        cmd_timeout_ms: config.cmd_timeout_ms,
        idle_brake: config.idle_brake,
        full_throttle_speed: crate::FULL_THROTTLE_SPEED,
        i2c_khz: None,
        addresses: status_lines::Addresses {
            steering: a.steering,
            brake: a.brake,
            dac: a.dac,
            expander: a.expander,
        },
        steering: status_lines::SteeringLimits {
            min: s.min,
            centre: s.centre,
            max: s.max,
        },
        brake: status_lines::BrakeLimits {
            released: b.released,
            full: b.full,
        },
        throttle_full_code: cal.throttle.full_code,
        last_reset: LAST_RESET,
        steering_more: status_lines::SteeringSettings {
            counts_per_rad: s.counts_per_rad,
            max_angle_rad: config.max_steering_angle,
            rate_rad_s: config.max_steer_rate,
            cal_rate: steer_cal.then_some(legacy_link::CAL_RATE),
            cal_timeout_ms: steer_cal.then_some(legacy_link::CAL_TIMEOUT_MS),
            on_stop: match config.estop_steering {
                EstopSteering::Hold => "hold",
                EstopSteering::Centre => "centre",
            },
        },
        brake_more: status_lines::BrakeSettings {
            stop: config.stop_brake,
            overspeed: config.overspeed_brake,
            rc_band: cal.rc.brake_band,
            rc_brake_ms: spine_core::RC_BRAKE_MS,
            rc_lost_below: cal.rc.lost_below,
            rc_fitted: config.rc_fitted,
        },
        speed: status_lines::SpeedSettings {
            max_forward: config.max_speed,
            max_reverse: config.max_reverse_speed,
            cap_code: config
                .open_loop_throttle_cap(false)
                .map(|cap| cal.throttle.code(cap)),
            reverse_cap_code: config
                .open_loop_throttle_cap(true)
                .map(|cap| cal.throttle.code(cap)),
            frame_dac_cap: Some(legacy_link::DAC_CAP),
            rise_per_s: cal.throttle.rise_per_s,
            max_accel: config.max_accel,
            max_decel: config.max_decel,
            overspeed_margin: config.overspeed_margin,
        },
        timeouts: status_lines::Timeouts {
            rc_lost_ms: cal.rc_timeout_ms,
            speed_lost_ms: config.speed_timeout_ms,
            overspeed_ms: spine_core::OVERSPEED_WINDOW_MS,
            estop_release_ms: cal.estop_release_ms,
            bad_frames_ms: Some(spine_board::BAD_WINDOW_MS),
        },
        rates: status_lines::Rates {
            uart_baud: Some(legacy_link::BAUD),
            frame_hz: Some(legacy_link::FRAME_HZ),
            uplink_ms: Some(spine_board::UPLINK_PERIOD_MS),
            can_kbit: None,
            // ibus::BAUD: the RC receiver's port, though none is fitted here.
            ibus_baud: 115_200,
            jrk_write_ms: 10,
            dac_write_ms: 10,
            status_ms: STATUS_EVERY_TICKS * 10,
        },
    }
}

fn device_seen(f: &Found) -> DeviceSeen<'static> {
    DeviceSeen {
        name: f.device.name(),
        address: f.address,
        answered: f.answered,
        feedback: match (f.device.is_jrk() && f.answered, f.feedback) {
            (false, _) => None,
            (true, Some(mode)) => Some(feedback_name(mode)),
            (true, None) => Some("unknown"),
        },
    }
}

fn changes(
    was: &Watched,
    now: &Watched,
) -> impl Iterator<Item = (&'static str, Value<'static>, Value<'static>)> {
    [
        (was.mode != now.mode).then_some((
            "mode",
            Value::Text(was.mode.name()),
            Value::Text(now.mode.name()),
        )),
        (was.armed != now.armed).then_some((
            "armed",
            Value::Flag(was.armed),
            Value::Flag(now.armed),
        )),
        (was.estop != now.estop).then_some((
            "estop",
            Value::Flag(was.estop),
            Value::Flag(now.estop),
        )),
        (was.faults != now.faults).then_some((
            "faults",
            Value::Number(u32::from(was.faults)),
            Value::Number(u32::from(now.faults)),
        )),
    ]
    .into_iter()
    .flatten()
}

fn counts(stats: &LinkStats, health: Option<&LinkReport>) -> LinkCounts {
    LinkCounts {
        frames: stats.frames,
        bad: stats.bad,
        skipped: stats.skipped,
        repeats: stats.repeats,
        verdict: health.map(|h| verdict(h.verdict)),
    }
}

fn verdict(v: LinkVerdict) -> Verdict {
    match v {
        LinkVerdict::NoFramesYet => Verdict::NoFrames,
        LinkVerdict::Ok => Verdict::Ok,
        LinkVerdict::Stale { .. } => Verdict::Stale,
        LinkVerdict::CounterFrozen { .. } => Verdict::CounterFrozen,
        LinkVerdict::BadFrames { .. } => Verdict::BadFrames,
    }
}

/// The spine's mode as a status line names it.
#[must_use]
pub fn mode(m: Mode) -> status_lines::Mode {
    match m {
        Mode::Manual => status_lines::Mode::Stopped,
        Mode::Rc => status_lines::Mode::Rc,
        Mode::Computer => status_lines::Mode::Computer,
        Mode::Test => status_lines::Mode::Test,
    }
}
