//! What the page shows, as one JSON snapshot: built from the panel's state (panel-core,
//! the same rules as `hw_panel`'s GRAPHS page) and the ROS tab's rows. No ROS here.
//!
//! Times are seconds before now (0 = now, -60 = a minute ago), so the page needs no clock.

use std::time::Instant;

use panel_core::{
    graphs::{Channel, Esp32Line, Hop, Line, Point},
    state::{Esp32Link, PanelState, Row as PanelRow},
};
use serde::Serialize;

use crate::contract;

/// Points in a line, at most: the page draws 60 s, so one point per 0.1 s is plenty.
const MAX_POINTS: usize = 600;

/// Everything the page shows.
#[derive(Debug, Clone, Serialize)]
pub struct Snapshot {
    /// What `/system/source` says: REAL, SIM, BENCH, REPLAY, UNKNOWN.
    pub source: String,
    /// `/safety/estop`, if heard.
    pub estop: Option<bool>,
    /// The Car tab: each sensor and actuator.
    pub rows: Vec<CarRow>,
    /// The Hops tab: each hop's verdict.
    pub hops: Vec<HopRow>,
    /// The Hops tab: the three channels' graphs.
    pub graphs: Vec<Graph>,
    /// The counts that prove the wire.
    pub counts: Counts,
    /// The delays between hops, where commands pair one to one.
    pub delays: Delays,
    /// The ESP32's WebSocket: off, connecting, connected or lost.
    pub esp32: String,
    /// The ROS tab.
    pub ros: Ros,
}

/// One sensor or actuator.
#[derive(Debug, Clone, Serialize)]
pub struct CarRow {
    /// Its name.
    pub name: &'static str,
    /// OK, STALE or NEVER.
    pub health: &'static str,
    /// Seconds since it was heard.
    pub age_s: Option<f64>,
    /// Messages a second.
    pub rate_hz: Option<f64>,
    /// Its numbers, as hw_panel writes them.
    pub detail: String,
}

/// One hop.
#[derive(Debug, Clone, Serialize)]
pub struct HopRow {
    /// ego, mux, Pi wrote, ESP32 got.
    pub hop: &'static str,
    /// OK, DIFFERENT, STALE, NO DATA, NOT TESTED.
    pub verdict: &'static str,
    /// Why, in words.
    pub why: String,
}

/// One channel: what ROS asked (its units), and what went on the wire (the frame's -1..1).
#[derive(Debug, Clone, Serialize)]
pub struct Graph {
    /// steering, brake, throttle.
    pub channel: &'static str,
    /// rad or m/s.
    pub asked_unit: &'static str,
    /// ego's command: [seconds, value, gap before].
    pub ego: Vec<(f64, f64, bool)>,
    /// The mux's.
    pub mux: Vec<(f64, f64, bool)>,
    /// What the Pi (CORTEX) wrote.
    pub pi: Vec<(f64, f64, bool)>,
    /// What the ESP32 got (or drives, without its status lines: see `esp32_from`).
    pub esp32: Vec<(f64, f64, bool)>,
    /// "received", "uplink output" or none.
    pub esp32_from: Option<&'static str>,
}

/// A delay between two hops, ms, as this computer heard both.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Delay {
    /// The median.
    pub median_ms: f64,
    /// The least.
    pub low_ms: f64,
    /// The most.
    pub high_ms: f64,
    /// How many commands were paired.
    pub count: usize,
}

/// The delays the page shows.
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct Delays {
    /// ego to the output mux (by stamp).
    pub ego_to_mux: Option<Delay>,
    /// What CORTEX wrote to what the ESP32 got (by counter and check; version 2 frames).
    pub pi_to_esp32: Option<Delay>,
}

/// The status strip's counts.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Counts {
    /// The Pi's frames written since its bridge started.
    pub pi_written: Option<u32>,
    /// The Pi's failed writes.
    pub pi_failed: Option<u32>,
    /// The ESP32's frames counted, bad, skipped, repeated (since boot).
    pub esp32_frames: Option<u32>,
    pub esp32_bad: Option<u32>,
    pub esp32_skipped: Option<u32>,
    pub esp32_repeats: Option<u32>,
    /// The uplink's frames, lost, bad, and whether fresh.
    pub uplink_frames: Option<u32>,
    pub uplink_lost: Option<u32>,
    pub uplink_bad: Option<u32>,
    pub uplink_fresh: Option<bool>,
    /// Status lines that never came.
    pub lines_lost: u64,
    /// The board restarted in the last minute.
    pub restarted: bool,
}

/// The ROS tab.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Ros {
    /// The profile judged against.
    pub profile: String,
    /// Whether the graph has been read yet.
    pub read: bool,
    /// Every row, the problems first.
    pub rows: Vec<contract::Row>,
    /// How many are problems.
    pub problems: usize,
    /// How many the profile needs live, and how many are.
    pub needed: usize,
    pub needed_live: usize,
}

impl Ros {
    /// The rows for `profile`; `seen` None: not read yet.
    #[cfg_attr(not(feature = "ros"), allow(dead_code))]
    #[must_use]
    pub fn judge(
        contract: &contract::Contract,
        profile: &str,
        seen: Option<&[contract::Seen]>,
    ) -> Self {
        let Some(seen) = seen else {
            return Self {
                profile: profile.to_owned(),
                ..Self::default()
            };
        };
        let rows = contract.judge(profile, seen);
        Self {
            profile: profile.to_owned(),
            read: true,
            problems: rows.iter().filter(|r| r.verdict.is_problem()).count(),
            needed: rows.iter().filter(|r| r.needed).count(),
            needed_live: rows
                .iter()
                .filter(|r| r.needed && r.verdict == contract::Verdict::Live)
                .count(),
            rows,
        }
    }
}

/// The rows of the Car tab, top to bottom.
const CAR_ROWS: [PanelRow; 12] = [
    PanelRow::Imu,
    PanelRow::Wheels,
    PanelRow::Gps,
    PanelRow::Lidar,
    PanelRow::Radar,
    PanelRow::Camera,
    PanelRow::Battery,
    PanelRow::Vehicle,
    PanelRow::Steering,
    PanelRow::Throttle,
    PanelRow::Brake,
    PanelRow::Mux,
];

/// The snapshot at `now`.
#[must_use]
pub fn build(state: &PanelState, ros: Ros, now: Instant) -> Snapshot {
    let rows = CAR_ROWS
        .iter()
        .map(|&row| {
            let health = state.health(row, now);
            CarRow {
                name: row.label(),
                health: health.word(),
                age_s: health.age().map(|d| d.as_secs_f64()),
                rate_hz: state.rate(row, now),
                detail: state.detail(row, now, None),
            }
        })
        .collect();
    let g = &state.graphs;
    let hops = g
        .hops(now, state.stale_after())
        .into_iter()
        .map(|r| HopRow {
            hop: r.hop.label(),
            verdict: r.verdict.word(),
            why: r.verdict.why().to_owned(),
        })
        .collect();
    let line = |points: Vec<Point>| thin(&points, now);
    let esp32_from = g.esp32_line(now).map(|l| match l {
        Esp32Line::Received => "received",
        Esp32Line::UplinkOutput => "uplink output",
    });
    let graphs = [Channel::Steering, Channel::Brake, Channel::Throttle]
        .into_iter()
        .map(|c| Graph {
            channel: c.label(),
            asked_unit: match c {
                Channel::Steering => "rad",
                Channel::Brake | Channel::Throttle => "m/s",
            },
            ego: line(g.drive_line(c, Hop::Ego, now)),
            mux: line(g.drive_line(c, Hop::Mux, now)),
            pi: line(g.line(c, Line::PiWrote, now)),
            esp32: line(g.line(c, Line::Esp32, now)),
            esp32_from,
        })
        .collect();
    let c = g.counts(now);
    let counts = Counts {
        pi_written: c.pi.map(|p| p.0),
        pi_failed: c.pi.map(|p| p.1),
        esp32_frames: c.esp32.map(|e| e.0.frames),
        esp32_bad: c.esp32.map(|e| e.0.bad),
        esp32_skipped: c.esp32.map(|e| e.0.skipped),
        esp32_repeats: c.esp32.map(|e| e.0.repeats),
        uplink_frames: c.uplink.map(|u| u.frames),
        uplink_lost: c.uplink.map(|u| u.lost),
        uplink_bad: c.uplink.map(|u| u.bad),
        uplink_fresh: c.uplink.map(|u| u.fresh),
        lines_lost: c.lines_lost,
        restarted: c.restarted,
    };
    let d = g.delays(now);
    let delay = |d: Option<panel_core::graphs::Delay>| {
        d.map(|d| Delay {
            median_ms: d.median_ms,
            low_ms: d.low_ms,
            high_ms: d.high_ms,
            count: d.count,
        })
    };
    let delays = Delays {
        ego_to_mux: delay(d.ego_to_mux),
        pi_to_esp32: delay(d.pi_to_esp32),
    };
    let esp32 = match state.esp32.as_ref().map(|e| &e.link) {
        None => "off".to_owned(),
        Some(Esp32Link::Connecting(to)) => format!("connecting to {to}"),
        Some(Esp32Link::Connected) => "connected".to_owned(),
        Some(Esp32Link::Lost(why)) => format!("lost: {why}"),
    };
    Snapshot {
        source: format!("{:?}", state.kind()).to_uppercase(),
        estop: state.estop,
        rows,
        hops,
        graphs,
        counts,
        delays,
        esp32,
        ros,
    }
}

/// A line as [seconds before now, value, gap before], at most [`MAX_POINTS`]: every
/// n-th point, the last always kept, and a gap kept where any skipped point had one.
fn thin(points: &[Point], now: Instant) -> Vec<(f64, f64, bool)> {
    let step = points.len().div_ceil(MAX_POINTS).max(1);
    let at = |p: &Point| -(now.saturating_duration_since(p.at).as_secs_f64());
    let mut out: Vec<(f64, f64, bool)> = points
        .chunks(step)
        .map(|chunk| {
            let last = chunk[chunk.len() - 1];
            (at(&last), last.value, chunk.iter().any(|p| p.gap_before))
        })
        .collect();
    if let (Some(last), Some(kept)) = (points.last(), out.last_mut()) {
        kept.0 = at(last);
        kept.1 = last.value;
    }
    out
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use panel_core::graphs::Drive;

    use super::*;

    #[test]
    fn a_quiet_car_says_never_and_no_data() {
        let now = Instant::now();
        let state = PanelState::new(Duration::from_secs(1));
        let s = build(&state, Ros::default(), now);
        assert_eq!(s.source, "UNKNOWN");
        assert_eq!(s.rows.len(), CAR_ROWS.len());
        assert!(s.rows.iter().all(|r| r.health == "NEVER"));
        assert_eq!(s.hops.len(), Hop::ALL.len());
        assert!(
            s.hops.iter().all(|h| h.verdict == "NO DATA"),
            "{:?}",
            s.hops
        );
        assert_eq!(s.esp32, "off");
        let json = serde_json::to_string(&s).expect("serialises");
        assert!(json.contains("\"graphs\""));
        assert!(json.contains("\"delays\":{\"ego_to_mux\":null,\"pi_to_esp32\":null}"));
    }

    #[test]
    fn egos_command_shows_in_seconds_before_now() {
        let t = Instant::now();
        let mut state = PanelState::new(Duration::from_secs(1));
        for i in 0..20u32 {
            let d = Drive {
                stamp_ns: u64::from(i) + 1,
                speed: 0.05 * f64::from(i),
                steering: 0.0,
            };
            state
                .graphs
                .record_ego(t + Duration::from_millis(50 * u64::from(i)), d);
        }
        let now = t + Duration::from_millis(950);
        let s = build(&state, Ros::default(), now);
        let throttle = s
            .graphs
            .iter()
            .find(|g| g.channel == "THROTTLE")
            .expect("throttle");
        assert_eq!(throttle.ego.len(), 20);
        let (at, value, _) = throttle.ego[19];
        assert!(at.abs() < 1e-9 && (value - 0.95).abs() < 1e-9);
        assert!((throttle.ego[0].0 + 0.95).abs() < 1e-6);
        assert!(throttle.pi.is_empty());
    }

    #[test]
    fn a_long_line_is_thinned_and_keeps_its_last_point_and_gaps() {
        let now = Instant::now();
        let points: Vec<Point> = (0..3000u32)
            .map(|i| Point {
                at: now - Duration::from_millis(u64::from(3000 - i) * 20),
                value: f64::from(i),
                gap_before: i == 1001,
            })
            .collect();
        let out = thin(&points, now);
        assert!(out.len() <= MAX_POINTS);
        assert_eq!(out.last().map(|p| p.1), Some(2999.0));
        assert_eq!(out.iter().filter(|p| p.2).count(), 1);
    }
}
