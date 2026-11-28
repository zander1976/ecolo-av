//! The hardware tests the ESP32's page runs: fixed moves, a recording, a pass rule.
//!
//! Each [`BenchTest`] is a list of [`Move`]s, each a spine request held for a time. The
//! [`Run`] hands out this tick's request ([`Run::tick`]); the board records a
//! [`crate::trace::Sample`] a tick meanwhile, and [`judge`] gives the verdict from the
//! recording. A run is a person holding RUN: like a button, it stops (`Hold`) when the
//! page's heartbeat stops ([`crate::controls::HOLD_MS`]).
//!
//! | Test | Moves | Measures | Passes when |
//! |---|---|---|---|
//! | steering steps | centre, +0.2 rad, -0.2 rad, centre (1.5 s each) | the steering actuator's position | each step gets 90 % of the way, and ends within 0.03 rad |
//! | brake travel | off, on, off (1.5 s each) | the brake actuator's position | each step gets 90 % of the way, and ends within 0.1 |
//! | throttle spin-up | the test-mode creep for 3 s, then hold 2 s; **on stands** | the wheels' speed | the wheels turn (over 0.1 m/s), and stop again under 0.05 m/s |
//!
//! The spine's test mode carries each request out or refuses it (throttle needs a wheel
//! speed; anything above 0.5 m/s leaves the mode): a refused move shows in the recording
//! as a step that never moved.

use spine_core::TestRequest;

use crate::{
    controls::{Where, CREEP, HOLD_MS},
    step::{respond, steps, Why},
    trace::Sample,
};

/// One request, held.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Move {
    /// What the spine is asked.
    pub request: TestRequest,
    /// For how long, ms.
    pub hold_ms: u32,
}

const fn hold(request: TestRequest, hold_ms: u32) -> Move {
    Move { request, hold_ms }
}

const STEER_STEP: f32 = 0.2;
const STEERING: [Move; 4] = [
    hold(TestRequest::Steering(0.0), 1500),
    hold(TestRequest::Steering(STEER_STEP), 1500),
    hold(TestRequest::Steering(-STEER_STEP), 1500),
    hold(TestRequest::Steering(0.0), 1500),
];
const BRAKE: [Move; 3] = [
    hold(TestRequest::Brake(0.0), 1500),
    hold(TestRequest::Brake(1.0), 1500),
    hold(TestRequest::Brake(0.0), 1500),
];
const THROTTLE: [Move; 2] = [
    hold(TestRequest::Throttle(CREEP), 3000),
    hold(TestRequest::Hold, 2000),
];

/// A hardware test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BenchTest {
    /// The steering actuator follows steps.
    SteeringSteps,
    /// The brake actuator goes on and off.
    BrakeTravel,
    /// The wheels spin up and stop, on stands.
    ThrottleSpinUp,
}

impl BenchTest {
    /// Every test, in the page's order.
    pub const ALL: [BenchTest; 3] = [Self::SteeringSteps, Self::BrakeTravel, Self::ThrottleSpinUp];

    /// Its name on the page.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::SteeringSteps => "steering_steps",
            Self::BrakeTravel => "brake_travel",
            Self::ThrottleSpinUp => "throttle_spin_up",
        }
    }

    /// Its moves.
    #[must_use]
    pub fn moves(self) -> &'static [Move] {
        match self {
            Self::SteeringSteps => &STEERING,
            Self::BrakeTravel => &BRAKE,
            Self::ThrottleSpinUp => &THROTTLE,
        }
    }

    /// How long it takes, ms: the recording's length.
    #[must_use]
    pub fn length_ms(self) -> u32 {
        self.moves().iter().map(|m| m.hold_ms).sum()
    }

    /// Whether it may start here: the throttle only on stands.
    #[must_use]
    pub fn may_start(self, place: Where) -> bool {
        self != Self::ThrottleSpinUp || place == Where::OnStands
    }
}

/// Where a run is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Progress {
    /// Moving through its moves.
    Running,
    /// Every move done.
    Finished,
    /// The page's heartbeat stopped, or it was stopped: holding still.
    Stopped,
}

/// One run of a test.
#[derive(Debug, Clone)]
pub struct Run {
    test: BenchTest,
    from_ms: u32,
    heartbeat_ms: u32,
    progress: Progress,
}

impl Run {
    /// Starts `test` at `now_ms`; None if it may not start here.
    #[must_use]
    pub fn start(test: BenchTest, now_ms: u32, place: Where) -> Option<Self> {
        test.may_start(place).then_some(Self {
            test,
            from_ms: now_ms,
            heartbeat_ms: now_ms,
            progress: Progress::Running,
        })
    }

    /// The page still holds RUN.
    pub fn heartbeat(&mut self, now_ms: u32) {
        self.heartbeat_ms = now_ms;
    }

    /// Stop now.
    pub fn stop(&mut self) {
        if self.progress == Progress::Running {
            self.progress = Progress::Stopped;
        }
    }

    /// Which test.
    #[must_use]
    pub fn test(&self) -> BenchTest {
        self.test
    }

    /// Where it is.
    #[must_use]
    pub fn progress(&self) -> Progress {
        self.progress
    }

    /// This tick's request: the move due now, or `Hold` once finished or stopped.
    pub fn tick(&mut self, now_ms: u32) -> TestRequest {
        if self.progress == Progress::Running && now_ms.wrapping_sub(self.heartbeat_ms) > HOLD_MS {
            self.progress = Progress::Stopped;
        }
        if self.progress != Progress::Running {
            return TestRequest::Hold;
        }
        let mut at = now_ms.wrapping_sub(self.from_ms);
        for m in self.test.moves() {
            if at < m.hold_ms {
                return m.request;
            }
            at -= m.hold_ms;
        }
        self.progress = Progress::Finished;
        TestRequest::Hold
    }
}

/// A test's verdict.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Verdict {
    /// Every check held.
    Pass,
    /// A step didn't get there, or ended too far off: which step (from 1), and how.
    Fail {
        /// The step, from 1.
        step: usize,
        /// Why.
        why: Failure,
    },
    /// The recording can't say: too short, no position, nothing asked changed.
    NotMeasured(&'static str),
}

/// How a step failed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Failure {
    /// The time it never reached.
    Timing(Why),
    /// It ended this far off what was asked.
    Off(f32),
    /// The wheels never turned (the highest speed seen).
    NeverTurned(f32),
    /// The wheels never stopped (the speed at the end).
    NeverStopped(f32),
}

/// Most steps a test has: its recording holds no more.
const MAX_STEPS: usize = 8;
/// Most samples [`judge`] reads (a recording longer than this is read only this far):
/// the longest test, 6.4 s at 100 Hz. [`judge`] copies them onto the stack, about 7.5 KB:
/// call it from a task with room, not from an interrupt.
const MAX_SAMPLES: usize = 640;

/// The verdict of `test` from its recording.
#[must_use]
pub fn judge(test: BenchTest, samples: &[Sample]) -> Verdict {
    let samples = &samples[..samples.len().min(MAX_SAMPLES)];
    if samples.len() < 10 {
        return Verdict::NotMeasured("fewer than 10 samples");
    }
    match test {
        BenchTest::SteeringSteps => steps_follow(samples, |s| (s.steer_asked, s.steer), 0.03),
        BenchTest::BrakeTravel => steps_follow(samples, |s| (s.brake_asked, s.brake), 0.1),
        BenchTest::ThrottleSpinUp => spin_up(samples),
    }
}

fn steps_follow(samples: &[Sample], pick: impl Fn(&Sample) -> (f32, f32), within: f32) -> Verdict {
    let mut t = [0u32; MAX_SAMPLES];
    let mut asked = [0f32; MAX_SAMPLES];
    let mut measured = [0f32; MAX_SAMPLES];
    for (i, s) in samples.iter().enumerate() {
        let (a, m) = pick(s);
        if m.is_nan() {
            return Verdict::NotMeasured("no position from the actuator");
        }
        t[i] = s.at_ms;
        asked[i] = a;
        measured[i] = m;
    }
    let n = samples.len();
    let (found, count) = steps::<MAX_STEPS>(&asked[..n], within);
    if count == 0 {
        return Verdict::NotMeasured("nothing asked changed");
    }
    for (k, step) in found.iter().flatten().enumerate() {
        let r = respond(&t[..n], &measured[..n], *step, within);
        let failure = match (r.delay_ms, r.rise_ms) {
            (Err(Why::AlreadyThere), _) => None,
            (Err(why), _) | (Ok(_), Err(why)) => Some(Failure::Timing(why)),
            (Ok(_), Ok(_)) if r.off.abs() > within => Some(Failure::Off(r.off)),
            (Ok(_), Ok(_)) => None,
        };
        if let Some(why) = failure {
            return Verdict::Fail { step: k + 1, why };
        }
    }
    Verdict::Pass
}

fn spin_up(samples: &[Sample]) -> Verdict {
    if samples.iter().any(|s| s.speed.is_nan()) {
        return Verdict::NotMeasured("no wheel speed");
    }
    let top = samples.iter().map(|s| s.speed.abs()).fold(0.0, f32::max);
    if top < 0.1 {
        return Verdict::Fail {
            step: 1,
            why: Failure::NeverTurned(top),
        };
    }
    let end = samples.last().map_or(0.0, |s| s.speed.abs());
    if end >= 0.05 {
        return Verdict::Fail {
            step: 2,
            why: Failure::NeverStopped(end),
        };
    }
    Verdict::Pass
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A pretend steering actuator: a first-order lag, 10 ms ticks.
    fn steering_run(lag: f32, stuck_at: Option<f32>) -> ([Sample; 600], usize) {
        let mut run = Run::start(BenchTest::SteeringSteps, 0, Where::OnTheGround).expect("starts");
        let mut out = [Sample::default(); 600];
        let mut pos = 0.0f32;
        let mut n = 0;
        for i in 0..600u32 {
            let now = i * 10;
            run.heartbeat(now);
            let TestRequest::Steering(asked) = run.tick(now) else {
                break;
            };
            pos += (asked - pos) * lag;
            if let Some(limit) = stuck_at {
                pos = pos.min(limit);
            }
            out[n] = Sample {
                at_ms: now,
                steer_asked: asked,
                steer: pos,
                ..Sample::default()
            };
            n += 1;
        }
        (out, n)
    }

    #[test]
    fn a_good_steering_actuator_passes() {
        let (s, n) = steering_run(0.2, None);
        assert_eq!(n, 600, "6 s of moves");
        assert_eq!(judge(BenchTest::SteeringSteps, &s[..n]), Verdict::Pass);
    }

    #[test]
    fn one_stuck_short_of_the_left_fails_that_step() {
        let (s, n) = steering_run(0.2, Some(0.1));
        assert_eq!(
            judge(BenchTest::SteeringSteps, &s[..n]),
            Verdict::Fail {
                step: 1,
                why: Failure::Timing(Why::NeverArrived)
            }
        );
    }

    #[test]
    fn the_moves_come_in_order_then_hold() {
        let mut run = Run::start(BenchTest::BrakeTravel, 1000, Where::OnTheGround).expect("ok");
        let mut at = |t: u32| {
            run.heartbeat(t);
            run.tick(t)
        };
        assert_eq!(at(1000), TestRequest::Brake(0.0));
        assert_eq!(at(2600), TestRequest::Brake(1.0));
        assert_eq!(at(4100), TestRequest::Brake(0.0));
        assert_eq!(at(5600), TestRequest::Hold);
        assert_eq!(run.progress(), Progress::Finished);
        assert_eq!(BenchTest::BrakeTravel.length_ms(), 4500);
    }

    #[test]
    fn no_heartbeat_stops_the_run() {
        let mut run = Run::start(BenchTest::SteeringSteps, 0, Where::OnTheGround).expect("ok");
        assert_eq!(run.tick(50), TestRequest::Steering(0.0));
        assert_eq!(run.tick(200), TestRequest::Hold);
        assert_eq!(run.progress(), Progress::Stopped);
        run.heartbeat(210);
        assert_eq!(run.tick(210), TestRequest::Hold, "stopped stays stopped");
    }

    #[test]
    fn the_throttle_test_only_on_stands() {
        assert!(Run::start(BenchTest::ThrottleSpinUp, 0, Where::OnTheGround).is_none());
        let mut run = Run::start(BenchTest::ThrottleSpinUp, 0, Where::OnStands).expect("ok");
        assert_eq!(run.tick(0), TestRequest::Throttle(CREEP));
    }

    #[test]
    fn spin_up_needs_the_wheels_to_turn_and_stop() {
        let mut s = [Sample::default(); 50];
        assert!(matches!(
            judge(BenchTest::ThrottleSpinUp, &s),
            Verdict::Fail {
                step: 1,
                why: Failure::NeverTurned(_)
            }
        ));
        for (i, x) in s.iter_mut().enumerate() {
            x.speed = if (10..30).contains(&i) { 0.4 } else { 0.0 };
        }
        assert_eq!(judge(BenchTest::ThrottleSpinUp, &s), Verdict::Pass);
        s[49].speed = 0.3;
        assert!(matches!(
            judge(BenchTest::ThrottleSpinUp, &s),
            Verdict::Fail {
                step: 2,
                why: Failure::NeverStopped(_)
            }
        ));
        s[0].speed = f32::NAN;
        assert_eq!(
            judge(BenchTest::ThrottleSpinUp, &s),
            Verdict::NotMeasured("no wheel speed")
        );
    }

    #[test]
    fn no_position_is_not_measured() {
        let s = [Sample {
            steer: f32::NAN,
            ..Sample::default()
        }; 20];
        assert_eq!(
            judge(BenchTest::SteeringSteps, &s),
            Verdict::NotMeasured("no position from the actuator")
        );
    }
}
