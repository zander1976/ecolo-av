//! The virtual ESP32 on a laptop: frames in, the page's buttons and tests, a pretend car.

use super::*;
use legacy_link::{Flags, Frame, FrameV2};
use spine_core::protocol::Mode;
use spine_core::{TEST_MAX_SPEED, TEST_MAX_THROTTLE};

/// The virtual ESP32 and a pretend car: the steering follows its target with a lag.
struct Car {
    esp: VirtualEsp32,
    now: u32,
    steering: f32,
    speed: f32,
    on_stands: bool,
    /// The steering's time constant, s; `None`: stuck where it is.
    lag: Option<f32>,
    last: Option<TickOut>,
}

impl Car {
    fn new(frame_version: FrameVersion) -> Self {
        let settings = Settings {
            frame_version,
            ..Settings::default()
        };
        Car {
            esp: VirtualEsp32::new(settings, 0x1234_5678),
            now: 1000,
            steering: 0.0,
            speed: 0.0,
            on_stands: false,
            lag: Some(0.1),
            last: None,
        }
    }

    /// One 10 ms tick; the car moves after it.
    fn tick(&mut self) -> TickOut {
        self.now += 10;
        let out = self.esp.tick(
            self.now,
            Measured {
                speed: self.speed,
                steering: self.steering,
                brake: 0.0,
                on_stands: self.on_stands,
            },
        );
        if let Some(lag) = self.lag {
            if !out.drive.limp {
                self.steering += (out.drive.steering - self.steering) * (0.01 / lag).min(1.0);
            }
        }
        self.last = Some(out.clone());
        out
    }

    fn ticks(&mut self, n: usize) -> TickOut {
        (0..n).map(|_| self.tick()).last().expect("n > 0")
    }

    /// A 13-byte frame, then two ticks: the bridge's 50 Hz.
    fn v1(&mut self, steering: f32, speed: f32, accel: f32) -> TickOut {
        let frame = Frame {
            steering,
            speed,
            accel,
        };
        self.esp.feed(&frame.encode());
        self.tick();
        self.tick()
    }

    fn v1s(&mut self, n: usize, steering: f32, speed: f32, accel: f32) -> TickOut {
        (0..n)
            .map(|_| self.v1(steering, speed, accel))
            .last()
            .expect("n > 0")
    }

    /// A button held (its heartbeat every 50 ms) for `ms`.
    fn hold(&mut self, button: Button, ms: u32) -> TickOut {
        let mut out = None;
        for i in 0..ms / 10 {
            if i % 5 == 0 {
                self.esp.press(button, self.now + 10);
            }
            out = Some(self.tick());
        }
        out.expect("ms >= 10")
    }
}

#[test]
fn version_1_frames_drive_it_once_armed() {
    let mut car = Car::new(FrameVersion::V1);
    car.v1s(5, 0.0, 0.0, -1.0); // stop frames: the arming edge's first half
    let out = car.v1s(100, 0.5, 0.3, 0.0);
    assert_eq!(out.outputs.mode, Mode::Computer);
    assert!(out.drive.throttle > 0.0, "{:?}", out.drive);
    assert_eq!(out.drive.brake, 0.0);
    // +0.5 is the C++'s 2725 counts: left.
    assert!(out.drive.steering > 0.2, "{:?}", out.drive);
}

#[test]
fn version_2_frames_drive_it_and_the_uplink_echoes_the_counter() {
    let mut car = Car::new(FrameVersion::V2);
    let mut counter = 0u8;
    let mut out = None;
    for i in 0..150 {
        counter = counter.wrapping_add(1);
        let (speed, accel) = if i < 5 { (0.0, -1.0) } else { (0.3, 0.0) };
        let frame = FrameV2 {
            values: Frame {
                steering: 0.0,
                speed,
                accel,
            },
            counter,
            flags: Flags {
                armed: i >= 5,
                estop: false,
            },
        };
        car.esp.feed(&frame.encode());
        car.tick();
        out = Some(car.tick());
    }
    let out = out.expect("ticked");
    assert_eq!(out.outputs.mode, Mode::Computer);
    assert!(out.drive.throttle > 0.0);
    // The uplink echoes the Orin's counter back.
    let uplink = legacy_link::Uplink::decode(&out.uplink).or_else(|_| {
        let last = car.tick();
        legacy_link::Uplink::decode(&last.uplink)
    });
    assert!(uplink.is_ok_and(|u| u.echo == Some(counter)));
}

#[test]
fn the_uplink_goes_out_every_20_ms() {
    let mut car = Car::new(FrameVersion::V2);
    let bytes: usize = (0..100).map(|_| car.tick().uplink.len()).sum();
    assert_eq!(bytes, 50 * legacy_link::UPLINK_LEN);
}

#[test]
fn creep_on_stands_stays_under_the_test_limit() {
    let mut car = Car::new(FrameVersion::V1);
    car.on_stands = true;
    car.ticks(10);
    let mut top: f32 = 0.0;
    for i in 0..100 {
        if i % 5 == 0 {
            car.esp.press(Button::Creep, car.now + 10);
        }
        let out = car.tick();
        top = top.max(out.drive.throttle);
        assert!(out.outputs.throttle <= TEST_MAX_THROTTLE);
        assert!(
            out.drive.throttle <= TEST_MAX_THROTTLE + 0.002,
            "{:?}",
            out.drive
        );
    }
    assert_eq!(car.last.as_ref().map(|o| o.outputs.mode), Some(Mode::Test));
    assert!(top > 0.0, "creep moved nothing");
}

#[test]
fn creep_on_the_ground_gives_no_throttle() {
    let mut car = Car::new(FrameVersion::V1);
    car.ticks(10);
    let out = car.hold(Button::Creep, 1000);
    assert_eq!(out.drive.throttle, 0.0);
    assert_eq!(car.esp.refused(), Some(Refused::NotOnStands));
}

#[test]
fn creep_too_fast_leaves_the_test_mode() {
    let mut car = Car::new(FrameVersion::V1);
    car.on_stands = true;
    car.ticks(10);
    car.hold(Button::Creep, 300);
    car.speed = TEST_MAX_SPEED + 0.5; // the free wheels run away
    let out = car.hold(Button::Creep, 1500);
    assert_ne!(out.outputs.mode, Mode::Test);
    assert_eq!(out.drive.throttle, 0.0);
}

#[test]
fn a_button_let_go_holds_within_100_ms() {
    let mut car = Car::new(FrameVersion::V1);
    car.on_stands = true;
    car.ticks(10);
    let out = car.hold(Button::Creep, 500);
    assert!(out.drive.throttle > 0.0);
    // The last heartbeat was at most 40 ms ago; 110 ms more without one: held.
    let out = car.ticks(11);
    assert_eq!(out.drive.throttle, 0.0);
    assert!(out.drive.brake > 0.0);
    assert_eq!(car.esp.held(car.now), None);
}

#[test]
fn left_and_right_turn_the_wheels_through_the_test_mode() {
    let mut car = Car::new(FrameVersion::V1);
    car.ticks(10);
    for _ in 0..4 {
        car.hold(Button::Left, 60);
        car.hold(Button::Stop, 20);
    }
    let out = car.hold(Button::Left, 1500);
    assert_eq!(out.outputs.mode, Mode::Test);
    assert!(
        (out.outputs.steering_target - 0.25).abs() < 1e-3,
        "{:?}",
        out.outputs
    );
    assert!(
        car.steering > 0.2,
        "the wheels turned left: {}",
        car.steering
    );
    let out = car.hold(Button::Right, 1500);
    assert!((out.outputs.steering_target - 0.2).abs() < 1e-3);
}

#[test]
fn limp_turns_the_steering_motor_off_while_stopped() {
    let mut car = Car::new(FrameVersion::V1);
    car.v1s(5, 0.0, 0.0, -1.0);
    for i in 0..20 {
        if i % 2 == 0 {
            car.esp.press(Button::Limp, car.now + 10);
        }
        car.v1(0.0, 0.0, -1.0);
    }
    let (_, drive, state) = car.esp.last().expect("ticked");
    assert_eq!(state, CalState::Limp);
    assert!(drive.limp);
    assert_eq!(drive.throttle, 0.0);
    // A person turns the wheels: the outputs follow them, not a target.
    car.steering = 0.3;
    car.esp.press(Button::Limp, car.now + 10);
    let out = car.v1(0.0, 0.0, -1.0);
    assert!((out.drive.steering - 0.3).abs() < 1e-6);
}

#[test]
fn limp_is_refused_while_the_frames_ask_for_speed() {
    let mut car = Car::new(FrameVersion::V1);
    car.v1s(5, 0.0, 0.0, -1.0);
    for i in 0..40 {
        if i % 2 == 0 {
            car.esp.press(Button::Limp, car.now + 10);
        }
        car.v1(0.0, 0.3, 0.0);
    }
    let (out, drive, state) = car.esp.last().expect("ticked");
    assert_eq!(state, CalState::Refused(legacy_link::Refusal::Moving));
    assert!(!drive.limp);
    assert_eq!(out.mode, Mode::Computer);
}

/// Run a test to its end, RUN held all the way.
fn run_test(car: &mut Car, test: BenchTest) -> Outcome {
    car.ticks(10);
    car.esp.start_test(test, car.now).expect("starts");
    for i in 0..800 {
        if i % 5 == 0 {
            car.esp.test_heartbeat(car.now + 10);
        }
        car.tick();
        if car.esp.running().is_none() {
            break;
        }
    }
    car.esp.outcome().expect("a verdict")
}

#[test]
fn a_steering_step_test_passes_on_a_car_that_follows() {
    let mut car = Car::new(FrameVersion::V1);
    let outcome = run_test(&mut car, BenchTest::SteeringSteps);
    assert_eq!(outcome.progress, Progress::Finished);
    assert_eq!(outcome.verdict, Verdict::Pass, "{outcome:?}");
    let steps = car.esp.step_responses(Actuator::Steering);
    assert_eq!(steps.len(), 3, "{steps:?}");
    // The spine slews at its own rate, the car lags 0.1 s: the 0.4 rad step takes time.
    assert!(steps.iter().all(|r| r.rise_ms.is_ok()), "{steps:?}");
    assert_eq!(car.esp.running(), None);
}

#[test]
fn a_steering_step_test_fails_on_a_stuck_actuator() {
    let mut car = Car::new(FrameVersion::V1);
    car.lag = None;
    let outcome = run_test(&mut car, BenchTest::SteeringSteps);
    assert!(
        matches!(outcome.verdict, Verdict::Fail { step: 1, .. }),
        "{outcome:?}"
    );
}

#[test]
fn a_test_stops_when_run_is_let_go() {
    let mut car = Car::new(FrameVersion::V1);
    car.ticks(10);
    car.esp
        .start_test(BenchTest::BrakeTravel, car.now)
        .expect("starts");
    for i in 0..50 {
        if i % 5 == 0 {
            car.esp.test_heartbeat(car.now + 10);
        }
        car.tick();
    }
    car.ticks(20);
    assert_eq!(car.esp.running(), None);
    let outcome = car.esp.outcome().expect("a result");
    assert_eq!(outcome.progress, Progress::Stopped);
}

#[test]
fn the_throttle_test_needs_stands() {
    let mut car = Car::new(FrameVersion::V1);
    assert_eq!(
        car.esp.start_test(BenchTest::ThrottleSpinUp, car.now),
        Err(TestRefused::NotOnStands)
    );
}

#[test]
fn it_writes_the_status_lines() {
    let mut car = Car::new(FrameVersion::V1);
    let lines: Vec<String> = (0..150).flat_map(|_| car.tick().lines).collect();
    let kinds = ["hb", "status", "health", "config", "devices"];
    for kind in kinds {
        let tag = format!("{{\"t\":\"{kind}\",\"schema\":1,");
        assert!(lines.iter().any(|l| l.starts_with(&tag)), "no {kind} line");
    }
    let status = lines
        .iter()
        .find(|l| l.contains("\"t\":\"status\""))
        .expect("a status line");
    assert!(status.contains("\"steer_counts\":1950"), "{status}");
    assert!(lines
        .iter()
        .any(|l| l.contains("\"build\":\"virtual car\"")));
}
