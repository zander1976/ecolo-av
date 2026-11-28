//! The page's buttons, held to move.
//!
//! The browser sends the button being held every 50 ms ([`Controls::press`]). Each tick
//! ([`Controls::tick`]) says what to hand the spine: while a button is held, its request;
//! once nothing has come for [`HOLD_MS`], the spine's `Hold` (throttle 0, brake on) and
//! the limp's end. A frozen tab, a lost WiFi or a closed laptop stops the car.
//!
//! | Button | Becomes |
//! |---|---|
//! | CREEP | `TestRequest::Throttle(CREEP)`, forward; **only on stands** |
//! | LEFT, RIGHT | `TestRequest::Steering`: [`TURN_STEP`] rad further each press, within the spine's steering angle |
//! | CENTRE | `TestRequest::Steering(0.0)` |
//! | BRAKE ON, BRAKE OFF | `TestRequest::Brake(1.0)`, `Brake(0.0)` |
//! | STOP | `TestRequest::Hold`, always allowed, never refused |
//! | LIMP | `CalRequest::Limp`: the steering motor off (SteerCal's rules) |
//!
//! Entering the spine's test mode takes two requests, `Exit` then `Enter` (as the spine
//! asks); [`Controls`] sends them before the first move, and one `Exit` when the page
//! lets go ([`Controls::leave`]).

use legacy_link::CalRequest;
use spine_core::{TestRequest, TEST_MAX_THROTTLE};

/// No heartbeat for this long: the button is let go. The spine's own command timeout.
pub const HOLD_MS: u32 = 100;
/// CREEP's throttle (0..1): under the spine's test-mode cap.
pub const CREEP: f32 = 0.08;
/// LEFT and RIGHT: rad a press.
pub const TURN_STEP: f32 = 0.05;

const _: () = assert!(CREEP <= TEST_MAX_THROTTLE);

/// A button on the page.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Button {
    /// Forward, slowly; on stands only.
    Creep,
    /// Steer further left.
    Left,
    /// Steer further right.
    Right,
    /// Steer straight.
    Centre,
    /// The brake on, fully.
    BrakeOn,
    /// The brake off.
    BrakeOff,
    /// Hold still now.
    Stop,
    /// The steering motor off, to turn the wheels by hand.
    Limp,
}

impl Button {
    /// Its name on the page and in the request, as the browser sends it.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Creep => "creep",
            Self::Left => "left",
            Self::Right => "right",
            Self::Centre => "centre",
            Self::BrakeOn => "brake_on",
            Self::BrakeOff => "brake_off",
            Self::Stop => "stop",
            Self::Limp => "limp",
        }
    }

    /// The button with this name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        [
            Self::Creep,
            Self::Left,
            Self::Right,
            Self::Centre,
            Self::BrakeOn,
            Self::BrakeOff,
            Self::Stop,
            Self::Limp,
        ]
        .into_iter()
        .find(|b| b.name() == name)
    }
}

/// Where the car is, as the page's own rule needs it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Where {
    /// Wheels off the ground: a person chose it (the RC switch, or the virtual car's jacks).
    OnStands,
    /// On the ground, or not known: no throttle from the page.
    OnTheGround,
}

/// Why the page itself refused a button (the spine and the link have their own reasons).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refused {
    /// CREEP, not on stands.
    NotOnStands,
}

/// What to hand the spine and the link this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    /// The spine's test mode: None to send nothing (the page is idle).
    pub test: Option<TestRequest>,
    /// The steering calibration's mode: `Off` unless LIMP is held.
    pub cal: CalRequest,
}

/// The buttons' state between ticks.
#[derive(Debug, Clone, Default)]
pub struct Controls {
    held: Option<(Button, u32)>,
    steering: f32,
    entry: Entry,
    refused: Option<Refused>,
}

/// How far into the spine's test mode the page is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Entry {
    /// Not asked.
    #[default]
    Out,
    /// `Exit` sent; `Enter` next.
    Exiting,
    /// In: the moves go out.
    In,
    /// The page let go: `Exit` next, then out.
    Leaving,
}

impl Controls {
    /// The browser's heartbeat: `button` is held at `now_ms`. LEFT and RIGHT move the
    /// steering target once a press (a new press, not a held one).
    pub fn press(&mut self, button: Button, now_ms: u32, max_steering: f32) {
        let new = self.held.is_none_or(|(b, _)| b != button);
        if new {
            match button {
                Button::Left => self.steering = (self.steering + TURN_STEP).min(max_steering),
                Button::Right => self.steering = (self.steering - TURN_STEP).max(-max_steering),
                Button::Centre => self.steering = 0.0,
                _ => {}
            }
        }
        self.held = Some((button, now_ms));
    }

    /// The page let go (closed, or left the Controls tab): out of the test mode.
    pub fn leave(&mut self) {
        self.held = None;
        if self.entry != Entry::Out {
            self.entry = Entry::Leaving;
        }
    }

    /// Why the last tick refused its button, if it did.
    #[must_use]
    pub fn refused(&self) -> Option<Refused> {
        self.refused
    }

    /// The button held now, if its heartbeat is fresh.
    #[must_use]
    pub fn held(&self, now_ms: u32) -> Option<Button> {
        self.held
            .filter(|&(_, at)| now_ms.wrapping_sub(at) <= HOLD_MS)
            .map(|(b, _)| b)
    }

    /// This tick's command.
    pub fn tick(&mut self, now_ms: u32, place: Where) -> Command {
        self.refused = None;
        if self.entry == Entry::Leaving {
            self.entry = Entry::Out;
            return Command {
                test: Some(TestRequest::Exit),
                cal: CalRequest::Off,
            };
        }
        let Some(button) = self.held(now_ms) else {
            self.held = None;
            return Command {
                test: (self.entry == Entry::In).then_some(TestRequest::Hold),
                cal: CalRequest::Off,
            };
        };
        if button == Button::Limp {
            return Command {
                test: (self.entry == Entry::In).then_some(TestRequest::Hold),
                cal: CalRequest::Limp,
            };
        }
        // Into the spine's test mode first: Exit, then Enter, then the moves.
        match self.entry {
            Entry::Out => {
                self.entry = Entry::Exiting;
                return Command {
                    test: Some(TestRequest::Exit),
                    cal: CalRequest::Off,
                };
            }
            Entry::Exiting => {
                self.entry = Entry::In;
                return Command {
                    test: Some(TestRequest::Enter),
                    cal: CalRequest::Off,
                };
            }
            Entry::In | Entry::Leaving => {}
        }
        let request = match button {
            Button::Creep if place == Where::OnStands => TestRequest::Throttle(CREEP),
            Button::Creep => {
                self.refused = Some(Refused::NotOnStands);
                TestRequest::Hold
            }
            Button::Left | Button::Right | Button::Centre => TestRequest::Steering(self.steering),
            Button::BrakeOn => TestRequest::Brake(1.0),
            Button::BrakeOff => TestRequest::Brake(0.0),
            Button::Stop | Button::Limp => TestRequest::Hold,
        };
        Command {
            test: Some(request),
            cal: CalRequest::Off,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAX: f32 = 0.5;

    fn held_for(c: &mut Controls, b: Button, from: u32, ticks: u32, place: Where) -> [Command; 4] {
        let mut last = [Command {
            test: None,
            cal: CalRequest::Off,
        }; 4];
        for i in 0..ticks {
            let now = from + i * 10;
            if i % 5 == 0 {
                c.press(b, now, MAX);
            }
            let cmd = c.tick(now, place);
            last.rotate_left(1);
            last[3] = cmd;
        }
        last
    }

    #[test]
    fn the_first_move_enters_test_mode_exit_then_enter() {
        let mut c = Controls::default();
        c.press(Button::BrakeOn, 0, MAX);
        assert_eq!(c.tick(0, Where::OnTheGround).test, Some(TestRequest::Exit));
        assert_eq!(
            c.tick(10, Where::OnTheGround).test,
            Some(TestRequest::Enter)
        );
        assert_eq!(
            c.tick(20, Where::OnTheGround).test,
            Some(TestRequest::Brake(1.0))
        );
    }

    #[test]
    fn let_go_for_100_ms_and_it_holds() {
        let mut c = Controls::default();
        held_for(&mut c, Button::BrakeOff, 0, 20, Where::OnTheGround);
        // The last heartbeat at 150 ms: fresh until 250.
        assert_eq!(
            c.tick(250, Where::OnTheGround).test,
            Some(TestRequest::Brake(0.0))
        );
        assert_eq!(
            c.tick(260, Where::OnTheGround).test,
            Some(TestRequest::Hold)
        );
        assert_eq!(c.held(260), None);
    }

    #[test]
    fn creep_only_on_stands() {
        let mut c = Controls::default();
        let ground = held_for(&mut c, Button::Creep, 0, 5, Where::OnTheGround);
        assert_eq!(ground[3].test, Some(TestRequest::Hold));
        assert_eq!(c.refused(), Some(Refused::NotOnStands));
        let stands = held_for(&mut c, Button::Creep, 100, 5, Where::OnStands);
        assert_eq!(stands[3].test, Some(TestRequest::Throttle(CREEP)));
        assert_eq!(c.refused(), None);
    }

    #[test]
    fn left_and_right_step_once_a_press_within_the_angle() {
        let mut c = Controls::default();
        held_for(&mut c, Button::Left, 0, 30, Where::OnTheGround);
        assert_eq!(
            c.tick(290, Where::OnTheGround).test,
            Some(TestRequest::Steering(TURN_STEP)),
            "held is one press"
        );
        // Twenty more presses, each a new one (STOP between): no further than the angle.
        for i in 0..20 {
            c.press(Button::Stop, 300 + i * 20, MAX);
            c.press(Button::Left, 310 + i * 20, MAX);
        }
        assert_eq!(
            c.tick(700, Where::OnTheGround).test,
            Some(TestRequest::Steering(MAX))
        );
        c.press(Button::Centre, 710, MAX);
        assert_eq!(
            c.tick(710, Where::OnTheGround).test,
            Some(TestRequest::Steering(0.0))
        );
        c.press(Button::Right, 720, MAX);
        assert_eq!(
            c.tick(720, Where::OnTheGround).test,
            Some(TestRequest::Steering(-TURN_STEP))
        );
    }

    #[test]
    fn limp_goes_to_the_link_and_ends_when_let_go() {
        let mut c = Controls::default();
        c.press(Button::Limp, 0, MAX);
        let cmd = c.tick(0, Where::OnTheGround);
        assert_eq!(cmd.cal, CalRequest::Limp);
        assert_eq!(cmd.test, None, "limp asks nothing of the test mode");
        assert_eq!(c.tick(200, Where::OnTheGround).cal, CalRequest::Off);
    }

    #[test]
    fn idle_sends_nothing_and_leave_exits() {
        let mut c = Controls::default();
        assert_eq!(c.tick(0, Where::OnStands).test, None);
        held_for(&mut c, Button::Stop, 0, 5, Where::OnStands);
        c.leave();
        assert_eq!(c.tick(100, Where::OnStands).test, Some(TestRequest::Exit));
        assert_eq!(c.tick(110, Where::OnStands).test, None);
    }

    #[test]
    fn names_go_both_ways() {
        for b in [
            Button::Creep,
            Button::Left,
            Button::Right,
            Button::Centre,
            Button::BrakeOn,
            Button::BrakeOff,
            Button::Stop,
            Button::Limp,
        ] {
            assert_eq!(Button::from_name(b.name()), Some(b));
        }
        assert_eq!(Button::from_name("reverse"), None);
    }
}
