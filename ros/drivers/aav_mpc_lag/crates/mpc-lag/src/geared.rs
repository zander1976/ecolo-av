//! The MPC forward and back: a whole path, its stretches one at a time (TODO 213).
//! What `aav_track/track_node.py` does around a tracker since TODO 174, here without ROS
//! so the node only moves messages.
//!
//! ```text
//! path ─▶ Gears (cut at cusps) ─▶ this stretch, mirrored if backward ─▶ MpcLag
//! car  ─▶ mirror ──────────────────────────────────────────────────────▶ MpcLag ─▶ (steer, v)
//!                                                     signs turned back ◀─┘
//! ```
//!
//! | Step | What |
//! |---|---|
//! | [`Geared::follow`] | the path cut at its cusps; one that reverses is refused when the car may not |
//! | [`Geared::advance`] | at the cusp (stopped there by the stretch's own end), the next stretch; the MPC turned round |
//! | [`Geared::step`] | the MPC on the car as it sees it; the command with the real signs, backwards at `reverse_speed` at most; held at 0 within `reach` of a cusp, so the car stands there for the change of gear |
//!
//! A path with no cusp is one forward stretch: the MPC gets it as it is, and every
//! command is the same as without this.

use crate::barrier::Rules;
use crate::corridor::{Corridor, Frame, Lanes};
use crate::gears::{mirror, CuspRules, Direction, Gears, Pose};
use crate::path::PathError;
use crate::tracker::{Command, MpcLag, Observed, TrackError};

/// The barriers' rules, and the round things: each (x, y, radius), m.
type Fence = (Rules, Vec<(f64, f64, f64)>);

/// Why a path can't be followed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FollowError {
    /// The path itself: empty, or not numbers.
    Path(PathError),
    /// The path reverses, and the car may not (`max_reverse_speed` 0).
    MayNotReverse,
}

impl std::fmt::Display for FollowError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FollowError::Path(e) => e.fmt(f),
            FollowError::MayNotReverse => {
                f.write_str("the path reverses, and this car may not (reversing:=false)")
            }
        }
    }
}

impl std::error::Error for FollowError {}

impl From<PathError> for FollowError {
    fn from(e: PathError) -> Self {
        FollowError::Path(e)
    }
}

/// The MPC with gears: a path that changes direction, one stretch at a time.
#[derive(Debug, Clone)]
pub struct Geared {
    mpc: MpcLag,
    gears: Option<Gears>,
    rules: CuspRules,
    /// Round things to keep off, (x, y, radius) in the path's frame, and how (`barrier`).
    fence: Option<Fence>,
    /// The lane map, for each forward stretch's corridor (none: the ±tube).
    lanes: Option<Lanes>,
    /// Where the path's frame is in the lane map's, for the next path (none: no corridor).
    frame: Option<Frame>,
    /// Each stretch's corridor, made when the path came (backwards: none).
    corridors: Vec<Option<Corridor>>,
    /// m/s: the MPC's cruise forwards (its settings' when made).
    forward: f64,
    /// m/s: the most backwards (0: never).
    reverse: f64,
}

impl Geared {
    /// The MPC, allowed to back up at `reverse_speed` m/s at most (0: forward only).
    #[must_use]
    pub fn new(mpc: MpcLag, reverse_speed: f64, rules: CuspRules) -> Self {
        Geared {
            forward: mpc.settings().cruise,
            mpc,
            gears: None,
            rules,
            fence: None,
            lanes: None,
            frame: None,
            corridors: Vec::new(),
            reverse: reverse_speed.max(0.0),
        }
    }

    /// The MPC inside.
    #[must_use]
    pub fn mpc(&self) -> &MpcLag {
        &self.mpc
    }

    /// The stretches, once a path is given.
    #[must_use]
    pub fn gears(&self) -> Option<&Gears> {
        self.gears.as_ref()
    }

    /// Keep the car off these round things from the next tick on: each (x, y, radius) in
    /// the path's frame, the radius with the car's own and a margin in it. None: no
    /// barriers. Forward stretches only.
    pub fn set_things(&mut self, rules: Rules, things: &[(f64, f64, f64)]) {
        self.fence = (!things.is_empty()).then(|| (rules, things.to_vec()));
    }

    /// Keep between the lane map's lines from the next path on (`corridor`): forward
    /// stretches only. None: the ±tube, as always.
    pub fn set_lanes(&mut self, lanes: Option<Lanes>) {
        self.lanes = lanes;
    }

    /// Where the next path's frame (odom) is in the lane map's (map). None: no corridor
    /// for it (the ±tube).
    pub fn set_frame(&mut self, frame: Option<Frame>) {
        self.frame = frame;
    }

    /// Forget the MPC's estimate and plan ([`MpcLag::forget`]).
    pub fn forget(&mut self) {
        self.mpc.forget();
    }

    /// Follow a new path (the planner's poses), from its first stretch. The estimate
    /// carries on.
    ///
    /// # Errors
    ///
    /// [`FollowError::Path`] for a path that can't be made; [`FollowError::MayNotReverse`]
    /// for one that reverses with `reverse_speed` 0.
    pub fn follow(&mut self, poses: &[Pose]) -> Result<(), FollowError> {
        if poses
            .iter()
            .any(|&(x, y, yaw)| !(x.is_finite() && y.is_finite() && yaw.is_finite()))
        {
            return Err(FollowError::Path(PathError::NotANumber));
        }
        let gears = Gears::new(poses, self.rules);
        if gears.any_backward() && self.reverse <= 0.0 {
            return Err(FollowError::MayNotReverse);
        }
        let path = gears.current().path()?;
        let turned = self
            .gears
            .as_ref()
            .is_some_and(|g| g.direction() != gears.direction());
        if turned {
            // The newest path starts the other way from the one running: the estimate too.
            self.mpc.turn_round();
        }
        self.set_cruise(gears.direction());
        // Each forward stretch's corridor, while the frame is the one the path came by: a
        // corridor is along the path (m), so the map correction's moves don't change it.
        self.corridors = match (&self.lanes, self.frame) {
            (Some(lanes), Some(frame)) => gears
                .parts()
                .iter()
                .map(|part| match (part.direction(), part.path()) {
                    (Direction::Forward, Ok(path)) => Some(lanes.corridor(&path, frame)),
                    _ => None,
                })
                .collect(),
            _ => Vec::new(),
        };
        self.mpc.reset(path);
        self.mpc
            .set_corridor(self.corridors.first().cloned().flatten());
        self.gears = Some(gears);
        Ok(())
    }

    fn set_cruise(&mut self, direction: Direction) {
        let cruise = match direction {
            Direction::Forward => self.forward,
            Direction::Backward => self.reverse.min(self.forward),
        };
        self.mpc.set_cruise(cruise);
    }

    /// Take the next stretch if the car is at the cusp: at (x, y), `v` m/s (signed), at
    /// `now` s. True when it changed (a new stretch: the progress check starts again).
    pub fn advance(&mut self, x: f64, y: f64, v: f64, now: f64) -> bool {
        let Some(left) = self.mpc.left_from(x, y) else {
            return false;
        };
        let Some(gears) = self.gears.as_mut() else {
            return false;
        };
        let before = gears.direction();
        if !gears.update(left, v, now) {
            return false;
        }
        let after = gears.direction();
        let index = gears.index();
        match gears.current().path() {
            Ok(path) => {
                if after != before {
                    self.mpc.turn_round();
                }
                self.set_cruise(after);
                self.mpc.reset(path);
                self.mpc
                    .set_corridor(self.corridors.get(index).cloned().flatten());
                true
            }
            // Can't happen: every stretch has two poses or more, all numbers.
            Err(_) => false,
        }
    }

    /// On the last stretch (or none): only then is the goal checked.
    #[must_use]
    pub fn on_last(&self) -> bool {
        self.gears.as_ref().is_none_or(Gears::is_last)
    }

    /// The goal: the whole path's last pose.
    #[must_use]
    pub fn goal(&self) -> Option<Pose> {
        self.gears.as_ref().and_then(Gears::goal)
    }

    /// The way the car drives now.
    #[must_use]
    pub fn direction(&self) -> Direction {
        self.gears
            .as_ref()
            .map_or(Direction::Forward, Gears::direction)
    }

    /// m from (x, y) to the goal: the rest of this stretch, and every one after.
    #[must_use]
    pub fn left(&self, x: f64, y: f64) -> f64 {
        let here = self.mpc.left_from(x, y).unwrap_or(0.0).max(0.0);
        here + self.gears.as_ref().map_or(0.0, Gears::left_after)
    }

    /// Move the path, every stretch (the held map -> odom moved), keeping the place on it.
    pub fn move_path(&mut self, f: impl Fn(f64, f64, f64) -> Pose) {
        if let Some(g) = self.gears.as_mut() {
            g.move_poses(&f);
        }
        // A rigid move: it commutes with the mirroring (yaw + pi).
        self.mpc.move_path(f);
    }

    /// One tick: the steering and set speed to send now, with the real car's signs (the
    /// speed negative backwards). `seen` is the car as measured; `dt` s since the last tick.
    /// The report's foot is on the stretch as the MPC sees it.
    ///
    /// # Errors
    ///
    /// As [`MpcLag::step`].
    pub fn step(&mut self, seen: &Observed, dt: f64) -> Result<Command, TrackError> {
        let direction = self.direction();
        // The barriers on the stretch now running, where the things are now. Forwards
        // only: backwards the MPC sees the car turned round, and its body's point with it.
        match (&self.fence, direction) {
            (Some((rules, things)), Direction::Forward) => self.mpc.fence(*rules, things)?,
            _ => self.mpc.clear_barriers(),
        }
        // At a cusp not yet taken: stand for the change of gear.
        let at_cusp = !self.on_last()
            && self
                .mpc
                .left_from(seen.x, seen.y)
                .is_some_and(|left| left <= self.rules.reach);
        self.mpc.hold(at_cusp);
        let cmd = self.mpc.step(&mirror(seen, direction), dt)?;
        let sign = direction.sign();
        Ok(Command {
            steer: sign * cmd.steer,
            speed: sign * cmd.speed,
            report: cmd.report,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tracker::{Limits, Settings};

    const LIMITS: Limits = Limits {
        wheelbase: 1.57,
        max_steer: 0.5,
        max_steer_rate: 0.6,
    };

    fn geared(reverse: f64) -> Geared {
        let mpc = MpcLag::new(LIMITS, Default::default(), Settings::default()).expect("an MPC");
        Geared::new(mpc, reverse, CuspRules::default())
    }

    #[test]
    fn a_thing_is_fenced_forwards_and_not_backwards() {
        use ego_core::stop_law::StopCurve;
        let rules = Rules::new(StopCurve::from_numbers(3.0, 2.0, 2.0).expect("a curve"));
        let seen = Observed::default();
        let mut g = geared(0.5);
        g.follow(&forward_then_back()).expect("a path");
        // No things: no barrier in the report.
        assert!(g
            .step(&seen, 0.05)
            .expect("a tick")
            .report
            .barrier
            .is_none());
        // A thing 3 m ahead, 2 m across with the car's own: 1.2 m from the body's point.
        g.set_things(rules, &[(3.0, 0.0, 1.0)]);
        let gap = g
            .step(&seen, 0.05)
            .expect("a tick")
            .report
            .barrier
            .expect("fenced");
        assert!((gap - 1.2).abs() < 1e-6, "{gap}");
        // Taken away again.
        g.set_things(rules, &[]);
        assert!(g
            .step(&seen, 0.05)
            .expect("a tick")
            .report
            .barrier
            .is_none());
        // On the backwards stretch: none, whatever is set.
        let mut back = geared(0.5);
        let behind: Vec<Pose> = (0..=40).map(|i| (-f64::from(i) * 0.1, 0.0, 0.0)).collect();
        back.follow(&behind).expect("a path");
        back.set_things(rules, &[(-3.0, 0.0, 1.0)]);
        assert_eq!(back.direction(), Direction::Backward);
        assert!(back
            .step(&seen, 0.05)
            .expect("a tick")
            .report
            .barrier
            .is_none());
    }

    #[test]
    fn the_corridor_is_set_forwards_and_not_backwards() {
        use crate::corridor::{Frame, Keep, Lanes};
        // A two-way road along x, 8 m wide: the lane driving east is y in [-3.5, 0].
        let map = behaviour_core::lanes::from_yaml(
            "roads:\n- name: road\n  width: 8.0\n  lanes: 2\n  points: [[-20.0, 0.0], [100.0, 0.0]]\n",
        )
        .expect("a map");
        let keep = Keep {
            half_width: 0.5,
            margin: 0.2,
        };
        let east: Vec<Pose> = (0..=200).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect();
        // No lanes, or no frame: no corridor.
        let mut g = geared(0.5);
        g.follow(&east).expect("a path");
        assert!(g.mpc().corridor().is_none());
        g.set_lanes(Some(Lanes::new(map, keep).expect("lanes")));
        g.follow(&east).expect("a path");
        assert!(g.mpc().corridor().is_none());
        // odom 1.75 m south of map's origin: the path on the lane's centre, 1.05 m each side.
        g.set_frame(Some(Frame {
            x: 0.0,
            y: -1.75,
            yaw: 0.0,
        }));
        g.follow(&east).expect("a path");
        let room = g
            .step(&Observed::default(), 0.05)
            .expect("a tick")
            .report
            .room
            .expect("a room");
        assert!((room.left - 1.05).abs() < 1e-9 && (room.right - 1.05).abs() < 1e-9);
        // A path that reverses: the forward stretch has one, the backwards one none.
        g.follow(&forward_then_back()).expect("a path");
        assert!(g.mpc().corridor().is_some());
        let back = Observed {
            x: 4.0,
            ..Observed::default()
        };
        assert!(g.advance(back.x, back.y, 0.0, 0.0));
        assert_eq!(g.direction(), Direction::Backward);
        assert!(g.mpc().corridor().is_none());
        assert!(g.step(&back, 0.05).expect("a tick").report.room.is_none());
    }

    fn forward_then_back() -> Vec<Pose> {
        let mut p: Vec<Pose> = (0..=40).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect();
        p.extend((1..=30).map(|i| (4.0 - i as f64 * 0.1, 0.0, 0.0)));
        p
    }

    #[test]
    fn a_path_that_reverses_is_refused_forward_only() {
        let mut g = geared(0.0);
        assert_eq!(
            g.follow(&forward_then_back()),
            Err(FollowError::MayNotReverse)
        );
        assert_eq!(g.follow(&[]), Err(FollowError::Path(PathError::Empty)));
        let nan = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.5, f64::NAN, 0.0)];
        assert_eq!(
            g.follow(&nan),
            Err(FollowError::Path(PathError::NotANumber))
        );
        let mut g = geared(0.5);
        assert_eq!(g.follow(&forward_then_back()), Ok(()));
        assert!(!g.on_last());
        assert!((g.left(0.0, 0.0) - 7.0).abs() < 1e-9);
    }

    #[test]
    fn backwards_it_asks_a_negative_speed_within_the_reverse_limit() {
        let mut g = geared(0.5);
        let back: Vec<Pose> = (0..=40).map(|i| (-(i as f64) * 0.1, 0.0, 0.0)).collect();
        g.follow(&back).expect("a path");
        assert_eq!(g.direction(), Direction::Backward);
        let mut seen = Observed::default();
        for _ in 0..40 {
            let cmd = g.step(&seen, 0.05).expect("a command");
            assert!((-0.5 - 1e-9..=0.0).contains(&cmd.speed), "{}", cmd.speed);
            assert!(cmd.steer.abs() < 1e-3);
            seen.v = cmd.speed;
            seen.x += cmd.speed * 0.05;
        }
        assert!(seen.x < -0.3, "backed up: {}", seen.x);
    }

    #[test]
    fn a_forward_path_gives_the_mpcs_own_commands() {
        let line: Vec<Pose> = (0..=100).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect();
        let mut g = geared(0.5);
        g.follow(&line).expect("a path");
        let mut mpc = MpcLag::new(LIMITS, Default::default(), Settings::default()).expect("MPC");
        mpc.reset(crate::Path::new(&line).expect("a path"));
        let seen = Observed {
            y: 0.3,
            ..Observed::default()
        };
        for _ in 0..5 {
            let a = g.step(&seen, 0.05).expect("a command");
            let b = mpc.step(&seen, 0.05).expect("a command");
            assert_eq!((a.steer, a.speed), (b.steer, b.speed));
        }
    }
}
