//! The corridor: two lines, not one (Ben, 26 Nov: "maybe instead of a line we use two
//! lines and the car just needs to stay between them"; `docs/designs/2026-11-26_corridor.md`).
//!
//! ```text
//!    left line  ─────────────────────────────────────
//!               ↑ left room = to the line − half the car − margin
//!    the plan   ═════════════════════════════════════   free anywhere in between
//!               ↓ right room
//!    right line ─────────────────────────────────────
//! ```
//!
//! The tube's clip with two numbers instead of one: the target for e_y is e_y clipped to
//! `[−right room, +left room]` at each point of the horizon ([`target`]). A room below 0
//! (a lane narrower than the car and its margins) is 0: the car keeps to the plan there.
//! Where no lane is known, the old ±tube.
//!
//! The lines come from the lane map (`lanes.yaml`, the behaviour planner's reader,
//! `behaviour-core`'s [`LaneMap`]): the car's own lane, its two painted lines.
//!
//! The idea: Apollo's and Autoware's path optimizers bound the sideways offset by the
//! drivable area's edges, point by point (`docs/acknowledgements.md`).

use std::fmt;

use behaviour_core::lanes::{self, LaneMap, LanesError};

use crate::model::clip;
use crate::path::Path;

/// m between the corridor's samples along the path.
pub const STEP: f64 = 0.5;
/// m before the path's end over which the room shrinks to 0, so the car comes to the
/// goal (Nav2's goal check: within 1 m) and not past it beside the plan.
pub const TO_GOAL: f64 = 5.0;

/// m from a point to its lane's two lines (below 0: the point is past that line).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lines {
    /// To the left line.
    pub left: f64,
    /// To the right line.
    pub right: f64,
}

/// What the car keeps from each line: half its width (vehicle.yaml) and a margin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Keep {
    /// m: half the car's width.
    pub half_width: f64,
    /// m beyond that.
    pub margin: f64,
}

impl Keep {
    /// Both finite and 0 or more.
    #[must_use]
    pub fn valid(&self) -> bool {
        [self.half_width, self.margin]
            .iter()
            .all(|v| v.is_finite() && *v >= 0.0)
    }
}

/// m each side of the plan where being off it costs nothing (0 or more).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Room {
    /// Toward the left line.
    pub left: f64,
    /// Toward the right line.
    pub right: f64,
}

impl Room {
    /// The room between these lines: each side's distance, less half the car and the
    /// margin, floored at 0.
    #[must_use]
    pub fn between(lines: Lines, keep: Keep) -> Room {
        let reserve = keep.half_width + keep.margin;
        Room {
            left: (lines.left - reserve).max(0.0),
            right: (lines.right - reserve).max(0.0),
        }
    }

    /// Both sides times `share` (0 to 1).
    #[must_use]
    pub fn scaled(self, share: f64) -> Room {
        Room {
            left: self.left * share,
            right: self.right * share,
        }
    }

    /// The tighter of two rooms, side by side.
    #[must_use]
    pub fn min(self, other: Room) -> Room {
        Room {
            left: self.left.min(other.left),
            right: self.right.min(other.right),
        }
    }
}

/// e_y's target: e_y clipped into the room (`[−right, +left]`), or into ±`tube` where no
/// lane is known. Inside, the target is where the plan already is: no pull to the centre.
#[must_use]
pub fn target(e_y: f64, room: Option<Room>, tube: f64) -> f64 {
    let (low, high) = room.map_or((-tube, tube), |r| (-r.right, r.left));
    clip(e_y, low, high)
}

/// The room along one path, a sample every [`STEP`] m (`None`: no lane known there).
#[derive(Debug, Clone, PartialEq)]
pub struct Corridor {
    rooms: Vec<Option<Room>>,
}

impl Corridor {
    /// The room along `path`, from `lines` at each sample: (x, y, heading) in the path's
    /// frame to the lines there, or `None` where no lane is known. Over the last
    /// [`TO_GOAL`] m it shrinks to 0 at the end.
    #[must_use]
    pub fn along(
        path: &Path,
        keep: Keep,
        lines: impl Fn(f64, f64, f64) -> Option<Lines>,
    ) -> Corridor {
        let n = (path.length() / STEP).ceil() as usize + 1;
        let rooms = (0..n)
            .map(|i| {
                let s = (i as f64 * STEP).min(path.length());
                let (x, y) = path.point_at(s);
                let share = ((path.length() - s) / TO_GOAL).clamp(0.0, 1.0);
                lines(x, y, path.heading_at(s)).map(|l| Room::between(l, keep).scaled(share))
            })
            .collect();
        Corridor { rooms }
    }

    /// The room `s` m along the path: the tighter of the two samples round it that are
    /// known; `None` if neither is (or before the start, past the end: the nearest).
    #[must_use]
    pub fn room_at(&self, s: f64) -> Option<Room> {
        let last = self.rooms.len().checked_sub(1)?;
        let at = (s.max(0.0) / STEP).floor();
        let i = if at.is_finite() {
            (at as usize).min(last)
        } else {
            last
        };
        let (a, b) = (self.rooms[i], self.rooms[(i + 1).min(last)]);
        match (a, b) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (one, other) => one.or(other),
        }
    }

    /// The samples known, of all.
    #[must_use]
    pub fn known(&self) -> (usize, usize) {
        let known = self.rooms.iter().filter(|r| r.is_some()).count();
        (known, self.rooms.len())
    }
}

/// The lines from the car's lane in the lane map, at (x, y) driving `heading` (map frame):
/// its left and right painted lines, measured from the point (not the lane's centre).
/// `None` off every road.
#[must_use]
pub fn lines_at(map: &LaneMap, x: f64, y: f64, heading: f64) -> Option<Lines> {
    let lane = map.lane_at(x, y, heading)?;
    // m the point is left of the lane's centre, the way the lane runs.
    let left_of = (x - lane.x) * -lane.heading.sin() + (y - lane.y) * lane.heading.cos();
    Some(Lines {
        left: lane.left_soft - left_of,
        right: lane.right_soft + left_of,
    })
}

/// A pose of the path's frame (odom) in the lane map's (map): x, y (m), yaw (rad).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Frame {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad.
    pub yaw: f64,
}

impl Frame {
    /// (x, y, heading) of the path's frame, in the map's.
    #[must_use]
    pub fn to_map(&self, x: f64, y: f64, heading: f64) -> (f64, f64, f64) {
        let (s, c) = self.yaw.sin_cos();
        (
            self.x + c * x - s * y,
            self.y + s * x + c * y,
            heading + self.yaw,
        )
    }
}

/// Why the corridor can't be set up.
#[derive(Debug)]
pub enum CorridorError {
    /// The lane map's file.
    Lanes(LanesError),
    /// Half the width or the margin below 0, or not a number.
    Keep,
}

impl fmt::Display for CorridorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CorridorError::Lanes(e) => e.fmt(f),
            CorridorError::Keep => f.write_str("the corridor: half_width and margin 0 or more"),
        }
    }
}

impl std::error::Error for CorridorError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            CorridorError::Lanes(e) => Some(e),
            CorridorError::Keep => None,
        }
    }
}

/// The lane map and what the car keeps from its lines: makes each path's corridor.
#[derive(Debug, Clone)]
pub struct Lanes {
    map: LaneMap,
    keep: Keep,
}

impl Lanes {
    /// These lanes, kept from by `keep`.
    ///
    /// # Errors
    ///
    /// [`CorridorError::Keep`] for a bad `keep`.
    pub fn new(map: LaneMap, keep: Keep) -> Result<Self, CorridorError> {
        if !keep.valid() {
            return Err(CorridorError::Keep);
        }
        Ok(Lanes { map, keep })
    }

    /// The lanes in a `lanes.yaml`.
    ///
    /// # Errors
    ///
    /// [`CorridorError::Lanes`] if the file can't be read; [`CorridorError::Keep`].
    pub fn load(file: impl AsRef<std::path::Path>, keep: Keep) -> Result<Self, CorridorError> {
        Self::new(lanes::load(file).map_err(CorridorError::Lanes)?, keep)
    }

    /// How many roads the map has.
    #[must_use]
    pub fn roads(&self) -> usize {
        self.map.roads().len()
    }

    /// The corridor along `path`, whose frame is at `frame` in the map.
    #[must_use]
    pub fn corridor(&self, path: &Path, frame: Frame) -> Corridor {
        Corridor::along(path, self.keep, |x, y, h| {
            let (mx, my, mh) = frame.to_map(x, y, h);
            lines_at(&self.map, mx, my, mh)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEEP: Keep = Keep {
        half_width: 0.5,
        margin: 0.2,
    };

    fn straight(length: f64) -> Path {
        let n = (length / 0.1) as usize;
        let poses: Vec<_> = (0..=n).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect();
        Path::new(&poses).expect("a path")
    }

    /// A lane along x from `right` to `left` (y, m); none past `ends`.
    fn lane(right: f64, left: f64, ends: f64) -> impl Fn(f64, f64, f64) -> Option<Lines> {
        move |x, y, _| {
            (x <= ends).then_some(Lines {
                left: left - y,
                right: y - right,
            })
        }
    }

    #[test]
    fn a_straight_lane_gives_the_same_room_each_side() {
        // 3.5 m lane, the plan on its centre: 1.75 - 0.5 - 0.2 = 1.05 m each side.
        let c = Corridor::along(&straight(20.0), KEEP, lane(-1.75, 1.75, 100.0));
        for s in [0.0, 3.3, 10.0, 14.5] {
            let r = c.room_at(s).expect("known");
            assert!(
                (r.left - 1.05).abs() < 1e-9 && (r.right - 1.05).abs() < 1e-9,
                "{r:?}"
            );
        }
        assert_eq!(c.known(), (41, 41));
        // To the goal it shrinks: at 17.5 m the tighter of 17.5 m's (half) and 18 m's (0.4),
        // none at the end and past it.
        let r = c.room_at(17.5).expect("known");
        assert!(
            (r.left - 0.42).abs() < 1e-9 && (r.right - 0.42).abs() < 1e-9,
            "{r:?}"
        );
        for s in [20.0, 25.0] {
            assert_eq!(
                c.room_at(s),
                Some(Room {
                    left: 0.0,
                    right: 0.0
                })
            );
        }
    }

    #[test]
    fn a_plan_off_centre_has_a_lopsided_room() {
        // The plan 0.5 m right of the lane's centre: more room to the left.
        let c = Corridor::along(&straight(20.0), KEEP, lane(-1.25, 2.25, 100.0));
        let r = c.room_at(5.0).expect("known");
        assert!(
            (r.left - 1.55).abs() < 1e-9 && (r.right - 0.55).abs() < 1e-9,
            "{r:?}"
        );
        // So the clip: 1 m left is free, 1 m right is pulled back to 0.55.
        assert!((target(1.0, Some(r), 0.3) - 1.0).abs() < 1e-12);
        assert!((target(-1.0, Some(r), 0.3) + 0.55).abs() < 1e-12);
    }

    #[test]
    fn a_lane_narrower_than_the_car_has_no_room() {
        // 1.2 m between the lines; the car and its margins need 1.4 m.
        let c = Corridor::along(&straight(20.0), KEEP, lane(-0.6, 0.6, 100.0));
        let r = c.room_at(5.0).expect("known");
        assert_eq!(
            r,
            Room {
                left: 0.0,
                right: 0.0
            }
        );
        // Every offset is costed: the car keeps to the plan, as with no tube.
        assert_eq!(target(0.2, Some(r), 0.3), 0.0);
        assert_eq!(target(-0.2, Some(r), 0.3), 0.0);
    }

    #[test]
    fn with_no_lane_it_falls_back_to_the_tube() {
        // The lane ends at x = 5: past it, ±tube.
        let c = Corridor::along(&straight(20.0), KEEP, lane(-1.75, 1.75, 5.0));
        assert!(c.room_at(2.0).is_some());
        assert!(c.room_at(8.0).is_none());
        assert_eq!(target(0.8, c.room_at(8.0), 0.3), 0.3);
        assert_eq!(target(-0.8, c.room_at(8.0), 0.3), -0.3);
        assert_eq!(target(0.1, None, 0.3), 0.1);
        // No lane anywhere: none known.
        let none = Corridor::along(&straight(20.0), KEEP, |_, _, _| None);
        assert_eq!(none.known().0, 0);
        assert!(none.room_at(3.0).is_none());
    }

    #[test]
    fn between_two_samples_the_tighter_counts() {
        // The lane narrows at x = 5 (its left line 1.75 -> 1.0).
        let c = Corridor::along(&straight(20.0), KEEP, |x, y, _| {
            Some(Lines {
                left: if x < 5.0 { 1.75 } else { 1.0 } - y,
                right: y + 1.75,
            })
        });
        assert!((c.room_at(4.6).expect("known").left - 0.3).abs() < 1e-9);
        assert!((c.room_at(4.4).expect("known").left - 1.05).abs() < 1e-9);
    }

    #[test]
    fn the_lane_map_gives_its_lines_from_the_point() {
        // A two-way road 8 m wide along x, 2 lanes: the car's lane (driving east) is
        // y in [-3.5, 0] (painted 7 m: 3.5 m a lane), its centre y = -1.75.
        let map = lanes::from_yaml(
            "roads:\n- name: road\n  width: 8.0\n  lanes: 2\n  points: [[-20.0, 0.0], [100.0, 0.0]]\n",
        )
        .expect("a map");
        let l = lines_at(&map, 10.0, -1.75, 0.0).expect("on the road");
        assert!(
            (l.left - 1.75).abs() < 1e-9 && (l.right - 1.75).abs() < 1e-9,
            "{l:?}"
        );
        let l = lines_at(&map, 10.0, -1.25, 0.0).expect("on the road");
        assert!(
            (l.left - 1.25).abs() < 1e-9 && (l.right - 2.25).abs() < 1e-9,
            "{l:?}"
        );
        // Driving west: the other lane, its lines the other way round.
        let l = lines_at(&map, 10.0, 1.75, std::f64::consts::PI).expect("on the road");
        assert!(
            (l.left - 1.75).abs() < 1e-9 && (l.right - 1.75).abs() < 1e-9,
            "{l:?}"
        );
        assert!(lines_at(&map, 10.0, 30.0, 0.0).is_none());
        // Through a frame: odom 10 m east of map's origin, the path at odom y = -1.75.
        let lanes = Lanes::new(map, KEEP).expect("lanes");
        let path = Path::new(&[(0.0, -1.75, 0.0), (20.0, -1.75, 0.0)]).expect("a path");
        let frame = Frame {
            x: 10.0,
            y: 0.0,
            yaw: 0.0,
        };
        let r = lanes.corridor(&path, frame).room_at(5.0).expect("known");
        assert!(
            (r.left - 1.05).abs() < 1e-9 && (r.right - 1.05).abs() < 1e-9,
            "{r:?}"
        );
    }

    #[test]
    fn a_bad_keep_is_refused() {
        let bad = Keep {
            half_width: -0.1,
            margin: 0.2,
        };
        assert!(matches!(
            Lanes::new(LaneMap::default(), bad),
            Err(CorridorError::Keep)
        ));
    }
}
