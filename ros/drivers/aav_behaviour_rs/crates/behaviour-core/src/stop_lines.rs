//! The stop lines on the route (TODO 220): stop at each, then go, as at a seen stop sign
//! (`behaviour.py`'s `StopLines`).
//!
//! The planner sends the route's stop lines (`/drivers/nav/planner/route_stops`, from the
//! Lanelet2 map). Each one on the plan is a stop (`stop_line`, WAIT) whose `left` puts the
//! front bumper on the line. Standing within `near` m of it for `dwell` s serves it: it
//! goes, and the car goes on (GO, gently), unless something else holds it (a person
//! crossing: the moving hold). One passed by more than `near` m is served too. A served
//! line is forgotten once the car is `forget` m from it, so the next lap stops again.
//!
//! The map's stop lines too (`lanes.yaml`'s `stops`, the node's `map_stops`; the campus:
//! OpenStreetMap's stop signs, 26 Nov): each one with the way its traffic drives
//! ([`MapStop`]), so a plan only meets the stops of its own way. One counts when it is
//! within `off_path` m of the plan and the plan near it runs within `max_turn` of its
//! way; the cross road's lines (a car driving through the junction) and the other way's
//! never do. One as near as `same` to a route line is that line.
//!
//! A stop for the seen sign serves the line it stands for: the camera stops the car a
//! few metres short of the sign (the watcher's 4.7 m), so while the stop-sign lock holds
//! a standing car, every line up to `sign_reach` m ahead is served ([`StopLines::serve_near`]):
//! one stop at a sign, not two.
//!
//! ```text
//!   car ●──────────────▶│ line          left = along to the line - front
//!             front ◀──▶│
//! ```

use std::f64::consts::PI;

use crate::hypot;
use crate::paths::{path_left, Point};

/// When a stop line counts as stopped at. m, m/s, s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StopLineSettings {
    /// m: the car's stop within this of the line's: standing at it.
    pub near: f64,
    /// m/s: slower than this is standing.
    pub still: f64,
    /// s standing at it: a full stop, then go.
    pub dwell: f64,
    /// m: a line further than this off the plan isn't on the way.
    pub off_path: f64,
    /// m: a served line this far from the car is forgotten.
    pub forget: f64,
    /// m: two lines this close are the same line.
    pub same: f64,
    /// rad: a map stop counts only where the plan runs within this of its way.
    pub max_turn: f64,
    /// m: map stops further than this from the car aren't looked at (only to save time).
    pub look: f64,
    /// m ahead of the bumper a standing stop for the seen sign serves a line.
    pub sign_reach: f64,
}

impl Default for StopLineSettings {
    fn default() -> Self {
        Self {
            near: 1.5,
            still: 0.05,
            dwell: 2.0,
            off_path: 3.0,
            forget: 30.0,
            same: 1.0,
            max_turn: PI / 4.0,
            look: 80.0,
            sign_reach: 8.0,
        }
    }
}

/// A stop line on the map: its middle (map frame, m) and the way its traffic drives (rad).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapStop {
    /// The line's middle across the lanes that stop.
    pub at: Point,
    /// The way the traffic that stops there drives.
    pub heading: f64,
}

/// The route's stop lines and the map's, and which the car has stopped at.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct StopLines {
    s: StopLineSettings,
    lines: Vec<Point>,
    map: Vec<MapStop>,
    served: Vec<Point>,
    still_since: Option<f64>,
}

/// The plan's heading (rad) at its point nearest (x, y); None for fewer than two points.
fn heading_near(plan: &[Point], x: f64, y: f64) -> Option<f64> {
    plan.windows(2)
        .filter(|w| w[0] != w[1])
        .map(|w| {
            let (a, b) = (w[0], w[1]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let t = (((x - a.0) * dx + (y - a.1) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
            (hypot(a.0 + t * dx - x, a.1 + t * dy - y), dy.atan2(dx))
        })
        .min_by(|p, q| p.0.total_cmp(&q.0))
        .map(|(_, h)| h)
}

/// `angle` in (-pi, pi].
fn wrap(angle: f64) -> f64 {
    let a = angle.rem_euclid(2.0 * PI);
    if a > PI {
        a - 2.0 * PI
    } else {
        a
    }
}

impl StopLines {
    /// No lines yet.
    #[must_use]
    pub fn new(settings: StopLineSettings) -> Self {
        Self {
            s: settings,
            ..Self::default()
        }
    }

    /// Take the route's stop lines (map frame, in order).
    pub fn set_lines(&mut self, lines: Vec<Point>) {
        self.lines = lines;
    }

    /// Take the map's stop lines (`lanes.yaml`'s `stops`), kept for the whole run.
    pub fn set_map_stops(&mut self, stops: Vec<MapStop>) {
        self.map = stops;
    }

    /// The route's lines, then the map's that this plan meets on their own way.
    fn candidates(&self, plan: &[Point], car: Point) -> Vec<Point> {
        let s = self.s;
        let mut out = self.lines.clone();
        for stop in &self.map {
            let (x, y) = stop.at;
            if hypot(x - car.0, y - car.1) > s.look
                || out.iter().any(|p| hypot(p.0 - x, p.1 - y) < s.same)
            {
                continue;
            }
            if heading_near(plan, x, y).is_some_and(|h| wrap(h - stop.heading).abs() <= s.max_turn)
            {
                out.push(stop.at);
            }
        }
        out
    }

    fn is_served(&self, line: Point) -> bool {
        self.served
            .iter()
            .any(|p| hypot(line.0 - p.0, line.1 - p.1) < self.s.same)
    }

    /// The car stands for the seen sign: serve every line up to `sign_reach` m ahead (and
    /// any within `near` behind), so the sign and its line are one stop.
    pub fn serve_near(&mut self, plan: &[Point], car: Point, front: f64) {
        if plan.len() < 2 {
            return;
        }
        let s = self.s;
        let (car_left, _) = path_left(plan, car.0, car.1);
        for line in self.candidates(plan, car) {
            if self.is_served(line) {
                continue;
            }
            let (line_left, off) = path_left(plan, line.0, line.1);
            let left = car_left - line_left - front;
            if off <= s.off_path && (-s.near..=s.sign_reach).contains(&left) {
                self.served.push(line);
            }
        }
    }

    /// The unserved lines' `left`s (m from the car to its stop), nearest first.
    pub fn update(
        &mut self,
        now: f64,
        plan: &[Point],
        car: Option<Point>,
        speed: Option<f64>,
        front: f64,
    ) -> Vec<f64> {
        let s = self.s;
        let Some((x, y)) = car.filter(|_| plan.len() >= 2) else {
            self.still_since = None;
            return Vec::new();
        };
        self.served.retain(|p| hypot(p.0 - x, p.1 - y) <= s.forget);
        let (car_left, _) = path_left(plan, x, y);
        let mut ahead: Vec<(f64, Point)> = Vec::new();
        for line in self.candidates(plan, (x, y)) {
            if self.is_served(line) {
                continue;
            }
            let (line_left, off) = path_left(plan, line.0, line.1);
            if off > s.off_path {
                continue;
            }
            let left = car_left - line_left - front;
            if left < -s.near {
                // Passed it.
                self.served.push(line);
                continue;
            }
            ahead.push((left, line));
        }
        // The nearest (the first of equals, as Python's min()).
        let nearest =
            ahead
                .iter()
                .enumerate()
                .fold(None::<(usize, f64)>, |best, (i, a)| match best {
                    Some((_, b)) if b <= a.0 => best,
                    _ => Some((i, a.0)),
                });
        match nearest {
            Some((k, left)) if left.abs() <= s.near && speed.is_some_and(|v| v.abs() < s.still) => {
                let since = *self.still_since.get_or_insert(now);
                if now - since >= s.dwell {
                    self.served.push(ahead[k].1);
                    self.still_since = None;
                    ahead.remove(k);
                }
            }
            _ => self.still_since = None,
        }
        let mut lefts: Vec<f64> = ahead.into_iter().map(|a| a.0).collect();
        lefts.sort_by(f64::total_cmp);
        lefts
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stop_stand_then_go_and_stop_again_next_lap() {
        let plan: Vec<Point> = (0..=60).map(|i| (f64::from(i), 0.0)).collect();
        let mut lines = StopLines::new(StopLineSettings::default());
        // One on the way at x = 20, one 10 m off it.
        lines.set_lines(vec![(20.0, 0.5), (40.0, 10.0)]);
        let front = 2.0;
        let lefts = lines.update(0.0, &plan, Some((5.0, 0.0)), Some(2.0), front);
        assert_eq!(lefts.len(), 1);
        assert!((lefts[0] - 13.0).abs() < 1e-9, "{lefts:?}");
        // At the line, standing: still there until 2 s have passed, then gone.
        let at = Some((17.8, 0.0));
        assert_eq!(lines.update(1.0, &plan, at, Some(0.0), front).len(), 1);
        assert_eq!(lines.update(2.9, &plan, at, Some(0.0), front).len(), 1);
        assert!(lines.update(3.0, &plan, at, Some(0.0), front).is_empty());
        // Driving off: still served, then forgotten 30 m on (the line behind is off the
        // plan's end there, so not a stop).
        assert!(lines
            .update(4.0, &plan, Some((25.0, 0.0)), Some(2.0), front)
            .is_empty());
        assert!(lines
            .update(9.0, &plan, Some((52.0, 0.0)), Some(2.0), front)
            .is_empty());
        // A creep that never stands still is never served.
        let mut creep = StopLines::new(StopLineSettings::default());
        creep.set_lines(vec![(20.0, 0.0)]);
        for i in 0..100 {
            let got = creep.update(f64::from(i) * 0.1, &plan, at, Some(0.2), front);
            assert_eq!(got.len(), 1);
        }
        // No plan or no pose: nothing.
        assert!(creep
            .update(11.0, &plan[..1], at, Some(0.0), front)
            .is_empty());
        assert!(creep.update(11.0, &plan, None, Some(0.0), front).is_empty());
    }

    #[test]
    fn a_map_stop_counts_only_on_its_own_way() {
        // East along y = 0; the map's stops: one for traffic going east (ours), one for
        // traffic going west (the other lane), one for a cross road's traffic going north.
        let plan: Vec<Point> = (0..=60).map(|i| (f64::from(i), 0.0)).collect();
        let mut lines = StopLines::new(StopLineSettings::default());
        lines.set_map_stops(vec![
            MapStop {
                at: (20.0, -1.75),
                heading: 0.0,
            },
            MapStop {
                at: (30.0, 1.75),
                heading: PI,
            },
            MapStop {
                at: (40.0, -2.5),
                heading: PI / 2.0,
            },
        ]);
        let front = 2.0;
        let lefts = lines.update(0.0, &plan, Some((5.0, 0.0)), Some(2.0), front);
        assert_eq!(lefts.len(), 1, "{lefts:?}");
        assert!((lefts[0] - 13.0).abs() < 1e-9, "{lefts:?}");
        // Stood at it 2 s: served, as a route line is.
        let at = Some((17.8, 0.0));
        assert_eq!(lines.update(1.0, &plan, at, Some(0.0), front).len(), 1);
        assert!(lines.update(3.0, &plan, at, Some(0.0), front).is_empty());
        // The same line from the route and the map is one line.
        let mut both = StopLines::new(StopLineSettings::default());
        both.set_lines(vec![(20.0, -1.5)]);
        both.set_map_stops(vec![MapStop {
            at: (20.0, -1.75),
            heading: 0.0,
        }]);
        assert_eq!(
            both.update(0.0, &plan, Some((5.0, 0.0)), Some(2.0), front)
                .len(),
            1
        );
    }

    #[test]
    fn a_stop_for_the_seen_sign_serves_its_line() {
        let plan: Vec<Point> = (0..=60).map(|i| (f64::from(i), 0.0)).collect();
        let mut lines = StopLines::new(StopLineSettings::default());
        lines.set_map_stops(vec![
            MapStop {
                at: (20.0, -1.75),
                heading: 0.0,
            },
            MapStop {
                at: (40.0, -1.75),
                heading: 0.0,
            },
        ]);
        let front = 2.0;
        // The camera stopped the car 3.5 m short of the first line: it is served, the next
        // (18 m on) is not.
        lines.serve_near(&plan, (14.5, 0.0), front);
        let lefts = lines.update(0.0, &plan, Some((14.5, 0.0)), Some(0.0), front);
        assert_eq!(lefts.len(), 1, "{lefts:?}");
        assert!((lefts[0] - 23.5).abs() < 1e-9, "{lefts:?}");
    }

    #[test]
    fn the_heading_near_a_point_is_the_nearest_segments() {
        let plan = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)];
        assert!(heading_near(&plan, 5.0, -1.0).is_some_and(|h| h.abs() < 1e-9));
        assert!(heading_near(&plan, 11.0, 8.0).is_some_and(|h| (h - PI / 2.0).abs() < 1e-9));
        assert!(heading_near(&plan[..1], 0.0, 0.0).is_none());
        assert!((wrap(1.5 * PI) + PI / 2.0).abs() < 1e-12);
    }
}
