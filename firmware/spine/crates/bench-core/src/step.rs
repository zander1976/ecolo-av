//! How a measured value followed a step in what was asked: `tools/step_response.py`,
//! ported (the same rules, the same numbers; times in ms).
//!
//! | Number | Means |
//! |---|---|
//! | delay | from the step until the measured value has moved 10 % of the way |
//! | rise | from 10 % to 90 % of the way |
//! | end, off | the measured value at the end of the step (the mean of its last quarter), and that less the value asked |
//!
//! "The way" is from the measured value just before the step to the value asked. A number
//! the samples don't hold is `Err` with [`Why`]: nothing is guessed, fitted or smoothed.

/// The shares of the way that time the delay and the rise.
pub const START: f32 = 0.1;
/// See [`START`].
pub const END: f32 = 0.9;

/// One step in what was asked: samples `first..last` hold the new value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Step {
    /// Its first sample.
    pub first: usize,
    /// One past its last.
    pub last: usize,
    /// What was asked before.
    pub asked_from: f32,
    /// What was asked now.
    pub asked_to: f32,
}

/// Why a time isn't there.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Why {
    /// The measured value was already where it was asked to go.
    AlreadyThere,
    /// It never moved 10 % of the way while the step was held.
    NeverMoved,
    /// It moved, but never got 90 % of the way.
    NeverArrived,
}

/// How the measured value answered one step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Response {
    /// The step.
    pub step: Step,
    /// When it was asked, ms.
    pub at_ms: u32,
    /// How long it was held, ms.
    pub held_ms: u32,
    /// The measured value just before.
    pub before: f32,
    /// The delay, ms.
    pub delay_ms: Result<u32, Why>,
    /// The 10 to 90 % rise, ms.
    pub rise_ms: Result<u32, Why>,
    /// The measured value at the end.
    pub end: f32,
    /// `end` less what was asked.
    pub off: f32,
}

/// The steps in `asked`: a jump of at least `least` from the level before. Jumps on
/// consecutive samples are one step; a wobble smaller than `least` moves nothing. At most
/// `M` steps are kept (`count` says how many there were).
#[must_use]
pub fn steps<const M: usize>(asked: &[f32], least: f32) -> ([Option<Step>; M], usize) {
    let mut found: [Option<Step>; M] = [None; M];
    let mut count = 0;
    let Some(&first) = asked.first() else {
        return (found, 0);
    };
    let mut level = first;
    let mut last_index: Option<usize> = None;
    for (i, &a) in asked.iter().enumerate().skip(1) {
        if (a - level).abs() < least {
            continue;
        }
        let same = last_index
            .is_some_and(|k| count > 0 && count <= M && found[k].is_some_and(|s| s.first == i - 1));
        if same {
            if let Some(k) = last_index {
                if let Some(s) = found[k].as_mut() {
                    s.asked_to = a;
                }
            }
        } else {
            if let Some(k) = last_index {
                if let Some(s) = found[k].as_mut() {
                    s.last = i;
                }
            }
            if count < M {
                found[count] = Some(Step {
                    first: i,
                    last: asked.len(),
                    asked_from: level,
                    asked_to: a,
                });
                last_index = Some(count);
            } else {
                last_index = None;
            }
            count += 1;
        }
        level = a;
    }
    (found, count)
}

/// How `measured` answered `step`; `t_ms` the samples' times. A way shorter than `least`
/// is not timed.
#[must_use]
pub fn respond(t_ms: &[u32], measured: &[f32], step: Step, least: f32) -> Response {
    let at_ms = t_ms[step.first];
    let before = if step.first > 0 {
        measured[step.first - 1]
    } else {
        measured[0]
    };
    let way = step.asked_to - before;
    let n = step.last - step.first;
    // round(n * 0.25), at least 1, as Python's round (half to even barely matters here).
    let tail = ((n as f32 * 0.25) + 0.5) as usize;
    let tail = tail.clamp(1, n.max(1));
    let tail_slice = &measured[step.last - tail..step.last];
    let end = tail_slice.iter().sum::<f32>() / tail_slice.len() as f32;
    let held_ms = t_ms[step.last - 1] - at_ms;
    let mut r = Response {
        step,
        at_ms,
        held_ms,
        before,
        delay_ms: Err(Why::AlreadyThere),
        rise_ms: Err(Why::AlreadyThere),
        end,
        off: end - step.asked_to,
    };
    if way.abs() < least {
        return r;
    }
    let sign = if way > 0.0 { 1.0 } else { -1.0 };
    let reached = |share: f32, from: usize| {
        (from..step.last).find(|&i| (measured[i] - before) * sign >= share * way.abs())
    };
    let Some(started) = reached(START, step.first) else {
        r.delay_ms = Err(Why::NeverMoved);
        r.rise_ms = Err(Why::NeverMoved);
        return r;
    };
    r.delay_ms = Ok(t_ms[started] - at_ms);
    r.rise_ms = match reached(END, started) {
        Some(there) => Ok(t_ms[there] - t_ms[started]),
        None => Err(Why::NeverArrived),
    };
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_are_jumps_and_wobbles_are_not() {
        let asked = [0.0, 0.0, 0.01, 1.0, 1.0, 1.0, 0.0, 0.0];
        let (found, n) = steps::<4>(&asked, 0.1);
        assert_eq!(n, 2);
        assert_eq!(
            found[0],
            Some(Step {
                first: 3,
                last: 6,
                asked_from: 0.0,
                asked_to: 1.0
            })
        );
        assert_eq!(
            found[1],
            Some(Step {
                first: 6,
                last: 8,
                asked_from: 1.0,
                asked_to: 0.0
            })
        );
    }

    #[test]
    fn a_jump_in_two_parts_is_one_step() {
        let asked = [0.0, 0.5, 1.0, 1.0];
        let (found, n) = steps::<4>(&asked, 0.1);
        assert_eq!(n, 1);
        assert_eq!(found[0].map(|s| (s.first, s.asked_to)), Some((1, 1.0)));
    }

    /// A first-order lag: 10 ms samples, the step at 100 ms, the value moving a fifth of the
    /// way left each sample. tools/step_response.py gives the same on these samples (checked
    /// 27 Nov: delay 0.01 s, rise 0.1 s, held 0.49 s).
    #[test]
    fn a_lag_gives_its_delay_and_rise() {
        let t: [u32; 60] = core::array::from_fn(|i| i as u32 * 10);
        let asked: [f32; 60] = core::array::from_fn(|i| if i >= 10 { 1.0 } else { 0.0 });
        let mut measured = [0.0f32; 60];
        for i in 11..60 {
            measured[i] = measured[i - 1] + (1.0 - measured[i - 1]) * 0.2;
        }
        let (found, _) = steps::<2>(&asked, 0.1);
        let r = respond(&t, &measured, found[0].expect("a step"), 0.05);
        // 0.2 at i=11 (>= 0.1): delay 10 ms; 0.9 first at i=21 (1 - 0.8^11 = 0.914).
        assert_eq!(r.delay_ms, Ok(10));
        assert_eq!(r.rise_ms, Ok(100));
        assert!(r.off.abs() < 0.01, "{}", r.off);
        assert_eq!(r.held_ms, 490);
    }

    #[test]
    fn never_moving_and_never_arriving_say_so() {
        let t: [u32; 10] = core::array::from_fn(|i| i as u32 * 10);
        let asked: [f32; 10] = core::array::from_fn(|i| if i >= 2 { 1.0 } else { 0.0 });
        let (found, _) = steps::<1>(&asked, 0.1);
        let step = found[0].expect("a step");
        let r = respond(&t, &[0.0; 10], step, 0.05);
        assert_eq!(r.delay_ms, Err(Why::NeverMoved));
        let half = [0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5];
        let r = respond(&t, &half, step, 0.05);
        assert_eq!((r.delay_ms, r.rise_ms), (Ok(0), Err(Why::NeverArrived)));
        let there = [1.0; 10];
        assert_eq!(
            respond(&t, &there, step, 0.05).delay_ms,
            Err(Why::AlreadyThere)
        );
    }
}
