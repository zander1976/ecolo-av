//! A short recording in RAM: one [`Sample`] a tick, for a few seconds, then read out.
//!
//! [`Recorder::start`] clears it and records until it is full or the time is up; nothing is
//! kept before the start. `N` samples of 28 bytes: 500 (5 s at 100 Hz) is 14 KB, a small
//! part of the ESP32's 520 KB.

/// One tick's numbers: what was asked, and what the actuators did.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Sample {
    /// ms since the recording started.
    pub at_ms: u32,
    /// The spine's steering target, rad (+ left).
    pub steer_asked: f32,
    /// The steering actuator's position, rad (NaN: no feedback).
    pub steer: f32,
    /// The brake asked for, 0..1.
    pub brake_asked: f32,
    /// The brake actuator's position, 0..1 (NaN: no feedback).
    pub brake: f32,
    /// The throttle asked for, 0..1.
    pub throttle: f32,
    /// The measured speed, m/s (NaN: none).
    pub speed: f32,
}

/// What the recorder is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// Nothing asked.
    Idle,
    /// Recording, until this many ms after the start.
    Recording {
        /// When it started (the board's clock, ms).
        from_ms: u32,
        /// How long.
        for_ms: u32,
    },
    /// Done: full, or the time was up.
    Done,
}

/// The recording.
#[derive(Debug, Clone)]
pub struct Recorder<const N: usize> {
    samples: [Sample; N],
    len: usize,
    state: State,
}

impl<const N: usize> Default for Recorder<N> {
    fn default() -> Self {
        Self {
            samples: [Sample::default(); N],
            len: 0,
            state: State::Idle,
        }
    }
}

impl<const N: usize> Recorder<N> {
    /// Clears it and starts: `for_ms` from `now_ms`, or until full.
    pub fn start(&mut self, now_ms: u32, for_ms: u32) {
        self.len = 0;
        self.state = State::Recording {
            from_ms: now_ms,
            for_ms,
        };
    }

    /// Ends it now (a test aborted, or the page asked).
    pub fn stop(&mut self) {
        if matches!(self.state, State::Recording { .. }) {
            self.state = State::Done;
        }
    }

    /// This tick's numbers; `at_ms` is set from `now_ms`. Ignored unless recording.
    pub fn push(&mut self, now_ms: u32, mut sample: Sample) {
        let State::Recording { from_ms, for_ms } = self.state else {
            return;
        };
        let at = now_ms.wrapping_sub(from_ms);
        if at > for_ms || self.len == N {
            self.state = State::Done;
            return;
        }
        sample.at_ms = at;
        self.samples[self.len] = sample;
        self.len += 1;
        if self.len == N {
            self.state = State::Done;
        }
    }

    /// What it is doing.
    #[must_use]
    pub fn state(&self) -> State {
        self.state
    }

    /// The samples so far, oldest first.
    #[must_use]
    pub fn samples(&self) -> &[Sample] {
        &self.samples[..self.len]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn records_from_the_start_until_the_time_is_up() {
        let mut r = Recorder::<100>::default();
        r.push(0, Sample::default());
        assert!(r.samples().is_empty(), "nothing before the start");
        r.start(1000, 50);
        for t in (1000..1100).step_by(10) {
            r.push(
                t,
                Sample {
                    steer: 0.1,
                    ..Sample::default()
                },
            );
        }
        assert_eq!(r.state(), State::Done);
        let at: [u32; 6] = [0, 10, 20, 30, 40, 50];
        assert!(r.samples().iter().map(|s| s.at_ms).eq(at));
    }

    #[test]
    fn stops_when_full() {
        let mut r = Recorder::<3>::default();
        r.start(0, 10_000);
        for t in 0..10 {
            r.push(t * 10, Sample::default());
        }
        assert_eq!(r.samples().len(), 3);
        assert_eq!(r.state(), State::Done);
        r.start(500, 100);
        assert!(r.samples().is_empty(), "a start clears it");
        r.stop();
        assert_eq!(r.state(), State::Done);
    }
}
