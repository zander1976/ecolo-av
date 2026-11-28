//! The laptop's stand-ins for the board's parts: the clock and the wheel counter (the
//! [`Board`]), the I2C bus with healthy devices on it ([`StandInBus`], shared), the Motor
//! Pi's serial line as byte buffers, and an RC receiver that never speaks.

use std::collections::VecDeque;
use std::convert::Infallible;
use std::sync::{Arc, Mutex};

use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};
use spine_board::{Board, CanFrame, StandInBus, WheelCount};

/// The board: the simulator's clock, no e-stop pressed, a wheel counter turned by the
/// speed the simulator measured, no CAN.
#[derive(Debug, Clone)]
pub struct LaptopBoard {
    now_ms: u32,
    metres_per_count: f64,
    /// How far the wheel has rolled since boot, m (backwards: less).
    rolled_m: f64,
    wheel: WheelCount,
    last_ms: Option<u32>,
}

impl LaptopBoard {
    /// A counter of `metres_per_count` (the calibration's wheel).
    #[must_use]
    pub fn new(metres_per_count: f32) -> Self {
        LaptopBoard {
            now_ms: 0,
            metres_per_count: f64::from(metres_per_count),
            rolled_m: 0.0,
            wheel: WheelCount::default(),
            last_ms: None,
        }
    }

    /// The time now, and the speed the wheels turned at since the last call (m/s).
    pub fn measure(&mut self, now_ms: u32, speed: f32) {
        let dt_ms = self.last_ms.map_or(0, |t| now_ms.wrapping_sub(t).min(100));
        self.last_ms = Some(now_ms);
        self.now_ms = now_ms;
        if speed.is_finite() && self.metres_per_count > 0.0 {
            self.rolled_m += f64::from(speed) * f64::from(dt_ms) / 1000.0;
        }
        let count = (self.rolled_m / self.metres_per_count + 1e-9).floor();
        // Beyond i32 the count wraps, as a board's does.
        let count = (count as i64) as i32;
        if count != self.wheel.count {
            self.wheel = WheelCount {
                count,
                at_ms: now_ms,
            };
        }
    }
}

impl Board for LaptopBoard {
    fn now_ms(&mut self) -> u32 {
        self.now_ms
    }

    fn estop_open(&mut self) -> bool {
        false
    }

    fn wheel(&mut self) -> WheelCount {
        self.wheel
    }

    fn can_receive(&mut self) -> Option<CanFrame> {
        None
    }

    fn can_send(&mut self, _frame: &CanFrame) -> bool {
        true
    }
}

/// One handle on the shared stand-in bus, as `embedded-hal-bus` gives each driver on a
/// real one.
#[derive(Debug, Clone)]
pub struct BusHandle(pub Arc<Mutex<StandInBus>>);

impl ErrorType for BusHandle {
    type Error = ErrorKind;
}

impl I2c for BusHandle {
    fn transaction(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), ErrorKind> {
        // A poisoned lock: a device that doesn't answer, which the spine faults on.
        let mut bus = self.0.lock().map_err(|_| ErrorKind::Other)?;
        bus.transaction(address, ops)
    }
}

/// The Motor Pi's serial line into the board: bytes queued by [`ByteQueue::push`], read
/// by the link.
#[derive(Debug, Clone, Default)]
pub struct ByteQueue(Arc<Mutex<VecDeque<u8>>>);

impl ByteQueue {
    /// Bytes from the bridge.
    pub fn push(&self, bytes: &[u8]) {
        if let Ok(mut q) = self.0.lock() {
            q.extend(bytes);
        }
    }
}

impl embedded_io::ErrorType for ByteQueue {
    type Error = Infallible;
}

impl embedded_io::Read for ByteQueue {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Infallible> {
        let Ok(mut q) = self.0.lock() else {
            return Ok(0);
        };
        let n = buf.len().min(q.len());
        for (b, v) in buf.iter_mut().zip(q.drain(..n)) {
            *b = v;
        }
        Ok(n)
    }
}

impl embedded_io::ReadReady for ByteQueue {
    fn read_ready(&mut self) -> Result<bool, Infallible> {
        Ok(self.0.lock().is_ok_and(|q| !q.is_empty()))
    }
}

/// The board's TX back to the Motor Pi (the uplink): every byte written, until taken.
#[derive(Debug, Clone, Default)]
pub struct ByteSink(Arc<Mutex<Vec<u8>>>);

impl ByteSink {
    /// The bytes written since the last call.
    #[must_use]
    pub fn take(&self) -> Vec<u8> {
        self.0
            .lock()
            .map(|mut v| std::mem::take(&mut *v))
            .unwrap_or_default()
    }
}

impl embedded_io::ErrorType for ByteSink {
    type Error = Infallible;
}

impl embedded_io::Write for ByteSink {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Infallible> {
        if let Ok(mut v) = self.0.lock() {
            v.extend_from_slice(buf);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Infallible> {
        Ok(())
    }
}

/// The RC receiver's serial port, with no receiver on it: nothing ever arrives.
#[derive(Debug, Clone, Copy, Default)]
pub struct NoReceiver;

impl embedded_io::ErrorType for NoReceiver {
    type Error = Infallible;
}

impl embedded_io::Read for NoReceiver {
    fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Infallible> {
        Ok(0)
    }
}

impl embedded_io::ReadReady for NoReceiver {
    fn read_ready(&mut self) -> Result<bool, Infallible> {
        Ok(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_wheel_counts_what_the_car_rolled() {
        let mut b = LaptopBoard::new(0.125);
        for t in (0..=1000).step_by(10) {
            b.measure(t, 1.25); // 1.25 m/s for a second: 10 counts
        }
        assert_eq!(b.wheel().count, 10);
        for t in (1010..=1500).step_by(10) {
            b.measure(t, -1.25);
        }
        assert_eq!(b.wheel().count, 5);
        assert!(!b.estop_open());
    }

    #[test]
    fn the_queue_gives_back_what_it_was_fed() {
        use embedded_io::{Read, ReadReady};
        let mut q = ByteQueue::default();
        assert!(!q.read_ready().unwrap_or(true));
        q.push(&[1, 2, 3]);
        let mut buf = [0; 2];
        assert_eq!(q.read(&mut buf).ok(), Some(2));
        assert_eq!(buf, [1, 2]);
        assert_eq!(q.read(&mut buf).ok(), Some(1));
    }
}
