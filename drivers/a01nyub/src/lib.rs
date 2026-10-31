//! DFRobot A01NYUB (SEN0313) and A02YYUW (SEN0311) waterproof ultrasonic
//! distance sensors, UART output.
//!
//! References: DFRobot wiki "A01NYUB Waterproof Ultrasonic Sensor"
//! (SKU SEN0313) and "A02YYUW Waterproof Ultrasonic Sensor" (SKU SEN0311).
//!
//! * UART 9600 8N1, sensor TX only.
//! * Frame: `0xFF, DATA_H, DATA_L, SUM` with
//!   `SUM = (0xFF + DATA_H + DATA_L) & 0xFF`; distance in mm.
//!   Wiki example: `FF 07 A1 A7` = 1953 mm.
//! * Output mode is set by the level on the sensor's RX pin: floating /
//!   high = processed (filtered) value every 100–300 ms, low = real-time
//!   value every ~100 ms ([`OutputMode`], [`set_output_mode`]).
//!
//! The sensor streams continuously, so `start_measurement` returns `0` and
//! `read_measurement` returns the newest frame received since the last
//! call, or [`Error::NotReady`].

#![no_std]

use embedded_hal::digital::OutputPin;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// UART baud rate.
pub const BAUD_RATE: u32 = 9_600;
/// Frame header byte.
pub const HEADER: u8 = 0xFF;

/// Range limits (flat target), mm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// A01NYUB: 280–7500 mm, ≤ 15 mA.
    A01nyub,
    /// A02YYUW: 30–4500 mm, < 8 mA.
    A02yyuw,
}

impl Variant {
    /// Blind zone (minimum distance) in mm.
    pub const fn min_mm(self) -> u16 {
        match self {
            Self::A01nyub => 280,
            Self::A02yyuw => 30,
        }
    }

    /// Maximum rated distance in mm.
    pub const fn max_mm(self) -> u16 {
        match self {
            Self::A01nyub => 7_500,
            Self::A02yyuw => 4_500,
        }
    }
}

/// Selected by the level of the sensor's RX pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OutputMode {
    /// RX high or floating: filtered value (default).
    #[default]
    Processed,
    /// RX low: unfiltered real-time value.
    RealTime,
}

/// Drive the GPIO wired to the sensor's RX pin for `mode`.
pub fn set_output_mode<P: OutputPin>(pin: &mut P, mode: OutputMode) -> Result<(), P::Error> {
    match mode {
        OutputMode::Processed => pin.set_high(),
        OutputMode::RealTime => pin.set_low(),
    }
}

/// Checksum of a frame (low byte of the sum of the first three bytes).
pub const fn checksum(data_h: u8, data_l: u8) -> u8 {
    HEADER.wrapping_add(data_h).wrapping_add(data_l)
}

/// Decode a complete 4-byte frame.
pub fn decode_frame<E>(frame: &[u8; 4]) -> Result<u16, Error<E>> {
    if frame[0] != HEADER {
        return Err(Error::InvalidData);
    }
    if checksum(frame[1], frame[2]) != frame[3] {
        return Err(Error::Crc);
    }
    Ok(u16::from_be_bytes([frame[1], frame[2]]))
}

/// Incremental frame parser; resynchronises on the 0xFF header.
#[derive(Debug, Clone, Default)]
pub struct Parser {
    buf: [u8; 4],
    len: usize,
    checksum_errors: u32,
}

impl Parser {
    /// Empty parser.
    pub const fn new() -> Self {
        Self {
            buf: [0; 4],
            len: 0,
            checksum_errors: 0,
        }
    }

    /// Frames dropped because of a bad checksum (wrapping counter).
    pub const fn checksum_errors(&self) -> u32 {
        self.checksum_errors
    }

    /// Feed one received byte; returns the distance in mm when a valid
    /// frame completes.
    pub fn feed(&mut self, byte: u8) -> Option<u16> {
        if self.len == 0 && byte != HEADER {
            return None;
        }
        self.buf[self.len] = byte;
        self.len += 1;
        if self.len < 4 {
            return None;
        }
        match decode_frame::<()>(&self.buf) {
            Ok(mm) => {
                self.len = 0;
                Some(mm)
            }
            Err(_) => {
                self.checksum_errors = self.checksum_errors.wrapping_add(1);
                // A data byte may have been 0xFF: restart at the next header.
                let next = self.buf[1..].iter().position(|&b| b == HEADER);
                match next {
                    Some(i) => {
                        self.buf.copy_within(i + 1.., 0);
                        self.len = 3 - i;
                    }
                    None => self.len = 0,
                }
                None
            }
        }
    }
}

/// One distance sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Distance in mm as sent by the sensor.
    pub distance: u16,
    /// `distance` is within the rated range of the variant.
    pub in_range: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Distance(u32::from(self.distance)));
    }
}

macro_rules! common {
    () => {
        /// Create a driver; no UART traffic.
        pub const fn new(uart: U, variant: Variant) -> Self {
            Self {
                uart,
                variant,
                parser: Parser::new(),
            }
        }

        /// Give the UART back.
        pub fn release(self) -> U {
            self.uart
        }

        fn measurement(&self, distance: u16) -> Measurement {
            Measurement {
                distance,
                in_range: (self.variant.min_mm()..=self.variant.max_mm()).contains(&distance),
            }
        }
    };
}

/// Blocking driver over a UART with `embedded_io::Read + ReadReady`.
pub struct A01nyub<U> {
    uart: U,
    variant: Variant,
    parser: Parser,
}

impl<U> A01nyub<U> {
    common!();
}

impl<U: embedded_io::Read + embedded_io::ReadReady> A01nyub<U> {
    /// Drain the bytes already received (never blocks) and return the
    /// newest complete frame. `NotReady` if none; `Crc` if only corrupt
    /// frames were received.
    pub fn poll(&mut self) -> Result<Measurement, Error<U::Error>> {
        let errors = self.parser.checksum_errors();
        let mut latest = None;
        let mut buf = [0u8; 16];
        while self.uart.read_ready().map_err(Error::bus)? {
            let n = self.uart.read(&mut buf).map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            for &b in &buf[..n] {
                if let Some(mm) = self.parser.feed(b) {
                    latest = Some(mm);
                }
            }
        }
        match latest {
            Some(mm) => Ok(self.measurement(mm)),
            None if self.parser.checksum_errors() != errors => Err(Error::Crc),
            None => Err(Error::NotReady),
        }
    }
}

impl<U: embedded_io::Read + embedded_io::ReadReady> zigbee_sensor::blocking::Sensor for A01nyub<U> {
    type BusError = U::Error;
    type Measurement = Measurement;

    /// Free-running: returns `0`.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        Ok(0)
    }

    /// See [`A01nyub::poll`].
    fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
        self.poll()
    }
}

/// Async driver over `embedded_io_async::Read`.
pub mod asynch {
    use super::{Measurement, Parser, Variant};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::A01nyub`].
    pub struct A01nyub<U> {
        uart: U,
        variant: Variant,
        parser: Parser,
    }

    impl<U> A01nyub<U> {
        common!();
    }

    impl<U: embedded_io_async::Read> A01nyub<U> {
        /// Await the next valid frame. Corrupt frames are skipped; after
        /// 8 consecutive corrupt frames returns [`Error::Crc`].
        pub async fn next(&mut self) -> Result<Measurement, Error<U::Error>> {
            let start = self.parser.checksum_errors();
            let mut byte = [0u8; 1];
            loop {
                let n = self.uart.read(&mut byte).await.map_err(Error::bus)?;
                if n == 1 {
                    if let Some(mm) = self.parser.feed(byte[0]) {
                        return Ok(self.measurement(mm));
                    }
                }
                if self.parser.checksum_errors().wrapping_sub(start) >= 8 {
                    return Err(Error::Crc);
                }
            }
        }
    }

    impl<U: embedded_io_async::Read> zigbee_sensor::asynch::Sensor for A01nyub<U> {
        type BusError = U::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
            Ok(0)
        }

        /// Awaits the next frame; see [`A01nyub::next`].
        async fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
            self.next().await
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::collections::VecDeque;
    use zigbee_sensor::blocking::Sensor;

    struct FakeUart(VecDeque<u8>);

    impl embedded_io::ErrorType for FakeUart {
        type Error = embedded_io::ErrorKind;
    }

    impl embedded_io::Read for FakeUart {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.0.len()).min(5);
            for b in &mut buf[..n] {
                *b = self.0.pop_front().unwrap();
            }
            Ok(n)
        }
    }

    impl embedded_io::ReadReady for FakeUart {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            Ok(!self.0.is_empty())
        }
    }

    impl embedded_io_async::Read for FakeUart {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            if self.0.is_empty() {
                return Err(embedded_io::ErrorKind::TimedOut);
            }
            embedded_io::Read::read(self, buf)
        }
    }

    #[test]
    fn wiki_example_frame() {
        assert_eq!(checksum(0x07, 0xA1), 0xA7);
        assert_eq!(decode_frame::<()>(&[0xFF, 0x07, 0xA1, 0xA7]), Ok(1_953));
        assert_eq!(
            decode_frame::<()>(&[0xFF, 0x07, 0xA1, 0xA6]),
            Err(Error::Crc)
        );
    }

    #[test]
    fn parser_skips_garbage_and_resyncs() {
        let mut p = Parser::new();
        let mut out = std::vec::Vec::new();
        // Garbage, a corrupt frame whose data contains 0xFF, then a valid
        // frame starting inside it.
        let stream = [
            0x12, 0x34, 0xFF, 0x00, 0xFF, 0x07, 0xA1, 0xA7, 0xFF, 0x01, 0x2C, 0x2C,
        ];
        for b in stream {
            if let Some(mm) = p.feed(b) {
                out.push(mm);
            }
        }
        assert_eq!(out, [1_953, 300]);
        assert_eq!(p.checksum_errors(), 1);
    }

    #[test]
    fn blocking_returns_newest_frame() {
        let rx = [0xFF, 0x07, 0xA1, 0xA7, 0xFF, 0x00, 0xC8, 0xC7, 0xFF, 0x01];
        let mut s = A01nyub::new(FakeUart(rx.into_iter().collect()), Variant::A02yyuw);
        assert_eq!(s.start_measurement(), Ok(0));
        assert_eq!(
            s.read_measurement(),
            Ok(Measurement {
                distance: 200,
                in_range: true
            })
        );
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        let mut uart = s.release();
        uart.0.extend([0x2C, 0x2C]);
        let mut s = A01nyub::new(uart, Variant::A01nyub);
        // Parser state was reset with the new driver: header lost.
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
    }

    #[test]
    fn blocking_reports_crc() {
        let rx = [0xFF, 0x07, 0xA1, 0x00];
        let mut s = A01nyub::new(FakeUart(rx.into_iter().collect()), Variant::A01nyub);
        assert_eq!(s.read_measurement(), Err(Error::Crc));
    }

    #[test]
    fn in_range_flag() {
        let s = A01nyub::new(FakeUart(VecDeque::new()), Variant::A01nyub);
        assert!(!s.measurement(200).in_range);
        assert!(s.measurement(1_953).in_range);
    }

    #[test]
    fn async_awaits_frame() {
        use zigbee_sensor::asynch::Sensor as _;
        let rx = [0x00, 0xFF, 0x07, 0xA1, 0xA7];
        let mut s = asynch::A01nyub::new(FakeUart(rx.into_iter().collect()), Variant::A01nyub);
        assert_eq!(block_on(s.read_measurement()).unwrap().distance, 1_953);
        assert_eq!(
            block_on(s.read_measurement()),
            Err(Error::Bus(embedded_io::ErrorKind::TimedOut))
        );
    }

    fn block_on<F: core::future::Future>(fut: F) -> F::Output {
        let mut cx = core::task::Context::from_waker(core::task::Waker::noop());
        let mut fut = core::pin::pin!(fut);
        loop {
            if let core::task::Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }
}
