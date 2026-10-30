//! Winsen MH-Z19B / MH-Z19C / MH-Z19E NDIR CO2 sensors over UART.
//!
//! UART 9600 8N1, 9-byte frames `FF <addr|cmd> b2..b7 <checksum>` with
//! checksum `(!(b1 + .. + b7)) + 1`. The sensor only answers requests
//! (query mode): [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! discards stale input, sends "read CO2" (0x86) and returns the expected
//! response time; [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//! parses whatever bytes are available and returns [`Error::NotReady`] if
//! the response is not complete yet.
//!
//! The driver needs [`embedded_io::ReadReady`] so it never blocks on a
//! silent sensor.
//!
//! References: Winsen "Intelligent Infrared CO2 Module (Model: MH-Z19B)
//! User's Manual" v1.0; MH-Z19C/MH-Z19E manuals use the same command set.
//!
//! ```ignore
//! let mut mhz = Mhz19::new(uart);
//! mhz.set_abc(false)?; // indoor, not ventilated to fresh air daily
//! let wait_us = mhz.start_measurement()?;
//! // ... wait_us later:
//! let m = mhz.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Frame length.
pub const FRAME_LEN: usize = 9;
/// Start byte.
pub const START: u8 = 0xFF;
/// Sensor number byte of requests.
pub const SENSOR_ADDRESS: u8 = 0x01;

/// Command bytes.
pub mod cmd {
    /// Read CO2 concentration.
    pub const READ_CO2: u8 = 0x86;
    /// Zero point (400 ppm) calibration.
    pub const ZERO_CALIBRATION: u8 = 0x87;
    /// Span point calibration.
    pub const SPAN_CALIBRATION: u8 = 0x88;
    /// Automatic baseline correction on/off.
    pub const ABC: u8 = 0x79;
    /// Detection range.
    pub const DETECTION_RANGE: u8 = 0x99;
}

/// Time to wait for the 0x86 response: 9 bytes at 9600 Bd take 9.4 ms; the
/// manual gives no processing time, 100 ms is a conservative allowance.
pub const RESPONSE_US: WaitUs = 100_000;
/// Pre-heat time after power-on.
pub const PREHEAT_US: WaitUs = 180_000_000;

const MAX_READ_BYTES: usize = 4 * FRAME_LEN;
const MAX_DRAIN_BYTES: usize = 256;

/// Checksum of a 9-byte frame: two's complement of the sum of bytes 1..=7.
pub fn checksum(frame: &[u8; FRAME_LEN]) -> u8 {
    frame[1..8]
        .iter()
        .fold(0u8, |acc, b| acc.wrapping_add(*b))
        .wrapping_neg()
}

/// Build a request `FF 01 cmd data.. cs`.
pub fn request(command: u8, data: [u8; 5]) -> [u8; FRAME_LEN] {
    let mut f = [
        START,
        SENSOR_ADDRESS,
        command,
        data[0],
        data[1],
        data[2],
        data[3],
        data[4],
        0,
    ];
    f[8] = checksum(&f);
    f
}

/// "Read CO2" request (`FF 01 86 00 00 00 00 00 79`).
pub fn read_co2_command() -> [u8; FRAME_LEN] {
    request(cmd::READ_CO2, [0; 5])
}

/// Zero-point calibration: the sensor must have been in 400 ppm air for
/// ≥ 20 minutes. No response.
pub fn zero_calibration_command() -> [u8; FRAME_LEN] {
    request(cmd::ZERO_CALIBRATION, [0; 5])
}

/// Span calibration at `ppm` (≥ 1000, 2000 suggested) after a zero
/// calibration and ≥ 20 minutes at that concentration. No response.
pub fn span_calibration_command(ppm: u16) -> [u8; FRAME_LEN] {
    let [hi, lo] = ppm.to_be_bytes();
    request(cmd::SPAN_CALIBRATION, [hi, lo, 0, 0, 0])
}

/// Automatic baseline correction on (`0xA0`) or off (`0x00`). No response.
pub fn abc_command(enabled: bool) -> [u8; FRAME_LEN] {
    request(cmd::ABC, [if enabled { 0xA0 } else { 0x00 }, 0, 0, 0, 0])
}

/// Where the 0x99 command carries the range value.
///
/// The MH-Z19B manual v1.0 shows bytes 3–4; MH-Z19C/E manuals and the
/// behaviour observed on B units in the field use bytes 6–7. Sending the
/// wrong layout is ignored by the sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RangeLayout {
    /// Bytes 6–7 (MH-Z19C/E manuals; works on B firmware in the field).
    #[default]
    Bytes6And7,
    /// Bytes 3–4 (MH-Z19B manual v1.0).
    Bytes3And4,
}

/// Detection range (2000, 5000 or 10000 ppm depending on model). No
/// response.
pub fn detection_range_command(range_ppm: u16, layout: RangeLayout) -> [u8; FRAME_LEN] {
    let [hi, lo] = range_ppm.to_be_bytes();
    let data = match layout {
        RangeLayout::Bytes6And7 => [0, 0, 0, hi, lo],
        RangeLayout::Bytes3And4 => [hi, lo, 0, 0, 0],
    };
    request(cmd::DETECTION_RANGE, data)
}

/// A checksum-valid 9-byte frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame(pub [u8; FRAME_LEN]);

impl Frame {
    /// Byte 1: command code of a response (sensor number in requests).
    pub const fn command(&self) -> u8 {
        self.0[1]
    }

    /// Decode a 0x86 response.
    pub fn measurement(&self) -> Option<Measurement> {
        (self.command() == cmd::READ_CO2).then(|| Measurement {
            co2_ppm: u16::from_be_bytes([self.0[2], self.0[3]]),
        })
    }
}

/// Byte-stream parser: resynchronises on `0xFF` and only yields frames with
/// a valid checksum.
#[derive(Debug, Clone, Default)]
pub struct Parser {
    buf: [u8; FRAME_LEN],
    len: usize,
    checksum_error: bool,
}

impl Parser {
    /// Empty parser.
    pub const fn new() -> Self {
        Self {
            buf: [0; FRAME_LEN],
            len: 0,
            checksum_error: false,
        }
    }

    /// Drop any partial frame and the error flag.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// `true` once if a complete frame with a bad checksum was seen.
    pub fn take_checksum_error(&mut self) -> bool {
        core::mem::take(&mut self.checksum_error)
    }

    /// Feed one received byte.
    pub fn feed(&mut self, byte: u8) -> Option<Frame> {
        if self.len == 0 && byte != START {
            return None;
        }
        self.buf[self.len] = byte;
        self.len += 1;
        if self.len < FRAME_LEN {
            return None;
        }
        if checksum(&self.buf) == self.buf[8] {
            self.len = 0;
            return Some(Frame(self.buf));
        }
        self.checksum_error = true;
        match self.buf[1..].iter().position(|b| *b == START) {
            Some(p) => {
                self.buf.copy_within(p + 1.., 0);
                self.len = FRAME_LEN - 1 - p;
            }
            None => self.len = 0,
        }
        None
    }
}

/// One CO2 reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// CO2 in ppm.
    pub co2_ppm: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Co2(u32::from(self.co2_ppm)));
    }
}

fn finish<E>(found: Option<Measurement>, parser: &mut Parser) -> Result<Measurement, Error<E>> {
    match found {
        Some(m) => Ok(m),
        None if parser.take_checksum_error() => Err(Error::Crc),
        None => Err(Error::NotReady),
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic.
        pub const fn new(uart: U) -> Self {
            Self {
                uart,
                parser: Parser::new(),
                range_layout: RangeLayout::Bytes6And7,
            }
        }

        /// Byte layout used by `set_detection_range`.
        pub const fn with_range_layout(mut self, layout: RangeLayout) -> Self {
            self.range_layout = layout;
            self
        }

        /// Give the UART back.
        pub fn release(self) -> U {
            self.uart
        }
    };
}

/// Blocking driver.
pub struct Mhz19<U> {
    uart: U,
    parser: Parser,
    range_layout: RangeLayout,
}

impl<U> Mhz19<U> {
    constructors!();
}

impl<U> Mhz19<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    fn send(&mut self, frame: &[u8; FRAME_LEN]) -> Result<(), Error<U::Error>> {
        self.uart.write_all(frame).map_err(Error::bus)?;
        self.uart.flush().map_err(Error::bus)
    }

    fn drain(&mut self) -> Result<(), Error<U::Error>> {
        let mut buf = [0u8; 16];
        let mut left = MAX_DRAIN_BYTES;
        while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
            let n = self.uart.read(&mut buf).map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            left = left.saturating_sub(n);
        }
        self.parser.reset();
        Ok(())
    }

    /// Feed all available bytes (bounded) to the parser and return the
    /// newest frame, if any.
    pub fn poll_frame(&mut self) -> Result<Option<Frame>, Error<U::Error>> {
        let mut latest = None;
        let mut buf = [0u8; FRAME_LEN];
        let mut left = MAX_READ_BYTES;
        while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
            let want = left.min(buf.len());
            let n = self.uart.read(&mut buf[..want]).map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            left -= n;
            for b in &buf[..n] {
                if let Some(f) = self.parser.feed(*b) {
                    latest = Some(f);
                }
            }
        }
        Ok(latest)
    }

    /// Enable or disable automatic baseline correction.
    pub fn set_abc(&mut self, enabled: bool) -> Result<(), Error<U::Error>> {
        self.send(&abc_command(enabled))
    }

    /// Zero-point calibration (after ≥ 20 min in 400 ppm air).
    pub fn calibrate_zero(&mut self) -> Result<(), Error<U::Error>> {
        self.send(&zero_calibration_command())
    }

    /// Span calibration at `ppm`.
    pub fn calibrate_span(&mut self, ppm: u16) -> Result<(), Error<U::Error>> {
        self.send(&span_calibration_command(ppm))
    }

    /// Set the detection range in ppm (see [`RangeLayout`]).
    pub fn set_detection_range(&mut self, range_ppm: u16) -> Result<(), Error<U::Error>> {
        self.send(&detection_range_command(range_ppm, self.range_layout))
    }
}

impl<U> zigbee_sensor::blocking::Sensor for Mhz19<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type BusError = U::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        self.drain()?;
        self.send(&read_co2_command())?;
        Ok(RESPONSE_US)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
        let found = self.poll_frame()?.and_then(|f| f.measurement());
        finish(found, &mut self.parser)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        FRAME_LEN, Frame, MAX_DRAIN_BYTES, MAX_READ_BYTES, Measurement, Parser, RESPONSE_US,
        RangeLayout, abc_command, detection_range_command, finish, read_co2_command,
        span_calibration_command, zero_calibration_command,
    };
    use embedded_io_async::{Read, ReadReady, Write};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Mhz19`].
    pub struct Mhz19<U> {
        uart: U,
        parser: Parser,
        range_layout: RangeLayout,
    }

    impl<U> Mhz19<U> {
        constructors!();
    }

    impl<U: Read + ReadReady + Write> Mhz19<U> {
        async fn send(&mut self, frame: &[u8; FRAME_LEN]) -> Result<(), Error<U::Error>> {
            self.uart.write_all(frame).await.map_err(Error::bus)?;
            self.uart.flush().await.map_err(Error::bus)
        }

        async fn drain(&mut self) -> Result<(), Error<U::Error>> {
            let mut buf = [0u8; 16];
            let mut left = MAX_DRAIN_BYTES;
            while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
                let n = self.uart.read(&mut buf).await.map_err(Error::bus)?;
                if n == 0 {
                    break;
                }
                left = left.saturating_sub(n);
            }
            self.parser.reset();
            Ok(())
        }

        /// See [`super::Mhz19::poll_frame`].
        pub async fn poll_frame(&mut self) -> Result<Option<Frame>, Error<U::Error>> {
            let mut latest = None;
            let mut buf = [0u8; FRAME_LEN];
            let mut left = MAX_READ_BYTES;
            while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
                let want = left.min(buf.len());
                let n = self.uart.read(&mut buf[..want]).await.map_err(Error::bus)?;
                if n == 0 {
                    break;
                }
                left -= n;
                for b in &buf[..n] {
                    if let Some(f) = self.parser.feed(*b) {
                        latest = Some(f);
                    }
                }
            }
            Ok(latest)
        }

        /// See [`super::Mhz19::set_abc`].
        pub async fn set_abc(&mut self, enabled: bool) -> Result<(), Error<U::Error>> {
            self.send(&abc_command(enabled)).await
        }

        /// See [`super::Mhz19::calibrate_zero`].
        pub async fn calibrate_zero(&mut self) -> Result<(), Error<U::Error>> {
            self.send(&zero_calibration_command()).await
        }

        /// See [`super::Mhz19::calibrate_span`].
        pub async fn calibrate_span(&mut self, ppm: u16) -> Result<(), Error<U::Error>> {
            self.send(&span_calibration_command(ppm)).await
        }

        /// See [`super::Mhz19::set_detection_range`].
        pub async fn set_detection_range(&mut self, range_ppm: u16) -> Result<(), Error<U::Error>> {
            self.send(&detection_range_command(range_ppm, self.range_layout))
                .await
        }
    }

    impl<U: Read + ReadReady + Write> zigbee_sensor::asynch::Sensor for Mhz19<U> {
        type BusError = U::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
            self.drain().await?;
            self.send(&read_co2_command()).await?;
            Ok(RESPONSE_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
            let found = self.poll_frame().await?.and_then(|f| f.measurement());
            finish(found, &mut self.parser)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::collections::VecDeque;
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    /// In-memory UART: `rx` is what the sensor sends, `tx` what we wrote.
    #[derive(Default)]
    struct FakeUart {
        rx: VecDeque<u8>,
        tx: Vec<u8>,
        on_write: Vec<u8>,
    }

    impl embedded_io::ErrorType for FakeUart {
        type Error = core::convert::Infallible;
    }
    impl embedded_io::ReadReady for FakeUart {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            Ok(!self.rx.is_empty())
        }
    }
    impl embedded_io::Read for FakeUart {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.rx.len());
            for b in &mut buf[..n] {
                *b = self.rx.pop_front().unwrap();
            }
            Ok(n)
        }
    }
    impl embedded_io::Write for FakeUart {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.tx.extend_from_slice(buf);
            let reply = core::mem::take(&mut self.on_write);
            self.rx.extend(reply);
            Ok(buf.len())
        }
        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }
    impl embedded_io_async::Read for FakeUart {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            embedded_io::Read::read(self, buf)
        }
    }
    impl embedded_io_async::Write for FakeUart {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            embedded_io::Write::write(self, buf)
        }
    }

    fn response(ppm: u16) -> [u8; 9] {
        let [hi, lo] = ppm.to_be_bytes();
        let mut f = [0xFF, 0x86, hi, lo, 0x41, 0x00, 0x00, 0x00, 0];
        f[8] = checksum(&f);
        f
    }

    #[test]
    fn commands_match_manual() {
        assert_eq!(
            read_co2_command(),
            [0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79]
        );
        assert_eq!(
            zero_calibration_command(),
            [0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78]
        );
        assert_eq!(
            span_calibration_command(2000),
            [0xFF, 0x01, 0x88, 0x07, 0xD0, 0x00, 0x00, 0x00, 0xA0]
        );
        assert_eq!(
            abc_command(true),
            [0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6]
        );
        assert_eq!(
            abc_command(false),
            [0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86]
        );
        assert_eq!(
            detection_range_command(5000, RangeLayout::Bytes6And7),
            [0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x13, 0x88, 0xCB]
        );
        assert_eq!(
            detection_range_command(2000, RangeLayout::Bytes3And4),
            [0xFF, 0x01, 0x99, 0x07, 0xD0, 0x00, 0x00, 0x00, 0x8F]
        );
    }

    #[test]
    fn parser_resyncs_and_rejects_bad_checksum() {
        let mut p = Parser::new();
        let good = response(1234);
        let mut bad = good;
        bad[8] ^= 1;
        // Garbage, a corrupted frame, then a good one.
        let mut stream = vec![0x00, 0x12];
        stream.extend_from_slice(&bad);
        stream.extend_from_slice(&good);
        let frames: Vec<Frame> = stream.iter().filter_map(|b| p.feed(*b)).collect();
        assert_eq!(frames, vec![Frame(good)]);
        assert!(p.take_checksum_error());
        assert!(!p.take_checksum_error());
        assert_eq!(frames[0].measurement(), Some(Measurement { co2_ppm: 1234 }));
    }

    #[test]
    fn parser_recovers_from_ff_inside_bad_frame() {
        let mut p = Parser::new();
        let good = response(400);
        // A truncated frame start (FF 86 01) followed directly by a good frame.
        let mut stream = vec![0xFF, 0x86, 0x01];
        stream.extend_from_slice(&good);
        let frames: Vec<Frame> = stream.iter().filter_map(|b| p.feed(*b)).collect();
        assert_eq!(frames, vec![Frame(good)]);
    }

    #[test]
    fn query_cycle() {
        let mut uart = FakeUart::default();
        uart.rx.extend([0x55, 0xAA]); // stale bytes are drained
        uart.on_write = response(400).to_vec();
        let mut mhz = Mhz19::new(&mut uart);
        assert_eq!(mhz.start_measurement().unwrap(), RESPONSE_US);
        let m = mhz.read_measurement().unwrap();
        assert_eq!(m.co2_ppm, 400);
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(seen, vec![Quantity::Co2(400)]);
        assert_eq!(mhz.read_measurement(), Err(Error::NotReady));
        mhz.set_abc(false).unwrap();
        assert_eq!(&uart.tx[..9], &read_co2_command());
        assert_eq!(&uart.tx[9..], &abc_command(false));
    }

    #[test]
    fn corrupt_response_is_crc_error() {
        let mut bad = response(800);
        bad[3] ^= 0x10;
        let mut uart = FakeUart::default();
        uart.rx.extend(bad);
        let mut mhz = Mhz19::new(&mut uart);
        assert_eq!(mhz.read_measurement(), Err(Error::Crc));
        assert_eq!(mhz.read_measurement(), Err(Error::NotReady));
    }

    #[test]
    fn split_response_across_reads() {
        let mut uart = FakeUart::default();
        let r = response(800);
        let mut mhz = Mhz19::new(&mut uart);
        mhz.start_measurement().unwrap();
        mhz.uart.rx.extend(&r[..4]);
        assert_eq!(mhz.read_measurement(), Err(Error::NotReady));
        mhz.uart.rx.extend(&r[4..]);
        assert_eq!(mhz.read_measurement().unwrap().co2_ppm, 800);
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let mut uart = FakeUart {
            on_write: response(5000).to_vec(),
            ..FakeUart::default()
        };
        let mut mhz = asynch::Mhz19::new(&mut uart);
        let mut delay = RecordingDelay(0);
        let m = block_on(mhz.measure(&mut delay)).unwrap();
        assert_eq!(m.co2_ppm, 5000);
        assert_eq!(delay.0, RESPONSE_US);
        block_on(mhz.set_detection_range(5000)).unwrap();
        assert_eq!(
            &uart.tx[9..],
            &detection_range_command(5000, RangeLayout::Bytes6And7)
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
