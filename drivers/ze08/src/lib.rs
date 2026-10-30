//! Winsen ZE08-CH2O / ZE07-CH2O electrochemical formaldehyde modules over
//! UART.
//!
//! UART 9600 8N1 (3.3 V levels), 9-byte frames `FF b1..b7 <checksum>` with
//! checksum `(!(b1 + .. + b7)) + 1` (same framing as Winsen MH-Z19).
//!
//! Two communication types:
//!
//! - **Active upload** (power-on default): the module sends
//!   `FF 17 04 00 CH CL FH FL CS` (CH2O and full range in ppb) every second.
//!   [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//!   returns 0 and
//!   [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//!   returns the newest uploaded frame or [`Error::NotReady`].
//! - **Q&A**: after [`Ze08::set_mode`]`(Mode::QuestionAnswer)`,
//!   `start_measurement` discards stale input, sends `FF 01 86 ..` and
//!   returns [`RESPONSE_US`]; the reply carries µg/m³ and ppb.
//!
//! The driver needs [`embedded_io::ReadReady`] so it never blocks. The
//! modules have no sleep command.
//!
//! References: Winsen "Electrochemical CH2O Detection Module ZE08-CH2O
//! User's Manual" v1.0 (2014-12-12); ZE07-CH2O User's Manual (identical
//! protocol tables).
//!
//! ```ignore
//! let mut ze = Ze08::new(uart);
//! ze.set_mode(Mode::QuestionAnswer)?;
//! let wait_us = ze.start_measurement()?;
//! // ... wait_us later:
//! let m = ze.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Frame length.
pub const FRAME_LEN: usize = 9;
/// Start byte.
pub const START: u8 = 0xFF;
/// Gas name byte of active-upload frames (CH2O).
pub const GAS_CH2O: u8 = 0x17;
/// Unit byte of active-upload frames (ppb).
pub const UNIT_PPB: u8 = 0x04;

/// Command bytes.
pub mod cmd {
    /// Switch communication type (data byte 0x40 active, 0x41 Q&A).
    pub const SWITCH_MODE: u8 = 0x78;
    /// Read concentration (Q&A).
    pub const READ: u8 = 0x86;
}

/// Time to wait for the Q&A response: 9 bytes at 9600 Bd take 9.4 ms; the
/// manual gives no processing time, 100 ms is a conservative allowance.
pub const RESPONSE_US: WaitUs = 100_000;
/// Active-upload interval.
pub const UPLOAD_INTERVAL_US: WaitUs = 1_000_000;
/// Warm-up time after power-on (manual: ≤ 3 min; ≥ 5 min on first use).
pub const WARM_UP_US: WaitUs = 180_000_000;

const MAX_READ_BYTES: usize = 256;
const MAX_DRAIN_BYTES: usize = 256;

/// Checksum of a 9-byte frame: two's complement of the sum of bytes 1..=7.
pub fn checksum(frame: &[u8; FRAME_LEN]) -> u8 {
    frame[1..8]
        .iter()
        .fold(0u8, |acc, b| acc.wrapping_add(*b))
        .wrapping_neg()
}

fn request(command: u8, data: u8) -> [u8; FRAME_LEN] {
    let mut f = [START, 0x01, command, data, 0, 0, 0, 0, 0];
    f[8] = checksum(&f);
    f
}

/// Communication type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// The module uploads a reading every second (power-on default).
    #[default]
    Active,
    /// The module only answers read requests.
    QuestionAnswer,
}

/// Switch command: `FF 01 78 40 .. 47` (active) or `FF 01 78 41 .. 46`
/// (Q&A). No response.
pub fn switch_mode_command(mode: Mode) -> [u8; FRAME_LEN] {
    let data = match mode {
        Mode::Active => 0x40,
        Mode::QuestionAnswer => 0x41,
    };
    request(cmd::SWITCH_MODE, data)
}

/// Q&A read request `FF 01 86 00 00 00 00 00 79`.
pub fn read_command() -> [u8; FRAME_LEN] {
    request(cmd::READ, 0)
}

/// Formaldehyde µg/m³ → ppb at 25 °C, 1013 hPa
/// (ppb = µg/m³ × 24.45 / 30.03), rounded.
pub fn ug_m3_to_ppb(ug_m3: u32) -> u32 {
    (ug_m3 * 2445 + 1501) / 3003
}

/// Formaldehyde ppb → µg/m³ at 25 °C, 1013 hPa, rounded.
pub fn ppb_to_ug_m3(ppb: u32) -> u32 {
    (ppb * 3003 + 1222) / 2445
}

/// A checksum-valid 9-byte frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame(pub [u8; FRAME_LEN]);

impl Frame {
    fn word(&self, i: usize) -> u16 {
        u16::from_be_bytes([self.0[i], self.0[i + 1]])
    }

    /// Decode an active-upload frame (`FF 17 04 ..`) or a Q&A response
    /// (`FF 86 ..`).
    pub fn measurement(&self) -> Option<Measurement> {
        match self.0[1] {
            GAS_CH2O if self.0[2] == UNIT_PPB => Some(Measurement {
                ppb: self.word(4),
                ug_m3: None,
                full_range_ppb: Some(self.word(6)),
            }),
            cmd::READ => Some(Measurement {
                ppb: self.word(6),
                ug_m3: Some(self.word(2)),
                full_range_ppb: None,
            }),
            _ => None,
        }
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

/// One formaldehyde reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// CH2O in ppb.
    pub ppb: u16,
    /// CH2O in µg/m³ (Q&A responses only).
    pub ug_m3: Option<u16>,
    /// Full range in ppb (active-upload frames only; 5000 on ZE08).
    pub full_range_ppb: Option<u16>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Formaldehyde(u32::from(self.ppb)));
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
        /// Create a driver assuming the power-on [`Mode::Active`]; no bus
        /// traffic.
        pub const fn new(uart: U) -> Self {
            Self {
                uart,
                parser: Parser::new(),
                mode: Mode::Active,
            }
        }

        /// Assume the module is already in `mode` (e.g. after an MCU reset
        /// without a sensor power cycle); no bus traffic.
        pub const fn with_mode(mut self, mode: Mode) -> Self {
            self.mode = mode;
            self
        }

        /// Communication type the driver assumes.
        pub const fn mode(&self) -> Mode {
            self.mode
        }

        /// Give the UART back.
        pub fn release(self) -> U {
            self.uart
        }
    };
}

/// Blocking driver.
pub struct Ze08<U> {
    uart: U,
    parser: Parser,
    mode: Mode,
}

impl<U> Ze08<U> {
    constructors!();
}

impl<U> Ze08<U>
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

    /// Switch the communication type and discard pending input.
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<U::Error>> {
        self.send(&switch_mode_command(mode))?;
        self.drain()?;
        self.mode = mode;
        Ok(())
    }
}

impl<U> zigbee_sensor::blocking::Sensor for Ze08<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type BusError = U::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        match self.mode {
            Mode::Active => Ok(0),
            Mode::QuestionAnswer => {
                self.drain()?;
                self.send(&read_command())?;
                Ok(RESPONSE_US)
            }
        }
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
        let found = self.poll_frame()?.and_then(|f| f.measurement());
        finish(found, &mut self.parser)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        FRAME_LEN, Frame, MAX_DRAIN_BYTES, MAX_READ_BYTES, Measurement, Mode, Parser, RESPONSE_US,
        finish, read_command, switch_mode_command,
    };
    use embedded_io_async::{Read, ReadReady, Write};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Ze08`].
    pub struct Ze08<U> {
        uart: U,
        parser: Parser,
        mode: Mode,
    }

    impl<U> Ze08<U> {
        constructors!();
    }

    impl<U: Read + ReadReady + Write> Ze08<U> {
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

        /// See [`super::Ze08::poll_frame`].
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

        /// See [`super::Ze08::set_mode`].
        pub async fn set_mode(&mut self, mode: Mode) -> Result<(), Error<U::Error>> {
            self.send(&switch_mode_command(mode)).await?;
            self.drain().await?;
            self.mode = mode;
            Ok(())
        }
    }

    impl<U: Read + ReadReady + Write> zigbee_sensor::asynch::Sensor for Ze08<U> {
        type BusError = U::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
            match self.mode {
                Mode::Active => Ok(0),
                Mode::QuestionAnswer => {
                    self.drain().await?;
                    self.send(&read_command()).await?;
                    Ok(RESPONSE_US)
                }
            }
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

    /// Manual Table 4 example (37 ppb, full range 5000 ppb).
    const UPLOAD: [u8; 9] = [0xFF, 0x17, 0x04, 0x00, 0x00, 0x25, 0x13, 0x88, 0x25];
    /// Manual Table 8 example (42 µg/m³, 32 ppb).
    const ANSWER: [u8; 9] = [0xFF, 0x86, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x20, 0x30];

    fn upload(ppb: u16) -> [u8; 9] {
        let [hi, lo] = ppb.to_be_bytes();
        let mut f = [0xFF, 0x17, 0x04, 0x00, hi, lo, 0x13, 0x88, 0];
        f[8] = checksum(&f);
        f
    }

    #[test]
    fn commands_and_frames_match_manual() {
        assert_eq!(
            switch_mode_command(Mode::QuestionAnswer),
            [0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46]
        );
        assert_eq!(
            switch_mode_command(Mode::Active),
            [0xFF, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47]
        );
        assert_eq!(
            read_command(),
            [0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79]
        );
        let mut p = Parser::new();
        let frames: Vec<Frame> = UPLOAD
            .iter()
            .chain(&ANSWER)
            .filter_map(|b| p.feed(*b))
            .collect();
        assert_eq!(
            frames[0].measurement(),
            Some(Measurement {
                ppb: 37,
                ug_m3: None,
                full_range_ppb: Some(5000)
            })
        );
        assert_eq!(
            frames[1].measurement(),
            Some(Measurement {
                ppb: 32,
                ug_m3: Some(42),
                full_range_ppb: None
            })
        );
    }

    #[test]
    fn parser_resyncs_and_flags_bad_checksum() {
        let mut p = Parser::new();
        let mut bad = UPLOAD;
        bad[5] ^= 1;
        let mut stream = vec![0x00, 0xFF, 0x17];
        stream.extend_from_slice(&bad);
        stream.extend_from_slice(&UPLOAD);
        let frames: Vec<Frame> = stream.iter().filter_map(|b| p.feed(*b)).collect();
        assert_eq!(frames, vec![Frame(UPLOAD)]);
        assert!(p.take_checksum_error());
    }

    #[test]
    fn conversions() {
        assert_eq!(ug_m3_to_ppb(1228), 1000);
        assert_eq!(ppb_to_ug_m3(1000), 1228);
        assert_eq!(ug_m3_to_ppb(0), 0);
    }

    #[test]
    fn active_mode_returns_newest_upload() {
        let mut uart = FakeUart::default();
        let mut ze = Ze08::new(&mut uart);
        assert_eq!(ze.start_measurement().unwrap(), 0);
        assert_eq!(ze.read_measurement(), Err(Error::NotReady));
        ze.uart.rx.extend(upload(10));
        ze.uart.rx.extend(upload(20));
        ze.uart.rx.extend(&upload(30)[..5]);
        let m = ze.read_measurement().unwrap();
        assert_eq!(m.ppb, 20);
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(seen, vec![Quantity::Formaldehyde(20)]);
        ze.uart.rx.extend(&upload(30)[5..]);
        assert_eq!(ze.read_measurement().unwrap().ppb, 30);
        assert!(uart.tx.is_empty());
    }

    #[test]
    fn question_answer_cycle() {
        let mut uart = FakeUart::default();
        uart.rx.extend(UPLOAD); // pending upload is discarded by set_mode
        let mut ze = Ze08::new(&mut uart);
        ze.set_mode(Mode::QuestionAnswer).unwrap();
        assert_eq!(ze.mode(), Mode::QuestionAnswer);
        assert_eq!(ze.read_measurement(), Err(Error::NotReady));
        ze.uart.on_write = ANSWER.to_vec();
        assert_eq!(ze.start_measurement().unwrap(), RESPONSE_US);
        assert_eq!(ze.read_measurement().unwrap().ug_m3, Some(42));
        assert_eq!(&uart.tx[..9], &switch_mode_command(Mode::QuestionAnswer));
        assert_eq!(&uart.tx[9..], &read_command());
    }

    #[test]
    fn corrupt_response_is_crc_error() {
        let mut bad = ANSWER;
        bad[7] ^= 0x40;
        let mut uart = FakeUart::default();
        uart.rx.extend(bad);
        let mut ze = Ze08::new(&mut uart);
        assert_eq!(ze.read_measurement(), Err(Error::Crc));
        assert_eq!(ze.read_measurement(), Err(Error::NotReady));
    }

    #[test]
    fn async_question_answer() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let mut uart = FakeUart::default();
        block_on(asynch::Ze08::new(&mut uart).set_mode(Mode::QuestionAnswer)).unwrap();
        uart.on_write = ANSWER.to_vec();
        let mut ze = asynch::Ze08::new(&mut uart).with_mode(Mode::QuestionAnswer);
        let mut delay = RecordingDelay(0);
        let m = block_on(ze.measure(&mut delay)).unwrap();
        assert_eq!(m.ppb, 32);
        assert_eq!(delay.0, RESPONSE_US);
        assert_eq!(&uart.tx[9..], &read_command());
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
