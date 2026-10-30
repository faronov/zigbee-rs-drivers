//! Plantower PMS5003 / PMS7003 / PMSA003 particulate matter sensors over
//! UART and PMSA003I over I2C.
//!
//! All parts produce the same 32-byte frame: `42 4D 00 1C`, 13 big-endian
//! data words and a 16-bit sum of the first 30 bytes. Data 1–3 are
//! PM1.0/2.5/10 for CF=1 "standard particle" (factory use), data 4–6 the
//! same under atmospheric environment, data 7–12 particle counts per 0.1 L
//! (> 0.3, 0.5, 1.0, 2.5, 5.0, 10 µm), data 13 reserved (PMSA003I: version
//! and error code). [`Readings`] report the **atmospheric** values in
//! 0.1 µg/m³.
//!
//! UART (9600 8N1, 3.3 V levels) after power-up is in **active** mode: a
//! frame every 2.3 s when stable, every 200–800 ms when the concentration
//! changes quickly.
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! returns 0 and
//! [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//! returns the newest frame or [`Error::NotReady`]. In **passive** mode
//! ([`Pmsx003::set_mode`]) `start_measurement` sends the read command
//! (0xE2) and returns [`RESPONSE_US`]. Commands are 7 bytes
//! `42 4D cmd DH DL sumH sumL`. Sleep/wake (0xE4) implement
//! [`LowPower`](zigbee_sensor::blocking::LowPower); the SET pin (low =
//! sleep) does the same in hardware.
//!
//! PMSA003I: I2C address 0x12, ≤ 100 kHz; the frame is read from register
//! 0x00. It is free-running, so `start_measurement` returns 0.
//!
//! References: Plantower "PMS5003 series data manual" V2.3 (2016-06-01);
//! "PMSA003I series data manual" V2.6 (2018-02-26).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Frame length (header, length, 13 data words, checksum).
pub const FRAME_LEN: usize = 32;
/// First start character.
pub const START1: u8 = 0x42;
/// Second start character.
pub const START2: u8 = 0x4D;
/// Value of the frame length field (2 × 13 + 2).
pub const FRAME_LENGTH_FIELD: u16 = 28;
/// PMSA003I I2C address.
pub const I2C_ADDRESS: u8 = 0x12;

/// Command bytes.
pub mod cmd {
    /// Read in passive mode.
    pub const READ: u8 = 0xE2;
    /// Change mode (0 passive, 1 active).
    pub const CHANGE_MODE: u8 = 0xE1;
    /// Sleep set (0 sleep, 1 wake up).
    pub const SLEEP: u8 = 0xE4;
}

/// Allowance for a passive-mode answer: 32 bytes at 9600 Bd take 33 ms; the
/// manual gives no processing time, 100 ms is a conservative allowance.
pub const RESPONSE_US: WaitUs = 100_000;
/// Active-mode frame interval in stable mode.
pub const ACTIVE_INTERVAL_US: WaitUs = 2_300_000;
/// Data are stable ≥ 30 s after wake-up (fan spin-up).
pub const WAKE_STABILIZE_US: WaitUs = 30_000_000;

const MAX_READ_BYTES: usize = 256;
const MAX_DRAIN_BYTES: usize = 256;

/// Build a command `42 4D cmd DH DL sumH sumL`.
pub fn command(code: u8, data: u16) -> [u8; 7] {
    let [dh, dl] = data.to_be_bytes();
    let mut f = [START1, START2, code, dh, dl, 0, 0];
    let sum = f[..5].iter().map(|b| u16::from(*b)).sum::<u16>();
    f[5..].copy_from_slice(&sum.to_be_bytes());
    f
}

/// UART output mode (not retained: active after power-up).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// Frames are sent automatically.
    #[default]
    Active,
    /// Frames are sent only in answer to [`read_command`].
    Passive,
}

/// Passive-mode read request `42 4D E2 00 00 01 71`.
pub fn read_command() -> [u8; 7] {
    command(cmd::READ, 0)
}

/// Change mode: `42 4D E1 00 00 01 70` (passive), `.. 01 01 71` (active).
pub fn mode_command(mode: Mode) -> [u8; 7] {
    command(cmd::CHANGE_MODE, u16::from(mode == Mode::Active))
}

/// Sleep (`42 4D E4 00 00 01 73`) or wake up (`.. 00 01 01 74`).
pub fn sleep_command(sleep: bool) -> [u8; 7] {
    command(cmd::SLEEP, u16::from(!sleep))
}

/// Mass concentrations in µg/m³.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Pm {
    /// PM1.0.
    pub pm1_0: u16,
    /// PM2.5.
    pub pm2_5: u16,
    /// PM10.
    pub pm10: u16,
}

/// Particle counts per 0.1 L with diameter beyond the given size.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ParticleCounts {
    /// > 0.3 µm.
    pub um0_3: u16,
    /// > 0.5 µm.
    pub um0_5: u16,
    /// > 1.0 µm.
    pub um1_0: u16,
    /// > 2.5 µm.
    pub um2_5: u16,
    /// > 5.0 µm.
    pub um5_0: u16,
    /// > 10 µm.
    pub um10: u16,
}

/// One decoded frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// CF=1, standard particle (for factory environments).
    pub standard: Pm,
    /// Under atmospheric environment (reported as [`Quantity`]).
    pub atmospheric: Pm,
    /// Particle counts per 0.1 L.
    pub counts: ParticleCounts,
    /// Data 13: reserved on PMS5003; the PMSA003I manual documents the
    /// version in the high byte and an error code in the low byte.
    pub reserved: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        let a = self.atmospheric;
        f(Quantity::Pm1(u32::from(a.pm1_0) * 10));
        f(Quantity::Pm25(u32::from(a.pm2_5) * 10));
        f(Quantity::Pm10(u32::from(a.pm10) * 10));
    }
}

/// Why a 32-byte frame was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Wrong start characters or length field.
    Header,
    /// Checksum mismatch.
    Checksum,
}

impl<E> From<FrameError> for Error<E> {
    fn from(e: FrameError) -> Self {
        match e {
            FrameError::Header => Error::InvalidData,
            FrameError::Checksum => Error::Crc,
        }
    }
}

/// Check header, length and checksum of a frame and decode it.
pub fn decode_frame(frame: &[u8; FRAME_LEN]) -> Result<Measurement, FrameError> {
    let word = |i: usize| u16::from_be_bytes([frame[i], frame[i + 1]]);
    if frame[0] != START1 || frame[1] != START2 || word(2) != FRAME_LENGTH_FIELD {
        return Err(FrameError::Header);
    }
    let sum = frame[..30].iter().map(|b| u16::from(*b)).sum::<u16>();
    if sum != word(30) {
        return Err(FrameError::Checksum);
    }
    let d = |n: usize| word(2 + 2 * n);
    Ok(Measurement {
        standard: Pm {
            pm1_0: d(1),
            pm2_5: d(2),
            pm10: d(3),
        },
        atmospheric: Pm {
            pm1_0: d(4),
            pm2_5: d(5),
            pm10: d(6),
        },
        counts: ParticleCounts {
            um0_3: d(7),
            um0_5: d(8),
            um1_0: d(9),
            um2_5: d(10),
            um5_0: d(11),
            um10: d(12),
        },
        reserved: d(13),
    })
}

/// UART byte-stream parser: resynchronises on `42 4D`, ignores frames with
/// another length field (e.g. command acknowledgements) and only yields
/// checksum-valid data frames.
#[derive(Debug, Clone)]
pub struct Parser {
    buf: [u8; FRAME_LEN],
    len: usize,
    checksum_error: bool,
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
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

    fn prefix_ok(&self) -> bool {
        (self.len < 2 || self.buf[1] == START2)
            && (self.len < 4
                || u16::from_be_bytes([self.buf[2], self.buf[3]]) == FRAME_LENGTH_FIELD)
    }

    /// Drop bytes up to the next start character that begins a plausible
    /// header.
    fn resync(&mut self) {
        loop {
            match self.buf[1..self.len].iter().position(|b| *b == START1) {
                Some(p) => {
                    self.buf.copy_within(p + 1..self.len, 0);
                    self.len -= p + 1;
                }
                None => self.len = 0,
            }
            if self.prefix_ok() {
                return;
            }
        }
    }

    /// Feed one received byte.
    pub fn feed(&mut self, byte: u8) -> Option<Measurement> {
        if self.len == 0 && byte != START1 {
            return None;
        }
        self.buf[self.len] = byte;
        self.len += 1;
        if !self.prefix_ok() {
            self.resync();
            return None;
        }
        if self.len < FRAME_LEN {
            return None;
        }
        match decode_frame(&self.buf) {
            Ok(m) => {
                self.len = 0;
                Some(m)
            }
            Err(_) => {
                self.checksum_error = true;
                self.resync();
                None
            }
        }
    }
}

fn finish<E>(found: Option<Measurement>, parser: &mut Parser) -> Result<Measurement, Error<E>> {
    match found {
        Some(m) => Ok(m),
        None if parser.take_checksum_error() => Err(Error::Crc),
        None => Err(Error::NotReady),
    }
}

macro_rules! uart_constructors {
    () => {
        /// Create a driver assuming the power-up [`Mode::Active`]; no bus
        /// traffic.
        pub const fn new(uart: U) -> Self {
            Self {
                uart,
                parser: Parser::new(),
                mode: Mode::Active,
            }
        }

        /// Assume the sensor is already in `mode`; no bus traffic.
        pub const fn with_mode(mut self, mode: Mode) -> Self {
            self.mode = mode;
            self
        }

        /// Mode the driver assumes.
        pub const fn mode(&self) -> Mode {
            self.mode
        }

        /// Give the UART back.
        pub fn release(self) -> U {
            self.uart
        }
    };
}

macro_rules! i2c_constructors {
    () => {
        /// Create a PMSA003I driver at [`I2C_ADDRESS`]; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self { i2c }
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking UART driver (PMS5003 / PMS7003 / PMSA003).
pub struct Pmsx003<U> {
    uart: U,
    parser: Parser,
    mode: Mode,
}

impl<U> Pmsx003<U> {
    uart_constructors!();
}

impl<U> Pmsx003<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    fn send(&mut self, frame: &[u8; 7]) -> Result<(), Error<U::Error>> {
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
    /// newest valid frame, if any.
    pub fn poll_frame(&mut self) -> Result<Option<Measurement>, Error<U::Error>> {
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
                if let Some(m) = self.parser.feed(*b) {
                    latest = Some(m);
                }
            }
        }
        Ok(latest)
    }

    /// Switch between active and passive output; discards pending input.
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<U::Error>> {
        self.send(&mode_command(mode))?;
        self.drain()?;
        self.mode = mode;
        Ok(())
    }
}

impl<U> zigbee_sensor::blocking::Sensor for Pmsx003<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type BusError = U::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        match self.mode {
            Mode::Active => Ok(0),
            Mode::Passive => {
                self.drain()?;
                self.send(&read_command())?;
                Ok(RESPONSE_US)
            }
        }
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
        let found = self.poll_frame()?;
        finish(found, &mut self.parser)
    }
}

impl<U> zigbee_sensor::blocking::LowPower for Pmsx003<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type BusError = U::Error;

    fn sleep(&mut self) -> Result<(), Error<U::Error>> {
        self.send(&sleep_command(true))
    }

    fn wake(&mut self) -> Result<WaitUs, Error<U::Error>> {
        self.send(&sleep_command(false))?;
        Ok(WAKE_STABILIZE_US)
    }
}

/// Blocking PMSA003I (I2C) driver.
pub struct Pmsa003i<I2C> {
    i2c: I2C,
}

impl<I2C> Pmsa003i<I2C> {
    i2c_constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Pmsa003i<I2C> {
    /// Read the raw 32-byte frame from register 0x00.
    pub fn read_frame(&mut self) -> Result<[u8; FRAME_LEN], Error<I2C::Error>> {
        let mut f = [0u8; FRAME_LEN];
        self.i2c
            .write_read(I2C_ADDRESS, &[0x00], &mut f)
            .map_err(Error::bus)?;
        Ok(f)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Pmsa003i<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        Ok(0)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        Ok(decode_frame(&self.read_frame()?)?)
    }
}

/// Async drivers.
pub mod asynch {
    use super::{
        FRAME_LEN, I2C_ADDRESS, MAX_DRAIN_BYTES, MAX_READ_BYTES, Measurement, Mode, Parser,
        RESPONSE_US, WAKE_STABILIZE_US, decode_frame, finish, mode_command, read_command,
        sleep_command,
    };
    use embedded_io_async::{Read, ReadReady, Write};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Pmsx003`].
    pub struct Pmsx003<U> {
        uart: U,
        parser: Parser,
        mode: Mode,
    }

    impl<U> Pmsx003<U> {
        uart_constructors!();
    }

    impl<U: Read + ReadReady + Write> Pmsx003<U> {
        async fn send(&mut self, frame: &[u8; 7]) -> Result<(), Error<U::Error>> {
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

        /// See [`super::Pmsx003::poll_frame`].
        pub async fn poll_frame(&mut self) -> Result<Option<Measurement>, Error<U::Error>> {
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
                    if let Some(m) = self.parser.feed(*b) {
                        latest = Some(m);
                    }
                }
            }
            Ok(latest)
        }

        /// See [`super::Pmsx003::set_mode`].
        pub async fn set_mode(&mut self, mode: Mode) -> Result<(), Error<U::Error>> {
            self.send(&mode_command(mode)).await?;
            self.drain().await?;
            self.mode = mode;
            Ok(())
        }
    }

    impl<U: Read + ReadReady + Write> zigbee_sensor::asynch::Sensor for Pmsx003<U> {
        type BusError = U::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
            match self.mode {
                Mode::Active => Ok(0),
                Mode::Passive => {
                    self.drain().await?;
                    self.send(&read_command()).await?;
                    Ok(RESPONSE_US)
                }
            }
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
            let found = self.poll_frame().await?;
            finish(found, &mut self.parser)
        }
    }

    impl<U: Read + ReadReady + Write> zigbee_sensor::asynch::LowPower for Pmsx003<U> {
        type BusError = U::Error;

        async fn sleep(&mut self) -> Result<(), Error<U::Error>> {
            self.send(&sleep_command(true)).await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<U::Error>> {
            self.send(&sleep_command(false)).await?;
            Ok(WAKE_STABILIZE_US)
        }
    }

    /// Async twin of [`super::Pmsa003i`].
    pub struct Pmsa003i<I2C> {
        i2c: I2C,
    }

    impl<I2C> Pmsa003i<I2C> {
        i2c_constructors!();
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Pmsa003i<I2C> {
        /// See [`super::Pmsa003i::read_frame`].
        pub async fn read_frame(&mut self) -> Result<[u8; FRAME_LEN], Error<I2C::Error>> {
            let mut f = [0u8; FRAME_LEN];
            self.i2c
                .write_read(I2C_ADDRESS, &[0x00], &mut f)
                .await
                .map_err(Error::bus)?;
            Ok(f)
        }
    }

    impl<I2C: embedded_hal_async::i2c::I2c> zigbee_sensor::asynch::Sensor for Pmsa003i<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            Ok(decode_frame(&self.read_frame().await?)?)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::collections::VecDeque;
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

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

    /// Frame with data words `d[0..13]` (data 1..13).
    fn frame(d: [u16; 13]) -> [u8; FRAME_LEN] {
        let mut f = [0u8; FRAME_LEN];
        f[..4].copy_from_slice(&[0x42, 0x4D, 0x00, 0x1C]);
        for (i, w) in d.iter().enumerate() {
            f[4 + 2 * i..6 + 2 * i].copy_from_slice(&w.to_be_bytes());
        }
        let sum = f[..30].iter().map(|b| u16::from(*b)).sum::<u16>();
        f[30..].copy_from_slice(&sum.to_be_bytes());
        f
    }

    fn sample(pm25: u16) -> [u8; FRAME_LEN] {
        frame([5, pm25 + 1, 12, 4, pm25, 11, 1200, 350, 60, 8, 2, 1, 0x9100])
    }

    #[test]
    fn commands_match_manual() {
        assert_eq!(read_command(), [0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71]);
        assert_eq!(
            mode_command(Mode::Passive),
            [0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70]
        );
        assert_eq!(
            mode_command(Mode::Active),
            [0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71]
        );
        assert_eq!(
            sleep_command(true),
            [0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73]
        );
        assert_eq!(
            sleep_command(false),
            [0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74]
        );
    }

    #[test]
    fn decode_fields_and_errors() {
        let f = sample(9);
        let m = decode_frame(&f).unwrap();
        assert_eq!(
            m.standard,
            Pm {
                pm1_0: 5,
                pm2_5: 10,
                pm10: 12
            }
        );
        assert_eq!(m.atmospheric.pm2_5, 9);
        assert_eq!(m.counts.um0_3, 1200);
        assert_eq!(m.counts.um10, 1);
        assert_eq!(m.reserved, 0x9100);
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(
            seen,
            vec![Quantity::Pm1(40), Quantity::Pm25(90), Quantity::Pm10(110)]
        );
        let mut bad = f;
        bad[31] ^= 1;
        assert_eq!(decode_frame(&bad), Err(FrameError::Checksum));
        bad = f;
        bad[3] = 0x14;
        assert_eq!(decode_frame(&bad), Err(FrameError::Header));
    }

    #[test]
    fn parser_skips_acks_and_garbage() {
        let mut p = Parser::new();
        // Garbage, a command acknowledgement (length 4), a "42 42 4D" start,
        // a corrupted frame, then a good one.
        let mut stream = vec![0x00, 0x42, 0x00];
        stream.extend_from_slice(&[0x42, 0x4D, 0x00, 0x04, 0xE1, 0x00, 0x01, 0x74]);
        stream.push(0x42);
        let mut bad = sample(3);
        bad[10] ^= 0x01;
        stream.extend_from_slice(&bad);
        // A corrupted frame whose payload contains another header start.
        let mut hidden = sample(3);
        hidden[8..12].copy_from_slice(&[0x42, 0x4D, 0x00, 0x04]);
        stream.extend_from_slice(&hidden);
        stream.extend_from_slice(&sample(7));
        let out: Vec<Measurement> = stream.iter().filter_map(|b| p.feed(*b)).collect();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].atmospheric.pm2_5, 7);
        assert!(p.take_checksum_error());
    }

    #[test]
    fn active_mode_newest_frame() {
        let mut uart = FakeUart::default();
        uart.rx.extend(sample(1));
        uart.rx.extend(sample(2));
        let mut pms = Pmsx003::new(&mut uart);
        assert_eq!(pms.start_measurement().unwrap(), 0);
        assert_eq!(pms.read_measurement().unwrap().atmospheric.pm2_5, 2);
        assert_eq!(pms.read_measurement(), Err(Error::NotReady));
        let mut bad = sample(3);
        bad[30] ^= 0x80;
        pms.uart.rx.extend(bad);
        assert_eq!(pms.read_measurement(), Err(Error::Crc));
        assert!(uart.tx.is_empty());
    }

    #[test]
    fn passive_mode_and_sleep() {
        let mut uart = FakeUart::default();
        uart.rx.extend(sample(1)); // discarded by set_mode
        let mut pms = Pmsx003::new(&mut uart);
        pms.set_mode(Mode::Passive).unwrap();
        assert_eq!(pms.read_measurement(), Err(Error::NotReady));
        pms.uart.on_write = sample(25).to_vec();
        assert_eq!(pms.start_measurement().unwrap(), RESPONSE_US);
        assert_eq!(pms.read_measurement().unwrap().atmospheric.pm2_5, 25);
        pms.sleep().unwrap();
        assert_eq!(pms.wake().unwrap(), WAKE_STABILIZE_US);
        let expected: Vec<u8> = [
            mode_command(Mode::Passive),
            read_command(),
            sleep_command(true),
            sleep_command(false),
        ]
        .concat();
        assert_eq!(uart.tx, expected);
    }

    #[test]
    fn pmsa003i_i2c() {
        let mut bad = sample(4).to_vec();
        bad[0] = 0;
        let expectations = [
            Transaction::write_read(I2C_ADDRESS, vec![0x00], sample(4).to_vec()),
            Transaction::write_read(I2C_ADDRESS, vec![0x00], bad),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut pm = Pmsa003i::new(&mut i2c);
        assert_eq!(pm.start_measurement().unwrap(), 0);
        assert_eq!(pm.read_measurement().unwrap().atmospheric.pm2_5, 4);
        assert_eq!(pm.read_measurement(), Err(Error::InvalidData));
        i2c.done();
    }

    #[test]
    fn async_drivers() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let mut uart = FakeUart {
            on_write: sample(30).to_vec(),
            ..FakeUart::default()
        };
        let mut pms = asynch::Pmsx003::new(&mut uart).with_mode(Mode::Passive);
        let mut delay = RecordingDelay(0);
        let m = block_on(pms.measure(&mut delay)).unwrap();
        assert_eq!(m.atmospheric.pm2_5, 30);
        assert_eq!(delay.0, RESPONSE_US);

        let expectations = [Transaction::write_read(
            I2C_ADDRESS,
            vec![0x00],
            sample(5).to_vec(),
        )];
        let mut i2c = Mock::new(&expectations);
        let mut pm = asynch::Pmsa003i::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        assert_eq!(
            block_on(pm.measure(&mut delay)).unwrap().atmospheric.pm2_5,
            5
        );
        assert_eq!(delay.0, 0);
        i2c.done();
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
