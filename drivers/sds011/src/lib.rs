//! Nova Fitness SDS011 / SDS021 laser PM2.5 / PM10 sensors over UART.
//!
//! UART 9600 8N1. The sensor sends 10-byte frames `AA cmd d1..d6 cs AB`
//! (`C0` data, `C5` command reply; cs = low byte of d1+…+d6) and accepts
//! 19-byte commands `AA B4 cmd d2..d13 id1 id2 cs AB` (cs = low byte of the
//! sum of bytes 2..=16). Device ID `FFFF` addresses every sensor.
//!
//! Reporting modes (persistent, factory default active):
//!
//! - **Active**: a data frame every second (or every working period).
//!   [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//!   returns 0 and
//!   [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//!   returns the newest data frame or [`Error::NotReady`].
//! - **Query**: `start_measurement` discards stale input, sends "query
//!   data" (0x04) and returns [`RESPONSE_US`].
//!
//! [`LowPower`](zigbee_sensor::blocking::LowPower) uses the sleep/work
//! command (laser and fan off, < 4 mA); `wake` returns the 30 s
//! stabilisation time. The sleep state is not retained over power cycles.
//!
//! References: Nova Fitness "Laser Dust Sensor Control Protocol" V1.3
//! (2015-07-20); "SDS011 Laser PM2.5 Sensor specification" V1.3.
//!
//! ```ignore
//! let mut sds = Sds011::new(uart).with_reporting_mode(ReportingMode::Query);
//! let wait_us = sds.wake()?;        // 30 s until the readings are stable
//! // ... later:
//! let m = measure(&mut sds, &mut delay)?;
//! sds.sleep()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Length of frames sent by the sensor.
pub const FRAME_LEN: usize = 10;
/// Length of command packets.
pub const COMMAND_LEN: usize = 19;
/// Head byte.
pub const HEAD: u8 = 0xAA;
/// Tail byte.
pub const TAIL: u8 = 0xAB;
/// Command ID of packets to the sensor.
pub const COMMAND_ID: u8 = 0xB4;
/// Command ID of data frames.
pub const DATA_ID: u8 = 0xC0;
/// Command ID of command replies.
pub const REPLY_ID: u8 = 0xC5;
/// Device ID that addresses every sensor.
pub const BROADCAST_ID: u16 = 0xFFFF;

/// Command codes (data byte 1).
pub mod cmd {
    /// Query or set the reporting mode.
    pub const REPORTING_MODE: u8 = 2;
    /// Query data.
    pub const QUERY_DATA: u8 = 4;
    /// Set device ID.
    pub const SET_DEVICE_ID: u8 = 5;
    /// Query or set sleep/work.
    pub const SLEEP_WORK: u8 = 6;
    /// Firmware version.
    pub const FIRMWARE_VERSION: u8 = 7;
    /// Query or set the working period.
    pub const WORKING_PERIOD: u8 = 8;
}

/// Allowance for a command reply: 10 bytes at 9600 Bd take 10.4 ms; the
/// protocol gives no processing time, 100 ms is a conservative allowance.
pub const RESPONSE_US: WaitUs = 100_000;
/// Data frame interval in continuous active mode.
pub const REPORT_INTERVAL_US: WaitUs = 1_000_000;
/// Readings are stable 30 s after work starts.
pub const STABILIZE_US: WaitUs = 30_000_000;
/// Recommended minimum interval between query-data commands.
pub const MIN_QUERY_INTERVAL_US: WaitUs = 3_000_000;
/// Longest working period in minutes.
pub const MAX_WORKING_PERIOD_MIN: u8 = 30;

const MAX_READ_BYTES: usize = 256;
const MAX_DRAIN_BYTES: usize = 256;

fn sum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |acc, b| acc.wrapping_add(*b))
}

/// Build a command packet; `data` is bytes 3..=14 (data bytes 2..13).
pub fn command(code: u8, data: [u8; 12], device_id: u16) -> [u8; COMMAND_LEN] {
    let mut f = [0u8; COMMAND_LEN];
    f[0] = HEAD;
    f[1] = COMMAND_ID;
    f[2] = code;
    f[3..15].copy_from_slice(&data);
    f[15..17].copy_from_slice(&device_id.to_be_bytes());
    f[17] = sum(&f[2..17]);
    f[18] = TAIL;
    f
}

fn query_or_set(code: u8, set: Option<u8>, device_id: u16) -> [u8; COMMAND_LEN] {
    let mut data = [0u8; 12];
    if let Some(v) = set {
        data[0] = 1;
        data[1] = v;
    }
    command(code, data, device_id)
}

/// Reporting mode (retained over power cycles).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ReportingMode {
    /// Data frames are sent automatically (factory default).
    #[default]
    Active,
    /// Data frames are sent only in answer to "query data".
    Query,
}

/// Set the reporting mode.
pub fn set_reporting_mode_command(mode: ReportingMode, device_id: u16) -> [u8; COMMAND_LEN] {
    query_or_set(
        cmd::REPORTING_MODE,
        Some(u8::from(mode == ReportingMode::Query)),
        device_id,
    )
}

/// Query the reporting mode (reply: [`Reply::reporting_mode`]).
pub fn query_reporting_mode_command(device_id: u16) -> [u8; COMMAND_LEN] {
    query_or_set(cmd::REPORTING_MODE, None, device_id)
}

/// Query data (reply: a data frame).
pub fn query_data_command(device_id: u16) -> [u8; COMMAND_LEN] {
    command(cmd::QUERY_DATA, [0; 12], device_id)
}

/// Change the device ID of `device_id` to `new_id` (retained).
pub fn set_device_id_command(new_id: u16, device_id: u16) -> [u8; COMMAND_LEN] {
    let mut data = [0u8; 12];
    data[10..12].copy_from_slice(&new_id.to_be_bytes());
    command(cmd::SET_DEVICE_ID, data, device_id)
}

/// Sleep (`false`) or work (`true`).
pub fn set_working_command(work: bool, device_id: u16) -> [u8; COMMAND_LEN] {
    query_or_set(cmd::SLEEP_WORK, Some(u8::from(work)), device_id)
}

/// Query sleep/work (reply: [`Reply::working`]).
pub fn query_working_command(device_id: u16) -> [u8; COMMAND_LEN] {
    query_or_set(cmd::SLEEP_WORK, None, device_id)
}

/// Working period: 0 = continuous, 1–30 = work 30 s then sleep
/// `n × 60 − 30` s (retained). Returns `None` above 30.
pub fn set_working_period_command(minutes: u8, device_id: u16) -> Option<[u8; COMMAND_LEN]> {
    (minutes <= MAX_WORKING_PERIOD_MIN)
        .then(|| query_or_set(cmd::WORKING_PERIOD, Some(minutes), device_id))
}

/// Query the working period (reply: [`Reply::working_period`]).
pub fn query_working_period_command(device_id: u16) -> [u8; COMMAND_LEN] {
    query_or_set(cmd::WORKING_PERIOD, None, device_id)
}

/// Firmware version (reply: [`Reply::firmware_version`]).
pub fn firmware_version_command(device_id: u16) -> [u8; COMMAND_LEN] {
    command(cmd::FIRMWARE_VERSION, [0; 12], device_id)
}

/// A 10-byte frame with valid head, tail and checksum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame(pub [u8; FRAME_LEN]);

impl Frame {
    /// Device ID of the sender.
    pub const fn device_id(&self) -> u16 {
        u16::from_be_bytes([self.0[6], self.0[7]])
    }

    /// Decode a data frame (`AA C0 ..`).
    pub fn measurement(&self) -> Option<Measurement> {
        (self.0[1] == DATA_ID).then(|| Measurement {
            pm2_5: u16::from_le_bytes([self.0[2], self.0[3]]),
            pm10: u16::from_le_bytes([self.0[4], self.0[5]]),
            device_id: self.device_id(),
        })
    }

    /// Decode a command reply (`AA C5 ..`).
    pub fn reply(&self) -> Option<Reply> {
        (self.0[1] == REPLY_ID).then(|| Reply {
            command: self.0[2],
            data: [self.0[3], self.0[4], self.0[5]],
            device_id: self.device_id(),
        })
    }
}

/// Command reply (`AA C5 cmd d1 d2 d3 id1 id2 cs AB`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reply {
    /// Command code the reply belongs to.
    pub command: u8,
    /// Data bytes 2..4.
    pub data: [u8; 3],
    /// Device ID of the sender (the new ID for "set device ID").
    pub device_id: u16,
}

impl Reply {
    /// Reporting mode in a 0x02 reply.
    pub fn reporting_mode(&self) -> Option<ReportingMode> {
        match (self.command, self.data[1]) {
            (cmd::REPORTING_MODE, 0) => Some(ReportingMode::Active),
            (cmd::REPORTING_MODE, 1) => Some(ReportingMode::Query),
            _ => None,
        }
    }

    /// `true` = working, `false` = sleeping, from a 0x06 reply.
    pub fn working(&self) -> Option<bool> {
        (self.command == cmd::SLEEP_WORK).then_some(self.data[1] != 0)
    }

    /// Working period in minutes from a 0x08 reply.
    pub fn working_period(&self) -> Option<u8> {
        (self.command == cmd::WORKING_PERIOD).then_some(self.data[1])
    }

    /// Firmware date from a 0x07 reply.
    pub fn firmware_version(&self) -> Option<FirmwareVersion> {
        (self.command == cmd::FIRMWARE_VERSION).then_some(FirmwareVersion {
            year: self.data[0],
            month: self.data[1],
            day: self.data[2],
        })
    }
}

/// Firmware build date (`year` is years since 2000).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareVersion {
    /// Year − 2000.
    pub year: u8,
    /// Month.
    pub month: u8,
    /// Day.
    pub day: u8,
}

/// Byte-stream parser: resynchronises on `0xAA` and only yields frames with
/// a valid tail and checksum.
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

    /// `true` once if a complete frame with a bad checksum or tail was seen.
    pub fn take_checksum_error(&mut self) -> bool {
        core::mem::take(&mut self.checksum_error)
    }

    /// Feed one received byte.
    pub fn feed(&mut self, byte: u8) -> Option<Frame> {
        if self.len == 0 && byte != HEAD {
            return None;
        }
        self.buf[self.len] = byte;
        self.len += 1;
        if self.len < FRAME_LEN {
            return None;
        }
        if self.buf[9] == TAIL && sum(&self.buf[2..8]) == self.buf[8] {
            self.len = 0;
            return Some(Frame(self.buf));
        }
        self.checksum_error = true;
        match self.buf[1..].iter().position(|b| *b == HEAD) {
            Some(p) => {
                self.buf.copy_within(p + 1.., 0);
                self.len = FRAME_LEN - 1 - p;
            }
            None => self.len = 0,
        }
        None
    }
}

/// One PM reading (0.1 µg/m³).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// PM2.5 in 0.1 µg/m³.
    pub pm2_5: u16,
    /// PM10 in 0.1 µg/m³.
    pub pm10: u16,
    /// Device ID of the sender.
    pub device_id: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Pm25(u32::from(self.pm2_5)));
        f(Quantity::Pm10(u32::from(self.pm10)));
    }
}

fn finish<T, E>(found: Option<T>, parser: &mut Parser) -> Result<T, Error<E>> {
    match found {
        Some(m) => Ok(m),
        None if parser.take_checksum_error() => Err(Error::Crc),
        None => Err(Error::NotReady),
    }
}

fn from_device(frame: &Frame, device_id: u16) -> bool {
    device_id == BROADCAST_ID || frame.device_id() == device_id
}

macro_rules! constructors {
    () => {
        /// Create a driver addressing every sensor ([`BROADCAST_ID`]) and
        /// assuming the factory [`ReportingMode::Active`]; no bus traffic.
        pub const fn new(uart: U) -> Self {
            Self {
                uart,
                parser: Parser::new(),
                device_id: BROADCAST_ID,
                mode: ReportingMode::Active,
            }
        }

        /// Address one sensor and only accept its frames.
        pub const fn with_device_id(mut self, device_id: u16) -> Self {
            self.device_id = device_id;
            self
        }

        /// Assume the (persistent) reporting mode without bus traffic.
        pub const fn with_reporting_mode(mut self, mode: ReportingMode) -> Self {
            self.mode = mode;
            self
        }

        /// Reporting mode the driver assumes.
        pub const fn reporting_mode(&self) -> ReportingMode {
            self.mode
        }

        /// Give the UART back.
        pub fn release(self) -> U {
            self.uart
        }
    };
}

/// Blocking driver.
pub struct Sds011<U> {
    uart: U,
    parser: Parser,
    device_id: u16,
    mode: ReportingMode,
}

impl<U> Sds011<U> {
    constructors!();
}

impl<U> Sds011<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
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
    /// newest frame from the addressed sensor accepted by `want`.
    pub fn poll_frame(
        &mut self,
        want: impl Fn(&Frame) -> bool,
    ) -> Result<Option<Frame>, Error<U::Error>> {
        let mut latest = None;
        let mut buf = [0u8; FRAME_LEN];
        let mut left = MAX_READ_BYTES;
        while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
            let want_n = left.min(buf.len());
            let n = self.uart.read(&mut buf[..want_n]).map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            left -= n;
            for b in &buf[..n] {
                if let Some(f) = self.parser.feed(*b) {
                    if from_device(&f, self.device_id) && want(&f) {
                        latest = Some(f);
                    }
                }
            }
        }
        Ok(latest)
    }

    /// Send a command packet built by one of the `*_command` functions.
    /// Returns the time to wait before [`read_reply`](Self::read_reply).
    pub fn send_command(&mut self, packet: &[u8; COMMAND_LEN]) -> Result<WaitUs, Error<U::Error>> {
        self.uart.write_all(packet).map_err(Error::bus)?;
        self.uart.flush().map_err(Error::bus)?;
        Ok(RESPONSE_US)
    }

    /// Newest reply to `command` (`NotReady` if none has arrived).
    pub fn read_reply(&mut self, command: u8) -> Result<Reply, Error<U::Error>> {
        let found = self
            .poll_frame(|f| f.reply().is_some_and(|r| r.command == command))?
            .and_then(|f| f.reply());
        finish(found, &mut self.parser)
    }

    /// Set the (persistent) reporting mode.
    pub fn set_reporting_mode(&mut self, mode: ReportingMode) -> Result<WaitUs, Error<U::Error>> {
        let wait = self.send_command(&set_reporting_mode_command(mode, self.device_id))?;
        self.mode = mode;
        Ok(wait)
    }

    /// Set the (persistent) working period, 0–30 minutes.
    pub fn set_working_period(&mut self, minutes: u8) -> Result<WaitUs, Error<U::Error>> {
        let packet =
            set_working_period_command(minutes, self.device_id).ok_or(Error::Unsupported)?;
        self.send_command(&packet)
    }

    /// Change the addressed sensor's ID (persistent) and address it by the
    /// new ID afterwards (unless broadcasting).
    pub fn set_device_id(&mut self, new_id: u16) -> Result<WaitUs, Error<U::Error>> {
        let wait = self.send_command(&set_device_id_command(new_id, self.device_id))?;
        if self.device_id != BROADCAST_ID {
            self.device_id = new_id;
        }
        Ok(wait)
    }

    /// Request the firmware version; read it with
    /// [`read_firmware_version`](Self::read_firmware_version).
    pub fn request_firmware_version(&mut self) -> Result<WaitUs, Error<U::Error>> {
        self.send_command(&firmware_version_command(self.device_id))
    }

    /// Firmware version reply.
    pub fn read_firmware_version(&mut self) -> Result<FirmwareVersion, Error<U::Error>> {
        self.read_reply(cmd::FIRMWARE_VERSION)?
            .firmware_version()
            .ok_or(Error::InvalidData)
    }
}

impl<U> zigbee_sensor::blocking::Sensor for Sds011<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type BusError = U::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        match self.mode {
            ReportingMode::Active => Ok(0),
            ReportingMode::Query => {
                self.drain()?;
                self.send_command(&query_data_command(self.device_id))
            }
        }
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
        let found = self
            .poll_frame(|f| f.measurement().is_some())?
            .and_then(|f| f.measurement());
        finish(found, &mut self.parser)
    }
}

impl<U> zigbee_sensor::blocking::LowPower for Sds011<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type BusError = U::Error;

    fn sleep(&mut self) -> Result<(), Error<U::Error>> {
        self.send_command(&set_working_command(false, self.device_id))
            .map(|_| ())
    }

    fn wake(&mut self) -> Result<WaitUs, Error<U::Error>> {
        self.send_command(&set_working_command(true, self.device_id))?;
        Ok(STABILIZE_US)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        BROADCAST_ID, COMMAND_LEN, FRAME_LEN, FirmwareVersion, Frame, MAX_DRAIN_BYTES,
        MAX_READ_BYTES, Measurement, Parser, RESPONSE_US, Reply, ReportingMode, STABILIZE_US, cmd,
        finish, firmware_version_command, from_device, query_data_command, set_device_id_command,
        set_reporting_mode_command, set_working_command, set_working_period_command,
    };
    use embedded_io_async::{Read, ReadReady, Write};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Sds011`].
    pub struct Sds011<U> {
        uart: U,
        parser: Parser,
        device_id: u16,
        mode: ReportingMode,
    }

    impl<U> Sds011<U> {
        constructors!();
    }

    impl<U: Read + ReadReady + Write> Sds011<U> {
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

        /// See [`super::Sds011::poll_frame`].
        pub async fn poll_frame(
            &mut self,
            want: impl Fn(&Frame) -> bool,
        ) -> Result<Option<Frame>, Error<U::Error>> {
            let mut latest = None;
            let mut buf = [0u8; FRAME_LEN];
            let mut left = MAX_READ_BYTES;
            while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
                let want_n = left.min(buf.len());
                let n = self
                    .uart
                    .read(&mut buf[..want_n])
                    .await
                    .map_err(Error::bus)?;
                if n == 0 {
                    break;
                }
                left -= n;
                for b in &buf[..n] {
                    if let Some(f) = self.parser.feed(*b) {
                        if from_device(&f, self.device_id) && want(&f) {
                            latest = Some(f);
                        }
                    }
                }
            }
            Ok(latest)
        }

        /// See [`super::Sds011::send_command`].
        pub async fn send_command(
            &mut self,
            packet: &[u8; COMMAND_LEN],
        ) -> Result<WaitUs, Error<U::Error>> {
            self.uart.write_all(packet).await.map_err(Error::bus)?;
            self.uart.flush().await.map_err(Error::bus)?;
            Ok(RESPONSE_US)
        }

        /// See [`super::Sds011::read_reply`].
        pub async fn read_reply(&mut self, command: u8) -> Result<Reply, Error<U::Error>> {
            let found = self
                .poll_frame(|f| f.reply().is_some_and(|r| r.command == command))
                .await?
                .and_then(|f| f.reply());
            finish(found, &mut self.parser)
        }

        /// See [`super::Sds011::set_reporting_mode`].
        pub async fn set_reporting_mode(
            &mut self,
            mode: ReportingMode,
        ) -> Result<WaitUs, Error<U::Error>> {
            let wait = self
                .send_command(&set_reporting_mode_command(mode, self.device_id))
                .await?;
            self.mode = mode;
            Ok(wait)
        }

        /// See [`super::Sds011::set_working_period`].
        pub async fn set_working_period(&mut self, minutes: u8) -> Result<WaitUs, Error<U::Error>> {
            let packet =
                set_working_period_command(minutes, self.device_id).ok_or(Error::Unsupported)?;
            self.send_command(&packet).await
        }

        /// See [`super::Sds011::set_device_id`].
        pub async fn set_device_id(&mut self, new_id: u16) -> Result<WaitUs, Error<U::Error>> {
            let wait = self
                .send_command(&set_device_id_command(new_id, self.device_id))
                .await?;
            if self.device_id != BROADCAST_ID {
                self.device_id = new_id;
            }
            Ok(wait)
        }

        /// See [`super::Sds011::request_firmware_version`].
        pub async fn request_firmware_version(&mut self) -> Result<WaitUs, Error<U::Error>> {
            self.send_command(&firmware_version_command(self.device_id))
                .await
        }

        /// See [`super::Sds011::read_firmware_version`].
        pub async fn read_firmware_version(&mut self) -> Result<FirmwareVersion, Error<U::Error>> {
            self.read_reply(cmd::FIRMWARE_VERSION)
                .await?
                .firmware_version()
                .ok_or(Error::InvalidData)
        }
    }

    impl<U: Read + ReadReady + Write> zigbee_sensor::asynch::Sensor for Sds011<U> {
        type BusError = U::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
            match self.mode {
                ReportingMode::Active => Ok(0),
                ReportingMode::Query => {
                    self.drain().await?;
                    self.send_command(&query_data_command(self.device_id)).await
                }
            }
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
            let found = self
                .poll_frame(|f| f.measurement().is_some())
                .await?
                .and_then(|f| f.measurement());
            finish(found, &mut self.parser)
        }
    }

    impl<U: Read + ReadReady + Write> zigbee_sensor::asynch::LowPower for Sds011<U> {
        type BusError = U::Error;

        async fn sleep(&mut self) -> Result<(), Error<U::Error>> {
            self.send_command(&set_working_command(false, self.device_id))
                .await
                .map(|_| ())
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<U::Error>> {
            self.send_command(&set_working_command(true, self.device_id))
                .await?;
            Ok(STABILIZE_US)
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

    fn hex(s: &str) -> Vec<u8> {
        s.split_whitespace()
            .map(|b| u8::from_str_radix(b, 16).unwrap())
            .collect()
    }

    /// Protocol example: PM2.5 123.6, PM10 261.8 µg/m³ from sensor A160.
    const DATA: [u8; 10] = [0xAA, 0xC0, 0xD4, 0x04, 0x3A, 0x0A, 0xA1, 0x60, 0x1D, 0xAB];

    #[test]
    fn commands_match_protocol_examples() {
        let id = 0xA160;
        let cases: [(&[u8], &str); 9] = [
            (
                &query_data_command(BROADCAST_ID),
                "AA B4 04 00 00 00 00 00 00 00 00 00 00 00 00 FF FF 02 AB",
            ),
            (
                &query_reporting_mode_command(BROADCAST_ID),
                "AA B4 02 00 00 00 00 00 00 00 00 00 00 00 00 FF FF 00 AB",
            ),
            (
                &set_reporting_mode_command(ReportingMode::Query, id),
                "AA B4 02 01 01 00 00 00 00 00 00 00 00 00 00 A1 60 05 AB",
            ),
            (
                &set_device_id_command(0xA001, id),
                "AA B4 05 00 00 00 00 00 00 00 00 00 00 A0 01 A1 60 A7 AB",
            ),
            (
                &set_working_command(false, id),
                "AA B4 06 01 00 00 00 00 00 00 00 00 00 00 00 A1 60 08 AB",
            ),
            (
                &set_working_command(true, id),
                "AA B4 06 01 01 00 00 00 00 00 00 00 00 00 00 A1 60 09 AB",
            ),
            (
                &query_working_command(id),
                "AA B4 06 00 00 00 00 00 00 00 00 00 00 00 00 A1 60 07 AB",
            ),
            (
                &set_working_period_command(1, id).unwrap(),
                "AA B4 08 01 01 00 00 00 00 00 00 00 00 00 00 A1 60 0B AB",
            ),
            (
                &firmware_version_command(id),
                "AA B4 07 00 00 00 00 00 00 00 00 00 00 00 00 A1 60 08 AB",
            ),
        ];
        for (got, want) in cases {
            assert_eq!(got, hex(want).as_slice());
        }
        assert_eq!(query_working_period_command(id)[17], 0x09);
        assert_eq!(set_working_period_command(31, id), None);
    }

    #[test]
    fn decodes_frames_and_replies() {
        let mut p = Parser::new();
        let mut stream = vec![0x01, 0xAA];
        stream.extend_from_slice(&DATA);
        for r in [
            "AA C5 07 0F 07 0A A1 60 28 AB",
            "AA C5 02 00 01 00 A1 60 04 AB",
            "AA C5 08 00 02 00 A1 60 0B AB",
            "AA C5 06 01 00 00 A1 60 08 AB",
        ] {
            stream.extend(hex(r));
        }
        let frames: Vec<Frame> = stream.iter().filter_map(|b| p.feed(*b)).collect();
        assert_eq!(frames.len(), 5);
        assert_eq!(
            frames[0].measurement(),
            Some(Measurement {
                pm2_5: 1236,
                pm10: 2618,
                device_id: 0xA160
            })
        );
        let replies: Vec<Reply> = frames[1..].iter().map(|f| f.reply().unwrap()).collect();
        assert_eq!(
            replies[0].firmware_version(),
            Some(FirmwareVersion {
                year: 15,
                month: 7,
                day: 10
            })
        );
        assert_eq!(replies[1].reporting_mode(), Some(ReportingMode::Query));
        assert_eq!(replies[2].working_period(), Some(2));
        assert_eq!(replies[3].working(), Some(false));
        assert_eq!(replies[3].firmware_version(), None);
    }

    #[test]
    fn bad_checksum_or_tail_rejected() {
        let mut p = Parser::new();
        let mut bad = DATA;
        bad[9] = 0x00;
        assert!(bad.iter().all(|b| p.feed(*b).is_none()));
        assert!(p.take_checksum_error());
        bad = DATA;
        bad[2] ^= 1;
        assert!(bad.iter().all(|b| p.feed(*b).is_none()));
        assert!(p.take_checksum_error());
        assert!(DATA.iter().filter_map(|b| p.feed(*b)).next().is_some());
    }

    #[test]
    fn active_mode_newest_frame() {
        let mut uart = FakeUart::default();
        uart.rx.extend(hex("AA C5 06 01 01 00 A1 60 09 AB"));
        uart.rx.extend(DATA);
        let newer = [0xAA, 0xC0, 0x0A, 0x00, 0x14, 0x00, 0xA1, 0x60, 0x1F, 0xAB];
        uart.rx.extend(newer);
        let mut sds = Sds011::new(&mut uart);
        assert_eq!(sds.start_measurement().unwrap(), 0);
        let m = sds.read_measurement().unwrap();
        assert_eq!((m.pm2_5, m.pm10), (10, 20));
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(seen, vec![Quantity::Pm25(10), Quantity::Pm10(20)]);
        assert_eq!(sds.read_measurement(), Err(Error::NotReady));
        assert!(uart.tx.is_empty());
    }

    #[test]
    fn query_mode_device_filter_and_low_power() {
        let mut uart = FakeUart::default();
        let mut sds = Sds011::new(&mut uart)
            .with_device_id(0xA160)
            .with_reporting_mode(ReportingMode::Query);
        // A frame from another sensor is ignored.
        let mut other = DATA;
        other[7] = 0x61;
        other[8] = other[8].wrapping_add(1);
        sds.uart.on_write = other.iter().chain(&DATA).copied().collect();
        assert_eq!(sds.start_measurement().unwrap(), RESPONSE_US);
        assert_eq!(sds.read_measurement().unwrap().pm2_5, 1236);
        sds.sleep().unwrap();
        assert_eq!(sds.wake().unwrap(), STABILIZE_US);
        assert_eq!(sds.set_working_period(31), Err(Error::Unsupported));
        sds.uart.on_write = hex("AA C5 07 0F 07 0A A1 60 28 AB");
        sds.request_firmware_version().unwrap();
        assert_eq!(sds.read_firmware_version().unwrap().year, 15);
        sds.set_device_id(0xA001).unwrap();
        let tx = &uart.tx;
        assert_eq!(&tx[..19], &query_data_command(0xA160));
        assert_eq!(&tx[19..38], &set_working_command(false, 0xA160));
        assert_eq!(&tx[38..57], &set_working_command(true, 0xA160));
        assert_eq!(&tx[76..], &set_device_id_command(0xA001, 0xA160));
    }

    #[test]
    fn async_query_measure() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let mut uart = FakeUart::default();
        block_on(asynch::Sds011::new(&mut uart).set_reporting_mode(ReportingMode::Query)).unwrap();
        uart.on_write = DATA.to_vec();
        let mut sds = asynch::Sds011::new(&mut uart).with_reporting_mode(ReportingMode::Query);
        let mut delay = RecordingDelay(0);
        let m = block_on(sds.measure(&mut delay)).unwrap();
        assert_eq!(m.pm10, 2618);
        assert_eq!(delay.0, RESPONSE_US);
        assert_eq!(&uart.tx[19..], &query_data_command(BROADCAST_ID));
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
