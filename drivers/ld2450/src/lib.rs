//! Hi-Link HLK-LD2450 24 GHz multi-target (up to 3) tracking radar, UART
//! protocol.
//!
//! Reference: "HLK-LD2450 Motion Target Detection and Tracking Module
//! Serial Communication Protocol" V1.03 (2023-10-17).
//!
//! UART 256000 8N1 by default (see [`Baud`]); words little-endian.
//!
//! * Report frames, 10 per second: `AA FF 03 00`, three 8-byte targets
//!   (x, y, speed, resolution), `55 CC`. X/Y (mm) and speed (cm/s) use a
//!   sign-magnitude encoding where bit 15 set means **positive**
//!   ([`decode_signed`]). An all-zero target slot is empty.
//! * Command frames and ACKs use the same `FD FC FB FA … 04 03 02 01`
//!   framing as the LD2410; commands must be bracketed by
//!   [`Command::EnableConfig`] / [`Command::EndConfig`].
//!
//! `read_measurement` returns the newest report (blocking: drains received
//! bytes, [`Error::NotReady`] if none; async: awaits the next report).
//! Command helpers read until the matching ACK and block/await on a silent
//! UART; use [`Ld2450::send`] + [`Ld2450::poll_ack`] for a bounded wait.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Factory-default baud rate.
pub const DEFAULT_BAUD_RATE: u32 = 256_000;

const REPORT_HEADER: [u8; 4] = [0xAA, 0xFF, 0x03, 0x00];
const REPORT_TAIL: [u8; 2] = [0x55, 0xCC];
const REPORT_LEN: usize = 30;
const CMD_HEADER: [u8; 4] = [0xFD, 0xFC, 0xFB, 0xFA];
const CMD_TAIL: [u8; 4] = [0x04, 0x03, 0x02, 0x01];
const BUF_LEN: usize = 48;
/// Longest encoded command frame.
pub const MAX_COMMAND_LEN: usize = 38;
const ACK_BYTE_BUDGET: usize = 1_024;

/// Decode the report's sign-magnitude field: bit 15 set = positive,
/// clear = negative (protocol §2.3, "the highest bit 1 corresponds to
/// positive").
pub const fn decode_signed(raw: u16) -> i16 {
    if raw & 0x8000 != 0 {
        (raw & 0x7FFF) as i16
    } else {
        -((raw & 0x7FFF) as i16)
    }
}

/// Integer square root (floor).
pub const fn isqrt(n: u32) -> u32 {
    let mut x = n;
    let mut y = x.div_ceil(2);
    if n < 2 {
        return n;
    }
    while y < x {
        x = y;
        y = (x + n / x) / 2;
    }
    x
}

/// One tracked target.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Target {
    /// Lateral position, mm (sign per the module's axis convention).
    pub x_mm: i16,
    /// Distance in front of the module, mm.
    pub y_mm: i16,
    /// Radial speed, cm/s (negative = approaching per the document's
    /// example).
    pub speed_cm_s: i16,
    /// Distance-gate size, mm.
    pub resolution_mm: u16,
}

impl Target {
    /// Straight-line distance from the module, mm.
    pub fn distance_mm(&self) -> u32 {
        let x = i32::from(self.x_mm).unsigned_abs();
        let y = i32::from(self.y_mm).unsigned_abs();
        isqrt(x * x + y * y)
    }

    fn decode(b: &[u8]) -> Option<Self> {
        if b.iter().all(|&x| x == 0) {
            return None;
        }
        let w = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        Some(Self {
            x_mm: decode_signed(w(0)),
            y_mm: decode_signed(w(2)),
            speed_cm_s: decode_signed(w(4)),
            resolution_mm: w(6),
        })
    }
}

/// One report frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Report {
    /// Target slots; `None` = empty.
    pub targets: [Option<Target>; 3],
}

impl Report {
    /// Decode a complete 30-byte report frame.
    pub fn decode(frame: &[u8]) -> Option<Self> {
        if frame.len() != REPORT_LEN || frame[..4] != REPORT_HEADER || frame[28..] != REPORT_TAIL {
            return None;
        }
        let t = |i: usize| Target::decode(&frame[4 + 8 * i..12 + 8 * i]);
        Some(Self {
            targets: [t(0), t(1), t(2)],
        })
    }

    /// Number of tracked targets.
    pub fn count(&self) -> usize {
        self.targets.iter().flatten().count()
    }

    /// Distance to the nearest target, mm.
    pub fn nearest_mm(&self) -> Option<u32> {
        self.targets.iter().flatten().map(Target::distance_mm).min()
    }
}

impl Readings for Report {
    /// `Occupancy`, plus `Distance` to the nearest target (mm) if any.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Occupancy(self.count() > 0));
        if let Some(mm) = self.nearest_mm() {
            f(Quantity::Distance(mm));
        }
    }
}

/// Command ACK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ack {
    /// Command word the ACK answers (`0x0100` bit removed).
    pub command: u16,
    /// 0 = success, 1 = failure.
    pub status: u16,
    data: [u8; 32],
    len: u8,
}

impl Ack {
    /// Decode an ACK payload (between the length field and the tail).
    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() < 4 || payload.len() > 36 || payload[1] & 0x01 == 0 {
            return None;
        }
        let data = &payload[4..];
        let mut ack = Self {
            command: u16::from_le_bytes([payload[0], payload[1]]) & !0x0100,
            status: u16::from_le_bytes([payload[2], payload[3]]),
            data: [0; 32],
            len: data.len() as u8,
        };
        ack.data[..data.len()].copy_from_slice(data);
        Some(ack)
    }

    /// Return data after the status word.
    pub fn data(&self) -> &[u8] {
        &self.data[..usize::from(self.len)]
    }

    fn word(&self, i: usize) -> Option<u16> {
        let d = self.data();
        Some(u16::from_le_bytes([*d.get(i)?, *d.get(i + 1)?]))
    }

    /// Enable-config ACK: (protocol version, buffer size).
    pub fn config_info(&self) -> Option<(u16, u16)> {
        Some((self.word(0)?, self.word(2)?))
    }

    /// Tracking-mode query ACK.
    pub fn tracking_mode(&self) -> Option<TrackingMode> {
        match self.word(0)? {
            1 => Some(TrackingMode::Single),
            2 => Some(TrackingMode::Multi),
            _ => None,
        }
    }

    /// Firmware-version ACK.
    pub fn firmware_version(&self) -> Option<FirmwareVersion> {
        let d = self.data();
        if d.len() < 8 {
            return None;
        }
        Some(FirmwareVersion {
            kind: self.word(0)?,
            major: d[3],
            minor: d[2],
            build: u32::from_le_bytes([d[4], d[5], d[6], d[7]]),
        })
    }

    /// MAC-address ACK (6 bytes as in the document's example).
    pub fn mac(&self) -> Option<[u8; 6]> {
        self.data().get(..6)?.try_into().ok()
    }

    /// Zone-filter query ACK.
    pub fn zone_filter(&self) -> Option<ZoneFilter> {
        let d = self.data();
        if d.len() < 26 {
            return None;
        }
        let mode = match self.word(0)? {
            0 => ZoneMode::Disabled,
            1 => ZoneMode::DetectInside,
            2 => ZoneMode::IgnoreInside,
            _ => return None,
        };
        let c = |i: usize| i16::from_le_bytes([d[i], d[i + 1]]);
        let z = |i: usize| Zone {
            x1: c(2 + 8 * i),
            y1: c(4 + 8 * i),
            x2: c(6 + 8 * i),
            y2: c(8 + 8 * i),
        };
        Some(ZoneFilter {
            mode,
            zones: [z(0), z(1), z(2)],
        })
    }
}

/// Firmware version; printed as `V{major}.{minor:02}.{build:08x}`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareVersion {
    /// Firmware type word.
    pub kind: u16,
    /// Major version.
    pub major: u8,
    /// Minor version.
    pub minor: u8,
    /// Build, BCD date/time `YYMMDDHH`.
    pub build: u32,
}

/// Tracking mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingMode {
    /// Track one target.
    Single,
    /// Track up to three targets (default).
    Multi,
}

/// UART baud-rate index for [`Command::SetBaudRate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum Baud {
    /// 9600.
    B9600 = 1,
    /// 19200.
    B19200,
    /// 38400.
    B38400,
    /// 57600.
    B57600,
    /// 115200.
    B115200,
    /// 230400.
    B230400,
    /// 256000 (default).
    B256000,
    /// 460800.
    B460800,
}

/// Zone-filter behaviour.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ZoneMode {
    /// No filtering (default).
    Disabled = 0,
    /// Report only targets inside the zones.
    DetectInside = 1,
    /// Ignore targets inside the zones.
    IgnoreInside = 2,
}

/// Rectangle given by two diagonal corners, mm (two's complement; unlike
/// report coordinates). All zero = unused.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Zone {
    /// Corner 1 X.
    pub x1: i16,
    /// Corner 1 Y.
    pub y1: i16,
    /// Corner 2 X.
    pub x2: i16,
    /// Corner 2 Y.
    pub y2: i16,
}

/// Zone filtering (persistent, applies immediately).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ZoneFilter {
    /// Behaviour.
    pub mode: ZoneMode,
    /// Up to three zones.
    pub zones: [Zone; 3],
}

/// Commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    /// Enter configuration mode (0x00FF).
    EnableConfig,
    /// Leave configuration mode (0x00FE).
    EndConfig,
    /// Single- or multi-target tracking (0x0080 / 0x0090).
    SetTrackingMode(TrackingMode),
    /// Query tracking mode (0x0091).
    QueryTrackingMode,
    /// Read firmware version (0x00A0).
    ReadFirmwareVersion,
    /// Set baud rate (0x00A1); applies after restart.
    SetBaudRate(Baud),
    /// Factory reset (0x00A2); applies after restart.
    FactoryReset,
    /// Restart (0x00A3).
    Restart,
    /// Bluetooth on/off (0x00A4); applies after restart.
    Bluetooth(bool),
    /// Read MAC address (0x00A5).
    ReadMac,
    /// Query zone filter (0x00C1).
    QueryZoneFilter,
    /// Set zone filter (0x00C2).
    SetZoneFilter(ZoneFilter),
}

impl Command {
    /// Command word.
    pub const fn code(&self) -> u16 {
        match self {
            Self::EnableConfig => 0x00FF,
            Self::EndConfig => 0x00FE,
            Self::SetTrackingMode(TrackingMode::Single) => 0x0080,
            Self::SetTrackingMode(TrackingMode::Multi) => 0x0090,
            Self::QueryTrackingMode => 0x0091,
            Self::ReadFirmwareVersion => 0x00A0,
            Self::SetBaudRate(_) => 0x00A1,
            Self::FactoryReset => 0x00A2,
            Self::Restart => 0x00A3,
            Self::Bluetooth(_) => 0x00A4,
            Self::ReadMac => 0x00A5,
            Self::QueryZoneFilter => 0x00C1,
            Self::SetZoneFilter(_) => 0x00C2,
        }
    }

    fn value(&self, v: &mut [u8]) -> usize {
        let mut word = |i: usize, w: u16| v[i..i + 2].copy_from_slice(&w.to_le_bytes());
        match *self {
            Self::EnableConfig | Self::ReadMac => word(0, 1),
            Self::SetBaudRate(b) => word(0, b as u16),
            Self::Bluetooth(on) => word(0, if on { 0x0100 } else { 0 }),
            Self::SetZoneFilter(f) => {
                word(0, f.mode as u16);
                for (i, z) in f.zones.iter().enumerate() {
                    for (j, c) in [z.x1, z.y1, z.x2, z.y2].into_iter().enumerate() {
                        word(2 + 8 * i + 2 * j, c as u16);
                    }
                }
                return 26;
            }
            _ => return 0,
        }
        2
    }

    /// Encode the full command frame; returns its length.
    pub fn encode(&self, out: &mut [u8; MAX_COMMAND_LEN]) -> usize {
        out[..4].copy_from_slice(&CMD_HEADER);
        out[6..8].copy_from_slice(&self.code().to_le_bytes());
        let n = 2 + self.value(&mut out[8..]);
        out[4..6].copy_from_slice(&(n as u16).to_le_bytes());
        out[6 + n..10 + n].copy_from_slice(&CMD_TAIL);
        10 + n
    }
}

/// A decoded frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Frame {
    /// Report frame.
    Report(Report),
    /// Command ACK.
    Ack(Ack),
}

/// Incremental frame parser for report and ACK frames.
#[derive(Debug, Clone)]
pub struct Parser {
    buf: [u8; BUF_LEN],
    len: usize,
    errors: u32,
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

enum Check {
    More,
    Bad,
    Done(usize),
}

impl Parser {
    /// Empty parser.
    pub const fn new() -> Self {
        Self {
            buf: [0; BUF_LEN],
            len: 0,
            errors: 0,
        }
    }

    /// Frames dropped (bad tail, oversize or undecodable), wrapping.
    pub const fn errors(&self) -> u32 {
        self.errors
    }

    fn header_prefix(b: &[u8]) -> bool {
        let n = b.len().min(4);
        b[..n] == REPORT_HEADER[..n] || b[..n] == CMD_HEADER[..n]
    }

    fn check(&self) -> Check {
        let b = &self.buf[..self.len];
        if !Self::header_prefix(b) {
            return Check::Bad;
        }
        if b.len() < 6 {
            return Check::More;
        }
        let report = b[..4] == REPORT_HEADER;
        let total = if report {
            REPORT_LEN
        } else {
            10 + usize::from(u16::from_le_bytes([b[4], b[5]]))
        };
        if total > BUF_LEN {
            return Check::Bad;
        }
        if b.len() < total {
            return Check::More;
        }
        let ok = if report {
            b[total - 2..total] == REPORT_TAIL
        } else {
            b[total - 4..total] == CMD_TAIL
        };
        if ok { Check::Done(total) } else { Check::Bad }
    }

    fn realign(&mut self) {
        let start = (1..self.len)
            .find(|&i| Self::header_prefix(&self.buf[i..self.len]))
            .unwrap_or(self.len);
        self.buf.copy_within(start..self.len, 0);
        self.len -= start;
    }

    /// Feed one byte; returns a frame when one completes.
    pub fn feed(&mut self, byte: u8) -> Option<Frame> {
        if self.len == BUF_LEN {
            self.realign();
        }
        self.buf[self.len] = byte;
        self.len += 1;
        loop {
            if self.len == 0 {
                return None;
            }
            match self.check() {
                Check::More => return None,
                Check::Bad => {
                    if self.len >= 4 {
                        self.errors = self.errors.wrapping_add(1);
                    }
                    self.realign();
                }
                Check::Done(total) => {
                    let frame = if self.buf[0] == REPORT_HEADER[0] {
                        Report::decode(&self.buf[..total]).map(Frame::Report)
                    } else {
                        Ack::decode(&self.buf[6..total - 4]).map(Frame::Ack)
                    };
                    self.buf.copy_within(total..self.len, 0);
                    self.len -= total;
                    if frame.is_none() {
                        self.errors = self.errors.wrapping_add(1);
                    }
                    return frame;
                }
            }
        }
    }
}

fn check_status<E>(ack: Ack) -> Result<Ack, Error<E>> {
    if ack.status == 0 {
        Ok(ack)
    } else {
        Err(Error::InvalidData)
    }
}

macro_rules! api {
    ($($asyncness:ident)?; $($aw:tt)*) => {
        /// Create a driver; no UART traffic.
        pub const fn new(uart: U) -> Self {
            Self {
                uart,
                parser: Parser::new(),
            }
        }

        /// Give the UART back.
        pub fn release(self) -> U {
            self.uart
        }

        /// Enter configuration mode; returns (protocol version, buffer size).
        pub $($asyncness)? fn enable_config(&mut self) -> Result<(u16, u16), Error<U::Error>> {
            let ack = self.command(&Command::EnableConfig) $($aw)* ?;
            ack.config_info().ok_or(Error::InvalidData)
        }

        /// Leave configuration mode; reporting resumes.
        pub $($asyncness)? fn end_config(&mut self) -> Result<(), Error<U::Error>> {
            self.command(&Command::EndConfig) $($aw)* .map(drop)
        }

        /// Single- or multi-target tracking.
        pub $($asyncness)? fn set_tracking_mode(&mut self, mode: TrackingMode) -> Result<(), Error<U::Error>> {
            self.command(&Command::SetTrackingMode(mode)) $($aw)* .map(drop)
        }

        /// Current tracking mode.
        pub $($asyncness)? fn tracking_mode(&mut self) -> Result<TrackingMode, Error<U::Error>> {
            let ack = self.command(&Command::QueryTrackingMode) $($aw)* ?;
            ack.tracking_mode().ok_or(Error::InvalidData)
        }

        /// Firmware version.
        pub $($asyncness)? fn firmware_version(&mut self) -> Result<FirmwareVersion, Error<U::Error>> {
            let ack = self.command(&Command::ReadFirmwareVersion) $($aw)* ?;
            ack.firmware_version().ok_or(Error::InvalidData)
        }

        /// Read the zone filter.
        pub $($asyncness)? fn zone_filter(&mut self) -> Result<ZoneFilter, Error<U::Error>> {
            let ack = self.command(&Command::QueryZoneFilter) $($aw)* ?;
            ack.zone_filter().ok_or(Error::InvalidData)
        }

        /// Write the zone filter.
        pub $($asyncness)? fn set_zone_filter(&mut self, filter: ZoneFilter) -> Result<(), Error<U::Error>> {
            self.command(&Command::SetZoneFilter(filter)) $($aw)* .map(drop)
        }

        /// Write `cmd` and read until its ACK; a failure status maps to
        /// [`Error::InvalidData`], `Timeout` after ~1 KiB of other traffic.
        pub $($asyncness)? fn command(&mut self, cmd: &Command) -> Result<Ack, Error<U::Error>> {
            self.send(cmd) $($aw)* ?;
            let mut byte = [0u8; 1];
            for _ in 0..ACK_BYTE_BUDGET {
                let n = self.uart.read(&mut byte) $($aw)* .map_err(Error::bus)?;
                if n == 1 {
                    if let Some(Frame::Ack(ack)) = self.parser.feed(byte[0]) {
                        if ack.command == cmd.code() {
                            return check_status(ack);
                        }
                    }
                }
            }
            Err(Error::Timeout)
        }

        /// Write a command frame without waiting for the ACK.
        pub $($asyncness)? fn send(&mut self, cmd: &Command) -> Result<(), Error<U::Error>> {
            let mut frame = [0u8; MAX_COMMAND_LEN];
            let n = cmd.encode(&mut frame);
            self.uart.write_all(&frame[..n]) $($aw)* .map_err(Error::bus)?;
            self.uart.flush() $($aw)* .map_err(Error::bus)
        }
    };
}

/// Blocking driver over `embedded_io::Read + ReadReady + Write`.
pub struct Ld2450<U> {
    uart: U,
    parser: Parser,
}

impl<U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write> Ld2450<U> {
    api!(;);

    fn drain(&mut self, command: Option<u16>) -> Result<Frame, Error<U::Error>> {
        let mut latest = None;
        let mut buf = [0u8; 16];
        while self.uart.read_ready().map_err(Error::bus)? {
            let n = self.uart.read(&mut buf).map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            for &b in &buf[..n] {
                match self.parser.feed(b) {
                    Some(Frame::Ack(ack)) if Some(ack.command) == command => {
                        return Ok(Frame::Ack(ack));
                    }
                    Some(Frame::Report(r)) if command.is_none() => latest = Some(r),
                    _ => {}
                }
            }
        }
        latest.map(Frame::Report).ok_or(Error::NotReady)
    }

    /// Newest report received so far (never blocks), or `NotReady`.
    pub fn poll(&mut self) -> Result<Report, Error<U::Error>> {
        match self.drain(None)? {
            Frame::Report(r) => Ok(r),
            Frame::Ack(_) => Err(Error::NotReady),
        }
    }

    /// Non-blocking check for the ACK of a command written with
    /// [`Self::send`]; `NotReady` until it arrives.
    pub fn poll_ack(&mut self, cmd: &Command) -> Result<Ack, Error<U::Error>> {
        match self.drain(Some(cmd.code()))? {
            Frame::Ack(ack) => check_status(ack),
            Frame::Report(_) => Err(Error::NotReady),
        }
    }
}

impl<U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write>
    zigbee_sensor::blocking::Sensor for Ld2450<U>
{
    type BusError = U::Error;
    type Measurement = Report;

    /// Free-running: returns `0`.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        Ok(0)
    }

    /// See [`Ld2450::poll`].
    fn read_measurement(&mut self) -> Result<Report, Error<U::Error>> {
        self.poll()
    }
}

/// Async driver over `embedded_io_async::Read + Write`.
pub mod asynch {
    use super::{
        ACK_BYTE_BUDGET, Ack, Command, FirmwareVersion, Frame, MAX_COMMAND_LEN, Parser, Report,
        TrackingMode, ZoneFilter, check_status,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Ld2450`].
    pub struct Ld2450<U> {
        uart: U,
        parser: Parser,
    }

    impl<U: embedded_io_async::Read + embedded_io_async::Write> Ld2450<U> {
        api!(async; .await);

        /// Await the next report (ACKs and corrupt frames are skipped).
        pub async fn next_report(&mut self) -> Result<Report, Error<U::Error>> {
            let mut buf = [0u8; 16];
            loop {
                let n = self.uart.read(&mut buf).await.map_err(Error::bus)?;
                for &b in &buf[..n] {
                    if let Some(Frame::Report(r)) = self.parser.feed(b) {
                        return Ok(r);
                    }
                }
            }
        }
    }

    impl<U: embedded_io_async::Read + embedded_io_async::Write> zigbee_sensor::asynch::Sensor
        for Ld2450<U>
    {
        type BusError = U::Error;
        type Measurement = Report;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
            Ok(0)
        }

        /// Awaits the next report.
        async fn read_measurement(&mut self) -> Result<Report, Error<U::Error>> {
            self.next_report().await
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::collections::VecDeque;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    // Protocol V1.03 §2.3 example: one target.
    const REPORT: [u8; 30] = [
        0xAA, 0xFF, 0x03, 0x00, 0x0E, 0x03, 0xB1, 0x86, 0x10, 0x00, 0x40, 0x01, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x55, 0xCC,
    ];

    #[derive(Default)]
    struct FakeUart {
        rx: VecDeque<u8>,
        tx: Vec<u8>,
    }

    impl embedded_io::ErrorType for FakeUart {
        type Error = embedded_io::ErrorKind;
    }

    impl embedded_io::Read for FakeUart {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            if self.rx.is_empty() {
                return Err(embedded_io::ErrorKind::TimedOut);
            }
            let n = buf.len().min(self.rx.len()).min(7);
            for b in &mut buf[..n] {
                *b = self.rx.pop_front().unwrap();
            }
            Ok(n)
        }
    }

    impl embedded_io::ReadReady for FakeUart {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            Ok(!self.rx.is_empty())
        }
    }

    impl embedded_io::Write for FakeUart {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.tx.extend_from_slice(buf);
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

    fn encode(cmd: Command) -> Vec<u8> {
        let mut buf = [0; MAX_COMMAND_LEN];
        let n = cmd.encode(&mut buf);
        buf[..n].to_vec()
    }

    fn ack(cmd: u8, data: &[u8]) -> Vec<u8> {
        let mut v = std::vec![
            0xFD,
            0xFC,
            0xFB,
            0xFA,
            4 + data.len() as u8,
            0,
            cmd,
            1,
            0,
            0
        ];
        v.extend_from_slice(data);
        v.extend_from_slice(&CMD_TAIL);
        v
    }

    #[test]
    fn sign_magnitude_example() {
        // §2.3: X 0x030E = -782, Y 0x86B1 = 1713, speed 0x0010 = -16.
        assert_eq!(decode_signed(0x030E), -782);
        assert_eq!(decode_signed(0x86B1), 1_713);
        assert_eq!(decode_signed(0x0010), -16);
        assert_eq!(isqrt(0), 0);
        assert_eq!(isqrt(99), 9);
        assert_eq!(isqrt(100), 10);
        assert_eq!(isqrt(u32::MAX), 65_535);
    }

    #[test]
    fn report_example() {
        let r = Report::decode(&REPORT).unwrap();
        assert_eq!(
            r.targets[0],
            Some(Target {
                x_mm: -782,
                y_mm: 1_713,
                speed_cm_s: -16,
                resolution_mm: 320
            })
        );
        assert_eq!(r.targets[1..], [None, None]);
        let mut q = Vec::new();
        r.for_each(&mut |x| q.push(x));
        // sqrt(782² + 1713²) = 1883.1
        assert_eq!(q, [Quantity::Occupancy(true), Quantity::Distance(1_883)]);
    }

    #[test]
    fn command_encoding() {
        assert_eq!(
            encode(Command::SetTrackingMode(TrackingMode::Single)),
            [
                0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01
            ]
        );
        assert_eq!(
            &encode(Command::SetTrackingMode(TrackingMode::Multi))[6..8],
            [0x90, 0]
        );
        // §2.2.13: ignore zone 1 with corners (1000, 1000) and (-1000, 5000).
        let mut zones = [Zone::default(); 3];
        zones[0] = Zone {
            x1: 1_000,
            y1: 1_000,
            x2: -1_000,
            y2: 5_000,
        };
        let f = encode(Command::SetZoneFilter(ZoneFilter {
            mode: ZoneMode::IgnoreInside,
            zones,
        }));
        let mut want = std::vec![0xFD, 0xFC, 0xFB, 0xFA, 0x1C, 0x00, 0xC2, 0x00, 0x02, 0x00];
        want.extend([0xE8, 0x03, 0xE8, 0x03, 0x18, 0xFC, 0x88, 0x13]);
        want.extend([0; 16]);
        want.extend(CMD_TAIL);
        assert_eq!(f, want);
    }

    #[test]
    fn ack_decoders() {
        let mut zone = std::vec![0x01, 0x00, 0xE8, 0x03, 0xE8, 0x03, 0x18, 0xFC, 0x88, 0x13];
        zone.extend([0; 16]);
        let mut p = Parser::new();
        let mut stream = std::vec![0x55, 0xAA];
        stream.extend(ack(0xC1, &zone));
        stream.extend(ack(0x91, &[0x01, 0x00]));
        let frames: Vec<Frame> = stream.iter().filter_map(|&b| p.feed(b)).collect();
        let Frame::Ack(a) = frames[0] else { panic!() };
        let f = a.zone_filter().unwrap();
        assert_eq!(f.mode, ZoneMode::DetectInside);
        assert_eq!((f.zones[0].x2, f.zones[0].y2), (-1_000, 5_000));
        let Frame::Ack(a) = frames[1] else { panic!() };
        assert_eq!(a.tracking_mode(), Some(TrackingMode::Single));
    }

    #[test]
    fn blocking_poll_and_command() {
        let mut uart = FakeUart::default();
        uart.rx.extend(REPORT);
        let mut bad = REPORT;
        bad[29] = 0;
        uart.rx.extend(bad);
        let mut s = Ld2450::new(uart);
        assert_eq!(s.start_measurement(), Ok(0));
        assert_eq!(s.read_measurement().unwrap().count(), 1);
        assert_eq!(s.read_measurement(), Err(Error::NotReady));

        s.uart.rx.extend(REPORT);
        s.uart.rx.extend(ack(0x91, &[0x02, 0x00]));
        assert_eq!(s.tracking_mode(), Ok(TrackingMode::Multi));
        assert_eq!(s.uart.tx, encode(Command::QueryTrackingMode));

        s.send(&Command::EndConfig).unwrap();
        assert_eq!(s.poll_ack(&Command::EndConfig), Err(Error::NotReady));
        s.uart.rx.extend(ack(0xFE, &[]));
        assert!(s.poll_ack(&Command::EndConfig).is_ok());
    }

    #[test]
    fn async_report() {
        use zigbee_sensor::asynch::Sensor as _;
        let mut uart = FakeUart::default();
        uart.rx.extend(ack(0xFF, &[0x01, 0x00, 0x40, 0x00]));
        uart.rx.extend(REPORT);
        let mut s = asynch::Ld2450::new(uart);
        assert_eq!(block_on(s.enable_config()), Ok((1, 0x40)));
        assert_eq!(
            block_on(s.read_measurement()).unwrap().nearest_mm(),
            Some(1_883)
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
