//! Iclegend S3KM1110 24 GHz human-presence radar SoC, as used on the
//! Ai-Thinker Rd-03 and Waveshare HMMD modules (UART protocol of the
//! module firmware).
//!
//! References:
//! * Ai-Thinker "Rd-03 Serial communication protocol" V1.0.1 (2024-11-28)
//! * Waveshare "HMMD mmWave Sensor" wiki (firmware-version / serial-number
//!   commands)
//!
//! No Iclegend-authored protocol document is public; both module vendors
//! publish the same command set.
//!
//! UART 115200 8N1; words little-endian.
//!
//! * After power-up the module is in *normal* mode and prints text lines;
//!   [`S3km1110::enable_report_mode`] switches to binary *report* frames
//!   `F4 F3 F2 F1, len, detected, distance cm, 16 × gate energy,
//!   F8 F7 F6 F5` (every 100 ms). *Debug* mode (raw range-Doppler maps)
//!   is not decoded.
//! * Commands use `FD FC FB FA, len, cmd, value…, 04 03 02 01`; ACKs echo
//!   `cmd | 0x0100` plus a status word. Commands must be bracketed by
//!   [`Command::Open`] / [`Command::Close`]; the vendor recommends sending
//!   `Open` again if the first one is not acknowledged (the module may be
//!   flushing output).
//!
//! `read_measurement` returns the newest report (blocking: drains received
//! bytes, [`Error::NotReady`] if none; async: awaits the next report).
//! Command helpers read until the matching ACK and block/await on a silent
//! UART; use [`S3km1110::send`] + [`S3km1110::poll_ack`] for a bounded
//! wait.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// UART baud rate.
pub const BAUD_RATE: u32 = 115_200;
/// Number of distance gates.
pub const GATES: usize = 16;
/// Gate length, mm.
pub const GATE_MM: u32 = 700;

const REPORT_HEADER: [u8; 4] = [0xF4, 0xF3, 0xF2, 0xF1];
const REPORT_TAIL: [u8; 4] = [0xF8, 0xF7, 0xF6, 0xF5];
const CMD_HEADER: [u8; 4] = [0xFD, 0xFC, 0xFB, 0xFA];
const CMD_TAIL: [u8; 4] = [0x04, 0x03, 0x02, 0x01];
const BUF_LEN: usize = 64;
/// Longest encoded command frame.
pub const MAX_COMMAND_LEN: usize = 26;
const ACK_BYTE_BUDGET: usize = 1_024;
/// Parameter word that makes a write persistent ("ABD", value 0x64).
const PERSIST_WORD: u16 = 0x002F;
const PERSIST_VALUE: u32 = 0x64;

/// One report-mode frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Report {
    /// Target present.
    pub detected: bool,
    /// Target distance, cm.
    pub distance_cm: u16,
    /// Energy per gate (gate 0 = nearest).
    pub energy: [u16; GATES],
}

impl Report {
    /// Decode a report payload (between the length field and the tail).
    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() != 3 + 2 * GATES || payload[0] > 1 {
            return None;
        }
        let mut energy = [0u16; GATES];
        for (e, b) in energy.iter_mut().zip(payload[3..].chunks_exact(2)) {
            *e = u16::from_le_bytes([b[0], b[1]]);
        }
        Some(Self {
            detected: payload[0] == 1,
            distance_cm: u16::from_le_bytes([payload[1], payload[2]]),
            energy,
        })
    }
}

impl Readings for Report {
    /// `Occupancy`, plus `Distance` (mm) when a target is present.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Occupancy(self.detected));
        if self.detected {
            f(Quantity::Distance(u32::from(self.distance_cm) * 10));
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

    /// Open ACK: (protocol version, buffer size).
    pub fn config_info(&self) -> Option<(u16, u16)> {
        Some((self.word(0)?, self.word(2)?))
    }

    /// Read-parameter ACK: first 32-bit value.
    pub fn param_value(&self) -> Option<u32> {
        let d = self.data().get(..4)?;
        Some(u32::from_le_bytes([d[0], d[1], d[2], d[3]]))
    }

    /// Auto-threshold progress ACK, percent.
    pub fn progress(&self) -> Option<u16> {
        self.word(0)
    }

    /// Length-prefixed byte string (firmware version text, serial number).
    pub fn string(&self) -> Option<&[u8]> {
        let n = usize::from(self.word(0)?);
        self.data().get(2..2 + n)
    }
}

/// Configuration parameters (command 0x0007 / 0x0008).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Param {
    /// Nearest detection gate 0–15 (0x0000).
    MinGate,
    /// Farthest detection gate 0–15 (0x0001).
    MaxGate,
    /// Target-disappearance delay, s (0x0004).
    DisappearDelay,
    /// Trigger threshold of a gate 0–15 (0x0010 + gate), linear power
    /// ratio: `10^(dB / 10)`.
    TriggerThreshold(u8),
    /// Hold threshold of a gate 0–14 (0x0020 + gate). Gate 15 is not
    /// addressable: 0x002F is the persistence word.
    HoldThreshold(u8),
}

impl Param {
    /// Parameter word, `None` if the gate is out of range.
    pub const fn word(self) -> Option<u16> {
        match self {
            Self::MinGate => Some(0x0000),
            Self::MaxGate => Some(0x0001),
            Self::DisappearDelay => Some(0x0004),
            Self::TriggerThreshold(g) if (g as usize) < GATES => Some(0x0010 + g as u16),
            Self::HoldThreshold(g) if (g as usize) < GATES - 1 => Some(0x0020 + g as u16),
            _ => None,
        }
    }
}

/// System mode (command 0x0012).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Mode {
    /// Raw range-Doppler output (not decoded by this crate).
    Debug = 0x00,
    /// Binary report frames.
    Report = 0x04,
    /// Text output (power-up default).
    Normal = 0x64,
}

/// Commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    /// Open command mode (0x00FF); detection pauses.
    Open,
    /// Close command mode (0x00FE).
    Close,
    /// Write a parameter (0x0007); `persist` also stores it in flash.
    WriteParam {
        /// Parameter.
        param: Param,
        /// Value.
        value: u32,
        /// Survive power cycles.
        persist: bool,
    },
    /// Read a parameter (0x0008).
    ReadParam(Param),
    /// Generate per-gate thresholds from the current (empty-room) noise
    /// (0x0009); coefficients ×10, 10–200 (1.0–20.0).
    AutoThreshold {
        /// Trigger coefficient ×10.
        trigger_x10: u16,
        /// Hold coefficient ×10.
        hold_x10: u16,
    },
    /// Auto-threshold progress (0x000A).
    AutoThresholdProgress,
    /// Select output mode (0x0012).
    SetMode(Mode),
    /// Firmware version string (0x0000).
    ReadFirmwareVersion,
    /// Serial number (0x0011).
    ReadSerialNumber,
}

fn param(buf: &mut [u8], word: u16, value: u32) -> usize {
    buf[..2].copy_from_slice(&word.to_le_bytes());
    buf[2..6].copy_from_slice(&value.to_le_bytes());
    6
}

impl Command {
    /// Command word.
    pub const fn code(&self) -> u16 {
        match self {
            Self::Open => 0x00FF,
            Self::Close => 0x00FE,
            Self::WriteParam { .. } => 0x0007,
            Self::ReadParam(_) => 0x0008,
            Self::AutoThreshold { .. } => 0x0009,
            Self::AutoThresholdProgress => 0x000A,
            Self::SetMode(_) => 0x0012,
            Self::ReadFirmwareVersion => 0x0000,
            Self::ReadSerialNumber => 0x0011,
        }
    }

    fn value(&self, v: &mut [u8]) -> Option<usize> {
        let word = |v: &mut [u8], i: usize, w: u16| v[i..i + 2].copy_from_slice(&w.to_le_bytes());
        Some(match *self {
            Self::Open => {
                word(v, 0, 1);
                2
            }
            Self::WriteParam {
                param: p,
                value,
                persist,
            } => {
                let n = param(v, p.word()?, value);
                if persist {
                    n + param(&mut v[n..], PERSIST_WORD, PERSIST_VALUE)
                } else {
                    n
                }
            }
            Self::ReadParam(p) => {
                word(v, 0, p.word()?);
                2
            }
            Self::AutoThreshold {
                trigger_x10,
                hold_x10,
            } => {
                word(v, 0, trigger_x10);
                word(v, 2, hold_x10);
                4
            }
            Self::SetMode(m) => param(v, 0x0000, m as u32),
            _ => 0,
        })
    }

    /// Encode the full command frame; returns its length, or `None` for an
    /// out-of-range gate.
    pub fn encode(&self, out: &mut [u8; MAX_COMMAND_LEN]) -> Option<usize> {
        out[..4].copy_from_slice(&CMD_HEADER);
        out[6..8].copy_from_slice(&self.code().to_le_bytes());
        let n = 2 + self.value(&mut out[8..])?;
        out[4..6].copy_from_slice(&(n as u16).to_le_bytes());
        out[6 + n..10 + n].copy_from_slice(&CMD_TAIL);
        Some(10 + n)
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

/// Incremental frame parser for both report and ACK frames.
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
        let total = 10 + usize::from(u16::from_le_bytes([b[4], b[5]]));
        if total > BUF_LEN {
            return Check::Bad;
        }
        if b.len() < total {
            return Check::More;
        }
        let tail = if b[..4] == REPORT_HEADER {
            REPORT_TAIL
        } else {
            CMD_TAIL
        };
        if b[total - 4..total] == tail {
            Check::Done(total)
        } else {
            Check::Bad
        }
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
                    let payload = &self.buf[6..total - 4];
                    let frame = if self.buf[0] == REPORT_HEADER[0] {
                        Report::decode(payload).map(Frame::Report)
                    } else {
                        Ack::decode(payload).map(Frame::Ack)
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

        /// Open command mode; returns (protocol version, buffer size).
        pub $($asyncness)? fn open(&mut self) -> Result<(u16, u16), Error<U::Error>> {
            let ack = self.command(&Command::Open) $($aw)* ?;
            ack.config_info().ok_or(Error::InvalidData)
        }

        /// Close command mode; detection resumes.
        pub $($asyncness)? fn close(&mut self) -> Result<(), Error<U::Error>> {
            self.command(&Command::Close) $($aw)* .map(drop)
        }

        /// Open, select report mode, close. Call once after power-up.
        pub $($asyncness)? fn enable_report_mode(&mut self) -> Result<(), Error<U::Error>> {
            self.open() $($aw)* ?;
            self.command(&Command::SetMode(Mode::Report)) $($aw)* ?;
            self.close() $($aw)*
        }

        /// Write a parameter (inside command mode).
        pub $($asyncness)? fn write_param(
            &mut self,
            param: Param,
            value: u32,
            persist: bool,
        ) -> Result<(), Error<U::Error>> {
            let cmd = Command::WriteParam { param, value, persist };
            self.command(&cmd) $($aw)* .map(drop)
        }

        /// Read a parameter (inside command mode).
        pub $($asyncness)? fn read_param(&mut self, param: Param) -> Result<u32, Error<U::Error>> {
            let ack = self.command(&Command::ReadParam(param)) $($aw)* ?;
            ack.param_value().ok_or(Error::InvalidData)
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

        /// Write a command frame without waiting for the ACK;
        /// `InvalidData` for an out-of-range gate.
        pub $($asyncness)? fn send(&mut self, cmd: &Command) -> Result<(), Error<U::Error>> {
            let mut frame = [0u8; MAX_COMMAND_LEN];
            let n = cmd.encode(&mut frame).ok_or(Error::InvalidData)?;
            self.uart.write_all(&frame[..n]) $($aw)* .map_err(Error::bus)?;
            self.uart.flush() $($aw)* .map_err(Error::bus)
        }
    };
}

/// Blocking driver over `embedded_io::Read + ReadReady + Write`.
pub struct S3km1110<U> {
    uart: U,
    parser: Parser,
}

impl<U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write> S3km1110<U> {
    api!(;);

    /// Drain received bytes (never blocks); returns the newest report, or
    /// the ACK for `command` if `Some`. `NotReady` if neither arrived.
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

    /// Newest report received so far, or `NotReady`.
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
    zigbee_sensor::blocking::Sensor for S3km1110<U>
{
    type BusError = U::Error;
    type Measurement = Report;

    /// Free-running: returns `0`.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        Ok(0)
    }

    /// See [`S3km1110::poll`].
    fn read_measurement(&mut self) -> Result<Report, Error<U::Error>> {
        self.poll()
    }
}

/// Async driver over `embedded_io_async::Read + Write`.
pub mod asynch {
    use super::{
        ACK_BYTE_BUDGET, Ack, Command, Frame, MAX_COMMAND_LEN, Mode, Param, Parser, Report,
        check_status,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::S3km1110`].
    pub struct S3km1110<U> {
        uart: U,
        parser: Parser,
    }

    impl<U: embedded_io_async::Read + embedded_io_async::Write> S3km1110<U> {
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
        for S3km1110<U>
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
        let n = cmd.encode(&mut buf).unwrap();
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

    fn report(detected: u8, cm: u16) -> Vec<u8> {
        let mut v = std::vec![0xF4, 0xF3, 0xF2, 0xF1, 0x23, 0x00, detected];
        v.extend(cm.to_le_bytes());
        for g in 0..16u16 {
            v.extend((g * 100).to_le_bytes());
        }
        v.extend(REPORT_TAIL);
        v
    }

    #[test]
    fn command_encoding() {
        // HMMD wiki / Rd-03 protocol examples.
        assert_eq!(
            encode(Command::SetMode(Mode::Report)),
            [
                0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
                0x04, 0x03, 0x02, 0x01
            ]
        );
        assert_eq!(
            &encode(Command::SetMode(Mode::Normal))[10..14],
            [0x64, 0, 0, 0]
        );
        assert_eq!(
            encode(Command::ReadParam(Param::MaxGate)),
            [
                0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x08, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01
            ]
        );
        assert_eq!(
            encode(Command::ReadFirmwareVersion),
            [
                0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01
            ]
        );
        assert_eq!(&encode(Command::Open)[4..10], [4, 0, 0xFF, 0, 1, 0]);
        // Trigger threshold of gate 2 = 47.47 dB -> 59429, persisted.
        let w = encode(Command::WriteParam {
            param: Param::TriggerThreshold(2),
            value: 59_429,
            persist: true,
        });
        assert_eq!(
            &w[4..22],
            [
                0x0E, 0, 0x07, 0, 0x12, 0, 0x25, 0xE8, 0, 0, 0x2F, 0, 0x64, 0, 0, 0, 4, 3
            ]
        );
        assert_eq!(
            &encode(Command::AutoThreshold {
                trigger_x10: 30,
                hold_x10: 25
            })[8..12],
            [30, 0, 25, 0]
        );
        let mut buf = [0; MAX_COMMAND_LEN];
        assert_eq!(
            Command::ReadParam(Param::HoldThreshold(15)).encode(&mut buf),
            None
        );
        assert_eq!(
            Command::ReadParam(Param::TriggerThreshold(16)).encode(&mut buf),
            None
        );
    }

    #[test]
    fn ack_and_report_decoding() {
        let mut p = Parser::new();
        let mut stream = std::vec![b'O', b'N', b'\r', b'\n'];
        stream.extend([
            0xFD, 0xFC, 0xFB, 0xFA, 0x0C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x06, 0x00,
        ]);
        stream.extend(*b"v1.5.5");
        stream.extend(CMD_TAIL);
        stream.extend([
            0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x08, 0x01, 0x00, 0x00, 0x0C, 0, 0, 0,
        ]);
        stream.extend(CMD_TAIL);
        stream.extend(report(1, 250));
        let frames: Vec<Frame> = stream.iter().filter_map(|&b| p.feed(b)).collect();
        assert_eq!(p.errors(), 0);
        let Frame::Ack(fw) = frames[0] else { panic!() };
        assert_eq!((fw.command, fw.string()), (0x0000, Some(&b"v1.5.5"[..])));
        let Frame::Ack(gate) = frames[1] else {
            panic!()
        };
        assert_eq!(gate.param_value(), Some(12));
        let Frame::Report(r) = frames[2] else {
            panic!()
        };
        assert!(r.detected);
        assert_eq!(r.energy[15], 1_500);
        let mut q = Vec::new();
        r.for_each(&mut |x| q.push(x));
        assert_eq!(q, [Quantity::Occupancy(true), Quantity::Distance(2_500)]);
    }

    #[test]
    fn blocking_report_mode_and_poll() {
        let mut uart = FakeUart::default();
        uart.rx.extend(ack(0xFF, &[0x02, 0x00, 0x20, 0x00]));
        uart.rx.extend(ack(0x12, &[]));
        uart.rx.extend(ack(0xFE, &[]));
        let mut s = S3km1110::new(uart);
        s.enable_report_mode().unwrap();
        let mut want = encode(Command::Open);
        want.extend(encode(Command::SetMode(Mode::Report)));
        want.extend(encode(Command::Close));
        assert_eq!(s.uart.tx, want);

        assert_eq!(s.start_measurement(), Ok(0));
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        s.uart.rx.extend(report(1, 100));
        s.uart.rx.extend(report(0, 0));
        assert!(!s.read_measurement().unwrap().detected);

        s.uart
            .rx
            .extend([0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x07, 0x01, 0x01, 0x00]);
        s.uart.rx.extend(CMD_TAIL);
        assert_eq!(
            s.write_param(Param::MaxGate, 12, false),
            Err(Error::InvalidData)
        );
        assert_eq!(
            s.read_param(Param::HoldThreshold(15)),
            Err(Error::InvalidData)
        );
    }

    #[test]
    fn async_read() {
        use zigbee_sensor::asynch::Sensor as _;
        let mut uart = FakeUart::default();
        uart.rx.extend(ack(0x08, &[0x0C, 0, 0, 0]));
        uart.rx.extend(report(1, 70));
        let mut s = asynch::S3km1110::new(uart);
        assert_eq!(block_on(s.read_param(Param::MaxGate)), Ok(12));
        assert_eq!(block_on(s.read_measurement()).unwrap().distance_cm, 70);
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
