//! Hi-Link HLK-LD2410, LD2410B and LD2410C 24 GHz human-presence radars,
//! UART protocol.
//!
//! References (Shenzhen Hi-Link):
//! * "HLK-LD2410 Serial communication protocol" V1.02 (2022-07-01)
//! * "HLK-LD2410B Serial communication protocol" V1.06 (2023-02-21)
//! * "HLK-LD2410C Serial communication protocol" V1.00 (2022-11-07) and
//!   V1.07 (2024-08-05)
//!
//! UART 256000 8N1 by default (see [`Baud`]); all words little-endian.
//!
//! * Report frames `F4 F3 F2 F1, len, type, 0xAA, target…, 0x55, 0x00,
//!   F8 F7 F6 F5` stream continuously (~10 Hz) outside configuration mode;
//!   type `0x02` = basic, `0x01` = engineering (per-gate energies).
//! * Command frames `FD FC FB FA, len, cmd, value…, 04 03 02 01`; the ACK
//!   echoes `cmd | 0x0100` followed by a status word (0 = success).
//!   Every command except [`Command::EnableConfig`] must be sent between
//!   [`Command::EnableConfig`] and [`Command::EndConfig`]; reports stop
//!   while configuration mode is active.
//!
//! The pure layer ([`Parser`], [`Command::encode`], [`Report`], [`Ack`]
//! decoders) has no I/O. The drivers add `embedded-io` transport:
//! `read_measurement` returns the newest report (blocking: drains the bytes
//! already received and returns [`Error::NotReady`] if none; async: awaits
//! the next report). Command helpers write a frame and read until the
//! matching ACK; they block/await on the UART, so a silent module stalls
//! them — use [`Ld2410::send`] + [`Ld2410::poll_ack`] for a bounded wait.
//!
//! B/C-only commands (Bluetooth, MAC, distance resolution, light control,
//! noise detection) are rejected by the base LD2410.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Factory-default baud rate.
pub const DEFAULT_BAUD_RATE: u32 = 256_000;
/// Maximum distance-gate index (gates 0..=8).
pub const MAX_GATE: u8 = 8;

const REPORT_HEADER: [u8; 4] = [0xF4, 0xF3, 0xF2, 0xF1];
const REPORT_TAIL: [u8; 4] = [0xF8, 0xF7, 0xF6, 0xF5];
const CMD_HEADER: [u8; 4] = [0xFD, 0xFC, 0xFB, 0xFA];
const CMD_TAIL: [u8; 4] = [0x04, 0x03, 0x02, 0x01];
const BUF_LEN: usize = 64;
/// Longest encoded command frame.
pub const MAX_COMMAND_LEN: usize = 30;
/// Bytes read while waiting for an ACK before giving up with `Timeout`.
const ACK_BYTE_BUDGET: usize = 1_024;

/// Target state of a report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetState {
    /// No target.
    None,
    /// Moving target.
    Moving,
    /// Stationary target.
    Stationary,
    /// Moving and stationary targets.
    Both,
    /// Background-noise detection running (C V1.07+ / B V1.07+).
    NoiseDetecting,
    /// Background-noise detection succeeded.
    NoiseDetectionOk,
    /// Background-noise detection failed.
    NoiseDetectionFailed,
    /// Undocumented value.
    Unknown(u8),
}

impl TargetState {
    /// Decode the state byte.
    pub const fn from_byte(b: u8) -> Self {
        match b {
            0 => Self::None,
            1 => Self::Moving,
            2 => Self::Stationary,
            3 => Self::Both,
            4 => Self::NoiseDetecting,
            5 => Self::NoiseDetectionOk,
            6 => Self::NoiseDetectionFailed,
            x => Self::Unknown(x),
        }
    }

    /// A person is present (moving and/or stationary target).
    pub const fn occupied(self) -> bool {
        matches!(self, Self::Moving | Self::Stationary | Self::Both)
    }
}

/// Engineering-mode extras (per-gate energies).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Engineering {
    /// Highest moving gate reported (N; energies for gates 0..=N).
    pub max_moving_gate: u8,
    /// Highest stationary gate reported.
    pub max_static_gate: u8,
    /// Moving energy per gate (0–100); entries past `max_moving_gate` are 0.
    pub moving_energy: [u8; 9],
    /// Stationary energy per gate (0–100).
    pub static_energy: [u8; 9],
    /// Trailing bytes. LD2410B (and LD2410C V1.07 firmware) send the
    /// light-sensor value (0–255) and the OUT pin state (0/1); the LD2410
    /// and LD2410C V1.00 documents mark them reserved.
    pub aux: [u8; 2],
}

/// One report frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Report {
    /// Target state.
    pub state: TargetState,
    /// Moving target distance, cm.
    pub moving_distance_cm: u16,
    /// Moving target energy (0–100).
    pub moving_energy: u8,
    /// Stationary target distance, cm.
    pub static_distance_cm: u16,
    /// Stationary target energy (0–100).
    pub static_energy: u8,
    /// Detection distance, cm.
    pub detection_distance_cm: u16,
    /// Present in engineering-mode frames.
    pub engineering: Option<Engineering>,
}

impl Report {
    /// Distance to the target in cm: the detection distance, or the
    /// moving/stationary distance when that field is 0. `None` when
    /// unoccupied or no distance is reported.
    pub fn distance_cm(&self) -> Option<u16> {
        if !self.state.occupied() {
            return None;
        }
        if self.detection_distance_cm != 0 {
            return Some(self.detection_distance_cm);
        }
        let cm = match self.state {
            TargetState::Stationary => self.static_distance_cm,
            _ => self.moving_distance_cm,
        };
        (cm != 0).then_some(cm)
    }

    /// Decode a report payload (between the length field and the tail).
    pub fn decode(payload: &[u8]) -> Option<Self> {
        let n = payload.len();
        if n < 13 || payload[1] != 0xAA || payload[n - 2] != 0x55 || payload[n - 1] != 0x00 {
            return None;
        }
        let w = |i: usize| u16::from_le_bytes([payload[i], payload[i + 1]]);
        let engineering = match payload[0] {
            0x02 if n == 13 => None,
            0x01 => {
                let body = &payload[11..n - 2];
                let (nm, ns) = (*body.first()?, *body.get(1)?);
                if nm > MAX_GATE || ns > MAX_GATE {
                    return None;
                }
                let (nm, ns) = (usize::from(nm) + 1, usize::from(ns) + 1);
                let rest = body.get(2..)?;
                if rest.len() < nm + ns {
                    return None;
                }
                let mut e = Engineering {
                    max_moving_gate: body[0],
                    max_static_gate: body[1],
                    moving_energy: [0; 9],
                    static_energy: [0; 9],
                    aux: [0; 2],
                };
                e.moving_energy[..nm].copy_from_slice(&rest[..nm]);
                e.static_energy[..ns].copy_from_slice(&rest[nm..nm + ns]);
                let aux = &rest[nm + ns..];
                let k = aux.len().min(2);
                e.aux[..k].copy_from_slice(&aux[..k]);
                Some(e)
            }
            _ => return None,
        };
        Some(Self {
            state: TargetState::from_byte(payload[2]),
            moving_distance_cm: w(3),
            moving_energy: payload[5],
            static_distance_cm: w(6),
            static_energy: payload[8],
            detection_distance_cm: w(9),
            engineering,
        })
    }
}

impl Readings for Report {
    /// `Occupancy`, plus `Distance` (mm) when occupied.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Occupancy(self.state.occupied()));
        if let Some(cm) = self.distance_cm() {
            f(Quantity::Distance(u32::from(cm) * 10));
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

    /// Read-parameters ACK.
    pub fn parameters(&self) -> Option<Parameters> {
        let d = self.data();
        if d.first() != Some(&0xAA) || d.len() < 4 {
            return None;
        }
        let n = usize::from(d[1]);
        if n > usize::from(MAX_GATE) || d.len() < 4 + 2 * (n + 1) + 2 {
            return None;
        }
        let mut p = Parameters {
            max_gate: d[1],
            max_moving_gate: d[2],
            max_static_gate: d[3],
            moving_sensitivity: [0; 9],
            static_sensitivity: [0; 9],
            unmanned_duration_s: 0,
        };
        p.moving_sensitivity[..=n].copy_from_slice(&d[4..5 + n]);
        p.static_sensitivity[..=n].copy_from_slice(&d[5 + n..6 + 2 * n]);
        p.unmanned_duration_s = self.word(6 + 2 * n)?;
        Some(p)
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

    /// Single-word ACK (distance resolution, noise-detection status).
    pub fn value(&self) -> Option<u16> {
        self.word(0)
    }

    /// MAC-address ACK (6 bytes as in the documents' examples).
    pub fn mac(&self) -> Option<[u8; 6]> {
        self.data().get(..6)?.try_into().ok()
    }

    /// Light-control query ACK.
    pub fn light_control(&self) -> Option<LightControl> {
        let d = self.data();
        if d.len() < 3 {
            return None;
        }
        Some(LightControl {
            mode: match d[0] {
                1 => LightMode::BelowThreshold,
                2 => LightMode::AboveThreshold,
                _ => LightMode::Off,
            },
            threshold: d[1],
            out_default_high: d[2] != 0,
        })
    }
}

/// Configuration read back with [`Command::ReadParameters`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Parameters {
    /// Number of the highest gate supported (8).
    pub max_gate: u8,
    /// Configured farthest moving gate.
    pub max_moving_gate: u8,
    /// Configured farthest stationary gate.
    pub max_static_gate: u8,
    /// Moving sensitivity per gate (0–100).
    pub moving_sensitivity: [u8; 9],
    /// Stationary sensitivity per gate (gates 0–1 not settable).
    pub static_sensitivity: [u8; 9],
    /// Delay before reporting "no target", s.
    pub unmanned_duration_s: u16,
}

/// Firmware version; printed as `V{major}.{minor:02}.{build:08x}`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareVersion {
    /// Firmware type word, little-endian (bytes `00 00` on LD2410 and
    /// `00 01` = 0x0100 on LD2410B in the documents' examples).
    pub kind: u16,
    /// Major version.
    pub major: u8,
    /// Minor version.
    pub minor: u8,
    /// Build, BCD date/time `YYMMDDHH`.
    pub build: u32,
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

impl Baud {
    /// Bits per second.
    pub const fn bps(self) -> u32 {
        match self {
            Self::B9600 => 9_600,
            Self::B19200 => 19_200,
            Self::B38400 => 38_400,
            Self::B57600 => 57_600,
            Self::B115200 => 115_200,
            Self::B230400 => 230_400,
            Self::B256000 => 256_000,
            Self::B460800 => 460_800,
        }
    }
}

/// Gate length (LD2410B/C).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum Resolution {
    /// 0.75 m per gate (index 0).
    Cm75 = 0,
    /// 0.2 m per gate (index 1).
    Cm20 = 1,
}

/// Light-sensor condition for the OUT pin (LD2410B, LD2410C V1.06+).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LightMode {
    /// Light sensor not used.
    Off,
    /// OUT triggers only when light < threshold.
    BelowThreshold,
    /// OUT triggers only when light > threshold.
    AboveThreshold,
}

/// Auxiliary (light) control of the OUT pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LightControl {
    /// Condition.
    pub mode: LightMode,
    /// Threshold 0–255 (default 0x80).
    pub threshold: u8,
    /// OUT idles high and goes low on trigger.
    pub out_default_high: bool,
}

/// Commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    /// Enter configuration mode (0x00FF).
    EnableConfig,
    /// Leave configuration mode (0x00FE).
    EndConfig,
    /// Farthest gates (2–8) and unmanned duration in s (0x0060).
    SetMaxGates {
        /// Farthest moving gate.
        moving: u8,
        /// Farthest stationary gate.
        stationary: u8,
        /// "No one" delay, s.
        unmanned_s: u16,
    },
    /// Read configuration (0x0061).
    ReadParameters,
    /// Engineering mode on/off (0x0062 / 0x0063); not persistent.
    EngineeringMode(bool),
    /// Gate sensitivities 0–100 (0x0064); `gate: None` = all gates.
    SetSensitivity {
        /// Gate 0–8, or all.
        gate: Option<u8>,
        /// Moving sensitivity.
        moving: u8,
        /// Stationary sensitivity.
        stationary: u8,
    },
    /// Read firmware version (0x00A0).
    ReadFirmwareVersion,
    /// Set baud rate (0x00A1); applies after restart.
    SetBaudRate(Baud),
    /// Factory reset (0x00A2); applies after restart.
    FactoryReset,
    /// Restart (0x00A3); the module restarts after the ACK.
    Restart,
    /// Bluetooth on/off (0x00A4, B/C); applies after restart.
    Bluetooth(bool),
    /// Read MAC address (0x00A5, B/C).
    ReadMac,
    /// Set gate length (0x00AA, B/C); applies after restart.
    SetResolution(Resolution),
    /// Read gate length (0x00AB, B/C).
    ReadResolution,
    /// Set light control (0x00AD, B, C V1.06+).
    SetLightControl(LightControl),
    /// Read light control (0x00AE).
    ReadLightControl,
    /// Background-noise detection and auto-sensitivity for `s` seconds
    /// (0x000B, B/C V1.07+); the room must be empty.
    StartNoiseDetection(u16),
    /// Noise-detection status: 0 idle, 1 running, 2 done (0x001B).
    QueryNoiseDetection,
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
            Self::EnableConfig => 0x00FF,
            Self::EndConfig => 0x00FE,
            Self::SetMaxGates { .. } => 0x0060,
            Self::ReadParameters => 0x0061,
            Self::EngineeringMode(true) => 0x0062,
            Self::EngineeringMode(false) => 0x0063,
            Self::SetSensitivity { .. } => 0x0064,
            Self::ReadFirmwareVersion => 0x00A0,
            Self::SetBaudRate(_) => 0x00A1,
            Self::FactoryReset => 0x00A2,
            Self::Restart => 0x00A3,
            Self::Bluetooth(_) => 0x00A4,
            Self::ReadMac => 0x00A5,
            Self::SetResolution(_) => 0x00AA,
            Self::ReadResolution => 0x00AB,
            Self::SetLightControl(_) => 0x00AD,
            Self::ReadLightControl => 0x00AE,
            Self::StartNoiseDetection(_) => 0x000B,
            Self::QueryNoiseDetection => 0x001B,
        }
    }

    fn value(&self, v: &mut [u8]) -> usize {
        let word = |v: &mut [u8], w: u16| {
            v[..2].copy_from_slice(&w.to_le_bytes());
            2
        };
        match *self {
            Self::EnableConfig | Self::ReadMac => word(v, 1),
            Self::SetMaxGates {
                moving,
                stationary,
                unmanned_s,
            } => {
                param(&mut v[0..], 0, moving.into())
                    + param(&mut v[6..], 1, stationary.into())
                    + param(&mut v[12..], 2, unmanned_s.into())
            }
            Self::SetSensitivity {
                gate,
                moving,
                stationary,
            } => {
                let g = gate.map_or(0xFFFF, u32::from);
                param(&mut v[0..], 0, g)
                    + param(&mut v[6..], 1, moving.into())
                    + param(&mut v[12..], 2, stationary.into())
            }
            Self::SetBaudRate(b) => word(v, b as u16),
            Self::Bluetooth(on) => word(v, if on { 0x0100 } else { 0 }),
            Self::SetResolution(r) => word(v, r as u16),
            Self::SetLightControl(l) => {
                v[0] = match l.mode {
                    LightMode::Off => 0,
                    LightMode::BelowThreshold => 1,
                    LightMode::AboveThreshold => 2,
                };
                v[1] = l.threshold;
                v[2] = u8::from(l.out_default_high);
                v[3] = 0;
                4
            }
            Self::StartNoiseDetection(s) => word(v, s),
            _ => 0,
        }
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

        /// Enter configuration mode; returns (protocol version, buffer size).
        pub $($asyncness)? fn enable_config(&mut self) -> Result<(u16, u16), Error<U::Error>> {
            let ack = self.command(&Command::EnableConfig) $($aw)* ?;
            ack.config_info().ok_or(Error::InvalidData)
        }

        /// Leave configuration mode; reporting resumes.
        pub $($asyncness)? fn end_config(&mut self) -> Result<(), Error<U::Error>> {
            self.command(&Command::EndConfig) $($aw)* .map(drop)
        }

        /// Farthest moving/stationary gate (2–8) and unmanned duration (s).
        pub $($asyncness)? fn set_max_gates(
            &mut self,
            moving: u8,
            stationary: u8,
            unmanned_s: u16,
        ) -> Result<(), Error<U::Error>> {
            if !(2..=MAX_GATE).contains(&moving) || !(2..=MAX_GATE).contains(&stationary) {
                return Err(Error::InvalidData);
            }
            let cmd = Command::SetMaxGates { moving, stationary, unmanned_s };
            self.command(&cmd) $($aw)* .map(drop)
        }

        /// Sensitivities 0–100 for `gate` (0–8) or all gates (`None`).
        pub $($asyncness)? fn set_sensitivity(
            &mut self,
            gate: Option<u8>,
            moving: u8,
            stationary: u8,
        ) -> Result<(), Error<U::Error>> {
            if gate.is_some_and(|g| g > MAX_GATE) || moving > 100 || stationary > 100 {
                return Err(Error::InvalidData);
            }
            let cmd = Command::SetSensitivity { gate, moving, stationary };
            self.command(&cmd) $($aw)* .map(drop)
        }

        /// Read the configuration.
        pub $($asyncness)? fn read_parameters(&mut self) -> Result<Parameters, Error<U::Error>> {
            let ack = self.command(&Command::ReadParameters) $($aw)* ?;
            ack.parameters().ok_or(Error::InvalidData)
        }

        /// Engineering mode (per-gate energies in reports).
        pub $($asyncness)? fn set_engineering_mode(&mut self, on: bool) -> Result<(), Error<U::Error>> {
            self.command(&Command::EngineeringMode(on)) $($aw)* .map(drop)
        }

        /// Firmware version.
        pub $($asyncness)? fn firmware_version(&mut self) -> Result<FirmwareVersion, Error<U::Error>> {
            let ack = self.command(&Command::ReadFirmwareVersion) $($aw)* ?;
            ack.firmware_version().ok_or(Error::InvalidData)
        }

        /// Gate length (B/C).
        pub $($asyncness)? fn resolution(&mut self) -> Result<Resolution, Error<U::Error>> {
            let ack = self.command(&Command::ReadResolution) $($aw)* ?;
            match ack.value() {
                Some(0) => Ok(Resolution::Cm75),
                Some(1) => Ok(Resolution::Cm20),
                _ => Err(Error::InvalidData),
            }
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
pub struct Ld2410<U> {
    uart: U,
    parser: Parser,
}

impl<U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write> Ld2410<U> {
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
    zigbee_sensor::blocking::Sensor for Ld2410<U>
{
    type BusError = U::Error;
    type Measurement = Report;

    /// Free-running: returns `0`.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        Ok(0)
    }

    /// See [`Ld2410::poll`].
    fn read_measurement(&mut self) -> Result<Report, Error<U::Error>> {
        self.poll()
    }
}

/// Async driver over `embedded_io_async::Read + Write`.
pub mod asynch {
    use super::{
        ACK_BYTE_BUDGET, Ack, Command, FirmwareVersion, Frame, MAX_COMMAND_LEN, MAX_GATE,
        Parameters, Parser, Report, Resolution, check_status,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Ld2410`].
    pub struct Ld2410<U> {
        uart: U,
        parser: Parser,
    }

    impl<U: embedded_io_async::Read + embedded_io_async::Write> Ld2410<U> {
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
        for Ld2410<U>
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

    const BASIC: [u8; 23] = [
        0xF4, 0xF3, 0xF2, 0xF1, 0x0D, 0x00, 0x02, 0xAA, 0x02, 0x51, 0x00, 0x00, 0x00, 0x00, 0x3B,
        0x00, 0x00, 0x55, 0x00, 0xF8, 0xF7, 0xF6, 0xF5,
    ];
    const ENGINEERING: [u8; 45] = [
        0xF4, 0xF3, 0xF2, 0xF1, 0x23, 0x00, 0x01, 0xAA, 0x03, 0x1E, 0x00, 0x3C, 0x00, 0x00, 0x39,
        0x00, 0x00, 0x08, 0x08, 0x3C, 0x22, 0x05, 0x03, 0x03, 0x04, 0x03, 0x06, 0x05, 0x00, 0x00,
        0x39, 0x10, 0x13, 0x06, 0x06, 0x08, 0x04, 0x60, 0x01, 0x55, 0x00, 0xF8, 0xF7, 0xF6, 0xF5,
    ];
    const ACK_ENABLE: [u8; 18] = [
        0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00, 0x04,
        0x03, 0x02, 0x01,
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

    fn parse_all(bytes: &[u8]) -> (Vec<Frame>, u32) {
        let mut p = Parser::new();
        let frames = bytes.iter().filter_map(|&b| p.feed(b)).collect();
        (frames, p.errors())
    }

    fn encode(cmd: Command) -> Vec<u8> {
        let mut buf = [0; MAX_COMMAND_LEN];
        let n = cmd.encode(&mut buf);
        buf[..n].to_vec()
    }

    #[test]
    fn basic_report_example() {
        // Protocol §2.3 (all three documents).
        let (frames, errors) = parse_all(&BASIC);
        assert_eq!(errors, 0);
        let Frame::Report(r) = frames[0] else {
            panic!()
        };
        assert_eq!(r.state, TargetState::Stationary);
        assert_eq!((r.moving_distance_cm, r.moving_energy), (81, 0));
        assert_eq!((r.static_distance_cm, r.static_energy), (0, 59));
        assert_eq!(r.engineering, None);
        let mut q = Vec::new();
        r.for_each(&mut |x| q.push(x));
        assert_eq!(q, [Quantity::Occupancy(true)]);
    }

    #[test]
    fn engineering_report_example() {
        // LD2410B V1.06 §2.3.2 example.
        let (frames, _) = parse_all(&ENGINEERING);
        let Frame::Report(r) = frames[0] else {
            panic!()
        };
        assert_eq!(r.state, TargetState::Both);
        assert_eq!(r.moving_distance_cm, 30);
        assert_eq!(r.distance_cm(), Some(30));
        let e = r.engineering.unwrap();
        assert_eq!((e.max_moving_gate, e.max_static_gate), (8, 8));
        assert_eq!(e.moving_energy, [60, 34, 5, 3, 3, 4, 3, 6, 5]);
        assert_eq!(e.static_energy, [0, 0, 57, 16, 19, 6, 6, 8, 4]);
        assert_eq!(e.aux, [0x60, 0x01]);
    }

    #[test]
    fn parser_resyncs_after_garbage_and_bad_tail() {
        let mut stream = std::vec![0xF4, 0xF3, 0x00, 0xFD];
        let mut bad = BASIC;
        bad[22] = 0;
        stream.extend_from_slice(&bad);
        stream.extend_from_slice(&ACK_ENABLE);
        stream.extend_from_slice(&BASIC);
        let (frames, errors) = parse_all(&stream);
        assert_eq!(frames.len(), 2);
        assert!(matches!(frames[0], Frame::Ack(a) if a.config_info() == Some((1, 0x40))));
        assert!(matches!(frames[1], Frame::Report(_)));
        assert_eq!(errors, 1);
    }

    #[test]
    fn command_encoding_examples() {
        // Protocol §2.2 send examples.
        assert_eq!(
            encode(Command::EnableConfig),
            [
                0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01
            ]
        );
        assert_eq!(
            encode(Command::SetMaxGates {
                moving: 8,
                stationary: 8,
                unmanned_s: 5
            }),
            [
                0xFD, 0xFC, 0xFB, 0xFA, 0x14, 0x00, 0x60, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
                0x01, 0x00, 0x08, 0x00, 0x00, 0x00, 0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x04, 0x03,
                0x02, 0x01
            ]
        );
        let all = encode(Command::SetSensitivity {
            gate: None,
            moving: 40,
            stationary: 40,
        });
        assert_eq!(
            &all[4..14],
            [0x14, 0x00, 0x64, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00]
        );
        assert_eq!(&all[14..26], [1, 0, 40, 0, 0, 0, 2, 0, 40, 0, 0, 0]);
        assert_eq!(
            &encode(Command::SetBaudRate(Baud::B256000))[4..10],
            [4, 0, 0xA1, 0, 7, 0]
        );
        assert_eq!(
            &encode(Command::Bluetooth(true))[6..10],
            [0xA4, 0, 0x00, 0x01]
        );
        assert_eq!(&encode(Command::EndConfig)[4..8], [2, 0, 0xFE, 0]);
        assert_eq!(
            &encode(Command::SetLightControl(LightControl {
                mode: LightMode::BelowThreshold,
                threshold: 0x60,
                out_default_high: false
            }))[4..12],
            [6, 0, 0xAD, 0, 1, 0x60, 0, 0]
        );
    }

    #[test]
    fn ack_decoders() {
        let mut params = std::vec![
            0xFD, 0xFC, 0xFB, 0xFA, 0x1C, 0x00, 0x61, 0x01, 0, 0, 0xAA, 8, 8, 8
        ];
        params.extend([0x14; 9]);
        params.extend([0x19; 9]);
        params.extend([0x05, 0x00, 0x04, 0x03, 0x02, 0x01]);
        let fw = [
            0xFD, 0xFC, 0xFB, 0xFA, 0x0C, 0x00, 0xA0, 0x01, 0x00, 0x00, 0x00, 0x01, 0x02, 0x01,
            0x16, 0x24, 0x06, 0x22, 0x04, 0x03, 0x02, 0x01,
        ];
        let mac = [
            0xFD, 0xFC, 0xFB, 0xFA, 0x0A, 0x00, 0xA5, 0x01, 0x00, 0x00, 0x8F, 0x27, 0x2E, 0xB8,
            0x0F, 0x65, 0x04, 0x03, 0x02, 0x01,
        ];
        params.extend(fw);
        params.extend(mac);
        let (frames, errors) = parse_all(&params);
        assert_eq!(errors, 0);
        let acks: Vec<Ack> = frames
            .into_iter()
            .map(|f| match f {
                Frame::Ack(a) => a,
                Frame::Report(_) => panic!(),
            })
            .collect();
        let p = acks[0].parameters().unwrap();
        assert_eq!(
            (p.max_gate, p.max_moving_gate, p.max_static_gate),
            (8, 8, 8)
        );
        assert_eq!(p.moving_sensitivity, [20; 9]);
        assert_eq!(p.static_sensitivity, [25; 9]);
        assert_eq!(p.unmanned_duration_s, 5);
        assert_eq!(
            acks[1].firmware_version(),
            Some(FirmwareVersion {
                kind: 0x0100,
                major: 1,
                minor: 2,
                build: 0x2206_2416
            })
        );
        assert_eq!(acks[2].command, 0x00A5);
        assert_eq!(acks[2].mac(), Some([0x8F, 0x27, 0x2E, 0xB8, 0x0F, 0x65]));
    }

    #[test]
    fn blocking_command_skips_reports_and_checks_status() {
        let mut uart = FakeUart::default();
        uart.rx.extend(BASIC);
        uart.rx.extend(ACK_ENABLE);
        let mut s = Ld2410::new(uart);
        assert_eq!(s.enable_config(), Ok((1, 0x40)));
        let uart = s.release();
        assert_eq!(uart.tx, encode(Command::EnableConfig));

        let mut uart = FakeUart::default();
        uart.rx.extend([
            0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x62, 0x01, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01,
        ]);
        let mut s = Ld2410::new(uart);
        assert_eq!(s.set_engineering_mode(true), Err(Error::InvalidData));
        assert_eq!(s.set_max_gates(1, 8, 5), Err(Error::InvalidData));
    }

    #[test]
    fn blocking_poll_and_poll_ack() {
        let mut uart = FakeUart::default();
        uart.rx.extend(BASIC);
        uart.rx.extend(ENGINEERING);
        let mut s = Ld2410::new(uart);
        assert_eq!(s.start_measurement(), Ok(0));
        let r = s.read_measurement().unwrap();
        assert!(r.engineering.is_some());
        assert_eq!(s.read_measurement(), Err(Error::NotReady));

        s.send(&Command::EnableConfig).unwrap();
        assert_eq!(s.poll_ack(&Command::EnableConfig), Err(Error::NotReady));
        s.uart.rx.extend(ACK_ENABLE);
        assert_eq!(s.poll_ack(&Command::EnableConfig).map(|a| a.status), Ok(0));
    }

    #[test]
    fn async_report_and_command() {
        use zigbee_sensor::asynch::Sensor as _;
        let mut uart = FakeUart::default();
        uart.rx.extend(ACK_ENABLE);
        uart.rx.extend(BASIC);
        let mut s = asynch::Ld2410::new(uart);
        assert_eq!(block_on(s.enable_config()), Ok((1, 0x40)));
        let r = block_on(s.read_measurement()).unwrap();
        assert_eq!(r.state, TargetState::Stationary);
        assert_eq!(s.release().tx, encode(Command::EnableConfig));
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
