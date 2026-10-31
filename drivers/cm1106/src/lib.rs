//! Cubic CM1106 / CM1106-C / CM1106SL-NS NDIR CO2 sensor modules.
//!
//! Three interfaces are covered:
//!
//! - **UART** ([`Cm1106`], all variants): 9600 8N1, request
//!   `11 LEN CMD DATA.. CS`, response `16 LEN CMD DATA.. CS` with
//!   `LEN = 1 + data length` and `CS = 256 − (sum of all previous bytes) % 256`.
//!   The sensor only answers requests (query mode, no active streaming):
//!   [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//!   discards stale input, sends `11 01 01 ED` and returns [`RESPONSE_US`];
//!   [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//!   parses the reply or returns [`Error::NotReady`].
//! - **CM1106-C I2C** ([`Cm1106C`], address `0x31`): the host writes
//!   `CMD DATA..` and then reads `CMD DATA.. CS` (`CS = −sum`); the sensor
//!   keeps returning the last command's reply until a new command arrives.
//! - **CM1106SL-NS I2C** ([`Cm1106Sl`], address `0x34`): a big-endian
//!   register map ([`reg`]) with single/continuous measurement modes.
//!
//! The CM1106SL-NS is switched on and off by the host through its EN pin and
//! signals "data ready" by pulling RDY low (≈ 730 ms after EN goes high);
//! neither the CM1106 nor the CM1106SL-NS has a sleep command, so no
//! [`LowPower`](zigbee_sensor::blocking::LowPower) implementation is
//! provided.
//!
//! References: Cubic "Single Beam NDIR CO2 Sensor Module CM1106-C
//! Specification" V0.6 (2022-12-22, UART + I2C protocol, same UART protocol
//! as CM1106); Cubic "Super Low Power NDIR CO2 Sensor Module CM1106SL-NS
//! Specification" V0.7 (2021-11-05).
//!
//! ```ignore
//! let mut co2 = Cm1106::new(uart);
//! let wait_us = co2.start_measurement()?;
//! // ... wait_us later:
//! let m = co2.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Start byte of host → sensor UART frames.
pub const HEAD_REQUEST: u8 = 0x11;
/// Start byte of sensor → host UART frames.
pub const HEAD_RESPONSE: u8 = 0x16;
/// Longest UART request (`set ABC`: 3 + 6 data + CS).
pub const MAX_COMMAND_LEN: usize = 10;
/// Longest UART response (`software version`: 3 + 11 data + CS).
pub const MAX_FRAME_LEN: usize = 15;
const MAX_LEN_BYTE: u8 = (MAX_FRAME_LEN - 3) as u8;

/// 7-bit I2C address of the CM1106-C.
pub const I2C_ADDRESS: u8 = 0x31;
/// Default 7-bit I2C address of the CM1106SL-NS (register [`reg::ADDRESS`]).
pub const SL_I2C_ADDRESS: u8 = 0x34;

/// UART reply allowance: a 15-byte frame takes 16 ms at 9600 Bd; the
/// datasheet gives no processing time, 100 ms is a conservative assumption.
pub const RESPONSE_US: WaitUs = 100_000;
/// CM1106-C I2C reply allowance (assumed; the datasheet only says the
/// sensor MCU is slow and asks for ~100 µs gaps around ACKs).
pub const I2C_RESPONSE_US: WaitUs = 20_000;
/// CM1106 / CM1106-C time to first reading (≤ 30 s; 550 ppm and the
/// preheating flag until then).
pub const WARM_UP_US: WaitUs = 30_000_000;
/// CM1106SL-NS cycle after EN goes high: 500 ms power-on, 100 ms lamp,
/// 100 ms calculation, 30 ms communication. RDY goes low when data is ready.
pub const SL_CYCLE_US: WaitUs = 730_000;
/// CM1106SL-NS EEPROM write time per register (< 25 ms); do not remove EN
/// power during the write.
pub const SL_EEPROM_WRITE_US: WaitUs = 25_000;

const MAX_READ_BYTES: usize = 256;
const MAX_DRAIN_BYTES: usize = 256;

/// UART and CM1106-C I2C command codes.
pub mod cmd {
    /// Read CO2 (all variants).
    pub const READ_CO2: u8 = 0x01;
    /// Calibrate to a known concentration, 400–1500 ppm.
    pub const CALIBRATE: u8 = 0x03;
    /// Read ABC parameters (CM1106SL-NS UART).
    pub const CHECK_ABC: u8 = 0x0F;
    /// Open/close ABC and set its parameters.
    pub const SET_ABC: u8 = 0x10;
    /// Store ABC data in single measurement mode (CM1106SL-NS UART).
    pub const STORE_ABC: u8 = 0x11;
    /// Read software version.
    pub const VERSION: u8 = 0x1E;
    /// Read serial number.
    pub const SERIAL_NUMBER: u8 = 0x1F;
    /// Set/check measurement period and smoothing (CM1106SL-NS UART).
    pub const PERIOD: u8 = 0x50;
    /// Set/check working mode (CM1106SL-NS UART).
    pub const WORKING_MODE: u8 = 0x51;
}

/// CM1106SL-NS I2C registers (16-bit values are big-endian, high byte at the
/// given address). "EE" registers are EEPROM-backed.
pub mod reg {
    /// Error status (bit 0 fatal, bit 5 out of range).
    pub const ERROR_STATUS: u8 = 0x00;
    /// CO2 in ppm (u16).
    pub const CO2: u8 = 0x06;
    /// Chip temperature in 0.01 °C (u16).
    pub const TEMPERATURE: u8 = 0x08;
    /// Measurement counter (u8, wraps).
    pub const MEASUREMENT_COUNT: u8 = 0x0D;
    /// Time in the current measurement cycle, units of 2 s (u8).
    pub const CYCLE_TIME: u8 = 0x0E;
    /// Calibration status (bit 3 ABC, 4 target, 5 background, 6 zero).
    pub const CALIBRATION_STATUS: u8 = 0x81;
    /// Calibration command (u16, see [`super::SlCalibration`]).
    pub const CALIBRATION_COMMAND: u8 = 0x82;
    /// Target for target calibration in ppm (u16).
    pub const CALIBRATION_TARGET: u8 = 0x84;
    /// CO2 override (u16, 32767 = none).
    pub const CO2_OVERRIDE: u8 = 0x86;
    /// Time since the last ABC calibration in half hours (u16).
    pub const ABC_TIME: u8 = 0x88;
    /// Write 1 to start a measurement in single measurement mode.
    pub const START_SINGLE: u8 = 0x93;
    /// Measurement mode (EE, u8: 0 continuous, 1 single = default).
    pub const MEASUREMENT_MODE: u8 = 0x95;
    /// Measurement period in s (EE, u16, 4–65534; continuous mode only).
    pub const MEASUREMENT_PERIOD: u8 = 0x96;
    /// ABC period in hours (EE, u16, 24–240, default 168).
    pub const ABC_PERIOD: u8 = 0x9A;
    /// ABC / background target in ppm (EE, u16, default 400).
    pub const ABC_TARGET: u8 = 0x9E;
    /// Meter control (EE, u8: bit 1 set = ABC disabled).
    pub const METER_CONTROL: u8 = 0xA5;
    /// I2C address (EE, u8, 1–0x7F).
    pub const ADDRESS: u8 = 0xA7;
}

/// Checksum used by the UART and CM1106-C I2C protocols: two's complement
/// of the byte sum.
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes
        .iter()
        .fold(0u8, |acc, b| acc.wrapping_add(*b))
        .wrapping_neg()
}

/// An encoded UART request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Command {
    buf: [u8; MAX_COMMAND_LEN],
    len: u8,
}

impl Command {
    /// Bytes to send.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..usize::from(self.len)]
    }

    /// Command code.
    pub const fn code(&self) -> u8 {
        self.buf[2]
    }
}

/// Encode `11 LEN code data.. CS`; `None` if `data` is longer than 6 bytes.
pub fn command(code: u8, data: &[u8]) -> Option<Command> {
    if data.len() > MAX_COMMAND_LEN - 4 {
        return None;
    }
    let mut buf = [0u8; MAX_COMMAND_LEN];
    let n = data.len() + 3;
    buf[0] = HEAD_REQUEST;
    buf[1] = data.len() as u8 + 1;
    buf[2] = code;
    buf[3..n].copy_from_slice(data);
    buf[n] = checksum(&buf[..n]);
    Some(Command {
        buf,
        len: n as u8 + 1,
    })
}

fn fixed(code: u8, data: &[u8]) -> Command {
    command(code, data).unwrap_or(Command {
        buf: [0; MAX_COMMAND_LEN],
        len: 0,
    })
}

/// `11 01 01 ED`: read CO2.
pub fn read_co2_command() -> Command {
    fixed(cmd::READ_CO2, &[])
}

/// `11 01 1F CF`: read the serial number.
pub fn serial_number_command() -> Command {
    fixed(cmd::SERIAL_NUMBER, &[])
}

/// `11 01 1E D0`: read the software version.
pub fn version_command() -> Command {
    fixed(cmd::VERSION, &[])
}

/// `11 01 0F DF`: read ABC parameters (CM1106SL-NS).
pub fn check_abc_command() -> Command {
    fixed(cmd::CHECK_ABC, &[])
}

/// `11 01 11 DD`: store ABC data (CM1106SL-NS single measurement mode; send
/// every 30 min, or after each measurement if powered less often).
pub fn store_abc_command() -> Command {
    fixed(cmd::STORE_ABC, &[])
}

/// `11 01 50 9E`: read measurement period and smoothing (CM1106SL-NS).
pub fn check_period_command() -> Command {
    fixed(cmd::PERIOD, &[])
}

/// `11 01 51 9D`: read the working mode (CM1106SL-NS).
pub fn check_working_mode_command() -> Command {
    fixed(cmd::WORKING_MODE, &[])
}

fn valid_target(ppm: u16) -> bool {
    (400..=1500).contains(&ppm)
}

/// Calibrate to `ppm` (400–1500; the gas must be stable for 2 min, and on
/// CM1106SL-NS the sensor must have been powered ≥ 5 s). Reply `16 01 03 E6`.
pub fn calibrate_command(ppm: u16) -> Option<Command> {
    valid_target(ppm).then(|| fixed(cmd::CALIBRATE, &ppm.to_be_bytes()))
}

/// Automatic baseline calibration parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Abc {
    /// ABC open (CM1106 default) or closed (CM1106SL-NS default).
    pub enabled: bool,
    /// Calibration cycle, 1–30 days (CM1106 default 15, CM1106SL-NS 7).
    pub cycle_days: u8,
    /// Baseline in ppm, 400–1500 (default 400).
    pub base_ppm: u16,
}

impl Abc {
    fn data(&self) -> Option<[u8; 6]> {
        if !(1..=30).contains(&self.cycle_days) || !valid_target(self.base_ppm) {
            return None;
        }
        let [hi, lo] = self.base_ppm.to_be_bytes();
        let switch = if self.enabled { 0 } else { 2 };
        Some([100, switch, self.cycle_days, hi, lo, 100])
    }

    /// Decode the 6 data bytes of a set/check ABC frame.
    pub fn decode(data: &[u8]) -> Option<Self> {
        match *data {
            [_, switch, cycle_days, hi, lo, _] => Some(Self {
                enabled: switch == 0,
                cycle_days,
                base_ppm: u16::from_be_bytes([hi, lo]),
            }),
            _ => None,
        }
    }
}

/// `11 07 10 64 DF2 DF3 DF4 DF5 64 CS`; `None` if a field is out of range.
/// Reply `16 01 10 D9`.
pub fn set_abc_command(abc: Abc) -> Option<Command> {
    abc.data().map(|d| fixed(cmd::SET_ABC, &d))
}

/// CM1106SL-NS continuous-mode period and moving average.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Period {
    /// Measurement period, 4–600 s.
    pub period_s: u16,
    /// Number of smoothed readings; `period_s × smoothing` should not
    /// exceed 30 min.
    pub smoothing: u8,
}

/// `11 04 50 PH PL N CS`; `None` if the period is outside 4–600 s or
/// `smoothing` is 0.
pub fn set_period_command(p: Period) -> Option<Command> {
    if !(4..=600).contains(&p.period_s) || p.smoothing == 0 {
        return None;
    }
    let [hi, lo] = p.period_s.to_be_bytes();
    Some(fixed(cmd::PERIOD, &[hi, lo, p.smoothing]))
}

/// CM1106SL-NS working mode (UART command 0x51 encoding).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WorkingMode {
    /// Mode A: one measurement per EN power cycle, no averaging (default).
    #[default]
    Single,
    /// Mode B: measures every period (default 2 min) with moving average.
    Continuous,
}

/// `11 02 51 DF1 CS` (0 single, 1 continuous). Reply `16 01 51 98`.
pub fn set_working_mode_command(mode: WorkingMode) -> Command {
    fixed(
        cmd::WORKING_MODE,
        &[u8::from(mode == WorkingMode::Continuous)],
    )
}

/// CO2 status byte (UART DF3 / CM1106-C I2C DF2; reserved on CM1106SL-NS).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Status(pub u8);

impl Status {
    /// Bit 0: preheating (reading fixed at 550 ppm).
    pub const fn preheating(self) -> bool {
        self.0 & 0x01 != 0
    }
    /// Bit 1: sensor error.
    pub const fn error(self) -> bool {
        self.0 & 0x02 != 0
    }
    /// Bit 2: over measurement range.
    pub const fn over_range(self) -> bool {
        self.0 & 0x04 != 0
    }
    /// Bit 3: under measurement range.
    pub const fn under_range(self) -> bool {
        self.0 & 0x08 != 0
    }
    /// Bit 4: not calibrated.
    pub const fn non_calibrated(self) -> bool {
        self.0 & 0x10 != 0
    }
    /// Bit 5: light source aging.
    pub const fn light_aging(self) -> bool {
        self.0 & 0x20 != 0
    }
    /// Bit 6: drift.
    pub const fn drift(self) -> bool {
        self.0 & 0x40 != 0
    }
}

/// CO2 reading from the UART or CM1106-C I2C protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// CO2 in ppm.
    pub co2_ppm: u16,
    /// Status flags.
    pub status: Status,
}

impl Measurement {
    /// Decode the data bytes of a read-CO2 reply (`DF1 DF2 status [DF4]`).
    pub fn decode(data: &[u8]) -> Option<Self> {
        match *data {
            [hi, lo, status] | [hi, lo, status, _] => Some(Self {
                co2_ppm: u16::from_be_bytes([hi, lo]),
                status: Status(status),
            }),
            _ => None,
        }
    }
}

impl Readings for Measurement {
    /// CO2 is omitted while preheating (fixed 550 ppm) or on sensor error.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if !self.status.preheating() && !self.status.error() {
            f(Quantity::Co2(u32::from(self.co2_ppm)));
        }
    }
}

/// Decode a serial number (five 0–9999 integers) from 10 data bytes.
pub fn decode_serial_number(data: &[u8]) -> Option<[u16; 5]> {
    let d: &[u8; 10] = data.get(..10)?.try_into().ok()?;
    let mut sn = [0u16; 5];
    for (i, w) in sn.iter_mut().enumerate() {
        *w = u16::from_be_bytes([d[2 * i], d[2 * i + 1]]);
    }
    Some(sn)
}

/// Decode the 10 ASCII version bytes (UART DF1..DF10; CM1106-C I2C
/// DF0..DF9), e.g. `b"CM V0.0.20"`.
pub fn decode_version(data: &[u8]) -> Option<[u8; 10]> {
    data.get(..10)?.try_into().ok()
}

/// A checksum-valid UART response frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    buf: [u8; MAX_FRAME_LEN],
    len: u8,
}

impl Frame {
    /// Raw frame bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..usize::from(self.len)]
    }

    /// Command code.
    pub const fn command(&self) -> u8 {
        self.buf[2]
    }

    /// Data bytes between the command code and the checksum.
    pub fn data(&self) -> &[u8] {
        &self.buf[3..usize::from(self.len) - 1]
    }

    fn data_for(&self, code: u8) -> Option<&[u8]> {
        (self.command() == code).then(|| self.data())
    }

    /// Read-CO2 reply.
    pub fn measurement(&self) -> Option<Measurement> {
        Measurement::decode(self.data_for(cmd::READ_CO2)?)
    }

    /// Check-ABC reply (CM1106SL-NS).
    pub fn abc(&self) -> Option<Abc> {
        Abc::decode(self.data_for(cmd::CHECK_ABC)?)
    }

    /// Serial-number reply.
    pub fn serial_number(&self) -> Option<[u16; 5]> {
        decode_serial_number(self.data_for(cmd::SERIAL_NUMBER)?)
    }

    /// Software-version reply.
    pub fn version(&self) -> Option<[u8; 10]> {
        decode_version(self.data_for(cmd::VERSION)?)
    }

    /// Check-period reply (CM1106SL-NS).
    pub fn period(&self) -> Option<Period> {
        match *self.data_for(cmd::PERIOD)? {
            [hi, lo, smoothing] => Some(Period {
                period_s: u16::from_be_bytes([hi, lo]),
                smoothing,
            }),
            _ => None,
        }
    }

    /// Check-working-mode reply (CM1106SL-NS).
    pub fn working_mode(&self) -> Option<WorkingMode> {
        match *self.data_for(cmd::WORKING_MODE)? {
            [0] => Some(WorkingMode::Single),
            [1] => Some(WorkingMode::Continuous),
            _ => None,
        }
    }

    /// `true` for a data-less acknowledgement of `code` (e.g. `16 01 10 D9`).
    pub fn is_ack(&self, code: u8) -> bool {
        self.data_for(code).is_some_and(<[u8]>::is_empty)
    }
}

/// UART byte-stream parser: resynchronises on `0x16`, checks the length
/// byte and only yields frames with a valid checksum.
#[derive(Debug, Clone, Default)]
pub struct Parser {
    buf: [u8; MAX_FRAME_LEN],
    len: usize,
    checksum_error: bool,
}

impl Parser {
    /// Empty parser.
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_FRAME_LEN],
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
        if self.len == 0 && byte != HEAD_RESPONSE {
            return None;
        }
        if self.len == 1 && !(1..=MAX_LEN_BYTE).contains(&byte) {
            self.len = 0;
            return self.feed(byte);
        }
        self.buf[self.len] = byte;
        self.len += 1;
        if self.len < 2 || self.len < usize::from(self.buf[1]) + 3 {
            return None;
        }
        let n = self.len;
        self.len = 0;
        if checksum(&self.buf[..n - 1]) == self.buf[n - 1] {
            return Some(Frame {
                buf: self.buf,
                len: n as u8,
            });
        }
        self.checksum_error = true;
        // Re-scan the rejected bytes for a later frame start.
        let old = self.buf;
        let mut found = None;
        for b in &old[1..n] {
            if let Some(f) = self.feed(*b) {
                found = Some(f);
            }
        }
        found
    }
}

/// Why a CM1106-C I2C reply was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplyError {
    /// Too short or not a reply to the expected command.
    Command,
    /// Checksum mismatch.
    Checksum,
}

impl<E> From<ReplyError> for Error<E> {
    fn from(e: ReplyError) -> Self {
        match e {
            ReplyError::Command => Error::InvalidData,
            ReplyError::Checksum => Error::Crc,
        }
    }
}

/// Verify a CM1106-C I2C reply `CMD DF.. CS` to `code`; returns the data
/// bytes.
pub fn check_i2c_reply(code: u8, reply: &[u8]) -> Result<&[u8], ReplyError> {
    let (cs, body) = reply.split_last().ok_or(ReplyError::Command)?;
    if body.first() != Some(&code) {
        return Err(ReplyError::Command);
    }
    if checksum(body) != *cs {
        return Err(ReplyError::Checksum);
    }
    Ok(&body[1..])
}

/// CM1106-C I2C set-ABC write `10 64 DF1 DF2 DF3 DF4 64`; `None` if out of
/// range.
pub fn i2c_set_abc(abc: Abc) -> Option<[u8; 7]> {
    let d = abc.data()?;
    Some([cmd::SET_ABC, d[0], d[1], d[2], d[3], d[4], d[5]])
}

/// CM1106-C I2C calibrate write `03 DF0 DF1`; `None` outside 400–1500 ppm.
pub fn i2c_calibrate(ppm: u16) -> Option<[u8; 3]> {
    let [hi, lo] = ppm.to_be_bytes();
    valid_target(ppm).then_some([cmd::CALIBRATE, hi, lo])
}

/// Length of the CM1106-C read-CO2 reply.
pub const I2C_READ_LEN: usize = 5;

/// CM1106SL-NS measurement mode (I2C register 0x95 encoding — note the
/// values are the reverse of the UART [`WorkingMode`] command).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SlMode {
    /// Register value 0: measures every [`reg::MEASUREMENT_PERIOD`].
    Continuous,
    /// Register value 1 (default): write [`reg::START_SINGLE`] to measure.
    #[default]
    Single,
}

/// CM1106SL-NS calibration commands (register 0x82).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlCalibration {
    /// `0x7C05`: calibrate to [`reg::CALIBRATION_TARGET`] (set it first).
    Target,
    /// `0x7C06`: calibrate to [`reg::ABC_TARGET`] (fresh air, 400 ppm).
    Background,
    /// `0x7C07`: zero calibration (0 ppm, e.g. nitrogen).
    Zero,
}

impl SlCalibration {
    /// Register value.
    pub const fn value(self) -> u16 {
        match self {
            Self::Target => 0x7C05,
            Self::Background => 0x7C06,
            Self::Zero => 0x7C07,
        }
    }
}

/// Bytes read from [`reg::ERROR_STATUS`] by [`Cm1106Sl`] (0x00..=0x0E).
pub const SL_READ_LEN: usize = 15;

/// CM1106SL-NS I2C reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlMeasurement {
    /// CO2 in ppm.
    pub co2_ppm: u16,
    /// Sensor chip temperature in 0.01 °C (not ambient; not reported as a
    /// [`Quantity`]).
    pub chip_temperature: i16,
    /// Error status register (bit 0 fatal, bit 5 out of range).
    pub error_status: u8,
    /// Measurement counter (wraps at 255).
    pub count: u8,
}

impl SlMeasurement {
    /// Decode registers 0x00..=0x0E.
    pub fn from_registers(r: &[u8; SL_READ_LEN]) -> Self {
        Self {
            co2_ppm: u16::from_be_bytes([r[6], r[7]]),
            chip_temperature: i16::from_be_bytes([r[8], r[9]]),
            error_status: r[0],
            count: r[13],
        }
    }

    /// Fatal error: analog front end initialisation failed.
    pub const fn fatal(&self) -> bool {
        self.error_status & 0x01 != 0
    }

    /// Concentration outside the measurement range.
    pub const fn out_of_range(&self) -> bool {
        self.error_status & 0x20 != 0
    }
}

impl Readings for SlMeasurement {
    /// CO2 is omitted on a fatal error.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if !self.fatal() {
            f(Quantity::Co2(u32::from(self.co2_ppm)));
        }
    }
}

fn finish<T, E>(found: Option<T>, parser: &mut Parser) -> Result<T, Error<E>> {
    match found {
        Some(m) => Ok(m),
        None if parser.take_checksum_error() => Err(Error::Crc),
        None => Err(Error::NotReady),
    }
}

macro_rules! uart_constructors {
    () => {
        /// Create a UART driver; no bus traffic.
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
    };
}

macro_rules! c_constructors {
    () => {
        /// Create a CM1106-C driver at [`I2C_ADDRESS`]; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self { i2c }
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

macro_rules! sl_constructors {
    () => {
        /// Create a CM1106SL-NS driver at [`SL_I2C_ADDRESS`] assuming the
        /// default [`SlMode::Single`]; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                address: SL_I2C_ADDRESS,
                mode: SlMode::Single,
            }
        }

        /// Use a different 7-bit address.
        pub const fn with_address(mut self, address: u8) -> Self {
            self.address = address;
            self
        }

        /// Assume the (EEPROM-stored) measurement mode; no bus traffic.
        pub const fn with_mode(mut self, mode: SlMode) -> Self {
            self.mode = mode;
            self
        }

        /// Measurement mode the driver assumes.
        pub const fn mode(&self) -> SlMode {
            self.mode
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking UART driver (CM1106, CM1106-C, CM1106SL-NS).
pub struct Cm1106<U> {
    uart: U,
    parser: Parser,
}

impl<U> Cm1106<U> {
    uart_constructors!();
}

impl<U> Cm1106<U>
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
    /// newest frame for which `code` matches.
    pub fn poll_frame(&mut self, code: u8) -> Result<Option<Frame>, Error<U::Error>> {
        let mut latest = None;
        let mut buf = [0u8; MAX_FRAME_LEN];
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
                    if f.command() == code {
                        latest = Some(f);
                    }
                }
            }
        }
        Ok(latest)
    }

    /// Discard pending input and send `command` (from one of the
    /// `*_command` encoders). Returns the time to wait before
    /// [`read_reply`](Self::read_reply).
    pub fn send_command(&mut self, command: &Command) -> Result<WaitUs, Error<U::Error>> {
        self.drain()?;
        self.uart
            .write_all(command.as_bytes())
            .map_err(Error::bus)?;
        self.uart.flush().map_err(Error::bus)?;
        Ok(RESPONSE_US)
    }

    /// Newest reply to command `code` (`NotReady` if none has arrived).
    pub fn read_reply(&mut self, code: u8) -> Result<Frame, Error<U::Error>> {
        let found = self.poll_frame(code)?;
        finish(found, &mut self.parser)
    }

    /// Send a set-ABC command (`InvalidArgument` if out of range); the reply is
    /// an ack for [`cmd::SET_ABC`].
    pub fn set_abc(&mut self, abc: Abc) -> Result<WaitUs, Error<U::Error>> {
        let c = set_abc_command(abc).ok_or(Error::InvalidArgument)?;
        self.send_command(&c)
    }

    /// Send a calibrate command (`InvalidArgument` outside 400–1500 ppm); the
    /// reply is an ack for [`cmd::CALIBRATE`].
    pub fn calibrate(&mut self, ppm: u16) -> Result<WaitUs, Error<U::Error>> {
        let c = calibrate_command(ppm).ok_or(Error::InvalidArgument)?;
        self.send_command(&c)
    }
}

impl<U> zigbee_sensor::blocking::Sensor for Cm1106<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type BusError = U::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        self.send_command(&read_co2_command())
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
        let found = self
            .poll_frame(cmd::READ_CO2)?
            .and_then(|f| f.measurement());
        finish(found, &mut self.parser)
    }
}

/// Blocking CM1106-C I2C driver.
pub struct Cm1106C<I2C> {
    i2c: I2C,
}

impl<I2C> Cm1106C<I2C> {
    c_constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Cm1106C<I2C> {
    /// Write a raw command (`CMD DF..`). Returns the time to wait before
    /// [`read_reply`](Self::read_reply).
    pub fn send(&mut self, bytes: &[u8]) -> Result<WaitUs, Error<I2C::Error>> {
        self.i2c.write(I2C_ADDRESS, bytes).map_err(Error::bus)?;
        Ok(I2C_RESPONSE_US)
    }

    /// Read `buf.len()` bytes (`CMD DF.. CS`) of the reply to `code`,
    /// verify it and return the data bytes.
    pub fn read_reply<'a>(
        &mut self,
        code: u8,
        buf: &'a mut [u8],
    ) -> Result<&'a [u8], Error<I2C::Error>> {
        self.i2c.read(I2C_ADDRESS, buf).map_err(Error::bus)?;
        Ok(check_i2c_reply(code, buf)?)
    }

    /// Write set-ABC (`InvalidArgument` if out of range).
    pub fn set_abc(&mut self, abc: Abc) -> Result<WaitUs, Error<I2C::Error>> {
        self.send(&i2c_set_abc(abc).ok_or(Error::InvalidArgument)?)
    }

    /// Write calibrate (`InvalidArgument` outside 400–1500 ppm).
    pub fn calibrate(&mut self, ppm: u16) -> Result<WaitUs, Error<I2C::Error>> {
        self.send(&i2c_calibrate(ppm).ok_or(Error::InvalidArgument)?)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Cm1106C<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.send(&[cmd::READ_CO2])
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut buf = [0u8; I2C_READ_LEN];
        let data = self.read_reply(cmd::READ_CO2, &mut buf)?;
        Measurement::decode(data).ok_or(Error::InvalidData)
    }
}

/// Blocking CM1106SL-NS I2C driver.
pub struct Cm1106Sl<I2C> {
    i2c: I2C,
    address: u8,
    mode: SlMode,
}

impl<I2C> Cm1106Sl<I2C> {
    sl_constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Cm1106Sl<I2C> {
    /// Read consecutive registers starting at `start` (register write, then
    /// a separate read, as in the datasheet example).
    pub fn read_registers(&mut self, start: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, &[start]).map_err(Error::bus)?;
        self.i2c.read(self.address, buf).map_err(Error::bus)
    }

    /// Write one byte register. EEPROM registers take [`SL_EEPROM_WRITE_US`].
    pub fn write_u8(&mut self, register: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register, value])
            .map_err(Error::bus)
    }

    /// Write a big-endian 16-bit register pair.
    pub fn write_u16(&mut self, register: u8, value: u16) -> Result<(), Error<I2C::Error>> {
        let [hi, lo] = value.to_be_bytes();
        self.i2c
            .write(self.address, &[register, hi, lo])
            .map_err(Error::bus)
    }

    /// Read a big-endian 16-bit register pair.
    pub fn read_u16(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
        let mut b = [0u8; 2];
        self.read_registers(register, &mut b)?;
        Ok(u16::from_be_bytes(b))
    }

    /// Set the measurement mode (EEPROM). Returns the write time.
    pub fn set_mode(&mut self, mode: SlMode) -> Result<WaitUs, Error<I2C::Error>> {
        self.write_u8(reg::MEASUREMENT_MODE, u8::from(mode == SlMode::Single))?;
        self.mode = mode;
        Ok(SL_EEPROM_WRITE_US)
    }

    /// Enable or disable ABC (meter control bit 1, EEPROM; read-modify-write).
    pub fn set_abc_enabled(&mut self, enabled: bool) -> Result<WaitUs, Error<I2C::Error>> {
        let mut b = [0u8];
        self.read_registers(reg::METER_CONTROL, &mut b)?;
        let v = if enabled { b[0] & !0x02 } else { b[0] | 0x02 };
        self.write_u8(reg::METER_CONTROL, v)?;
        Ok(SL_EEPROM_WRITE_US)
    }

    /// Start a calibration.
    pub fn calibrate(&mut self, c: SlCalibration) -> Result<(), Error<I2C::Error>> {
        self.write_u16(reg::CALIBRATION_COMMAND, c.value())
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Cm1106Sl<I2C> {
    type BusError = I2C::Error;
    type Measurement = SlMeasurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        match self.mode {
            SlMode::Continuous => Ok(0),
            SlMode::Single => {
                self.write_u8(reg::START_SINGLE, 1)?;
                Ok(SL_CYCLE_US)
            }
        }
    }

    fn read_measurement(&mut self) -> Result<SlMeasurement, Error<I2C::Error>> {
        let mut r = [0u8; SL_READ_LEN];
        self.read_registers(reg::ERROR_STATUS, &mut r)?;
        Ok(SlMeasurement::from_registers(&r))
    }
}

/// Async drivers.
pub mod asynch {
    use super::{
        Abc, Command, Frame, I2C_ADDRESS, I2C_READ_LEN, I2C_RESPONSE_US, MAX_DRAIN_BYTES,
        MAX_FRAME_LEN, MAX_READ_BYTES, Measurement, Parser, RESPONSE_US, SL_CYCLE_US,
        SL_EEPROM_WRITE_US, SL_I2C_ADDRESS, SL_READ_LEN, SlCalibration, SlMeasurement, SlMode,
        calibrate_command, check_i2c_reply, cmd, finish, i2c_calibrate, i2c_set_abc,
        read_co2_command, reg, set_abc_command,
    };
    use embedded_hal_async::i2c::I2c;
    use embedded_io_async::{Read, ReadReady, Write};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Cm1106`].
    pub struct Cm1106<U> {
        uart: U,
        parser: Parser,
    }

    impl<U> Cm1106<U> {
        uart_constructors!();
    }

    impl<U: Read + ReadReady + Write> Cm1106<U> {
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

        /// See [`super::Cm1106::poll_frame`].
        pub async fn poll_frame(&mut self, code: u8) -> Result<Option<Frame>, Error<U::Error>> {
            let mut latest = None;
            let mut buf = [0u8; MAX_FRAME_LEN];
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
                        if f.command() == code {
                            latest = Some(f);
                        }
                    }
                }
            }
            Ok(latest)
        }

        /// See [`super::Cm1106::send_command`].
        pub async fn send_command(&mut self, command: &Command) -> Result<WaitUs, Error<U::Error>> {
            self.drain().await?;
            self.uart
                .write_all(command.as_bytes())
                .await
                .map_err(Error::bus)?;
            self.uart.flush().await.map_err(Error::bus)?;
            Ok(RESPONSE_US)
        }

        /// See [`super::Cm1106::read_reply`].
        pub async fn read_reply(&mut self, code: u8) -> Result<Frame, Error<U::Error>> {
            let found = self.poll_frame(code).await?;
            finish(found, &mut self.parser)
        }

        /// See [`super::Cm1106::set_abc`].
        pub async fn set_abc(&mut self, abc: Abc) -> Result<WaitUs, Error<U::Error>> {
            let c = set_abc_command(abc).ok_or(Error::InvalidArgument)?;
            self.send_command(&c).await
        }

        /// See [`super::Cm1106::calibrate`].
        pub async fn calibrate(&mut self, ppm: u16) -> Result<WaitUs, Error<U::Error>> {
            let c = calibrate_command(ppm).ok_or(Error::InvalidArgument)?;
            self.send_command(&c).await
        }
    }

    impl<U: Read + ReadReady + Write> zigbee_sensor::asynch::Sensor for Cm1106<U> {
        type BusError = U::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
            self.send_command(&read_co2_command()).await
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
            let found = self
                .poll_frame(cmd::READ_CO2)
                .await?
                .and_then(|f| f.measurement());
            finish(found, &mut self.parser)
        }
    }

    /// Async twin of [`super::Cm1106C`].
    pub struct Cm1106C<I2C> {
        i2c: I2C,
    }

    impl<I2C> Cm1106C<I2C> {
        c_constructors!();
    }

    impl<I2C: I2c> Cm1106C<I2C> {
        /// See [`super::Cm1106C::send`].
        pub async fn send(&mut self, bytes: &[u8]) -> Result<WaitUs, Error<I2C::Error>> {
            self.i2c
                .write(I2C_ADDRESS, bytes)
                .await
                .map_err(Error::bus)?;
            Ok(I2C_RESPONSE_US)
        }

        /// See [`super::Cm1106C::read_reply`].
        pub async fn read_reply<'a>(
            &mut self,
            code: u8,
            buf: &'a mut [u8],
        ) -> Result<&'a [u8], Error<I2C::Error>> {
            self.i2c.read(I2C_ADDRESS, buf).await.map_err(Error::bus)?;
            Ok(check_i2c_reply(code, buf)?)
        }

        /// See [`super::Cm1106C::set_abc`].
        pub async fn set_abc(&mut self, abc: Abc) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(&i2c_set_abc(abc).ok_or(Error::InvalidArgument)?)
                .await
        }

        /// See [`super::Cm1106C::calibrate`].
        pub async fn calibrate(&mut self, ppm: u16) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(&i2c_calibrate(ppm).ok_or(Error::InvalidArgument)?)
                .await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Cm1106C<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(&[cmd::READ_CO2]).await
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut buf = [0u8; I2C_READ_LEN];
            let data = self.read_reply(cmd::READ_CO2, &mut buf).await?;
            Measurement::decode(data).ok_or(Error::InvalidData)
        }
    }

    /// Async twin of [`super::Cm1106Sl`].
    pub struct Cm1106Sl<I2C> {
        i2c: I2C,
        address: u8,
        mode: SlMode,
    }

    impl<I2C> Cm1106Sl<I2C> {
        sl_constructors!();
    }

    impl<I2C: I2c> Cm1106Sl<I2C> {
        /// See [`super::Cm1106Sl::read_registers`].
        pub async fn read_registers(
            &mut self,
            start: u8,
            buf: &mut [u8],
        ) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[start])
                .await
                .map_err(Error::bus)?;
            self.i2c.read(self.address, buf).await.map_err(Error::bus)
        }

        /// See [`super::Cm1106Sl::write_u8`].
        pub async fn write_u8(&mut self, register: u8, value: u8) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[register, value])
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Cm1106Sl::write_u16`].
        pub async fn write_u16(
            &mut self,
            register: u8,
            value: u16,
        ) -> Result<(), Error<I2C::Error>> {
            let [hi, lo] = value.to_be_bytes();
            self.i2c
                .write(self.address, &[register, hi, lo])
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Cm1106Sl::read_u16`].
        pub async fn read_u16(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
            let mut b = [0u8; 2];
            self.read_registers(register, &mut b).await?;
            Ok(u16::from_be_bytes(b))
        }

        /// See [`super::Cm1106Sl::set_mode`].
        pub async fn set_mode(&mut self, mode: SlMode) -> Result<WaitUs, Error<I2C::Error>> {
            self.write_u8(reg::MEASUREMENT_MODE, u8::from(mode == SlMode::Single))
                .await?;
            self.mode = mode;
            Ok(SL_EEPROM_WRITE_US)
        }

        /// See [`super::Cm1106Sl::set_abc_enabled`].
        pub async fn set_abc_enabled(
            &mut self,
            enabled: bool,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            let mut b = [0u8];
            self.read_registers(reg::METER_CONTROL, &mut b).await?;
            let v = if enabled { b[0] & !0x02 } else { b[0] | 0x02 };
            self.write_u8(reg::METER_CONTROL, v).await?;
            Ok(SL_EEPROM_WRITE_US)
        }

        /// See [`super::Cm1106Sl::calibrate`].
        pub async fn calibrate(&mut self, c: SlCalibration) -> Result<(), Error<I2C::Error>> {
            self.write_u16(reg::CALIBRATION_COMMAND, c.value()).await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Cm1106Sl<I2C> {
        type BusError = I2C::Error;
        type Measurement = SlMeasurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            match self.mode {
                SlMode::Continuous => Ok(0),
                SlMode::Single => {
                    self.write_u8(reg::START_SINGLE, 1).await?;
                    Ok(SL_CYCLE_US)
                }
            }
        }

        async fn read_measurement(&mut self) -> Result<SlMeasurement, Error<I2C::Error>> {
            let mut r = [0u8; SL_READ_LEN];
            self.read_registers(reg::ERROR_STATUS, &mut r).await?;
            Ok(SlMeasurement::from_registers(&r))
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

    /// Datasheet 4.1 example: 600 ppm.
    const READING: [u8; 8] = [0x16, 0x05, 0x01, 0x02, 0x58, 0x00, 0x00, 0x8A];

    fn frames(bytes: &[u8]) -> Vec<Frame> {
        let mut p = Parser::new();
        bytes.iter().filter_map(|b| p.feed(*b)).collect()
    }

    #[test]
    fn uart_commands_match_datasheet() {
        assert_eq!(read_co2_command().as_bytes(), [0x11, 0x01, 0x01, 0xED]);
        assert_eq!(serial_number_command().as_bytes(), [0x11, 0x01, 0x1F, 0xCF]);
        assert_eq!(version_command().as_bytes(), [0x11, 0x01, 0x1E, 0xD0]);
        assert_eq!(check_abc_command().as_bytes(), [0x11, 0x01, 0x0F, 0xDF]);
        assert_eq!(
            calibrate_command(600).unwrap().as_bytes(),
            [0x11, 0x03, 0x03, 0x02, 0x58, 0x8F]
        );
        let abc = |enabled, cycle_days| Abc {
            enabled,
            cycle_days,
            base_ppm: 400,
        };
        let expect = |sw, days, cs| [0x11, 0x07, 0x10, 0x64, sw, days, 0x01, 0x90, 0x64, cs];
        assert_eq!(
            set_abc_command(abc(false, 7)).unwrap().as_bytes(),
            expect(2, 7, 0x76)
        );
        assert_eq!(
            set_abc_command(abc(true, 7)).unwrap().as_bytes(),
            expect(0, 7, 0x78)
        );
        assert_eq!(
            set_abc_command(abc(true, 10)).unwrap().as_bytes(),
            expect(0, 10, 0x75)
        );
        let period = Period {
            period_s: 120,
            smoothing: 15,
        };
        assert_eq!(
            set_period_command(period).unwrap().as_bytes(),
            [0x11, 0x04, 0x50, 0x00, 0x78, 0x0F, 0x14]
        );
        assert_eq!(
            set_working_mode_command(WorkingMode::Continuous).as_bytes(),
            [0x11, 0x02, 0x51, 0x01, 0x9B]
        );
        assert_eq!(calibrate_command(399), None);
        assert_eq!(set_abc_command(abc(true, 31)), None);
        assert_eq!(command(0x10, &[0; 7]), None);
    }

    #[test]
    fn uart_responses_decode() {
        // Datasheet 4.5 example; its printed checksum 0x97 does not match the
        // printed bytes (erratum), the correct one is 0x9C.
        let version = [
            0x16, 0x0C, 0x1E, 0x43, 0x4D, 0x20, 0x56, 0x30, 0x2E, 0x30, 0x2E, 0x32, 0x30, 0x00,
        ];
        let mut printed = version.to_vec();
        printed.push(0x97);
        assert!(frames(&printed).is_empty());
        let mut stream = READING.to_vec();
        stream.extend(version);
        stream.push(0x9C);
        stream.extend([0x16, 0x07, 0x0F, 0x64, 0x02, 0x07, 0x01, 0x90, 0x64, 0x72]);
        stream.extend([0x16, 0x01, 0x10, 0xD9, 0x16, 0x01, 0x03, 0xE6]);
        let f = frames(&stream);
        assert_eq!(f.len(), 5);
        assert_eq!(
            f[0].measurement(),
            Some(Measurement {
                co2_ppm: 600,
                status: Status(0)
            })
        );
        assert_eq!(&f[1].version().unwrap(), b"CM V0.0.20");
        assert_eq!(
            f[2].abc(),
            Some(Abc {
                enabled: false,
                cycle_days: 7,
                base_ppm: 400
            })
        );
        assert!(f[3].is_ack(cmd::SET_ABC) && f[4].is_ack(cmd::CALIBRATE));
        assert_eq!(f[0].abc(), None);

        let mut sn = vec![0x16, 0x0B, 0x1F, 0, 1, 0, 2, 0, 3, 0x27, 0x0F, 0, 5];
        sn.push(checksum(&sn));
        assert_eq!(frames(&sn)[0].serial_number(), Some([1, 2, 3, 9999, 5]));
    }

    #[test]
    fn parser_resyncs_and_flags_bad_checksum() {
        let mut bad = READING;
        bad[4] ^= 1;
        let mut stream = vec![0x00, 0x16, 0x40, 0x16];
        stream.extend_from_slice(&bad);
        stream.extend_from_slice(&READING);
        let mut p = Parser::new();
        let f: Vec<Frame> = stream.iter().filter_map(|b| p.feed(*b)).collect();
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].as_bytes(), READING);
        assert!(p.take_checksum_error());
    }

    #[test]
    fn uart_query_cycle() {
        let mut uart = FakeUart::default();
        uart.rx.extend([0x16, 0x05]); // stale partial frame is discarded
        uart.on_write = READING.to_vec();
        let mut co2 = Cm1106::new(&mut uart);
        assert_eq!(co2.start_measurement().unwrap(), RESPONSE_US);
        let m = co2.read_measurement().unwrap();
        assert_eq!(m.co2_ppm, 600);
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(seen, vec![Quantity::Co2(600)]);
        assert_eq!(co2.read_measurement(), Err(Error::NotReady));

        let preheat = Measurement {
            co2_ppm: 550,
            status: Status(0x01),
        };
        let mut seen = Vec::new();
        preheat.for_each(&mut |q| seen.push(q));
        assert!(seen.is_empty());

        co2.uart.on_write = vec![0x16, 0x01, 0x10, 0xD9];
        let abc = Abc {
            enabled: false,
            cycle_days: 7,
            base_ppm: 400,
        };
        co2.set_abc(abc).unwrap();
        assert!(co2.read_reply(cmd::SET_ABC).unwrap().is_ack(cmd::SET_ABC));
        assert_eq!(co2.calibrate(2000), Err(Error::InvalidArgument));
        assert_eq!(&uart.tx[..4], read_co2_command().as_bytes());
        assert_eq!(&uart.tx[4..], set_abc_command(abc).unwrap().as_bytes());
    }

    #[test]
    fn corrupt_response_is_crc_error() {
        let mut bad = READING;
        bad[3] ^= 0x10;
        let mut uart = FakeUart {
            on_write: bad.to_vec(),
            ..FakeUart::default()
        };
        let mut co2 = Cm1106::new(&mut uart);
        co2.start_measurement().unwrap();
        assert_eq!(co2.read_measurement(), Err(Error::Crc));
        assert_eq!(co2.read_measurement(), Err(Error::NotReady));
    }

    #[test]
    fn cm1106c_i2c() {
        let expectations = [
            Transaction::write(I2C_ADDRESS, vec![0x01]),
            Transaction::read(I2C_ADDRESS, vec![0x01, 0x03, 0x20, 0x00, 0xDC]),
            Transaction::read(I2C_ADDRESS, vec![0x01, 0x03, 0x20, 0x00, 0xDD]),
            Transaction::read(I2C_ADDRESS, vec![0x10, 0x03, 0x20, 0x00, 0xCD]),
            Transaction::write(I2C_ADDRESS, vec![0x10, 0x64, 0x00, 0x0F, 0x01, 0x90, 0x64]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut co2 = Cm1106C::new(&mut i2c);
        assert_eq!(co2.start_measurement().unwrap(), I2C_RESPONSE_US);
        assert_eq!(co2.read_measurement().unwrap().co2_ppm, 800);
        assert_eq!(co2.read_measurement(), Err(Error::Crc));
        assert_eq!(co2.read_measurement(), Err(Error::InvalidData));
        co2.set_abc(Abc {
            enabled: true,
            cycle_days: 15,
            base_ppm: 400,
        })
        .unwrap();
        assert_eq!(co2.calibrate(300), Err(Error::InvalidArgument));
        i2c.done();
    }

    fn sl_registers(co2: u16, error: u8) -> Vec<u8> {
        let mut r = vec![0u8; SL_READ_LEN];
        r[0] = error;
        r[6..8].copy_from_slice(&co2.to_be_bytes());
        r[8..10].copy_from_slice(&2223i16.to_be_bytes());
        r[13] = 7;
        r
    }

    #[test]
    fn cm1106sl_i2c() {
        let a = SL_I2C_ADDRESS;
        let expectations = [
            Transaction::write(a, vec![reg::START_SINGLE, 1]),
            Transaction::write(a, vec![reg::ERROR_STATUS]),
            Transaction::read(a, sl_registers(1262, 0)),
            Transaction::write(a, vec![reg::ERROR_STATUS]),
            Transaction::read(a, sl_registers(0, 0x01)),
            Transaction::write(a, vec![reg::METER_CONTROL]),
            Transaction::read(a, vec![0x01]),
            Transaction::write(a, vec![reg::METER_CONTROL, 0x03]),
            Transaction::write(a, vec![reg::MEASUREMENT_MODE, 0x00]),
            Transaction::write(a, vec![reg::CALIBRATION_COMMAND, 0x7C, 0x06]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut co2 = Cm1106Sl::new(&mut i2c);
        assert_eq!(co2.start_measurement().unwrap(), SL_CYCLE_US);
        let m = co2.read_measurement().unwrap();
        assert_eq!(
            m,
            SlMeasurement {
                co2_ppm: 1262,
                chip_temperature: 2223,
                error_status: 0,
                count: 7
            }
        );
        let fatal = co2.read_measurement().unwrap();
        assert!(fatal.fatal());
        let mut seen = Vec::new();
        fatal.for_each(&mut |q| seen.push(q));
        assert!(seen.is_empty());
        assert_eq!(co2.set_abc_enabled(false).unwrap(), SL_EEPROM_WRITE_US);
        co2.set_mode(SlMode::Continuous).unwrap();
        assert_eq!(co2.start_measurement().unwrap(), 0);
        co2.calibrate(SlCalibration::Background).unwrap();
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
            on_write: READING.to_vec(),
            ..FakeUart::default()
        };
        let mut co2 = asynch::Cm1106::new(&mut uart);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(co2.measure(&mut delay)).unwrap().co2_ppm, 600);
        assert_eq!(delay.0, RESPONSE_US);

        let expectations = [
            Transaction::write(I2C_ADDRESS, vec![0x01]),
            Transaction::read(I2C_ADDRESS, vec![0x01, 0x03, 0x20, 0x00, 0xDC]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut c = asynch::Cm1106C::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(c.measure(&mut delay)).unwrap().co2_ppm, 800);
        assert_eq!(delay.0, I2C_RESPONSE_US);
        i2c.done();

        let a = SL_I2C_ADDRESS;
        let expectations = [
            Transaction::write(a, vec![reg::START_SINGLE, 1]),
            Transaction::write(a, vec![reg::ERROR_STATUS]),
            Transaction::read(a, sl_registers(1262, 0)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sl = asynch::Cm1106Sl::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(sl.measure(&mut delay)).unwrap().co2_ppm, 1262);
        assert_eq!(delay.0, SL_CYCLE_US);
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
