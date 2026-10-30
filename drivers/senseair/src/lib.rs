//! Senseair S8 and Sunrise / Sunlight NDIR CO2 sensors over Modbus RTU.
//!
//! Both families speak Modbus RTU on a 9600 8N1 UART (the S8 transmits two
//! stop bits, which a 1-stop-bit receiver accepts). A request is
//! `ADDR FUNC DATA.. CRC_LO CRC_HI` with the Modbus CRC-16
//! ([`zigbee_sensor::crc::crc16_modbus`]); the sensor answers within
//! [`RESPONSE_US`] (180 ms response time-out) or not at all.
//!
//! | | S8 | Sunrise / Sunlight |
//! |---|---|---|
//! | Default address | `0xFE` ("any sensor") | `0x68` |
//! | Functions | 03, 04, 06 | 03, 04, 0x10 |
//! | Registers per read | ≤ 17 (39-byte packet limit) | ≤ 32 |
//! | Register map | [`s8`] | [`sunrise`] |
//!
//! [`Device`] holds the model and address and encodes every request as a
//! pure function; [`Parser`] reassembles and CRC-checks the reply to the
//! last request. [`Senseair`] (blocking, `embedded_io`) and
//! [`asynch::Senseair`] (`embedded_io_async`) tie them to a UART:
//!
//! - [`send`](Senseair::send) discards stale input, writes a request and
//!   returns [`RESPONSE_US`]; [`read_registers`](Senseair::read_registers) /
//!   [`read_ack`](Senseair::read_ack) then return the reply or
//!   [`Error::NotReady`].
//! - [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//!   requests IR1–IR4 (status + CO2) and
//!   [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//!   decodes them into a [`Measurement`].
//!
//! Neither family has a sleep command, so there is no
//! [`LowPower`](zigbee_sensor::blocking::LowPower) implementation: the S8
//! measures continuously (2 s lamp cycle), and the Sunrise / Sunlight is
//! powered down between measurements with its EN pin in single-measurement
//! mode.
//!
//! # Sunrise / Sunlight single-measurement mode
//!
//! After [`Device::set_measurement_mode`]`(true)` and a reset, each
//! measurement is (Senseair TDE7318, section 3.3):
//!
//! 1. Drive EN high and wait [`SUNRISE_STARTUP_US`] (35 ms).
//! 2. Send [`Device::start_single`] with the state saved last time (start
//!    command + HR35–HR46, optionally HR47 pressure). The very first time
//!    there is no state: send the start command alone — never write zeros
//!    or arbitrary data to the state registers.
//! 3. Wait for nRDY to go low, or [`SUNRISE_SINGLE_US`] (2.4 s with the
//!    default 8 samples).
//! 4. Read the measurement (IR1–IR4).
//! 5. Send [`Device::read_state`] and keep the [`SunriseState`]
//!    ([`Senseair::read_state`]); if ABC is used, add the hours spent
//!    powered down with [`SunriseState::add_abc_hours`].
//! 6. Drive EN low (only after the reply to any write has arrived).
//!
//! If both pressure compensation and ABC are enabled, also save and restore
//! HR48 ([`sunrise::ABC_PRESSURE`]).
//!
//! Registers marked "(EE)" in the datasheet are EEPROM-backed: the total
//! number of EEPROM write cycles must stay below 10 000, so configure once
//! rather than at every power-up.
//!
//! References: Senseair "Modbus on Senseair S8" TDE2067; Senseair
//! "Modbus on Senseair Sunrise & Sunlight" TDE7318; Senseair Sunrise
//! "Handling and integration manual".
//!
//! ```ignore
//! let mut co2 = Senseair::s8(uart);
//! let wait_us = co2.start_measurement()?;
//! // ... wait_us later:
//! let m = co2.read_measurement()?;
//! // Set the ABC period to 8 days:
//! let wait_us = co2.send(co2.device().set_abc_period(192))?;
//! // ... wait_us later:
//! co2.read_ack()?;
//! ```

#![no_std]

use zigbee_sensor::crc::crc16_modbus;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Senseair S8 default ("any sensor") Modbus address.
pub const S8_ADDRESS: u8 = 0xFE;
/// Senseair Sunrise / Sunlight default Modbus address.
pub const SUNRISE_ADDRESS: u8 = 0x68;

/// Modbus response time-out of both families.
pub const RESPONSE_US: WaitUs = 180_000;
/// S8: wait at least one 2 s lamp cycle after a calibration command before
/// reading the acknowledgement register.
pub const S8_CALIBRATION_US: WaitUs = 2_000_000;
/// Sunrise / Sunlight: start-up time after EN goes high.
pub const SUNRISE_STARTUP_US: WaitUs = 35_000;
/// Sunrise / Sunlight: single measurement with the default configuration
/// (used when nRDY is not monitored).
pub const SUNRISE_SINGLE_US: WaitUs = 2_400_000;

/// Most registers in one read request (Sunrise; the S8 allows 17).
pub const MAX_READ_REGS: usize = 32;
/// Most registers in one write-multiple request.
pub const MAX_WRITE_REGS: usize = 16;
/// Longest request: write-multiple with [`MAX_WRITE_REGS`] values.
pub const MAX_REQUEST_LEN: usize = 9 + 2 * MAX_WRITE_REGS;
/// Longest reply: read with [`MAX_READ_REGS`] values.
pub const MAX_FRAME_LEN: usize = 5 + 2 * MAX_READ_REGS;

const MAX_READ_BYTES: usize = 256;
const MAX_DRAIN_BYTES: usize = 256;

/// Modbus function codes.
pub mod function {
    /// Read holding registers.
    pub const READ_HOLDING: u8 = 0x03;
    /// Read input registers.
    pub const READ_INPUT: u8 = 0x04;
    /// Write single register (S8 only).
    pub const WRITE_SINGLE: u8 = 0x06;
    /// Write multiple registers (Sunrise / Sunlight only).
    pub const WRITE_MULTIPLE: u8 = 0x10;
    /// Set in the function code of an exception reply.
    pub const EXCEPTION: u8 = 0x80;
}

/// Modbus exception codes.
pub mod exception {
    /// Function not implemented.
    pub const ILLEGAL_FUNCTION: u8 = 0x01;
    /// Register reserved or out of range.
    pub const ILLEGAL_DATA_ADDRESS: u8 = 0x02;
    /// Bad quantity or value.
    pub const ILLEGAL_DATA_VALUE: u8 = 0x03;
}

/// Senseair S8 register addresses (register number − 1).
pub mod s8 {
    /// IR1 meter status (see the `STATUS_*` bits).
    pub const METER_STATUS: u16 = 0;
    /// IR2 alarm status.
    pub const ALARM_STATUS: u16 = 1;
    /// IR3 output status.
    pub const OUTPUT_STATUS: u16 = 2;
    /// IR4 CO2 in ppm.
    pub const CO2: u16 = 3;
    /// IR22 PWM output (0x3FFF = 100 %).
    pub const PWM_OUTPUT: u16 = 21;
    /// IR26 sensor type ID high (low byte used).
    pub const TYPE_ID_HIGH: u16 = 25;
    /// IR27 sensor type ID low.
    pub const TYPE_ID_LOW: u16 = 26;
    /// IR28 memory map version.
    pub const MEMORY_MAP_VERSION: u16 = 27;
    /// IR29 firmware version (main in the high byte, sub in the low byte).
    pub const FIRMWARE: u16 = 28;
    /// IR30 sensor serial number high word.
    pub const SENSOR_ID_HIGH: u16 = 29;
    /// IR31 sensor serial number low word.
    pub const SENSOR_ID_LOW: u16 = 30;

    /// HR1 acknowledgement register (see [`crate::calibration`] bits).
    pub const ACK: u16 = 0;
    /// HR2 special command register (write only).
    pub const COMMAND: u16 = 1;
    /// HR32 ABC period in hours (0 suspends ABC; default 180).
    pub const ABC_PERIOD: u16 = 0x1F;

    /// Meter status: fatal error.
    pub const STATUS_FATAL: u16 = 1 << 0;
    /// Meter status: offset regulation error.
    pub const STATUS_OFFSET_REGULATION: u16 = 1 << 1;
    /// Meter status: algorithm error.
    pub const STATUS_ALGORITHM: u16 = 1 << 2;
    /// Meter status: output error.
    pub const STATUS_OUTPUT: u16 = 1 << 3;
    /// Meter status: self-diagnostics error.
    pub const STATUS_SELF_DIAGNOSTICS: u16 = 1 << 4;
    /// Meter status: out of range.
    pub const STATUS_OUT_OF_RANGE: u16 = 1 << 5;
    /// Meter status: memory error.
    pub const STATUS_MEMORY: u16 = 1 << 6;
}

/// Senseair Sunrise / Sunlight register addresses (register number − 1).
pub mod sunrise {
    /// IR1 error status (see the `STATUS_*` bits).
    pub const ERROR_STATUS: u16 = 0x00;
    /// IR4 filtered, pressure-compensated CO2 (signed, ppm).
    pub const CO2: u16 = 0x03;
    /// IR5 chip temperature (signed, 0.01 °C).
    pub const TEMPERATURE: u16 = 0x04;
    /// IR7 measurement count (0–255).
    pub const MEASUREMENT_COUNT: u16 = 0x06;
    /// IR8 time in the current measurement cycle (units of 2 s).
    pub const CYCLE_TIME: u16 = 0x07;
    /// IR24 firmware type.
    pub const FIRMWARE_TYPE: u16 = 0x17;
    /// IR29 firmware revision (main in the high byte, sub in the low byte).
    pub const FIRMWARE: u16 = 0x1C;
    /// IR30 sensor ID high word.
    pub const SENSOR_ID_HIGH: u16 = 0x1D;
    /// IR31 sensor ID low word.
    pub const SENSOR_ID_LOW: u16 = 0x1E;

    /// HR1 calibration status (see [`crate::calibration`] bits).
    pub const CALIBRATION_STATUS: u16 = 0x00;
    /// HR2 calibration command.
    pub const CALIBRATION_COMMAND: u16 = 0x01;
    /// HR3 calibration target (ppm) for target calibration.
    pub const CALIBRATION_TARGET: u16 = 0x02;
    /// HR4 measured concentration override (32767 = none).
    pub const CONCENTRATION_OVERRIDE: u16 = 0x03;
    /// HR10 start single measurement.
    pub const START_SINGLE: u16 = 0x09;
    /// HR11 measurement mode (EE): 0 continuous, 1 single; needs a reset.
    pub const MEASUREMENT_MODE: u16 = 0x0A;
    /// HR12 measurement period in s (EE, 2–65534); needs a reset.
    pub const MEASUREMENT_PERIOD: u16 = 0x0B;
    /// HR13 number of samples (EE, 1–1024); needs a reset.
    pub const NUMBER_OF_SAMPLES: u16 = 0x0C;
    /// HR14 ABC period in hours (EE, 1–65534; 0 or 65535 disables).
    pub const ABC_PERIOD: u16 = 0x0D;
    /// HR15 clear error status (write any value).
    pub const CLEAR_ERROR_STATUS: u16 = 0x0E;
    /// HR16 ABC / background calibration target in ppm (EE); needs a reset.
    pub const ABC_TARGET: u16 = 0x0F;
    /// HR17 static IIR filter parameter (EE, 2–10).
    pub const IIR_FILTER: u16 = 0x10;
    /// HR18 SCR: write [`SCR_RESET`] to restart the sensor.
    pub const SCR: u16 = 0x11;
    /// HR19 meter control bits (EE, see the `METER_*` bits).
    pub const METER_CONTROL: u16 = 0x12;
    /// HR20 Modbus address (EE, 1–247); needs a reset.
    pub const ADDRESS: u16 = 0x13;
    /// HR34 start single measurement (mirror of HR10, first of the state
    /// write block).
    pub const START_SINGLE_MIRROR: u16 = 0x21;
    /// HR35 ABC time: first of the [`STATE_LEN`] state registers
    /// (HR35 ABC time, HR36–39 ABC parameters, HR40–46 filter parameters).
    pub const STATE: u16 = 0x22;
    /// Number of state registers.
    pub const STATE_LEN: usize = 12;
    /// HR47 barometric pressure (0.1 hPa, 3000–13000).
    pub const PRESSURE: u16 = 0x2E;
    /// HR48 ABC barometric pressure (state when ABC and pressure
    /// compensation are both enabled).
    pub const ABC_PRESSURE: u16 = 0x2F;

    /// SCR value that resets the sensor.
    pub const SCR_RESET: u16 = 0xFF;

    /// Meter control: nRDY disabled.
    pub const METER_NRDY_DISABLE: u16 = 1 << 0;
    /// Meter control: ABC disabled.
    pub const METER_ABC_DISABLE: u16 = 1 << 1;
    /// Meter control: static IIR filter disabled.
    pub const METER_STATIC_IIR_DISABLE: u16 = 1 << 2;
    /// Meter control: dynamic IIR filter disabled.
    pub const METER_DYNAMIC_IIR_DISABLE: u16 = 1 << 3;
    /// Meter control: pressure compensation disabled (default set).
    pub const METER_PRESSURE_COMPENSATION_DISABLE: u16 = 1 << 4;
    /// Meter control: nRDY not inverted, i.e. high during measurement
    /// (default set).
    pub const METER_NRDY_INVERT_DISABLE: u16 = 1 << 5;

    /// Error status: fatal error.
    pub const STATUS_FATAL: u16 = 1 << 0;
    /// Error status: communication error.
    pub const STATUS_COMMUNICATION: u16 = 1 << 1;
    /// Error status: algorithm error.
    pub const STATUS_ALGORITHM: u16 = 1 << 2;
    /// Error status: calibration error.
    pub const STATUS_CALIBRATION: u16 = 1 << 3;
    /// Error status: self-diagnostics error.
    pub const STATUS_SELF_DIAGNOSTICS: u16 = 1 << 4;
    /// Error status: out of range.
    pub const STATUS_OUT_OF_RANGE: u16 = 1 << 5;
    /// Error status: memory error.
    pub const STATUS_MEMORY: u16 = 1 << 6;
    /// Error status: no measurement completed since start-up.
    pub const STATUS_NO_MEASUREMENT: u16 = 1 << 7;
    /// Error status: low internal regulated voltage (data not valid).
    pub const STATUS_LOW_VOLTAGE: u16 = 1 << 8;
    /// Error status: measurement timeout.
    pub const STATUS_MEASUREMENT_TIMEOUT: u16 = 1 << 9;
    /// Error status: abnormal signal level.
    pub const STATUS_ABNORMAL_SIGNAL: u16 = 1 << 10;
    /// Error status: scale factor error.
    pub const STATUS_SCALE_FACTOR: u16 = 1 << 15;
}

/// Calibration commands and the status / acknowledgement bits.
pub mod calibration {
    /// Restore factory calibration (Sunrise only).
    pub const COMMAND_FACTORY: u16 = 0x7C02;
    /// Forced ABC calibration (Sunrise only).
    pub const COMMAND_FORCED_ABC: u16 = 0x7C03;
    /// Target calibration against HR3 (Sunrise only).
    pub const COMMAND_TARGET: u16 = 0x7C05;
    /// Background calibration (fresh air / ABC target, 400 ppm on the S8).
    pub const COMMAND_BACKGROUND: u16 = 0x7C06;
    /// Zero calibration (0 ppm / nitrogen).
    pub const COMMAND_ZERO: u16 = 0x7C07;

    /// Status: factory calibration restored (Sunrise).
    pub const STATUS_FACTORY: u16 = 1 << 2;
    /// Status: ABC calibration (Sunrise).
    pub const STATUS_ABC: u16 = 1 << 3;
    /// Status: target calibration (Sunrise).
    pub const STATUS_TARGET: u16 = 1 << 4;
    /// Status: background calibration performed (S8 CI6, Sunrise).
    pub const STATUS_BACKGROUND: u16 = 1 << 5;
    /// Status: zero / nitrogen calibration performed (S8 CI7, Sunrise).
    pub const STATUS_ZERO: u16 = 1 << 6;
}

/// Sensor family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Model {
    /// Senseair S8 (function 06 writes).
    S8,
    /// Senseair Sunrise / Sunlight (function 0x10 writes).
    Sunrise,
}

impl Model {
    /// Factory-default Modbus address.
    pub const fn default_address(self) -> u8 {
        match self {
            Model::S8 => S8_ADDRESS,
            Model::Sunrise => SUNRISE_ADDRESS,
        }
    }

    /// Most registers one read request may cover.
    pub const fn max_read_registers(self) -> u16 {
        match self {
            Model::S8 => 17,
            Model::Sunrise => MAX_READ_REGS as u16,
        }
    }

    /// Status bits that make the CO2 value invalid.
    pub const fn invalid_status_mask(self) -> u16 {
        match self {
            Model::S8 => {
                s8::STATUS_FATAL
                    | s8::STATUS_OFFSET_REGULATION
                    | s8::STATUS_ALGORITHM
                    | s8::STATUS_SELF_DIAGNOSTICS
                    | s8::STATUS_OUT_OF_RANGE
                    | s8::STATUS_MEMORY
            }
            Model::Sunrise => {
                sunrise::STATUS_FATAL
                    | sunrise::STATUS_ALGORITHM
                    | sunrise::STATUS_SELF_DIAGNOSTICS
                    | sunrise::STATUS_OUT_OF_RANGE
                    | sunrise::STATUS_MEMORY
                    | sunrise::STATUS_NO_MEASUREMENT
                    | sunrise::STATUS_LOW_VOLTAGE
                    | sunrise::STATUS_MEASUREMENT_TIMEOUT
                    | sunrise::STATUS_ABNORMAL_SIGNAL
            }
        }
    }
}

/// An encoded Modbus RTU request (CRC included).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Request {
    buf: [u8; MAX_REQUEST_LEN],
    len: u8,
}

impl Request {
    fn new(address: u8, function: u8) -> Self {
        let mut buf = [0; MAX_REQUEST_LEN];
        buf[0] = address;
        buf[1] = function;
        Self { buf, len: 2 }
    }

    fn push(&mut self, byte: u8) {
        self.buf[usize::from(self.len)] = byte;
        self.len += 1;
    }

    fn push_u16(&mut self, value: u16) {
        for b in value.to_be_bytes() {
            self.push(b);
        }
    }

    fn finish(mut self) -> Self {
        let crc = crc16_modbus(self.as_bytes());
        for b in crc.to_le_bytes() {
            self.push(b);
        }
        self
    }

    /// Bytes to transmit.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..usize::from(self.len)]
    }

    /// Target address.
    pub fn address(&self) -> u8 {
        self.buf[0]
    }

    /// Function code.
    pub fn function(&self) -> u8 {
        self.buf[1]
    }
}

fn read_unchecked(address: u8, function: u8, start: u16, count: u16) -> Request {
    let mut r = Request::new(address, function);
    r.push_u16(start);
    r.push_u16(count);
    r.finish()
}

fn read_request(address: u8, function: u8, start: u16, count: u16) -> Option<Request> {
    (1..=MAX_READ_REGS as u16)
        .contains(&count)
        .then(|| read_unchecked(address, function, start, count))
}

/// Function 04: read `count` (1–32) input registers from `start`.
pub fn read_input_registers(address: u8, start: u16, count: u16) -> Option<Request> {
    read_request(address, function::READ_INPUT, start, count)
}

/// Function 03: read `count` (1–32) holding registers from `start`.
pub fn read_holding_registers(address: u8, start: u16, count: u16) -> Option<Request> {
    read_request(address, function::READ_HOLDING, start, count)
}

/// Function 06: write one holding register.
pub fn write_single_register(address: u8, register: u16, value: u16) -> Request {
    let mut r = Request::new(address, function::WRITE_SINGLE);
    r.push_u16(register);
    r.push_u16(value);
    r.finish()
}

/// Function 0x10: write 1–16 consecutive holding registers from `start`.
pub fn write_multiple_registers(address: u8, start: u16, values: &[u16]) -> Option<Request> {
    if values.is_empty() || values.len() > MAX_WRITE_REGS {
        return None;
    }
    let mut r = Request::new(address, function::WRITE_MULTIPLE);
    r.push_u16(start);
    r.push_u16(values.len() as u16);
    r.push((values.len() * 2) as u8);
    for v in values {
        r.push_u16(*v);
    }
    Some(r.finish())
}

/// Calibration to trigger with [`Device::calibrate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Calibration {
    /// Background calibration: S8 against 400 ppm fresh air, Sunrise
    /// against the ABC target (HR16).
    Background,
    /// Zero calibration against 0 ppm (nitrogen).
    Zero,
    /// Target calibration against HR3 (Sunrise only).
    Target,
    /// Restore factory calibration (Sunrise only).
    Factory,
    /// Forced ABC calibration (Sunrise only, ABC must be enabled).
    ForcedAbc,
}

impl Calibration {
    /// HR2 command value.
    pub const fn command(self) -> u16 {
        match self {
            Calibration::Background => calibration::COMMAND_BACKGROUND,
            Calibration::Zero => calibration::COMMAND_ZERO,
            Calibration::Target => calibration::COMMAND_TARGET,
            Calibration::Factory => calibration::COMMAND_FACTORY,
            Calibration::ForcedAbc => calibration::COMMAND_FORCED_ABC,
        }
    }
}

/// Sunrise / Sunlight state registers HR35–HR46, read after each single
/// measurement and written back with the next start command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SunriseState(pub [u16; sunrise::STATE_LEN]);

impl SunriseState {
    /// Decode a read of HR35–HR46.
    pub fn decode(regs: &[u16]) -> Option<Self> {
        regs.try_into().ok().map(Self)
    }

    /// Hours since the last ABC calibration (HR35).
    pub const fn abc_time_hours(&self) -> u16 {
        self.0[0]
    }

    /// Add the hours spent powered down to the ABC time; the host must do
    /// this every hour while ABC is enabled in single-measurement mode.
    pub fn add_abc_hours(&mut self, hours: u16) {
        self.0[0] = self.0[0].saturating_add(hours);
    }
}

/// Model + address: encodes every supported request.
///
/// Methods return `None` (mapped to [`Error::Unsupported`] by
/// [`Senseair::send`]) for out-of-range arguments or requests the model does
/// not implement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Device {
    /// Sensor family.
    pub model: Model,
    /// Modbus address.
    pub address: u8,
}

impl Device {
    /// S8 at [`S8_ADDRESS`].
    pub const fn s8() -> Self {
        Self {
            model: Model::S8,
            address: S8_ADDRESS,
        }
    }

    /// Sunrise / Sunlight at [`SUNRISE_ADDRESS`].
    pub const fn sunrise() -> Self {
        Self {
            model: Model::Sunrise,
            address: SUNRISE_ADDRESS,
        }
    }

    fn sunrise_only(&self) -> Option<()> {
        (self.model == Model::Sunrise).then_some(())
    }

    /// Read `count` input registers from `start`.
    pub fn read_input(&self, start: u16, count: u16) -> Option<Request> {
        if count > self.model.max_read_registers() {
            return None;
        }
        read_input_registers(self.address, start, count)
    }

    /// Read `count` holding registers from `start`.
    pub fn read_holding(&self, start: u16, count: u16) -> Option<Request> {
        if count > self.model.max_read_registers() {
            return None;
        }
        read_holding_registers(self.address, start, count)
    }

    /// Write one holding register (function 06 on the S8, 0x10 on the
    /// Sunrise).
    pub fn write(&self, register: u16, value: u16) -> Request {
        match self.model {
            Model::S8 => write_single_register(self.address, register, value),
            Model::Sunrise => {
                let mut r = Request::new(self.address, function::WRITE_MULTIPLE);
                r.push_u16(register);
                r.push_u16(1);
                r.push(2);
                r.push_u16(value);
                r.finish()
            }
        }
    }

    /// Write consecutive holding registers (Sunrise only).
    pub fn write_many(&self, start: u16, values: &[u16]) -> Option<Request> {
        self.sunrise_only()?;
        write_multiple_registers(self.address, start, values)
    }

    /// IR1–IR4: status and CO2 (decode with [`Measurement::decode`]).
    pub fn read_measurement(&self) -> Request {
        read_unchecked(self.address, function::READ_INPUT, 0, 4)
    }

    /// IR29–IR31: firmware and serial number (decode with
    /// [`Info::decode`]).
    pub fn read_info(&self) -> Request {
        read_unchecked(self.address, function::READ_INPUT, s8::FIRMWARE, 3)
    }

    /// Read the ABC period (hours).
    pub fn read_abc_period(&self) -> Request {
        read_unchecked(self.address, function::READ_HOLDING, self.abc_register(), 1)
    }

    fn abc_register(&self) -> u16 {
        match self.model {
            Model::S8 => s8::ABC_PERIOD,
            Model::Sunrise => sunrise::ABC_PERIOD,
        }
    }

    /// Set the ABC period in hours; 0 disables (S8: suspends) ABC. On the
    /// Sunrise this is an EEPROM write, and ABC also needs
    /// [`sunrise::METER_ABC_DISABLE`] clear in HR19.
    pub fn set_abc_period(&self, hours: u16) -> Request {
        self.write(self.abc_register(), hours)
    }

    /// Clear the calibration status / acknowledgement register (HR1);
    /// recommended before every calibration.
    pub fn clear_calibration_status(&self) -> Request {
        self.write(0, 0)
    }

    /// Read the calibration status / acknowledgement register (HR1, see
    /// [`calibration`] bits).
    pub fn read_calibration_status(&self) -> Request {
        read_unchecked(self.address, function::READ_HOLDING, 0, 1)
    }

    /// Trigger a calibration (HR2). S8: background and zero only; read HR1
    /// after [`S8_CALIBRATION_US`]. Sunrise: performed on the next
    /// measurement.
    pub fn calibrate(&self, c: Calibration) -> Option<Request> {
        if self.model == Model::S8 && !matches!(c, Calibration::Background | Calibration::Zero) {
            return None;
        }
        Some(self.write(1, c.command()))
    }

    /// Sunrise: set the target-calibration concentration (HR3, ppm).
    pub fn set_calibration_target(&self, ppm: u16) -> Option<Request> {
        self.sunrise_only()?;
        Some(self.write(sunrise::CALIBRATION_TARGET, ppm))
    }

    /// Sunrise: continuous (`false`) or single (`true`) measurement mode
    /// (EEPROM; reset afterwards with [`Device::reset`]).
    pub fn set_measurement_mode(&self, single: bool) -> Option<Request> {
        self.sunrise_only()?;
        Some(self.write(sunrise::MEASUREMENT_MODE, u16::from(single)))
    }

    /// Sunrise: continuous-mode measurement period, 2–65534 s (EEPROM;
    /// needs a reset).
    pub fn set_measurement_period(&self, seconds: u16) -> Option<Request> {
        self.sunrise_only()?;
        (2..=65534)
            .contains(&seconds)
            .then(|| self.write(sunrise::MEASUREMENT_PERIOD, seconds))
    }

    /// Sunrise: samples per measurement, 1–1024, ≤ 300 ms each (EEPROM;
    /// needs a reset).
    pub fn set_number_of_samples(&self, samples: u16) -> Option<Request> {
        self.sunrise_only()?;
        (1..=1024)
            .contains(&samples)
            .then(|| self.write(sunrise::NUMBER_OF_SAMPLES, samples))
    }

    /// Sunrise: read the meter control bits (HR19).
    pub fn read_meter_control(&self) -> Option<Request> {
        self.sunrise_only()?;
        read_holding_registers(self.address, sunrise::METER_CONTROL, 1)
    }

    /// Sunrise: write the meter control bits (HR19, EEPROM); read-modify-
    /// write with [`Device::read_meter_control`].
    pub fn set_meter_control(&self, bits: u16) -> Option<Request> {
        self.sunrise_only()?;
        Some(self.write(sunrise::METER_CONTROL, bits))
    }

    /// Sunrise: barometric pressure for compensation, 3000–13000 (0.1 hPa).
    pub fn set_pressure(&self, deci_hpa: u16) -> Option<Request> {
        self.sunrise_only()?;
        (3000..=13000)
            .contains(&deci_hpa)
            .then(|| self.write(sunrise::PRESSURE, deci_hpa))
    }

    /// Sunrise: clear the error status register.
    pub fn clear_errors(&self) -> Option<Request> {
        self.sunrise_only()?;
        Some(self.write(sunrise::CLEAR_ERROR_STATUS, 0))
    }

    /// Sunrise: restart the sensor (SCR = 0xFF).
    pub fn reset(&self) -> Option<Request> {
        self.sunrise_only()?;
        Some(self.write(sunrise::SCR, sunrise::SCR_RESET))
    }

    /// Sunrise single-measurement mode: start a measurement, restoring the
    /// saved `state` (HR35–HR46) and optionally the pressure (HR47) in the
    /// same write. Without saved state only the start command is written;
    /// `pressure` then has to be written first with
    /// [`Device::set_pressure`] (`None` is returned).
    pub fn start_single(
        &self,
        state: Option<&SunriseState>,
        pressure_deci_hpa: Option<u16>,
    ) -> Option<Request> {
        self.sunrise_only()?;
        let Some(state) = state else {
            return pressure_deci_hpa
                .is_none()
                .then(|| self.write(sunrise::START_SINGLE_MIRROR, 1));
        };
        let mut values = [0u16; 2 + sunrise::STATE_LEN];
        values[0] = 1;
        values[1..=sunrise::STATE_LEN].copy_from_slice(&state.0);
        let n = match pressure_deci_hpa {
            None => 1 + sunrise::STATE_LEN,
            Some(p) if (3000..=13000).contains(&p) => {
                values[1 + sunrise::STATE_LEN] = p;
                2 + sunrise::STATE_LEN
            }
            Some(_) => return None,
        };
        write_multiple_registers(self.address, sunrise::START_SINGLE_MIRROR, &values[..n])
    }

    /// Sunrise: read the state registers HR35–HR46 (decode with
    /// [`SunriseState::decode`]).
    pub fn read_state(&self) -> Option<Request> {
        self.sunrise_only()?;
        read_holding_registers(self.address, sunrise::STATE, sunrise::STATE_LEN as u16)
    }
}

/// Decoded registers from a read reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Registers {
    regs: [u16; MAX_READ_REGS],
    len: u8,
}

impl Registers {
    /// Register values in request order.
    pub fn as_slice(&self) -> &[u16] {
        &self.regs[..usize::from(self.len)]
    }
}

/// A CRC-checked Modbus reply.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    buf: [u8; MAX_FRAME_LEN],
    len: u8,
}

impl Frame {
    /// Raw bytes, CRC included.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..usize::from(self.len)]
    }

    /// Sender address.
    pub fn address(&self) -> u8 {
        self.buf[0]
    }

    /// Function code (with [`function::EXCEPTION`] set for exceptions).
    pub fn function(&self) -> u8 {
        self.buf[1]
    }

    /// Exception code of an exception reply.
    pub fn exception(&self) -> Option<u8> {
        (self.buf[1] & function::EXCEPTION != 0).then_some(self.buf[2])
    }

    /// Register values of a function 03 / 04 reply.
    pub fn registers(&self) -> Option<Registers> {
        if !matches!(self.buf[1], function::READ_HOLDING | function::READ_INPUT) {
            return None;
        }
        let data = &self.buf[3..usize::from(self.len) - 2];
        let mut regs = [0u16; MAX_READ_REGS];
        for (r, c) in regs.iter_mut().zip(data.chunks_exact(2)) {
            *r = u16::from_be_bytes([c[0], c[1]]);
        }
        Some(Registers {
            regs,
            len: (data.len() / 2) as u8,
        })
    }

    /// `(register, value)` of a function 06 echo or `(start, count)` of a
    /// function 0x10 reply.
    pub fn write_echo(&self) -> Option<(u16, u16)> {
        if !matches!(
            self.buf[1],
            function::WRITE_SINGLE | function::WRITE_MULTIPLE
        ) {
            return None;
        }
        let b = &self.buf;
        Some((
            u16::from_be_bytes([b[2], b[3]]),
            u16::from_be_bytes([b[4], b[5]]),
        ))
    }
}

/// Byte-stream parser for the reply to one request: resynchronises on the
/// expected address and function code (or its exception code), derives the
/// length and only yields frames with a valid CRC.
#[derive(Debug, Clone)]
pub struct Parser {
    buf: [u8; MAX_FRAME_LEN],
    len: usize,
    address: u8,
    function: u8,
    crc_error: bool,
}

impl Parser {
    /// Parser for replies from `address` to `function`.
    pub const fn new(address: u8, function: u8) -> Self {
        Self {
            buf: [0; MAX_FRAME_LEN],
            len: 0,
            address,
            function,
            crc_error: false,
        }
    }

    /// Parser for the reply to `request`.
    pub fn for_request(request: &Request) -> Self {
        Self::new(request.address(), request.function())
    }

    /// Drop any partial frame and the error flag.
    pub fn reset(&mut self) {
        self.len = 0;
        self.crc_error = false;
    }

    /// `true` once if a complete frame with a bad CRC was seen.
    pub fn take_checksum_error(&mut self) -> bool {
        core::mem::take(&mut self.crc_error)
    }

    fn frame_len(&self) -> Option<usize> {
        if self.len < 2 {
            return None;
        }
        if self.buf[1] & function::EXCEPTION != 0 {
            return Some(5);
        }
        match self.buf[1] {
            function::READ_HOLDING | function::READ_INPUT if self.len >= 3 => {
                Some(5 + usize::from(self.buf[2]))
            }
            function::READ_HOLDING | function::READ_INPUT => None,
            _ => Some(8),
        }
    }

    fn rescan(&mut self, bytes: &[u8]) -> Option<Frame> {
        let mut copy = [0u8; MAX_FRAME_LEN];
        copy[..bytes.len()].copy_from_slice(bytes);
        self.len = 0;
        let mut found = None;
        for b in &copy[..bytes.len()] {
            if let Some(f) = self.feed(*b) {
                found = Some(f);
            }
        }
        found
    }

    /// Feed one received byte.
    pub fn feed(&mut self, byte: u8) -> Option<Frame> {
        match self.len {
            0 if byte != self.address => return None,
            1 if byte != self.function && byte != self.function | function::EXCEPTION => {
                self.len = 0;
                return self.feed(byte);
            }
            2 if matches!(self.buf[1], function::READ_HOLDING | function::READ_INPUT)
                && (byte == 0 || byte % 2 != 0 || usize::from(byte) > 2 * MAX_READ_REGS) =>
            {
                let old = [self.buf[1], byte];
                return self.rescan(&old);
            }
            _ => {}
        }
        self.buf[self.len] = byte;
        self.len += 1;
        let n = self.frame_len()?;
        if self.len < n {
            return None;
        }
        self.len = 0;
        let crc = crc16_modbus(&self.buf[..n - 2]);
        if crc.to_le_bytes() == [self.buf[n - 2], self.buf[n - 1]] {
            return Some(Frame {
                buf: self.buf,
                len: n as u8,
            });
        }
        self.crc_error = true;
        let old = self.buf;
        self.rescan(&old[1..n])
    }
}

/// Status and CO2 from IR1–IR4.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Model the status bits belong to.
    pub model: Model,
    /// IR1: S8 meter status ([`s8`] `STATUS_*`) or Sunrise error status
    /// ([`sunrise`] `STATUS_*`).
    pub status: u16,
    /// IR4: CO2 in ppm (signed on the Sunrise).
    pub co2_ppm: i16,
}

impl Measurement {
    /// Decode IR1–IR4.
    pub fn decode(model: Model, regs: &[u16]) -> Option<Self> {
        match *regs {
            [status, _, _, co2] => Some(Self {
                model,
                status,
                co2_ppm: co2 as i16,
            }),
            _ => None,
        }
    }

    /// `true` if no status bit invalidates the CO2 value and it is not
    /// negative.
    pub const fn is_valid(&self) -> bool {
        self.status & self.model.invalid_status_mask() == 0 && self.co2_ppm >= 0
    }
}

impl Readings for Measurement {
    /// CO2 is omitted when [`Measurement::is_valid`] is `false`.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if self.is_valid() {
            f(Quantity::Co2(self.co2_ppm as u32));
        }
    }
}

/// Firmware and serial number from IR29–IR31 (same addresses on both
/// families).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Info {
    /// Firmware main revision.
    pub firmware_main: u8,
    /// Firmware sub revision.
    pub firmware_sub: u8,
    /// Sensor serial number / ID.
    pub sensor_id: u32,
}

impl Info {
    /// Decode IR29–IR31.
    pub fn decode(regs: &[u16]) -> Option<Self> {
        match *regs {
            [fw, hi, lo] => {
                let [firmware_main, firmware_sub] = fw.to_be_bytes();
                Some(Self {
                    firmware_main,
                    firmware_sub,
                    sensor_id: (u32::from(hi) << 16) | u32::from(lo),
                })
            }
            _ => None,
        }
    }
}

/// S8: decode the 24-bit sensor type ID from IR26–IR27 (request with
/// `device.read_input(s8::TYPE_ID_HIGH, 2)`).
pub fn decode_s8_type_id(regs: &[u16]) -> Option<u32> {
    match *regs {
        [hi, lo] => Some((u32::from(hi & 0xFF) << 16) | u32::from(lo)),
        _ => None,
    }
}

fn finish<E>(found: Option<Frame>, parser: &mut Parser) -> Result<Frame, Error<E>> {
    let frame = match found {
        Some(f) => f,
        None if parser.take_checksum_error() => return Err(Error::Crc),
        None => return Err(Error::NotReady),
    };
    match frame.exception() {
        Some(exception::ILLEGAL_FUNCTION | exception::ILLEGAL_DATA_ADDRESS) => {
            Err(Error::Unsupported)
        }
        Some(_) => Err(Error::InvalidData),
        None => Ok(frame),
    }
}

fn registers_of<E>(frame: &Frame) -> Result<Registers, Error<E>> {
    frame.registers().ok_or(Error::InvalidData)
}

fn ack_of<E>(frame: &Frame) -> Result<(), Error<E>> {
    frame.write_echo().map(|_| ()).ok_or(Error::InvalidData)
}

fn state_of<E>(regs: &Registers) -> Result<SunriseState, Error<E>> {
    SunriseState::decode(regs.as_slice()).ok_or(Error::InvalidData)
}

fn info_of<E>(regs: &Registers) -> Result<Info, Error<E>> {
    Info::decode(regs.as_slice()).ok_or(Error::InvalidData)
}

fn measurement_of<E>(model: Model, regs: &Registers) -> Result<Measurement, Error<E>> {
    Measurement::decode(model, regs.as_slice()).ok_or(Error::InvalidData)
}

macro_rules! constructors {
    () => {
        /// Driver for `device`; no bus traffic.
        pub const fn new(uart: U, device: Device) -> Self {
            Self {
                uart,
                device,
                parser: Parser::new(device.address, function::READ_INPUT),
            }
        }

        /// Driver for an S8 at [`S8_ADDRESS`](crate::S8_ADDRESS); no bus
        /// traffic.
        pub const fn s8(uart: U) -> Self {
            Self::new(uart, Device::s8())
        }

        /// Driver for a Sunrise / Sunlight at
        /// [`SUNRISE_ADDRESS`](crate::SUNRISE_ADDRESS); no bus traffic.
        pub const fn sunrise(uart: U) -> Self {
            Self::new(uart, Device::sunrise())
        }

        /// Model and address (request encoders).
        pub const fn device(&self) -> Device {
            self.device
        }

        /// Give the UART back.
        pub fn release(self) -> U {
            self.uart
        }
    };
}

/// Blocking Modbus RTU driver (S8, Sunrise, Sunlight).
pub struct Senseair<U> {
    uart: U,
    device: Device,
    parser: Parser,
}

impl<U> Senseair<U> {
    constructors!();
}

impl<U> Senseair<U>
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
        Ok(())
    }

    /// Discard pending input and send `request` (from a [`Device`]
    /// encoder; `None` → [`Error::Unsupported`]). Returns the time to wait
    /// before reading the reply.
    pub fn send(&mut self, request: impl Into<Option<Request>>) -> Result<WaitUs, Error<U::Error>> {
        let request = request.into().ok_or(Error::Unsupported)?;
        self.drain()?;
        self.parser = Parser::for_request(&request);
        self.uart
            .write_all(request.as_bytes())
            .map_err(Error::bus)?;
        self.uart.flush().map_err(Error::bus)?;
        Ok(RESPONSE_US)
    }

    /// Feed all available bytes (bounded) to the parser and return the
    /// newest reply to the last request.
    pub fn poll_frame(&mut self) -> Result<Option<Frame>, Error<U::Error>> {
        let mut latest = None;
        let mut buf = [0u8; 16];
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

    /// Reply to the last request: `NotReady` if incomplete, `Crc` on a CRC
    /// error, `Unsupported` for an illegal function / address exception,
    /// `InvalidData` for other exceptions.
    pub fn read_reply(&mut self) -> Result<Frame, Error<U::Error>> {
        let found = self.poll_frame()?;
        finish(found, &mut self.parser)
    }

    /// Register values replying to a read request.
    pub fn read_registers(&mut self) -> Result<Registers, Error<U::Error>> {
        registers_of(&self.read_reply()?)
    }

    /// Wait for the reply to a write request.
    pub fn read_ack(&mut self) -> Result<(), Error<U::Error>> {
        ack_of(&self.read_reply()?)
    }

    /// Sunrise state after [`Device::read_state`].
    pub fn read_state(&mut self) -> Result<SunriseState, Error<U::Error>> {
        state_of(&self.read_registers()?)
    }

    /// Firmware and serial number after [`Device::read_info`].
    pub fn read_info(&mut self) -> Result<Info, Error<U::Error>> {
        info_of(&self.read_registers()?)
    }
}

impl<U> zigbee_sensor::blocking::Sensor for Senseair<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type BusError = U::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        self.send(self.device.read_measurement())
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
        measurement_of(self.device.model, &self.read_registers()?)
    }
}

/// Async twin over `embedded_io_async`.
pub mod asynch {
    use super::{
        Device, Frame, Info, MAX_DRAIN_BYTES, MAX_READ_BYTES, Measurement, Parser, RESPONSE_US,
        Registers, Request, SunriseState, ack_of, finish, function, info_of, measurement_of,
        registers_of, state_of,
    };
    use embedded_io_async::{Read, ReadReady, Write};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Senseair`].
    pub struct Senseair<U> {
        uart: U,
        device: Device,
        parser: Parser,
    }

    impl<U> Senseair<U> {
        constructors!();
    }

    impl<U: Read + ReadReady + Write> Senseair<U> {
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
            Ok(())
        }

        /// See [`super::Senseair::send`].
        pub async fn send(
            &mut self,
            request: impl Into<Option<Request>>,
        ) -> Result<WaitUs, Error<U::Error>> {
            let request = request.into().ok_or(Error::Unsupported)?;
            self.drain().await?;
            self.parser = Parser::for_request(&request);
            self.uart
                .write_all(request.as_bytes())
                .await
                .map_err(Error::bus)?;
            self.uart.flush().await.map_err(Error::bus)?;
            Ok(RESPONSE_US)
        }

        /// See [`super::Senseair::poll_frame`].
        pub async fn poll_frame(&mut self) -> Result<Option<Frame>, Error<U::Error>> {
            let mut latest = None;
            let mut buf = [0u8; 16];
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

        /// See [`super::Senseair::read_reply`].
        pub async fn read_reply(&mut self) -> Result<Frame, Error<U::Error>> {
            let found = self.poll_frame().await?;
            finish(found, &mut self.parser)
        }

        /// See [`super::Senseair::read_registers`].
        pub async fn read_registers(&mut self) -> Result<Registers, Error<U::Error>> {
            registers_of(&self.read_reply().await?)
        }

        /// See [`super::Senseair::read_ack`].
        pub async fn read_ack(&mut self) -> Result<(), Error<U::Error>> {
            ack_of(&self.read_reply().await?)
        }

        /// See [`super::Senseair::read_state`].
        pub async fn read_state(&mut self) -> Result<SunriseState, Error<U::Error>> {
            state_of(&self.read_registers().await?)
        }

        /// See [`super::Senseair::read_info`].
        pub async fn read_info(&mut self) -> Result<Info, Error<U::Error>> {
            info_of(&self.read_registers().await?)
        }
    }

    impl<U: Read + ReadReady + Write> zigbee_sensor::asynch::Sensor for Senseair<U> {
        type BusError = U::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
            self.send(self.device.read_measurement()).await
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
            measurement_of(self.device.model, &self.read_registers().await?)
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

    /// In-memory UART: `rx` is what the sensor sends, `tx` what we wrote;
    /// each write queues the next `replies` entry.
    #[derive(Default)]
    struct FakeUart {
        rx: VecDeque<u8>,
        tx: Vec<u8>,
        replies: VecDeque<Vec<u8>>,
    }

    impl FakeUart {
        fn replying(replies: &[&[u8]]) -> Self {
            Self {
                replies: replies.iter().map(|r| r.to_vec()).collect(),
                ..Self::default()
            }
        }
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
            if let Some(reply) = self.replies.pop_front() {
                self.rx.extend(reply);
            }
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

    /// TDE7318 3.3 state data (HR35–HR46).
    const STATE: SunriseState = SunriseState([
        0x0000, 0x0000, 0x0000, 0x7FFF, 0x0008, 0x0002, 0x0001, 0x0001, 0x97DC, 0x00F5, 0xFF64,
        0x00F5,
    ]);
    const STATE_REPLY: [u8; 29] = [
        0x68, 0x03, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0x00, 0x08, 0x00, 0x02,
        0x00, 0x01, 0x00, 0x01, 0x97, 0xDC, 0x00, 0xF5, 0xFF, 0x64, 0x00, 0xF5, 0x5A, 0xFB,
    ];
    /// TDE2067 example: status 0, CO2 400 ppm.
    const S8_READING: [u8; 13] = [
        0xFE, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x90, 0x16, 0xE6,
    ];
    /// TDE7318 3.1 example: status 0, CO2 1351 ppm.
    const SUNRISE_READING: [u8; 13] = [
        0x68, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x47, 0xB7, 0xF2,
    ];

    fn bytes(r: Option<Request>) -> Vec<u8> {
        r.unwrap().as_bytes().to_vec()
    }

    fn frames(address: u8, function: u8, data: &[u8]) -> Vec<Frame> {
        let mut p = Parser::new(address, function);
        data.iter().filter_map(|b| p.feed(*b)).collect()
    }

    fn one_frame(address: u8, function: u8, data: &[u8]) -> Frame {
        let f = frames(address, function, data);
        assert_eq!(f.len(), 1, "{data:02X?}");
        f[0].clone()
    }

    fn with_crc(body: &[u8]) -> Vec<u8> {
        let mut v = body.to_vec();
        v.extend_from_slice(&crc16_modbus(body).to_le_bytes());
        v
    }

    #[test]
    fn s8_requests_match_datasheet() {
        let d = Device::s8();
        let cases: [(Option<Request>, &[u8]); 8] = [
            (
                d.read_input(s8::CO2, 1),
                &[0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5],
            ),
            (
                Some(d.read_measurement()),
                &[0xFE, 0x04, 0x00, 0x00, 0x00, 0x04, 0xE5, 0xC6],
            ),
            (
                Some(d.clear_calibration_status()),
                &[0xFE, 0x06, 0x00, 0x00, 0x00, 0x00, 0x9D, 0xC5],
            ),
            (
                d.calibrate(Calibration::Background),
                &[0xFE, 0x06, 0x00, 0x01, 0x7C, 0x06, 0x6C, 0xC7],
            ),
            (
                Some(d.read_calibration_status()),
                &[0xFE, 0x03, 0x00, 0x00, 0x00, 0x01, 0x90, 0x05],
            ),
            (
                Some(d.read_abc_period()),
                &[0xFE, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xA1, 0xC3],
            ),
            (
                Some(d.set_abc_period(0)),
                &[0xFE, 0x06, 0x00, 0x1F, 0x00, 0x00, 0xAC, 0x03],
            ),
            (
                Some(d.set_abc_period(180)),
                &[0xFE, 0x06, 0x00, 0x1F, 0x00, 0xB4, 0xAC, 0x74],
            ),
        ];
        for (req, want) in cases {
            assert_eq!(bytes(req), want);
        }
        // Sunrise-only requests and over-long reads are rejected.
        assert!(d.calibrate(Calibration::Target).is_none());
        assert!(d.start_single(None, None).is_none());
        assert!(d.write_many(0, &[1]).is_none());
        assert!(d.read_input(0, 18).is_none());
        assert!(d.read_input(0, 17).is_some());
    }

    #[test]
    fn sunrise_requests_match_datasheet() {
        let d = Device::sunrise();
        let full: &[u8] = &[
            0x68, 0x10, 0x00, 0x21, 0x00, 0x0D, 0x1A, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x7F, 0xFF, 0x00, 0x08, 0x00, 0x02, 0x00, 0x01, 0x00, 0x01, 0x97, 0xDC, 0x00,
            0xF5, 0xFF, 0x64, 0x00, 0xF5, 0x07, 0x7B,
        ];
        let with_pressure: &[u8] = &[
            0x68, 0x10, 0x00, 0x21, 0x00, 0x0E, 0x1C, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x7F, 0xFF, 0x00, 0x08, 0x00, 0x02, 0x00, 0x01, 0x00, 0x01, 0x97, 0xDC, 0x00,
            0xF5, 0xFF, 0x64, 0x00, 0xF5, 0x29, 0x04, 0xDF, 0xB4,
        ];
        let cases: [(Option<Request>, &[u8]); 13] = [
            (
                Some(d.read_measurement()),
                &[0x68, 0x04, 0x00, 0x00, 0x00, 0x04, 0xF8, 0xF0],
            ),
            (
                d.set_measurement_mode(true),
                &[
                    0x68, 0x10, 0x00, 0x0A, 0x00, 0x01, 0x02, 0x00, 0x01, 0xA5, 0x68,
                ],
            ),
            (d.start_single(Some(&STATE), None), full),
            (d.start_single(Some(&STATE), Some(10500)), with_pressure),
            (
                d.start_single(None, None),
                &[
                    0x68, 0x10, 0x00, 0x21, 0x00, 0x01, 0x02, 0x00, 0x01, 0xA3, 0x73,
                ],
            ),
            (
                d.read_state(),
                &[0x68, 0x03, 0x00, 0x22, 0x00, 0x0C, 0xEC, 0xFC],
            ),
            (
                d.read_meter_control(),
                &[0x68, 0x03, 0x00, 0x12, 0x00, 0x01, 0x2D, 0x36],
            ),
            (
                d.set_meter_control(0x00F0),
                &[
                    0x68, 0x10, 0x00, 0x12, 0x00, 0x01, 0x02, 0x00, 0xF0, 0x67, 0x34,
                ],
            ),
            (
                Some(d.read_abc_period()),
                &[0x68, 0x03, 0x00, 0x0D, 0x00, 0x01, 0x1C, 0xF0],
            ),
            (
                Some(d.set_abc_period(200)),
                &[
                    0x68, 0x10, 0x00, 0x0D, 0x00, 0x01, 0x02, 0x00, 0xC8, 0x64, 0x89,
                ],
            ),
            (
                d.calibrate(Calibration::Background),
                &[
                    0x68, 0x10, 0x00, 0x01, 0x00, 0x01, 0x02, 0x7C, 0x06, 0xC5, 0x11,
                ],
            ),
            (
                d.set_calibration_target(500),
                &[
                    0x68, 0x10, 0x00, 0x02, 0x00, 0x01, 0x02, 0x01, 0xF4, 0x65, 0xF7,
                ],
            ),
            (
                Some(d.read_calibration_status()),
                &[0x68, 0x03, 0x00, 0x00, 0x00, 0x01, 0x8D, 0x33],
            ),
        ];
        for (req, want) in cases {
            assert_eq!(bytes(req), want);
        }
        assert!(d.start_single(None, Some(10500)).is_none());
        assert!(d.start_single(Some(&STATE), Some(2999)).is_none());
        assert!(d.set_pressure(13001).is_none());
        assert!(d.set_measurement_period(1).is_none());
        assert!(d.set_number_of_samples(1025).is_none());
        assert!(d.read_input(0, 33).is_none());
        assert!(write_multiple_registers(0x68, 0, &[0; 17]).is_none());
    }

    #[test]
    fn parser_decodes_replies() {
        let f = one_frame(0xFE, 0x04, &[0xFE, 0x04, 0x02, 0x01, 0x90, 0xAC, 0xD8]);
        assert_eq!(f.registers().unwrap().as_slice(), &[400]);
        let f = one_frame(0xFE, 0x03, &[0xFE, 0x03, 0x02, 0x00, 0xB4, 0xAC, 0x27]);
        assert_eq!(f.registers().unwrap().as_slice(), &[180]);
        let f = one_frame(0xFE, 0x03, &[0xFE, 0x03, 0x02, 0x00, 0x20, 0xAD, 0x88]);
        assert_eq!(
            f.registers().unwrap().as_slice(),
            &[calibration::STATUS_BACKGROUND]
        );
        let f = one_frame(0x68, 0x03, &[0x68, 0x03, 0x02, 0x00, 0xF2, 0x65, 0xC8]);
        assert_eq!(f.registers().unwrap().as_slice(), &[0xF2]);
        let f = one_frame(0x68, 0x03, &STATE_REPLY);
        assert_eq!(
            SunriseState::decode(f.registers().unwrap().as_slice()),
            Some(STATE)
        );

        // S8 function 06 replies echo the request.
        let echo = [0xFE, 0x06, 0x00, 0x01, 0x7C, 0x06, 0x6C, 0xC7];
        assert_eq!(one_frame(0xFE, 0x06, &echo).write_echo(), Some((1, 0x7C06)));
        for (reply, start, count) in [
            ([0x68, 0x10, 0x00, 0x0A, 0x00, 0x01, 0x28, 0xF2], 0x0A, 1),
            ([0x68, 0x10, 0x00, 0x21, 0x00, 0x0D, 0x58, 0xFF], 0x21, 13),
            ([0x68, 0x10, 0x00, 0x21, 0x00, 0x0E, 0x18, 0xFE], 0x21, 14),
            ([0x68, 0x10, 0x00, 0x21, 0x00, 0x01, 0x58, 0xFA], 0x21, 1),
            ([0x68, 0x10, 0x00, 0x0D, 0x00, 0x01, 0x99, 0x33], 0x0D, 1),
            ([0x68, 0x10, 0x00, 0x2E, 0x00, 0x01, 0x68, 0xF9], 0x2E, 1),
            ([0x68, 0x10, 0x00, 0x01, 0x00, 0x01, 0x59, 0x30], 0x01, 1),
            ([0x68, 0x10, 0x00, 0x12, 0x00, 0x01, 0xA8, 0xF5], 0x12, 1),
        ] {
            assert_eq!(
                one_frame(0x68, 0x10, &reply).write_echo(),
                Some((start, count))
            );
        }

        // Noise, a wrong address and a corrupted copy before the real reply.
        let mut stream = std::vec![0x00, 0x68, 0x04, 0x07, 0x67, 0x04];
        let mut bad = SUNRISE_READING;
        bad[10] ^= 1;
        stream.extend_from_slice(&bad);
        stream.extend_from_slice(&SUNRISE_READING);
        let mut p = Parser::new(0x68, 0x04);
        let found: Vec<Frame> = stream.iter().filter_map(|b| p.feed(*b)).collect();
        assert_eq!(found.len(), 1);
        assert!(p.take_checksum_error());
        assert_eq!(
            Measurement::decode(Model::Sunrise, found[0].registers().unwrap().as_slice())
                .unwrap()
                .co2_ppm,
            1351
        );

        let exc = one_frame(0x68, 0x04, &with_crc(&[0x68, 0x84, 0x02]));
        assert_eq!(exc.exception(), Some(exception::ILLEGAL_DATA_ADDRESS));
        assert!(exc.registers().is_none());
    }

    #[test]
    fn measurement_validity_and_info() {
        let ok = Measurement::decode(Model::S8, &[0, 0, 0, 400]).unwrap();
        let mut got = Vec::new();
        ok.for_each(&mut |q| got.push(q));
        assert_eq!(got, [Quantity::Co2(400)]);
        // Output error does not invalidate the S8 reading; out of range does.
        assert!(
            Measurement::decode(Model::S8, &[s8::STATUS_OUTPUT, 0, 0, 400])
                .unwrap()
                .is_valid()
        );
        assert!(
            !Measurement::decode(Model::S8, &[s8::STATUS_OUT_OF_RANGE, 0, 0, 400])
                .unwrap()
                .is_valid()
        );
        // Sunrise: no measurement completed yet, negative concentration.
        let first = [sunrise::STATUS_NO_MEASUREMENT, 0, 0, 0];
        assert!(
            !Measurement::decode(Model::Sunrise, &first)
                .unwrap()
                .is_valid()
        );
        assert!(
            !Measurement::decode(Model::Sunrise, &[0, 0, 0, 0xFFF6])
                .unwrap()
                .is_valid()
        );
        let cal = [sunrise::STATUS_CALIBRATION, 0, 0, 500];
        assert!(
            Measurement::decode(Model::Sunrise, &cal)
                .unwrap()
                .is_valid()
        );

        let info = Info::decode(&[0x0410, 0x0123, 0x4567]).unwrap();
        assert_eq!((info.firmware_main, info.firmware_sub), (4, 16));
        assert_eq!(info.sensor_id, 0x0123_4567);
        assert_eq!(decode_s8_type_id(&[0xAB01, 0x0203]), Some(0x01_0203));

        let mut s = STATE;
        s.add_abc_hours(5);
        assert_eq!(s.abc_time_hours(), 5);
    }

    #[test]
    fn s8_driver() {
        let ack = [0xFE, 0x06, 0x00, 0x1F, 0x00, 0xB4, 0xAC, 0x74];
        let mut uart = FakeUart::replying(&[&S8_READING, &ack, &with_crc(&[0xFE, 0x83, 0x02])]);
        uart.rx.extend([0x12, 0x34]);
        let mut co2 = Senseair::s8(&mut uart);
        assert_eq!(co2.read_measurement(), Err(Error::NotReady));
        assert_eq!(co2.start_measurement(), Ok(RESPONSE_US));
        let m = co2.read_measurement().unwrap();
        assert_eq!((m.status, m.co2_ppm), (0, 400));

        assert_eq!(co2.send(co2.device().set_abc_period(180)), Ok(RESPONSE_US));
        assert_eq!(co2.read_ack(), Ok(()));
        co2.send(co2.device().read_holding(0x20, 1)).unwrap();
        assert_eq!(co2.read_registers(), Err(Error::Unsupported));
        assert_eq!(
            co2.send(co2.device().calibrate(Calibration::Factory)),
            Err(Error::Unsupported)
        );

        let mut tx = Vec::new();
        tx.extend_from_slice(Device::s8().read_measurement().as_bytes());
        tx.extend_from_slice(&ack);
        tx.extend_from_slice(Device::s8().read_holding(0x20, 1).unwrap().as_bytes());
        assert_eq!(uart.tx, tx);

        // A corrupted reply reports a CRC error.
        let mut bad = S8_READING;
        bad[12] ^= 0xFF;
        let mut uart = FakeUart::replying(&[&bad]);
        let mut co2 = Senseair::s8(&mut uart);
        co2.start_measurement().unwrap();
        assert_eq!(co2.read_measurement(), Err(Error::Crc));
    }

    #[test]
    fn sunrise_single_measurement_sequence() {
        let start_ack = [0x68, 0x10, 0x00, 0x21, 0x00, 0x0D, 0x58, 0xFF];
        let mut uart = FakeUart::replying(&[&start_ack, &SUNRISE_READING, &STATE_REPLY]);
        let mut co2 = Senseair::sunrise(&mut uart);
        let d = co2.device();
        assert_eq!(
            co2.send(d.start_single(Some(&STATE), None)),
            Ok(RESPONSE_US)
        );
        assert_eq!(co2.read_ack(), Ok(()));
        co2.start_measurement().unwrap();
        assert_eq!(co2.read_measurement().unwrap().co2_ppm, 1351);
        co2.send(d.read_state()).unwrap();
        assert_eq!(co2.read_state(), Ok(STATE));
        assert_eq!(co2.read_state(), Err(Error::NotReady));
    }

    #[test]
    fn async_driver() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let mut uart = FakeUart::replying(&[&SUNRISE_READING, &STATE_REPLY]);
        let mut co2 = asynch::Senseair::sunrise(&mut uart);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(co2.measure(&mut delay)).unwrap().co2_ppm, 1351);
        assert_eq!(delay.0, RESPONSE_US);
        block_on(co2.send(co2.device().read_state())).unwrap();
        assert_eq!(block_on(co2.read_state()), Ok(STATE));
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
