//! Sensirion SEN6x environmental sensor nodes (I2C): SEN60 (PM, 0x6C) and
//! SEN62 / SEN63C / SEN65 / SEN66 / SEN68 / SEN69C (0x6B) with PM1.0 /
//! PM2.5 / PM4.0 / PM10, RH and temperature, VOC and NOx index, CO2 and
//! formaldehyde depending on the variant.
//!
//! Each variant has its own `Read Measured Values` command and layout;
//! [`Variant`] selects them. Start / stop, data-ready flag, number
//! concentrations, device status (and read-and-clear), product name
//! (`probe`), serial number, firmware version, reset, fan cleaning, SHT
//! heater, temperature offset / acceleration, VOC algorithm state and the
//! CO2 settings (forced recalibration, ASC, ambient pressure, altitude,
//! factory reset) are supported. Commands a variant lacks return
//! [`Error::Unsupported`] without bus traffic.
//!
//! # Measurement through the [`Sensor`](zigbee_sensor::blocking::Sensor) trait
//!
//! The first `start_measurement` starts continuous measurement and returns
//! 1.1 s (time to the first sample); the next read is [`Error::NotReady`].
//! Later starts send the variant's `Read Measured Values` and return its
//! execution time (20 ms, SEN60 1 ms); the read then returns the latest
//! sample (the sensor is free-running at 1 Hz). The SEN60 NACKs when no
//! new sample is buffered, which also gives [`Error::NotReady`].
//!
//! Outputs the variant does not have, and values the sensor flags as
//! unknown (0xFFFF for unsigned, 0x7FFF for signed values — e.g. NOx during
//! its first 11 s, CO2 during its start-up, HCHO during its first 60 s),
//! are `None`.
//!
//! The SEN6x has no sleep mode (idle 1.7–4.6 mA), so there is no
//! `LowPower` implementation; switch the 3.3 V supply to save power.
//!
//! References: Sensirion "Datasheet SEN6x", version 1.0 (D1) for SEN62 –
//! SEN69C, and version 0.9 (November 2024, D1, distribution engineering
//! samples) for the SEN60 commands.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address of SEN62 / SEN63C / SEN65 / SEN66 / SEN68 / SEN69C.
pub const ADDRESS: u8 = 0x6B;
/// I2C address of the SEN60.
pub const ADDRESS_SEN60: u8 = 0x6C;

mod cmd {
    pub const START_MEASUREMENT: u16 = 0x0021;
    pub const STOP_MEASUREMENT: u16 = 0x0104;
    pub const DATA_READY: u16 = 0x0202;
    pub const READ_SEN62: u16 = 0x04A3;
    pub const READ_SEN63C: u16 = 0x0471;
    pub const READ_SEN65: u16 = 0x0446;
    pub const READ_SEN66: u16 = 0x0300;
    pub const READ_SEN68: u16 = 0x0467;
    pub const READ_SEN69C: u16 = 0x04B5;
    pub const NUMBER_CONCENTRATION: u16 = 0x0316;
    pub const TEMPERATURE_OFFSET: u16 = 0x60B2;
    pub const TEMPERATURE_ACCELERATION: u16 = 0x6100;
    pub const PRODUCT_NAME: u16 = 0xD014;
    pub const SERIAL_NUMBER: u16 = 0xD033;
    pub const DEVICE_STATUS: u16 = 0xD206;
    pub const READ_AND_CLEAR_STATUS: u16 = 0xD210;
    pub const VERSION: u16 = 0xD100;
    pub const RESET: u16 = 0xD304;
    pub const FAN_CLEANING: u16 = 0x5607;
    pub const ACTIVATE_SHT_HEATER: u16 = 0x6765;
    pub const SHT_HEATER_MEASUREMENTS: u16 = 0x6790;
    pub const VOC_STATE: u16 = 0x6181;
    pub const FORCED_RECALIBRATION: u16 = 0x6707;
    pub const CO2_FACTORY_RESET: u16 = 0x6754;
    pub const ASC: u16 = 0x6711;
    pub const AMBIENT_PRESSURE: u16 = 0x6720;
    pub const ALTITUDE: u16 = 0x6736;

    pub const SEN60_START: u16 = 0x2152;
    pub const SEN60_STOP: u16 = 0x3F86;
    pub const SEN60_DATA_READY: u16 = 0xE4B8;
    pub const SEN60_READ: u16 = 0xEC05;
    pub const SEN60_SERIAL_NUMBER: u16 = 0x3682;
    pub const SEN60_DEVICE_STATUS: u16 = 0xE00B;
    pub const SEN60_RESET: u16 = 0x3F8D;
    pub const SEN60_FAN_CLEANING: u16 = 0x3730;
}

/// Start measurement execution time.
pub const START_WAIT_US: WaitUs = 50_000;
/// Stop measurement execution time (wait before the next start).
pub const STOP_WAIT_US: WaitUs = 1_400_000;
/// Execution time of the other commands (wait before reading a response).
pub const COMMAND_WAIT_US: WaitUs = 20_000;
/// Device reset execution time.
pub const RESET_WAIT_US: WaitUs = 1_200_000;
/// SEN60 command execution time (start, reads, reset, fan cleaning).
pub const SEN60_COMMAND_WAIT_US: WaitUs = 1_000;
/// SEN60 stop measurement execution time.
pub const SEN60_STOP_WAIT_US: WaitUs = 1_000_000;
/// Time from start to the first sample.
pub const FIRST_SAMPLE_US: WaitUs = 1_100_000;
/// SHT heater execution time (older firmware; newer firmware: 20 ms).
pub const HEATER_WAIT_US: WaitUs = 1_300_000;
/// Forced CO2 recalibration execution time.
pub const FRC_WAIT_US: WaitUs = 500_000;
/// CO2 sensor factory reset execution time.
pub const CO2_FACTORY_RESET_WAIT_US: WaitUs = 1_400_000;

/// Device status: fan speed more than 10 % off target (warning).
pub const STATUS_SPEED: u32 = 1 << 21;
/// Device status: CO2 sensor error (SEN63C, SEN69C).
pub const STATUS_CO2_1: u32 = 1 << 12;
/// Device status: PM sensor error.
pub const STATUS_PM: u32 = 1 << 11;
/// Device status: formaldehyde sensor error (SEN68, SEN69C).
pub const STATUS_HCHO: u32 = 1 << 10;
/// Device status: CO2 sensor error (SEN66).
pub const STATUS_CO2_2: u32 = 1 << 9;
/// Device status: gas (VOC / NOx) sensor error.
pub const STATUS_GAS: u32 = 1 << 7;
/// Device status: RH&T sensor error.
pub const STATUS_RHT: u32 = 1 << 6;
/// Device status: fan blocked or broken (also SEN60).
pub const STATUS_FAN: u32 = 1 << 4;
/// SEN60 device status: fan speed warning.
pub const STATUS_SEN60_SPEED: u32 = 1 << 1;

/// Product variant; selects the address, commands and outputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// PM mass and number concentration (address 0x6C).
    Sen60,
    /// PM, RH/T.
    Sen62,
    /// PM, RH/T, CO2.
    Sen63c,
    /// PM, RH/T, VOC and NOx index.
    Sen65,
    /// PM, RH/T, VOC and NOx index, CO2.
    Sen66,
    /// PM, RH/T, VOC and NOx index, formaldehyde.
    Sen68,
    /// PM, RH/T, VOC and NOx index, formaldehyde, CO2.
    Sen69c,
}

impl Variant {
    /// I2C address of this variant.
    pub const fn address(self) -> u8 {
        match self {
            Self::Sen60 => ADDRESS_SEN60,
            _ => ADDRESS,
        }
    }

    /// `true` for the SEN60, which has its own command set.
    pub const fn is_sen60(self) -> bool {
        matches!(self, Self::Sen60)
    }

    /// `true` if the variant measures the VOC and NOx index.
    pub const fn has_gas(self) -> bool {
        matches!(self, Self::Sen65 | Self::Sen66 | Self::Sen68 | Self::Sen69c)
    }

    /// `true` if the variant measures CO2.
    pub const fn has_co2(self) -> bool {
        matches!(self, Self::Sen63c | Self::Sen66 | Self::Sen69c)
    }

    /// `true` if the variant measures formaldehyde.
    pub const fn has_hcho(self) -> bool {
        matches!(self, Self::Sen68 | Self::Sen69c)
    }

    const fn read_command(self) -> (u16, usize) {
        match self {
            Self::Sen60 => (cmd::SEN60_READ, 9),
            Self::Sen62 => (cmd::READ_SEN62, 6),
            Self::Sen63c => (cmd::READ_SEN63C, 7),
            Self::Sen65 => (cmd::READ_SEN65, 8),
            Self::Sen66 => (cmd::READ_SEN66, 9),
            Self::Sen68 => (cmd::READ_SEN68, 9),
            Self::Sen69c => (cmd::READ_SEN69C, 10),
        }
    }

    const fn command_wait(self) -> WaitUs {
        if self.is_sen60() {
            SEN60_COMMAND_WAIT_US
        } else {
            COMMAND_WAIT_US
        }
    }
}

/// Number concentrations in 0.1 particles/cm³ (`None` = unknown).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NumberConcentration {
    /// Particles 0.3–0.5 µm.
    pub pm0_5: Option<u32>,
    /// Particles 0.3–1.0 µm.
    pub pm1_0: Option<u32>,
    /// Particles 0.3–2.5 µm.
    pub pm2_5: Option<u32>,
    /// Particles 0.3–4.0 µm.
    pub pm4_0: Option<u32>,
    /// Particles 0.3–10 µm.
    pub pm10: Option<u32>,
}

impl NumberConcentration {
    fn from_words(w: &[u16]) -> Self {
        Self {
            pm0_5: unsigned(w[0]),
            pm1_0: unsigned(w[1]),
            pm2_5: unsigned(w[2]),
            pm4_0: unsigned(w[3]),
            pm10: unsigned(w[4]),
        }
    }
}

/// One `Read Measured Values` result; `None` = not available.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Measurement {
    /// PM1.0 mass concentration, 0.1 µg/m³.
    pub pm1_0: Option<u32>,
    /// PM2.5 mass concentration, 0.1 µg/m³.
    pub pm2_5: Option<u32>,
    /// PM4.0 mass concentration, 0.1 µg/m³.
    pub pm4_0: Option<u32>,
    /// PM10 mass concentration, 0.1 µg/m³.
    pub pm10: Option<u32>,
    /// Relative humidity, 0.01 %RH.
    pub humidity: Option<u16>,
    /// Temperature, 0.01 °C.
    pub temperature: Option<i32>,
    /// VOC index (1–500).
    pub voc_index: Option<u16>,
    /// NOx index (1–500).
    pub nox_index: Option<u16>,
    /// CO2, ppm.
    pub co2: Option<u32>,
    /// Formaldehyde, ppb (rounded from 0.1 ppb).
    pub formaldehyde: Option<u32>,
    /// Number concentrations (SEN60 only; other variants:
    /// `read_number_concentration`).
    pub number_concentration: Option<NumberConcentration>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        let values = [
            self.temperature.map(Quantity::Temperature),
            self.humidity.map(Quantity::Humidity),
            self.pm1_0.map(Quantity::Pm1),
            self.pm2_5.map(Quantity::Pm25),
            self.pm4_0.map(Quantity::Pm4),
            self.pm10.map(Quantity::Pm10),
            self.voc_index.map(Quantity::VocIndex),
            self.nox_index.map(Quantity::NoxIndex),
            self.co2.map(Quantity::Co2),
            self.formaldehyde.map(Quantity::Formaldehyde),
        ];
        values.into_iter().flatten().for_each(f);
    }
}

/// One temperature offset slot:
/// `T = T_measured + slope·T_measured + offset`, applied with the time
/// constant. The five slots add up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TemperatureOffset {
    /// Offset in 0.01 °C (-163.84–163.83 °C).
    pub offset: i32,
    /// Normalised slope in 1/10 000.
    pub slope: i16,
    /// Time constant in seconds (0 = within the next sample).
    pub time_constant_s: u16,
    /// Slot 0–4 (0 usually compensates the base self-heating).
    pub slot: u16,
}

/// RH/T engine temperature acceleration parameters, all scaled by 10.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TemperatureAcceleration {
    /// Filter constant K × 10.
    pub k: u16,
    /// Filter constant P × 10.
    pub p: u16,
    /// Time constant T1 in 0.1 s.
    pub t1: u16,
    /// Time constant T2 in 0.1 s.
    pub t2: u16,
}

fn unsigned(raw: u16) -> Option<u32> {
    (raw != 0xFFFF).then_some(u32::from(raw))
}

fn signed(raw: u16) -> Option<i16> {
    (raw != 0x7FFF).then_some(raw as i16)
}

fn index(raw: u16) -> Option<u16> {
    signed(raw).map(|v| ((i32::from(v) + 5) / 10).clamp(0, 500) as u16)
}

fn humidity(raw: u16) -> Option<u16> {
    signed(raw).map(|v| v.clamp(0, 10_000) as u16)
}

fn temperature(raw: u16) -> Option<i32> {
    signed(raw).map(|v| i32::from(v) / 2)
}

/// CO2 is uint16 (0xFFFF unknown) on the SEN66 and int16 (0x7FFF unknown)
/// on the STCC4-based SEN63C / SEN69C.
fn co2(raw: u16, variant: Variant) -> Option<u32> {
    match variant {
        Variant::Sen66 => unsigned(raw),
        _ => signed(raw).filter(|v| *v >= 0).map(|v| v as u32),
    }
}

fn convert(w: &[u16], variant: Variant) -> Measurement {
    let mut m = Measurement {
        pm1_0: unsigned(w[0]),
        pm2_5: unsigned(w[1]),
        pm4_0: unsigned(w[2]),
        pm10: unsigned(w[3]),
        ..Measurement::default()
    };
    if variant.is_sen60() {
        m.number_concentration = Some(NumberConcentration::from_words(&w[4..9]));
        return m;
    }
    m.humidity = humidity(w[4]);
    m.temperature = temperature(w[5]);
    let mut i = 6;
    if variant.has_gas() {
        m.voc_index = index(w[6]);
        m.nox_index = index(w[7]);
        i = 8;
    }
    if variant.has_hcho() {
        m.formaldehyde = unsigned(w[i]).map(|v| (v + 5) / 10);
        i += 1;
    }
    if variant.has_co2() {
        m.co2 = co2(w[i], variant);
    }
    m
}

fn u32_from_words(w: [u16; 2]) -> u32 {
    (u32::from(w[0]) << 16) | u32::from(w[1])
}

/// Decode a NUL-terminated product name ("SEN62" … "SEN69C").
fn variant_from_name(name: &[u8; 32]) -> Option<Variant> {
    let len = name.iter().position(|&b| b == 0).unwrap_or(name.len());
    match &name[..len] {
        b"SEN62" => Some(Variant::Sen62),
        b"SEN63C" => Some(Variant::Sen63c),
        b"SEN65" => Some(Variant::Sen65),
        b"SEN66" => Some(Variant::Sen66),
        b"SEN68" => Some(Variant::Sen68),
        b"SEN69C" => Some(Variant::Sen69c),
        _ => None,
    }
}

/// SEN60 6-byte serial number as 12 upper-case hex digits, NUL-padded.
fn hex_serial(w: &[u16; 3]) -> [u8; 32] {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let mut out = [0u8; 32];
    for (i, word) in w.iter().enumerate() {
        for j in 0..4 {
            out[i * 4 + j] = HEX[usize::from((word >> (12 - 4 * j)) & 0xF)];
        }
    }
    out
}

fn require<E>(supported: bool) -> Result<(), Error<E>> {
    if supported {
        Ok(())
    } else {
        Err(Error::Unsupported)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    Nothing,
    Read,
}

macro_rules! constructors {
    () => {
        /// Create a driver for `variant` at [`Variant::address`]; no bus
        /// traffic.
        pub const fn new(i2c: I2C, variant: Variant) -> Self {
            Self {
                bus: Interface::new(i2c, variant.address()),
                variant,
                running: false,
                pending: Pending::Nothing,
            }
        }

        /// Configured variant (updated by `probe`).
        pub const fn variant(&self) -> Variant {
            self.variant
        }

        /// `true` while the driver has started a measurement.
        pub const fn is_running(&self) -> bool {
            self.running
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.bus.release()
        }
    };
}

// Bus methods shared by the blocking and async drivers: `$async` is empty or
// `async`, `$aw` is empty or `.await`.
macro_rules! methods {
    ([$($async:tt)*] [$($aw:tt)*] $Delay:path) => {
        $($async)* fn send(&mut self, cmd: u16, args: &[u16], wait: WaitUs) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(cmd, args) $($aw)* ?;
            Ok(wait)
        }

        $($async)* fn get<const N: usize, D: $Delay>(&mut self, cmd: u16, delay: &mut D) -> Result<[u16; N], Error<I2C::Error>> {
            let mut w = [0u16; N];
            let wait = self.variant.command_wait();
            self.bus.fetch(cmd, &[], wait, delay, &mut w) $($aw)* ?;
            Ok(w)
        }

        /// Identify the part. SEN62 – SEN69C: read the product name and
        /// select the [`Variant`] (an unknown name gives
        /// [`Error::UnexpectedId`] with its first four bytes). SEN60 (no
        /// product name command, different address): read the device
        /// status as a presence check.
        pub $($async)* fn probe<D: $Delay>(&mut self, delay: &mut D) -> Result<Variant, Error<I2C::Error>> {
            if self.variant.is_sen60() {
                self.device_status(delay) $($aw)* ?;
                return Ok(Variant::Sen60);
            }
            let name = self.product_name(delay) $($aw)* ?;
            let variant = variant_from_name(&name).ok_or_else(|| {
                let [a, b, c, d, ..] = name;
                Error::UnexpectedId(u32::from_be_bytes([a, b, c, d]))
            })?;
            self.variant = variant;
            Ok(variant)
        }

        /// Product name, NUL-terminated ASCII (e.g. `"SEN66"`; not SEN60).
        pub $($async)* fn product_name<D: $Delay>(&mut self, delay: &mut D) -> Result<[u8; 32], Error<I2C::Error>> {
            require(!self.variant.is_sen60())?;
            let w: [u16; 16] = self.get(cmd::PRODUCT_NAME, delay) $($aw)* ?;
            let mut out = [0u8; 32];
            words_to_bytes(&w, &mut out);
            Ok(out)
        }

        /// Serial number, NUL-terminated ASCII (SEN60: its 6-byte serial
        /// as 12 hex digits).
        pub $($async)* fn serial_number<D: $Delay>(&mut self, delay: &mut D) -> Result<[u8; 32], Error<I2C::Error>> {
            if self.variant.is_sen60() {
                let w: [u16; 3] = self.get(cmd::SEN60_SERIAL_NUMBER, delay) $($aw)* ?;
                return Ok(hex_serial(&w));
            }
            let w: [u16; 16] = self.get(cmd::SERIAL_NUMBER, delay) $($aw)* ?;
            let mut out = [0u8; 32];
            words_to_bytes(&w, &mut out);
            Ok(out)
        }

        /// Firmware version `(major, minor)` (not SEN60).
        pub $($async)* fn firmware_version<D: $Delay>(&mut self, delay: &mut D) -> Result<(u8, u8), Error<I2C::Error>> {
            require(!self.variant.is_sen60())?;
            let [w]: [u16; 1] = self.get(cmd::VERSION, delay) $($aw)* ?;
            let [major, minor] = w.to_be_bytes();
            Ok((major, minor))
        }

        /// Start continuous measurement (first sample after ~1.1 s).
        pub $($async)* fn start_continuous_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let wait = if self.variant.is_sen60() {
                self.send(cmd::SEN60_START, &[], SEN60_COMMAND_WAIT_US) $($aw)* ?
            } else {
                self.send(cmd::START_MEASUREMENT, &[], START_WAIT_US) $($aw)* ?
            };
            self.running = true;
            self.pending = Pending::Nothing;
            Ok(wait)
        }

        /// Return to idle (1.4 s, SEN60 1 s, before the next start). The
        /// SEN63C / SEN69C must not be restarted within 24 s of a start.
        pub $($async)* fn stop_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let wait = if self.variant.is_sen60() {
                self.send(cmd::SEN60_STOP, &[], SEN60_STOP_WAIT_US) $($aw)* ?
            } else {
                self.send(cmd::STOP_MEASUREMENT, &[], STOP_WAIT_US) $($aw)* ?
            };
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(wait)
        }

        /// `true` if a new sample is available.
        pub $($async)* fn read_data_ready<D: $Delay>(&mut self, delay: &mut D) -> Result<bool, Error<I2C::Error>> {
            if self.variant.is_sen60() {
                let [w]: [u16; 1] = self.get(cmd::SEN60_DATA_READY, delay) $($aw)* ?;
                return Ok(w & 0x07FF != 0);
            }
            let [w]: [u16; 1] = self.get(cmd::DATA_READY, delay) $($aw)* ?;
            Ok(w & 0x00FF == 1)
        }

        /// Request the latest values (clears the data-ready flag); read
        /// them after the returned time with
        /// [`read_measured_values`](Self::read_measured_values).
        pub $($async)* fn request_measured_values(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let (cmd, _) = self.variant.read_command();
            let wait = self.variant.command_wait();
            self.send(cmd, &[], wait) $($aw)* ?;
            self.pending = Pending::Read;
            Ok(wait)
        }

        /// Values requested by `request_measured_values`; [`Error::NotReady`]
        /// if nothing was requested, the sensor is busy, or (SEN60) no new
        /// sample is buffered.
        pub $($async)* fn read_measured_values(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if self.pending != Pending::Read {
                return Err(Error::NotReady);
            }
            let (_, n) = self.variant.read_command();
            let mut w = [0u16; 10];
            let result = self.bus.read(&mut w[..n]) $($aw)*;
            self.pending = Pending::Nothing;
            result.map_err(nack_as_not_ready)?;
            Ok(convert(&w[..n], self.variant))
        }

        /// Number concentrations (SEN62 – SEN69C; the SEN60 reports them
        /// in [`Measurement::number_concentration`](crate::Measurement)).
        pub $($async)* fn read_number_concentration<D: $Delay>(&mut self, delay: &mut D) -> Result<NumberConcentration, Error<I2C::Error>> {
            require(!self.variant.is_sen60())?;
            let w: [u16; 5] = self.get(cmd::NUMBER_CONCENTRATION, delay) $($aw)* ?;
            Ok(NumberConcentration::from_words(&w))
        }

        /// Set one temperature offset slot (volatile). An offset outside
        /// ±163.83 °C or a slot above 4 gives [`Error::InvalidData`]
        /// without bus traffic.
        pub $($async)* fn set_temperature_offset(&mut self, p: TemperatureOffset) -> Result<WaitUs, Error<I2C::Error>> {
            require(!self.variant.is_sen60())?;
            let offset = i16::try_from(p.offset * 2).map_err(|_| Error::InvalidData)?;
            if p.slot > 4 {
                return Err(Error::InvalidData);
            }
            let args = [offset as u16, p.slope as u16, p.time_constant_s, p.slot];
            self.send(cmd::TEMPERATURE_OFFSET, &args, COMMAND_WAIT_US) $($aw)*
        }

        /// Set the RH/T engine temperature acceleration (idle only;
        /// volatile).
        pub $($async)* fn set_temperature_acceleration(&mut self, p: TemperatureAcceleration) -> Result<WaitUs, Error<I2C::Error>> {
            require(!self.variant.is_sen60())?;
            let args = [p.k, p.p, p.t1, p.t2];
            self.send(cmd::TEMPERATURE_ACCELERATION, &args, COMMAND_WAIT_US) $($aw)*
        }

        /// Heat the SHT sensor at 200 mW for 1 s to reverse creep at high
        /// humidity (idle only; wait 20 s before measuring again). Returns
        /// 1.3 s (older firmware); newer firmware can poll
        /// `sht_heater_measurements` after 20 ms.
        pub $($async)* fn activate_sht_heater(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            require(!self.variant.is_sen60())?;
            self.send(cmd::ACTIVATE_SHT_HEATER, &[], HEATER_WAIT_US) $($aw)*
        }

        /// SHT `(temperature 0.01 °C, humidity 0.01 %RH)` once heating has
        /// finished, `None` before (firmware SEN62 ≥ 6, SEN63C / SEN65 ≥ 5,
        /// SEN66 ≥ 4, SEN68 ≥ 7, SEN69C ≥ 9).
        pub $($async)* fn sht_heater_measurements<D: $Delay>(&mut self, delay: &mut D) -> Result<Option<(i32, u16)>, Error<I2C::Error>> {
            require(!self.variant.is_sen60())?;
            let [rh, t]: [u16; 2] = self.get(cmd::SHT_HEATER_MEASUREMENTS, delay) $($aw)* ?;
            Ok(temperature(t).zip(humidity(rh)))
        }

        /// Back up the 8-byte VOC algorithm state (VOC variants).
        pub $($async)* fn voc_state<D: $Delay>(&mut self, delay: &mut D) -> Result<[u8; 8], Error<I2C::Error>> {
            require(self.variant.has_gas())?;
            let w: [u16; 4] = self.get(cmd::VOC_STATE, delay) $($aw)* ?;
            let mut out = [0u8; 8];
            words_to_bytes(&w, &mut out);
            Ok(out)
        }

        /// Restore the VOC algorithm state (idle only, before starting).
        pub $($async)* fn set_voc_state(&mut self, state: [u8; 8]) -> Result<WaitUs, Error<I2C::Error>> {
            require(self.variant.has_gas())?;
            let w = |i: usize| u16::from_be_bytes([state[i], state[i + 1]]);
            self.send(cmd::VOC_STATE, &[w(0), w(2), w(4), w(6)], COMMAND_WAIT_US) $($aw)*
        }

        /// Start a forced CO2 recalibration to `reference_ppm` (idle, after
        /// ≥ 3 min of measurement and a stop). Fetch the result with
        /// `read_forced_recalibration` after the returned 500 ms.
        pub $($async)* fn start_forced_recalibration(&mut self, reference_ppm: u16) -> Result<WaitUs, Error<I2C::Error>> {
            require(self.variant.has_co2())?;
            self.send(cmd::FORCED_RECALIBRATION, &[reference_ppm], FRC_WAIT_US) $($aw)*
        }

        /// Applied FRC correction in ppm; [`Error::InvalidData`] if the
        /// sensor reports a failed FRC (0xFFFF).
        pub $($async)* fn read_forced_recalibration(&mut self) -> Result<i16, Error<I2C::Error>> {
            require(self.variant.has_co2())?;
            let mut word = [0u16];
            self.bus.read(&mut word) $($aw)* ?;
            if word[0] == 0xFFFF {
                return Err(Error::InvalidData);
            }
            Ok((i32::from(word[0]) - 0x8000) as i16)
        }

        /// Reset the CO2 sensor's stored settings and FRC / ASC history
        /// (idle only; SEN66 firmware ≥ 1.2).
        pub $($async)* fn co2_factory_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            require(self.variant.has_co2())?;
            self.send(cmd::CO2_FACTORY_RESET, &[], CO2_FACTORY_RESET_WAIT_US) $($aw)*
        }

        /// `true` if CO2 automatic self-calibration is enabled (idle only).
        pub $($async)* fn automatic_self_calibration<D: $Delay>(&mut self, delay: &mut D) -> Result<bool, Error<I2C::Error>> {
            require(self.variant.has_co2())?;
            let [w]: [u16; 1] = self.get(cmd::ASC, delay) $($aw)* ?;
            Ok(w & 0x00FF == 1)
        }

        /// Enable or disable CO2 automatic self-calibration (default on;
        /// idle only; volatile).
        pub $($async)* fn set_automatic_self_calibration(&mut self, enabled: bool) -> Result<WaitUs, Error<I2C::Error>> {
            require(self.variant.has_co2())?;
            self.send(cmd::ASC, &[u16::from(enabled)], COMMAND_WAIT_US) $($aw)*
        }

        /// Ambient pressure used for CO2 compensation, hPa.
        pub $($async)* fn ambient_pressure<D: $Delay>(&mut self, delay: &mut D) -> Result<u16, Error<I2C::Error>> {
            require(self.variant.has_co2())?;
            let [w]: [u16; 1] = self.get(cmd::AMBIENT_PRESSURE, delay) $($aw)* ?;
            Ok(w)
        }

        /// Set the ambient pressure for CO2 compensation (700–1200 hPa,
        /// default 1013; overrides the altitude; volatile). Out of range
        /// gives [`Error::InvalidData`] without bus traffic.
        pub $($async)* fn set_ambient_pressure(&mut self, hpa: u16) -> Result<WaitUs, Error<I2C::Error>> {
            require(self.variant.has_co2())?;
            if !(700..=1200).contains(&hpa) {
                return Err(Error::InvalidData);
            }
            self.send(cmd::AMBIENT_PRESSURE, &[hpa], COMMAND_WAIT_US) $($aw)*
        }

        /// Sensor altitude used for CO2 compensation, m (idle only).
        pub $($async)* fn sensor_altitude<D: $Delay>(&mut self, delay: &mut D) -> Result<u16, Error<I2C::Error>> {
            require(self.variant.has_co2())?;
            let [w]: [u16; 1] = self.get(cmd::ALTITUDE, delay) $($aw)* ?;
            Ok(w)
        }

        /// Set the sensor altitude (0–3000 m, default 0; idle only;
        /// volatile). Out of range gives [`Error::InvalidData`].
        pub $($async)* fn set_sensor_altitude(&mut self, meters: u16) -> Result<WaitUs, Error<I2C::Error>> {
            require(self.variant.has_co2())?;
            if meters > 3000 {
                return Err(Error::InvalidData);
            }
            self.send(cmd::ALTITUDE, &[meters], COMMAND_WAIT_US) $($aw)*
        }

        /// Run the fan at full speed for 10 s (idle only; wait 10 s before
        /// starting a measurement).
        pub $($async)* fn start_fan_cleaning(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.variant.is_sen60() {
                return self.send(cmd::SEN60_FAN_CLEANING, &[], SEN60_COMMAND_WAIT_US) $($aw)*;
            }
            self.send(cmd::FAN_CLEANING, &[], COMMAND_WAIT_US) $($aw)*
        }

        /// Device status register (see the `STATUS_*` masks; the SEN60
        /// has a 16-bit register). Error flags are sticky.
        pub $($async)* fn device_status<D: $Delay>(&mut self, delay: &mut D) -> Result<u32, Error<I2C::Error>> {
            if self.variant.is_sen60() {
                let [w]: [u16; 1] = self.get(cmd::SEN60_DEVICE_STATUS, delay) $($aw)* ?;
                return Ok(u32::from(w));
            }
            Ok(u32_from_words(self.get(cmd::DEVICE_STATUS, delay) $($aw)* ?))
        }

        /// Read the device status, then clear it (not SEN60, which clears
        /// only on reset).
        pub $($async)* fn read_and_clear_device_status<D: $Delay>(&mut self, delay: &mut D) -> Result<u32, Error<I2C::Error>> {
            require(!self.variant.is_sen60())?;
            Ok(u32_from_words(self.get(cmd::READ_AND_CLEAR_STATUS, delay) $($aw)* ?))
        }

        /// Device reset, same as a power cycle (1.2 s, SEN60 1 ms; all
        /// volatile settings revert).
        pub $($async)* fn reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let wait = if self.variant.is_sen60() {
                self.send(cmd::SEN60_RESET, &[], SEN60_COMMAND_WAIT_US) $($aw)* ?
            } else {
                self.send(cmd::RESET, &[], RESET_WAIT_US) $($aw)* ?
            };
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(wait)
        }

        $($async)* fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.running {
                return self.request_measured_values() $($aw)*;
            }
            self.start_continuous_measurement() $($aw)* ?;
            Ok(FIRST_SAMPLE_US)
        }
    };
}

macro_rules! trait_impls {
    ($Sensor:path, $I2c:path, [$($async:tt)*] [$($aw:tt)*]) => {
        impl<I2C: $I2c> $Sensor for Sen6x<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Starts continuous measurement on the first call (returns
            /// 1.1 s), then requests the latest values (20 ms, SEN60 1 ms).
            $($async)* fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.start() $($aw)*
            }

            /// [`Error::NotReady`] right after starting.
            $($async)* fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                self.read_measured_values() $($aw)*
            }
        }
    };
}

use zigbee_sensirion_common::blocking::Interface;
use zigbee_sensirion_common::{nack_as_not_ready, words_to_bytes};

/// Blocking driver.
#[derive(Debug)]
pub struct Sen6x<I2C> {
    bus: Interface<I2C>,
    variant: Variant,
    running: bool,
    pending: Pending,
}

impl<I2C> Sen6x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Sen6x<I2C> {
    methods!([] [] embedded_hal::delay::DelayNs);
}

trait_impls!(
    zigbee_sensor::blocking::Sensor,
    embedded_hal::i2c::I2c,
    [] []
);

/// Async driver.
pub mod asynch {
    use super::{
        CO2_FACTORY_RESET_WAIT_US, COMMAND_WAIT_US, FIRST_SAMPLE_US, FRC_WAIT_US, HEATER_WAIT_US,
        Measurement, NumberConcentration, Pending, RESET_WAIT_US, SEN60_COMMAND_WAIT_US,
        SEN60_STOP_WAIT_US, START_WAIT_US, STOP_WAIT_US, TemperatureAcceleration,
        TemperatureOffset, Variant, cmd, convert, hex_serial, humidity, require, temperature,
        u32_from_words, variant_from_name,
    };
    use zigbee_sensirion_common::asynch::Interface;
    use zigbee_sensirion_common::{nack_as_not_ready, words_to_bytes};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Sen6x`].
    #[derive(Debug)]
    pub struct Sen6x<I2C> {
        bus: Interface<I2C>,
        variant: Variant,
        running: bool,
        pending: Pending,
    }

    impl<I2C> Sen6x<I2C> {
        constructors!();
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Sen6x<I2C> {
        methods!([async] [.await] embedded_hal_async::delay::DelayNs);
    }

    trait_impls!(
        zigbee_sensor::asynch::Sensor,
        embedded_hal_async::i2c::I2c,
        [async] [.await]
    );
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
    use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as D};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensirion_common::encode_word;
    use zigbee_sensor::blocking::Sensor;

    // SEN66: PM 1.2/2.5/3.1/4.0 µg/m³, 45.20 %RH, 22.55 °C (4510/200),
    // VOC 100.0, NOx 1.0, 612 ppm CO2.
    const SEN66: [u16; 9] = [12, 25, 31, 40, 4520, 4510, 1000, 10, 612];

    fn words(w: &[u16]) -> Vec<u8> {
        w.iter().flat_map(|&x| encode_word(x)).collect()
    }

    fn name(s: &[u8]) -> Vec<u8> {
        let mut bytes = [0u8; 32];
        bytes[..s.len()].copy_from_slice(s);
        let w: Vec<u16> = bytes
            .chunks(2)
            .map(|c| u16::from_be_bytes([c[0], c[1]]))
            .collect();
        words(&w)
    }

    fn nack(t: Transaction) -> Transaction {
        t.with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
    }

    #[test]
    fn probe_and_sen66_cycle() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0xD0, 0x14]),
            Transaction::read(ADDRESS, name(b"SEN66")),
            Transaction::write(ADDRESS, vec![0x00, 0x21]),
            Transaction::write(ADDRESS, vec![0x03, 0x00]),
            nack(Transaction::read(ADDRESS, vec![0; 27])),
            Transaction::write(ADDRESS, vec![0x03, 0x00]),
            Transaction::read(ADDRESS, words(&SEN66)),
            Transaction::write(ADDRESS, vec![0x02, 0x02]),
            Transaction::read(ADDRESS, words(&[0x0000])),
            Transaction::write(ADDRESS, vec![0x01, 0x04]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay =
            CheckedDelay::new(&[D::blocking_delay_us(20_000), D::blocking_delay_us(20_000)]);
        let mut sen = Sen6x::new(&mut i2c, Variant::Sen62);
        assert_eq!(sen.probe(&mut delay).unwrap(), Variant::Sen66);
        assert_eq!(sen.start_measurement().unwrap(), 1_100_000);
        assert_eq!(sen.read_measurement(), Err(Error::NotReady));
        assert_eq!(sen.start_measurement().unwrap(), 20_000);
        assert_eq!(sen.read_measurement(), Err(Error::NotReady));
        sen.start_measurement().unwrap();
        let m = sen.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                pm1_0: Some(12),
                pm2_5: Some(25),
                pm4_0: Some(31),
                pm10: Some(40),
                humidity: Some(4520),
                temperature: Some(2255),
                voc_index: Some(100),
                nox_index: Some(1),
                co2: Some(612),
                ..Measurement::default()
            }
        );
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(seen.len(), 9);
        assert_eq!(seen[0], Quantity::Temperature(2255));
        assert_eq!(seen[8], Quantity::Co2(612));
        assert!(!sen.read_data_ready(&mut delay).unwrap());
        assert_eq!(sen.stop_measurement().unwrap(), 1_400_000);
        delay.done();
        i2c.done();
    }

    #[test]
    fn sen60_commands() {
        let a = ADDRESS_SEN60;
        // PM 0.5/1.0/1.5/2.0 µg/m³; 10.0/20.0/30.0/40.0 #/cm³, PM10 unknown.
        let sample = [5, 10, 15, 20, 100, 200, 300, 400, 0xFFFF];
        let expectations = [
            Transaction::write(a, vec![0xE0, 0x0B]),
            Transaction::read(a, words(&[0x0000])),
            Transaction::write(a, vec![0x21, 0x52]),
            Transaction::write(a, vec![0xE4, 0xB8]),
            Transaction::read(a, words(&[0x0001])),
            Transaction::write(a, vec![0xEC, 0x05]),
            Transaction::read(a, words(&sample)),
            Transaction::write(a, vec![0x36, 0x82]),
            Transaction::read(a, words(&[0x0012, 0x34AB, 0xCDEF])),
            Transaction::write(a, vec![0xE0, 0x0B]),
            Transaction::read(a, words(&[0x0012])),
            Transaction::write(a, vec![0x37, 0x30]),
            Transaction::write(a, vec![0x3F, 0x86]),
            Transaction::write(a, vec![0x3F, 0x8D]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[
            D::blocking_delay_us(1_000),
            D::blocking_delay_us(1_000),
            D::blocking_delay_us(1_000),
            D::blocking_delay_us(1_000),
        ]);
        let mut sen = Sen6x::new(&mut i2c, Variant::Sen60);
        assert_eq!(sen.probe(&mut delay).unwrap(), Variant::Sen60);
        assert_eq!(sen.start_measurement().unwrap(), 1_100_000);
        assert!(sen.read_data_ready(&mut delay).unwrap());
        assert_eq!(sen.start_measurement().unwrap(), 1_000);
        let m = sen.read_measurement().unwrap();
        assert_eq!(m.pm2_5, Some(10));
        assert_eq!(m.humidity, None);
        assert_eq!(
            m.number_concentration,
            Some(NumberConcentration {
                pm0_5: Some(100),
                pm1_0: Some(200),
                pm2_5: Some(300),
                pm4_0: Some(400),
                pm10: None,
            })
        );
        let serial = sen.serial_number(&mut delay).unwrap();
        assert_eq!(&serial[..13], b"001234ABCDEF\0");
        let status = sen.device_status(&mut delay).unwrap();
        assert_eq!(status, STATUS_FAN | STATUS_SEN60_SPEED);
        // SEN6x-only commands: no bus traffic.
        assert_eq!(sen.product_name(&mut delay), Err(Error::Unsupported));
        assert_eq!(
            sen.read_number_concentration(&mut delay),
            Err(Error::Unsupported)
        );
        assert_eq!(
            sen.set_temperature_offset(TemperatureOffset::default()),
            Err(Error::Unsupported)
        );
        assert_eq!(sen.activate_sht_heater(), Err(Error::Unsupported));
        assert_eq!(sen.start_fan_cleaning().unwrap(), 1_000);
        assert_eq!(sen.stop_measurement().unwrap(), 1_000_000);
        assert_eq!(sen.reset().unwrap(), 1_000);
        delay.done();
        i2c.done();
    }

    #[test]
    fn variant_layouts_and_unknown_values() {
        let rht = [12, 25, 31, 40, 3000, (-1001i16) as u16];
        let m = convert(&rht, Variant::Sen62);
        assert_eq!(m.temperature, Some(-500));
        assert_eq!((m.voc_index, m.co2, m.formaldehyde), (None, None, None));

        // SEN63C: CO2 is int16, 0x7FFF while conditioning.
        let mut sen63c = [12, 25, 31, 40, 3000, 4000, 455];
        assert_eq!(convert(&sen63c, Variant::Sen63c).co2, Some(455));
        sen63c[6] = 0x7FFF;
        assert_eq!(convert(&sen63c, Variant::Sen63c).co2, None);

        // SEN68: HCHO 12.6 ppb → 13 ppb; NOx 0x7FFF during start-up.
        let sen68 = [0xFFFF, 25, 31, 40, 3000, 4000, 1000, 0x7FFF, 126];
        let m = convert(&sen68, Variant::Sen68);
        assert_eq!(m.pm1_0, None);
        assert_eq!((m.nox_index, m.formaldehyde, m.co2), (None, Some(13), None));

        // SEN69C: HCHO before CO2.
        let sen69c = [12, 25, 31, 40, 3000, 4000, 1000, 10, 0xFFFF, 812];
        let m = convert(&sen69c, Variant::Sen69c);
        assert_eq!((m.formaldehyde, m.co2), (None, Some(812)));

        // Unknown product name.
        let expectations = [
            Transaction::write(ADDRESS, vec![0xD0, 0x14]),
            Transaction::read(ADDRESS, name(b"SEN55")),
            Transaction::write(ADDRESS, vec![0xD0, 0x14]),
            Transaction::read(ADDRESS, name(b"SEN69C")),
            Transaction::write(ADDRESS, vec![0x04, 0xB5]),
            Transaction::read(ADDRESS, words(&sen69c)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay =
            CheckedDelay::new(&[D::blocking_delay_us(20_000), D::blocking_delay_us(20_000)]);
        let mut sen = Sen6x::new(&mut i2c, Variant::Sen65);
        assert_eq!(
            sen.probe(&mut delay),
            Err(Error::UnexpectedId(u32::from_be_bytes(*b"SEN5")))
        );
        assert_eq!(sen.variant(), Variant::Sen65);
        assert_eq!(sen.probe(&mut delay).unwrap(), Variant::Sen69c);
        sen.request_measured_values().unwrap();
        assert_eq!(sen.read_measured_values().unwrap().co2, Some(812));
        delay.done();
        i2c.done();
    }

    #[test]
    fn configuration_commands() {
        let mut offset_write = vec![0x60, 0xB2];
        // -1.50 °C → -300 (scale 200); slope 0.01 → 100; 600 s; slot 1.
        offset_write.extend(words(&[(-300i16) as u16, 100, 600, 1]));
        let mut accel_write = vec![0x61, 0x00];
        accel_write.extend(words(&[50, 20, 1000, 300]));
        let mut pressure_write = vec![0x67, 0x20];
        pressure_write.extend(words(&[1013]));
        let mut frc_write = vec![0x67, 0x07];
        frc_write.extend(words(&[420]));
        let expectations = [
            Transaction::write(ADDRESS, offset_write),
            Transaction::write(ADDRESS, accel_write),
            Transaction::write(ADDRESS, pressure_write),
            Transaction::write(ADDRESS, frc_write),
            Transaction::read(ADDRESS, words(&[0x8000 - 25])),
            Transaction::write(ADDRESS, vec![0x67, 0x07, 0x01, 0xA4, 0x4D]),
            Transaction::read(ADDRESS, words(&[0xFFFF])),
            Transaction::write(ADDRESS, vec![0x67, 0x11, 0x00, 0x00, 0x81]),
            Transaction::write(ADDRESS, vec![0x03, 0x16]),
            Transaction::read(ADDRESS, words(&[100, 200, 300, 400, 500])),
            Transaction::write(ADDRESS, vec![0xD2, 0x10]),
            Transaction::read(ADDRESS, words(&[0x0020, 0x0200])),
            Transaction::write(ADDRESS, vec![0xD1, 0x00]),
            Transaction::read(ADDRESS, words(&[0x0402])),
            Transaction::write(ADDRESS, vec![0x67, 0x65]),
            Transaction::write(ADDRESS, vec![0x67, 0x90]),
            Transaction::read(ADDRESS, words(&[4520, 4510])),
            Transaction::write(ADDRESS, vec![0xD3, 0x04]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[
            D::blocking_delay_us(20_000),
            D::blocking_delay_us(20_000),
            D::blocking_delay_us(20_000),
            D::blocking_delay_us(20_000),
        ]);
        let mut sen = Sen6x::new(&mut i2c, Variant::Sen66);
        let offset = TemperatureOffset {
            offset: -150,
            slope: 100,
            time_constant_s: 600,
            slot: 1,
        };
        assert_eq!(
            sen.set_temperature_offset(TemperatureOffset { slot: 5, ..offset }),
            Err(Error::InvalidData)
        );
        assert_eq!(
            sen.set_temperature_offset(TemperatureOffset {
                offset: 20_000,
                ..offset
            }),
            Err(Error::InvalidData)
        );
        sen.set_temperature_offset(offset).unwrap();
        sen.set_temperature_acceleration(TemperatureAcceleration {
            k: 50,
            p: 20,
            t1: 1000,
            t2: 300,
        })
        .unwrap();
        assert_eq!(sen.set_ambient_pressure(600), Err(Error::InvalidData));
        assert_eq!(sen.set_sensor_altitude(3001), Err(Error::InvalidData));
        sen.set_ambient_pressure(1013).unwrap();
        assert_eq!(sen.start_forced_recalibration(420).unwrap(), 500_000);
        assert_eq!(sen.read_forced_recalibration().unwrap(), -25);
        sen.start_forced_recalibration(420).unwrap();
        assert_eq!(sen.read_forced_recalibration(), Err(Error::InvalidData));
        sen.set_automatic_self_calibration(false).unwrap();
        let n = sen.read_number_concentration(&mut delay).unwrap();
        assert_eq!((n.pm0_5, n.pm10), (Some(100), Some(500)));
        let status = sen.read_and_clear_device_status(&mut delay).unwrap();
        assert_eq!(status, STATUS_SPEED | STATUS_CO2_2);
        assert_eq!(sen.firmware_version(&mut delay).unwrap(), (4, 2));
        assert_eq!(sen.activate_sht_heater().unwrap(), 1_300_000);
        assert_eq!(
            sen.sht_heater_measurements(&mut delay).unwrap(),
            Some((2255, 4520))
        );
        assert_eq!(sen.reset().unwrap(), 1_200_000);
        delay.done();
        i2c.done();

        // CO2 / VOC commands on variants without them: no bus traffic.
        let mut i2c = Mock::new(&[]);
        let mut sen = Sen6x::new(&mut i2c, Variant::Sen65);
        assert_eq!(sen.set_ambient_pressure(1013), Err(Error::Unsupported));
        assert_eq!(sen.co2_factory_reset(), Err(Error::Unsupported));
        let mut sen = Sen6x::new(sen.release(), Variant::Sen62);
        assert_eq!(sen.set_voc_state([0; 8]), Err(Error::Unsupported));
        i2c.done();
    }

    #[test]
    fn async_measure_and_crc() {
        use zigbee_sensor::asynch::Sensor as _;

        let sample = [12, 25, 31, 40, 4520, 4510, 1000, 10];
        let mut bad = words(&sample);
        bad[5] ^= 1;
        let mut state_write = vec![0x61, 0x81];
        state_write.extend(words(&[0x0102, 0x0304, 0x0506, 0x0708]));
        let expectations = [
            Transaction::write(ADDRESS, vec![0x61, 0x81]),
            Transaction::read(ADDRESS, words(&[0x0102, 0x0304, 0x0506, 0x0708])),
            Transaction::write(ADDRESS, state_write),
            Transaction::write(ADDRESS, vec![0x00, 0x21]),
            Transaction::write(ADDRESS, vec![0x04, 0x46]),
            Transaction::read(ADDRESS, words(&sample)),
            Transaction::write(ADDRESS, vec![0x04, 0x46]),
            Transaction::read(ADDRESS, bad),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[
            D::async_delay_us(20_000),
            D::async_delay_us(1_100_000),
            D::async_delay_us(20_000),
        ]);
        let mut sen = asynch::Sen6x::new(&mut i2c, Variant::Sen65);
        block_on(async {
            let state = sen.voc_state(&mut delay).await.unwrap();
            assert_eq!(state, [1, 2, 3, 4, 5, 6, 7, 8]);
            sen.set_voc_state(state).await.unwrap();
            assert_eq!(sen.measure(&mut delay).await, Err(Error::NotReady));
            let m = sen.measure(&mut delay).await.unwrap();
            assert_eq!((m.voc_index, m.nox_index), (Some(100), Some(1)));
            sen.start_measurement().await.unwrap();
            assert_eq!(sen.read_measurement().await, Err(Error::Crc));
        });
        delay.done();
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
