//! MCP9600 / MCP9601 (and the L0x / RL0x grades) thermocouple EMF to
//! temperature converter, I2C.
//!
//! The part digitises the thermocouple EMF with an 18-bit delta-sigma ADC,
//! measures the cold junction on-die and computes the hot-junction
//! temperature with the NIST ITS-90 tables for type K, J, T, N, S, E, B or
//! R. The driver uses it either in **burst mode** (default: one sample, then
//! the part returns to its 2 µA shutdown state; good for sleepy devices) or
//! in continuous mode.
//!
//! Reference: Microchip MCP960X/L0X/RL0X datasheet DS20005426G (Registers
//! 5-1 … 5-13, Tables 5-2 / 5-3, tCALC) and errata DS80000741.
//!
//! # Timing
//!
//! A sample needs one ADC conversion (320 / 80 / 20 / 5 ms typ for 18 / 16 /
//! 14 / 12 bit), in parallel with the cold-junction conversion (250 ms at
//! 0.0625 °C, 63 ms at 0.25 °C), plus tCALC (12 ms typ). The datasheet only
//! gives typical figures, so `start_measurement` returns
//! `samples × (max(adc, cold junction) + 12 ms) × 1.25`. If the result is
//! not ready yet, `read_measurement` returns [`Error::NotReady`]; retry
//! later.
//!
//! # I2C
//!
//! Standard mode only (≤ 100 kHz); the part stretches the clock up to 60 µs
//! and each access must end with a STOP, so the driver uses separate write
//! and read transfers (no repeated start). Errata DS80000741: devices with
//! date code before 1845 / Device ID-Revision ≤ 0x4012 may skip the clock
//! stretch in multi-byte reads above ≈ 85 kHz and repeat the first byte;
//! run those at ≤ 85 kHz ([`DeviceId::has_i2c_errata`]).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Register pointers.
pub mod reg {
    /// Hot-junction (thermocouple) temperature TH.
    pub const HOT_JUNCTION: u8 = 0x00;
    /// Junction temperature delta TΔ.
    pub const DELTA: u8 = 0x01;
    /// Cold-junction temperature TC.
    pub const COLD_JUNCTION: u8 = 0x02;
    /// Raw ADC data (24-bit).
    pub const RAW_ADC: u8 = 0x03;
    /// Status.
    pub const STATUS: u8 = 0x04;
    /// Thermocouple sensor configuration.
    pub const SENSOR_CONFIG: u8 = 0x05;
    /// Device configuration.
    pub const DEVICE_CONFIG: u8 = 0x06;
    /// Device ID / revision.
    pub const DEVICE_ID: u8 = 0x20;
}

/// Status register bits.
pub mod status {
    /// Burst-mode conversions complete.
    pub const BURST_COMPLETE: u8 = 0x80;
    /// TH updated.
    pub const TH_UPDATE: u8 = 0x40;
    /// MCP9601: thermocouple shorted to VDD or VSS.
    pub const SHORT_CIRCUIT: u8 = 0x20;
    /// EMF out of range for the type (MCP9601: thermocouple open).
    pub const INPUT_RANGE: u8 = 0x10;
}

/// Lowest of the eight addresses (ADDR pin divider selects 0x60–0x67).
pub const BASE_ADDRESS: u8 = 0x60;

/// Temperature calculation time tCALC (typical).
pub const CALC_US: WaitUs = 12_000;

/// Thermocouple type (sensor configuration bits 6:4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThermocoupleType {
    /// Type K.
    K = 0,
    /// Type J.
    J = 1,
    /// Type T.
    T = 2,
    /// Type N.
    N = 3,
    /// Type S.
    S = 4,
    /// Type E.
    E = 5,
    /// Type B.
    B = 6,
    /// Type R.
    R = 7,
}

/// ADC resolution (device configuration bits 6:5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcResolution {
    /// 18 bit, 2 µV, 320 ms.
    Bits18 = 0,
    /// 16 bit, 8 µV, 80 ms.
    Bits16 = 1,
    /// 14 bit, 32 µV, 20 ms.
    Bits14 = 2,
    /// 12 bit, 128 µV, 5 ms.
    Bits12 = 3,
}

impl AdcResolution {
    /// Typical conversion time.
    pub const fn conversion_us(self) -> WaitUs {
        match self {
            Self::Bits18 => 320_000,
            Self::Bits16 => 80_000,
            Self::Bits14 => 20_000,
            Self::Bits12 => 5_000,
        }
    }
}

/// Cold-junction resolution (device configuration bit 7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColdJunctionResolution {
    /// 0.0625 °C, 250 ms.
    Fine = 0,
    /// 0.25 °C, 63 ms.
    Coarse = 1,
}

impl ColdJunctionResolution {
    /// Typical conversion time.
    pub const fn conversion_us(self) -> WaitUs {
        match self {
            Self::Fine => 250_000,
            Self::Coarse => 63_000,
        }
    }
}

/// Operating mode used by `start_measurement`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Burst of `2^n` samples (n = 0..=7), then automatic shutdown.
    Burst(u8),
    /// Continuous conversions (no shutdown between readings).
    Continuous,
}

/// Part variant from the device ID register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// MCP9600 / MCP96L00 / MCP96RL00 (ID 0x40).
    Mcp9600,
    /// MCP9601 / MCP96L01 / MCP96RL01 (ID 0x41), with open / short detection.
    Mcp9601,
}

/// Device ID and revision.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceId {
    /// Variant.
    pub variant: Variant,
    /// Revision byte (major.minor nibbles).
    pub revision: u8,
}

impl DeviceId {
    /// Decode the 2-byte device ID register.
    pub fn decode<E>(bytes: [u8; 2]) -> Result<Self, Error<E>> {
        let variant = match bytes[0] {
            0x40 => Variant::Mcp9600,
            0x41 => Variant::Mcp9601,
            id => return Err(Error::UnexpectedId(u32::from(id))),
        };
        Ok(Self {
            variant,
            revision: bytes[1],
        })
    }

    /// Affected by the multi-byte read erratum (MCP9600, ID/revision ≤
    /// 0x4012): keep SCL ≤ 85 kHz.
    pub const fn has_i2c_errata(&self) -> bool {
        matches!(self.variant, Variant::Mcp9600) && self.revision <= 0x12
    }
}

/// One reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Hot-junction (thermocouple) temperature in 0.01 °C.
    pub temperature: i32,
    /// Cold-junction (die) temperature in 0.01 °C.
    pub cold_junction: i32,
}

impl Readings for Measurement {
    /// Emits the hot-junction temperature only.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
    }
}

/// Convert a temperature register (0.0625 °C two's complement) to 0.01 °C,
/// rounded half away from zero.
pub const fn centi(bytes: [u8; 2]) -> i32 {
    let x = i16::from_be_bytes(bytes) as i32 * 25;
    if x >= 0 { (x + 2) / 4 } else { (x - 2) / 4 }
}

/// Check the status register for the given mode: fault bits are
/// [`Error::InvalidData`], a missing update flag is [`Error::NotReady`].
pub fn check_status<E>(value: u8, mode: Mode) -> Result<(), Error<E>> {
    if value & (status::SHORT_CIRCUIT | status::INPUT_RANGE) != 0 {
        return Err(Error::InvalidData);
    }
    let ready = match mode {
        Mode::Burst(_) => status::BURST_COMPLETE,
        Mode::Continuous => status::TH_UPDATE,
    };
    if value & ready == 0 {
        return Err(Error::NotReady);
    }
    Ok(())
}

/// Sign-extend the 24-bit raw ADC register (2 µV per LSB at 18 bit).
pub const fn raw_adc(bytes: [u8; 3]) -> i32 {
    i32::from_be_bytes([bytes[0], bytes[1], bytes[2], 0]) >> 8
}

macro_rules! constructors {
    () => {
        /// Type K, filter off, 18-bit ADC, 0.0625 °C cold junction, burst
        /// of one sample. No bus traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                thermocouple: ThermocoupleType::K,
                filter: 0,
                adc: AdcResolution::Bits18,
                cold_junction: ColdJunctionResolution::Fine,
                mode: Mode::Burst(0),
                configured: false,
            }
        }

        /// Thermocouple type.
        pub const fn with_thermocouple(mut self, thermocouple: ThermocoupleType) -> Self {
            self.thermocouple = thermocouple;
            self.configured = false;
            self
        }

        /// Digital filter coefficient 0 (off) ..= 7 (maximum).
        pub const fn with_filter(mut self, coefficient: u8) -> Self {
            self.filter = if coefficient > 7 { 7 } else { coefficient };
            self.configured = false;
            self
        }

        /// ADC and cold-junction resolution.
        pub const fn with_resolution(
            mut self,
            adc: AdcResolution,
            cold_junction: ColdJunctionResolution,
        ) -> Self {
            self.adc = adc;
            self.cold_junction = cold_junction;
            self
        }

        /// Burst (`Mode::Burst(n)`, 2^n samples, n ≤ 7) or continuous mode.
        pub const fn with_mode(mut self, mode: Mode) -> Self {
            self.mode = match mode {
                Mode::Burst(n) if n > 7 => Mode::Burst(7),
                m => m,
            };
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        /// Sensor configuration register value.
        pub const fn sensor_config(&self) -> u8 {
            ((self.thermocouple as u8) << 4) | self.filter
        }

        /// Device configuration register value for a shutdown-mode field.
        const fn device_config(&self, mode_bits: u8) -> u8 {
            let samples = match self.mode {
                Mode::Burst(n) => n,
                Mode::Continuous => 0,
            };
            ((self.cold_junction as u8) << 7) | ((self.adc as u8) << 5) | (samples << 2) | mode_bits
        }

        const fn mode_bits(&self) -> u8 {
            match self.mode {
                Mode::Burst(_) => 0b10,
                Mode::Continuous => 0b00,
            }
        }

        /// Wait returned by `start_measurement` (see the crate docs).
        pub const fn conversion_us(&self) -> WaitUs {
            let adc = self.adc.conversion_us();
            let cj = self.cold_junction.conversion_us();
            let sample = if adc > cj { adc } else { cj } + CALC_US;
            let samples = match self.mode {
                Mode::Burst(n) => 1u32 << n,
                Mode::Continuous => 1,
            };
            samples * sample / 4 * 5
        }
    };
}

/// Blocking driver.
pub struct Mcp9600<I2C> {
    i2c: I2C,
    address: u8,
    thermocouple: ThermocoupleType,
    filter: u8,
    adc: AdcResolution,
    cold_junction: ColdJunctionResolution,
    mode: Mode,
    configured: bool,
}

impl<I2C> Mcp9600<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Mcp9600<I2C> {
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[reg, value])
            .map_err(Error::bus)
    }

    fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, &[reg]).map_err(Error::bus)?;
        self.i2c.read(self.address, buf).map_err(Error::bus)
    }

    /// Read the device ID register.
    pub fn probe(&mut self) -> Result<DeviceId, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.read_reg(reg::DEVICE_ID, &mut buf)?;
        DeviceId::decode(buf)
    }

    /// Write the sensor configuration and put the device in shutdown.
    pub fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_reg(reg::SENSOR_CONFIG, self.sensor_config())?;
        self.write_reg(reg::DEVICE_CONFIG, self.device_config(0b01))?;
        self.configured = true;
        Ok(())
    }

    /// Read the status register.
    pub fn read_status(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8];
        self.read_reg(reg::STATUS, &mut buf)?;
        Ok(buf[0])
    }

    /// Read the raw ADC register, sign-extended.
    pub fn read_raw_adc(&mut self) -> Result<i32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.read_reg(reg::RAW_ADC, &mut buf)?;
        Ok(raw_adc(buf))
    }

    /// Read a temperature register (TH, TΔ or TC) in 0.01 °C.
    pub fn read_temperature(&mut self, reg: u8) -> Result<i32, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.read_reg(reg, &mut buf)?;
        Ok(centi(buf))
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Mcp9600<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Clears the status flags and starts a burst (or switches to
    /// continuous mode).
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        if !self.configured {
            self.write_reg(reg::SENSOR_CONFIG, self.sensor_config())?;
            self.configured = true;
        }
        self.write_reg(reg::STATUS, 0)?;
        self.write_reg(reg::DEVICE_CONFIG, self.device_config(self.mode_bits()))?;
        Ok(self.conversion_us())
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let status = self.read_status()?;
        check_status(status, self.mode)?;
        Ok(Measurement {
            temperature: self.read_temperature(reg::HOT_JUNCTION)?,
            cold_junction: self.read_temperature(reg::COLD_JUNCTION)?,
        })
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Mcp9600<I2C> {
    type BusError = I2C::Error;

    /// Shutdown mode (2 µA typ). Burst mode returns there by itself.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_reg(reg::DEVICE_CONFIG, self.device_config(0b01))
    }

    /// Continuous mode: resume conversions and return one conversion time.
    /// Burst mode: nothing to do (`start_measurement` starts the burst).
    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        match self.mode {
            Mode::Burst(_) => Ok(0),
            Mode::Continuous => {
                self.write_reg(reg::DEVICE_CONFIG, self.device_config(0b00))?;
                Ok(self.conversion_us())
            }
        }
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        AdcResolution, CALC_US, ColdJunctionResolution, DeviceId, Measurement, Mode,
        ThermocoupleType, centi, check_status, raw_adc, reg,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async driver; same API as [`super::Mcp9600`].
    pub struct Mcp9600<I2C> {
        i2c: I2C,
        address: u8,
        thermocouple: ThermocoupleType,
        filter: u8,
        adc: AdcResolution,
        cold_junction: ColdJunctionResolution,
        mode: Mode,
        configured: bool,
    }

    impl<I2C> Mcp9600<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Mcp9600<I2C> {
        async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[reg, value])
                .await
                .map_err(Error::bus)
        }

        async fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[reg])
                .await
                .map_err(Error::bus)?;
            self.i2c.read(self.address, buf).await.map_err(Error::bus)
        }

        /// See [`super::Mcp9600::probe`].
        pub async fn probe(&mut self) -> Result<DeviceId, Error<I2C::Error>> {
            let mut buf = [0u8; 2];
            self.read_reg(reg::DEVICE_ID, &mut buf).await?;
            DeviceId::decode(buf)
        }

        /// See [`super::Mcp9600::configure`].
        pub async fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_reg(reg::SENSOR_CONFIG, self.sensor_config())
                .await?;
            self.write_reg(reg::DEVICE_CONFIG, self.device_config(0b01))
                .await?;
            self.configured = true;
            Ok(())
        }

        /// See [`super::Mcp9600::read_status`].
        pub async fn read_status(&mut self) -> Result<u8, Error<I2C::Error>> {
            let mut buf = [0u8];
            self.read_reg(reg::STATUS, &mut buf).await?;
            Ok(buf[0])
        }

        /// See [`super::Mcp9600::read_raw_adc`].
        pub async fn read_raw_adc(&mut self) -> Result<i32, Error<I2C::Error>> {
            let mut buf = [0u8; 3];
            self.read_reg(reg::RAW_ADC, &mut buf).await?;
            Ok(raw_adc(buf))
        }

        /// See [`super::Mcp9600::read_temperature`].
        pub async fn read_temperature(&mut self, reg: u8) -> Result<i32, Error<I2C::Error>> {
            let mut buf = [0u8; 2];
            self.read_reg(reg, &mut buf).await?;
            Ok(centi(buf))
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Mcp9600<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if !self.configured {
                self.write_reg(reg::SENSOR_CONFIG, self.sensor_config())
                    .await?;
                self.configured = true;
            }
            self.write_reg(reg::STATUS, 0).await?;
            self.write_reg(reg::DEVICE_CONFIG, self.device_config(self.mode_bits()))
                .await?;
            Ok(self.conversion_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let status = self.read_status().await?;
            check_status(status, self.mode)?;
            Ok(Measurement {
                temperature: self.read_temperature(reg::HOT_JUNCTION).await?,
                cold_junction: self.read_temperature(reg::COLD_JUNCTION).await?,
            })
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Mcp9600<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_reg(reg::DEVICE_CONFIG, self.device_config(0b01))
                .await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            match self.mode {
                Mode::Burst(_) => Ok(0),
                Mode::Continuous => {
                    self.write_reg(reg::DEVICE_CONFIG, self.device_config(0b00))
                        .await?;
                    Ok(self.conversion_us())
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
