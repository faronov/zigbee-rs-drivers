//! Silicon Labs Si7006 / Si7013 / Si7020 / Si7021, TE HTU21D(F) and
//! Sensirion SHT20 / SHT21 / SHT25 temperature and humidity sensors (I2C).
//!
//! All of these parts share the same command set: no-hold-master
//! measurement (0xF5 / 0xF3), user register (0xE6 / 0xE7), soft reset
//! (0xFE), electronic ID (0xFA 0x0F / 0xFC 0xC9) and CRC-8 (polynomial
//! 0x31, init 0x00) on measurement words. The Silicon Labs parts can also
//! return the temperature measured during the last humidity conversion
//! (0xE0), have a heater current register and a firmware revision.
//!
//! The parts drop to standby (60 nA Si7021) automatically after each
//! conversion, so no sleep command exists.
//!
//! References:
//! - Silicon Labs "Si7021-A20" datasheet, Rev. 1.2 (also covers Si7006 /
//!   Si7013 / Si7020, which use the same command set and device codes).
//! - TE Connectivity "HTU21D(F) Sensor Digital Relative Humidity &
//!   Temperature Sensor", HPC199_3 (Oct 2013).
//! - Sensirion "Datasheet SHT21" v4 and application note "Electronic
//!   Identification Code" v1.1.
//!
//! # Two-phase reads on HTU21D / SHT2x
//!
//! The HTU21D and SHT2x have no "temperature from previous RH" command, so
//! a measurement needs two conversions. The first
//! [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//! after [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! reads the humidity, triggers the temperature conversion and returns
//! [`Error::Pending`] with the temperature conversion time; wait that long
//! and call it again. [`zigbee_sensor::blocking::measure`] and
//! [`zigbee_sensor::asynch::Sensor::measure`] follow this automatically.
//!
//! ```ignore
//! let mut si = Si70xx::new(i2c, Variant::Si7021);
//! let wait_us = si.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = si.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs, crc::crc8_msb};

/// I2C address of every part (Si7013 with AD0 low).
pub const DEFAULT_ADDRESS: u8 = 0x40;
/// Si7013 address with AD0 high.
pub const SI7013_ALT_ADDRESS: u8 = 0x41;

const CMD_MEASURE_RH: u8 = 0xF5;
const CMD_MEASURE_T: u8 = 0xF3;
const CMD_READ_PREVIOUS_T: u8 = 0xE0;
const CMD_RESET: u8 = 0xFE;
const CMD_WRITE_USER: u8 = 0xE6;
const CMD_READ_USER: u8 = 0xE7;
const CMD_WRITE_HEATER: u8 = 0x51;
const CMD_READ_HEATER: u8 = 0x11;
const CMD_EID_FIRST: [u8; 2] = [0xFA, 0x0F];
const CMD_EID_SECOND: [u8; 2] = [0xFC, 0xC9];
const CMD_FIRMWARE: [u8; 2] = [0x84, 0xB8];
const MAX_HEATER_LEVEL: u8 = 15;

/// Soft reset time (Si7021 t_RST max 15 ms; HTU21D / SHT2x < 15 ms).
pub const RESET_WAIT_US: WaitUs = 15_000;
/// Power-up time to first command (Si7021 80 ms max over temperature,
/// SHT2x / HTU21D 15 ms).
pub const POWER_UP_WAIT_US: WaitUs = 80_000;

/// Supported parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// Silicon Labs Si7006.
    Si7006,
    /// Silicon Labs Si7013.
    Si7013,
    /// Silicon Labs Si7020.
    Si7020,
    /// Silicon Labs Si7021.
    Si7021,
    /// TE Connectivity HTU21D / HTU21DF.
    Htu21d,
    /// Sensirion SHT20.
    Sht20,
    /// Sensirion SHT21.
    Sht21,
    /// Sensirion SHT25.
    Sht25,
}

impl Variant {
    /// Silicon Labs part: supports 0xE0, heater register and firmware revision.
    pub const fn is_silabs(self) -> bool {
        matches!(
            self,
            Self::Si7006 | Self::Si7013 | Self::Si7020 | Self::Si7021
        )
    }

    /// Expected SNB_3 device code of the electronic ID (Si7021 datasheet
    /// section 5.3), `None` for parts that do not encode the model.
    pub const fn device_code(self) -> Option<u8> {
        match self {
            Self::Si7006 => Some(0x06),
            Self::Si7013 => Some(0x0D),
            Self::Si7020 => Some(0x14),
            Self::Si7021 => Some(0x15),
            _ => None,
        }
    }

    /// Maximum humidity conversion time. On Silicon Labs parts this includes
    /// the temperature conversion that every RH measurement performs.
    pub const fn humidity_wait_us(self, resolution: Resolution) -> WaitUs {
        let i = resolution as usize;
        match self {
            Self::Htu21d => [16_000, 3_000, 5_000, 8_000][i],
            Self::Sht20 | Self::Sht21 | Self::Sht25 => [29_000, 4_000, 9_000, 15_000][i],
            _ => [12_000, 3_100, 4_500, 7_000][i] + self.temperature_wait_us(resolution),
        }
    }

    /// Maximum temperature conversion time.
    pub const fn temperature_wait_us(self, resolution: Resolution) -> WaitUs {
        let i = resolution as usize;
        match self {
            Self::Htu21d => [50_000, 13_000, 25_000, 7_000][i],
            Self::Sht20 | Self::Sht21 | Self::Sht25 => [85_000, 22_000, 43_000, 11_000][i],
            _ => [10_800, 3_800, 6_200, 2_400][i],
        }
    }
}

/// Measurement resolution (user register bits D7 and D0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 12-bit RH, 14-bit temperature (power-on default).
    #[default]
    Rh12T14 = 0,
    /// 8-bit RH, 12-bit temperature.
    Rh8T12 = 1,
    /// 10-bit RH, 13-bit temperature.
    Rh10T13 = 2,
    /// 11-bit RH, 11-bit temperature.
    Rh11T11 = 3,
}

impl Resolution {
    const fn bits(self) -> u8 {
        let v = self as u8;
        ((v & 0b10) << 6) | (v & 0b01)
    }

    const fn from_bits(reg: u8) -> Self {
        match ((reg >> 6) & 0b10) | (reg & 0b01) {
            0 => Self::Rh12T14,
            1 => Self::Rh8T12,
            2 => Self::Rh10T13,
            _ => Self::Rh11T11,
        }
    }
}

/// User register 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UserRegister(pub u8);

impl UserRegister {
    const RES_MASK: u8 = 0x81;
    const HEATER: u8 = 1 << 2;

    /// Measurement resolution.
    pub const fn resolution(self) -> Resolution {
        Resolution::from_bits(self.0)
    }

    /// On-chip heater enabled.
    pub const fn heater_enabled(self) -> bool {
        self.0 & Self::HEATER != 0
    }

    /// VDD below the operating threshold (end of battery).
    pub const fn low_voltage(self) -> bool {
        self.0 & (1 << 6) != 0
    }

    /// Same register with a new resolution; reserved bits preserved.
    pub const fn with_resolution(self, resolution: Resolution) -> Self {
        Self((self.0 & !Self::RES_MASK) | resolution.bits())
    }

    /// Same register with the heater bit changed; reserved bits preserved.
    pub const fn with_heater(self, on: bool) -> Self {
        Self(if on {
            self.0 | Self::HEATER
        } else {
            self.0 & !Self::HEATER
        })
    }
}

/// The 8 raw bytes of the electronic ID / serial number.
///
/// `first` holds the four data bytes of the 0xFA 0x0F read (SNA_3..SNA_0 on
/// Silicon Labs parts), `second` the four of the 0xFC 0xC9 read
/// (SNB_3..SNB_0). The interleaved CRC bytes are not verified: none of the
/// datasheets fully specify their coverage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ElectronicId {
    /// Data bytes of the first read.
    pub first: [u8; 4],
    /// Data bytes of the second read.
    pub second: [u8; 4],
}

impl ElectronicId {
    /// Decode the two raw reads (8 bytes with CRCs after each data byte,
    /// then 6 bytes with a CRC after each pair).
    pub const fn decode(first: &[u8; 8], second: &[u8; 6]) -> Self {
        Self {
            first: [first[0], first[2], first[4], first[6]],
            second: [second[0], second[1], second[3], second[4]],
        }
    }

    /// 64-bit serial number, first read in the high half.
    pub const fn serial(&self) -> u64 {
        ((u32::from_be_bytes(self.first) as u64) << 32) | u32::from_be_bytes(self.second) as u64
    }

    /// SNB_3, the Silicon Labs device code (0x15 = Si7021, ...).
    pub const fn device_code(&self) -> u8 {
        self.second[0]
    }
}

/// One temperature + humidity sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Relative humidity in 0.01 %RH.
    pub humidity: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
        f(Quantity::Humidity(self.humidity));
    }
}

/// CRC-8 used on measurement words (x^8 + x^5 + x^4 + 1, init 0x00).
pub fn crc8(bytes: &[u8]) -> u8 {
    crc8_msb(bytes, 0x31, 0x00)
}

/// Temperature code to 0.01 °C: T = 175.72 × code / 65536 − 46.85.
pub fn convert_temperature(code: u16) -> i32 {
    ((17_572 * i64::from(code & !0x3) + 32_768) >> 16) as i32 - 4_685
}

/// Humidity code to 0.01 %RH: RH = 125 × code / 65536 − 6, clamped to
/// 0..100 % as the datasheets recommend.
pub fn convert_humidity(code: u16) -> u16 {
    let rh = ((12_500 * i32::from(code & !0x3) + 32_768) >> 16) - 600;
    rh.clamp(0, 10_000) as u16
}

/// Check the CRC of a 3-byte measurement frame and return the raw word.
/// Bit 1 of the LSB must match `humidity` (1 = RH, 0 = temperature).
pub fn decode_word<E>(frame: &[u8; 3], humidity: bool) -> Result<u16, Error<E>> {
    if crc8(&frame[..2]) != frame[2] {
        return Err(Error::Crc);
    }
    if (frame[1] & 0x2 != 0) != humidity {
        return Err(Error::InvalidData);
    }
    Ok(u16::from_be_bytes([frame[0], frame[1]]))
}

/// Next step after reading the humidity frame.
enum Step {
    /// Read 0xE0 and finish.
    PreviousTemperature,
    /// Trigger a temperature conversion and come back later.
    MeasureTemperature,
}

macro_rules! constructors {
    () => {
        /// Create a driver at [`DEFAULT_ADDRESS`](crate::DEFAULT_ADDRESS); no bus traffic.
        pub const fn new(i2c: I2C, variant: Variant) -> Self {
            Self {
                i2c,
                address: DEFAULT_ADDRESS,
                variant,
                resolution: Resolution::Rh12T14,
                pending_humidity: None,
            }
        }

        /// Use another I2C address (Si7013 [`SI7013_ALT_ADDRESS`](crate::SI7013_ALT_ADDRESS)).
        pub const fn with_address(mut self, address: u8) -> Self {
            self.address = address;
            self
        }

        /// Tell the driver which resolution the part is configured for
        /// (used for wait times only; see `set_resolution`).
        pub const fn with_resolution(mut self, resolution: Resolution) -> Self {
            self.resolution = resolution;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        /// Configured variant.
        pub const fn variant(&self) -> Variant {
            self.variant
        }

        fn step_after_humidity(&self) -> Step {
            if self.variant.is_silabs() {
                Step::PreviousTemperature
            } else {
                Step::MeasureTemperature
            }
        }

        fn check_device_code(&self, id: &ElectronicId) -> Result<(), Error<I2C::Error>>
        where
            I2C: embedded_hal::i2c::ErrorType,
        {
            let code = id.device_code();
            match self.variant.device_code() {
                None => Err(Error::Unsupported),
                // Engineering samples report 0x00 or 0xFF.
                Some(_) if code == 0x00 || code == 0xFF => Ok(()),
                Some(expected) if expected == code => Ok(()),
                Some(_) => Err(Error::UnexpectedId(u32::from(code))),
            }
        }
    };
}

/// Blocking driver.
pub struct Si70xx<I2C> {
    i2c: I2C,
    address: u8,
    variant: Variant,
    resolution: Resolution,
    pending_humidity: Option<u16>,
}

impl<I2C> Si70xx<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Si70xx<I2C> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, bytes).map_err(Error::bus)
    }

    fn write_read(&mut self, cmd: &[u8], buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(self.address, cmd, buf)
            .map_err(Error::bus)
    }

    /// Soft reset; resets the user register to defaults. Wait [`RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.pending_humidity = None;
        self.resolution = Resolution::Rh12T14;
        self.write(&[CMD_RESET])?;
        Ok(RESET_WAIT_US)
    }

    /// Read user register 1.
    pub fn read_user_register(&mut self) -> Result<UserRegister, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.write_read(&[CMD_READ_USER], &mut buf)?;
        Ok(UserRegister(buf[0]))
    }

    /// Write user register 1 (use the value from `read_user_register` so
    /// reserved bits are preserved).
    pub fn write_user_register(&mut self, reg: UserRegister) -> Result<(), Error<I2C::Error>> {
        self.write(&[CMD_WRITE_USER, reg.0])?;
        self.resolution = reg.resolution();
        Ok(())
    }

    /// Change the resolution (read-modify-write of the user register).
    pub fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error<I2C::Error>> {
        let reg = self.read_user_register()?;
        self.write_user_register(reg.with_resolution(resolution))
    }

    /// Switch the on-chip heater (read-modify-write of the user register).
    pub fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
        let reg = self.read_user_register()?;
        self.write_user_register(reg.with_heater(on))
    }

    /// Set the heater current, 0 (3.09 mA) ..= 15 (94.20 mA). Silicon Labs
    /// only; a level above 15 is [`Error::InvalidArgument`].
    pub fn set_heater_level(&mut self, level: u8) -> Result<(), Error<I2C::Error>> {
        if !self.variant.is_silabs() {
            return Err(Error::Unsupported);
        }
        if level > MAX_HEATER_LEVEL {
            return Err(Error::InvalidArgument);
        }
        let mut buf = [0u8; 1];
        self.write_read(&[CMD_READ_HEATER], &mut buf)?;
        self.write(&[CMD_WRITE_HEATER, (buf[0] & 0xF0) | level])
    }

    /// Read the 64-bit electronic ID.
    pub fn read_electronic_id(&mut self) -> Result<ElectronicId, Error<I2C::Error>> {
        let mut first = [0u8; 8];
        let mut second = [0u8; 6];
        self.write_read(&CMD_EID_FIRST, &mut first)?;
        self.write_read(&CMD_EID_SECOND, &mut second)?;
        Ok(ElectronicId::decode(&first, &second))
    }

    /// Read the electronic ID and check the device code against the
    /// variant. Silicon Labs only; returns [`Error::UnexpectedId`] with the
    /// code found on mismatch.
    pub fn verify_device(&mut self) -> Result<ElectronicId, Error<I2C::Error>> {
        if !self.variant.is_silabs() {
            return Err(Error::Unsupported);
        }
        let id = self.read_electronic_id()?;
        self.check_device_code(&id)?;
        Ok(id)
    }

    /// Firmware revision byte (0xFF = 1.0, 0x20 = 2.0). Silicon Labs only.
    pub fn firmware_revision(&mut self) -> Result<u8, Error<I2C::Error>> {
        if !self.variant.is_silabs() {
            return Err(Error::Unsupported);
        }
        let mut buf = [0u8; 1];
        self.write_read(&CMD_FIRMWARE, &mut buf)?;
        Ok(buf[0])
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Si70xx<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.pending_humidity = None;
        self.write(&[CMD_MEASURE_RH])?;
        Ok(self.variant.humidity_wait_us(self.resolution))
    }

    /// Returns [`Error::Bus`] (NACK) while converting, and
    /// [`Error::Pending`] after starting the HTU21D / SHT2x temperature
    /// conversion (see the crate docs).
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut frame = [0u8; 3];
        if let Some(rh) = self.pending_humidity {
            self.i2c
                .read(self.address, &mut frame)
                .map_err(Error::bus)?;
            let t = decode_word(&frame, false)?;
            self.pending_humidity = None;
            return Ok(Measurement {
                temperature: convert_temperature(t),
                humidity: convert_humidity(rh),
            });
        }
        self.i2c
            .read(self.address, &mut frame)
            .map_err(Error::bus)?;
        let rh = decode_word(&frame, true)?;
        match self.step_after_humidity() {
            Step::PreviousTemperature => {
                let mut t = [0u8; 2];
                self.write_read(&[CMD_READ_PREVIOUS_T], &mut t)?;
                Ok(Measurement {
                    temperature: convert_temperature(u16::from_be_bytes(t)),
                    humidity: convert_humidity(rh),
                })
            }
            Step::MeasureTemperature => {
                self.write(&[CMD_MEASURE_T])?;
                self.pending_humidity = Some(rh);
                Err(Error::Pending(
                    self.variant.temperature_wait_us(self.resolution),
                ))
            }
        }
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_EID_FIRST, CMD_EID_SECOND, CMD_FIRMWARE, CMD_MEASURE_RH, CMD_MEASURE_T,
        CMD_READ_HEATER, CMD_READ_PREVIOUS_T, CMD_READ_USER, CMD_RESET, CMD_WRITE_HEATER,
        CMD_WRITE_USER, DEFAULT_ADDRESS, ElectronicId, MAX_HEATER_LEVEL, Measurement,
        RESET_WAIT_US, Resolution, Step, UserRegister, Variant, convert_humidity,
        convert_temperature, decode_word,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Si70xx`].
    pub struct Si70xx<I2C> {
        i2c: I2C,
        address: u8,
        variant: Variant,
        resolution: Resolution,
        pending_humidity: Option<u16>,
    }

    impl<I2C> Si70xx<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Si70xx<I2C> {
        async fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, bytes)
                .await
                .map_err(Error::bus)
        }

        async fn write_read(
            &mut self,
            cmd: &[u8],
            buf: &mut [u8],
        ) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write_read(self.address, cmd, buf)
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Si70xx::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.pending_humidity = None;
            self.resolution = Resolution::Rh12T14;
            self.write(&[CMD_RESET]).await?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Si70xx::read_user_register`].
        pub async fn read_user_register(&mut self) -> Result<UserRegister, Error<I2C::Error>> {
            let mut buf = [0u8; 1];
            self.write_read(&[CMD_READ_USER], &mut buf).await?;
            Ok(UserRegister(buf[0]))
        }

        /// See [`super::Si70xx::write_user_register`].
        pub async fn write_user_register(
            &mut self,
            reg: UserRegister,
        ) -> Result<(), Error<I2C::Error>> {
            self.write(&[CMD_WRITE_USER, reg.0]).await?;
            self.resolution = reg.resolution();
            Ok(())
        }

        /// See [`super::Si70xx::set_resolution`].
        pub async fn set_resolution(
            &mut self,
            resolution: Resolution,
        ) -> Result<(), Error<I2C::Error>> {
            let reg = self.read_user_register().await?;
            self.write_user_register(reg.with_resolution(resolution))
                .await
        }

        /// See [`super::Si70xx::set_heater`].
        pub async fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
            let reg = self.read_user_register().await?;
            self.write_user_register(reg.with_heater(on)).await
        }

        /// See [`super::Si70xx::set_heater_level`].
        pub async fn set_heater_level(&mut self, level: u8) -> Result<(), Error<I2C::Error>> {
            if !self.variant.is_silabs() {
                return Err(Error::Unsupported);
            }
            if level > MAX_HEATER_LEVEL {
                return Err(Error::InvalidArgument);
            }
            let mut buf = [0u8; 1];
            self.write_read(&[CMD_READ_HEATER], &mut buf).await?;
            self.write(&[CMD_WRITE_HEATER, (buf[0] & 0xF0) | level])
                .await
        }

        /// See [`super::Si70xx::read_electronic_id`].
        pub async fn read_electronic_id(&mut self) -> Result<ElectronicId, Error<I2C::Error>> {
            let mut first = [0u8; 8];
            let mut second = [0u8; 6];
            self.write_read(&CMD_EID_FIRST, &mut first).await?;
            self.write_read(&CMD_EID_SECOND, &mut second).await?;
            Ok(ElectronicId::decode(&first, &second))
        }

        /// See [`super::Si70xx::verify_device`].
        pub async fn verify_device(&mut self) -> Result<ElectronicId, Error<I2C::Error>> {
            if !self.variant.is_silabs() {
                return Err(Error::Unsupported);
            }
            let id = self.read_electronic_id().await?;
            self.check_device_code(&id)?;
            Ok(id)
        }

        /// See [`super::Si70xx::firmware_revision`].
        pub async fn firmware_revision(&mut self) -> Result<u8, Error<I2C::Error>> {
            if !self.variant.is_silabs() {
                return Err(Error::Unsupported);
            }
            let mut buf = [0u8; 1];
            self.write_read(&CMD_FIRMWARE, &mut buf).await?;
            Ok(buf[0])
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Si70xx<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.pending_humidity = None;
            self.write(&[CMD_MEASURE_RH]).await?;
            Ok(self.variant.humidity_wait_us(self.resolution))
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut frame = [0u8; 3];
            if let Some(rh) = self.pending_humidity {
                self.i2c
                    .read(self.address, &mut frame)
                    .await
                    .map_err(Error::bus)?;
                let t = decode_word(&frame, false)?;
                self.pending_humidity = None;
                return Ok(Measurement {
                    temperature: convert_temperature(t),
                    humidity: convert_humidity(rh),
                });
            }
            self.i2c
                .read(self.address, &mut frame)
                .await
                .map_err(Error::bus)?;
            let rh = decode_word(&frame, true)?;
            match self.step_after_humidity() {
                Step::PreviousTemperature => {
                    let mut t = [0u8; 2];
                    self.write_read(&[CMD_READ_PREVIOUS_T], &mut t).await?;
                    Ok(Measurement {
                        temperature: convert_temperature(u16::from_be_bytes(t)),
                        humidity: convert_humidity(rh),
                    })
                }
                Step::MeasureTemperature => {
                    self.write(&[CMD_MEASURE_T]).await?;
                    self.pending_humidity = Some(rh);
                    Err(Error::Pending(
                        self.variant.temperature_wait_us(self.resolution),
                    ))
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::Sensor;

    fn frame(word: u16) -> std::vec::Vec<u8> {
        let [a, b] = word.to_be_bytes();
        vec![a, b, crc8(&[a, b])]
    }

    #[test]
    fn crc_matches_htu21d_examples() {
        assert_eq!(crc8(&[0xDC]), 0x79);
        assert_eq!(crc8(&[0x68, 0x3A]), 0x7C);
        assert_eq!(crc8(&[0x4E, 0x85]), 0x6B);
    }

    #[test]
    fn conversion_matches_htu21d_examples() {
        // HTU21D: 0x683A -> 24.7 °C, 0x4E85 -> 32.3 %RH, 0x7C80 -> 54.8 %RH.
        assert_eq!(convert_temperature(0x683A), 2_469);
        assert_eq!(convert_humidity(0x4E85), 3_234);
        assert_eq!(convert_humidity(0x7C80), 5_479);
        // Ranges: -46.85 °C .. 128.87 °C, RH clamped to 0..100 %.
        assert_eq!(convert_temperature(0), -4_685);
        assert_eq!(convert_temperature(0xFFFC), 12_886);
        assert_eq!(convert_humidity(0), 0);
        assert_eq!(convert_humidity(0xFFFC), 10_000);
    }

    #[test]
    fn resolution_bits_round_trip() {
        let default_si = UserRegister(0x3A);
        assert_eq!(default_si.resolution(), Resolution::Rh12T14);
        let r = default_si.with_resolution(Resolution::Rh11T11);
        assert_eq!(r.0, 0xBB);
        assert_eq!(r.resolution(), Resolution::Rh11T11);
        assert_eq!(
            UserRegister(0x80).resolution(),
            Resolution::Rh10T13,
            "D7=1 D0=0"
        );
        assert_eq!(UserRegister(0x01).resolution(), Resolution::Rh8T12);
        assert!(UserRegister(0x3E).heater_enabled());
        assert_eq!(UserRegister(0x3E).with_heater(false).0, 0x3A);
    }

    #[test]
    fn si7021_reads_previous_temperature() {
        let expectations = [
            Transaction::write(DEFAULT_ADDRESS, vec![0xF5]),
            Transaction::read(DEFAULT_ADDRESS, frame(0x7C82)),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xE0], vec![0x68, 0x3A]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut si = Si70xx::new(&mut i2c, Variant::Si7021);
        assert_eq!(si.start_measurement().unwrap(), 12_000 + 10_800);
        assert_eq!(
            si.read_measurement().unwrap(),
            Measurement {
                temperature: 2_469,
                humidity: 5_479
            }
        );
        i2c.done();
    }

    #[test]
    fn htu21d_needs_second_conversion() {
        let expectations = [
            Transaction::write(DEFAULT_ADDRESS, vec![0xF5]),
            Transaction::read(DEFAULT_ADDRESS, frame(0x4E86)),
            Transaction::write(DEFAULT_ADDRESS, vec![0xF3]),
            Transaction::read(DEFAULT_ADDRESS, frame(0x6838)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut htu = Si70xx::new(&mut i2c, Variant::Htu21d);
        assert_eq!(htu.start_measurement().unwrap(), 16_000);
        assert_eq!(htu.read_measurement(), Err(Error::Pending(50_000)));
        assert_eq!(
            htu.read_measurement().unwrap(),
            Measurement {
                temperature: 2_469,
                humidity: 3_234
            }
        );
        i2c.done();
    }

    #[test]
    fn crc_error_and_wrong_status_bit() {
        let mut bad = frame(0x4E86);
        bad[2] ^= 1;
        let expectations = [
            Transaction::read(DEFAULT_ADDRESS, bad),
            Transaction::read(DEFAULT_ADDRESS, frame(0x683C)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sht = Si70xx::new(&mut i2c, Variant::Sht21);
        assert_eq!(sht.read_measurement(), Err(Error::Crc));
        // Temperature frame (bit 1 clear) where humidity was expected.
        assert_eq!(sht.read_measurement(), Err(Error::InvalidData));
        i2c.done();
    }

    #[test]
    fn set_resolution_preserves_reserved_bits() {
        let expectations = [
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xE7], vec![0x02]),
            Transaction::write(DEFAULT_ADDRESS, vec![0xE6, 0x03]),
            Transaction::write(DEFAULT_ADDRESS, vec![0xF5]),
            Transaction::read(DEFAULT_ADDRESS, frame(0x4E86)),
            Transaction::write(DEFAULT_ADDRESS, vec![0xF3]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut htu = Si70xx::new(&mut i2c, Variant::Htu21d);
        htu.set_resolution(Resolution::Rh8T12).unwrap();
        assert_eq!(htu.start_measurement().unwrap(), 3_000);
        assert_eq!(htu.read_measurement(), Err(Error::Pending(13_000)));
        i2c.done();
    }

    #[test]
    fn device_id_check() {
        let first = vec![0x11, 0, 0x22, 0, 0x33, 0, 0x44, 0];
        let second_ok = vec![0x15, 0xFF, 0, 0xAB, 0xCD, 0];
        let second_bad = vec![0x14, 0xFF, 0, 0xAB, 0xCD, 0];
        let expectations = [
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xFA, 0x0F], first.clone()),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xFC, 0xC9], second_ok),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xFA, 0x0F], first),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xFC, 0xC9], second_bad),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut si = Si70xx::new(&mut i2c, Variant::Si7021);
        let id = si.verify_device().unwrap();
        assert_eq!(id.serial(), 0x1122_3344_15FF_ABCD);
        assert_eq!(si.verify_device(), Err(Error::UnexpectedId(0x14)));
        i2c.done();

        let mut i2c = Mock::new(&[]);
        let mut htu = Si70xx::new(&mut i2c, Variant::Htu21d);
        assert_eq!(htu.firmware_revision(), Err(Error::Unsupported));
        assert_eq!(htu.set_heater_level(3), Err(Error::Unsupported));
        i2c.done();
    }

    #[test]
    fn heater_level_read_modify_write() {
        let expectations = [
            Transaction::write_read(DEFAULT_ADDRESS, vec![0x11], vec![0x30]),
            Transaction::write(DEFAULT_ADDRESS, vec![0x51, 0x3F]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut si = Si70xx::new(&mut i2c, Variant::Si7006);
        assert_eq!(si.set_heater_level(16), Err(Error::InvalidArgument));
        si.set_heater_level(15).unwrap();
        i2c.done();
    }

    #[test]
    fn async_two_phase_measure() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(DEFAULT_ADDRESS, vec![0xF5]),
            Transaction::read(DEFAULT_ADDRESS, frame(0x7C82)),
            Transaction::write(DEFAULT_ADDRESS, vec![0xF3]),
            Transaction::read(DEFAULT_ADDRESS, frame(0x6838)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sht = asynch::Si70xx::new(&mut i2c, Variant::Sht25);
        let mut delay = RecordingDelay(0);
        let m = block_on(sht.measure(&mut delay)).unwrap();
        assert_eq!(m.temperature, 2_469);
        assert_eq!(m.humidity, 5_479);
        assert_eq!(delay.0, 29_000 + 85_000);
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
