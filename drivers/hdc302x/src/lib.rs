//! Texas Instruments HDC3020 / HDC3021 / HDC3022 temperature and humidity
//! sensors (I2C).
//!
//! The three parts are electrically identical; HDC3021 adds a removable
//! assembly tape and HDC3022 a permanent IP67 PTFE membrane.
//!
//! The driver uses trigger-on-demand mode: one command starts a T + RH
//! conversion at the selected low-power mode (LPM0 lowest noise … LPM3
//! lowest power), after which the part returns to sleep (0.36 µA typ.).
//! Reading before the conversion is done is NACKed by the device, which
//! shows up as [`Error::Bus`]. Every word carries a CRC-8 (poly 0x31,
//! init 0xFF).
//!
//! Reference: TI SNAS778D "HDC302x" (rev. July 2024).
//!
//! ```ignore
//! let mut hdc = Hdc302x::new(i2c).with_mode(PowerMode::Lpm2);
//! hdc.verify_id()?;
//! let wait_us = hdc.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = hdc.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::crc::crc8_sensirion;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with ADDR = ADDR1 = GND.
pub const DEFAULT_ADDRESS: u8 = 0x44;
/// I2C address with ADDR = VDD, ADDR1 = GND.
pub const ADDRESS_45: u8 = 0x45;
/// I2C address with ADDR = GND, ADDR1 = VDD.
pub const ADDRESS_46: u8 = 0x46;
/// I2C address with ADDR = ADDR1 = VDD.
pub const ADDRESS_47: u8 = 0x47;

const CMD_SOFT_RESET: [u8; 2] = [0x30, 0xA2];
const CMD_READ_MANUFACTURER_ID: [u8; 2] = [0x37, 0x81];
const CMD_READ_STATUS: [u8; 2] = [0xF3, 0x2D];
const CMD_CLEAR_STATUS: [u8; 2] = [0x30, 0x41];
const CMD_HEATER_ENABLE: [u8; 2] = [0x30, 0x6D];
const CMD_HEATER_DISABLE: [u8; 2] = [0x30, 0x66];
const CMD_HEATER_CONFIG: [u8; 2] = [0x30, 0x6E];
const CMD_OFFSET: [u8; 2] = [0xA0, 0x04];
const CMD_NIST_ID: [[u8; 2]; 3] = [[0x36, 0x83], [0x36, 0x84], [0x36, 0x85]];

/// Manufacturer ID (Texas Instruments).
pub const MANUFACTURER_ID: u16 = 0x3000;
/// Sensor power-up ready time (max), also used after a soft reset.
pub const RESET_WAIT_US: WaitUs = 5_000;
/// Wait after programming the offset EEPROM before it can be read back.
pub const OFFSET_PROGRAM_WAIT_US: WaitUs = 77_000;

/// Heater configuration for full power (all 14 resistors).
pub const HEATER_FULL: u16 = 0x3FFF;
/// Heater configuration for half power.
pub const HEATER_HALF: u16 = 0x03FF;
/// Heater configuration for quarter power.
pub const HEATER_QUARTER: u16 = 0x009F;

/// Status register bit: at least one alert active.
pub const STATUS_ALERT: u16 = 1 << 15;
/// Status register bit: heater enabled.
pub const STATUS_HEATER: u16 = 1 << 13;
/// Status register bit: reset detected since the last clear.
pub const STATUS_RESET: u16 = 1 << 4;
/// Status register bit: CRC of the last write command failed.
pub const STATUS_WRITE_CRC_FAILED: u16 = 1 << 0;

/// Trigger-on-demand conversion mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PowerMode {
    /// LPM0: lowest noise, 12.5 ms typ.
    #[default]
    Lpm0,
    /// LPM1: 7.5 ms typ.
    Lpm1,
    /// LPM2: 5.0 ms typ.
    Lpm2,
    /// LPM3: lowest power, 3.7 ms typ.
    Lpm3,
}

impl PowerMode {
    /// Trigger-on-demand command.
    pub const fn command(self) -> [u8; 2] {
        match self {
            Self::Lpm0 => [0x24, 0x00],
            Self::Lpm1 => [0x24, 0x0B],
            Self::Lpm2 => [0x24, 0x16],
            Self::Lpm3 => [0x24, 0xFF],
        }
    }

    /// Maximum measurement duration t_MEAS.
    pub const fn wait_us(self) -> WaitUs {
        match self {
            Self::Lpm0 => 14_100,
            Self::Lpm1 => 8_400,
            Self::Lpm2 => 5_700,
            Self::Lpm3 => 4_200,
        }
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

/// Raw words to engineering units:
/// T = −45 + 175 × raw / 65535, RH = 100 × raw / 65535.
pub fn convert(raw_temperature: u16, raw_humidity: u16) -> Measurement {
    let t = (17_500 * i32::from(raw_temperature) + 32_767) / 65_535 - 4_500;
    let rh = (10_000 * u32::from(raw_humidity) + 32_767) / 65_535;
    Measurement {
        temperature: t,
        humidity: rh as u16,
    }
}

/// Check the CRC of one `[msb, lsb, crc]` word.
pub fn decode_word<E>(word: &[u8]) -> Result<u16, Error<E>> {
    if crc8_sensirion(&word[..2]) == word[2] {
        Ok(u16::from_be_bytes([word[0], word[1]]))
    } else {
        Err(Error::Crc)
    }
}

/// Decode the 6-byte readout (T, CRC, RH, CRC).
pub fn decode_measurement<E>(frame: &[u8; 6]) -> Result<Measurement, Error<E>> {
    let t = decode_word(&frame[..3])?;
    let rh = decode_word(&frame[3..])?;
    Ok(convert(t, rh))
}

/// Humidity offset byte (RH± bit 7, RHOS bits 13..7 in bits 6:0) for an offset in
/// 0.01 %RH; LSB 0.1953125 %RH, range ±24.80 %RH, outside it
/// [`Error::InvalidArgument`].
pub fn humidity_offset_byte<E>(offset: i32) -> Result<u8, Error<E>> {
    offset_byte(offset, 512, 10_000)
}

fn check_heater_power<E>(config: u16) -> Result<(), Error<E>> {
    if config > HEATER_FULL {
        Err(Error::InvalidArgument)
    } else {
        Ok(())
    }
}

fn offset_byte<E>(offset: i32, scale: u64, lsb: u64) -> Result<u8, Error<E>> {
    let magnitude = (u64::from(offset.unsigned_abs()) * scale + lsb / 2) / lsb;
    if magnitude > 0x7F {
        return Err(Error::InvalidArgument);
    }
    let magnitude = magnitude as u8;
    Ok(if offset > 0 {
        0x80 | magnitude
    } else {
        magnitude
    })
}

/// Offset in 0.01 %RH encoded by a humidity offset byte.
pub fn humidity_offset_centi(byte: u8) -> i32 {
    let magnitude = (i32::from(byte & 0x7F) * 10_000 + 256) / 512;
    if byte & 0x80 != 0 {
        magnitude
    } else {
        -magnitude
    }
}

/// Temperature offset byte (T± bit 7, TOS bits 12..6 in bits 6:0) for an offset in
/// 0.01 °C; LSB 0.1708984375 °C, range ±21.70 °C, outside it
/// [`Error::InvalidArgument`].
pub fn temperature_offset_byte<E>(offset: i32) -> Result<u8, Error<E>> {
    offset_byte(offset, 1_024, 17_500)
}

/// Offset in 0.01 °C encoded by a temperature offset byte.
pub fn temperature_offset_centi(byte: u8) -> i32 {
    let magnitude = (i32::from(byte & 0x7F) * 17_500 + 512) / 1_024;
    if byte & 0x80 != 0 {
        magnitude
    } else {
        -magnitude
    }
}

fn with_crc(command: [u8; 2], data: [u8; 2]) -> [u8; 5] {
    [
        command[0],
        command[1],
        data[0],
        data[1],
        crc8_sensirion(&data),
    ]
}

fn check_id<E>(word: &[u8; 3]) -> Result<(), Error<E>> {
    let id = decode_word(word)?;
    if id == MANUFACTURER_ID {
        Ok(())
    } else {
        Err(Error::UnexpectedId(u32::from(id)))
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver at [`DEFAULT_ADDRESS`](crate::DEFAULT_ADDRESS)
        /// using LPM0; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                address: DEFAULT_ADDRESS,
                mode: PowerMode::Lpm0,
            }
        }

        /// Use another I2C address.
        pub const fn with_address(mut self, address: u8) -> Self {
            self.address = address;
            self
        }

        /// Conversion mode used by `start_measurement`.
        pub const fn with_mode(mut self, mode: PowerMode) -> Self {
            self.mode = mode;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct Hdc302x<I2C> {
    i2c: I2C,
    address: u8,
    mode: PowerMode,
}

impl<I2C> Hdc302x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Hdc302x<I2C> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, bytes).map_err(Error::bus)
    }

    fn read_word(&mut self, command: [u8; 2]) -> Result<u16, Error<I2C::Error>> {
        let mut word = [0u8; 3];
        self.i2c
            .write_read(self.address, &command, &mut word)
            .map_err(Error::bus)?;
        decode_word(&word)
    }

    /// Check the manufacturer ID (0x3000); mismatch returns
    /// [`Error::UnexpectedId`].
    pub fn verify_id(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut word = [0u8; 3];
        self.i2c
            .write_read(self.address, &CMD_READ_MANUFACTURER_ID, &mut word)
            .map_err(Error::bus)?;
        check_id(&word)
    }

    /// 48-bit NIST-traceable unique ID.
    pub fn serial_number(&mut self) -> Result<u64, Error<I2C::Error>> {
        let mut id = 0u64;
        for command in CMD_NIST_ID {
            id = (id << 16) | u64::from(self.read_word(command)?);
        }
        Ok(id)
    }

    /// Soft reset. Wait [`RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&CMD_SOFT_RESET)?;
        Ok(RESET_WAIT_US)
    }

    /// Read the status register (see the `STATUS_*` constants).
    pub fn status(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_word(CMD_READ_STATUS)
    }

    /// Clear the reset and alert tracking bits.
    pub fn clear_status(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write(&CMD_CLEAR_STATUS)
    }

    /// Switch the heater on or off. Measurements are not valid while it is on.
    pub fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
        self.write(if on {
            &CMD_HEATER_ENABLE
        } else {
            &CMD_HEATER_DISABLE
        })
    }

    /// Select the heater resistors (e.g. [`HEATER_FULL`]); a value above
    /// [`HEATER_FULL`] (14 bits) is [`Error::InvalidArgument`].
    pub fn set_heater_power(&mut self, config: u16) -> Result<(), Error<I2C::Error>> {
        check_heater_power(config)?;
        self.write(&with_crc(CMD_HEATER_CONFIG, config.to_be_bytes()))
    }

    /// Program the non-volatile RH and T offset bytes (see
    /// [`humidity_offset_byte`] and [`temperature_offset_byte`]). The device
    /// must not be in auto-measurement mode. Returns
    /// [`OFFSET_PROGRAM_WAIT_US`] during which the EEPROM is busy.
    pub fn program_offsets(
        &mut self,
        humidity: u8,
        temperature: u8,
    ) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&with_crc(CMD_OFFSET, [humidity, temperature]))?;
        Ok(OFFSET_PROGRAM_WAIT_US)
    }

    /// Read back the programmed `(humidity, temperature)` offset bytes.
    pub fn offsets(&mut self) -> Result<(u8, u8), Error<I2C::Error>> {
        let [rh, t] = self.read_word(CMD_OFFSET)?.to_be_bytes();
        Ok((rh, t))
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Hdc302x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&self.mode.command())?;
        Ok(self.mode.wait_us())
    }

    /// Returns [`Error::Bus`] (NACK) if the conversion is still running.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut frame = [0u8; 6];
        self.i2c
            .read(self.address, &mut frame)
            .map_err(Error::bus)?;
        decode_measurement(&frame)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_CLEAR_STATUS, CMD_HEATER_CONFIG, CMD_HEATER_DISABLE, CMD_HEATER_ENABLE, CMD_NIST_ID,
        CMD_OFFSET, CMD_READ_MANUFACTURER_ID, CMD_READ_STATUS, CMD_SOFT_RESET, DEFAULT_ADDRESS,
        Measurement, OFFSET_PROGRAM_WAIT_US, PowerMode, RESET_WAIT_US, check_heater_power,
        check_id, decode_measurement, decode_word, with_crc,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Hdc302x`].
    pub struct Hdc302x<I2C> {
        i2c: I2C,
        address: u8,
        mode: PowerMode,
    }

    impl<I2C> Hdc302x<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Hdc302x<I2C> {
        async fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, bytes)
                .await
                .map_err(Error::bus)
        }

        async fn read_word(&mut self, command: [u8; 2]) -> Result<u16, Error<I2C::Error>> {
            let mut word = [0u8; 3];
            self.i2c
                .write_read(self.address, &command, &mut word)
                .await
                .map_err(Error::bus)?;
            decode_word(&word)
        }

        /// See [`super::Hdc302x::verify_id`].
        pub async fn verify_id(&mut self) -> Result<(), Error<I2C::Error>> {
            let mut word = [0u8; 3];
            self.i2c
                .write_read(self.address, &CMD_READ_MANUFACTURER_ID, &mut word)
                .await
                .map_err(Error::bus)?;
            check_id(&word)
        }

        /// See [`super::Hdc302x::serial_number`].
        pub async fn serial_number(&mut self) -> Result<u64, Error<I2C::Error>> {
            let mut id = 0u64;
            for command in CMD_NIST_ID {
                id = (id << 16) | u64::from(self.read_word(command).await?);
            }
            Ok(id)
        }

        /// See [`super::Hdc302x::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&CMD_SOFT_RESET).await?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Hdc302x::status`].
        pub async fn status(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.read_word(CMD_READ_STATUS).await
        }

        /// See [`super::Hdc302x::clear_status`].
        pub async fn clear_status(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write(&CMD_CLEAR_STATUS).await
        }

        /// See [`super::Hdc302x::set_heater`].
        pub async fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
            self.write(if on {
                &CMD_HEATER_ENABLE
            } else {
                &CMD_HEATER_DISABLE
            })
            .await
        }

        /// See [`super::Hdc302x::set_heater_power`].
        pub async fn set_heater_power(&mut self, config: u16) -> Result<(), Error<I2C::Error>> {
            check_heater_power(config)?;
            self.write(&with_crc(CMD_HEATER_CONFIG, config.to_be_bytes()))
                .await
        }

        /// See [`super::Hdc302x::program_offsets`].
        pub async fn program_offsets(
            &mut self,
            humidity: u8,
            temperature: u8,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&with_crc(CMD_OFFSET, [humidity, temperature]))
                .await?;
            Ok(OFFSET_PROGRAM_WAIT_US)
        }

        /// See [`super::Hdc302x::offsets`].
        pub async fn offsets(&mut self) -> Result<(u8, u8), Error<I2C::Error>> {
            let [rh, t] = self.read_word(CMD_OFFSET).await?.to_be_bytes();
            Ok((rh, t))
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Hdc302x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&self.mode.command()).await?;
            Ok(self.mode.wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut frame = [0u8; 6];
            self.i2c
                .read(self.address, &mut frame)
                .await
                .map_err(Error::bus)?;
            decode_measurement(&frame)
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

    #[test]
    fn crc_matches_datasheet_examples() {
        // Table 7-1: CRC(0xABCD) = 0x6F; Table 7-15 heater configurations.
        assert_eq!(decode_word::<()>(&[0xAB, 0xCD, 0x6F]), Ok(0xABCD));
        assert_eq!(decode_word::<()>(&[0x3F, 0xFF, 0x06]), Ok(HEATER_FULL));
        assert_eq!(decode_word::<()>(&[0x03, 0xFF, 0x00]), Ok(HEATER_HALF));
        assert_eq!(decode_word::<()>(&[0x00, 0x9F, 0x96]), Ok(HEATER_QUARTER));
        assert_eq!(decode_word::<()>(&[0xAB, 0xCD, 0x6E]), Err(Error::Crc));
    }

    #[test]
    fn conversion() {
        assert_eq!(convert(0, 0).temperature, -4_500);
        assert_eq!(
            convert(0xFFFF, 0xFFFF),
            Measurement {
                temperature: 13_000,
                humidity: 10_000
            }
        );
        // 0x6666 = 0.4 × 65535: −45 + 70 = 25.00 °C; 0x8000 = 50.00 %RH.
        assert_eq!(
            decode_measurement::<()>(&[0x66, 0x66, 0x93, 0x80, 0x00, 0xA2]),
            Ok(Measurement {
                temperature: 2_500,
                humidity: 5_000
            })
        );
        assert_eq!(
            decode_measurement::<()>(&[0x66, 0x66, 0x93, 0x80, 0x00, 0xA3]),
            Err(Error::Crc)
        );
    }

    #[test]
    fn offset_encoding_matches_datasheet_tables() {
        // Table 7-11 / 7-13: 0xAA = +8.203125 %RH, 0x2A = −7.177734375 °C.
        assert_eq!(humidity_offset_byte::<()>(820), Ok(0xAA));
        assert_eq!(humidity_offset_centi(0xAA), 820);
        assert_eq!(humidity_offset_centi(0x2A), -820);
        assert_eq!(humidity_offset_centi(0xFF), 2_480);
        assert_eq!(temperature_offset_byte::<()>(-717), Ok(0x2A));
        assert_eq!(temperature_offset_centi(0x2A), -718);
        assert_eq!(temperature_offset_centi(0x81), 17);
        assert_eq!(temperature_offset_centi(0xFF), 2_170);
        assert_eq!(temperature_offset_byte::<()>(2_170), Ok(0xFF));
        assert_eq!(
            temperature_offset_byte::<()>(10_000),
            Err(Error::InvalidArgument)
        );
        assert_eq!(humidity_offset_byte::<()>(-2_480), Ok(0x7F));
        assert_eq!(
            humidity_offset_byte::<()>(-2_500),
            Err(Error::InvalidArgument)
        );
        assert_eq!(humidity_offset_byte::<()>(0), Ok(0x00));
    }

    #[test]
    fn trigger_read_not_ready_and_commands() {
        let expectations = [
            Transaction::write(ADDRESS_45, vec![0x24, 0x16]),
            Transaction::read(ADDRESS_45, vec![0; 6])
                .with_error(embedded_hal::i2c::ErrorKind::Other),
            Transaction::read(ADDRESS_45, vec![0x66, 0x66, 0x93, 0x80, 0x00, 0xA2]),
            Transaction::write(ADDRESS_45, vec![0x30, 0x6E, 0x3F, 0xFF, 0x06]),
            Transaction::write(ADDRESS_45, vec![0x30, 0x6D]),
            Transaction::write(ADDRESS_45, vec![0xA0, 0x04, 0xAA, 0x2A, 0xCD]),
            Transaction::write_read(ADDRESS_45, vec![0xA0, 0x04], vec![0xAA, 0x2A, 0xCD]),
            Transaction::write_read(ADDRESS_45, vec![0xF3, 0x2D], vec![0x80, 0x10, 0xE1]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hdc = Hdc302x::new(&mut i2c)
            .with_address(ADDRESS_45)
            .with_mode(PowerMode::Lpm2);
        assert_eq!(hdc.start_measurement().unwrap(), 5_700);
        assert!(matches!(hdc.read_measurement(), Err(Error::Bus(_))));
        assert_eq!(hdc.read_measurement().unwrap().temperature, 2_500);
        assert_eq!(hdc.set_heater_power(0x4000), Err(Error::InvalidArgument));
        hdc.set_heater_power(HEATER_FULL).unwrap();
        hdc.set_heater(true).unwrap();
        assert_eq!(
            hdc.program_offsets(
                humidity_offset_byte::<()>(820).unwrap(),
                temperature_offset_byte::<()>(-717).unwrap()
            ),
            Ok(OFFSET_PROGRAM_WAIT_US)
        );
        assert_eq!(hdc.offsets(), Ok((0xAA, 0x2A)));
        assert_eq!(hdc.status(), Ok(STATUS_ALERT | STATUS_RESET));
        i2c.done();
    }

    #[test]
    fn id_and_serial() {
        let expectations = [
            Transaction::write_read(DEFAULT_ADDRESS, vec![0x37, 0x81], vec![0x30, 0x00, 0x33]),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0x37, 0x81], vec![0x00, 0x10, 0xC2]),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0x37, 0x81], vec![0x30, 0x00, 0x34]),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0x36, 0x83], vec![0x12, 0x34, 0x37]),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0x36, 0x84], vec![0x56, 0x78, 0x7D]),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0x36, 0x85], vec![0x9A, 0xBC, 0xE0]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hdc = Hdc302x::new(&mut i2c);
        hdc.verify_id().unwrap();
        assert_eq!(hdc.verify_id(), Err(Error::UnexpectedId(0x0010)));
        assert_eq!(hdc.verify_id(), Err(Error::Crc));
        assert_eq!(hdc.serial_number(), Ok(0x1234_5678_9ABC));
        i2c.done();
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(DEFAULT_ADDRESS, vec![0x24, 0x00]),
            Transaction::read(DEFAULT_ADDRESS, vec![0x66, 0x66, 0x93, 0x80, 0x00, 0xA2]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hdc = asynch::Hdc302x::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        let m = block_on(hdc.measure(&mut delay)).unwrap();
        assert_eq!(m.humidity, 5_000);
        assert_eq!(delay.0, 14_100);
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
