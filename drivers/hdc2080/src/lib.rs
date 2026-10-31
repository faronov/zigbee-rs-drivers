//! Texas Instruments HDC2010 / HDC2021 / HDC2022 / HDC2080 temperature and
//! humidity sensors (I2C).
//!
//! The parts share one register map and device ID (0x07D0); they differ in
//! package / protective cover and in the published temperature offset:
//! HDC2080 Rev C uses `T = raw/2^16 × 165 − 40.5`, the others `− 40`.
//!
//! The driver uses on-demand mode (AMM = 0): writing MEAS_TRIG starts one
//! T + RH conversion, after which the part returns to sleep (50 nA).
//! MEAS_TRIG self-clears when the conversion is done, which
//! `read_measurement` checks before reading the result.
//!
//! References: TI SNAS678C "HDC2080" (rev. July 2021), SNAS693D "HDC2010"
//! (rev. February 2021), SNAS773A "HDC2021" and SNAS774A "HDC2022"
//! (rev. June 2020).
//!
//! ```ignore
//! let mut hdc = Hdc2080::new(i2c, Variant::Hdc2080);
//! hdc.verify_id()?;
//! let wait_us = hdc.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = hdc.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with ADDR low or unconnected.
pub const PRIMARY_ADDRESS: u8 = 0x40;
/// I2C address with ADDR high.
pub const SECONDARY_ADDRESS: u8 = 0x41;

const REG_TEMPERATURE_LOW: u8 = 0x00;
const REG_TEMPERATURE_OFFSET: u8 = 0x08;
const REG_RESET_CONFIG: u8 = 0x0E;
const REG_MEASUREMENT_CONFIG: u8 = 0x0F;
const REG_MANUFACTURER_ID_LOW: u8 = 0xFC;

const RESET_CONFIG_SOFT_RESET: u8 = 1 << 7;
const RESET_CONFIG_HEATER: u8 = 1 << 3;
const MEAS_TRIG: u8 = 1 << 0;

/// Manufacturer ID (registers 0xFD:0xFC).
pub const MANUFACTURER_ID: u16 = 0x5449;
/// Device ID shared by all four parts (registers 0xFF:0xFE).
pub const DEVICE_ID: u16 = 0x07D0;
/// Power-on / soft-reset duration (HDC2021 t_POR 3.5 ms).
pub const RESET_WAIT_US: WaitUs = 3_500;

/// Supported parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// HDC2010 (WLCSP).
    Hdc2010,
    /// HDC2021 (removable assembly cover).
    Hdc2021,
    /// HDC2022 (IP67 PTFE membrane).
    Hdc2022,
    /// HDC2080 (WSON).
    Hdc2080,
}

impl Variant {
    /// Datasheet temperature offset in 0.01 °C.
    pub const fn temperature_offset(self) -> i32 {
        match self {
            Self::Hdc2080 => -4_050,
            _ => -4_000,
        }
    }
}

/// Resolution of a channel (TRES / HRES).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 14 bit.
    #[default]
    Bits14 = 0,
    /// 11 bit.
    Bits11 = 1,
    /// 9 bit.
    Bits9 = 2,
}

impl Resolution {
    /// Typical humidity conversion time (HDC2010 / HDC2080).
    pub const fn humidity_us(self) -> WaitUs {
        [660, 400, 275][self as usize]
    }

    /// Typical temperature conversion time (HDC2010 / HDC2080).
    pub const fn temperature_us(self) -> WaitUs {
        [610, 350, 225][self as usize]
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Temperature resolution.
    pub temperature: Resolution,
    /// Humidity resolution.
    pub humidity: Resolution,
}

impl Config {
    /// Measurement configuration register (0x0F) value with MEAS_TRIG set
    /// (MEAS_CONF = 00: humidity + temperature).
    pub const fn trigger_register(self) -> u8 {
        ((self.temperature as u8) << 6) | ((self.humidity as u8) << 4) | MEAS_TRIG
    }

    /// Wait for one T + RH conversion. The datasheets give typical values
    /// only, so 50 % margin is added.
    pub const fn wait_us(self) -> WaitUs {
        (self.temperature.temperature_us() + self.humidity.humidity_us()) * 3 / 2
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

/// Raw words to engineering units for `variant`:
/// T = raw / 2^16 × 165 + offset, RH = raw / 2^16 × 100.
pub fn convert(variant: Variant, raw_temperature: u16, raw_humidity: u16) -> Measurement {
    let t = ((16_500 * i32::from(raw_temperature) + 32_768) >> 16) + variant.temperature_offset();
    let rh = (10_000 * u32::from(raw_humidity) + 32_768) >> 16;
    Measurement {
        temperature: t,
        humidity: rh as u16,
    }
}

/// Decode registers 0x00..=0x03 (little-endian T, then RH).
pub fn decode_measurement(variant: Variant, frame: &[u8; 4]) -> Measurement {
    convert(
        variant,
        u16::from_le_bytes([frame[0], frame[1]]),
        u16::from_le_bytes([frame[2], frame[3]]),
    )
}

/// Temperature offset register (0x08) value for an offset in 0.01 °C.
/// Two's complement with an LSB of 165/1024 °C (±20.6 °C range).
pub fn temperature_offset_register(offset: i32) -> u8 {
    let raw = (offset * 1_024 + offset.signum() * 8_250) / 16_500;
    raw.clamp(-128, 127) as i8 as u8
}

/// Offset in 0.01 °C encoded by a temperature offset register value.
pub fn temperature_offset_centi(raw: u8) -> i32 {
    let v = i32::from(raw as i8) * 16_500;
    (v + v.signum() * 512) / 1_024
}

/// Humidity offset register (0x09) value for an offset in 0.01 %RH.
/// Two's complement with an LSB of 100/512 %RH (±25 %RH range).
pub fn humidity_offset_register(offset: i32) -> u8 {
    let raw = (offset * 512 + offset.signum() * 5_000) / 10_000;
    raw.clamp(-128, 127) as i8 as u8
}

/// Offset in 0.01 %RH encoded by a humidity offset register value.
pub fn humidity_offset_centi(raw: u8) -> i32 {
    let v = i32::from(raw as i8) * 10_000;
    (v + v.signum() * 256) / 512
}

fn check_ids<E>(ids: [u8; 4]) -> Result<(), Error<E>> {
    let manufacturer = u16::from_le_bytes([ids[0], ids[1]]);
    let device = u16::from_le_bytes([ids[2], ids[3]]);
    if manufacturer == MANUFACTURER_ID && device == DEVICE_ID {
        Ok(())
    } else {
        Err(Error::UnexpectedId(
            (u32::from(manufacturer) << 16) | u32::from(device),
        ))
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver at [`PRIMARY_ADDRESS`](crate::PRIMARY_ADDRESS);
        /// no bus traffic.
        pub const fn new(i2c: I2C, variant: Variant) -> Self {
            Self {
                i2c,
                address: PRIMARY_ADDRESS,
                variant,
                config: Config {
                    temperature: Resolution::Bits14,
                    humidity: Resolution::Bits14,
                },
            }
        }

        /// Use another I2C address.
        pub const fn with_address(mut self, address: u8) -> Self {
            self.address = address;
            self
        }

        /// Resolution used by `start_measurement` (written with the trigger).
        pub const fn with_config(mut self, config: Config) -> Self {
            self.config = config;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct Hdc2080<I2C> {
    i2c: I2C,
    address: u8,
    variant: Variant,
    config: Config,
}

impl<I2C> Hdc2080<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Hdc2080<I2C> {
    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(self.address, &[reg], buf)
            .map_err(Error::bus)
    }

    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, bytes).map_err(Error::bus)
    }

    /// Check manufacturer (0x5449) and device ID (0x07D0); mismatch returns
    /// [`Error::UnexpectedId`] with `manufacturer << 16 | device`.
    pub fn verify_id(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut ids = [0u8; 4];
        self.read(REG_MANUFACTURER_ID_LOW, &mut ids)?;
        check_ids(ids)
    }

    /// Soft reset (reloads defaults). Wait [`RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[REG_RESET_CONFIG, RESET_CONFIG_SOFT_RESET])?;
        Ok(RESET_WAIT_US)
    }

    /// Switch the heater (90 mA at 3.3 V; read-modify-write of 0x0E).
    pub fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
        let mut reg = [0u8; 1];
        self.read(REG_RESET_CONFIG, &mut reg)?;
        let value = if on {
            reg[0] | RESET_CONFIG_HEATER
        } else {
            reg[0] & !RESET_CONFIG_HEATER
        };
        self.write(&[REG_RESET_CONFIG, value])
    }

    /// Write the raw temperature (0x08) and humidity (0x09) offset
    /// registers; see [`temperature_offset_register`] and
    /// [`humidity_offset_register`].
    pub fn set_offsets(&mut self, temperature: u8, humidity: u8) -> Result<(), Error<I2C::Error>> {
        self.write(&[REG_TEMPERATURE_OFFSET, temperature, humidity])
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Hdc2080<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[REG_MEASUREMENT_CONFIG, self.config.trigger_register()])?;
        Ok(self.config.wait_us())
    }

    /// Returns [`Error::NotReady`] while MEAS_TRIG is still set.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut conf = [0u8; 1];
        self.read(REG_MEASUREMENT_CONFIG, &mut conf)?;
        if conf[0] & MEAS_TRIG != 0 {
            return Err(Error::NotReady);
        }
        let mut frame = [0u8; 4];
        self.read(REG_TEMPERATURE_LOW, &mut frame)?;
        Ok(decode_measurement(self.variant, &frame))
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        Config, MEAS_TRIG, Measurement, PRIMARY_ADDRESS, REG_MANUFACTURER_ID_LOW,
        REG_MEASUREMENT_CONFIG, REG_RESET_CONFIG, REG_TEMPERATURE_LOW, REG_TEMPERATURE_OFFSET,
        RESET_CONFIG_HEATER, RESET_CONFIG_SOFT_RESET, RESET_WAIT_US, Resolution, Variant,
        check_ids, decode_measurement,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Hdc2080`].
    pub struct Hdc2080<I2C> {
        i2c: I2C,
        address: u8,
        variant: Variant,
        config: Config,
    }

    impl<I2C> Hdc2080<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Hdc2080<I2C> {
        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write_read(self.address, &[reg], buf)
                .await
                .map_err(Error::bus)
        }

        async fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, bytes)
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Hdc2080::verify_id`].
        pub async fn verify_id(&mut self) -> Result<(), Error<I2C::Error>> {
            let mut ids = [0u8; 4];
            self.read(REG_MANUFACTURER_ID_LOW, &mut ids).await?;
            check_ids(ids)
        }

        /// See [`super::Hdc2080::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[REG_RESET_CONFIG, RESET_CONFIG_SOFT_RESET])
                .await?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Hdc2080::set_heater`].
        pub async fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
            let mut reg = [0u8; 1];
            self.read(REG_RESET_CONFIG, &mut reg).await?;
            let value = if on {
                reg[0] | RESET_CONFIG_HEATER
            } else {
                reg[0] & !RESET_CONFIG_HEATER
            };
            self.write(&[REG_RESET_CONFIG, value]).await
        }

        /// See [`super::Hdc2080::set_offsets`].
        pub async fn set_offsets(
            &mut self,
            temperature: u8,
            humidity: u8,
        ) -> Result<(), Error<I2C::Error>> {
            self.write(&[REG_TEMPERATURE_OFFSET, temperature, humidity])
                .await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Hdc2080<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[REG_MEASUREMENT_CONFIG, self.config.trigger_register()])
                .await?;
            Ok(self.config.wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut conf = [0u8; 1];
            self.read(REG_MEASUREMENT_CONFIG, &mut conf).await?;
            if conf[0] & MEAS_TRIG != 0 {
                return Err(Error::NotReady);
            }
            let mut frame = [0u8; 4];
            self.read(REG_TEMPERATURE_LOW, &mut frame).await?;
            Ok(decode_measurement(self.variant, &frame))
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
    fn conversion_per_variant() {
        // 0x6666 -> 0.4 × 165 = 66: 26.00 °C (−40) or 25.50 °C (−40.5).
        assert_eq!(convert(Variant::Hdc2010, 0x6666, 0x8000).temperature, 2_600);
        assert_eq!(convert(Variant::Hdc2080, 0x6666, 0x8000).temperature, 2_550);
        assert_eq!(convert(Variant::Hdc2021, 0, 0x8000).humidity, 5_000);
        assert_eq!(
            decode_measurement(Variant::Hdc2022, &[0x66, 0x66, 0x00, 0x80]),
            Measurement {
                temperature: 2_600,
                humidity: 5_000
            }
        );
    }

    #[test]
    fn offset_encoding_matches_datasheet_bit_weights() {
        // Table: bit0 = +0.16 °C, 0xFF = −0.16 °C, bit7 = −20.62 °C.
        assert_eq!(temperature_offset_centi(0x01), 16);
        assert_eq!(temperature_offset_centi(0xFF), -16);
        assert_eq!(temperature_offset_centi(0x80), -2_063);
        assert_eq!(temperature_offset_centi(0x40), 1_031);
        assert_eq!(temperature_offset_register(16), 0x01);
        assert_eq!(temperature_offset_register(-16), 0xFF);
        assert_eq!(temperature_offset_register(-5_000), 0x80);
        // Table: bit0 = +0.20 %RH, bit7 = −25 %RH, bit6 = +12.5 %RH.
        assert_eq!(humidity_offset_centi(0x01), 20);
        assert_eq!(humidity_offset_centi(0x80), -2_500);
        assert_eq!(humidity_offset_centi(0x40), 1_250);
        assert_eq!(humidity_offset_register(1_250), 0x40);
        assert_eq!(humidity_offset_register(-20), 0xFF);
    }

    #[test]
    fn trigger_and_read() {
        let config = Config {
            temperature: Resolution::Bits11,
            humidity: Resolution::Bits9,
        };
        let expectations = [
            Transaction::write(SECONDARY_ADDRESS, vec![0x0F, 0x61]),
            Transaction::write_read(SECONDARY_ADDRESS, vec![0x0F], vec![0x61]),
            Transaction::write_read(SECONDARY_ADDRESS, vec![0x0F], vec![0x60]),
            Transaction::write_read(SECONDARY_ADDRESS, vec![0x00], vec![0x66, 0x66, 0x00, 0x80]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hdc = Hdc2080::new(&mut i2c, Variant::Hdc2080)
            .with_address(SECONDARY_ADDRESS)
            .with_config(config);
        assert_eq!(hdc.start_measurement().unwrap(), (350 + 275) * 3 / 2);
        assert_eq!(hdc.read_measurement(), Err(Error::NotReady));
        assert_eq!(
            hdc.read_measurement().unwrap(),
            Measurement {
                temperature: 2_550,
                humidity: 5_000
            }
        );
        i2c.done();
    }

    #[test]
    fn id_heater_offsets() {
        let expectations = [
            Transaction::write_read(PRIMARY_ADDRESS, vec![0xFC], vec![0x49, 0x54, 0xD0, 0x07]),
            Transaction::write_read(PRIMARY_ADDRESS, vec![0xFC], vec![0x49, 0x54, 0x00, 0x10]),
            Transaction::write_read(PRIMARY_ADDRESS, vec![0x0E], vec![0x50]),
            Transaction::write(PRIMARY_ADDRESS, vec![0x0E, 0x58]),
            Transaction::write(PRIMARY_ADDRESS, vec![0x08, 0x01, 0xFF]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hdc = Hdc2080::new(&mut i2c, Variant::Hdc2010);
        hdc.verify_id().unwrap();
        assert_eq!(hdc.verify_id(), Err(Error::UnexpectedId(0x5449_1000)));
        hdc.set_heater(true).unwrap();
        hdc.set_offsets(0x01, 0xFF).unwrap();
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
            Transaction::write(PRIMARY_ADDRESS, vec![0x0F, 0x01]),
            Transaction::write_read(PRIMARY_ADDRESS, vec![0x0F], vec![0x00]),
            Transaction::write_read(PRIMARY_ADDRESS, vec![0x00], vec![0x66, 0x66, 0x00, 0x80]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hdc = asynch::Hdc2080::new(&mut i2c, Variant::Hdc2021);
        let mut delay = RecordingDelay(0);
        let m = block_on(hdc.measure(&mut delay)).unwrap();
        assert_eq!(m.temperature, 2_600);
        assert_eq!(delay.0, (610 + 660) * 3 / 2);
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
