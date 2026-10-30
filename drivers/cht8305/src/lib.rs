//! Sensylink CHT8305 / CHT8305C temperature and relative humidity sensor
//! (I2C).
//!
//! Writing the register pointer 0x00 triggers one combined temperature +
//! humidity conversion; after it the part returns to standby (0.05 µA).
//! While converting the part NACKs reads (or stretches the clock when
//! CLKSTR is set), so an early read shows up as [`Error::Bus`]. The result
//! is four bytes, temperature then humidity, MSB first, without CRC.
//!
//! Reference: Sensylink "CHT8305C Advanced Datasheet" Ver 2.5
//! (2021-10-26).
//!
//! ```ignore
//! let mut cht = Cht8305::new(i2c);
//! cht.verify_id()?;
//! let wait_us = cht.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = cht.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with AD0 to GND or floating.
pub const ADDRESS_AD0_GND: u8 = 0x40;
/// I2C address with AD0 to VCC.
pub const ADDRESS_AD0_VCC: u8 = 0x44;
/// I2C address with AD0 to SDA.
pub const ADDRESS_AD0_SDA: u8 = 0x48;
/// I2C address with AD0 to SCL.
pub const ADDRESS_AD0_SCL: u8 = 0x4C;
/// Manufacturer ID register value (shared with other Sensylink parts).
pub const MANUFACTURER_ID: u16 = 0x5959;

const REG_TEMPERATURE: u8 = 0x00;
const REG_CONFIG: u8 = 0x02;
const REG_MANUFACTURER_ID: u8 = 0xFE;
const REG_VERSION_ID: u8 = 0xFF;

const CONFIG_SOFT_RESET: u16 = 1 << 15;
const CONFIG_HEATER: u16 = 1 << 13;
const CONFIG_T_RES: u16 = 1 << 10;
const CONFIG_H_RES_SHIFT: u16 = 8;
const CONFIG_H_RES_MASK: u16 = 0b11 << CONFIG_H_RES_SHIFT;

/// Wait after the trigger. The datasheet gives 11 ms typical for 14-bit
/// T + RH and its reference code requires more than 20 ms.
pub const CONVERSION_WAIT_US: WaitUs = 21_000;

/// Temperature resolution (config T_RES).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TemperatureResolution {
    /// 14 bit (default).
    #[default]
    Bits14,
    /// 11 bit.
    Bits11,
}

/// Humidity resolution (config H_RES).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HumidityResolution {
    /// 14 bit (default).
    #[default]
    Bits14 = 0b00,
    /// 11 bit.
    Bits11 = 0b01,
    /// 8 bit.
    Bits8 = 0b10,
}

/// Raw temperature register → 0.01 °C: `165 · raw / 65535 − 40`.
pub const fn temperature_centi_celsius(raw: u16) -> i32 {
    ((raw as u32 * 16_500 + 32_767) / 65_535) as i32 - 4_000
}

/// Raw humidity register → 0.01 %RH: `100 · raw / 65535`.
pub const fn humidity_centi_percent(raw: u16) -> u16 {
    ((raw as u32 * 10_000 + 32_767) / 65_535) as u16
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

fn decode_frame(frame: &[u8; 4]) -> Measurement {
    Measurement {
        temperature: temperature_centi_celsius(u16::from_be_bytes([frame[0], frame[1]])),
        humidity: humidity_centi_percent(u16::from_be_bytes([frame[2], frame[3]])),
    }
}

fn check_manufacturer<E>(id: u16) -> Result<(), Error<E>> {
    if id == MANUFACTURER_ID {
        Ok(())
    } else {
        Err(Error::UnexpectedId(u32::from(id)))
    }
}

const fn with_resolution(
    config: u16,
    temperature: TemperatureResolution,
    humidity: HumidityResolution,
) -> u16 {
    let t = match temperature {
        TemperatureResolution::Bits14 => 0,
        TemperatureResolution::Bits11 => CONFIG_T_RES,
    };
    (config & !(CONFIG_T_RES | CONFIG_H_RES_MASK) & !CONFIG_SOFT_RESET)
        | t
        | ((humidity as u16) << CONFIG_H_RES_SHIFT)
}

const fn with_heater(config: u16, on: bool) -> u16 {
    let config = config & !CONFIG_SOFT_RESET;
    if on {
        config | CONFIG_HEATER
    } else {
        config & !CONFIG_HEATER
    }
}

const fn config_frame(config: u16) -> [u8; 3] {
    let [msb, lsb] = config.to_be_bytes();
    [REG_CONFIG, msb, lsb]
}

macro_rules! constructors {
    () => {
        /// Create a driver at [`ADDRESS_AD0_GND`](crate::ADDRESS_AD0_GND);
        /// no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                address: ADDRESS_AD0_GND,
            }
        }

        /// Use another AD0 address.
        pub const fn with_address(mut self, address: u8) -> Self {
            self.address = address;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct Cht8305<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Cht8305<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Cht8305<I2C> {
    fn read_register(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .map_err(Error::bus)?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Check the manufacturer ID (0x5959) and return the version ID
    /// (0x8XXX; exact value not documented). The manufacturer ID is shared
    /// with other Sensylink sensors.
    pub fn verify_id(&mut self) -> Result<u16, Error<I2C::Error>> {
        check_manufacturer(self.read_register(REG_MANUFACTURER_ID)?)?;
        self.read_register(REG_VERSION_ID)
    }

    /// Read the config register.
    pub fn config(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_register(REG_CONFIG)
    }

    fn write_config(&mut self, config: u16) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &config_frame(config))
            .map_err(Error::bus)
    }

    /// Set both resolutions (read-modify-write).
    pub fn set_resolution(
        &mut self,
        temperature: TemperatureResolution,
        humidity: HumidityResolution,
    ) -> Result<(), Error<I2C::Error>> {
        let config = self.config()?;
        self.write_config(with_resolution(config, temperature, humidity))
    }

    /// Switch the heater (7 mA peak).
    pub fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
        let config = self.config()?;
        self.write_config(with_heater(config, on))
    }

    /// Soft reset: registers return to defaults and calibration reloads.
    /// The datasheet gives no duration; SRST reads 0 in [`Cht8305::config`]
    /// once it is done.
    pub fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        let config = self.config()?;
        self.write_config(config | CONFIG_SOFT_RESET)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Cht8305<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Write the pointer 0x00 to start a conversion.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[REG_TEMPERATURE])
            .map_err(Error::bus)?;
        Ok(CONVERSION_WAIT_US)
    }

    /// Read T and RH; a read during conversion is NACKed ([`Error::Bus`]).
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut frame = [0u8; 4];
        self.i2c
            .read(self.address, &mut frame)
            .map_err(Error::bus)?;
        Ok(decode_frame(&frame))
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS_AD0_GND, CONFIG_SOFT_RESET, CONVERSION_WAIT_US, HumidityResolution, Measurement,
        REG_CONFIG, REG_MANUFACTURER_ID, REG_TEMPERATURE, REG_VERSION_ID, TemperatureResolution,
        check_manufacturer, config_frame, decode_frame, with_heater, with_resolution,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Cht8305`].
    pub struct Cht8305<I2C> {
        i2c: I2C,
        address: u8,
    }

    impl<I2C> Cht8305<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Cht8305<I2C> {
        async fn read_register(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
            let mut buf = [0u8; 2];
            self.i2c
                .write_read(self.address, &[reg], &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(u16::from_be_bytes(buf))
        }

        /// See [`super::Cht8305::verify_id`].
        pub async fn verify_id(&mut self) -> Result<u16, Error<I2C::Error>> {
            check_manufacturer(self.read_register(REG_MANUFACTURER_ID).await?)?;
            self.read_register(REG_VERSION_ID).await
        }

        /// See [`super::Cht8305::config`].
        pub async fn config(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.read_register(REG_CONFIG).await
        }

        async fn write_config(&mut self, config: u16) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &config_frame(config))
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Cht8305::set_resolution`].
        pub async fn set_resolution(
            &mut self,
            temperature: TemperatureResolution,
            humidity: HumidityResolution,
        ) -> Result<(), Error<I2C::Error>> {
            let config = self.config().await?;
            self.write_config(with_resolution(config, temperature, humidity))
                .await
        }

        /// See [`super::Cht8305::set_heater`].
        pub async fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
            let config = self.config().await?;
            self.write_config(with_heater(config, on)).await
        }

        /// See [`super::Cht8305::reset`].
        pub async fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
            let config = self.config().await?;
            self.write_config(config | CONFIG_SOFT_RESET).await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Cht8305<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[REG_TEMPERATURE])
                .await
                .map_err(Error::bus)?;
            Ok(CONVERSION_WAIT_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut frame = [0u8; 4];
            self.i2c
                .read(self.address, &mut frame)
                .await
                .map_err(Error::bus)?;
            Ok(decode_frame(&frame))
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

    // Datasheet section 1.1.4 worked example: 39680 → 59.9 °C,
    // 52224 → 79.7 %RH.
    const FRAME: [u8; 4] = [0x9B, 0x00, 0xCC, 0x00];
    const EXPECTED: Measurement = Measurement {
        temperature: 5_990,
        humidity: 7_969,
    };

    #[test]
    fn datasheet_formulas() {
        assert_eq!(decode_frame(&FRAME), EXPECTED);
        assert_eq!(temperature_centi_celsius(0), -4_000);
        assert_eq!(temperature_centi_celsius(u16::MAX), 12_500);
        assert_eq!(humidity_centi_percent(u16::MAX), 10_000);
        // 90 %RH / 80 °C threshold example: 0xE665 and 0xBA2D.
        assert_eq!(humidity_centi_percent(0xE665), 9_000);
        assert_eq!(temperature_centi_celsius(0xBA2D), 8_000);
    }

    #[test]
    fn measure_config_and_ids() {
        let expectations = [
            Transaction::write_read(0x48, vec![0xFE], vec![0x59, 0x59]),
            Transaction::write_read(0x48, vec![0xFF], vec![0x83, 0x05]),
            Transaction::write(0x48, vec![0x00]),
            Transaction::read(0x48, FRAME.to_vec()),
            Transaction::read(0x48, FRAME.to_vec()).with_error(embedded_hal::i2c::ErrorKind::Other),
            Transaction::write_read(0x48, vec![0x02], vec![0x10, 0x00]),
            Transaction::write(0x48, vec![0x02, 0x16, 0x00]),
            Transaction::write_read(0x48, vec![0x02], vec![0x16, 0x00]),
            Transaction::write(0x48, vec![0x02, 0x36, 0x00]),
            Transaction::write_read(0x48, vec![0x02], vec![0x36, 0x00]),
            Transaction::write(0x48, vec![0x02, 0xB6, 0x00]),
            Transaction::write_read(0x48, vec![0xFE], vec![0x54, 0x49]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut cht = Cht8305::new(&mut i2c).with_address(ADDRESS_AD0_SDA);
        assert_eq!(cht.verify_id(), Ok(0x8305));
        assert_eq!(cht.start_measurement(), Ok(CONVERSION_WAIT_US));
        assert_eq!(cht.read_measurement(), Ok(EXPECTED));
        assert!(matches!(cht.read_measurement(), Err(Error::Bus(_))));
        cht.set_resolution(TemperatureResolution::Bits11, HumidityResolution::Bits8)
            .unwrap();
        cht.set_heater(true).unwrap();
        cht.reset().unwrap();
        assert_eq!(cht.verify_id(), Err(Error::UnexpectedId(0x5449)));
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
            Transaction::write(ADDRESS_AD0_GND, vec![0x00]),
            Transaction::read(ADDRESS_AD0_GND, FRAME.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut cht = asynch::Cht8305::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(cht.measure(&mut delay)), Ok(EXPECTED));
        assert_eq!(delay.0, CONVERSION_WAIT_US);
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
