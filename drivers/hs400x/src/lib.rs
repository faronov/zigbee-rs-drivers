//! Renesas HS40xx (HS4001 / HS4003 / HS4011 / HS4013) temperature and
//! humidity sensors (I2C).
//!
//! The parts share one protocol; HS40x1 / HS40x3 differ in accuracy and
//! HS401x add a protective membrane. The driver uses the no-hold commands:
//! the sensor NACKs its address until the conversion is done (a read that
//! is too early returns [`Error::Bus`]) and returns to sleep after the
//! result has been read. Every result carries an 8-bit CRC (polynomial
//! 0x1D, init 0xFF) over the 4 data bytes; for temperature-only results
//! the humidity bytes are taken as zero.
//!
//! Reference: Renesas R36DS0034EU "HS40xx Datasheet" (rev. 1.4, Jun 14, 2024).
//!
//! ```ignore
//! let mut hs = Hs400x::new(i2c);
//! let wait_us = hs.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = hs.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::crc::crc8_msb;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const DEFAULT_ADDRESS: u8 = 0x54;

const CMD_MEASURE_T: u8 = 0xF3;
const CMD_MEASURE_RH_T: u8 = 0xF5;
const CMD_READ_REGISTER: u8 = 0xA7;
const CMD_WRITE_REGISTER: u8 = 0xA6;
const CMD_STOP_PERIODIC: u8 = 0x30;
const CMD_READ_ID: u8 = 0xD7;
const CMD_SOFT_RESET: u8 = 0xFE;
const REG_RESOLUTION: u8 = 0x00;
const CRC_POLYNOMIAL: u8 = 0x1D;
const CRC_INIT: u8 = 0xFF;

/// Wait after [`Hs400x::soft_reset`] before the next command.
pub const RESET_WAIT_US: WaitUs = 500_000;

/// Measurement resolution of one channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 8 bit.
    Bits8 = 0,
    /// 10 bit.
    Bits10 = 1,
    /// 12 bit.
    Bits12 = 2,
    /// 14 bit (default).
    #[default]
    Bits14 = 3,
}

impl Resolution {
    /// Typical temperature-only conversion time.
    pub const fn temperature_us(self) -> WaitUs {
        [370, 450, 600, 910][self as usize]
    }

    /// Typical humidity + temperature conversion time (same resolution for
    /// both channels).
    pub const fn humidity_temperature_us(self) -> WaitUs {
        [640, 800, 1_040, 1_700][self as usize]
    }

    const fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            0 => Self::Bits8,
            1 => Self::Bits10,
            2 => Self::Bits12,
            _ => Self::Bits14,
        }
    }
}

/// Measurement resolutions (register 0x00: T bits 1:0, RH bits 3:2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Humidity resolution.
    pub humidity: Resolution,
    /// Temperature resolution.
    pub temperature: Resolution,
}

impl Config {
    /// Wait for an RH + T conversion: the temperature-only time plus the
    /// humidity share of the RH + T table. The datasheet gives typical
    /// values only, so 50 % margin is added.
    pub const fn wait_us(self) -> WaitUs {
        let humidity = self.humidity.humidity_temperature_us() - self.humidity.temperature_us();
        (self.temperature.temperature_us() + humidity) * 3 / 2
    }

    /// Wait for a temperature-only conversion (typical + 50 %).
    pub const fn temperature_wait_us(self) -> WaitUs {
        self.temperature.temperature_us() * 3 / 2
    }

    const fn apply(self, register: u8) -> u8 {
        (register & !0x0F) | ((self.humidity as u8) << 2) | self.temperature as u8
    }

    const fn from_register(register: u8) -> Self {
        Self {
            humidity: Resolution::from_bits(register >> 2),
            temperature: Resolution::from_bits(register),
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

/// CRC-8 used by the HS40xx (polynomial 0x1D, init 0xFF).
pub fn crc8(bytes: &[u8]) -> u8 {
    crc8_msb(bytes, CRC_POLYNOMIAL, CRC_INIT)
}

/// Raw 14-bit humidity to 0.01 %RH: raw / (2^14 − 1) × 100.
pub fn convert_humidity(raw: u16) -> u16 {
    ((u32::from(raw & 0x3FFF) * 10_000 + 8_191) / 16_383) as u16
}

/// Raw 14-bit temperature to 0.01 °C: raw / (2^14 − 1) × 165 − 40.
pub fn convert_temperature(raw: u16) -> i32 {
    ((u32::from(raw & 0x3FFF) * 16_500 + 8_191) / 16_383) as i32 - 4_000
}

/// Decode an RH + T frame (RH MSB, LSB, T MSB, LSB, CRC).
pub fn decode_frame<E>(frame: &[u8; 5]) -> Result<Measurement, Error<E>> {
    if crc8(&frame[..4]) != frame[4] {
        return Err(Error::Crc);
    }
    Ok(Measurement {
        temperature: convert_temperature(u16::from_be_bytes([frame[2], frame[3]])),
        humidity: convert_humidity(u16::from_be_bytes([frame[0], frame[1]])),
    })
}

/// Decode a temperature-only frame (T MSB, LSB, CRC) to 0.01 °C.
pub fn decode_temperature<E>(frame: &[u8; 3]) -> Result<i32, Error<E>> {
    if crc8(&[0, 0, frame[0], frame[1]]) != frame[2] {
        return Err(Error::Crc);
    }
    Ok(convert_temperature(u16::from_be_bytes([
        frame[0], frame[1],
    ])))
}

macro_rules! constructors {
    () => {
        /// Create a driver at [`DEFAULT_ADDRESS`](crate::DEFAULT_ADDRESS)
        /// assuming the default 14-bit resolution; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                address: DEFAULT_ADDRESS,
                config: Config {
                    humidity: Resolution::Bits14,
                    temperature: Resolution::Bits14,
                },
            }
        }

        /// Use another I2C address.
        pub const fn with_address(mut self, address: u8) -> Self {
            self.address = address;
            self
        }

        /// Resolutions assumed for the conversion wait (written to the
        /// sensor by `set_config`).
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
pub struct Hs400x<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
}

impl<I2C> Hs400x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Hs400x<I2C> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, bytes).map_err(Error::bus)
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.read(self.address, buf).map_err(Error::bus)
    }

    /// Read a configuration register.
    pub fn read_register(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut value = [0u8; 1];
        self.i2c
            .write_read(self.address, &[CMD_READ_REGISTER, reg], &mut value)
            .map_err(Error::bus)?;
        Ok(value[0])
    }

    /// Write a configuration register (volatile; lost on power-off).
    pub fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.write(&[CMD_WRITE_REGISTER, reg, value])
    }

    /// Read the resolution register.
    pub fn config(&mut self) -> Result<Config, Error<I2C::Error>> {
        Ok(Config::from_register(self.read_register(REG_RESOLUTION)?))
    }

    /// Set both resolutions (read-modify-write of register 0x00) and use
    /// them for the conversion wait.
    pub fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(REG_RESOLUTION)?;
        self.write_register(REG_RESOLUTION, config.apply(current))?;
        self.config = config;
        Ok(())
    }

    /// Read the unique 32-bit sensor ID.
    pub fn sensor_id(&mut self) -> Result<u32, Error<I2C::Error>> {
        let mut id = [0u8; 4];
        self.i2c
            .write_read(self.address, &[CMD_READ_ID], &mut id)
            .map_err(Error::bus)?;
        Ok(u32::from_be_bytes(id))
    }

    /// Leave periodic measurement mode.
    pub fn stop_periodic_measurements(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write(&[CMD_STOP_PERIODIC])
    }

    /// Soft reset (registers back to defaults). Wait [`RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[CMD_SOFT_RESET])?;
        Ok(RESET_WAIT_US)
    }

    /// Start a temperature-only conversion.
    pub fn start_temperature(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[CMD_MEASURE_T])?;
        Ok(self.config.temperature_wait_us())
    }

    /// Read a temperature-only result in 0.01 °C.
    pub fn read_temperature(&mut self) -> Result<i32, Error<I2C::Error>> {
        let mut frame = [0u8; 3];
        self.read(&mut frame)?;
        decode_temperature(&frame)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Hs400x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// No-hold RH + T command (0xF5).
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[CMD_MEASURE_RH_T])?;
        Ok(self.config.wait_us())
    }

    /// The sensor NACKs ([`Error::Bus`]) while the conversion is running.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut frame = [0u8; 5];
        self.read(&mut frame)?;
        decode_frame(&frame)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_MEASURE_RH_T, CMD_MEASURE_T, CMD_READ_ID, CMD_READ_REGISTER, CMD_SOFT_RESET,
        CMD_STOP_PERIODIC, CMD_WRITE_REGISTER, Config, DEFAULT_ADDRESS, Measurement,
        REG_RESOLUTION, RESET_WAIT_US, Resolution, decode_frame, decode_temperature,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Hs400x`].
    pub struct Hs400x<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
    }

    impl<I2C> Hs400x<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Hs400x<I2C> {
        async fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, bytes)
                .await
                .map_err(Error::bus)
        }

        async fn read(&mut self, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c.read(self.address, buf).await.map_err(Error::bus)
        }

        /// See [`super::Hs400x::read_register`].
        pub async fn read_register(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
            let mut value = [0u8; 1];
            self.i2c
                .write_read(self.address, &[CMD_READ_REGISTER, reg], &mut value)
                .await
                .map_err(Error::bus)?;
            Ok(value[0])
        }

        /// See [`super::Hs400x::write_register`].
        pub async fn write_register(
            &mut self,
            reg: u8,
            value: u8,
        ) -> Result<(), Error<I2C::Error>> {
            self.write(&[CMD_WRITE_REGISTER, reg, value]).await
        }

        /// See [`super::Hs400x::config`].
        pub async fn config(&mut self) -> Result<Config, Error<I2C::Error>> {
            Ok(Config::from_register(
                self.read_register(REG_RESOLUTION).await?,
            ))
        }

        /// See [`super::Hs400x::set_config`].
        pub async fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
            let current = self.read_register(REG_RESOLUTION).await?;
            self.write_register(REG_RESOLUTION, config.apply(current))
                .await?;
            self.config = config;
            Ok(())
        }

        /// See [`super::Hs400x::sensor_id`].
        pub async fn sensor_id(&mut self) -> Result<u32, Error<I2C::Error>> {
            let mut id = [0u8; 4];
            self.i2c
                .write_read(self.address, &[CMD_READ_ID], &mut id)
                .await
                .map_err(Error::bus)?;
            Ok(u32::from_be_bytes(id))
        }

        /// See [`super::Hs400x::stop_periodic_measurements`].
        pub async fn stop_periodic_measurements(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write(&[CMD_STOP_PERIODIC]).await
        }

        /// See [`super::Hs400x::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[CMD_SOFT_RESET]).await?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Hs400x::start_temperature`].
        pub async fn start_temperature(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[CMD_MEASURE_T]).await?;
            Ok(self.config.temperature_wait_us())
        }

        /// See [`super::Hs400x::read_temperature`].
        pub async fn read_temperature(&mut self) -> Result<i32, Error<I2C::Error>> {
            let mut frame = [0u8; 3];
            self.read(&mut frame).await?;
            decode_temperature(&frame)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Hs400x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[CMD_MEASURE_RH_T]).await?;
            Ok(self.config.wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut frame = [0u8; 5];
            self.read(&mut frame).await?;
            decode_frame(&frame)
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

    // Datasheet Table 9 CRC example: CRC(0x05800580) = 0xF2.
    const FRAME: [u8; 5] = [0x05, 0x80, 0x05, 0x80, 0xF2];
    const EXPECTED: Measurement = Measurement {
        temperature: -2_582,
        humidity: 859,
    };

    #[test]
    fn crc_and_equations() {
        assert_eq!(crc8(&[0x05, 0x80, 0x05, 0x80]), 0xF2);
        assert_eq!(decode_frame::<()>(&FRAME), Ok(EXPECTED));
        let mut bad = FRAME;
        bad[4] ^= 1;
        assert_eq!(decode_frame::<()>(&bad), Err(Error::Crc));
        // Full scale and zero: 100 %RH, 125 °C, −40 °C.
        assert_eq!(convert_humidity(0x3FFF), 10_000);
        assert_eq!(convert_temperature(0x3FFF), 12_500);
        assert_eq!(convert_temperature(0), -4_000);
        // Temperature-only: humidity bytes count as zero for the CRC.
        assert_eq!(decode_temperature::<()>(&[0x1E, 0x5D, 0xA2]), Ok(3_829));
        assert_eq!(
            decode_temperature::<()>(&[0x1E, 0x5D, 0xE8]),
            Err(Error::Crc)
        );
        // Table 8: equal resolutions give the RH + T column.
        assert_eq!(Config::default().wait_us(), 1_700 * 3 / 2);
        let low = Config {
            humidity: Resolution::Bits8,
            temperature: Resolution::Bits8,
        };
        assert_eq!(low.wait_us(), 640 * 3 / 2);
        assert_eq!(low.temperature_wait_us(), 370 * 3 / 2);
    }

    #[test]
    fn no_hold_measurement_with_nack() {
        let expectations = [
            Transaction::write(DEFAULT_ADDRESS, vec![0xF5]),
            Transaction::read(DEFAULT_ADDRESS, vec![0; 5])
                .with_error(embedded_hal::i2c::ErrorKind::Other),
            Transaction::read(DEFAULT_ADDRESS, FRAME.to_vec()),
            Transaction::write(DEFAULT_ADDRESS, vec![0xF3]),
            Transaction::read(DEFAULT_ADDRESS, vec![0x1E, 0x5D, 0xA2]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hs = Hs400x::new(&mut i2c);
        assert_eq!(hs.start_measurement().unwrap(), 2_550);
        assert!(matches!(hs.read_measurement(), Err(Error::Bus(_))));
        assert_eq!(hs.read_measurement(), Ok(EXPECTED));
        assert_eq!(hs.start_temperature().unwrap(), 910 * 3 / 2);
        assert_eq!(hs.read_temperature(), Ok(3_829));
        i2c.done();
    }

    #[test]
    fn registers_id_reset() {
        let config = Config {
            humidity: Resolution::Bits10,
            temperature: Resolution::Bits12,
        };
        let expectations = [
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xA7, 0x00], vec![0xAF]),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xA7, 0x00], vec![0xAF]),
            Transaction::write(DEFAULT_ADDRESS, vec![0xA6, 0x00, 0xA6]),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xD7], vec![0x12, 0x34, 0x56, 0x78]),
            Transaction::write(DEFAULT_ADDRESS, vec![0x30]),
            Transaction::write(DEFAULT_ADDRESS, vec![0xFE]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hs = Hs400x::new(&mut i2c);
        assert_eq!(hs.config(), Ok(Config::default()));
        hs.set_config(config).unwrap();
        assert_eq!(hs.sensor_id(), Ok(0x1234_5678));
        hs.stop_periodic_measurements().unwrap();
        assert_eq!(hs.soft_reset(), Ok(RESET_WAIT_US));
        // T 12 bit (600) + RH 10 bit share (800 − 450).
        assert_eq!(hs.start_measurement_wait(), (600 + 350) * 3 / 2);
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
            Transaction::write(DEFAULT_ADDRESS, vec![0xF5]),
            Transaction::read(DEFAULT_ADDRESS, FRAME.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hs = asynch::Hs400x::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(hs.measure(&mut delay)), Ok(EXPECTED));
        assert_eq!(delay.0, 2_550);
        i2c.done();
    }

    impl<I2C> Hs400x<I2C> {
        fn start_measurement_wait(&self) -> WaitUs {
            self.config.wait_us()
        }
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
