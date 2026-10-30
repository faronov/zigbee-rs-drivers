//! Renesas HS3001 / HS3002 / HS3003 / HS3004 temperature and humidity
//! sensors (I2C).
//!
//! The four parts share one protocol and differ only in accuracy grade.
//! The sensor sleeps between measurements: a Measurement Request (MR, an
//! address-only write) wakes it for one RH + T conversion, and a Data
//! Fetch (DF, a 4-byte read) returns the result. The two MSBs of the first
//! byte are status bits; `01` (stale) means the conversion is not finished
//! or the data was already fetched and is reported as [`Error::NotReady`].
//!
//! The resolution is stored in non-volatile memory and can only be changed
//! in programming mode, which must be entered within 10 ms of power-up
//! (see [`Hs300x::enter_programming_mode`]).
//!
//! Reference: Renesas "HS300x Datasheet" (rev. April 9, 2021). The
//! programming-mode response status byte (0x81) is taken from the Renesas
//! FSP `rm_hs300x` driver.
//!
//! ```ignore
//! let mut hs = Hs300x::new(i2c);
//! let wait_us = hs.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = hs.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Default (factory) I2C address.
pub const DEFAULT_ADDRESS: u8 = 0x44;

const CMD_PROGRAMMING_ENTER: [u8; 3] = [0xA0, 0x00, 0x00];
const CMD_PROGRAMMING_EXIT: [u8; 3] = [0x80, 0x00, 0x00];
const REG_HUMIDITY_RESOLUTION_READ: u8 = 0x06;
const REG_TEMPERATURE_RESOLUTION_READ: u8 = 0x11;
const REG_WRITE_OFFSET: u8 = 0x40;
const REG_ID_UPPER: u8 = 0x1E;
const REG_ID_LOWER: u8 = 0x1F;
const PROGRAMMING_SUCCESS: u8 = 0x81;
const RESOLUTION_SHIFT: u16 = 10;
const RESOLUTION_MASK: u16 = 0b11 << RESOLUTION_SHIFT;

/// Wake-up time added to every measurement.
pub const WAKE_UP_US: WaitUs = 100;
/// Time for the non-volatile memory to load a register (programming mode).
pub const REGISTER_LOAD_US: WaitUs = 120;
/// Time for the non-volatile memory to store a register (programming mode).
pub const REGISTER_WRITE_US: WaitUs = 14_000;

/// Measurement resolution of one channel (register bits 11:10).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 8 bit.
    Bits8 = 0,
    /// 10 bit.
    Bits10 = 1,
    /// 12 bit.
    Bits12 = 2,
    /// 14 bit (factory default).
    #[default]
    Bits14 = 3,
}

impl Resolution {
    /// Typical conversion time of one channel.
    pub const fn conversion_us(self) -> WaitUs {
        [550, 1_310, 4_500, 16_900][self as usize]
    }

    const fn from_register(value: u16) -> Self {
        match (value & RESOLUTION_MASK) >> RESOLUTION_SHIFT {
            0 => Self::Bits8,
            1 => Self::Bits10,
            2 => Self::Bits12,
            _ => Self::Bits14,
        }
    }
}

/// Channel selector for the resolution registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    /// Relative humidity.
    Humidity,
    /// Temperature.
    Temperature,
}

impl Channel {
    const fn read_register(self) -> u8 {
        match self {
            Self::Humidity => REG_HUMIDITY_RESOLUTION_READ,
            Self::Temperature => REG_TEMPERATURE_RESOLUTION_READ,
        }
    }
}

/// Resolutions the sensor is programmed with; only used for timing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Humidity resolution.
    pub humidity: Resolution,
    /// Temperature resolution.
    pub temperature: Resolution,
}

impl Config {
    /// Wait after a Measurement Request: wake-up + both conversions. The
    /// datasheet gives typical values only, so 25 % margin is added.
    pub const fn wait_us(self) -> WaitUs {
        (WAKE_UP_US + self.humidity.conversion_us() + self.temperature.conversion_us()) * 5 / 4
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

/// 14-bit raw values to engineering units:
/// RH = raw / (2^14 − 1) × 100, T = raw / (2^14 − 1) × 165 − 40.
pub fn convert(raw_humidity: u16, raw_temperature: u16) -> Measurement {
    let h = u32::from(raw_humidity & 0x3FFF);
    let t = u32::from(raw_temperature & 0x3FFF);
    Measurement {
        temperature: ((t * 16_500 + 8_191) / 16_383) as i32 - 4_000,
        humidity: ((h * 10_000 + 8_191) / 16_383) as u16,
    }
}

/// Decode a 4-byte Data Fetch frame. Stale status → [`Error::NotReady`];
/// reserved status values → [`Error::InvalidData`].
pub fn decode_frame<E>(frame: &[u8; 4]) -> Result<Measurement, Error<E>> {
    match frame[0] >> 6 {
        0b00 => {}
        0b01 => return Err(Error::NotReady),
        _ => return Err(Error::InvalidData),
    }
    let humidity = u16::from_be_bytes([frame[0], frame[1]]) & 0x3FFF;
    let temperature = u16::from_be_bytes([frame[2], frame[3]]) >> 2;
    Ok(convert(humidity, temperature))
}

fn check_register<E>(answer: &[u8; 3]) -> Result<u16, Error<E>> {
    if answer[0] == PROGRAMMING_SUCCESS {
        Ok(u16::from_be_bytes([answer[1], answer[2]]))
    } else {
        Err(Error::InvalidData)
    }
}

const fn with_resolution(value: u16, resolution: Resolution) -> u16 {
    (value & !RESOLUTION_MASK) | ((resolution as u16) << RESOLUTION_SHIFT)
}

macro_rules! constructors {
    () => {
        /// Create a driver at [`DEFAULT_ADDRESS`](crate::DEFAULT_ADDRESS)
        /// assuming the factory 14-bit resolution; no bus traffic.
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

        /// Use another (custom-ordered) I2C address.
        pub const fn with_address(mut self, address: u8) -> Self {
            self.address = address;
            self
        }

        /// Resolutions stored in the sensor, used for the measurement wait.
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
pub struct Hs300x<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
}

impl<I2C> Hs300x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Hs300x<I2C> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, bytes).map_err(Error::bus)
    }

    fn read_register<D: embedded_hal::delay::DelayNs>(
        &mut self,
        reg: u8,
        delay: &mut D,
    ) -> Result<u16, Error<I2C::Error>> {
        self.write(&[reg, 0x00, 0x00])?;
        delay.delay_us(REGISTER_LOAD_US);
        let mut answer = [0u8; 3];
        self.i2c
            .read(self.address, &mut answer)
            .map_err(Error::bus)?;
        check_register(&answer)
    }

    /// Enter programming mode. Must be sent within 10 ms after power-up.
    pub fn enter_programming_mode(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write(&CMD_PROGRAMMING_ENTER)
    }

    /// Leave programming mode and return to normal measurements.
    pub fn exit_programming_mode(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write(&CMD_PROGRAMMING_EXIT)
    }

    /// Read the stored resolution of `channel` (programming mode only).
    pub fn resolution<D: embedded_hal::delay::DelayNs>(
        &mut self,
        channel: Channel,
        delay: &mut D,
    ) -> Result<Resolution, Error<I2C::Error>> {
        let value = self.read_register(channel.read_register(), delay)?;
        Ok(Resolution::from_register(value))
    }

    /// Store a new resolution for `channel` in non-volatile memory
    /// (programming mode only; blocks ~14 ms on `delay`). Also updates the
    /// measurement wait.
    pub fn set_resolution<D: embedded_hal::delay::DelayNs>(
        &mut self,
        channel: Channel,
        resolution: Resolution,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        let reg = channel.read_register();
        let value = with_resolution(self.read_register(reg, delay)?, resolution);
        let [msb, lsb] = value.to_be_bytes();
        self.write(&[reg | REG_WRITE_OFFSET, msb, lsb])?;
        delay.delay_us(REGISTER_WRITE_US);
        match channel {
            Channel::Humidity => self.config.humidity = resolution,
            Channel::Temperature => self.config.temperature = resolution,
        }
        Ok(())
    }

    /// Read the 32-bit sensor ID (programming mode only).
    pub fn sensor_id<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<u32, Error<I2C::Error>> {
        let upper = self.read_register(REG_ID_UPPER, delay)?;
        let lower = self.read_register(REG_ID_LOWER, delay)?;
        Ok((u32::from(upper) << 16) | u32::from(lower))
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Hs300x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Measurement Request (address-only write).
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[])?;
        Ok(self.config.wait_us())
    }

    /// Data Fetch; [`Error::NotReady`] while the data is stale.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut frame = [0u8; 4];
        self.i2c
            .read(self.address, &mut frame)
            .map_err(Error::bus)?;
        decode_frame(&frame)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_PROGRAMMING_ENTER, CMD_PROGRAMMING_EXIT, Channel, Config, DEFAULT_ADDRESS, Measurement,
        REG_ID_LOWER, REG_ID_UPPER, REG_WRITE_OFFSET, REGISTER_LOAD_US, REGISTER_WRITE_US,
        Resolution, check_register, decode_frame, with_resolution,
    };
    use embedded_hal_async::delay::DelayNs;
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Hs300x`].
    pub struct Hs300x<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
    }

    impl<I2C> Hs300x<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Hs300x<I2C> {
        async fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, bytes)
                .await
                .map_err(Error::bus)
        }

        async fn read_register<D: DelayNs>(
            &mut self,
            reg: u8,
            delay: &mut D,
        ) -> Result<u16, Error<I2C::Error>> {
            self.write(&[reg, 0x00, 0x00]).await?;
            delay.delay_us(REGISTER_LOAD_US).await;
            let mut answer = [0u8; 3];
            self.i2c
                .read(self.address, &mut answer)
                .await
                .map_err(Error::bus)?;
            check_register(&answer)
        }

        /// See [`super::Hs300x::enter_programming_mode`].
        pub async fn enter_programming_mode(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write(&CMD_PROGRAMMING_ENTER).await
        }

        /// See [`super::Hs300x::exit_programming_mode`].
        pub async fn exit_programming_mode(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write(&CMD_PROGRAMMING_EXIT).await
        }

        /// See [`super::Hs300x::resolution`].
        pub async fn resolution<D: DelayNs>(
            &mut self,
            channel: Channel,
            delay: &mut D,
        ) -> Result<Resolution, Error<I2C::Error>> {
            let value = self.read_register(channel.read_register(), delay).await?;
            Ok(Resolution::from_register(value))
        }

        /// See [`super::Hs300x::set_resolution`].
        pub async fn set_resolution<D: DelayNs>(
            &mut self,
            channel: Channel,
            resolution: Resolution,
            delay: &mut D,
        ) -> Result<(), Error<I2C::Error>> {
            let reg = channel.read_register();
            let value = with_resolution(self.read_register(reg, delay).await?, resolution);
            let [msb, lsb] = value.to_be_bytes();
            self.write(&[reg | REG_WRITE_OFFSET, msb, lsb]).await?;
            delay.delay_us(REGISTER_WRITE_US).await;
            match channel {
                Channel::Humidity => self.config.humidity = resolution,
                Channel::Temperature => self.config.temperature = resolution,
            }
            Ok(())
        }

        /// See [`super::Hs300x::sensor_id`].
        pub async fn sensor_id<D: DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<u32, Error<I2C::Error>> {
            let upper = self.read_register(REG_ID_UPPER, delay).await?;
            let lower = self.read_register(REG_ID_LOWER, delay).await?;
            Ok((u32::from(upper) << 16) | u32::from(lower))
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Hs300x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[]).await?;
            Ok(self.config.wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut frame = [0u8; 4];
            self.i2c
                .read(self.address, &mut frame)
                .await
                .map_err(Error::bus)?;
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

    // Status 00, RH raw 0x2000, T raw 0x1E5D (left-aligned 0x7974).
    const FRAME: [u8; 4] = [0x20, 0x00, 0x79, 0x74];
    const EXPECTED: Measurement = Measurement {
        temperature: 3_829,
        humidity: 5_000,
    };

    struct CountingDelay(u32);
    impl embedded_hal::delay::DelayNs for CountingDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.0 += ns / 1_000;
        }
    }

    #[test]
    fn datasheet_equations() {
        // Full scale: 100 %RH and 165 − 40 = 125 °C; zero: −40 °C.
        assert_eq!(
            convert(0x3FFF, 0x3FFF),
            Measurement {
                temperature: 12_500,
                humidity: 10_000
            }
        );
        assert_eq!(convert(0, 0).temperature, -4_000);
        // 8192 / 16383 × 165 − 40 = 42.505 °C.
        assert_eq!(convert(0x2000, 0x2000).temperature, 4_251);
        assert_eq!(decode_frame::<()>(&FRAME), Ok(EXPECTED));
        // Status bits are masked off the humidity; low 2 T bits ignored.
        assert_eq!(
            decode_frame::<()>(&[0x60, 0x00, 0x79, 0x77]),
            Err(Error::NotReady)
        );
        assert_eq!(
            decode_frame::<()>(&[0x80, 0x00, 0x79, 0x74]),
            Err(Error::InvalidData)
        );
        assert_eq!(decode_frame::<()>(&[0x20, 0x00, 0x79, 0x77]), Ok(EXPECTED));
        // Datasheet: 8-bit RH + 8-bit T = 1.20 ms incl. wake-up; 14+14 = 33.9 ms.
        let fast = Config {
            humidity: Resolution::Bits8,
            temperature: Resolution::Bits8,
        };
        assert_eq!(fast.wait_us(), 1_200 * 5 / 4);
        assert_eq!(Config::default().wait_us(), 33_900 * 5 / 4);
    }

    #[test]
    fn measurement_request_and_stale_fetch() {
        let expectations = [
            Transaction::write(DEFAULT_ADDRESS, vec![]),
            Transaction::read(DEFAULT_ADDRESS, vec![0x60, 0x00, 0x79, 0x74]),
            Transaction::read(DEFAULT_ADDRESS, FRAME.to_vec()),
            Transaction::read(DEFAULT_ADDRESS, FRAME.to_vec())
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hs = Hs300x::new(&mut i2c);
        assert_eq!(hs.start_measurement().unwrap(), 33_900 * 5 / 4);
        assert_eq!(hs.read_measurement(), Err(Error::NotReady));
        assert_eq!(hs.read_measurement(), Ok(EXPECTED));
        assert!(matches!(hs.read_measurement(), Err(Error::Bus(_))));
        i2c.done();
    }

    #[test]
    fn programming_mode_resolution_and_id() {
        let expectations = [
            Transaction::write(DEFAULT_ADDRESS, vec![0xA0, 0x00, 0x00]),
            // Change temperature resolution 14 -> 10 bit, other bits kept.
            Transaction::write(DEFAULT_ADDRESS, vec![0x11, 0x00, 0x00]),
            Transaction::read(DEFAULT_ADDRESS, vec![0x81, 0x5C, 0x21]),
            Transaction::write(DEFAULT_ADDRESS, vec![0x51, 0x54, 0x21]),
            Transaction::write(DEFAULT_ADDRESS, vec![0x06, 0x00, 0x00]),
            Transaction::read(DEFAULT_ADDRESS, vec![0x81, 0x08, 0x00]),
            Transaction::write(DEFAULT_ADDRESS, vec![0x1E, 0x00, 0x00]),
            Transaction::read(DEFAULT_ADDRESS, vec![0x81, 0x12, 0x34]),
            Transaction::write(DEFAULT_ADDRESS, vec![0x1F, 0x00, 0x00]),
            Transaction::read(DEFAULT_ADDRESS, vec![0x81, 0x56, 0x78]),
            // Not in programming mode: status byte is not 0x81.
            Transaction::write(DEFAULT_ADDRESS, vec![0x1E, 0x00, 0x00]),
            Transaction::read(DEFAULT_ADDRESS, vec![0x00, 0x00, 0x00]),
            Transaction::write(DEFAULT_ADDRESS, vec![0x80, 0x00, 0x00]),
            Transaction::write(DEFAULT_ADDRESS, vec![]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hs = Hs300x::new(&mut i2c);
        let mut delay = CountingDelay(0);
        hs.enter_programming_mode().unwrap();
        hs.set_resolution(Channel::Temperature, Resolution::Bits10, &mut delay)
            .unwrap();
        assert_eq!(delay.0, REGISTER_LOAD_US + REGISTER_WRITE_US);
        assert_eq!(
            hs.resolution(Channel::Humidity, &mut delay),
            Ok(Resolution::Bits12)
        );
        assert_eq!(hs.sensor_id(&mut delay), Ok(0x1234_5678));
        assert_eq!(hs.sensor_id(&mut delay), Err(Error::InvalidData));
        hs.exit_programming_mode().unwrap();
        assert_eq!(
            hs.start_measurement().unwrap(),
            (100 + 16_900 + 1_310) * 5 / 4
        );
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
            Transaction::write(0x45, vec![]),
            Transaction::read(0x45, FRAME.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let config = Config {
            humidity: Resolution::Bits12,
            temperature: Resolution::Bits12,
        };
        let mut hs = asynch::Hs300x::new(&mut i2c)
            .with_address(0x45)
            .with_config(config);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(hs.measure(&mut delay)), Ok(EXPECTED));
        // Datasheet: 12-bit RH + T = 9.10 ms incl. wake-up.
        assert_eq!(delay.0, 9_100 * 5 / 4);
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
