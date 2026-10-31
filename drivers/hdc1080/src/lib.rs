//! Texas Instruments HDC1010 / HDC1080 temperature and humidity sensors (I2C).
//!
//! The driver runs the parts in sequential mode (configuration MODE = 1):
//! a pointer write to 0x00 triggers a temperature conversion followed by a
//! humidity conversion, and a 4-byte read returns both results. The part
//! NACKs the read while it is still converting and goes back to sleep
//! (100 nA) on its own afterwards.
//!
//! References: TI "HDC1080 Low Power, High Accuracy Digital Humidity Sensor
//! with Temperature Sensor", SNAS672A (rev. January 2016); TI "HDC1010",
//! SNAS685A (rev. August 2016).
//!
//! ```ignore
//! let mut hdc = Hdc1080::new(i2c, Variant::Hdc1080);
//! hdc.configure(Config::default())?;
//! let wait_us = hdc.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = hdc.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// HDC1080 address, HDC1010 with ADR1 = ADR0 = 0 (HDC1010: 0x40..=0x43).
pub const DEFAULT_ADDRESS: u8 = 0x40;

const REG_TEMPERATURE: u8 = 0x00;
const REG_CONFIG: u8 = 0x02;
const REG_SERIAL: [u8; 3] = [0xFB, 0xFC, 0xFD];
const REG_MANUFACTURER_ID: u8 = 0xFE;
const REG_DEVICE_ID: u8 = 0xFF;

/// Texas Instruments manufacturer ID (register 0xFE).
pub const MANUFACTURER_ID: u16 = 0x5449;
/// Start-up time from power-on (and used after a soft reset), 15 ms max.
pub const STARTUP_WAIT_US: WaitUs = 15_000;

const CONFIG_RESET: u16 = 1 << 15;
const CONFIG_HEATER: u16 = 1 << 13;
const CONFIG_SEQUENTIAL: u16 = 1 << 12;
const CONFIG_BATTERY_LOW: u16 = 1 << 11;

/// Supported parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// HDC1010 (WLCSP, four addresses).
    Hdc1010,
    /// HDC1080 (PWSON, fixed address).
    Hdc1080,
}

impl Variant {
    /// Value of the device ID register (0xFF).
    pub const fn device_id(self) -> u16 {
        match self {
            Self::Hdc1010 => 0x1000,
            Self::Hdc1080 => 0x1050,
        }
    }
}

/// Temperature resolution (TRES).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TemperatureResolution {
    /// 14 bit, 6.35 ms.
    #[default]
    Bits14,
    /// 11 bit, 3.65 ms.
    Bits11,
}

/// Humidity resolution (HRES).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HumidityResolution {
    /// 14 bit, 6.50 ms.
    #[default]
    Bits14,
    /// 11 bit, 3.85 ms.
    Bits11,
    /// 8 bit, 2.50 ms.
    Bits8,
}

/// Measurement configuration written by `configure`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Temperature resolution.
    pub temperature: TemperatureResolution,
    /// Humidity resolution.
    pub humidity: HumidityResolution,
    /// Integrated heater (only on while converting; 7.2 mA peak).
    pub heater: bool,
}

impl Config {
    /// Configuration register value (sequential T + RH acquisition).
    pub const fn register(self) -> u16 {
        let t = match self.temperature {
            TemperatureResolution::Bits14 => 0,
            TemperatureResolution::Bits11 => 1 << 10,
        };
        let h = match self.humidity {
            HumidityResolution::Bits14 => 0,
            HumidityResolution::Bits11 => 1 << 8,
            HumidityResolution::Bits8 => 2 << 8,
        };
        let heat = if self.heater { CONFIG_HEATER } else { 0 };
        CONFIG_SEQUENTIAL | heat | t | h
    }

    /// Conversion time of one T + RH sequence. The datasheet gives typical
    /// values only, so 10 % margin is added.
    pub const fn wait_us(self) -> WaitUs {
        let t = match self.temperature {
            TemperatureResolution::Bits14 => 6_350,
            TemperatureResolution::Bits11 => 3_650,
        };
        let h = match self.humidity {
            HumidityResolution::Bits14 => 6_500,
            HumidityResolution::Bits11 => 3_850,
            HumidityResolution::Bits8 => 2_500,
        };
        (t + h) * 11 / 10
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

/// Raw words to engineering units: T = raw / 2^16 × 165 − 40 °C,
/// RH = raw / 2^16 × 100 %.
pub fn convert(raw_temperature: u16, raw_humidity: u16) -> Measurement {
    let t = ((16_500 * i32::from(raw_temperature) + 32_768) >> 16) - 4_000;
    let rh = (10_000 * u32::from(raw_humidity) + 32_768) >> 16;
    Measurement {
        temperature: t,
        humidity: rh as u16,
    }
}

/// Decode the 4-byte sequential read (T MSB, T LSB, RH MSB, RH LSB).
pub fn decode_measurement(frame: &[u8; 4]) -> Measurement {
    convert(
        u16::from_be_bytes([frame[0], frame[1]]),
        u16::from_be_bytes([frame[2], frame[3]]),
    )
}

/// Assemble the 41-bit serial number from registers 0xFB, 0xFC, 0xFD.
pub const fn decode_serial(words: [u16; 3]) -> u64 {
    ((words[0] as u64) << 25) | ((words[1] as u64) << 9) | ((words[2] >> 7) as u64)
}

macro_rules! constructors {
    () => {
        /// Create a driver at [`DEFAULT_ADDRESS`](crate::DEFAULT_ADDRESS);
        /// no bus traffic. Assumes the power-on configuration until
        /// `configure` is called.
        pub const fn new(i2c: I2C, variant: Variant) -> Self {
            Self {
                i2c,
                address: DEFAULT_ADDRESS,
                variant,
                config: Config {
                    temperature: TemperatureResolution::Bits14,
                    humidity: HumidityResolution::Bits14,
                    heater: false,
                },
            }
        }

        /// Use another address (HDC1010 0x41..=0x43).
        pub const fn with_address(mut self, address: u8) -> Self {
            self.address = address;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        fn check_ids<E>(&self, manufacturer: u16, device: u16) -> Result<(), Error<E>> {
            if manufacturer == MANUFACTURER_ID && device == self.variant.device_id() {
                Ok(())
            } else {
                Err(Error::UnexpectedId(
                    (u32::from(manufacturer) << 16) | u32::from(device),
                ))
            }
        }
    };
}

/// Blocking driver.
pub struct Hdc1080<I2C> {
    i2c: I2C,
    address: u8,
    variant: Variant,
    config: Config,
}

impl<I2C> Hdc1080<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Hdc1080<I2C> {
    fn read_register(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .map_err(Error::bus)?;
        Ok(u16::from_be_bytes(buf))
    }

    fn write_config(&mut self, value: u16) -> Result<(), Error<I2C::Error>> {
        let [msb, lsb] = value.to_be_bytes();
        self.i2c
            .write(self.address, &[REG_CONFIG, msb, lsb])
            .map_err(Error::bus)
    }

    /// Check manufacturer (0x5449) and device ID; mismatch returns
    /// [`Error::UnexpectedId`] with `manufacturer << 16 | device`.
    pub fn verify_id(&mut self) -> Result<(), Error<I2C::Error>> {
        let manufacturer = self.read_register(REG_MANUFACTURER_ID)?;
        let device = self.read_register(REG_DEVICE_ID)?;
        self.check_ids(manufacturer, device)
    }

    /// Write resolution / heater and select sequential acquisition.
    pub fn configure(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        self.write_config(config.register())?;
        self.config = config;
        Ok(())
    }

    /// Supply voltage below 2.8 V at the last measurement (BTST).
    pub fn battery_low(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok(self.read_register(REG_CONFIG)? & CONFIG_BATTERY_LOW != 0)
    }

    /// Software reset. Restores defaults; wait [`STARTUP_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write_config(CONFIG_RESET)?;
        self.config = Config::default();
        Ok(STARTUP_WAIT_US)
    }

    /// 41-bit unique serial number.
    pub fn read_serial(&mut self) -> Result<u64, Error<I2C::Error>> {
        let mut words = [0u16; 3];
        for (word, reg) in words.iter_mut().zip(REG_SERIAL) {
            *word = self.read_register(reg)?;
        }
        Ok(decode_serial(words))
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Hdc1080<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[REG_TEMPERATURE])
            .map_err(Error::bus)?;
        Ok(self.config.wait_us())
    }

    /// Returns [`Error::Bus`] (NACK) while the conversion is running.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut frame = [0u8; 4];
        self.i2c
            .read(self.address, &mut frame)
            .map_err(Error::bus)?;
        Ok(decode_measurement(&frame))
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CONFIG_BATTERY_LOW, CONFIG_RESET, Config, DEFAULT_ADDRESS, HumidityResolution,
        MANUFACTURER_ID, Measurement, REG_CONFIG, REG_DEVICE_ID, REG_MANUFACTURER_ID, REG_SERIAL,
        REG_TEMPERATURE, STARTUP_WAIT_US, TemperatureResolution, Variant, decode_measurement,
        decode_serial,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Hdc1080`].
    pub struct Hdc1080<I2C> {
        i2c: I2C,
        address: u8,
        variant: Variant,
        config: Config,
    }

    impl<I2C> Hdc1080<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Hdc1080<I2C> {
        async fn read_register(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
            let mut buf = [0u8; 2];
            self.i2c
                .write_read(self.address, &[reg], &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(u16::from_be_bytes(buf))
        }

        async fn write_config(&mut self, value: u16) -> Result<(), Error<I2C::Error>> {
            let [msb, lsb] = value.to_be_bytes();
            self.i2c
                .write(self.address, &[REG_CONFIG, msb, lsb])
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Hdc1080::verify_id`].
        pub async fn verify_id(&mut self) -> Result<(), Error<I2C::Error>> {
            let manufacturer = self.read_register(REG_MANUFACTURER_ID).await?;
            let device = self.read_register(REG_DEVICE_ID).await?;
            self.check_ids(manufacturer, device)
        }

        /// See [`super::Hdc1080::configure`].
        pub async fn configure(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
            self.write_config(config.register()).await?;
            self.config = config;
            Ok(())
        }

        /// See [`super::Hdc1080::battery_low`].
        pub async fn battery_low(&mut self) -> Result<bool, Error<I2C::Error>> {
            Ok(self.read_register(REG_CONFIG).await? & CONFIG_BATTERY_LOW != 0)
        }

        /// See [`super::Hdc1080::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write_config(CONFIG_RESET).await?;
            self.config = Config::default();
            Ok(STARTUP_WAIT_US)
        }

        /// See [`super::Hdc1080::read_serial`].
        pub async fn read_serial(&mut self) -> Result<u64, Error<I2C::Error>> {
            let mut words = [0u16; 3];
            for (word, reg) in words.iter_mut().zip(REG_SERIAL) {
                *word = self.read_register(reg).await?;
            }
            Ok(decode_serial(words))
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Hdc1080<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[REG_TEMPERATURE])
                .await
                .map_err(Error::bus)?;
            Ok(self.config.wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut frame = [0u8; 4];
            self.i2c
                .read(self.address, &mut frame)
                .await
                .map_err(Error::bus)?;
            Ok(decode_measurement(&frame))
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
    fn conversion_matches_datasheet_formula() {
        assert_eq!(
            convert(0, 0),
            Measurement {
                temperature: -4_000,
                humidity: 0
            }
        );
        // 0x6666: 0.4 × 165 − 40 = 26.00 °C; 0x8000: 50.00 %RH.
        assert_eq!(
            convert(0x6666, 0x8000),
            Measurement {
                temperature: 2_600,
                humidity: 5_000
            }
        );
        // Full scale: 0xFFFC (2 LSBs are always 0) -> 124.99 °C, 99.99 %RH.
        assert_eq!(
            convert(0xFFFC, 0xFFFC),
            Measurement {
                temperature: 12_499,
                humidity: 9_999
            }
        );
    }

    #[test]
    fn config_register_and_wait() {
        assert_eq!(Config::default().register(), 0x1000);
        assert_eq!(Config::default().wait_us(), 14_135);
        let fast = Config {
            temperature: TemperatureResolution::Bits11,
            humidity: HumidityResolution::Bits8,
            heater: true,
        };
        assert_eq!(fast.register(), 0x3600);
        assert_eq!(fast.wait_us(), 6_765);
    }

    #[test]
    fn configure_then_measure() {
        let expectations = [
            Transaction::write(DEFAULT_ADDRESS, vec![0x02, 0x15, 0x00]),
            Transaction::write(DEFAULT_ADDRESS, vec![0x00]),
            Transaction::read(DEFAULT_ADDRESS, vec![0x66, 0x66, 0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hdc = Hdc1080::new(&mut i2c, Variant::Hdc1080);
        hdc.configure(Config {
            humidity: HumidityResolution::Bits11,
            temperature: TemperatureResolution::Bits11,
            heater: false,
        })
        .unwrap();
        assert_eq!(hdc.start_measurement().unwrap(), (3_650 + 3_850) * 11 / 10);
        assert_eq!(
            hdc.read_measurement().unwrap(),
            Measurement {
                temperature: 2_600,
                humidity: 5_000
            }
        );
        i2c.done();
    }

    #[test]
    fn id_check() {
        let expectations = [
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xFE], vec![0x54, 0x49]),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xFF], vec![0x10, 0x50]),
            Transaction::write_read(0x43, vec![0xFE], vec![0x54, 0x49]),
            Transaction::write_read(0x43, vec![0xFF], vec![0x10, 0x50]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hdc = Hdc1080::new(&mut i2c, Variant::Hdc1080);
        hdc.verify_id().unwrap();
        let mut hdc = hdc.release();
        let mut hdc1010 = Hdc1080::new(&mut hdc, Variant::Hdc1010).with_address(0x43);
        assert_eq!(hdc1010.verify_id(), Err(Error::UnexpectedId(0x5449_1050)));
        i2c.done();
    }

    #[test]
    fn serial_and_battery() {
        let expectations = [
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xFB], vec![0x12, 0x34]),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xFC], vec![0x56, 0x78]),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0xFD], vec![0x9A, 0x80]),
            Transaction::write_read(DEFAULT_ADDRESS, vec![0x02], vec![0x18, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hdc = Hdc1080::new(&mut i2c, Variant::Hdc1080);
        assert_eq!(
            hdc.read_serial().unwrap(),
            (0x1234 << 25) | (0x5678 << 9) | 0x135
        );
        assert!(hdc.battery_low().unwrap());
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
            Transaction::write(DEFAULT_ADDRESS, vec![0x00]),
            Transaction::read(DEFAULT_ADDRESS, vec![0x66, 0x66, 0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hdc = asynch::Hdc1080::new(&mut i2c, Variant::Hdc1080);
        let mut delay = RecordingDelay(0);
        let m = block_on(hdc.measure(&mut delay)).unwrap();
        assert_eq!(m.temperature, 2_600);
        assert_eq!(delay.0, 14_135);
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
