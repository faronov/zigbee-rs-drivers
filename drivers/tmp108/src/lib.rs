//! TI TMP108 digital temperature sensor (I2C / SMBus).
//!
//! [`configure`](Tmp108::configure) leaves the sensor in shutdown
//! (0.3 µA typ.); every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! selects one-shot mode (M1:M0 = 01). The part converts once (33 ms max)
//! and returns to shutdown by itself.
//!
//! Reference: TI "TMP108 ±0.75°C Digital Temperature Sensor With Two-Wire
//! Interface and Alert", SBOS663A (September 2019).
//!
//! ```ignore
//! let mut tmp = Tmp108::new(i2c, ADDRESS_GND);
//! tmp.configure()?;
//! let wait_us = tmp.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = tmp.read_measurement()?;
//! ```
//!
//! The blocking driver and its [`asynch`] twin are generated from one macro
//! body, so they issue identical bus transactions.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with A0 tied to GND.
pub const ADDRESS_GND: u8 = 0x48;
/// I2C address with A0 tied to V+.
pub const ADDRESS_VCC: u8 = 0x49;
/// I2C address with A0 tied to SDA.
pub const ADDRESS_SDA: u8 = 0x4A;
/// I2C address with A0 tied to SCL.
pub const ADDRESS_SCL: u8 = 0x4B;

/// One-shot conversion time, data sheet maximum (27 ms typ.).
pub const CONVERSION_WAIT_US: WaitUs = 33_000;

const REG_TEMPERATURE: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;
const REG_T_LOW: u8 = 0x02;
const REG_T_HIGH: u8 = 0x03;

const CFG_FH: u16 = 1 << 12;
const CFG_FL: u16 = 1 << 11;
const CFG_TM: u16 = 1 << 10;
const CFG_POL: u16 = 1 << 7;
const CFG_RESERVED: u16 = (1 << 6) | 0x000F;

const MODE_SHUTDOWN: u16 = 0b00 << 8;
const MODE_ONE_SHOT: u16 = 0b01 << 8;
const MODE_CONTINUOUS: u16 = 0b10 << 8;

/// Continuous-mode conversion rate (CR1/CR0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConversionRate {
    /// 0.25 conversions per second (1 µA typ.).
    Hz0_25,
    /// 1 conversion per second (2 µA typ., power-on default).
    #[default]
    Hz1,
    /// 4 conversions per second (5 µA typ.).
    Hz4,
    /// 16 conversions per second (18 µA typ.).
    Hz16,
}

/// Comparator-mode hysteresis (HYS1/HYS0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Hysteresis {
    /// 0 °C.
    C0,
    /// 1 °C (default).
    #[default]
    C1,
    /// 2 °C.
    C2,
    /// 4 °C.
    C4,
}

/// ALERT pin behaviour (TM bit).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AlertMode {
    /// Window comparator: active while `T > T_HIGH` or `T < T_LOW`, released
    /// inside `T_LOW + HYS .. T_HIGH - HYS`.
    #[default]
    Comparator,
    /// Latched until the configuration register is read or the SMBus alert
    /// response.
    Interrupt,
}

/// Configuration written by [`Tmp108::configure`] and every one-shot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Rate used in continuous mode.
    pub conversion_rate: ConversionRate,
    /// ALERT comparator / interrupt mode.
    pub alert_mode: AlertMode,
    /// ALERT active high (`POL = 1`) instead of active low.
    pub alert_active_high: bool,
    /// Comparator hysteresis.
    pub hysteresis: Hysteresis,
}

impl Config {
    const fn register(self, mode: u16) -> u16 {
        let mut word = ((self.conversion_rate as u16) << 13) | ((self.hysteresis as u16) << 4);
        if matches!(self.alert_mode, AlertMode::Interrupt) {
            word |= CFG_TM;
        }
        if self.alert_active_high {
            word |= CFG_POL;
        }
        word | mode
    }
}

/// Limit flags from the configuration register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Flags {
    /// Temperature exceeded `T_HIGH` (FH).
    pub high: bool,
    /// Temperature fell below `T_LOW` (FL).
    pub low: bool,
}

/// One temperature sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
    }
}

/// Decode the 12-bit left-justified temperature register (data sheet
/// table 7). Returns `None` for `0xFFFF` (low nibble is always 0 on a
/// real conversion; all ones means a missing device).
pub const fn decode_temperature(raw: u16) -> Option<i32> {
    if raw & 0x000F != 0 {
        return None;
    }
    let counts = (raw as i16 >> 4) as i32;
    Some((counts * 100 + 8) >> 4)
}

/// Encode a limit in 0.01 °C into the `T_LOW`/`T_HIGH` format, saturating.
pub const fn encode_limit(centi_celsius: i32) -> u16 {
    let counts = (centi_celsius * 16 + 50).div_euclid(100);
    let c = if counts < -2048 {
        -2048
    } else if counts > 2047 {
        2047
    } else {
        counts
    };
    ((c << 4) as u16) & 0xFFF0
}

macro_rules! constructors {
    () => {
        /// Create a driver with the default [`Config`]; no bus traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                config: Config {
                    conversion_rate: ConversionRate::Hz1,
                    alert_mode: AlertMode::Comparator,
                    alert_active_high: false,
                    hysteresis: Hysteresis::C1,
                },
            }
        }

        /// Use `config` for [`Self::configure`] and later mode changes.
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

macro_rules! driver_impl {
    ($i2c:path, $sensor:path, $low_power:path, $($async:ident)?; $($await:tt)*) => {
        impl<I2C: $i2c> Tmp108<I2C> {
            $($async)? fn write_register(&mut self, register: u8, value: u16) -> Result<(), Error<I2C::Error>> {
                let [hi, lo] = value.to_be_bytes();
                self.i2c
                    .write(self.address, &[register, hi, lo])
                    $($await)*
                    .map_err(Error::bus)
            }

            $($async)? fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(self.address, &[register], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                Ok(u16::from_be_bytes(buf))
            }

            /// Plausibility check (the part has no ID register): the reserved
            /// configuration bits must read 0. Returns
            /// [`Error::UnexpectedId`] with the configuration word otherwise.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let config = self.read_register(REG_CONFIG) $($await)*?;
                if config & CFG_RESERVED != 0 {
                    return Err(Error::UnexpectedId(u32::from(config)));
                }
                Ok(())
            }

            /// Write the configuration and enter shutdown. Call once after
            /// power-up: the part starts in continuous 1 Hz mode.
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(MODE_SHUTDOWN)) $($await)*
            }

            /// Change the configuration and write it (enters shutdown).
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program the ALERT window in 0.01 °C.
            pub $($async)? fn set_limits(&mut self, low: i32, high: i32) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_T_LOW, encode_limit(low)) $($await)*?;
                self.write_register(REG_T_HIGH, encode_limit(high)) $($await)*
            }

            /// Read the FH/FL flags. In interrupt mode this read also clears
            /// the flags and releases ALERT.
            pub $($async)? fn read_flags(&mut self) -> Result<Flags, Error<I2C::Error>> {
                let config = self.read_register(REG_CONFIG) $($await)*?;
                Ok(Flags {
                    high: config & CFG_FH != 0,
                    low: config & CFG_FL != 0,
                })
            }

            /// `true` once a one-shot conversion has finished (M1:M0 back
            /// to 00).
            pub $($async)? fn conversion_done(&mut self) -> Result<bool, Error<I2C::Error>> {
                Ok(self.read_register(REG_CONFIG) $($await)*? & (0b11 << 8) == MODE_SHUTDOWN)
            }
        }

        impl<I2C: $i2c> $sensor for Tmp108<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot conversion from shutdown (M1:M0 = 01).
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(MODE_ONE_SHOT)) $($await)*?;
                Ok(CONVERSION_WAIT_US)
            }

            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let raw = self.read_register(REG_TEMPERATURE) $($await)*?;
                match decode_temperature(raw) {
                    Some(temperature) => Ok(Measurement { temperature }),
                    None => Err(Error::InvalidData),
                }
            }
        }

        impl<I2C: $i2c> $low_power for Tmp108<I2C> {
            type BusError = I2C::Error;

            /// Shutdown (M1:M0 = 00).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.configure() $($await)*
            }

            /// Continuous conversion at the configured rate. Returns the wait
            /// until the first result.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(MODE_CONTINUOUS)) $($await)*?;
                Ok(CONVERSION_WAIT_US)
            }
        }
    };
}

/// Blocking driver.
pub struct Tmp108<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
}

impl<I2C> Tmp108<I2C> {
    constructors!();
}

driver_impl!(
    embedded_hal::i2c::I2c,
    zigbee_sensor::blocking::Sensor,
    zigbee_sensor::blocking::LowPower,
    ;
);

/// Async driver.
pub mod asynch {
    use super::*;

    /// Async twin of [`super::Tmp108`].
    pub struct Tmp108<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
    }

    impl<I2C> Tmp108<I2C> {
        constructors!();
    }

    driver_impl!(
        embedded_hal_async::i2c::I2c,
        zigbee_sensor::asynch::Sensor,
        zigbee_sensor::asynch::LowPower,
        async; .await
    );
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = ADDRESS_GND;

    #[test]
    fn decodes_datasheet_table() {
        // SBOS663A table 7.
        assert_eq!(decode_temperature(0x7FF0), Some(12_794)); // 127.9375
        assert_eq!(decode_temperature(0x6400), Some(10_000));
        assert_eq!(decode_temperature(0x3200), Some(5_000));
        assert_eq!(decode_temperature(0x1900), Some(2_500));
        assert_eq!(decode_temperature(0x0040), Some(25));
        assert_eq!(decode_temperature(0x0000), Some(0));
        assert_eq!(decode_temperature(0xFFC0), Some(-25));
        assert_eq!(decode_temperature(0xE700), Some(-2_500));
        assert_eq!(decode_temperature(0xC900), Some(-5_500));
        assert_eq!(decode_temperature(0x8000), Some(-12_800));
        assert_eq!(decode_temperature(0xFFFF), None);
    }

    #[test]
    fn limits_use_temperature_format() {
        assert_eq!(encode_limit(5_000), 0x3200);
        assert_eq!(encode_limit(-2_500), 0xE700);
        assert_eq!(encode_limit(20_000), 0x7FF0);
        assert_eq!(encode_limit(-20_000), 0x8000);
    }

    #[test]
    fn default_register_values() {
        // Power-on 0x2210 = CR 01, continuous, HYS 1 °C.
        assert_eq!(Config::default().register(MODE_CONTINUOUS), 0x2210);
        assert_eq!(Config::default().register(MODE_ONE_SHOT), 0x2110);
    }

    #[test]
    fn one_shot_then_read() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x21, 0x10]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x19, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp108::new(&mut i2c, A);
        assert_eq!(tmp.start_measurement().unwrap(), 33_000);
        assert_eq!(tmp.read_measurement().unwrap().temperature, 2_500);
        i2c.done();
    }

    #[test]
    fn configure_alert_and_limits() {
        let config = Config {
            conversion_rate: ConversionRate::Hz0_25,
            alert_mode: AlertMode::Interrupt,
            alert_active_high: true,
            hysteresis: Hysteresis::C4,
        };
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x04, 0xB0]),
            Transaction::write(A, vec![REG_T_LOW, 0xE7, 0x00]),
            Transaction::write(A, vec![REG_T_HIGH, 0x32, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp108::new(&mut i2c, A).with_config(config);
        tmp.configure().unwrap();
        tmp.set_limits(-2_500, 5_000).unwrap();
        i2c.done();
    }

    #[test]
    fn flags_probe_and_done() {
        let expectations = [
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x30, 0x10]),
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x20, 0x10]),
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x21, 0x10]),
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x22, 0x1F]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp108::new(&mut i2c, A);
        assert_eq!(
            tmp.read_flags().unwrap(),
            Flags {
                high: true,
                low: false
            }
        );
        assert!(tmp.conversion_done().unwrap());
        assert!(!tmp.conversion_done().unwrap());
        assert_eq!(tmp.probe(), Err(Error::UnexpectedId(0x221F)));
        i2c.done();
    }

    #[test]
    fn sleep_wake_and_invalid_read() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x20, 0x10]),
            Transaction::write(A, vec![REG_CONFIG, 0x22, 0x10]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0xFF, 0xFF]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp108::new(&mut i2c, A);
        tmp.sleep().unwrap();
        assert_eq!(tmp.wake().unwrap(), CONVERSION_WAIT_US);
        assert_eq!(tmp.read_measurement(), Err(Error::InvalidData));
        i2c.done();
    }

    #[test]
    fn async_one_shot() {
        use zigbee_sensor::asynch::Sensor as _;
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x21, 0x10]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0xE7, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = asynch::Tmp108::new(&mut i2c, A);
        assert_eq!(block_on(tmp.start_measurement()).unwrap(), 33_000);
        assert_eq!(
            block_on(tmp.read_measurement()).unwrap().temperature,
            -2_500
        );
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
