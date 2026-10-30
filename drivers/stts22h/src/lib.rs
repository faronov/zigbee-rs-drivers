//! ST STTS22H digital temperature sensor (I2C / SMBus), 0.01 °C/LSB.
//!
//! The part powers up in one-shot mode (power-down, 0.5 µA typ.). Every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! sets `ONE_SHOT`; after the conversion the device powers down again.
//! [`LowPower::wake`](zigbee_sensor::blocking::LowPower::wake) starts the
//! 1 Hz low-ODR mode (2 µA) or freerun mode instead.
//!
//! Reference: ST "STTS22H Low-voltage, ultralow-power, 0.5 °C accuracy
//! I²C/SMBus 3.0 temperature sensor", DS12606 Rev 8 (March 2026).
//!
//! The data sheet does not state a one-shot conversion time. The driver
//! waits 1.5 × the freerun output period for the configured averaging
//! (8 / 4 / 2 / 1 samples ↔ 25 / 50 / 100 / 200 Hz → 60 / 30 / 15 / 7.5 ms);
//! [`conversion_busy`](Stts22h::conversion_busy) reports the BUSY bit.
//!
//! ```ignore
//! let mut t = Stts22h::new(i2c, ADDRESS_GND);
//! t.probe()?; // WHOAMI = 0xA0
//! t.configure()?;
//! let wait_us = t.start_measurement()?;
//! // ... sleep for wait_us ...
//! let m = t.read_measurement()?;
//! ```
//!
//! The blocking driver and its [`asynch`] twin are generated from one macro
//! body, so they issue identical bus transactions.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Address with the Addr pin tied to GND.
pub const ADDRESS_GND: u8 = 0x3F;
/// Address with a 56 kΩ pull-up on Addr.
pub const ADDRESS_56K: u8 = 0x3E;
/// Address with a 15 kΩ pull-up on Addr.
pub const ADDRESS_15K: u8 = 0x3C;
/// Address with the Addr pin tied to VDD.
pub const ADDRESS_VDD: u8 = 0x38;

/// WHOAMI register value.
pub const WHO_AM_I: u8 = 0xA0;

/// Boot time after power-up before the first access (Ton).
pub const BOOT_TIME_US: WaitUs = 12_000;

const REG_WHOAMI: u8 = 0x01;
const REG_TEMP_H_LIMIT: u8 = 0x02;
const REG_TEMP_L_LIMIT: u8 = 0x03;
const REG_CTRL: u8 = 0x04;
const REG_STATUS: u8 = 0x05;
const REG_TEMP_L_OUT: u8 = 0x06;

const CTRL_LOW_ODR_START: u8 = 1 << 7;
const CTRL_BDU: u8 = 1 << 6;
const CTRL_IF_ADD_INC: u8 = 1 << 3;
const CTRL_FREERUN: u8 = 1 << 2;
const CTRL_TIME_OUT_DIS: u8 = 1 << 1;
const CTRL_ONE_SHOT: u8 = 1 << 0;

const STATUS_UNDER_THL: u8 = 1 << 2;
const STATUS_OVER_THH: u8 = 1 << 1;
const STATUS_BUSY: u8 = 1 << 0;

/// Samples averaged per result (`AVG[1:0]`); also sets the freerun ODR.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Averaging {
    /// 8 samples, 25 Hz freerun, 0.02 °C RMS noise (power-on default).
    #[default]
    Avg8,
    /// 4 samples, 50 Hz.
    Avg4,
    /// 2 samples, 100 Hz.
    Avg2,
    /// 1 sample, 200 Hz, 0.055 °C RMS noise.
    Avg1,
}

impl Averaging {
    /// One-shot wait: 1.5 × the freerun period.
    pub const fn conversion_us(self) -> WaitUs {
        60_000 >> (self as u32)
    }
}

/// Mode entered by [`LowPower::wake`](zigbee_sensor::blocking::LowPower::wake).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ContinuousMode {
    /// 1 Hz low-ODR mode (2 µA typ.).
    #[default]
    LowOdr,
    /// Freerun at the ODR set by [`Averaging`].
    Freerun,
}

/// Configuration written by [`Stts22h::configure`] and every CTRL write.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Averaging (one-shot noise vs. time, freerun ODR).
    pub averaging: Averaging,
    /// Mode used by `wake`.
    pub continuous: ContinuousMode,
    /// Disable the 30 ms SMBus timeout.
    pub disable_timeout: bool,
}

impl Config {
    /// CTRL register byte. IF_ADD_INC and BDU are always set.
    pub const fn ctrl(self, mode: u8) -> u8 {
        let mut byte = CTRL_BDU | CTRL_IF_ADD_INC | ((self.averaging as u8) << 4) | mode;
        if self.disable_timeout {
            byte |= CTRL_TIME_OUT_DIS;
        }
        byte
    }

    const fn continuous_bit(self) -> u8 {
        match self.continuous {
            ContinuousMode::LowOdr => CTRL_LOW_ODR_START,
            ContinuousMode::Freerun => CTRL_FREERUN,
        }
    }

    const fn first_sample_us(self) -> WaitUs {
        match self.continuous {
            ContinuousMode::LowOdr => 1_500_000,
            ContinuousMode::Freerun => self.averaging.conversion_us(),
        }
    }
}

/// STATUS register flags. Reading STATUS clears them and releases
/// ALERT/INT.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    /// One-shot conversion in progress.
    pub busy: bool,
    /// Last result ≥ high limit.
    pub over_high: bool,
    /// Last result < low limit.
    pub under_low: bool,
}

impl Status {
    /// Decode the STATUS byte.
    pub const fn from_bits(bits: u8) -> Self {
        Self {
            busy: bits & STATUS_BUSY != 0,
            over_high: bits & STATUS_OVER_THH != 0,
            under_low: bits & STATUS_UNDER_THL != 0,
        }
    }
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

/// Decode TEMP_L_OUT / TEMP_H_OUT (two's complement, 0.01 °C/LSB).
pub const fn decode_temperature(lo: u8, hi: u8) -> i32 {
    i16::from_le_bytes([lo, hi]) as i32
}

/// Encode an interrupt threshold: `(reg - 63) × 0.64 °C`, rounded and
/// clamped to 1..=255 (−39.68 … +122.88 °C). `None` disables the limit
/// (register 0).
pub const fn encode_limit(centi_celsius: Option<i32>) -> u8 {
    match centi_celsius {
        None => 0,
        Some(c) => {
            let reg = (c + 32).div_euclid(64) + 63;
            if reg < 1 {
                1
            } else if reg > 255 {
                255
            } else {
                reg as u8
            }
        }
    }
}

/// Threshold in 0.01 °C for a limit register value (`None` = disabled).
pub const fn decode_limit(reg: u8) -> Option<i32> {
    if reg == 0 {
        None
    } else {
        Some((reg as i32 - 63) * 64)
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver with the default [`Config`]; no bus traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                config: Config {
                    averaging: Averaging::Avg8,
                    continuous: ContinuousMode::LowOdr,
                    disable_timeout: false,
                },
            }
        }

        /// Use `config` for later CTRL writes.
        pub const fn with_config(mut self, config: Config) -> Self {
            self.config = config;
            self
        }

        /// The active configuration.
        pub const fn config(&self) -> Config {
            self.config
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

macro_rules! driver_impl {
    ($i2c:path, $sensor:path, $low_power:path, $($async:ident)?; $($await:tt)*) => {
        impl<I2C: $i2c> Stts22h<I2C> {
            $($async)? fn write_reg(&mut self, register: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(self.address, &[register, value])
                    $($await)*
                    .map_err(Error::bus)
            }

            $($async)? fn read_reg(&mut self, register: u8) -> Result<u8, Error<I2C::Error>> {
                let mut buf = [0u8; 1];
                self.i2c
                    .write_read(self.address, &[register], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                Ok(buf[0])
            }

            /// Check WHOAMI (0xA0).
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let id = self.read_reg(REG_WHOAMI) $($await)*?;
                if id != WHO_AM_I {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(())
            }

            /// Write CTRL in power-down (one-shot mode). Required before
            /// changing between modes.
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_reg(REG_CTRL, self.config.ctrl(0)) $($await)*
            }

            /// Change the configuration and write it (powers down).
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program the ALERT/INT thresholds in 0.01 °C (0.64 °C steps,
            /// `None` disables). ALERT asserts at `T >= high` or `T < low`
            /// and stays latched until STATUS is read or ARA.
            pub $($async)? fn set_limits(&mut self, low: Option<i32>, high: Option<i32>) -> Result<(), Error<I2C::Error>> {
                self.write_reg(REG_TEMP_L_LIMIT, encode_limit(low)) $($await)*?;
                self.write_reg(REG_TEMP_H_LIMIT, encode_limit(high)) $($await)*
            }

            /// Read (and thereby clear) STATUS; releases ALERT/INT.
            pub $($async)? fn read_status(&mut self) -> Result<Status, Error<I2C::Error>> {
                Ok(Status::from_bits(self.read_reg(REG_STATUS) $($await)*?))
            }

            /// `true` while a one-shot conversion runs. Also clears the
            /// limit flags and ALERT.
            pub $($async)? fn conversion_busy(&mut self) -> Result<bool, Error<I2C::Error>> {
                Ok(self.read_status() $($await)*?.busy)
            }
        }

        impl<I2C: $i2c> $sensor for Stts22h<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot conversion from power-down.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_reg(REG_CTRL, self.config.ctrl(CTRL_ONE_SHOT)) $($await)*?;
                Ok(self.config.averaging.conversion_us())
            }

            /// Reads TEMP_L_OUT and TEMP_H_OUT in one auto-increment read.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(self.address, &[REG_TEMP_L_OUT], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                Ok(Measurement { temperature: decode_temperature(buf[0], buf[1]) })
            }
        }

        impl<I2C: $i2c> $low_power for Stts22h<I2C> {
            type BusError = I2C::Error;

            /// Power-down (FREERUN = LOW_ODR_START = 0).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.configure() $($await)*
            }

            /// Power down, then start the configured continuous mode.
            /// Returns the wait until the first result (1.5 s for low-ODR).
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.configure() $($await)*?;
                let mode = self.config.continuous_bit();
                self.write_reg(REG_CTRL, self.config.ctrl(mode)) $($await)*?;
                Ok(self.config.first_sample_us())
            }
        }
    };
}

/// Blocking driver.
pub struct Stts22h<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
}

impl<I2C> Stts22h<I2C> {
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

    /// Async twin of [`super::Stts22h`].
    pub struct Stts22h<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
    }

    impl<I2C> Stts22h<I2C> {
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
    fn decodes_temperature_and_limits() {
        assert_eq!(decode_temperature(0xC4, 0x09), 2_500);
        assert_eq!(decode_temperature(0x3C, 0xF6), -2_500);
        assert_eq!(decode_temperature(0x01, 0x00), 1);
        // Table 14: 255 → 122.88 °C, 63 → 0 °C, 1 → −39.68 °C, 0 off.
        assert_eq!(decode_limit(255), Some(12_288));
        assert_eq!(decode_limit(63), Some(0));
        assert_eq!(decode_limit(1), Some(-3_968));
        assert_eq!(decode_limit(0), None);
        assert_eq!(encode_limit(Some(12_288)), 255);
        assert_eq!(encode_limit(Some(0)), 63);
        assert_eq!(encode_limit(Some(-3_968)), 1);
        assert_eq!(encode_limit(Some(3_000)), 110); // 30.08 °C
        assert_eq!(encode_limit(Some(-10_000)), 1);
        assert_eq!(encode_limit(Some(20_000)), 255);
        assert_eq!(encode_limit(None), 0);
    }

    #[test]
    fn ctrl_bits_and_waits() {
        assert_eq!(Config::default().ctrl(0), 0x48);
        let c = Config {
            averaging: Averaging::Avg1,
            continuous: ContinuousMode::Freerun,
            disable_timeout: true,
        };
        assert_eq!(c.ctrl(CTRL_ONE_SHOT), 0x7B);
        assert_eq!(Averaging::Avg8.conversion_us(), 60_000);
        assert_eq!(Averaging::Avg1.conversion_us(), 7_500);
    }

    #[test]
    fn probe_and_one_shot() {
        let expectations = [
            Transaction::write_read(A, vec![REG_WHOAMI], vec![0xA0]),
            Transaction::write_read(A, vec![REG_WHOAMI], vec![0x51]),
            Transaction::write(A, vec![REG_CTRL, 0x49]),
            Transaction::write_read(A, vec![REG_STATUS], vec![0x01]),
            Transaction::write_read(A, vec![REG_STATUS], vec![0x02]),
            Transaction::write_read(A, vec![REG_TEMP_L_OUT], vec![0xC4, 0x09]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = Stts22h::new(&mut i2c, A);
        assert_eq!(t.probe(), Ok(()));
        assert_eq!(t.probe(), Err(Error::UnexpectedId(0x51)));
        assert_eq!(t.start_measurement().unwrap(), 60_000);
        assert!(t.conversion_busy().unwrap());
        assert_eq!(
            t.read_status().unwrap(),
            Status {
                busy: false,
                over_high: true,
                under_low: false
            }
        );
        assert_eq!(t.read_measurement().unwrap().temperature, 2_500);
        i2c.done();
    }

    #[test]
    fn limits_sleep_wake() {
        let expectations = [
            Transaction::write(A, vec![REG_TEMP_L_LIMIT, 0x00]),
            Transaction::write(A, vec![REG_TEMP_H_LIMIT, 110]),
            Transaction::write(A, vec![REG_CTRL, 0x48]),
            Transaction::write(A, vec![REG_CTRL, 0x48]),
            Transaction::write(A, vec![REG_CTRL, 0xC8]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = Stts22h::new(&mut i2c, A);
        t.set_limits(None, Some(3_000)).unwrap();
        t.sleep().unwrap();
        assert_eq!(t.wake().unwrap(), 1_500_000);
        i2c.done();
    }

    #[test]
    fn async_freerun_and_one_shot() {
        use zigbee_sensor::asynch::{LowPower as _, Sensor as _};
        let config = Config {
            averaging: Averaging::Avg2,
            continuous: ContinuousMode::Freerun,
            disable_timeout: false,
        };
        let expectations = [
            Transaction::write(A, vec![REG_CTRL, 0x68]),
            Transaction::write(A, vec![REG_CTRL, 0x6C]),
            Transaction::write(A, vec![REG_CTRL, 0x68]),
            Transaction::write(A, vec![REG_CTRL, 0x69]),
            Transaction::write_read(A, vec![REG_TEMP_L_OUT], vec![0x3C, 0xF6]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = asynch::Stts22h::new(&mut i2c, A).with_config(config);
        assert_eq!(block_on(t.wake()).unwrap(), 15_000);
        block_on(t.sleep()).unwrap();
        assert_eq!(block_on(t.start_measurement()).unwrap(), 15_000);
        assert_eq!(block_on(t.read_measurement()).unwrap().temperature, -2_500);
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
