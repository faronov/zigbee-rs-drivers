//! ST STTS751 digital temperature sensor (SMBus / I2C) with EVENT and
//! Addr/THERM outputs.
//!
//! [`configure`](Stts751::configure) puts the sensor in standby (3 µA typ.)
//! and every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! writes the one-shot register; after the conversion the part returns to
//! standby by itself.
//!
//! Reference: ST "STTS751 2.25 V low-voltage local digital temperature
//! sensor", DocID16483 Rev 7.
//!
//! ```ignore
//! let mut t = Stts751::new(i2c, 0x48);
//! let product = t.probe()?; // Manufacturer ID 0x53
//! t.configure()?; // standby, 10-bit, EVENT enabled
//! let wait_us = t.start_measurement()?;
//! // ... sleep for wait_us ...
//! let m = t.read_measurement()?;
//! ```
//!
//! The blocking driver and its [`asynch`] twin are generated from one macro
//! body, so they issue identical bus transactions.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Manufacturer ID register value.
pub const MANUFACTURER_ID: u8 = 0x53;

/// SMBus Alert Response Address.
pub const ALERT_RESPONSE_ADDRESS: u8 = 0x0C;

const REG_TEMP_HIGH: u8 = 0x00;
const REG_STATUS: u8 = 0x01;
const REG_TEMP_LOW: u8 = 0x02;
const REG_CONFIG: u8 = 0x03;
const REG_CONV_RATE: u8 = 0x04;
const REG_HIGH_LIMIT_HIGH: u8 = 0x05;
const REG_LOW_LIMIT_HIGH: u8 = 0x07;
const REG_ONE_SHOT: u8 = 0x0F;
const REG_THERM_LIMIT: u8 = 0x20;
const REG_THERM_HYST: u8 = 0x21;
const REG_SMBUS_TIMEOUT: u8 = 0x22;
const REG_PRODUCT_ID: u8 = 0xFD;
const REG_MANUFACTURER_ID: u8 = 0xFE;

const CFG_MASK1: u8 = 1 << 7;
const CFG_RUN_STOP: u8 = 1 << 6;
const TIMEOUT_ENABLE: u8 = 1 << 7;

/// Ordering version, identified by the Product ID register; selects the
/// address set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Product {
    /// STTS751-0 (product ID 0x00): 0x48 / 0x49 / 0x38 / 0x39 for
    /// 7.5 k / 12 k / 20 k / 33 k (or GND) on Addr/THERM.
    Stts751_0,
    /// STTS751-1 (product ID 0x01): 0x4A / 0x4B / 0x3A / 0x3B.
    Stts751_1,
}

impl Product {
    /// 7-bit address for the Addr/THERM pull-up selection `n`
    /// (0 = 7.5 kΩ, 1 = 12 kΩ, 2 = 20 kΩ, 3 = 33 kΩ or GND).
    pub const fn address(self, n: u8) -> u8 {
        let base = match n & 0x03 {
            0 => 0x48,
            1 => 0x49,
            2 => 0x38,
            _ => 0x39,
        };
        match self {
            Self::Stts751_0 => base,
            Self::Stts751_1 => base + 2,
        }
    }
}

/// Temperature resolution (Tres1:Tres0, note the non-monotonic encoding).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 9 bit, 0.5 °C.
    Bits9,
    /// 10 bit, 0.25 °C (power-on default).
    #[default]
    Bits10,
    /// 11 bit, 0.125 °C.
    Bits11,
    /// 12 bit, 0.0625 °C.
    Bits12,
}

impl Resolution {
    const fn tres(self) -> u8 {
        match self {
            Self::Bits9 => 0b10,
            Self::Bits10 => 0b00,
            Self::Bits11 => 0b01,
            Self::Bits12 => 0b11,
        }
    }

    /// Maximum conversion time (14 / 28 / 56 / 112 ms).
    pub const fn conversion_us(self) -> WaitUs {
        14_000 << (self as u32)
    }
}

/// Continuous conversion rate (`CONV[3:0]`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConversionRate {
    /// One conversion every 16 s.
    Hz0_0625,
    /// One every 8 s.
    Hz0_125,
    /// One every 4 s.
    Hz0_25,
    /// One every 2 s.
    Hz0_5,
    /// 1 per second (power-on default).
    #[default]
    Hz1,
    /// 2 per second.
    Hz2,
    /// 4 per second.
    Hz4,
    /// 8 per second.
    Hz8,
    /// 16 per second (≤ 11 bit).
    Hz16,
    /// 32 per second (≤ 10 bit).
    Hz32,
}

/// Configuration written by [`Stts751::configure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Temperature resolution.
    pub resolution: Resolution,
    /// Rate used in continuous mode.
    pub conversion_rate: ConversionRate,
    /// Drive EVENT on out-of-limit results (MASK1 = 0).
    pub event_enabled: bool,
    /// Keep the 25–35 ms SMBus timeout enabled.
    pub smbus_timeout: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            resolution: Resolution::Bits10,
            conversion_rate: ConversionRate::Hz1,
            event_enabled: true,
            smbus_timeout: true,
        }
    }
}

impl Config {
    /// Configuration register byte.
    pub const fn register(self, standby: bool) -> u8 {
        let mut byte = self.resolution.tres() << 2;
        if !self.event_enabled {
            byte |= CFG_MASK1;
        }
        if standby {
            byte |= CFG_RUN_STOP;
        }
        byte
    }

    /// `false` if the rate exceeds the limit for the resolution (16/s needs
    /// ≤ 11 bit, 32/s ≤ 10 bit).
    pub const fn is_valid(self) -> bool {
        match self.conversion_rate {
            ConversionRate::Hz16 => !matches!(self.resolution, Resolution::Bits12),
            ConversionRate::Hz32 => {
                matches!(self.resolution, Resolution::Bits9 | Resolution::Bits10)
            }
            _ => true,
        }
    }
}

/// Status register flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    /// Conversion in progress.
    pub busy: bool,
    /// Temperature above the high limit.
    pub high: bool,
    /// Temperature at or below the low limit.
    pub low: bool,
    /// THERM limit crossed (Addr/THERM asserted).
    pub therm: bool,
}

impl Status {
    /// Decode the status byte.
    pub const fn from_bits(bits: u8) -> Self {
        Self {
            busy: bits & 0x80 != 0,
            high: bits & 0x40 != 0,
            low: bits & 0x20 != 0,
            therm: bits & 0x01 != 0,
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

/// Decode the temperature high / low bytes (table 11). Returns `None` if
/// the always-zero low nibble is set (e.g. `0xFF` from a missing device).
pub const fn decode_temperature(high: u8, low: u8) -> Option<i32> {
    if low & 0x0F != 0 {
        return None;
    }
    let counts = (i16::from_be_bytes([high, low]) >> 4) as i32;
    Some((counts * 100 + 8) >> 4)
}

/// Encode a limit in 0.01 °C into high / low bytes, saturating to the
/// register range −64 … +127.9375 °C.
pub const fn encode_limit(centi_celsius: i32) -> [u8; 2] {
    let counts = (centi_celsius * 16 + 50).div_euclid(100);
    let c = if counts < -1024 {
        -1024
    } else if counts > 2047 {
        2047
    } else {
        counts
    };
    ((c << 4) as i16).to_be_bytes()
}

/// Encode a THERM limit / hysteresis in 0.01 °C as a whole-degree byte,
/// rounded and saturated to −64 … +127 °C.
pub const fn encode_therm(centi_celsius: i32) -> u8 {
    let c = (centi_celsius + 50).div_euclid(100);
    let c = if c < -64 {
        -64
    } else if c > 127 {
        127
    } else {
        c
    };
    c as i8 as u8
}

macro_rules! constructors {
    () => {
        /// Create a driver with the default [`Config`]; no bus traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                config: Config {
                    resolution: Resolution::Bits10,
                    conversion_rate: ConversionRate::Hz1,
                    event_enabled: true,
                    smbus_timeout: true,
                },
                in_standby: false,
            }
        }

        /// Use `config` for [`Self::configure`] and later mode changes.
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
        impl<I2C: $i2c> Stts751<I2C> {
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

            $($async)? fn write_config(&mut self, standby: bool) -> Result<(), Error<I2C::Error>> {
                self.write_reg(REG_CONFIG, self.config.register(standby)) $($await)*?;
                self.in_standby = standby;
                Ok(())
            }

            /// Check the Manufacturer ID (0x53) and return the version from
            /// the Product ID register.
            pub $($async)? fn probe(&mut self) -> Result<Product, Error<I2C::Error>> {
                let mfg = self.read_reg(REG_MANUFACTURER_ID) $($await)*?;
                let product = self.read_reg(REG_PRODUCT_ID) $($await)*?;
                match (mfg, product) {
                    (MANUFACTURER_ID, 0x00) => Ok(Product::Stts751_0),
                    (MANUFACTURER_ID, 0x01) => Ok(Product::Stts751_1),
                    _ => Err(Error::UnexpectedId((u32::from(mfg) << 8) | u32::from(product))),
                }
            }

            /// Write configuration, conversion rate and SMBus timeout, and
            /// enter standby. Returns [`Error::Unsupported`] for an invalid
            /// rate / resolution pair.
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                if !self.config.is_valid() {
                    return Err(Error::Unsupported);
                }
                self.write_config(true) $($await)*?;
                self.write_reg(REG_CONV_RATE, self.config.conversion_rate as u8) $($await)*?;
                let timeout = if self.config.smbus_timeout { TIMEOUT_ENABLE } else { 0 };
                self.write_reg(REG_SMBUS_TIMEOUT, timeout) $($await)*
            }

            /// Change the configuration and write it (enters standby).
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program the EVENT limits in 0.01 °C (0.0625 °C steps). EVENT
            /// asserts above `high` or at/below `low` and stays latched until
            /// [`Self::acknowledge_alert`].
            pub $($async)? fn set_limits(&mut self, low: i32, high: i32) -> Result<(), Error<I2C::Error>> {
                let [lh, ll] = encode_limit(low);
                let [hh, hl] = encode_limit(high);
                self.write_reg(REG_HIGH_LIMIT_HIGH, hh) $($await)*?;
                self.write_reg(REG_HIGH_LIMIT_HIGH + 1, hl) $($await)*?;
                self.write_reg(REG_LOW_LIMIT_HIGH, lh) $($await)*?;
                self.write_reg(REG_LOW_LIMIT_HIGH + 1, ll) $($await)*
            }

            /// Program the Addr/THERM comparator in 0.01 °C (whole degrees):
            /// asserts above `limit`, releases below `limit - hysteresis`.
            pub $($async)? fn set_therm(&mut self, limit: i32, hysteresis: i32) -> Result<(), Error<I2C::Error>> {
                self.write_reg(REG_THERM_LIMIT, encode_therm(limit)) $($await)*?;
                self.write_reg(REG_THERM_HYST, encode_therm(hysteresis)) $($await)*
            }

            /// Read the status register (clears THIGH / TLOW if the
            /// condition has gone).
            pub $($async)? fn read_status(&mut self) -> Result<Status, Error<I2C::Error>> {
                Ok(Status::from_bits(self.read_reg(REG_STATUS) $($await)*?))
            }

            /// SMBus Alert Response: releases a latched EVENT. Returns the
            /// 7-bit address of the device that answered (may be another
            /// device on the bus).
            pub $($async)? fn acknowledge_alert(&mut self) -> Result<u8, Error<I2C::Error>> {
                let mut buf = [0u8; 1];
                self.i2c
                    .read(ALERT_RESPONSE_ADDRESS, &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                Ok(buf[0] >> 1)
            }
        }

        impl<I2C: $i2c> $sensor for Stts751<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot conversion. Writes to the one-shot register are
            /// ignored unless in standby, so standby is entered first if
            /// needed (after power-up or `wake`).
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if !self.in_standby {
                    self.write_config(true) $($await)*?;
                }
                self.write_reg(REG_ONE_SHOT, 0x00) $($await)*?;
                Ok(self.config.resolution.conversion_us())
            }

            /// Reads the high then the low temperature byte (two
            /// transfers; registers 0x00 and 0x02 are not adjacent).
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let high = self.read_reg(REG_TEMP_HIGH) $($await)*?;
                let low = self.read_reg(REG_TEMP_LOW) $($await)*?;
                match decode_temperature(high, low) {
                    Some(temperature) => Ok(Measurement { temperature }),
                    None => Err(Error::InvalidData),
                }
            }
        }

        impl<I2C: $i2c> $low_power for Stts751<I2C> {
            type BusError = I2C::Error;

            /// Standby (RUN/STOP = 1).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_config(true) $($await)*
            }

            /// Continuous conversion at the configured rate. Returns the
            /// wait until the first result.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_config(false) $($await)*?;
                Ok(self.config.resolution.conversion_us())
            }
        }
    };
}

/// Blocking driver.
pub struct Stts751<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
    in_standby: bool,
}

impl<I2C> Stts751<I2C> {
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

    /// Async twin of [`super::Stts751`].
    pub struct Stts751<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
        in_standby: bool,
    }

    impl<I2C> Stts751<I2C> {
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

    const A: u8 = 0x48;

    #[test]
    fn decodes_datasheet_examples() {
        // Table 13.
        assert_eq!(decode_temperature(0xC0, 0x00), Some(-6_400));
        assert_eq!(decode_temperature(0xC1, 0x00), Some(-6_300));
        assert_eq!(decode_temperature(0xFF, 0x00), Some(-100));
        assert_eq!(decode_temperature(0x01, 0x00), Some(100));
        assert_eq!(decode_temperature(0x05, 0x50), Some(531)); // 5.3125
        assert_eq!(decode_temperature(0x7D, 0x00), Some(12_500));
        assert_eq!(decode_temperature(0xFF, 0xFF), None);
    }

    #[test]
    fn encodes_limits_and_config() {
        assert_eq!(encode_limit(8_500), [0x55, 0x00]);
        assert_eq!(encode_limit(531), [0x05, 0x50]);
        assert_eq!(encode_limit(-10_000), [0xC0, 0x00]);
        assert_eq!(encode_limit(20_000), [0x7F, 0xF0]);
        assert_eq!(encode_therm(8_500), 0x55);
        assert_eq!(encode_therm(1_000), 0x0A);
        assert_eq!(encode_therm(-200), 0xFE);
        assert_eq!(Config::default().register(false), 0x00);
        let c = Config {
            resolution: Resolution::Bits9,
            event_enabled: false,
            ..Config::default()
        };
        assert_eq!(c.register(true), 0xC8);
        assert_eq!(Resolution::Bits12.conversion_us(), 112_000);
        assert_eq!(Product::Stts751_1.address(3), 0x3B);
        assert!(
            !Config {
                conversion_rate: ConversionRate::Hz32,
                resolution: Resolution::Bits11,
                ..Config::default()
            }
            .is_valid()
        );
    }

    #[test]
    fn probe_configure_one_shot() {
        let expectations = [
            Transaction::write_read(A, vec![REG_MANUFACTURER_ID], vec![0x53]),
            Transaction::write_read(A, vec![REG_PRODUCT_ID], vec![0x01]),
            Transaction::write_read(A, vec![REG_MANUFACTURER_ID], vec![0x54]),
            Transaction::write_read(A, vec![REG_PRODUCT_ID], vec![0x00]),
            Transaction::write(A, vec![REG_CONFIG, 0x4C]),
            Transaction::write(A, vec![REG_CONV_RATE, 0x00]),
            Transaction::write(A, vec![REG_SMBUS_TIMEOUT, 0x00]),
            Transaction::write(A, vec![REG_ONE_SHOT, 0x00]),
            Transaction::write_read(A, vec![REG_STATUS], vec![0x41]),
            Transaction::write_read(A, vec![REG_TEMP_HIGH], vec![0x19]),
            Transaction::write_read(A, vec![REG_TEMP_LOW], vec![0x40]),
        ];
        let config = Config {
            resolution: Resolution::Bits12,
            conversion_rate: ConversionRate::Hz0_0625,
            event_enabled: true,
            smbus_timeout: false,
        };
        let mut i2c = Mock::new(&expectations);
        let mut t = Stts751::new(&mut i2c, A).with_config(config);
        assert_eq!(t.probe(), Ok(Product::Stts751_1));
        assert_eq!(t.probe(), Err(Error::UnexpectedId(0x5400)));
        t.configure().unwrap();
        assert_eq!(t.start_measurement().unwrap(), 112_000);
        assert_eq!(
            t.read_status().unwrap(),
            Status {
                busy: false,
                high: true,
                low: false,
                therm: true
            }
        );
        assert_eq!(t.read_measurement().unwrap().temperature, 2_525);
        i2c.done();
    }

    #[test]
    fn limits_alert_and_power_modes() {
        let expectations = [
            Transaction::write(A, vec![0x05, 0x1E]),
            Transaction::write(A, vec![0x06, 0x00]),
            Transaction::write(A, vec![0x07, 0x00]),
            Transaction::write(A, vec![0x08, 0x00]),
            Transaction::write(A, vec![REG_THERM_LIMIT, 0x46]),
            Transaction::write(A, vec![REG_THERM_HYST, 0x05]),
            Transaction::read(ALERT_RESPONSE_ADDRESS, vec![0x90]),
            Transaction::write(A, vec![REG_CONFIG, 0x00]),
            // One-shot after wake re-enters standby first.
            Transaction::write(A, vec![REG_CONFIG, 0x40]),
            Transaction::write(A, vec![REG_ONE_SHOT, 0x00]),
            Transaction::write(A, vec![REG_CONFIG, 0x40]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = Stts751::new(&mut i2c, A);
        t.set_limits(0, 3_000).unwrap();
        t.set_therm(7_000, 500).unwrap();
        assert_eq!(t.acknowledge_alert().unwrap(), 0x48);
        assert_eq!(t.wake().unwrap(), 28_000);
        assert_eq!(t.start_measurement().unwrap(), 28_000);
        t.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn invalid_rate_is_rejected() {
        let mut i2c = Mock::new(&[]);
        let config = Config {
            conversion_rate: ConversionRate::Hz16,
            resolution: Resolution::Bits12,
            ..Config::default()
        };
        let mut t = Stts751::new(&mut i2c, A);
        assert_eq!(t.set_config(config), Err(Error::Unsupported));
        i2c.done();
    }

    #[test]
    fn async_one_shot() {
        use zigbee_sensor::asynch::Sensor as _;
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x40]),
            Transaction::write(A, vec![REG_ONE_SHOT, 0x00]),
            Transaction::write_read(A, vec![REG_TEMP_HIGH], vec![0xE7]),
            Transaction::write_read(A, vec![REG_TEMP_LOW], vec![0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = asynch::Stts751::new(&mut i2c, A);
        assert_eq!(block_on(t.start_measurement()).unwrap(), 28_000);
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
