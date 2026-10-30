//! TI TMP116 / TMP117 / TMP119 high-accuracy digital temperature sensors
//! (I2C / SMBus).
//!
//! [`configure`](Tmp11x::configure) leaves the sensor in shutdown
//! (0.15–0.25 µA typ.); every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! selects one-shot mode (MOD = 11). The part converts (with the configured
//! averaging) and returns to shutdown by itself.
//!
//! References: TI SBOS740A (TMP116, May 2019), SNOSD82D (TMP117, September
//! 2022), SNIS236 (TMP119, January 2024).
//!
//! ```ignore
//! let mut tmp = Tmp11x::new(i2c, ADDRESS_GND);
//! let variant = tmp.probe()?;
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

/// I2C address with ADD0 tied to GND.
pub const ADDRESS_GND: u8 = 0x48;
/// I2C address with ADD0 tied to V+.
pub const ADDRESS_VCC: u8 = 0x49;
/// I2C address with ADD0 tied to SDA.
pub const ADDRESS_SDA: u8 = 0x4A;
/// I2C address with ADD0 tied to SCL.
pub const ADDRESS_SCL: u8 = 0x4B;

/// Worst-case single conversion time (TMP117/TMP119: 17.5 ms max, TMP116:
/// 17 ms max).
pub const SINGLE_CONVERSION_US: WaitUs = 17_500;

/// Wait after a software reset (2 ms, including the EEPROM reload).
pub const RESET_WAIT_US: WaitUs = 2_000;

const REG_TEMPERATURE: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;
const REG_HIGH_LIMIT: u8 = 0x02;
const REG_LOW_LIMIT: u8 = 0x03;
const REG_TEMP_OFFSET: u8 = 0x07;
const REG_DEVICE_ID: u8 = 0x0F;

const CFG_HIGH_ALERT: u16 = 1 << 15;
const CFG_LOW_ALERT: u16 = 1 << 14;
const CFG_DATA_READY: u16 = 1 << 13;
const CFG_EEPROM_BUSY: u16 = 1 << 12;
const CFG_THERM: u16 = 1 << 4;
const CFG_POL: u16 = 1 << 3;
const CFG_DR_ALERT: u16 = 1 << 2;
const CFG_SOFT_RESET: u16 = 1 << 1;

const MOD_CONTINUOUS: u16 = 0b00 << 10;
const MOD_SHUTDOWN: u16 = 0b01 << 10;
const MOD_ONE_SHOT: u16 = 0b11 << 10;

/// Temperature register value before the first conversion (−256 °C).
const RAW_NOT_READY: u16 = 0x8000;

/// Part identified by [`Tmp11x::probe`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// TMP116 (Device ID 0x1116), no offset register.
    Tmp116,
    /// TMP117 (Device ID 0x0117).
    #[default]
    Tmp117,
    /// TMP119 (Device ID 0x2117).
    Tmp119,
}

/// Conversion averaging (AVG). Also applies in one-shot mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Averaging {
    /// Single conversion (15.5 ms typ.).
    None,
    /// 8 conversions (125 ms typ., factory default).
    #[default]
    X8,
    /// 32 conversions (500 ms typ.).
    X32,
    /// 64 conversions (1 s typ.).
    X64,
}

impl Averaging {
    /// Number of averaged conversions.
    pub const fn count(self) -> u32 {
        match self {
            Self::None => 1,
            Self::X8 => 8,
            Self::X32 => 32,
            Self::X64 => 64,
        }
    }
}

/// Continuous-mode conversion cycle (CONV, table 7-7). The cycle is never
/// shorter than the averaging time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConversionCycle {
    /// 15.5 ms.
    Ms15_5,
    /// 125 ms.
    Ms125,
    /// 250 ms.
    Ms250,
    /// 500 ms.
    Ms500,
    /// 1 s (factory default).
    #[default]
    S1,
    /// 4 s.
    S4,
    /// 8 s.
    S8,
    /// 16 s.
    S16,
}

/// ALERT behaviour (T/nA bit).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AlertMode {
    /// Alert mode (default): HIGH/LOW flags latch until the configuration
    /// register is read.
    #[default]
    Alert,
    /// Therm mode: active above the high limit until below the low limit.
    Therm,
}

/// Configuration written by [`Tmp11x::configure`] and every one-shot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Conversions averaged per result.
    pub averaging: Averaging,
    /// Cycle used in continuous mode.
    pub cycle: ConversionCycle,
    /// Alert or therm mode.
    pub alert_mode: AlertMode,
    /// ALERT active high (`POL = 1`) instead of active low.
    pub alert_active_high: bool,
    /// ALERT pin shows Data_Ready instead of the limit flags.
    pub alert_data_ready: bool,
}

impl Config {
    const fn register(self, mode: u16) -> u16 {
        let mut word = ((self.cycle as u16) << 7) | ((self.averaging as u16) << 5) | mode;
        if matches!(self.alert_mode, AlertMode::Therm) {
            word |= CFG_THERM;
        }
        if self.alert_active_high {
            word |= CFG_POL;
        }
        if self.alert_data_ready {
            word |= CFG_DR_ALERT;
        }
        word
    }

    /// Worst-case one-shot time in µs.
    pub const fn one_shot_us(self) -> WaitUs {
        SINGLE_CONVERSION_US * self.averaging.count()
    }
}

/// Status flags from the configuration register (reading clears them).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Flags {
    /// Result above the high limit (HIGH_Alert).
    pub high: bool,
    /// Result below the low limit (LOW_Alert).
    pub low: bool,
    /// A new result is available (Data_Ready).
    pub data_ready: bool,
    /// EEPROM programming or power-up load in progress.
    pub eeprom_busy: bool,
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

/// Decode the temperature register (16-bit two's complement, 7.8125 m°C,
/// table 7-1) to 0.01 °C, rounded. Also used for limits and offset.
pub const fn decode_temperature(raw: u16) -> i32 {
    ((raw as i16 as i32) * 100 + 64) >> 7
}

/// Encode 0.01 °C into the 16-bit register format, saturating.
pub const fn encode_temperature(centi_celsius: i32) -> u16 {
    let counts = (centi_celsius * 128 + 50).div_euclid(100);
    let c = if counts < i16::MIN as i32 {
        i16::MIN as i32
    } else if counts > i16::MAX as i32 {
        i16::MAX as i32
    } else {
        counts
    };
    c as i16 as u16
}

/// Map a Device ID register value to the part.
pub const fn variant_from_id(id: u16) -> Option<Variant> {
    match (id >> 12, id & 0x0FFF) {
        (_, 0x116) => Some(Variant::Tmp116),
        (0 | 1, 0x117) => Some(Variant::Tmp117),
        (_, 0x117) => Some(Variant::Tmp119),
        _ => None,
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver with the factory-default [`Config`]; no bus
        /// traffic. The variant defaults to TMP117 until [`Self::probe`].
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                variant: Variant::Tmp117,
                config: Config {
                    averaging: Averaging::X8,
                    cycle: ConversionCycle::S1,
                    alert_mode: AlertMode::Alert,
                    alert_active_high: false,
                    alert_data_ready: false,
                },
            }
        }

        /// Use `config` for [`Self::configure`] and later mode changes.
        pub const fn with_config(mut self, config: Config) -> Self {
            self.config = config;
            self
        }

        /// Set the variant without probing.
        pub const fn with_variant(mut self, variant: Variant) -> Self {
            self.variant = variant;
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
        impl<I2C: $i2c> Tmp11x<I2C> {
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

            /// Read the Device ID register, identify the part and remember
            /// it. [`Error::UnexpectedId`] for anything else.
            pub $($async)? fn probe(&mut self) -> Result<Variant, Error<I2C::Error>> {
                let id = self.read_register(REG_DEVICE_ID) $($await)*?;
                match variant_from_id(id) {
                    Some(variant) => {
                        self.variant = variant;
                        Ok(variant)
                    }
                    None => Err(Error::UnexpectedId(u32::from(id))),
                }
            }

            /// Software reset: reloads the EEPROM (power-up defaults,
            /// continuous mode). Wait [`RESET_WAIT_US`], then
            /// [`Self::configure`] again.
            pub $($async)? fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(REG_CONFIG, CFG_SOFT_RESET) $($await)*?;
                Ok(RESET_WAIT_US)
            }

            /// Write the configuration and enter shutdown. Call once after
            /// power-up: the factory default is continuous mode.
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(MOD_SHUTDOWN)) $($await)*
            }

            /// Change the configuration and write it (enters shutdown).
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program the limits in 0.01 °C (7.8125 m°C steps). In therm
            /// mode `low` is the release (hysteresis) point.
            pub $($async)? fn set_limits(&mut self, low: i32, high: i32) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_LOW_LIMIT, encode_temperature(low)) $($await)*?;
                self.write_register(REG_HIGH_LIMIT, encode_temperature(high)) $($await)*
            }

            /// Program the temperature offset in 0.01 °C (added to every
            /// result; volatile). TMP117/TMP119 only: [`Error::Unsupported`]
            /// on TMP116, where 0x07 is a user EEPROM register.
            pub $($async)? fn set_offset(&mut self, offset: i32) -> Result<(), Error<I2C::Error>> {
                if matches!(self.variant, Variant::Tmp116) {
                    return Err(Error::Unsupported);
                }
                self.write_register(REG_TEMP_OFFSET, encode_temperature(offset)) $($await)*
            }

            /// Read the configuration register flags. This clears
            /// Data_Ready and, in alert mode, HIGH/LOW and the ALERT pin.
            pub $($async)? fn read_flags(&mut self) -> Result<Flags, Error<I2C::Error>> {
                let config = self.read_register(REG_CONFIG) $($await)*?;
                Ok(Flags {
                    high: config & CFG_HIGH_ALERT != 0,
                    low: config & CFG_LOW_ALERT != 0,
                    data_ready: config & CFG_DATA_READY != 0,
                    eeprom_busy: config & CFG_EEPROM_BUSY != 0,
                })
            }
        }

        impl<I2C: $i2c> $sensor for Tmp11x<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot conversion (MOD = 11) with the configured averaging.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(MOD_ONE_SHOT)) $($await)*?;
                Ok(self.config.one_shot_us())
            }

            /// [`Error::NotReady`] while the register still holds the
            /// power-up value (−256 °C).
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let raw = self.read_register(REG_TEMPERATURE) $($await)*?;
                if raw == RAW_NOT_READY {
                    return Err(Error::NotReady);
                }
                Ok(Measurement { temperature: decode_temperature(raw) })
            }
        }

        impl<I2C: $i2c> $low_power for Tmp11x<I2C> {
            type BusError = I2C::Error;

            /// Shutdown (MOD = 01).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.configure() $($await)*
            }

            /// Continuous conversion. Returns the wait until the first
            /// (averaged) result.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(MOD_CONTINUOUS)) $($await)*?;
                Ok(self.config.one_shot_us())
            }
        }
    };
}

/// Blocking driver.
pub struct Tmp11x<I2C> {
    i2c: I2C,
    address: u8,
    variant: Variant,
    config: Config,
}

impl<I2C> Tmp11x<I2C> {
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

    /// Async twin of [`super::Tmp11x`].
    pub struct Tmp11x<I2C> {
        i2c: I2C,
        address: u8,
        variant: Variant,
        config: Config,
    }

    impl<I2C> Tmp11x<I2C> {
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
        // SNOSD82D table 7-1.
        assert_eq!(decode_temperature(0xF380), -2_500);
        assert_eq!(decode_temperature(0xFFF0), -12);
        assert_eq!(decode_temperature(0xFFFF), -1);
        assert_eq!(decode_temperature(0x0000), 0);
        assert_eq!(decode_temperature(0x0001), 1);
        assert_eq!(decode_temperature(0x0080), 100);
        assert_eq!(decode_temperature(0x0C80), 2_500);
        assert_eq!(decode_temperature(0x3200), 10_000);
        assert_eq!(decode_temperature(0x7FFF), 25_599);
    }

    #[test]
    fn encoding_and_ids() {
        // Factory limits: high 6000h (192 °C), low 8000h (-256 °C).
        assert_eq!(encode_temperature(19_200), 0x6000);
        assert_eq!(encode_temperature(-25_600), 0x8000);
        assert_eq!(encode_temperature(-2_500), 0xF380);
        assert_eq!(encode_temperature(30_000), 0x7FFF);
        assert_eq!(variant_from_id(0x1116), Some(Variant::Tmp116));
        assert_eq!(variant_from_id(0x0117), Some(Variant::Tmp117));
        assert_eq!(variant_from_id(0x2117), Some(Variant::Tmp119));
        assert_eq!(variant_from_id(0x0075), None);
    }

    #[test]
    fn config_words() {
        // Factory default 0220h = continuous, 1 s, 8 averages.
        assert_eq!(Config::default().register(MOD_CONTINUOUS), 0x0220);
        let c = Config {
            averaging: Averaging::None,
            cycle: ConversionCycle::S16,
            alert_mode: AlertMode::Therm,
            alert_active_high: true,
            alert_data_ready: true,
        };
        assert_eq!(c.register(MOD_ONE_SHOT), 0x0F9C);
        assert_eq!(c.one_shot_us(), 17_500);
        assert_eq!(Config::default().one_shot_us(), 140_000);
    }

    #[test]
    fn one_shot_then_read() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x06, 0x20]),
            Transaction::write(A, vec![REG_CONFIG, 0x0E, 0x20]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x80, 0x00]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x0C, 0x80]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp11x::new(&mut i2c, A);
        tmp.configure().unwrap();
        assert_eq!(tmp.start_measurement().unwrap(), 140_000);
        assert_eq!(tmp.read_measurement(), Err(Error::NotReady));
        assert_eq!(tmp.read_measurement().unwrap().temperature, 2_500);
        i2c.done();
    }

    #[test]
    fn probe_limits_offset_flags() {
        let expectations = [
            Transaction::write_read(A, vec![REG_DEVICE_ID], vec![0x75, 0x00]),
            Transaction::write_read(A, vec![REG_DEVICE_ID], vec![0x21, 0x17]),
            Transaction::write(A, vec![REG_LOW_LIMIT, 0x0C, 0x80]),
            Transaction::write(A, vec![REG_HIGH_LIMIT, 0x0F, 0x00]),
            Transaction::write(A, vec![REG_TEMP_OFFSET, 0xFF, 0xC0]),
            Transaction::write_read(A, vec![REG_CONFIG], vec![0xA6, 0x20]),
            Transaction::write_read(A, vec![REG_DEVICE_ID], vec![0x11, 0x16]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp11x::new(&mut i2c, A);
        assert_eq!(tmp.probe(), Err(Error::UnexpectedId(0x7500)));
        assert_eq!(tmp.probe(), Ok(Variant::Tmp119));
        tmp.set_limits(2_500, 3_000).unwrap();
        tmp.set_offset(-50).unwrap();
        let flags = tmp.read_flags().unwrap();
        assert!(flags.high && flags.data_ready && !flags.low && !flags.eeprom_busy);
        assert_eq!(tmp.probe(), Ok(Variant::Tmp116));
        assert_eq!(tmp.set_offset(100), Err(Error::Unsupported));
        i2c.done();
    }

    #[test]
    fn reset_sleep_wake() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x00, 0x02]),
            Transaction::write(A, vec![REG_CONFIG, 0x06, 0x20]),
            Transaction::write(A, vec![REG_CONFIG, 0x02, 0x20]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp11x::new(&mut i2c, A);
        assert_eq!(tmp.soft_reset().unwrap(), RESET_WAIT_US);
        tmp.sleep().unwrap();
        assert_eq!(tmp.wake().unwrap(), 140_000);
        i2c.done();
    }

    #[test]
    fn async_one_shot_no_averaging() {
        use zigbee_sensor::asynch::Sensor as _;
        let config = Config {
            averaging: Averaging::None,
            ..Config::default()
        };
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x0E, 0x00]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0xF3, 0x80]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = asynch::Tmp11x::new(&mut i2c, A).with_config(config);
        assert_eq!(block_on(tmp.start_measurement()).unwrap(), 17_500);
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
