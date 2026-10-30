//! TI OPT3001 / OPT3004 / OPT3006 ambient light sensors (I2C).
//!
//! The three parts share the register map, the I2C addresses and the
//! device ID (0x3001); they differ in package and spectral filter only.
//!
//! Every sample is a single-shot conversion (`M = 01`) with the automatic
//! full-scale range (`RN = 1100`): the part measures once and shuts down by
//! itself (0.3 µA typ.). The result register is a 4-bit exponent and a
//! 12-bit mantissa: `lux = 0.01 × 2^E × R`.
//!
//! References: TI "OPT3001 Ambient Light Sensor (ALS)" SBOS681C (Nov 2017),
//! "OPT3004" SBOS929A (Dec 2021), "OPT3006" SBOS698 (Oct 2016).

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with ADDR to GND.
pub const ADDRESS_GND: u8 = 0x44;
/// I2C address with ADDR to VDD.
pub const ADDRESS_VDD: u8 = 0x45;
/// I2C address with ADDR to SDA.
pub const ADDRESS_SDA: u8 = 0x46;
/// I2C address with ADDR to SCL.
pub const ADDRESS_SCL: u8 = 0x47;

/// Manufacturer ID register value ("TI").
pub const MANUFACTURER_ID: u16 = 0x5449;
/// Device ID register value (all three parts).
pub const DEVICE_ID: u16 = 0x3001;

const REG_RESULT: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;
const REG_LOW_LIMIT: u8 = 0x02;
const REG_HIGH_LIMIT: u8 = 0x03;
const REG_MANUFACTURER_ID: u8 = 0x7E;
const REG_DEVICE_ID: u8 = 0x7F;

const CONFIG_CT: u16 = 1 << 11;
const CONFIG_MODE_SINGLE: u16 = 0b01 << 9;
const CONFIG_OVF: u16 = 1 << 8;
const CONFIG_CRF: u16 = 1 << 7;
const CONFIG_FH: u16 = 1 << 6;
const CONFIG_FL: u16 = 1 << 5;
const CONFIG_LATCH: u16 = 1 << 4;
const CONFIG_POL: u16 = 1 << 3;

/// Low-limit value that selects end-of-conversion interrupt mode
/// (`LE[3:2] = 11`).
pub const END_OF_CONVERSION_LOW_LIMIT: u16 = 0xC000;

/// Integration time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConversionTime {
    /// 100 ms (110 ms max).
    Ms100,
    /// 800 ms (880 ms max), full resolution.
    #[default]
    Ms800,
}

impl ConversionTime {
    /// Worst-case single-shot time in microseconds: integration (max) +
    /// 3 ms + the 10 ms range-assessment measurement (+ 3 ms) that
    /// auto-range performs first ("Automatic Full-Scale Setting Mode").
    pub const fn wait_us(self) -> WaitUs {
        match self {
            Self::Ms100 => 110_000 + 16_000,
            Self::Ms800 => 880_000 + 16_000,
        }
    }
}

/// Full-scale range: automatic or one of the fixed ranges (RN 0..=11,
/// full scale 40.95 lx × 2^RN).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Range {
    /// Automatic full-scale selection (`RN = 1100`).
    #[default]
    Auto,
    /// Fixed range; values above 11 are clamped.
    Fixed(u8),
}

impl Range {
    const fn bits(self) -> u16 {
        match self {
            Self::Auto => 0b1100,
            Self::Fixed(rn) => (if rn > 11 { 11 } else { rn }) as u16,
        }
    }
}

/// Number of consecutive out-of-window results before the flags/INT fire.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FaultCount {
    /// One fault.
    #[default]
    One,
    /// Two consecutive faults.
    Two,
    /// Four consecutive faults.
    Four,
    /// Eight consecutive faults.
    Eight,
}

/// Interrupt-related configuration bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterruptConfig {
    /// Latched window comparison (`true`, default) or transparent
    /// hysteresis (`false`).
    pub latched: bool,
    /// INT active high instead of active low.
    pub active_high: bool,
    /// Fault count.
    pub fault_count: FaultCount,
}

impl Default for InterruptConfig {
    fn default() -> Self {
        Self {
            latched: true,
            active_high: false,
            fault_count: FaultCount::One,
        }
    }
}

/// Configuration word for one single-shot conversion (or shutdown when
/// `start` is false).
pub const fn config_word(
    conversion_time: ConversionTime,
    range: Range,
    interrupt: InterruptConfig,
    start: bool,
) -> u16 {
    let mut word = range.bits() << 12;
    if matches!(conversion_time, ConversionTime::Ms800) {
        word |= CONFIG_CT;
    }
    if start {
        word |= CONFIG_MODE_SINGLE;
    }
    if interrupt.latched {
        word |= CONFIG_LATCH;
    }
    if interrupt.active_high {
        word |= CONFIG_POL;
    }
    word | interrupt.fault_count as u16
}

/// Decode a result (or limit) register to milli-lux:
/// `lux = 0.01 × 2^E × R` (datasheet "Result Register").
pub const fn decode_millilux(raw: u16) -> u32 {
    let exponent = raw >> 12;
    // E ≤ 11 on a real device; clamp so a corrupt value cannot overflow.
    let exponent = if exponent > 11 { 11 } else { exponent };
    (((raw & 0x0FFF) as u32) << exponent) * 10
}

/// Encode milli-lux as a limit register value with the smallest exponent
/// that keeps the 12-bit mantissa in range (rounded down).
pub const fn encode_millilux(millilux: u32) -> u16 {
    let mut centi = millilux / 10;
    let mut exponent = 0u16;
    while centi > 0x0FFF && exponent < 11 {
        centi >>= 1;
        exponent += 1;
    }
    if centi > 0x0FFF {
        centi = 0x0FFF;
    }
    (exponent << 12) | centi as u16
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// Raw result register (exponent + mantissa).
    pub raw: u16,
    /// Light above the device's full scale (OVF).
    pub overflow: bool,
    /// High-limit flag (FH).
    pub flag_high: bool,
    /// Low-limit flag (FL).
    pub flag_low: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// OPT3001 / OPT3004 / OPT3006 driver.
        pub struct Opt3001<I2C> {
            i2c: I2C,
            address: u8,
            conversion_time: ConversionTime,
            range: Range,
            interrupt: InterruptConfig,
        }

        impl<I2C> Opt3001<I2C> {
            /// Create a driver; no bus traffic.
            pub fn new(i2c: I2C, address: u8) -> Self {
                Self {
                    i2c,
                    address,
                    conversion_time: ConversionTime::Ms800,
                    range: Range::Auto,
                    interrupt: InterruptConfig::default(),
                }
            }

            /// Select the conversion time.
            pub const fn with_conversion_time(mut self, conversion_time: ConversionTime) -> Self {
                self.conversion_time = conversion_time;
                self
            }

            /// Select the full-scale range (default automatic).
            pub const fn with_range(mut self, range: Range) -> Self {
                self.range = range;
                self
            }

            /// Interrupt reporting configuration (latch, polarity, fault count).
            pub const fn with_interrupt_config(mut self, interrupt: InterruptConfig) -> Self {
                self.interrupt = interrupt;
                self
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> Opt3001<I2C> {
            $($async_)? fn read_reg(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(self.address, &[reg], &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                Ok(u16::from_be_bytes(buf))
            }

            $($async_)? fn write_reg(&mut self, reg: u8, value: u16) -> Result<(), Error<I2C::Error>> {
                let [hi, lo] = value.to_be_bytes();
                self.i2c
                    .write(self.address, &[reg, hi, lo])
                    $($await_)*
                    .map_err(Error::bus)
            }

            /// Check the manufacturer (0x5449) and device (0x3001) IDs.
            pub $($async_)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let manufacturer = self.read_reg(REG_MANUFACTURER_ID) $($await_)* ?;
                if manufacturer != MANUFACTURER_ID {
                    return Err(Error::UnexpectedId(u32::from(manufacturer)));
                }
                let device = self.read_reg(REG_DEVICE_ID) $($await_)* ?;
                if device != DEVICE_ID {
                    return Err(Error::UnexpectedId(u32::from(device)));
                }
                Ok(())
            }

            /// Program the comparison window (milli-lux). Takes effect from
            /// the next conversion.
            pub $($async_)? fn set_limits(&mut self, low: u32, high: u32) -> Result<(), Error<I2C::Error>> {
                self.write_reg(REG_LOW_LIMIT, encode_millilux(low)) $($await_)* ?;
                self.write_reg(REG_HIGH_LIMIT, encode_millilux(high)) $($await_)*
            }

            /// Make INT fire at the end of every conversion
            /// (datasheet "End-of-Conversion Mode").
            pub $($async_)? fn set_end_of_conversion_interrupt(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_reg(REG_LOW_LIMIT, END_OF_CONVERSION_LOW_LIMIT) $($await_)*
            }
        }

        impl<I2C: I2c> Sensor for Opt3001<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Write the configuration with `M = 01` (single shot).
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let word = config_word(self.conversion_time, self.range, self.interrupt, true);
                self.write_reg(REG_CONFIG, word) $($await_)* ?;
                Ok(self.conversion_time.wait_us())
            }

            /// [`Error::NotReady`] until the conversion-ready flag is set.
            /// Reading the configuration clears CRF and latched flags.
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let config = self.read_reg(REG_CONFIG) $($await_)* ?;
                if config & CONFIG_CRF == 0 {
                    return Err(Error::NotReady);
                }
                let raw = self.read_reg(REG_RESULT) $($await_)* ?;
                Ok(Measurement {
                    illuminance: decode_millilux(raw),
                    raw,
                    overflow: config & CONFIG_OVF != 0,
                    flag_high: config & CONFIG_FH != 0,
                    flag_low: config & CONFIG_FL != 0,
                })
            }
        }

        impl<I2C: I2c> LowPower for Opt3001<I2C> {
            type BusError = I2C::Error;

            /// Force shutdown (`M = 00`), aborting any conversion.
            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                let word = config_word(self.conversion_time, self.range, self.interrupt, false);
                self.write_reg(REG_CONFIG, word) $($await_)*
            }

            /// Nothing to do: single shots start from shutdown.
            $($async_)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                Ok(0)
            }
        }
    };
}

driver!(,);

/// Async driver.
pub mod asynch {
    use super::{
        CONFIG_CRF, CONFIG_FH, CONFIG_FL, CONFIG_OVF, ConversionTime, DEVICE_ID,
        END_OF_CONVERSION_LOW_LIMIT, InterruptConfig, MANUFACTURER_ID, Measurement, REG_CONFIG,
        REG_DEVICE_ID, REG_HIGH_LIMIT, REG_LOW_LIMIT, REG_MANUFACTURER_ID, REG_RESULT, Range,
        config_word, decode_millilux, encode_millilux,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::asynch::{LowPower, Sensor};
    use zigbee_sensor::{Error, WaitUs};

    driver!(async, .await);
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;

    #[test]
    fn config_words() {
        let irq = InterruptConfig::default();
        // Power-on default is C810 (auto range, 800 ms, shutdown, latched).
        assert_eq!(
            config_word(ConversionTime::Ms800, Range::Auto, irq, false),
            0xC810
        );
        assert_eq!(
            config_word(ConversionTime::Ms100, Range::Auto, irq, true),
            0xC210
        );
        let irq = InterruptConfig {
            latched: false,
            active_high: true,
            fault_count: FaultCount::Eight,
        };
        assert_eq!(
            config_word(ConversionTime::Ms800, Range::Fixed(3), irq, true),
            0x3A0B
        );
    }

    #[test]
    fn datasheet_result_examples() {
        // Datasheet Table 9 examples.
        assert_eq!(decode_millilux(0x0001), 10);
        assert_eq!(decode_millilux(0x0FFF), 40_950);
        assert_eq!(decode_millilux(0x3456), 88_800);
        assert_eq!(decode_millilux(0x789A), 2_818_560);
        assert_eq!(decode_millilux(0x8800), 5_242_880);
        assert_eq!(decode_millilux(0xBFFF), 83_865_600);
        // Corrupt exponent clamps.
        assert_eq!(decode_millilux(0xFFFF), 83_865_600);
    }

    #[test]
    fn limit_encoding_round_trips() {
        assert_eq!(encode_millilux(88_800), 0x28AC);
        assert_eq!(decode_millilux(encode_millilux(88_800)), 88_800);
        assert_eq!(encode_millilux(83_865_600), 0xBFFF);
        assert_eq!(encode_millilux(u32::MAX), 0xBFFF);
        assert_eq!(encode_millilux(0), 0);
    }

    #[test]
    fn single_shot_flow() {
        let expectations = [
            Transaction::write(ADDRESS_GND, vec![0x01, 0xC2, 0x10]),
            Transaction::write_read(ADDRESS_GND, vec![0x01], vec![0xC2, 0x10]),
            Transaction::write_read(ADDRESS_GND, vec![0x01], vec![0xC8, 0xD0]),
            Transaction::write_read(ADDRESS_GND, vec![0x00], vec![0x34, 0x56]),
            Transaction::write(ADDRESS_GND, vec![0x01, 0xC0, 0x10]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut opt =
            Opt3001::new(&mut i2c, ADDRESS_GND).with_conversion_time(ConversionTime::Ms100);
        assert_eq!(opt.start_measurement().unwrap(), 126_000);
        assert_eq!(opt.read_measurement(), Err(Error::NotReady));
        let m = opt.read_measurement().unwrap();
        assert_eq!(m.illuminance, 88_800);
        assert!(m.flag_high && !m.flag_low && !m.overflow);
        opt.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn probe_and_limits() {
        let expectations = [
            Transaction::write_read(ADDRESS_SCL, vec![0x7E], vec![0x54, 0x49]),
            Transaction::write_read(ADDRESS_SCL, vec![0x7F], vec![0x30, 0x01]),
            Transaction::write_read(ADDRESS_SCL, vec![0x7E], vec![0x54, 0x49]),
            Transaction::write_read(ADDRESS_SCL, vec![0x7F], vec![0x30, 0x02]),
            Transaction::write(ADDRESS_SCL, vec![0x02, 0x00, 0x64]),
            Transaction::write(ADDRESS_SCL, vec![0x03, 0x28, 0xAC]),
            Transaction::write(ADDRESS_SCL, vec![0x02, 0xC0, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut opt = Opt3001::new(&mut i2c, ADDRESS_SCL);
        opt.probe().unwrap();
        assert_eq!(opt.probe(), Err(Error::UnexpectedId(0x3002)));
        opt.set_limits(1_000, 88_800).unwrap();
        opt.set_end_of_conversion_interrupt().unwrap();
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
            Transaction::write(ADDRESS_VDD, vec![0x01, 0xCA, 0x10]),
            Transaction::write_read(ADDRESS_VDD, vec![0x01], vec![0xC9, 0x90]),
            Transaction::write_read(ADDRESS_VDD, vec![0x00], vec![0xBF, 0xFF]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut opt = asynch::Opt3001::new(&mut i2c, ADDRESS_VDD);
        let mut delay = RecordingDelay(0);
        let m = block_on(opt.measure(&mut delay)).unwrap();
        assert_eq!(m.illuminance, 83_865_600);
        assert!(m.overflow);
        assert_eq!(delay.0, 896_000);
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
