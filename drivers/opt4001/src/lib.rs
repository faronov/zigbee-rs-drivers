//! TI OPT4001 and OPT4003-Q1 high-precision ambient light sensors (I2C).
//!
//! The output is semi-logarithmic: a 4-bit exponent and a 20-bit mantissa,
//! linearised as `ADC_CODES = MANTISSA << E` (28 bits) and scaled per
//! package: 312.5 µlx/code (OPT4001 PicoStar), 437.5 µlx/code (OPT4001
//! SOT-5X3) and 535 µlx/code (OPT4003-Q1 visible channel). The OPT4003-Q1
//! also converts a near-IR channel (CH1), reported raw.
//!
//! Each sample uses the *forced auto-range one-shot* mode
//! (`OPERATING_MODE = 1`): the part leaves standby, picks the range,
//! converts and returns to standby (1.6 µA typ.) by itself. Every result
//! carries a 4-bit CRC that the driver checks.
//!
//! References: TI "OPT4001 High Speed, High Precision, Digital Ambient Light
//! Sensor" SBOS993A (Dec 2022); "OPT4003-Q1 High-Speed, Dual-Channel,
//! High-Precision, Digital Ambient Light Sensor" SBASA69E (Jan 2026).

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with ADDR to GND (SOT-5X3 / USON).
pub const ADDRESS_GND: u8 = 0x44;
/// I2C address with ADDR to VDD; the only address of the PicoStar variant.
pub const ADDRESS_VDD: u8 = 0x45;
/// I2C address with ADDR to SDA.
pub const ADDRESS_SDA: u8 = 0x46;
/// I2C address with ADDR to SCL (the OPT4001 table misprints it as
/// 1000101b; OPT4003-Q1 Table 6-7 gives 1000111b).
pub const ADDRESS_SCL: u8 = 0x47;

const REG_RESULT: u8 = 0x00;
const REG_THRESHOLD_L: u8 = 0x08;
const REG_THRESHOLD_H: u8 = 0x09;
const REG_CONFIG: u8 = 0x0A;
const REG_FLAGS: u8 = 0x0C;
const REG_DEVICE_ID: u8 = 0x11;

const FLAG_OVERLOAD: u16 = 1 << 3;
const FLAG_READY: u16 = 1 << 2;
const FLAG_HIGH: u16 = 1 << 1;
const FLAG_LOW: u16 = 1 << 0;

/// Supported part / package.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// OPT4001 PicoStar (YMN): 312.5 µlx per code, fixed address 0x45.
    Opt4001PicoStar,
    /// OPT4001 SOT-5X3 (DTS): 437.5 µlx per code.
    Opt4001Sot5x3,
    /// OPT4003-Q1 USON: 535 µlx per code, plus a near-IR channel.
    Opt4003,
}

impl Variant {
    /// `DIDH` value of register 0x11.
    pub const fn device_id(self) -> u16 {
        match self {
            Self::Opt4001PicoStar | Self::Opt4001Sot5x3 => 0x121,
            Self::Opt4003 => 0x221,
        }
    }

    /// Convert linear `ADC_CODES` to milli-lux (equations 5/6).
    pub const fn codes_to_millilux(self, codes: u32) -> u32 {
        match self {
            // 0.3125 mlx = 5/16, 0.4375 mlx = 7/16; codes < 2^28 so ×7 fits.
            Self::Opt4001PicoStar => codes * 5 / 16,
            Self::Opt4001Sot5x3 => codes * 7 / 16,
            // 0.535 mlx = 107/200, split to stay in u32.
            Self::Opt4003 => codes / 200 * 107 + codes % 200 * 107 / 200,
        }
    }

    /// Convert milli-lux to linear `ADC_CODES` (inverse of
    /// [`Self::codes_to_millilux`], saturating).
    pub const fn millilux_to_codes(self, millilux: u32) -> u32 {
        let codes = match self {
            Self::Opt4001PicoStar => millilux as u64 * 16 / 5,
            Self::Opt4001Sot5x3 => millilux as u64 * 16 / 7,
            Self::Opt4003 => millilux as u64 * 200 / 107,
        };
        if codes > u32::MAX as u64 {
            u32::MAX
        } else {
            codes as u32
        }
    }

    const fn channels(self) -> u32 {
        match self {
            Self::Opt4003 => 2,
            _ => 1,
        }
    }
}

/// Conversion time per channel (`CONVERSION_TIME` field).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum ConversionTime {
    Us600,
    Ms1,
    Ms1_8,
    Ms3_4,
    Ms6_5,
    Ms12_7,
    Ms25,
    Ms50,
    #[default]
    Ms100,
    Ms200,
    Ms400,
    Ms800,
}

impl ConversionTime {
    /// Typical conversion time in microseconds (Table 8-4).
    pub const fn typical_us(self) -> u32 {
        match self {
            Self::Us600 => 600,
            Self::Ms1 => 1_000,
            Self::Ms1_8 => 1_800,
            Self::Ms3_4 => 3_400,
            Self::Ms6_5 => 6_500,
            Self::Ms12_7 => 12_700,
            Self::Ms25 => 25_000,
            Self::Ms50 => 50_000,
            Self::Ms100 => 100_000,
            Self::Ms200 => 200_000,
            Self::Ms400 => 400_000,
            Self::Ms800 => 800_000,
        }
    }
}

/// Number of consecutive faults before FLAG_H / FLAG_L are set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum FaultCount {
    #[default]
    One,
    Two,
    Four,
    Eight,
}

/// Configuration register (0x0A) fields other than the operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Conversion time per channel.
    pub conversion_time: ConversionTime,
    /// `None` = automatic range (12), `Some(0..=8)` = fixed range.
    pub range: Option<u8>,
    /// Quick wake-up: faster start, higher standby current.
    pub quick_wake: bool,
    /// Latched (true, default) or transparent hysteresis threshold flags.
    pub latch: bool,
    /// INT active high.
    pub int_active_high: bool,
    /// Fault count.
    pub fault_count: FaultCount,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            conversion_time: ConversionTime::Ms100,
            range: None,
            quick_wake: false,
            latch: true,
            int_active_high: false,
            fault_count: FaultCount::One,
        }
    }
}

impl Config {
    /// Register 0x0A value with `OPERATING_MODE = mode` (0 power-down,
    /// 1 forced auto-range one-shot).
    pub const fn word(&self, mode: u16) -> u16 {
        let range = match self.range {
            None => 12,
            Some(r) => (if r > 8 { 8 } else { r }) as u16,
        };
        (if self.quick_wake { 1 << 15 } else { 0 })
            | range << 10
            | (self.conversion_time as u16) << 6
            | (mode & 0b11) << 4
            | (if self.latch { 1 << 3 } else { 0 })
            | (if self.int_active_high { 1 << 2 } else { 0 })
            | self.fault_count as u16
    }
}

/// 4-bit CRC of an output word pair (register 0x01 description):
/// `X0` = parity of all bits, `X1` = odd bits, `X2` = every 4th bit from 3,
/// `X3` = R3, R11, R19, over the word `E[3:0] R[19:0] C[3:0]`.
pub const fn crc4(exponent: u8, mantissa: u32, counter: u8) -> u8 {
    let w = (exponent as u32 & 0xF) << 24 | (mantissa & 0xF_FFFF) << 4 | (counter as u32 & 0xF);
    let x0 = (w & 0x0FFF_FFFF).count_ones() & 1;
    let x1 = (w & 0x0AAA_AAAA).count_ones() & 1;
    let x2 = (w & 0x0888_8888).count_ones() & 1;
    let x3 = (w & 0x0080_8080).count_ones() & 1;
    (x3 << 3 | x2 << 2 | x1 << 1 | x0) as u8
}

/// A decoded channel result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelResult {
    /// Linear `ADC_CODES` (`MANTISSA << EXPONENT`).
    pub codes: u32,
    /// Sample counter (rolls over at 16).
    pub counter: u8,
}

/// Decode and CRC-check the two registers of one channel (4 bytes, MSB
/// first).
pub fn decode_channel<E>(bytes: &[u8]) -> Result<ChannelResult, Error<E>> {
    let exponent = bytes[0] >> 4;
    let mantissa =
        u32::from(bytes[0] & 0x0F) << 16 | u32::from(bytes[1]) << 8 | u32::from(bytes[2]);
    let counter = bytes[3] >> 4;
    if crc4(exponent, mantissa, counter) != bytes[3] & 0x0F {
        return Err(Error::Crc);
    }
    if exponent > 8 {
        return Err(Error::InvalidData);
    }
    Ok(ChannelResult {
        codes: mantissa << exponent,
        counter,
    })
}

/// Encode linear codes into a threshold register (`R[11:0] << (8 + E)`),
/// rounding down, saturating at `E = 15`.
pub const fn encode_threshold(codes: u32) -> u16 {
    let mut value = codes >> 8;
    let mut exponent = 0u16;
    while value > 0x0FFF && exponent < 15 {
        value >>= 1;
        exponent += 1;
    }
    if value > 0x0FFF {
        value = 0x0FFF;
    }
    exponent << 12 | value as u16
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// Visible channel linear ADC codes.
    pub codes: u32,
    /// Near-IR channel linear ADC codes (OPT4003-Q1 only).
    pub ir_codes: Option<u32>,
    /// Sample counter of the visible channel.
    pub counter: u8,
    /// Light exceeded the full-scale range.
    pub overload: bool,
    /// Threshold-high flag.
    pub flag_high: bool,
    /// Threshold-low flag.
    pub flag_low: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// OPT4001 / OPT4003-Q1 driver.
        pub struct Opt4001<I2C> {
            i2c: I2C,
            address: u8,
            variant: Variant,
            config: Config,
        }

        impl<I2C> Opt4001<I2C> {
            /// Create a driver; no bus traffic.
            pub fn new(i2c: I2C, address: u8, variant: Variant) -> Self {
                Self {
                    i2c,
                    address,
                    variant,
                    config: Config::default(),
                }
            }

            /// Set the configuration used by every conversion.
            pub const fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self
            }

            /// Selected variant.
            pub const fn variant(&self) -> Variant {
                self.variant
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }

            /// Wait for one conversion: typical time per channel + 10 %,
            /// plus 0.5 ms standby recovery (Tss) and 0.5 ms auto-range
            /// reset.
            pub const fn wait_us(&self) -> WaitUs {
                self.config.conversion_time.typical_us() * self.variant.channels() * 11 / 10 + 1_000
            }
        }

        impl<I2C: I2c> Opt4001<I2C> {
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

            /// Check the device ID against the selected variant.
            pub $($async_)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let id = self.read_reg(REG_DEVICE_ID) $($await_)* ? & 0x0FFF;
                if id != self.variant.device_id() {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(())
            }

            /// Program the threshold window in milli-lux (visible channel).
            pub $($async_)? fn set_thresholds(&mut self, low: u32, high: u32) -> Result<(), Error<I2C::Error>> {
                let low = encode_threshold(self.variant.millilux_to_codes(low));
                let high = encode_threshold(self.variant.millilux_to_codes(high));
                self.write_reg(REG_THRESHOLD_L, low) $($await_)* ?;
                self.write_reg(REG_THRESHOLD_H, high) $($await_)*
            }
        }

        impl<I2C: I2c> Sensor for Opt4001<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Trigger a forced auto-range one-shot conversion.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_reg(REG_CONFIG, self.config.word(1)) $($await_)* ?;
                Ok(self.wait_us())
            }

            /// [`Error::NotReady`] until `CONVERSION_READY_FLAG` is set;
            /// [`Error::Crc`] if a result fails its CRC.
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let flags = self.read_reg(REG_FLAGS) $($await_)* ?;
                if flags & FLAG_READY == 0 {
                    return Err(Error::NotReady);
                }
                let mut buf = [0u8; 8];
                let len = 4 * self.variant.channels() as usize;
                // I2C_BURST (default on) auto-increments the register pointer.
                self.i2c
                    .write_read(self.address, &[REG_RESULT], &mut buf[..len])
                    $($await_)*
                    .map_err(Error::bus)?;
                let ch0 = decode_channel(&buf[..4])?;
                let ir_codes = if len == 8 {
                    Some(decode_channel(&buf[4..])?.codes)
                } else {
                    None
                };
                Ok(Measurement {
                    illuminance: self.variant.codes_to_millilux(ch0.codes),
                    codes: ch0.codes,
                    ir_codes,
                    counter: ch0.counter,
                    overload: flags & FLAG_OVERLOAD != 0,
                    flag_high: flags & FLAG_HIGH != 0,
                    flag_low: flags & FLAG_LOW != 0,
                })
            }
        }

        impl<I2C: I2c> LowPower for Opt4001<I2C> {
            type BusError = I2C::Error;

            /// `OPERATING_MODE = 0` (power-down / standby).
            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_reg(REG_CONFIG, self.config.word(0)) $($await_)*
            }

            /// Nothing to do: one-shot triggers wake the part.
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
        Config, FLAG_HIGH, FLAG_LOW, FLAG_OVERLOAD, FLAG_READY, Measurement, REG_CONFIG,
        REG_DEVICE_ID, REG_FLAGS, REG_RESULT, REG_THRESHOLD_H, REG_THRESHOLD_L, Variant,
        decode_channel, encode_threshold,
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

    fn channel(exponent: u8, mantissa: u32, counter: u8) -> [u8; 4] {
        [
            exponent << 4 | (mantissa >> 16) as u8,
            (mantissa >> 8) as u8,
            mantissa as u8,
            counter << 4 | crc4(exponent, mantissa, counter),
        ]
    }

    #[test]
    fn crc_formula() {
        // Hand-evaluated from the register 0x01 equations.
        assert_eq!(crc4(0, 0, 0), 0b0000);
        assert_eq!(crc4(1, 0, 0), 0b0001); // E0: X0 only
        assert_eq!(crc4(8, 0, 0), 0b0111); // E3: X0, X1, X2
        assert_eq!(crc4(0, 0x8, 0), 0b1111); // R3: all four
        assert_eq!(crc4(0, 0x3, 0), 0b0010); // R0 ^ R1: X0 cancels, X1 = R1
        assert_eq!(crc4(0, 0, 0x8), 0b0111); // C3
    }

    #[test]
    fn lux_scaling() {
        // 0.3125 / 0.4375 / 0.535 mlx per code.
        assert_eq!(Variant::Opt4001PicoStar.codes_to_millilux(16), 5);
        assert_eq!(Variant::Opt4001Sot5x3.codes_to_millilux(1_600), 700);
        assert_eq!(Variant::Opt4003.codes_to_millilux(200_000), 107_000);
        // Full scale: 0xFFFFF << 8 = 83 886 lx on PicoStar (Table 8-3).
        assert_eq!(
            Variant::Opt4001PicoStar.codes_to_millilux(0xF_FFFF << 8),
            83_886_000
        );
        assert_eq!(
            Variant::Opt4001Sot5x3.codes_to_millilux(0xF_FFFF << 8),
            117_440_400
        );
    }

    #[test]
    fn config_word_matches_reset_value() {
        // Register 0x0A resets to 0x3208: auto range, 100 ms, power-down, latch.
        assert_eq!(Config::default().word(0), 0x3208);
        assert_eq!(Config::default().word(1), 0x3218);
        let c = Config {
            conversion_time: ConversionTime::Ms800,
            range: Some(3),
            quick_wake: true,
            latch: false,
            int_active_high: true,
            fault_count: FaultCount::Eight,
        };
        assert_eq!(c.word(1), 0x8000 | 3 << 10 | 11 << 6 | 1 << 4 | 1 << 2 | 3);
    }

    #[test]
    fn thresholds() {
        // ADC_CODES_TH = R << (8 + E).
        assert_eq!(encode_threshold(0x0FFF << 8), 0x0FFF);
        assert_eq!(encode_threshold(0x0FFF << 16), 0x8FFF);
        assert_eq!(encode_threshold(u32::MAX), 0xCFFF);
    }

    #[test]
    fn opt4001_one_shot() {
        let ch = channel(3, 0x12345, 7);
        let expectations = [
            Transaction::write(ADDRESS_VDD, vec![0x0A, 0x32, 0x18]),
            Transaction::write_read(ADDRESS_VDD, vec![0x0C], vec![0x00, 0x00]),
            Transaction::write_read(ADDRESS_VDD, vec![0x0C], vec![0x00, 0x0E]),
            Transaction::write_read(ADDRESS_VDD, vec![0x00], ch.to_vec()),
            Transaction::write(ADDRESS_VDD, vec![0x0A, 0x32, 0x08]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut opt = Opt4001::new(&mut i2c, ADDRESS_VDD, Variant::Opt4001PicoStar);
        assert_eq!(opt.start_measurement().unwrap(), 111_000);
        assert_eq!(opt.read_measurement(), Err(Error::NotReady));
        let m = opt.read_measurement().unwrap();
        assert_eq!(m.codes, 0x12345 << 3);
        assert_eq!(m.illuminance, (0x12345u32 << 3) * 5 / 16);
        assert_eq!(m.counter, 7);
        assert_eq!(m.ir_codes, None);
        assert!(m.overload && m.flag_high && !m.flag_low);
        opt.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn crc_error_and_probe() {
        let mut ch = channel(0, 100, 1);
        ch[2] ^= 0x01;
        let expectations = [
            Transaction::write_read(ADDRESS_GND, vec![0x0C], vec![0x00, 0x04]),
            Transaction::write_read(ADDRESS_GND, vec![0x00], ch.to_vec()),
            Transaction::write_read(ADDRESS_GND, vec![0x11], vec![0x01, 0x21]),
            Transaction::write_read(ADDRESS_GND, vec![0x11], vec![0x02, 0x21]),
            Transaction::write(ADDRESS_GND, vec![0x08, 0x00, 0x02]),
            Transaction::write(ADDRESS_GND, vec![0x09, 0x0F, 0xFF]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut opt = Opt4001::new(&mut i2c, ADDRESS_GND, Variant::Opt4001Sot5x3);
        assert_eq!(opt.read_measurement(), Err(Error::Crc));
        opt.probe().unwrap();
        assert_eq!(opt.probe(), Err(Error::UnexpectedId(0x221)));
        // 0.4375 mlx/code: 224 mlx = 512 codes → R=2, E=0.
        // 458 lx ≈ 0xFFF << 8 codes.
        opt.set_thresholds(224, 458_640).unwrap();
        i2c.done();
    }

    #[test]
    fn async_opt4003_two_channels() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let mut data = vec![];
        data.extend_from_slice(&channel(0, 200_000, 2));
        data.extend_from_slice(&channel(1, 1_000, 2));
        let expectations = [
            Transaction::write(ADDRESS_SDA, vec![0x0A, 0x32, 0x18]),
            Transaction::write_read(ADDRESS_SDA, vec![0x0C], vec![0x00, 0x04]),
            Transaction::write_read(ADDRESS_SDA, vec![0x00], data),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut opt = asynch::Opt4001::new(&mut i2c, ADDRESS_SDA, Variant::Opt4003);
        let mut delay = RecordingDelay(0);
        let m = block_on(opt.measure(&mut delay)).unwrap();
        assert_eq!(m.illuminance, 107_000);
        assert_eq!(m.ir_codes, Some(2_000));
        assert_eq!(delay.0, 221_000);
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
