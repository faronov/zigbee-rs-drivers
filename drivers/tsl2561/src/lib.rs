//! TAOS / ams TSL2560 and TSL2561 light-to-digital converters (I2C/SMBus).
//!
//! The part has no one-shot mode and no data-ready flag, so a sample is:
//! power up → wait one integration period → read both channels → power
//! down. Between samples the part draws 3.2 µA typ. (power-down).
//!
//! Lux uses the datasheet's integer `CalculateLux()` routine (separate
//! coefficient tables for the T/FN/CL and CS packages), with the final
//! shift done at milli-lux resolution.
//!
//! Reference: TAOS TAOS059N "TSL2560, TSL2561 Light-to-Digital Converter"
//! (March 2009), "Simplified Lux Calculation" and "Calculating Lux".
//!
//! ```ignore
//! let mut tsl = Tsl2561::new(i2c, ADDRESS_FLOAT).with_gain(Gain::X16);
//! tsl.probe()?;
//! let wait_us = tsl.start_measurement()?;
//! // ... sleep wait_us ...
//! let lux_milli = tsl.read_measurement()?.illuminance;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with ADDR SEL tied to GND.
pub const ADDRESS_GND: u8 = 0x29;
/// I2C address with ADDR SEL floating.
pub const ADDRESS_FLOAT: u8 = 0x39;
/// I2C address with ADDR SEL tied to VDD.
pub const ADDRESS_VDD: u8 = 0x49;

const CMD: u8 = 0x80;
const CMD_CLEAR: u8 = 0x40;
const CMD_WORD: u8 = 0x20;

const REG_CONTROL: u8 = 0x00;
const REG_TIMING: u8 = 0x01;
const REG_THRESH_LOW: u8 = 0x02;
const REG_THRESH_HIGH: u8 = 0x04;
const REG_INTERRUPT: u8 = 0x06;
const REG_ID: u8 = 0x0A;
const REG_DATA0: u8 = 0x0C;
const REG_DATA1: u8 = 0x0E;

const POWER_ON: u8 = 0x03;
const POWER_OFF: u8 = 0x00;

const TIMING_GAIN_16X: u8 = 1 << 4;

/// Power-on to first valid conversion margin added to the integration time.
const STARTUP_MARGIN_US: u32 = 2_000;

/// ADC gain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gain {
    /// Low gain (×1), for bright light.
    #[default]
    X1,
    /// High gain (×16), for dim light.
    X16,
}

/// Integration time (TIMING.INTEG). Nominal values at f_osc = 735 kHz.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Integration {
    /// 13.7 ms; ADC saturates at 5047 counts.
    Ms13,
    /// 101 ms; ADC saturates at 37177 counts.
    Ms101,
    /// 402 ms; ADC saturates at 65535 counts.
    #[default]
    Ms402,
}

impl Integration {
    const fn bits(self) -> u8 {
        match self {
            Self::Ms13 => 0b00,
            Self::Ms101 => 0b01,
            Self::Ms402 => 0b10,
        }
    }

    /// Nominal integration time in microseconds.
    pub const fn micros(self) -> u32 {
        match self {
            Self::Ms13 => 13_700,
            Self::Ms101 => 101_000,
            Self::Ms402 => 402_000,
        }
    }

    /// Full-scale ADC count (datasheet note 6).
    pub const fn full_scale(self) -> u16 {
        match self {
            Self::Ms13 => 5047,
            Self::Ms101 => 37177,
            Self::Ms402 => 65535,
        }
    }

    /// `CHSCALE` for the datasheet integer routine (402 ms → 1 << 10).
    const fn ch_scale(self) -> u32 {
        match self {
            Self::Ms13 => 0x7517,
            Self::Ms101 => 0x0FE7,
            Self::Ms402 => 1 << CH_SCALE,
        }
    }
}

/// Package, which selects the lux coefficient table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Package {
    /// T (TMB-6), FN (dual flat no-lead) and CL (ChipLED).
    #[default]
    TFnCl,
    /// CS (chipscale).
    Cs,
}

/// Part identified by the ID register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PartId {
    /// `true` for TSL2561 (SMBus interface), `false` for TSL2560.
    pub tsl2561: bool,
    /// Package hint from PARTNO bit 6 (see [`Tsl2561::probe`]).
    pub package: Package,
    /// Silicon revision (REVNO).
    pub revision: u8,
}

/// Decode the ID register.
///
/// TAOS059N only lists PARTNO 0000 (TSL2560) and 0001 (TSL2561); shipping
/// T/FN/CL parts report 0100 / 0101 (as documented in earlier datasheet
/// revisions), so bit 6 is taken as the package hint.
pub const fn decode_id(id: u8) -> Option<PartId> {
    let part = id >> 4;
    if part & !0b0101 != 0 {
        return None;
    }
    Some(PartId {
        tsl2561: part & 0b0001 != 0,
        package: if part & 0b0100 != 0 {
            Package::TFnCl
        } else {
            Package::Cs
        },
        revision: id & 0x0F,
    })
}

const LUX_SCALE: u32 = 14;
const RATIO_SCALE: u32 = 9;
const CH_SCALE: u32 = 10;

/// `(K, B, M)` segments from the datasheet; ratios above the last K give 0.
const COEFF_T: [(u32, u32, u32); 7] = [
    (0x0040, 0x01F2, 0x01BE),
    (0x0080, 0x0214, 0x02D1),
    (0x00C0, 0x023F, 0x037B),
    (0x0100, 0x0270, 0x03FE),
    (0x0138, 0x016F, 0x01FC),
    (0x019A, 0x00D2, 0x00FB),
    (0x029A, 0x0018, 0x0012),
];
const COEFF_CS: [(u32, u32, u32); 7] = [
    (0x0043, 0x0204, 0x01AD),
    (0x0085, 0x0228, 0x02C1),
    (0x00C8, 0x0253, 0x0363),
    (0x010A, 0x0282, 0x03DF),
    (0x014D, 0x0177, 0x01DD),
    (0x019A, 0x0101, 0x0127),
    (0x029A, 0x0037, 0x002B),
];

/// Datasheet `CalculateLux()` with the final `>> LUX_SCALE` done after
/// multiplying by 1000, so the result is in milli-lux.
pub const fn calculate_millilux(
    ch0: u16,
    ch1: u16,
    gain: Gain,
    integration: Integration,
    package: Package,
) -> u32 {
    let mut ch_scale = integration.ch_scale();
    if let Gain::X1 = gain {
        ch_scale <<= 4;
    }
    let channel0 = (ch0 as u32 * ch_scale) >> CH_SCALE;
    let channel1 = (ch1 as u32 * ch_scale) >> CH_SCALE;

    let mut ratio1 = 0;
    if channel0 != 0 {
        ratio1 = ((channel1 as u64) << (RATIO_SCALE + 1)) / channel0 as u64;
    }
    let ratio = ((ratio1 + 1) >> 1) as u32;

    let table = match package {
        Package::TFnCl => &COEFF_T,
        Package::Cs => &COEFF_CS,
    };
    let mut b = 0;
    let mut m = 0;
    let mut i = 0;
    while i < table.len() {
        if ratio <= table[i].0 {
            b = table[i].1;
            m = table[i].2;
            break;
        }
        i += 1;
    }

    let pos = channel0 as u64 * b as u64;
    let neg = channel1 as u64 * m as u64;
    let temp = pos.saturating_sub(neg);
    let lux = (temp * 1000 + (1 << (LUX_SCALE - 1))) >> LUX_SCALE;
    if lux > u32::MAX as u64 {
        u32::MAX
    } else {
        lux as u32
    }
}

/// Simple auto-gain step: switch to ×1 when a ×16 sample is above half of
/// full scale, to ×16 when a ×1 sample is below 1/32 of full scale.
pub const fn suggest_gain(ch0: u16, gain: Gain, integration: Integration) -> Gain {
    let fs = integration.full_scale() as u32;
    match gain {
        Gain::X16 if ch0 as u32 * 2 > fs => Gain::X1,
        Gain::X1 if (ch0 as u32) * 32 < fs => Gain::X16,
        g => g,
    }
}

/// Interrupt output mode (INTERRUPT.INTR).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptMode {
    /// Interrupt output disabled.
    Disabled,
    /// Level interrupt (cleared with [`Tsl2561::clear_interrupt`]).
    Level,
    /// SMBAlert-compliant.
    SmbAlert,
    /// Assert immediately (connectivity test).
    Test,
}

/// One illuminance sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// Channel 0 (visible + IR) raw count.
    pub ch0: u16,
    /// Channel 1 (IR) raw count.
    pub ch1: u16,
    /// A channel reached full scale; `illuminance` is unreliable.
    pub saturated: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// TSL2560 / TSL2561 driver.
        pub struct Tsl2561<I2C> {
            i2c: I2C,
            address: u8,
            gain: Gain,
            integration: Integration,
            package: Package,
            continuous: bool,
        }

        impl<I2C> Tsl2561<I2C> {
            /// Create a driver (×1 gain, 402 ms, T/FN/CL package); no bus traffic.
            pub const fn new(i2c: I2C, address: u8) -> Self {
                Self {
                    i2c,
                    address,
                    gain: Gain::X1,
                    integration: Integration::Ms402,
                    package: Package::TFnCl,
                    continuous: false,
                }
            }

            /// Select the gain.
            pub const fn with_gain(mut self, gain: Gain) -> Self {
                self.gain = gain;
                self
            }

            /// Select the integration time.
            pub const fn with_integration(mut self, integration: Integration) -> Self {
                self.integration = integration;
                self
            }

            /// Select the package (lux coefficient table).
            pub const fn with_package(mut self, package: Package) -> Self {
                self.package = package;
                self
            }

            /// Keep the part powered between samples (needed for the
            /// threshold interrupt). Default: power down after each read.
            pub const fn with_continuous(mut self, continuous: bool) -> Self {
                self.continuous = continuous;
                self
            }

            /// Current gain.
            pub const fn gain(&self) -> Gain {
                self.gain
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }

            const fn timing(&self) -> u8 {
                let gain = match self.gain {
                    Gain::X1 => 0,
                    Gain::X16 => TIMING_GAIN_16X,
                };
                gain | self.integration.bits()
            }
        }

        impl<I2C: I2c> Tsl2561<I2C> {
            $($async_)? fn write_byte(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(self.address, &[CMD | reg, value])
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn write_word(&mut self, reg: u8, value: u16) -> Result<(), Error<I2C::Error>> {
                let [lo, hi] = value.to_le_bytes();
                self.i2c
                    .write(self.address, &[CMD | CMD_WORD | reg, lo, hi])
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn read_byte(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
                let mut buf = [0u8; 1];
                self.i2c
                    .write_read(self.address, &[CMD | reg], &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                Ok(buf[0])
            }

            $($async_)? fn read_word(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(self.address, &[CMD | CMD_WORD | reg], &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                Ok(u16::from_le_bytes(buf))
            }

            /// Read and check the ID register. Returns
            /// [`Error::UnexpectedId`] for an unknown PARTNO. The package
            /// hint is not applied automatically; use [`Self::with_package`].
            pub $($async_)? fn probe(&mut self) -> Result<PartId, Error<I2C::Error>> {
                let id = self.read_byte(REG_ID) $($await_)* ?;
                decode_id(id).ok_or(Error::UnexpectedId(id as u32))
            }

            /// Change the gain (takes effect from the next sample).
            pub $($async_)? fn set_gain(&mut self, gain: Gain) -> Result<(), Error<I2C::Error>> {
                self.gain = gain;
                let timing = self.timing();
                self.write_byte(REG_TIMING, timing) $($await_)*
            }

            /// Program the channel-0 thresholds (raw counts) and interrupt
            /// mode. `persistence` 0 = every cycle, 1 = any out-of-range
            /// sample, 2..=15 = that many consecutive periods.
            pub $($async_)? fn set_interrupt(
                &mut self,
                low: u16,
                high: u16,
                mode: InterruptMode,
                persistence: u8,
            ) -> Result<(), Error<I2C::Error>> {
                self.write_word(REG_THRESH_LOW, low) $($await_)* ?;
                self.write_word(REG_THRESH_HIGH, high) $($await_)* ?;
                let intr = match mode {
                    InterruptMode::Disabled => 0b00,
                    InterruptMode::Level => 0b01,
                    InterruptMode::SmbAlert => 0b10,
                    InterruptMode::Test => 0b11,
                };
                self.write_byte(REG_INTERRUPT, (intr << 4) | (persistence & 0x0F)) $($await_)*
            }

            /// Clear a pending level interrupt.
            pub $($async_)? fn clear_interrupt(&mut self) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(self.address, &[CMD | CMD_CLEAR])
                    $($await_)*
                    .map_err(Error::bus)
            }
        }

        impl<I2C: I2c> Sensor for Tsl2561<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Power up and program gain / integration time; the first
            /// conversion completes after one integration period.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_byte(REG_CONTROL, POWER_ON) $($await_)* ?;
                let timing = self.timing();
                self.write_byte(REG_TIMING, timing) $($await_)* ?;
                Ok(self.integration.micros() + self.integration.micros() / 10 + STARTUP_MARGIN_US)
            }

            /// Read both channels and power down (unless continuous).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let ch0 = self.read_word(REG_DATA0) $($await_)* ?;
                let ch1 = self.read_word(REG_DATA1) $($await_)* ?;
                if !self.continuous {
                    self.write_byte(REG_CONTROL, POWER_OFF) $($await_)* ?;
                }
                let fs = self.integration.full_scale();
                Ok(Measurement {
                    illuminance: calculate_millilux(ch0, ch1, self.gain, self.integration, self.package),
                    ch0,
                    ch1,
                    saturated: ch0 >= fs || ch1 >= fs,
                })
            }
        }

        impl<I2C: I2c> LowPower for Tsl2561<I2C> {
            type BusError = I2C::Error;

            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_byte(REG_CONTROL, POWER_OFF) $($await_)*
            }

            /// Samples power the part up themselves; nothing to do.
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
        CMD, CMD_CLEAR, CMD_WORD, Gain, Integration, InterruptMode, Measurement, POWER_OFF,
        POWER_ON, Package, PartId, REG_CONTROL, REG_DATA0, REG_DATA1, REG_ID, REG_INTERRUPT,
        REG_THRESH_HIGH, REG_THRESH_LOW, REG_TIMING, STARTUP_MARGIN_US, TIMING_GAIN_16X,
        calculate_millilux, decode_id,
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

    const A: u8 = ADDRESS_FLOAT;

    #[test]
    fn lux_matches_float_formula() {
        // T package, 402 ms, ×16: ratio 0.25 lies in the 0.125–0.25 segment:
        // (1000·0x214 − 250·0x2D1) / 2^14 = 21.469 lx.
        assert_eq!(
            calculate_millilux(1000, 250, Gain::X16, Integration::Ms402, Package::TFnCl),
            21_469
        );
        // ×1 scales by 16.
        assert_eq!(
            calculate_millilux(1000, 0, Gain::X1, Integration::Ms402, Package::TFnCl),
            486_328
        );
        // Ratio > 1.3 → 0 lux; ch0 = 0 → 0.
        assert_eq!(
            calculate_millilux(100, 200, Gain::X16, Integration::Ms402, Package::Cs),
            0
        );
        assert_eq!(
            calculate_millilux(0, 0, Gain::X1, Integration::Ms13, Package::Cs),
            0
        );
        // 13.7 ms channel scale 0x7517/1024 ≈ 29.27 (= 322/11).
        let fast = calculate_millilux(100, 0, Gain::X16, Integration::Ms13, Package::Cs);
        let slow = calculate_millilux(2927, 0, Gain::X16, Integration::Ms402, Package::Cs);
        assert!(fast.abs_diff(slow) <= 20, "{fast} vs {slow}");
    }

    #[test]
    fn id_decoding() {
        let t = decode_id(0x50).unwrap();
        assert!(t.tsl2561 && t.package == Package::TFnCl && t.revision == 0);
        let cs = decode_id(0x0A).unwrap();
        assert!(!cs.tsl2561 && cs.package == Package::Cs && cs.revision == 0xA);
        assert_eq!(decode_id(0x20), None);
        assert_eq!(
            suggest_gain(40_000, Gain::X16, Integration::Ms402),
            Gain::X1
        );
        assert_eq!(suggest_gain(100, Gain::X1, Integration::Ms402), Gain::X16);
    }

    #[test]
    fn measurement_cycle() {
        let expectations = [
            Transaction::write_read(A, vec![0x8A], vec![0x50]),
            Transaction::write(A, vec![0x80, 0x03]),
            Transaction::write(A, vec![0x81, 0x11]),
            Transaction::write_read(A, vec![0xAC], vec![0xE8, 0x03]),
            Transaction::write_read(A, vec![0xAE], vec![0xFA, 0x00]),
            Transaction::write(A, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tsl = Tsl2561::new(&mut i2c, A)
            .with_gain(Gain::X16)
            .with_integration(Integration::Ms101);
        tsl.probe().unwrap();
        assert_eq!(tsl.start_measurement().unwrap(), 113_100);
        let m = tsl.read_measurement().unwrap();
        assert_eq!((m.ch0, m.ch1, m.saturated), (1000, 250, false));
        assert_eq!(
            m.illuminance,
            calculate_millilux(1000, 250, Gain::X16, Integration::Ms101, Package::TFnCl)
        );
        i2c.done();
    }

    #[test]
    fn saturation_and_bad_id() {
        let expectations = [
            Transaction::write_read(A, vec![0x8A], vec![0x20]),
            Transaction::write(A, vec![0x80, 0x03]),
            Transaction::write(A, vec![0x81, 0x00]),
            Transaction::write_read(A, vec![0xAC], vec![0xB7, 0x13]),
            Transaction::write_read(A, vec![0xAE], vec![0x00, 0x01]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tsl = Tsl2561::new(&mut i2c, A)
            .with_integration(Integration::Ms13)
            .with_continuous(true);
        assert_eq!(tsl.probe(), Err(Error::UnexpectedId(0x20)));
        tsl.start_measurement().unwrap();
        assert!(tsl.read_measurement().unwrap().saturated);
        i2c.done();
    }

    #[test]
    fn interrupt_setup() {
        let expectations = [
            Transaction::write(A, vec![0xA2, 0x10, 0x00]),
            Transaction::write(A, vec![0xA4, 0x00, 0x10]),
            Transaction::write(A, vec![0x86, 0x13]),
            Transaction::write(A, vec![0xC0]),
            Transaction::write(A, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tsl = Tsl2561::new(&mut i2c, A);
        tsl.set_interrupt(0x10, 0x1000, InterruptMode::Level, 3)
            .unwrap();
        tsl.clear_interrupt().unwrap();
        tsl.sleep().unwrap();
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
            Transaction::write(A, vec![0x80, 0x03]),
            Transaction::write(A, vec![0x81, 0x02]),
            Transaction::write_read(A, vec![0xAC], vec![0x64, 0x00]),
            Transaction::write_read(A, vec![0xAE], vec![0x00, 0x00]),
            Transaction::write(A, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tsl = asynch::Tsl2561::new(&mut i2c, A);
        let mut delay = RecordingDelay(0);
        let m = block_on(tsl.measure(&mut delay)).unwrap();
        assert_eq!(m.ch0, 100);
        // 100 counts ×16 × 0x1F2 / 2^14 lx = 48.6 lx.
        assert_eq!(m.illuminance, 48_633);
        assert_eq!(delay.0, 444_200);
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
