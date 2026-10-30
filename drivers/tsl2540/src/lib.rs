//! ams-OSRAM TSL2540 ambient light sensor (visible + IR, I2C, 1.8 V).
//!
//! The part has no data-valid flag and no one-shot mode. For a sleepy
//! device the driver uses the ams "sleep after interrupt" scheme: with
//! `APERS = 0` (interrupt every ALS cycle), `AIEN = 1` and `CFG3.SAI = 1`
//! the oscillator stops by itself after the first integration and the
//! `AINT` status bit doubles as data-ready. `read_measurement` reads the
//! data, clears the flags and writes `PON = 0`. (`INT` pulses low once per
//! sample in this mode.)
//!
//! With [`Tsl2540::with_continuous`] the ALS keeps running and the
//! interrupt pin follows the visible-channel thresholds instead.
//!
//! The datasheet leaves the lux equation to the application; the default
//! here is the ams device factor used by the Zephyr `tsl2540` driver:
//! `lux = VIS · DF / (ATIME_ms · AGAIN)` with `DF = 53` (open air,
//! adjustable for cover glass with [`Tsl2540::with_device_factor`]).
//!
//! Reference: ams-OSRAM "TSL2540 Ambient Light Sensor" datasheet
//! DS000564 v2-00 (2022-12-05).
//!
//! ```ignore
//! let mut tsl = Tsl2540::new(i2c).with_gain(Gain::X16);
//! tsl.init()?; // ID check + static configuration
//! let wait_us = tsl.start_measurement()?;
//! // ... sleep wait_us ...
//! let lux_milli = tsl.read_measurement()?.illuminance;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x39;
/// `ID[7:2]` = 111001b.
pub const DEVICE_ID: u8 = 0xE4;

const REG_ENABLE: u8 = 0x80;
const REG_ATIME: u8 = 0x81;
const REG_AILTL: u8 = 0x84;
const REG_PERS: u8 = 0x8C;
const REG_CFG1: u8 = 0x90;
const REG_ID: u8 = 0x92;
const REG_STATUS: u8 = 0x93;
const REG_CFG2: u8 = 0x9F;
const REG_CFG3: u8 = 0xAB;
const REG_INTENAB: u8 = 0xDD;

const ENABLE_AEN: u8 = 1 << 1;
const ENABLE_PON: u8 = 1 << 0;

const STATUS_ASAT: u8 = 1 << 7;
const STATUS_AINT: u8 = 1 << 4;

const CFG3_INT_READ_CLEAR: u8 = 1 << 7;
const CFG3_SAI: u8 = 1 << 4;

const INTENAB_AIEN: u8 = 1 << 4;

/// Max integration step (datasheet: 2.68 / 2.78 / 2.90 ms).
const STEP_MAX_US: u32 = 2_900;
/// Nominal integration step used by the lux equation.
pub const STEP_US: u32 = 2_810;
/// Allowance for the auto-zero calibration that precedes the first cycle.
const AUTOZERO_US: u32 = 3_000;

/// Default lux device factor (open air), in milli-units.
pub const DEVICE_FACTOR_DEFAULT: u32 = 53_000;

/// ALS gain (CFG1.AGAIN + CFG2.AGAINMAX / AGAINL).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gain {
    /// ×½.
    Half,
    /// ×1.
    #[default]
    X1,
    /// ×4.
    X4,
    /// ×16.
    X16,
    /// ×64 (67 typ.).
    X64,
    /// ×128 (140 typ.).
    X128,
}

impl Gain {
    /// `(CFG1, CFG2)` register values (datasheet Figure 33).
    const fn regs(self) -> (u8, u8) {
        match self {
            Self::Half => (0, 0x00),
            Self::X1 => (0, 0x04),
            Self::X4 => (1, 0x04),
            Self::X16 => (2, 0x04),
            Self::X64 => (3, 0x04),
            Self::X128 => (3, 0x14),
        }
    }

    /// Typical gain relative to ×1, times two (datasheet "Gain scaling").
    pub const fn factor_x2(self) -> u32 {
        match self {
            Self::Half => 1,
            Self::X1 => 2,
            Self::X4 => 8,
            Self::X16 => 32,
            Self::X64 => 134,
            Self::X128 => 280,
        }
    }

    const fn up(self) -> Option<Self> {
        match self {
            Self::Half => Some(Self::X1),
            Self::X1 => Some(Self::X4),
            Self::X4 => Some(Self::X16),
            Self::X16 => Some(Self::X64),
            Self::X64 => Some(Self::X128),
            Self::X128 => None,
        }
    }

    const fn down(self) -> Option<Self> {
        match self {
            Self::Half => None,
            Self::X1 => Some(Self::Half),
            Self::X4 => Some(Self::X1),
            Self::X16 => Some(Self::X4),
            Self::X64 => Some(Self::X16),
            Self::X128 => Some(Self::X64),
        }
    }
}

/// Full-scale count for `atime` (1024 counts per 2.81 ms step).
pub const fn full_scale(atime: u8) -> u16 {
    let fs = (atime as u32 + 1) * 1024 - 1;
    if fs > 0xFFFF { 0xFFFF } else { fs as u16 }
}

/// ATIME register value for an integration time in milliseconds
/// (2.81 ms steps, clamped to 1..=256 steps).
pub const fn atime_for_ms(ms: u32) -> u8 {
    let steps = (ms * 1000).div_ceil(STEP_US);
    if steps == 0 {
        0
    } else if steps > 256 {
        255
    } else {
        (steps - 1) as u8
    }
}

/// Visible count → milli-lux: `VIS · DF / (t_ms · gain)`.
/// `device_factor` is in milli-units (53 000 = 53).
pub const fn calculate_millilux(vis: u16, atime: u8, gain: Gain, device_factor: u32) -> u32 {
    let num = vis as u64 * device_factor as u64 * 2_000;
    let den = (atime as u64 + 1) * STEP_US as u64 * gain.factor_x2() as u64;
    let mlx = num / den;
    if mlx > u32::MAX as u64 {
        u32::MAX
    } else {
        mlx as u32
    }
}

/// Pure auto-gain step: drop the gain above 90 % of full scale, raise it
/// when the next gain keeps the visible count below 50 % of full scale.
pub const fn suggest_gain(vis: u16, gain: Gain, atime: u8) -> Gain {
    let fs = full_scale(atime) as u64;
    let v = vis as u64;
    if v * 10 > fs * 9 {
        if let Some(g) = gain.down() {
            return g;
        }
    } else if let Some(g) = gain.up() {
        if v * g.factor_x2() as u64 * 2 < fs * gain.factor_x2() as u64 {
            return g;
        }
    }
    gain
}

/// One illuminance sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// Visible channel raw count.
    pub visible: u16,
    /// IR channel raw count.
    pub ir: u16,
    /// Digital full scale reached or analog saturation (ASAT).
    pub saturated: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// TSL2540 driver.
        pub struct Tsl2540<I2C> {
            i2c: I2C,
            gain: Gain,
            atime: u8,
            device_factor: u32,
            continuous: bool,
            persistence: u8,
            thresholds: bool,
        }

        impl<I2C> Tsl2540<I2C> {
            /// Create a driver (×1, ~100 ms); no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    gain: Gain::X1,
                    atime: atime_for_ms(100),
                    device_factor: DEVICE_FACTOR_DEFAULT,
                    continuous: false,
                    persistence: 1,
                    thresholds: false,
                }
            }

            /// Select the gain (applied by [`Self::init`]).
            pub const fn with_gain(mut self, gain: Gain) -> Self {
                self.gain = gain;
                self
            }

            /// Select the raw ATIME value (integration = (ATIME+1) × 2.81 ms).
            pub const fn with_atime(mut self, atime: u8) -> Self {
                self.atime = atime;
                self
            }

            /// Lux device factor in milli-units (default 53 000 = open air;
            /// multiply by the cover glass attenuation).
            pub const fn with_device_factor(mut self, device_factor: u32) -> Self {
                self.device_factor = device_factor;
                self
            }

            /// Keep the ALS running; `INT` follows the thresholds.
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
        }

        impl<I2C: I2c> Tsl2540<I2C> {
            $($async_)? fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(ADDRESS, &[reg, value])
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write_read(ADDRESS, &[reg], buf)
                    $($await_)*
                    .map_err(Error::bus)
            }

            /// Check the ID, power down and write the full configuration.
            pub $($async_)? fn init(&mut self) -> Result<(), Error<I2C::Error>> {
                let mut id = [0u8; 1];
                self.read(REG_ID, &mut id) $($await_)* ?;
                if id[0] & 0xFC != DEVICE_ID {
                    return Err(Error::UnexpectedId(id[0] as u32));
                }
                self.write(REG_ENABLE, 0) $($await_)* ?;
                let mut cfg3 = [0u8; 1];
                self.read(REG_CFG3, &mut cfg3) $($await_)* ?;
                let mut cfg3 = (cfg3[0] & !CFG3_SAI) | CFG3_INT_READ_CLEAR;
                if !self.continuous {
                    cfg3 |= CFG3_SAI;
                }
                self.write(REG_CFG3, cfg3) $($await_)* ?;
                self.write_interrupt_config() $($await_)* ?;
                let atime = self.atime;
                self.write(REG_ATIME, atime) $($await_)* ?;
                self.set_gain(self.gain) $($await_)*
            }

            $($async_)? fn write_interrupt_config(&mut self) -> Result<(), Error<I2C::Error>> {
                let (pers, intenab) = if self.continuous {
                    (self.persistence, if self.thresholds { INTENAB_AIEN } else { 0 })
                } else {
                    (0, INTENAB_AIEN)
                };
                self.write(REG_PERS, pers) $($await_)* ?;
                self.write(REG_INTENAB, intenab) $($await_)*
            }

            /// Change the gain.
            pub $($async_)? fn set_gain(&mut self, gain: Gain) -> Result<(), Error<I2C::Error>> {
                let (cfg1, cfg2) = gain.regs();
                self.write(REG_CFG1, cfg1) $($await_)* ?;
                self.write(REG_CFG2, cfg2) $($await_)* ?;
                self.gain = gain;
                Ok(())
            }

            /// Change the integration time (raw ATIME).
            pub $($async_)? fn set_atime(&mut self, atime: u8) -> Result<(), Error<I2C::Error>> {
                self.write(REG_ATIME, atime) $($await_)* ?;
                self.atime = atime;
                Ok(())
            }

            /// Continuous mode only: visible-channel thresholds (raw counts)
            /// with APERS `persistence` (0..=15, see datasheet Figure 21).
            /// `None` disables the interrupt.
            pub $($async_)? fn set_thresholds(
                &mut self,
                window: Option<(u16, u16)>,
                persistence: u8,
            ) -> Result<(), Error<I2C::Error>> {
                if !self.continuous {
                    return Err(Error::Unsupported);
                }
                if let Some((low, high)) = window {
                    let [ll, lh] = low.to_le_bytes();
                    let [hl, hh] = high.to_le_bytes();
                    self.i2c
                        .write(ADDRESS, &[REG_AILTL, ll, lh, hl, hh])
                        $($await_)*
                        .map_err(Error::bus)?;
                }
                self.thresholds = window.is_some();
                self.persistence = persistence & 0x0F;
                self.write_interrupt_config() $($await_)*
            }

            /// Read (and thereby clear) the status flags; returns `AINT`.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<bool, Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_STATUS, &mut status) $($await_)* ?;
                Ok(status[0] & STATUS_AINT != 0)
            }
        }

        impl<I2C: I2c> Sensor for Tsl2540<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Set `PON | AEN` (auto-zero, then one integration).
            /// Requires a prior [`Tsl2540::init`].
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write(REG_ENABLE, ENABLE_PON | ENABLE_AEN) $($await_)* ?;
                Ok(AUTOZERO_US + (self.atime as u32 + 1) * STEP_MAX_US)
            }

            /// One-shot: require `AINT`, read, power down. Continuous: read.
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut buf = [0u8; 5];
                self.read(REG_STATUS, &mut buf) $($await_)* ?;
                let status = buf[0];
                if !self.continuous {
                    if status & STATUS_AINT == 0 {
                        return Err(Error::NotReady);
                    }
                    self.write(REG_ENABLE, 0) $($await_)* ?;
                }
                let visible = u16::from_le_bytes([buf[1], buf[2]]);
                let ir = u16::from_le_bytes([buf[3], buf[4]]);
                let fs = full_scale(self.atime);
                Ok(Measurement {
                    illuminance: calculate_millilux(visible, self.atime, self.gain, self.device_factor),
                    visible,
                    ir,
                    saturated: status & STATUS_ASAT != 0 || visible >= fs || ir >= fs,
                })
            }
        }

        impl<I2C: I2c> LowPower for Tsl2540<I2C> {
            type BusError = I2C::Error;

            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_ENABLE, 0) $($await_)*
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
        ADDRESS, AUTOZERO_US, CFG3_INT_READ_CLEAR, CFG3_SAI, DEVICE_FACTOR_DEFAULT, DEVICE_ID,
        ENABLE_AEN, ENABLE_PON, Gain, INTENAB_AIEN, Measurement, REG_AILTL, REG_ATIME, REG_CFG1,
        REG_CFG2, REG_CFG3, REG_ENABLE, REG_ID, REG_INTENAB, REG_PERS, REG_STATUS, STATUS_AINT,
        STATUS_ASAT, STEP_MAX_US, atime_for_ms, calculate_millilux, full_scale,
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
    use std::vec::Vec;

    fn init_tx(cfg3_in: u8, cfg3_out: u8, pers: u8, intenab: u8) -> Vec<Transaction> {
        vec![
            Transaction::write_read(ADDRESS, vec![0x92], vec![0xE5]),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
            Transaction::write_read(ADDRESS, vec![0xAB], vec![cfg3_in]),
            Transaction::write(ADDRESS, vec![0xAB, cfg3_out]),
            Transaction::write(ADDRESS, vec![0x8C, pers]),
            Transaction::write(ADDRESS, vec![0xDD, intenab]),
        ]
    }

    #[test]
    fn lux_and_helpers() {
        // 64 steps (179.84 ms), ×16: 1000 / (179.84 · 16) · 53 = 18.419 lx.
        assert_eq!(
            calculate_millilux(1000, 0x3F, Gain::X16, DEVICE_FACTOR_DEFAULT),
            18_419
        );
        // ×½, 2.81 ms, full scale 1023 → ~38.6 klx.
        assert_eq!(
            calculate_millilux(1023, 0, Gain::Half, DEVICE_FACTOR_DEFAULT),
            38_590_035
        );
        assert_eq!(full_scale(0), 1023);
        assert_eq!(full_scale(0x3F), 65535);
        assert_eq!(atime_for_ms(100), 35);
        assert_eq!(atime_for_ms(0), 0);
        assert_eq!(atime_for_ms(10_000), 255);
        assert_eq!(suggest_gain(60_000, Gain::X16, 0x3F), Gain::X4);
        assert_eq!(suggest_gain(100, Gain::X16, 0x3F), Gain::X64);
        assert_eq!(suggest_gain(10_000, Gain::X16, 0x3F), Gain::X16);
    }

    #[test]
    fn one_shot_cycle() {
        let mut tx = init_tx(0x0C, 0x9C, 0x00, 0x10);
        tx.extend([
            Transaction::write(ADDRESS, vec![0x81, 0x3F]),
            Transaction::write(ADDRESS, vec![0x90, 0x02]),
            Transaction::write(ADDRESS, vec![0x9F, 0x04]),
            Transaction::write(ADDRESS, vec![0x80, 0x03]),
            Transaction::write_read(ADDRESS, vec![0x93], vec![0x00, 0, 0, 0, 0]),
            Transaction::write_read(ADDRESS, vec![0x93], vec![0x10, 0xE8, 0x03, 0x20, 0x00]),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
        ]);
        let mut i2c = Mock::new(&tx);
        let mut tsl = Tsl2540::new(&mut i2c).with_gain(Gain::X16).with_atime(0x3F);
        tsl.init().unwrap();
        assert_eq!(tsl.start_measurement().unwrap(), 3_000 + 64 * 2_900);
        assert_eq!(tsl.read_measurement(), Err(Error::NotReady));
        let m = tsl.read_measurement().unwrap();
        assert_eq!((m.visible, m.ir, m.saturated), (1000, 32, false));
        assert_eq!(m.illuminance, 18_419);
        i2c.done();
    }

    #[test]
    fn continuous_thresholds() {
        let mut i2c = Mock::new(&[Transaction::write_read(ADDRESS, vec![0x92], vec![0x50])]);
        let mut tsl = Tsl2540::new(&mut i2c);
        assert_eq!(tsl.init(), Err(Error::UnexpectedId(0x50)));
        i2c.done();

        let mut tx = init_tx(0x9C, 0x8C, 0x01, 0x00);
        tx.extend([
            Transaction::write(ADDRESS, vec![0x81, 0x00]),
            Transaction::write(ADDRESS, vec![0x90, 0x03]),
            Transaction::write(ADDRESS, vec![0x9F, 0x14]),
            Transaction::write(ADDRESS, vec![0x84, 0x10, 0x00, 0x00, 0x02]),
            Transaction::write(ADDRESS, vec![0x8C, 0x04]),
            Transaction::write(ADDRESS, vec![0xDD, 0x10]),
            Transaction::write(ADDRESS, vec![0x80, 0x03]),
            Transaction::write_read(ADDRESS, vec![0x93], vec![0x80, 0xFF, 0x03, 0x00, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x93], vec![0x10]),
        ]);
        let mut i2c = Mock::new(&tx);
        let mut tsl = Tsl2540::new(&mut i2c)
            .with_continuous(true)
            .with_gain(Gain::X128)
            .with_atime(0);
        tsl.init().unwrap();
        tsl.set_thresholds(Some((0x10, 0x200)), 4).unwrap();
        tsl.start_measurement().unwrap();
        assert!(tsl.read_measurement().unwrap().saturated);
        assert!(tsl.take_interrupt().unwrap());
        i2c.done();
    }

    #[test]
    fn thresholds_need_continuous() {
        let mut i2c = Mock::new(&[]);
        let mut tsl = Tsl2540::new(&mut i2c);
        assert_eq!(tsl.set_thresholds(None, 0), Err(Error::Unsupported));
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
            Transaction::write(ADDRESS, vec![0x80, 0x03]),
            Transaction::write_read(ADDRESS, vec![0x93], vec![0x10, 0x64, 0x00, 0x00, 0x00]),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tsl = asynch::Tsl2540::new(&mut i2c).with_atime(0);
        let mut delay = RecordingDelay(0);
        let m = block_on(tsl.measure(&mut delay)).unwrap();
        // 100 / 2.81 · 53 = 1886.1 lx.
        assert_eq!(m.illuminance, 1_886_120);
        assert_eq!(delay.0, 5_900);
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
