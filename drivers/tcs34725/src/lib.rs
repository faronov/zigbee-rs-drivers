//! ams (TAOS) TCS3472x RGBC colour light sensor (I2C).
//!
//! Covers TCS34721/TCS34725 (ID 0x44) and TCS34723/TCS34727 (ID 0x4D).
//! `start_measurement` powers the part and enables the RGBC engine,
//! `read_measurement` checks `AVALID`, burst-reads status + C/R/G/B and
//! returns the part to sleep (2.5 µA typ.).
//!
//! Illuminance and correlated colour temperature follow ams design note
//! DN40 ("Lux and CCT Calculations using ams Color Sensors"), in integer
//! arithmetic:
//!
//! ```text
//! IR  = (R + G + B − C) / 2          R' = R − IR, G' = G − IR, B' = B − IR
//! G'' = R_coef·R' + G_coef·G' + B_coef·B'
//! CPL = (T_int[ms] · AGAIN) / (GA · DF)
//! lux = G'' / CPL                    CCT = CT_coef · B'/R' + CT_offset
//! ```
//!
//! with the TCS3472 open-air coefficients ([`DN40_TCS3472`]); `GA` (glass
//! attenuation) defaults to 1.
//!
//! Reference: TAOS135 "TCS3472 Color Light-to-Digital Converter with IR
//! Filter" datasheet; ams DN40.
//!
//! ```ignore
//! let mut rgb = Tcs34725::new(i2c).with_gain(Gain::X16);
//! rgb.probe()?;
//! let m = rgb.measure(&mut delay)?;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x29;
/// ID of TCS34721 / TCS34725.
pub const ID_TCS34725: u8 = 0x44;
/// ID of TCS34723 / TCS34727.
pub const ID_TCS34727: u8 = 0x4D;

const CMD: u8 = 0x80;
const CMD_AUTO_INC: u8 = 0xA0;
const CMD_CLEAR_INT: u8 = 0xE6;

const REG_ENABLE: u8 = 0x00;
const REG_ATIME: u8 = 0x01;
const REG_AILTL: u8 = 0x04;
const REG_PERS: u8 = 0x0C;
const REG_CONTROL: u8 = 0x0F;
const REG_ID: u8 = 0x12;
const REG_STATUS: u8 = 0x13;

const ENABLE_PON: u8 = 1 << 0;
const ENABLE_AEN: u8 = 1 << 1;
const ENABLE_AIEN: u8 = 1 << 4;

const STATUS_AVALID: u8 = 1 << 0;
const STATUS_AINT: u8 = 1 << 4;

/// Integration cycle (and RGBC init time) in microseconds.
pub const CYCLE_US: u32 = 2_400;

/// RGBC gain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Gain {
    X1,
    #[default]
    X4,
    X16,
    X60,
}

impl Gain {
    const ALL: [Gain; 4] = [Self::X1, Self::X4, Self::X16, Self::X60];

    /// Numeric gain.
    pub const fn factor(self) -> u32 {
        match self {
            Self::X1 => 1,
            Self::X4 => 4,
            Self::X16 => 16,
            Self::X60 => 60,
        }
    }
}

/// DN40 coefficients (`r`, `g`, `b` × 1000).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Dn40 {
    /// Red coefficient × 1000.
    pub r: i32,
    /// Green coefficient × 1000.
    pub g: i32,
    /// Blue coefficient × 1000.
    pub b: i32,
    /// Device factor.
    pub df: u32,
    /// CCT coefficient (K).
    pub ct_coef: u32,
    /// CCT offset (K).
    pub ct_offset: u32,
}

/// DN40 open-air coefficients for the TCS3472.
pub const DN40_TCS3472: Dn40 = Dn40 {
    r: 136,
    g: 1000,
    b: -444,
    df: 310,
    ct_coef: 3810,
    ct_offset: 1391,
};

/// Usable clear-channel maximum for `cycles` (1..=256): digital limit
/// `min(1024 · cycles, 65535)`, reduced to 75 % up to 63 cycles because of
/// ripple (DN40).
pub const fn saturation(cycles: u16) -> u32 {
    let sat = 1024 * cycles as u32;
    if sat > 65_535 {
        65_535
    } else if cycles <= 63 {
        sat - sat / 4
    } else {
        sat
    }
}

/// DN40 illuminance (milli-lux) and CCT (K; `None` when R' ≤ 0).
/// `ga_milli` is the glass attenuation × 1000.
pub fn dn40(
    rgbc: [u16; 4],
    gain: u32,
    integration_us: u32,
    ga_milli: u32,
    k: &Dn40,
) -> (u32, Option<u32>) {
    let [c, r, g, b] = rgbc.map(i64::from);
    let ir = (r + g + b - c).max(0) / 2;
    let (r1, g1, b1) = (r - ir, g - ir, b - ir);
    let g2 = k.r as i64 * r1 + k.g as i64 * g1 + k.b as i64 * b1;
    let den = integration_us as i64 * gain as i64;
    let mlx = if g2 <= 0 || den == 0 {
        0
    } else {
        (g2 * ga_milli as i64 * k.df as i64 / den).min(u32::MAX as i64) as u32
    };
    let cct = if r1 > 0 {
        Some((k.ct_coef as i64 * b1.max(0) / r1 + k.ct_offset as i64).min(u32::MAX as i64) as u32)
    } else {
        None
    };
    (mlx, cct)
}

/// Pick a gain for the next sample from the clear count: step down above
/// 90 % of [`saturation`], step up while the reading would stay below 50 %.
pub fn suggest_gain(current: Gain, cycles: u16, clear: u16) -> Gain {
    let sat = saturation(cycles) as u64;
    let clear = clear as u64;
    let idx = Gain::ALL.iter().position(|&g| g == current).unwrap_or(1);
    if clear * 10 > sat * 9 {
        return Gain::ALL[idx.saturating_sub(1)];
    }
    match Gain::ALL.get(idx + 1) {
        Some(&next) if clear * next.factor() as u64 / current.factor() as u64 * 2 < sat => next,
        _ => current,
    }
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux (DN40).
    pub illuminance: u32,
    /// Correlated colour temperature in kelvin (DN40), if computable.
    pub cct: Option<u32>,
    /// Clear count.
    pub clear: u16,
    /// Red count.
    pub red: u16,
    /// Green count.
    pub green: u16,
    /// Blue count.
    pub blue: u16,
    /// Clear channel at or above [`saturation`]; lux/CCT unreliable.
    pub saturated: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// TCS3472x driver.
        pub struct Tcs34725<I2C> {
            i2c: I2C,
            gain: Gain,
            cycles: u16,
            ga_milli: u32,
            coefficients: Dn40,
            interrupt: bool,
            continuous: bool,
        }

        impl<I2C> Tcs34725<I2C> {
            /// Create a driver (4×, 64 cycles = 153.6 ms); no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    gain: Gain::X4,
                    cycles: 64,
                    ga_milli: 1000,
                    coefficients: DN40_TCS3472,
                    interrupt: false,
                    continuous: false,
                }
            }

            /// Select the gain.
            pub const fn with_gain(mut self, gain: Gain) -> Self {
                self.gain = gain;
                self
            }

            /// Integration time in 2.4 ms cycles (clamped to 1..=256).
            pub const fn with_integration_cycles(mut self, cycles: u16) -> Self {
                self.cycles = if cycles == 0 {
                    1
                } else if cycles > 256 {
                    256
                } else {
                    cycles
                };
                self
            }

            /// Glass attenuation × 1000 (1000 = open air).
            pub const fn with_glass_attenuation(mut self, ga_milli: u32) -> Self {
                self.ga_milli = ga_milli;
                self
            }

            /// Replace the DN40 coefficients.
            pub const fn with_coefficients(mut self, coefficients: Dn40) -> Self {
                self.coefficients = coefficients;
                self
            }

            /// Keep measuring after a read (needed for the interrupt).
            pub const fn with_continuous(mut self, continuous: bool) -> Self {
                self.continuous = continuous;
                self
            }

            /// Change the gain for the next `start_measurement`.
            pub fn set_gain(&mut self, gain: Gain) {
                self.gain = gain;
            }

            /// Current gain.
            pub const fn gain(&self) -> Gain {
                self.gain
            }

            /// Integration time in microseconds.
            pub const fn integration_us(&self) -> u32 {
                self.cycles as u32 * CYCLE_US
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> Tcs34725<I2C> {
            $($async_)? fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(ADDRESS, &[CMD | reg, value])
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write_read(ADDRESS, &[CMD_AUTO_INC | reg], buf)
                    $($await_)*
                    .map_err(Error::bus)
            }

            /// Check the ID register (0x44 or 0x4D); returns it.
            pub $($async_)? fn probe(&mut self) -> Result<u8, Error<I2C::Error>> {
                let mut id = [0u8; 1];
                self.read(REG_ID, &mut id) $($await_)* ?;
                match id[0] {
                    ID_TCS34725 | ID_TCS34727 => Ok(id[0]),
                    other => Err(Error::UnexpectedId(other as u32)),
                }
            }

            /// Configure the clear-channel interrupt (`None` disables it from
            /// the next `start_measurement`). `persistence` is the raw
            /// `APERS` code 0..=15 (0 = every cycle, 4 = 5, 15 = 60 cycles).
            pub $($async_)? fn set_interrupt(
                &mut self,
                window: Option<(u16, u16)>,
                persistence: u8,
            ) -> Result<(), Error<I2C::Error>> {
                self.interrupt = window.is_some();
                let Some((low, high)) = window else {
                    return Ok(());
                };
                let [l0, l1] = low.to_le_bytes();
                let [h0, h1] = high.to_le_bytes();
                self.i2c
                    .write(ADDRESS, &[CMD_AUTO_INC | REG_AILTL, l0, l1, h0, h1])
                    $($await_)*
                    .map_err(Error::bus)?;
                self.write(REG_PERS, persistence & 0x0F) $($await_)*
            }

            /// Read the status; if the interrupt is set, clear it. Returns
            /// whether it was set.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<bool, Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_STATUS, &mut status) $($await_)* ?;
                if status[0] & STATUS_AINT == 0 {
                    return Ok(false);
                }
                self.i2c
                    .write(ADDRESS, &[CMD_CLEAR_INT])
                    $($await_)*
                    .map_err(Error::bus)?;
                Ok(true)
            }
        }

        impl<I2C: I2c> Sensor for Tcs34725<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Write ATIME / gain, then power on with RGBC enabled; the
            /// 2.4 ms warm-up precedes the first integration.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write(REG_ATIME, (256 - self.cycles) as u8) $($await_)* ?;
                self.write(REG_CONTROL, self.gain as u8) $($await_)* ?;
                let aien = if self.interrupt { ENABLE_AIEN } else { 0 };
                self.write(REG_ENABLE, ENABLE_PON | ENABLE_AEN | aien) $($await_)* ?;
                let t = CYCLE_US + self.integration_us();
                Ok(t + t / 10)
            }

            /// Check `AVALID`, read C/R/G/B and sleep (unless continuous).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut buf = [0u8; 9];
                self.read(REG_STATUS, &mut buf) $($await_)* ?;
                if buf[0] & STATUS_AVALID == 0 {
                    return Err(Error::NotReady);
                }
                if !self.continuous {
                    self.write(REG_ENABLE, 0) $($await_)* ?;
                }
                let ch = |i: usize| u16::from_le_bytes([buf[1 + 2 * i], buf[2 + 2 * i]]);
                let rgbc = [ch(0), ch(1), ch(2), ch(3)];
                let (illuminance, cct) = dn40(
                    rgbc,
                    self.gain.factor(),
                    self.integration_us(),
                    self.ga_milli,
                    &self.coefficients,
                );
                Ok(Measurement {
                    illuminance,
                    cct,
                    clear: rgbc[0],
                    red: rgbc[1],
                    green: rgbc[2],
                    blue: rgbc[3],
                    saturated: rgbc[0] as u32 >= saturation(self.cycles),
                })
            }
        }

        impl<I2C: I2c> LowPower for Tcs34725<I2C> {
            type BusError = I2C::Error;

            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_ENABLE, 0) $($await_)*
            }

            /// Samples power the part themselves; nothing to do.
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
        ADDRESS, CMD, CMD_AUTO_INC, CMD_CLEAR_INT, CYCLE_US, DN40_TCS3472, Dn40, ENABLE_AEN,
        ENABLE_AIEN, ENABLE_PON, Gain, ID_TCS34725, ID_TCS34727, Measurement, REG_AILTL, REG_ATIME,
        REG_CONTROL, REG_ENABLE, REG_ID, REG_PERS, REG_STATUS, STATUS_AINT, STATUS_AVALID, dn40,
        saturation,
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
    fn helpers() {
        // IR = 250 → R'=G'=B'=750 → G'' = 519; 153.6 ms, 1×.
        let (mlx, cct) = dn40([2500, 1000, 1000, 1000], 1, 153_600, 1000, &DN40_TCS3472);
        assert_eq!(mlx, 1_047_460);
        assert_eq!(cct, Some(5201));
        assert_eq!(dn40([0; 4], 1, 2400, 1000, &DN40_TCS3472), (0, None));
        assert_eq!(saturation(1), 768);
        assert_eq!(saturation(64), 65_535);
        assert_eq!(suggest_gain(Gain::X4, 64, 60_000), Gain::X1);
        assert_eq!(suggest_gain(Gain::X4, 64, 1_000), Gain::X16);
        assert_eq!(suggest_gain(Gain::X16, 64, 10_000), Gain::X16);
    }

    #[test]
    fn one_shot_cycle() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0xB2], vec![0x44]),
            Transaction::write(ADDRESS, vec![0x81, 0xC0]),
            Transaction::write(ADDRESS, vec![0x8F, 0x01]),
            Transaction::write(ADDRESS, vec![0x80, 0x03]),
            Transaction::write_read(ADDRESS, vec![0xB3], vec![0; 9]),
            Transaction::write_read(
                ADDRESS,
                vec![0xB3],
                vec![0x01, 0xC4, 0x09, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03],
            ),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut rgb = Tcs34725::new(&mut i2c);
        assert_eq!(rgb.probe().unwrap(), ID_TCS34725);
        assert_eq!(rgb.start_measurement().unwrap(), 171_600);
        assert_eq!(rgb.read_measurement(), Err(Error::NotReady));
        let m = rgb.read_measurement().unwrap();
        assert_eq!((m.clear, m.red, m.green, m.blue), (2500, 1000, 1000, 1000));
        assert_eq!(m.illuminance, 261_865);
        assert_eq!(m.cct, Some(5201));
        assert!(!m.saturated);
        i2c.done();
    }

    #[test]
    fn interrupts() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0xB2], vec![0x69]),
            Transaction::write(ADDRESS, vec![0xA4, 100, 0, 0x88, 0x13]),
            Transaction::write(ADDRESS, vec![0x8C, 0x04]),
            Transaction::write(ADDRESS, vec![0x81, 0x00]),
            Transaction::write(ADDRESS, vec![0x8F, 0x01]),
            Transaction::write(ADDRESS, vec![0x80, 0x13]),
            Transaction::write_read(ADDRESS, vec![0xB3], vec![0x11]),
            Transaction::write(ADDRESS, vec![0xE6]),
            Transaction::write_read(ADDRESS, vec![0xB3], vec![0x01]),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut rgb = Tcs34725::new(&mut i2c)
            .with_integration_cycles(300)
            .with_continuous(true);
        assert_eq!(rgb.probe(), Err(Error::UnexpectedId(0x69)));
        rgb.set_interrupt(Some((100, 5000)), 4).unwrap();
        assert_eq!(rgb.start_measurement().unwrap(), 678_480);
        assert!(rgb.take_interrupt().unwrap());
        assert!(!rgb.take_interrupt().unwrap());
        rgb.set_interrupt(None, 0).unwrap();
        rgb.sleep().unwrap();
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
            Transaction::write(ADDRESS, vec![0x81, 0xFF]),
            Transaction::write(ADDRESS, vec![0x8F, 0x03]),
            Transaction::write(ADDRESS, vec![0x80, 0x03]),
            Transaction::write_read(
                ADDRESS,
                vec![0xB3],
                vec![0x01, 0x20, 0x03, 0x2C, 0x01, 0x2C, 0x01, 0x2C, 0x01],
            ),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut rgb = asynch::Tcs34725::new(&mut i2c)
            .with_gain(Gain::X60)
            .with_integration_cycles(1);
        let mut delay = RecordingDelay(0);
        let m = block_on(rgb.measure(&mut delay)).unwrap();
        assert!(m.saturated);
        assert_eq!(m.illuminance, 372_430);
        assert_eq!(delay.0, 5_280);
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
