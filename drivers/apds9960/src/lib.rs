//! Broadcom APDS-9960 RGBC / ambient light + proximity sensor (I2C).
//!
//! `start_measurement` powers the part and enables the colour/ALS and/or
//! proximity engines, `read_measurement` checks the valid flags,
//! burst-reads status, C/R/G/B and `PDATA` in one transaction and powers
//! the part down again (1 µA typ. sleep). Gesture mode is not implemented.
//!
//! The datasheet gives no lux equation. Illuminance and CCT are computed
//! with the ams DN40 method (the RGBC engine is the same TAOS design as the
//! TCS3472), defaulting to the TCS3472 open-air coefficients
//! ([`DN40_DEFAULT`]); treat the result as an estimate and calibrate with
//! [`Apds9960::with_coefficients`] / [`Apds9960::with_glass_attenuation`].
//!
//! Reference: Broadcom AV02-4191EN "APDS-9960 Digital Proximity, Ambient
//! Light, RGB and Gesture Sensor" datasheet.
//!
//! ```ignore
//! let mut s = Apds9960::new(i2c);
//! s.probe()?;
//! let m = s.measure(&mut delay)?;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x39;
/// `ID` register values accepted by [`Apds9960::probe`] (0xAB per
/// datasheet; the others are seen on other silicon revisions / modules).
pub const IDS: [u8; 4] = [0xAB, 0x9C, 0xA8, 0x9E];

const REG_ENABLE: u8 = 0x80;
const REG_ATIME: u8 = 0x81;
const REG_AILTL: u8 = 0x84;
const REG_PILT: u8 = 0x89;
const REG_PIHT: u8 = 0x8B;
const REG_PERS: u8 = 0x8C;
const REG_PPULSE: u8 = 0x8E;
const REG_CONTROL: u8 = 0x8F;
const REG_ID: u8 = 0x92;
const REG_STATUS: u8 = 0x93;
const REG_POFFSET_UR: u8 = 0x9D;
const REG_AICLEAR: u8 = 0xE7;

const ENABLE_PON: u8 = 1 << 0;
const ENABLE_AEN: u8 = 1 << 1;
const ENABLE_PEN: u8 = 1 << 2;
const ENABLE_AIEN: u8 = 1 << 4;
const ENABLE_PIEN: u8 = 1 << 5;

const STATUS_CPSAT: u8 = 1 << 7;
const STATUS_PGSAT: u8 = 1 << 6;
const STATUS_PINT: u8 = 1 << 5;
const STATUS_AINT: u8 = 1 << 4;
const STATUS_PVALID: u8 = 1 << 1;
const STATUS_AVALID: u8 = 1 << 0;

/// ALS integration cycle in microseconds.
pub const CYCLE_US: u32 = 2_780;
/// Exit-sleep pause before the first engine runs.
const EXIT_SLEEP_US: u32 = 7_000;

/// ALS / colour gain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Gain {
    X1,
    #[default]
    X4,
    X16,
    X64,
}

impl Gain {
    const ALL: [Gain; 4] = [Self::X1, Self::X4, Self::X16, Self::X64];

    /// Numeric gain.
    pub const fn factor(self) -> u32 {
        match self {
            Self::X1 => 1,
            Self::X4 => 4,
            Self::X16 => 16,
            Self::X64 => 64,
        }
    }
}

/// Proximity receiver gain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum ProxGain {
    X1,
    X2,
    #[default]
    X4,
    X8,
}

/// IR LED drive current.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum LedDrive {
    #[default]
    Ma100,
    Ma50,
    Ma25,
    Ma12_5,
}

/// Proximity pulse length.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum PulseLength {
    Us4,
    Us8,
    #[default]
    Us16,
    Us32,
}

/// Proximity engine settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Proximity {
    /// Receiver gain.
    pub gain: ProxGain,
    /// LED current.
    pub led: LedDrive,
    /// Pulse length.
    pub pulse_length: PulseLength,
    /// Pulse count 1..=64.
    pub pulses: u8,
}

impl Default for Proximity {
    /// 4×, 100 mA, 16 µs × 10 pulses.
    fn default() -> Self {
        Self {
            gain: ProxGain::X4,
            led: LedDrive::Ma100,
            pulse_length: PulseLength::Us16,
            pulses: 10,
        }
    }
}

impl Proximity {
    /// Proximity result time `t_INIT + t_CNVT + PPULSE · t_ACC` (Table 2),
    /// rounded up to µs.
    pub const fn result_us(&self) -> u32 {
        // 0.1 µs units.
        let (init, acc) = match self.pulse_length {
            PulseLength::Us4 => (408, 286),
            PulseLength::Us8 => (449, 367),
            PulseLength::Us16 => (530, 531),
            PulseLength::Us32 => (694, 857),
        };
        (init + 7966 + self.pulses_clamped() as u32 * acc).div_ceil(10)
    }

    const fn pulses_clamped(&self) -> u8 {
        if self.pulses == 0 {
            1
        } else if self.pulses > 64 {
            64
        } else {
            self.pulses
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

/// Default coefficients: ams DN40 TCS3472 open-air values (no APDS-9960
/// values are published).
pub const DN40_DEFAULT: Dn40 = Dn40 {
    r: 136,
    g: 1000,
    b: -444,
    df: 310,
    ct_coef: 3810,
    ct_offset: 1391,
};

/// Maximum clear count for `cycles` (1..=256): `min(1025 · cycles, 65535)`.
pub const fn max_count(cycles: u16) -> u32 {
    let max = 1025 * cycles as u32;
    if max > 65_535 { 65_535 } else { max }
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

/// Pick an ALS gain for the next sample from the clear count: step down
/// above 90 % of [`max_count`], step up while the reading would stay below
/// 50 %.
pub fn suggest_gain(current: Gain, cycles: u16, clear: u16) -> Gain {
    let max = max_count(cycles) as u64;
    let clear = clear as u64;
    let idx = Gain::ALL.iter().position(|&g| g == current).unwrap_or(1);
    if clear * 10 > max * 9 {
        return Gain::ALL[idx.saturating_sub(1)];
    }
    match Gain::ALL.get(idx + 1) {
        Some(&next) if clear * next.factor() as u64 / current.factor() as u64 * 2 < max => next,
        _ => current,
    }
}

/// Interrupt window (raw counts; proximity thresholds are clamped to 255).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Window {
    /// Lower threshold.
    pub low: u16,
    /// Upper threshold.
    pub high: u16,
    /// Raw persistence code 0..=15 (`APERS`: 0, 1, 2, 3, 5, 10 … 60
    /// samples; `PPERS`: 0..=15 samples).
    pub persistence: u8,
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux (DN40 estimate), if ALS enabled.
    pub illuminance: Option<u32>,
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
    /// Proximity count 0..=255, if proximity enabled.
    pub proximity: Option<u8>,
    /// Clear channel analog (`CPSAT`) or digital saturation.
    pub als_saturated: bool,
    /// Proximity analog saturation (`PGSAT`).
    pub ps_saturated: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if let Some(lux) = self.illuminance {
            f(Quantity::Illuminance(lux));
        }
        if let Some(ps) = self.proximity {
            f(Quantity::Proximity(ps as u32));
        }
    }
}

const fn sign_magnitude(v: i8) -> u8 {
    if v < 0 {
        0x80 | (v.unsigned_abs() & 0x7F)
    } else {
        v as u8
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// APDS-9960 driver.
        pub struct Apds9960<I2C> {
            i2c: I2C,
            gain: Gain,
            cycles: u16,
            als: bool,
            proximity: Option<Proximity>,
            ga_milli: u32,
            coefficients: Dn40,
            interrupts: u8,
            continuous: bool,
        }

        impl<I2C> Apds9960<I2C> {
            /// Create a driver (ALS 4× / 37 cycles = 103 ms, proximity with
            /// [`Proximity::default`]); no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    gain: Gain::X4,
                    cycles: 37,
                    als: true,
                    proximity: Some(Proximity {
                        gain: ProxGain::X4,
                        led: LedDrive::Ma100,
                        pulse_length: PulseLength::Us16,
                        pulses: 10,
                    }),
                    ga_milli: 1000,
                    coefficients: DN40_DEFAULT,
                    interrupts: 0,
                    continuous: false,
                }
            }

            /// Select the ALS gain.
            pub const fn with_gain(mut self, gain: Gain) -> Self {
                self.gain = gain;
                self
            }

            /// ALS integration in 2.78 ms cycles (clamped to 1..=256).
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

            /// Enable / disable the colour+ALS engine.
            pub const fn with_als(mut self, enabled: bool) -> Self {
                self.als = enabled;
                self
            }

            /// Proximity settings (`None` disables proximity).
            pub const fn with_proximity(mut self, proximity: Option<Proximity>) -> Self {
                self.proximity = proximity;
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

            /// Keep measuring after a read (needed for the interrupts).
            pub const fn with_continuous(mut self, continuous: bool) -> Self {
                self.continuous = continuous;
                self
            }

            /// Change the ALS gain for the next `start_measurement`.
            pub fn set_gain(&mut self, gain: Gain) {
                self.gain = gain;
            }

            /// Current ALS gain.
            pub const fn gain(&self) -> Gain {
                self.gain
            }

            /// ALS integration time in microseconds.
            pub const fn integration_us(&self) -> u32 {
                self.cycles as u32 * CYCLE_US
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> Apds9960<I2C> {
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

            /// Check the `ID` register against [`IDS`]; returns it.
            pub $($async_)? fn probe(&mut self) -> Result<u8, Error<I2C::Error>> {
                let mut id = [0u8; 1];
                self.read(REG_ID, &mut id) $($await_)* ?;
                if IDS.contains(&id[0]) {
                    Ok(id[0])
                } else {
                    Err(Error::UnexpectedId(id[0] as u32))
                }
            }

            /// Proximity crosstalk offsets (UP/RIGHT, DOWN/LEFT pairs),
            /// −127..=127; positive values reduce the result.
            pub $($async_)? fn set_proximity_offset(
                &mut self,
                up_right: i8,
                down_left: i8,
            ) -> Result<(), Error<I2C::Error>> {
                self.write(REG_POFFSET_UR, sign_magnitude(up_right)) $($await_)* ?;
                self.write(REG_POFFSET_UR + 1, sign_magnitude(down_left)) $($await_)*
            }

            /// Configure the clear-channel (`als`) and proximity (`ps`)
            /// interrupts; they take effect at the next `start_measurement`.
            pub $($async_)? fn set_interrupt(
                &mut self,
                als: Option<Window>,
                ps: Option<Window>,
            ) -> Result<(), Error<I2C::Error>> {
                let mut pers = 0;
                self.interrupts = 0;
                if let Some(w) = als {
                    let [l0, l1] = w.low.to_le_bytes();
                    let [h0, h1] = w.high.to_le_bytes();
                    for (i, v) in [l0, l1, h0, h1].into_iter().enumerate() {
                        self.write(REG_AILTL + i as u8, v) $($await_)* ?;
                    }
                    pers |= w.persistence & 0x0F;
                    self.interrupts |= ENABLE_AIEN;
                }
                if let Some(w) = ps {
                    self.write(REG_PILT, w.low.min(255) as u8) $($await_)* ?;
                    self.write(REG_PIHT, w.high.min(255) as u8) $($await_)* ?;
                    pers |= (w.persistence & 0x0F) << 4;
                    self.interrupts |= ENABLE_PIEN;
                }
                self.write(REG_PERS, pers) $($await_)*
            }

            /// Read the status; if an ALS or proximity interrupt is set,
            /// clear all of them (`AICLEAR`). Returns whether one was set.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<bool, Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_STATUS, &mut status) $($await_)* ?;
                if status[0] & (STATUS_AINT | STATUS_PINT) == 0 {
                    return Ok(false);
                }
                self.i2c
                    .write(ADDRESS, &[REG_AICLEAR])
                    $($await_)*
                    .map_err(Error::bus)?;
                Ok(true)
            }
        }

        impl<I2C: I2c> Sensor for Apds9960<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Write ATIME / pulses / gains, then power on with the selected
            /// engines.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if !self.als && self.proximity.is_none() {
                    return Err(Error::Unsupported);
                }
                let mut enable = ENABLE_PON | self.interrupts;
                let mut control = self.gain as u8;
                let mut t = EXIT_SLEEP_US;
                if self.als {
                    self.write(REG_ATIME, (256 - self.cycles) as u8) $($await_)* ?;
                    enable |= ENABLE_AEN;
                    t += self.integration_us();
                }
                if let Some(p) = self.proximity {
                    let ppulse = (p.pulse_length as u8) << 6 | (p.pulses_clamped() - 1);
                    self.write(REG_PPULSE, ppulse) $($await_)* ?;
                    control |= (p.led as u8) << 6 | (p.gain as u8) << 2;
                    enable |= ENABLE_PEN;
                    t += p.result_us();
                }
                self.write(REG_CONTROL, control) $($await_)* ?;
                self.write(REG_ENABLE, enable) $($await_)* ?;
                Ok(t + t / 10)
            }

            /// Check the valid flags, read status..PDATA and power down
            /// (unless continuous).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut buf = [0u8; 10];
                self.read(REG_STATUS, &mut buf) $($await_)* ?;
                let s = buf[0];
                let ps = self.proximity.is_some();
                if (self.als && s & STATUS_AVALID == 0) || (ps && s & STATUS_PVALID == 0) {
                    return Err(Error::NotReady);
                }
                if !self.continuous {
                    self.write(REG_ENABLE, 0) $($await_)* ?;
                }
                let ch = |i: usize| u16::from_le_bytes([buf[1 + 2 * i], buf[2 + 2 * i]]);
                let rgbc = [ch(0), ch(1), ch(2), ch(3)];
                let mut m = Measurement {
                    illuminance: None,
                    cct: None,
                    clear: rgbc[0],
                    red: rgbc[1],
                    green: rgbc[2],
                    blue: rgbc[3],
                    proximity: None,
                    als_saturated: false,
                    ps_saturated: false,
                };
                if self.als {
                    let (lux, cct) = dn40(
                        rgbc,
                        self.gain.factor(),
                        self.integration_us(),
                        self.ga_milli,
                        &self.coefficients,
                    );
                    m.illuminance = Some(lux);
                    m.cct = cct;
                    m.als_saturated =
                        s & STATUS_CPSAT != 0 || rgbc[0] as u32 >= max_count(self.cycles);
                }
                if ps {
                    m.proximity = Some(buf[9]);
                    m.ps_saturated = s & STATUS_PGSAT != 0;
                }
                Ok(m)
            }
        }

        impl<I2C: I2c> LowPower for Apds9960<I2C> {
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
        ADDRESS, CYCLE_US, DN40_DEFAULT, Dn40, ENABLE_AEN, ENABLE_AIEN, ENABLE_PEN, ENABLE_PIEN,
        ENABLE_PON, EXIT_SLEEP_US, Gain, IDS, LedDrive, Measurement, ProxGain, Proximity,
        PulseLength, REG_AICLEAR, REG_AILTL, REG_ATIME, REG_CONTROL, REG_ENABLE, REG_ID, REG_PERS,
        REG_PIHT, REG_PILT, REG_POFFSET_UR, REG_PPULSE, REG_STATUS, STATUS_AINT, STATUS_AVALID,
        STATUS_CPSAT, STATUS_PGSAT, STATUS_PINT, STATUS_PVALID, Window, dn40, max_count,
        sign_magnitude,
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
        assert_eq!(Proximity::default().result_us(), 1381);
        let p = Proximity {
            pulse_length: PulseLength::Us32,
            pulses: 64,
            ..Proximity::default()
        };
        assert_eq!(p.result_us(), 6351);
        assert_eq!(max_count(1), 1025);
        assert_eq!(max_count(256), 65_535);
        assert_eq!(
            dn40([2500, 1000, 1000, 1000], 1, 153_600, 1000, &DN40_DEFAULT),
            (1_047_460, Some(5201))
        );
        assert_eq!(sign_magnitude(-3), 0x83);
        assert_eq!(sign_magnitude(-128), 0x80);
        assert_eq!(suggest_gain(Gain::X4, 37, 37_000), Gain::X1);
        assert_eq!(suggest_gain(Gain::X4, 37, 1_000), Gain::X16);
        assert_eq!(suggest_gain(Gain::X64, 37, 10), Gain::X64);
    }

    #[test]
    fn one_shot_als_and_proximity() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x92], vec![0xAB]),
            Transaction::write(ADDRESS, vec![0x81, 0xDB]),
            Transaction::write(ADDRESS, vec![0x8E, 0x89]),
            Transaction::write(ADDRESS, vec![0x8F, 0x09]),
            Transaction::write(ADDRESS, vec![0x80, 0x07]),
            Transaction::write_read(ADDRESS, vec![0x93], vec![0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
            Transaction::write_read(
                ADDRESS,
                vec![0x93],
                vec![0x43, 0xC4, 0x09, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0x80],
            ),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut s = Apds9960::new(&mut i2c);
        assert_eq!(s.probe().unwrap(), 0xAB);
        assert_eq!(s.start_measurement().unwrap(), 122_365);
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        let m = s.read_measurement().unwrap();
        assert_eq!((m.clear, m.red, m.green, m.blue), (2500, 1000, 1000, 1000));
        assert_eq!(m.illuminance, Some(391_041));
        assert_eq!(m.cct, Some(5201));
        assert_eq!(m.proximity, Some(0x80));
        assert!(!m.als_saturated);
        assert!(m.ps_saturated);
        i2c.done();
    }

    #[test]
    fn interrupts_and_offsets() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x92], vec![0x55]),
            Transaction::write(ADDRESS, vec![0x84, 100]),
            Transaction::write(ADDRESS, vec![0x85, 0]),
            Transaction::write(ADDRESS, vec![0x86, 0x88]),
            Transaction::write(ADDRESS, vec![0x87, 0x13]),
            Transaction::write(ADDRESS, vec![0x89, 10]),
            Transaction::write(ADDRESS, vec![0x8B, 255]),
            Transaction::write(ADDRESS, vec![0x8C, 0x32]),
            Transaction::write(ADDRESS, vec![0x9D, 0x05]),
            Transaction::write(ADDRESS, vec![0x9E, 0x83]),
            Transaction::write(ADDRESS, vec![0x8E, 0x07]),
            Transaction::write(ADDRESS, vec![0x8F, 0xC1]),
            Transaction::write(ADDRESS, vec![0x80, 0x35]),
            Transaction::write_read(ADDRESS, vec![0x93], vec![0x20]),
            Transaction::write(ADDRESS, vec![0xE7]),
            Transaction::write_read(ADDRESS, vec![0x93], vec![0x02]),
        ];
        let mut i2c = Mock::new(&expectations);
        let prox = Proximity {
            gain: ProxGain::X1,
            led: LedDrive::Ma12_5,
            pulse_length: PulseLength::Us4,
            pulses: 8,
        };
        let mut s = Apds9960::new(&mut i2c)
            .with_als(false)
            .with_proximity(Some(prox))
            .with_continuous(true);
        assert_eq!(s.probe(), Err(Error::UnexpectedId(0x55)));
        let als = Window {
            low: 100,
            high: 5000,
            persistence: 2,
        };
        let ps = Window {
            low: 10,
            high: 300,
            persistence: 3,
        };
        s.set_interrupt(Some(als), Some(ps)).unwrap();
        s.set_proximity_offset(5, -3).unwrap();
        assert_eq!(s.start_measurement().unwrap(), 8_873);
        assert!(s.take_interrupt().unwrap());
        assert!(!s.take_interrupt().unwrap());
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
                vec![0x93],
                vec![0x81, 0xE8, 0x03, 0x90, 0x01, 0x90, 0x01, 0x90, 0x01, 0],
            ),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut s = asynch::Apds9960::new(&mut i2c)
            .with_proximity(None)
            .with_gain(Gain::X64)
            .with_integration_cycles(1);
        let mut delay = RecordingDelay(0);
        let m = block_on(s.measure(&mut delay)).unwrap();
        assert!(m.als_saturated);
        assert_eq!(m.illuminance, Some(361_713));
        assert_eq!(m.proximity, None);
        assert_eq!(delay.0, 10_758);
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
