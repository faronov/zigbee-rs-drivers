//! ams-OSRAM AS7341 11-channel spectral sensor (I2C).
//!
//! The part has six ADCs and twelve photodiode groups. The sensor
//! multiplexer (SMUX) maps one [`Bank`] per measurement:
//! F1–F4 + Clear + NIR, or F5–F8 + Clear + NIR. Read the full spectrum
//! with two samples, either with [`As7341::set_bank`] or with
//! [`As7341::with_alternate`], and combine them with [`Measurement::merge`].
//!
//! `start_measurement` powers the part, writes the integration time and
//! gain, loads the SMUX table, waits for the SMUX command to finish (a
//! bounded `ENABLE` poll, no delay) and starts one integration.
//! `read_measurement` checks `AVALID`, burst-reads `ASTATUS` + CH0..CH5 and
//! powers down (0.7 µA typ. sleep).
//!
//! The raw counts are the main output. The datasheet gives no lux formula,
//! so illuminance is only reported when a clear-channel calibration factor
//! is set with [`As7341::with_lux_factor`].
//!
//! Reference: ams DS000504 "AS7341 11-Channel Multi-Spectral Digital
//! Sensor" datasheet. The SMUX tables match the Adafruit_AS7341 library.
//!
//! ```ignore
//! let mut s = As7341::new(i2c).with_alternate(true);
//! s.probe()?;
//! let low = s.measure(&mut delay)?;
//! let all = s.measure(&mut delay)?.merge(&low);
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x39;
/// Value of `ID` bits 7:2.
pub const PART_ID: u8 = 0b00_1001;

const REG_ENABLE: u8 = 0x80;
const REG_ATIME: u8 = 0x81;
const REG_SP_TH_L: u8 = 0x84;
const REG_ID: u8 = 0x92;
const REG_STATUS: u8 = 0x93;
const REG_ASTATUS: u8 = 0x94;
const REG_STATUS2: u8 = 0xA3;
const REG_CFG1: u8 = 0xAA;
const REG_CFG6: u8 = 0xAF;
const REG_CFG12: u8 = 0xB5;
const REG_PERS: u8 = 0xBD;
const REG_ASTEP: u8 = 0xCA;
const REG_INTENAB: u8 = 0xF9;

const ENABLE_PON: u8 = 1 << 0;
const ENABLE_SP_EN: u8 = 1 << 1;
const ENABLE_SMUXEN: u8 = 1 << 4;
const SMUX_CMD_WRITE: u8 = 2 << 3;
const STATUS_AINT: u8 = 1 << 3;
const STATUS2_AVALID: u8 = 1 << 6;
const ASTATUS_ASAT: u8 = 1 << 7;
const INTENAB_SP_IEN: u8 = 1 << 3;
/// ADC that carries the clear channel in both SMUX tables.
const CLEAR_ADC: u8 = 4;

/// Maximum number of `ENABLE` reads while waiting for the SMUX command.
pub const SMUX_POLLS: u8 = 16;
/// Margin added to the integration time.
const START_US: u32 = 1_000;

/// SMUX RAM (0x00..=0x13) for F1–F4 + Clear + NIR on ADC0..ADC5.
pub const SMUX_F1_F4: [u8; 20] = [
    0x30, 0x01, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x20, 0x04, 0x00, 0x30,
    0x01, 0x50, 0x00, 0x06,
];
/// SMUX RAM (0x00..=0x13) for F5–F8 + Clear + NIR on ADC0..ADC5.
pub const SMUX_F5_F8: [u8; 20] = [
    0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x10, 0x03, 0x50, 0x10, 0x03, 0x00, 0x00, 0x00, 0x24, 0x00,
    0x00, 0x50, 0x00, 0x06,
];

/// Photodiode set measured in one integration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Bank {
    /// F1–F4, Clear, NIR.
    #[default]
    F1F4,
    /// F5–F8, Clear, NIR.
    F5F8,
}

impl Bank {
    /// SMUX table for this bank.
    pub const fn smux(self) -> &'static [u8; 20] {
        match self {
            Self::F1F4 => &SMUX_F1_F4,
            Self::F5F8 => &SMUX_F5_F8,
        }
    }

    const fn other(self) -> Self {
        match self {
            Self::F1F4 => Self::F5F8,
            Self::F5F8 => Self::F1F4,
        }
    }
}

/// Spectral gain (`AGAIN`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Gain {
    X0_5,
    X1,
    X2,
    X4,
    X8,
    X16,
    X32,
    #[default]
    X64,
    X128,
    X256,
    X512,
}

impl Gain {
    const ALL: [Gain; 11] = [
        Self::X0_5,
        Self::X1,
        Self::X2,
        Self::X4,
        Self::X8,
        Self::X16,
        Self::X32,
        Self::X64,
        Self::X128,
        Self::X256,
        Self::X512,
    ];

    /// Gain × 2 (1 for 0.5×, 1024 for 512×).
    pub const fn doubled(self) -> u32 {
        1 << self as u32
    }
}

/// Integration time `(ATIME + 1) · (ASTEP + 1) · 2.78 µs` in µs, rounded up.
pub const fn integration_us(atime: u8, astep: u16) -> u32 {
    let steps = (atime as u64 + 1) * (astep as u64 + 1);
    (steps * 2_780).div_ceil(1_000) as u32
}

/// ADC full scale `min((ATIME + 1) · (ASTEP + 1), 65535)`.
pub const fn full_scale(atime: u8, astep: u16) -> u32 {
    let steps = (atime as u32 + 1) * (astep as u32 + 1);
    if steps > 65_535 { 65_535 } else { steps }
}

/// Clear counts to milli-lux using a calibration `factor` in milli-lux
/// per basic count (counts / (gain · ms)).
pub fn clear_to_millilux(clear: u16, gain: Gain, integration_us: u32, factor: u32) -> u32 {
    let den = gain.doubled() as u64 * integration_us as u64;
    if den == 0 {
        return 0;
    }
    (clear as u64 * factor as u64 * 2_000 / den).min(u32::MAX as u64) as u32
}

/// Pick a gain for the next sample from the largest channel count: step
/// down above 90 % of `full_scale`, step up while the reading would stay
/// below 50 %.
pub fn suggest_gain(current: Gain, full_scale: u32, peak: u16) -> Gain {
    let idx = current as usize;
    let peak = peak as u64;
    let max = full_scale as u64;
    if peak * 10 > max * 9 {
        return Gain::ALL[idx.saturating_sub(1)];
    }
    match Gain::ALL.get(idx + 1) {
        Some(_) if peak * 4 < max => Gain::ALL[idx + 1],
        _ => current,
    }
}

/// Clear-channel interrupt window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Window {
    /// Lower threshold.
    pub low: u16,
    /// Upper threshold.
    pub high: u16,
    /// Raw `APERS` code 0..=15 (0 = every cycle).
    pub persistence: u8,
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Bank measured.
    pub bank: Bank,
    /// F1..F8 counts (index 0 = F1); channels outside the bank are `None`.
    pub f: [Option<u16>; 8],
    /// Clear count.
    pub clear: u16,
    /// Near-IR count.
    pub nir: u16,
    /// Illuminance in milli-lux, if a lux factor is set.
    pub illuminance: Option<u32>,
    /// Analog or digital saturation (`ASAT_STATUS`).
    pub saturated: bool,
}

impl Measurement {
    /// Fill the channels missing from `self` with those of `other`
    /// (normally the previous sample of the other bank).
    pub fn merge(mut self, other: &Measurement) -> Measurement {
        for (a, b) in self.f.iter_mut().zip(other.f) {
            if a.is_none() {
                *a = b;
            }
        }
        self
    }

    /// Largest raw count in this sample.
    pub fn peak(&self) -> u16 {
        self.f
            .iter()
            .flatten()
            .copied()
            .fold(self.clear.max(self.nir), u16::max)
    }
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if let Some(lux) = self.illuminance {
            f(Quantity::Illuminance(lux));
        }
    }
}

fn decode(bank: Bank, buf: &[u8; 13]) -> Measurement {
    let ch = |i: usize| u16::from_le_bytes([buf[1 + 2 * i], buf[2 + 2 * i]]);
    let mut f = [None; 8];
    let base = match bank {
        Bank::F1F4 => 0,
        Bank::F5F8 => 4,
    };
    for (i, slot) in f[base..base + 4].iter_mut().enumerate() {
        *slot = Some(ch(i));
    }
    Measurement {
        bank,
        f,
        clear: ch(4),
        nir: ch(5),
        illuminance: None,
        saturated: buf[0] & ASTATUS_ASAT != 0,
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// AS7341 driver.
        pub struct As7341<I2C> {
            i2c: I2C,
            gain: Gain,
            atime: u8,
            astep: u16,
            bank: Bank,
            alternate: bool,
            lux_factor: Option<u32>,
            continuous: bool,
        }

        impl<I2C> As7341<I2C> {
            /// Create a driver (64×, ATIME 29 / ASTEP 599 = 50 ms, bank
            /// F1–F4); no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    gain: Gain::X64,
                    atime: 29,
                    astep: 599,
                    bank: Bank::F1F4,
                    alternate: false,
                    lux_factor: None,
                    continuous: false,
                }
            }

            /// Select the gain.
            pub const fn with_gain(mut self, gain: Gain) -> Self {
                self.gain = gain;
                self
            }

            /// Integration: `ATIME` (steps − 1) and `ASTEP` (2.78 µs units
            /// − 1, 65535 is reserved and clamped to 65534).
            pub const fn with_integration(mut self, atime: u8, astep: u16) -> Self {
                self.atime = atime;
                self.astep = if astep == u16::MAX { u16::MAX - 1 } else { astep };
                self
            }

            /// Bank for the next sample.
            pub const fn with_bank(mut self, bank: Bank) -> Self {
                self.bank = bank;
                self
            }

            /// Switch bank after every read.
            pub const fn with_alternate(mut self, alternate: bool) -> Self {
                self.alternate = alternate;
                self
            }

            /// Clear-channel calibration in milli-lux per basic count
            /// (counts / (gain · ms)); enables [`Measurement::illuminance`].
            pub const fn with_lux_factor(mut self, factor: Option<u32>) -> Self {
                self.lux_factor = factor;
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

            /// Change the bank for the next `start_measurement`.
            pub fn set_bank(&mut self, bank: Bank) {
                self.bank = bank;
            }

            /// Bank of the next sample.
            pub const fn bank(&self) -> Bank {
                self.bank
            }

            /// Integration time in microseconds.
            pub const fn integration_us(&self) -> u32 {
                integration_us(self.atime, self.astep)
            }

            /// ADC full scale for the current integration.
            pub const fn full_scale(&self) -> u32 {
                full_scale(self.atime, self.astep)
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> As7341<I2C> {
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

            /// Check `ID` bits 7:2 against [`PART_ID`]; returns the register.
            pub $($async_)? fn probe(&mut self) -> Result<u8, Error<I2C::Error>> {
                let mut id = [0u8; 1];
                self.read(REG_ID, &mut id) $($await_)* ?;
                if id[0] >> 2 == PART_ID {
                    Ok(id[0])
                } else {
                    Err(Error::UnexpectedId(id[0] as u32))
                }
            }

            /// Configure (or disable with `None`) the clear-channel (ADC4)
            /// threshold interrupt.
            pub $($async_)? fn set_interrupt(&mut self, window: Option<Window>) -> Result<(), Error<I2C::Error>> {
                let Some(w) = window else {
                    return self.write(REG_INTENAB, 0) $($await_)*;
                };
                let [l0, l1] = w.low.to_le_bytes();
                let [h0, h1] = w.high.to_le_bytes();
                for (i, v) in [l0, l1, h0, h1].into_iter().enumerate() {
                    self.write(REG_SP_TH_L + i as u8, v) $($await_)* ?;
                }
                self.write(REG_CFG12, CLEAR_ADC) $($await_)* ?;
                self.write(REG_PERS, w.persistence & 0x0F) $($await_)* ?;
                self.write(REG_INTENAB, INTENAB_SP_IEN) $($await_)*
            }

            /// Read `STATUS`; if the spectral interrupt is set, clear it.
            /// Returns whether it was set.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<bool, Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_STATUS, &mut status) $($await_)* ?;
                if status[0] & STATUS_AINT == 0 {
                    return Ok(false);
                }
                self.write(REG_STATUS, STATUS_AINT) $($await_)* ?;
                Ok(true)
            }
        }

        impl<I2C: I2c> Sensor for As7341<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Power on, write ATIME/ASTEP/AGAIN and the SMUX table, run the
            /// SMUX command and start one integration. Returns
            /// [`Error::Timeout`] if the SMUX command does not finish within
            /// [`SMUX_POLLS`] reads.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write(REG_ENABLE, ENABLE_PON) $($await_)* ?;
                self.write(REG_ATIME, self.atime) $($await_)* ?;
                let [s0, s1] = self.astep.to_le_bytes();
                self.write(REG_ASTEP, s0) $($await_)* ?;
                self.write(REG_ASTEP + 1, s1) $($await_)* ?;
                self.write(REG_CFG1, self.gain as u8) $($await_)* ?;
                self.write(REG_CFG6, SMUX_CMD_WRITE) $($await_)* ?;
                for (i, v) in self.bank.smux().iter().enumerate() {
                    self.write(i as u8, *v) $($await_)* ?;
                }
                self.write(REG_ENABLE, ENABLE_PON | ENABLE_SMUXEN) $($await_)* ?;
                let mut done = false;
                for _ in 0..SMUX_POLLS {
                    let mut enable = [0u8; 1];
                    self.read(REG_ENABLE, &mut enable) $($await_)* ?;
                    if enable[0] & ENABLE_SMUXEN == 0 {
                        done = true;
                        break;
                    }
                }
                if !done {
                    return Err(Error::Timeout);
                }
                self.write(REG_ENABLE, ENABLE_PON | ENABLE_SP_EN) $($await_)* ?;
                let t = self.integration_us();
                Ok(t + t / 10 + START_US)
            }

            /// Check `AVALID`, read `ASTATUS` + CH0..CH5, power down (unless
            /// continuous) and switch bank if alternating.
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_STATUS2, &mut status) $($await_)* ?;
                if status[0] & STATUS2_AVALID == 0 {
                    return Err(Error::NotReady);
                }
                let mut buf = [0u8; 13];
                self.read(REG_ASTATUS, &mut buf) $($await_)* ?;
                if !self.continuous {
                    self.write(REG_ENABLE, 0) $($await_)* ?;
                }
                let mut m = decode(self.bank, &buf);
                if let Some(k) = self.lux_factor {
                    m.illuminance = Some(clear_to_millilux(m.clear, self.gain, self.integration_us(), k));
                }
                if self.alternate {
                    self.bank = self.bank.other();
                }
                Ok(m)
            }
        }

        impl<I2C: I2c> LowPower for As7341<I2C> {
            type BusError = I2C::Error;

            /// Clear `PON` (sleep state; registers are kept).
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
        ADDRESS, Bank, CLEAR_ADC, ENABLE_PON, ENABLE_SMUXEN, ENABLE_SP_EN, Gain, INTENAB_SP_IEN,
        Measurement, PART_ID, REG_ASTATUS, REG_ASTEP, REG_ATIME, REG_CFG1, REG_CFG6, REG_CFG12,
        REG_ENABLE, REG_ID, REG_INTENAB, REG_PERS, REG_SP_TH_L, REG_STATUS, REG_STATUS2,
        SMUX_CMD_WRITE, SMUX_POLLS, START_US, STATUS_AINT, STATUS2_AVALID, Window,
        clear_to_millilux, decode, full_scale, integration_us,
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

    fn start(atime: u8, astep: u16, gain: u8, smux: &[u8; 20], polls: &[u8]) -> Vec<Transaction> {
        let [s0, s1] = astep.to_le_bytes();
        let mut t = vec![
            Transaction::write(ADDRESS, vec![0x80, 0x01]),
            Transaction::write(ADDRESS, vec![0x81, atime]),
            Transaction::write(ADDRESS, vec![0xCA, s0]),
            Transaction::write(ADDRESS, vec![0xCB, s1]),
            Transaction::write(ADDRESS, vec![0xAA, gain]),
            Transaction::write(ADDRESS, vec![0xAF, 0x10]),
        ];
        for (i, v) in smux.iter().enumerate() {
            t.push(Transaction::write(ADDRESS, vec![i as u8, *v]));
        }
        t.push(Transaction::write(ADDRESS, vec![0x80, 0x11]));
        for p in polls {
            t.push(Transaction::write_read(ADDRESS, vec![0x80], vec![*p]));
        }
        t
    }

    fn data(astatus: u8, ch: [u16; 6]) -> Vec<u8> {
        let mut v = vec![astatus];
        for c in ch {
            v.extend_from_slice(&c.to_le_bytes());
        }
        v
    }

    #[test]
    fn helpers() {
        assert_eq!(integration_us(29, 599), 50_040);
        assert_eq!(integration_us(0, 0), 3);
        assert_eq!(full_scale(29, 599), 18_000);
        assert_eq!(full_scale(255, 999), 65_535);
        assert_eq!(Gain::X0_5.doubled(), 1);
        assert_eq!(Gain::X512.doubled(), 1024);
        // 1000 counts at 64× / 50 ms = 0.3125 basic counts; × 2000 mlx.
        assert_eq!(clear_to_millilux(1000, Gain::X64, 50_000, 2_000), 625);
        assert_eq!(suggest_gain(Gain::X64, 18_000, 17_000), Gain::X32);
        assert_eq!(suggest_gain(Gain::X64, 18_000, 4_000), Gain::X128);
        assert_eq!(suggest_gain(Gain::X64, 18_000, 5_000), Gain::X64);
        assert_eq!(suggest_gain(Gain::X512, 18_000, 1), Gain::X512);
        assert_eq!(suggest_gain(Gain::X0_5, 18_000, 18_000), Gain::X0_5);
    }

    #[test]
    fn alternating_banks() {
        let mut e = vec![Transaction::write_read(ADDRESS, vec![0x92], vec![0x24])];
        e.extend(start(29, 599, 7, &SMUX_F1_F4, &[0x11, 0x01]));
        e.push(Transaction::write(ADDRESS, vec![0x80, 0x03]));
        e.push(Transaction::write_read(ADDRESS, vec![0xA3], vec![0x00]));
        e.push(Transaction::write_read(ADDRESS, vec![0xA3], vec![0x40]));
        e.push(Transaction::write_read(
            ADDRESS,
            vec![0x94],
            data(0x07, [1, 2, 3, 4, 1000, 6]),
        ));
        e.push(Transaction::write(ADDRESS, vec![0x80, 0x00]));
        e.extend(start(29, 599, 7, &SMUX_F5_F8, &[0x01]));
        e.push(Transaction::write(ADDRESS, vec![0x80, 0x03]));
        e.push(Transaction::write_read(ADDRESS, vec![0xA3], vec![0x40]));
        e.push(Transaction::write_read(
            ADDRESS,
            vec![0x94],
            data(0x87, [5, 6, 7, 8, 2000, 9]),
        ));
        e.push(Transaction::write(ADDRESS, vec![0x80, 0x00]));
        let mut i2c = Mock::new(&e);
        let mut s = As7341::new(&mut i2c)
            .with_alternate(true)
            .with_lux_factor(Some(2_000));
        assert_eq!(s.probe().unwrap(), 0x24);
        assert_eq!(s.start_measurement().unwrap(), 56_044);
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        let low = s.read_measurement().unwrap();
        assert_eq!(low.bank, Bank::F1F4);
        assert_eq!(
            low.f,
            [Some(1), Some(2), Some(3), Some(4), None, None, None, None]
        );
        assert_eq!((low.clear, low.nir), (1000, 6));
        assert_eq!(low.illuminance, Some(624));
        assert!(!low.saturated);
        assert_eq!(s.bank(), Bank::F5F8);
        s.start_measurement().unwrap();
        let high = s.read_measurement().unwrap();
        assert!(high.saturated);
        assert_eq!(high.peak(), 2000);
        let all = high.merge(&low);
        assert_eq!(all.f.map(Option::unwrap), [1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(s.bank(), Bank::F1F4);
        i2c.done();
    }

    #[test]
    fn interrupt_probe_and_smux_timeout() {
        let mut e = vec![
            Transaction::write_read(ADDRESS, vec![0x92], vec![0x28]),
            Transaction::write(ADDRESS, vec![0x84, 0x64]),
            Transaction::write(ADDRESS, vec![0x85, 0x00]),
            Transaction::write(ADDRESS, vec![0x86, 0x88]),
            Transaction::write(ADDRESS, vec![0x87, 0x13]),
            Transaction::write(ADDRESS, vec![0xB5, 0x04]),
            Transaction::write(ADDRESS, vec![0xBD, 0x03]),
            Transaction::write(ADDRESS, vec![0xF9, 0x08]),
            Transaction::write_read(ADDRESS, vec![0x93], vec![0x88]),
            Transaction::write(ADDRESS, vec![0x93, 0x08]),
            Transaction::write_read(ADDRESS, vec![0x93], vec![0x00]),
            Transaction::write(ADDRESS, vec![0xF9, 0x00]),
        ];
        e.extend(start(
            255,
            u16::MAX - 1,
            10,
            &SMUX_F5_F8,
            &[0x11; SMUX_POLLS as usize],
        ));
        let mut i2c = Mock::new(&e);
        let mut s = As7341::new(&mut i2c)
            .with_gain(Gain::X512)
            .with_integration(255, u16::MAX)
            .with_bank(Bank::F5F8)
            .with_continuous(true);
        assert_eq!(s.probe(), Err(Error::UnexpectedId(0x28)));
        let w = Window {
            low: 100,
            high: 5000,
            persistence: 3,
        };
        s.set_interrupt(Some(w)).unwrap();
        assert!(s.take_interrupt().unwrap());
        assert!(!s.take_interrupt().unwrap());
        s.set_interrupt(None).unwrap();
        assert_eq!(s.start_measurement(), Err(Error::Timeout));
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

        let mut e = start(0, 999, 0, &SMUX_F1_F4, &[0x01]);
        e.push(Transaction::write(ADDRESS, vec![0x80, 0x03]));
        e.push(Transaction::write_read(ADDRESS, vec![0xA3], vec![0x40]));
        e.push(Transaction::write_read(
            ADDRESS,
            vec![0x94],
            data(0x00, [10, 20, 30, 40, 50, 60]),
        ));
        e.push(Transaction::write(ADDRESS, vec![0x80, 0x00]));
        let mut i2c = Mock::new(&e);
        let mut s = asynch::As7341::new(&mut i2c)
            .with_gain(Gain::X0_5)
            .with_integration(0, 999);
        let mut delay = RecordingDelay(0);
        let m = block_on(s.measure(&mut delay)).unwrap();
        assert_eq!(m.f[3], Some(40));
        assert_eq!(m.illuminance, None);
        assert_eq!(delay.0, 4_058);
        assert_eq!(s.bank(), Bank::F1F4);
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
