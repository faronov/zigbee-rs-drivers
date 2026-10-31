//! Texas Instruments FDC1004 4-channel capacitance-to-digital converter.
//!
//! Reference: TI "FDC1004 4-Channel Capacitance-to-Digital Converter for
//! Capacitive Sensing Applications", SNOSCY5C (revised October 2024).
//!
//! * Four measurement slots ([`Slot`]), each single-ended (against ground
//!   or the CAPDAC offset) or differential ([`MeasConfig`]).
//! * Results are reported in femtofarads (no ZCL capacitance quantity
//!   exists). An optional [`Mapping`] turns them into
//!   `Quantity::SoilMoisture` or a liquid level as `Quantity::Distance`.
//! * One configured slot is converted single-shot. The device allows
//!   only one single-shot trigger at a time, so several slots are
//!   converted in repeat mode, and repeat mode is stopped once all results
//!   are read. Afterwards the device idles in standby (29 µA).
//! * The datasheet gives no conversion time; the driver waits one sample
//!   period of the selected [`Rate`] per slot plus 10 % and returns
//!   `NotReady` until every DONE flag is set.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed 7-bit address.
pub const ADDRESS: u8 = 0x50;
/// Manufacturer ID register value ("TI").
pub const MANUFACTURER_ID: u16 = 0x5449;
/// Device ID register value.
pub const DEVICE_ID: u16 = 0x1004;
/// Femtofarads per CAPDAC step (3.125 pF).
pub const CAPDAC_STEP_FF: i32 = 3_125;

const REG_CONF_MEAS1: u8 = 0x08;
const REG_FDC_CONF: u8 = 0x0C;
const REG_OFFSET_CAL1: u8 = 0x0D;
const REG_GAIN_CAL1: u8 = 0x11;
const REG_MANUFACTURER_ID: u8 = 0xFE;
const REG_DEVICE_ID: u8 = 0xFF;
const RESET: u16 = 0x8000;
const REPEAT: u16 = 0x0100;

/// Capacitive input pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Input {
    /// CIN1.
    Cin1 = 0,
    /// CIN2.
    Cin2,
    /// CIN3.
    Cin3,
    /// CIN4.
    Cin4,
}

/// Negative side of a measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Negative {
    /// Single-ended against ground (CHB disabled; SHLD1/SHLD2 shorted).
    Ground,
    /// Single-ended with a CAPDAC offset of `code` × 3.125 pF (0–31).
    Capdac(u8),
    /// Differential against a higher-numbered input (|ΔC| < 15 pF).
    Input(Input),
}

/// Configuration of one measurement slot (CONF_MEASn).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeasConfig {
    /// Positive input (CHA).
    pub positive: Input,
    /// Negative input (CHB).
    pub negative: Negative,
}

impl MeasConfig {
    /// CONF_MEASn value; `None` if the combination is invalid.
    pub const fn register(&self) -> Option<u16> {
        let (chb, capdac) = match self.negative {
            Negative::Ground => (7, 0),
            Negative::Capdac(c) if c <= 31 => (4, c),
            Negative::Input(n) if n as u8 > self.positive as u8 => (n as u8, 0),
            _ => return None,
        };
        Some(((self.positive as u16) << 13) | ((chb as u16) << 10) | ((capdac as u16) << 5))
    }

    const fn offset_ff(&self) -> i32 {
        match self.negative {
            Negative::Capdac(c) => c as i32 * CAPDAC_STEP_FF,
            _ => 0,
        }
    }
}

/// Measurement slot (result registers and CONF_MEASn).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Slot {
    /// Measurement 1.
    M1 = 0,
    /// Measurement 2.
    M2,
    /// Measurement 3.
    M3,
    /// Measurement 4.
    M4,
}

/// Output rate; lower rates give higher resolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Rate {
    /// 100 S/s (16-bit effective resolution).
    #[default]
    S100 = 1,
    /// 200 S/s.
    S200 = 2,
    /// 400 S/s.
    S400 = 3,
}

impl Rate {
    /// Sample period, µs.
    pub const fn period_us(self) -> WaitUs {
        match self {
            Self::S100 => 10_000,
            Self::S200 => 5_000,
            Self::S400 => 2_500,
        }
    }
}

/// Optional conversion of capacitances to a ZCL quantity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mapping {
    /// Femtofarads only.
    #[default]
    None,
    /// Two-point soil-moisture calibration of one slot.
    SoilMoisture {
        /// Slot measuring the probe.
        slot: Slot,
        /// Capacitance in dry soil or air (0 %), fF.
        dry_ff: i32,
        /// Capacitance in saturated soil or water (100 %), fF.
        wet_ff: i32,
    },
    /// Three-electrode liquid level (datasheet §7.1.1), reported as
    /// `Quantity::Distance` in mm.
    Level {
        /// Level electrode slot.
        level: Slot,
        /// Reference-liquid electrode slot (always submerged).
        reference_liquid: Slot,
        /// Reference-environment electrode slot (always dry).
        reference_environment: Slot,
        /// Level electrode capacitance with the tank empty, fF.
        empty_ff: i32,
        /// Height of the reference electrodes, mm.
        reference_height_mm: u32,
    },
}

/// Soil moisture in 0.01 % from a capacitance between the `dry_ff` and
/// `wet_ff` points (clamped; 0 if the points are equal).
pub const fn soil_moisture(ff: i32, dry_ff: i32, wet_ff: i32) -> u16 {
    let span = wet_ff as i64 - dry_ff as i64;
    if span == 0 {
        return 0;
    }
    let pos = (ff as i64 - dry_ff as i64) * 10_000;
    let v = (pos + span / 2) / span;
    if v < 0 {
        0
    } else if v > 10_000 {
        10_000
    } else {
        v as u16
    }
}

/// Liquid level `h_ref × (C_lev − C_lev(0)) / (C_RL − C_RE)` in mm;
/// `None` if the reference electrodes read the same (no liquid contrast).
/// Negative levels clamp to 0.
pub const fn liquid_level_mm(
    level_ff: i32,
    empty_ff: i32,
    reference_liquid_ff: i32,
    reference_environment_ff: i32,
    reference_height_mm: u32,
) -> Option<u32> {
    let den = reference_liquid_ff as i64 - reference_environment_ff as i64;
    if den == 0 {
        return None;
    }
    let num = (level_ff as i64 - empty_ff as i64) * reference_height_mm as i64;
    let v = (num + den / 2) / den;
    Some(if v < 0 { 0 } else { v as u32 })
}

/// Capacitance in fF from the 24-bit two's-complement result plus the
/// CAPDAC offset: raw / 2^19 pF.
pub const fn capacitance_ff(raw24: u32, offset_ff: i32) -> i32 {
    let raw = ((raw24 << 8) as i32 >> 8) as i64 * 1_000;
    let half = if raw < 0 { -(1 << 18) } else { 1 << 18 };
    ((raw + half) / (1 << 19)) as i32 + offset_ff
}

/// OFFSET_CALn code (5.11 two's complement pF) for `ff`, clamped to
/// ±16 pF.
pub const fn offset_code(ff: i32) -> i16 {
    let v = ff as i64 * 2_048;
    let half = if v < 0 { -500 } else { 500 };
    let code = (v + half) / 1_000;
    if code > i16::MAX as i64 {
        i16::MAX
    } else if code < i16::MIN as i64 {
        i16::MIN
    } else {
        code as i16
    }
}

/// One set of results.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Measurement {
    /// Capacitance per slot in fF (`None` for slots not measured).
    pub capacitance_ff: [Option<i32>; 4],
    /// Value derived through the configured [`Mapping`].
    pub mapped: Option<Quantity>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if let Some(q) = self.mapped {
            f(q);
        }
    }
}

impl Mapping {
    fn apply(&self, c: &[Option<i32>; 4]) -> Option<Quantity> {
        match *self {
            Self::None => None,
            Self::SoilMoisture {
                slot,
                dry_ff,
                wet_ff,
            } => c[slot as usize].map(|v| Quantity::SoilMoisture(soil_moisture(v, dry_ff, wet_ff))),
            Self::Level {
                level,
                reference_liquid,
                reference_environment,
                empty_ff,
                reference_height_mm,
            } => {
                let (l, rl, re) = (
                    c[level as usize]?,
                    c[reference_liquid as usize]?,
                    c[reference_environment as usize]?,
                );
                liquid_level_mm(l, empty_ff, rl, re, reference_height_mm).map(Quantity::Distance)
            }
        }
    }
}

macro_rules! api {
    ($($asyncness:ident)?; $($aw:tt)*) => {
        /// Create a driver; no bus traffic. No slot is enabled yet.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                rate: Rate::S100,
                enabled: 0,
                offsets: [0; 4],
                mapping: Mapping::None,
            }
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        /// Output rate used by the next trigger.
        pub fn set_rate(&mut self, rate: Rate) {
            self.rate = rate;
        }

        /// Quantity derived from the results.
        pub fn set_mapping(&mut self, mapping: Mapping) {
            self.mapping = mapping;
        }

        /// Write a 16-bit register.
        pub $($asyncness)? fn write_register(&mut self, reg: u8, value: u16) -> Result<(), Error<I2C::Error>> {
            let [h, l] = value.to_be_bytes();
            self.i2c.write(ADDRESS, &[reg, h, l]) $($aw)* .map_err(Error::Bus)
        }

        /// Read a 16-bit register.
        pub $($asyncness)? fn read_register(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
            let mut b = [0u8; 2];
            self.i2c.write_read(ADDRESS, &[reg], &mut b) $($aw)* .map_err(Error::Bus)?;
            Ok(u16::from_be_bytes(b))
        }

        /// Check the manufacturer and device IDs.
        pub $($asyncness)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            let m = self.read_register(REG_MANUFACTURER_ID) $($aw)* ?;
            let d = self.read_register(REG_DEVICE_ID) $($aw)* ?;
            if (m, d) != (MANUFACTURER_ID, DEVICE_ID) {
                return Err(Error::UnexpectedId(((m as u32) << 16) | d as u32));
            }
            Ok(())
        }

        /// Software reset: configuration, offsets and gains return to
        /// their defaults (factory gain reloaded); all slots disabled.
        pub $($asyncness)? fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(REG_FDC_CONF, RESET) $($aw)* ?;
            self.enabled = 0;
            Ok(())
        }

        /// Configure and enable `slot`. `InvalidData` for a CAPDAC code
        /// above 31 or a differential pair whose negative input is not
        /// higher-numbered than the positive one.
        pub $($asyncness)? fn configure(&mut self, slot: Slot, config: &MeasConfig) -> Result<(), Error<I2C::Error>> {
            let reg = config.register().ok_or(Error::InvalidData)?;
            self.write_register(REG_CONF_MEAS1 + slot as u8, reg) $($aw)* ?;
            self.offsets[slot as usize] = config.offset_ff();
            self.enabled |= 1 << slot as u8;
            Ok(())
        }

        /// Stop measuring `slot` (its configuration stays in the device).
        pub fn disable(&mut self, slot: Slot) {
            self.enabled &= !(1 << slot as u8);
        }

        /// Parasitic offset added to `input` in fF (±16 pF, 0.49 fF
        /// steps; lost on reset/power-up).
        pub $($asyncness)? fn set_offset(&mut self, input: Input, ff: i32) -> Result<(), Error<I2C::Error>> {
            self.write_register(REG_OFFSET_CAL1 + input as u8, offset_code(ff) as u16) $($aw)*
        }

        /// Gain correction of `input`: gain = `code` / 16384 (0 to 4;
        /// reset restores the factory value).
        pub $($asyncness)? fn set_gain(&mut self, input: Input, code: u16) -> Result<(), Error<I2C::Error>> {
            self.write_register(REG_GAIN_CAL1 + input as u8, code) $($aw)*
        }

        $($asyncness)? fn trigger(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let n = self.enabled.count_ones();
            if n == 0 {
                return Err(Error::InvalidData);
            }
            let meas = (self.enabled.reverse_bits() >> 4) as u16;
            let repeat = if n > 1 { REPEAT } else { 0 };
            let conf = ((self.rate as u16) << 10) | repeat | (meas << 4);
            self.write_register(REG_FDC_CONF, conf) $($aw)* ?;
            let period = self.rate.period_us();
            Ok(n * period + period / 10)
        }

        $($asyncness)? fn read_results(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let done = self.read_register(REG_FDC_CONF) $($aw)* ? as u8 & 0x0F;
            let want = self.enabled.reverse_bits() >> 4;
            if done & want != want {
                return Err(Error::NotReady);
            }
            let mut m = Measurement::default();
            for slot in 0..4u8 {
                if self.enabled & (1 << slot) == 0 {
                    continue;
                }
                // MSB register first, then LSB.
                let msb = self.read_register(slot * 2) $($aw)* ?;
                let lsb = self.read_register(slot * 2 + 1) $($aw)* ?;
                let raw = ((msb as u32) << 8) | (lsb >> 8) as u32;
                m.capacitance_ff[slot as usize] = Some(capacitance_ff(raw, self.offsets[slot as usize]));
            }
            if self.enabled.count_ones() > 1 {
                self.write_register(REG_FDC_CONF, (self.rate as u16) << 10) $($aw)* ?;
            }
            m.mapped = self.mapping.apply(&m.capacitance_ff);
            Ok(m)
        }
    };
}

/// Blocking driver.
pub struct Fdc1004<I2C> {
    i2c: I2C,
    rate: Rate,
    enabled: u8,
    offsets: [i32; 4],
    mapping: Mapping,
}

impl<I2C: embedded_hal::i2c::I2c> Fdc1004<I2C> {
    api!(;);
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Fdc1004<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// `InvalidData` if no slot is configured.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.trigger()
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        self.read_results()
    }
}

/// Async driver (`embedded-hal-async` I2C).
pub mod asynch {
    use super::*;

    /// Async twin of [`super::Fdc1004`].
    pub struct Fdc1004<I2C> {
        i2c: I2C,
        rate: Rate,
        enabled: u8,
        offsets: [i32; 4],
        mapping: Mapping,
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Fdc1004<I2C> {
        api!(async; .await);
    }

    impl<I2C: embedded_hal_async::i2c::I2c> zigbee_sensor::asynch::Sensor for Fdc1004<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.trigger().await
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            self.read_results().await
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
    use std::vec;
    use zigbee_sensor::blocking::Sensor;

    fn reg(r: u8, v: u16) -> T {
        T::write_read(ADDRESS, vec![r], v.to_be_bytes().to_vec())
    }

    fn wr(r: u8, v: u16) -> T {
        let [h, l] = v.to_be_bytes();
        T::write(ADDRESS, vec![r, h, l])
    }

    #[test]
    fn conversions() {
        assert_eq!(capacitance_ff(0x08_0000, 0), 1_000);
        assert_eq!(capacitance_ff(0xF8_0000, 0), -1_000);
        assert_eq!(capacitance_ff(0x7F_FFFF, 0), 16_000);
        assert_eq!(capacitance_ff(0, 31 * CAPDAC_STEP_FF), 96_875);
        // §6.6.1.4: +2.5 pF = integer 2, fraction 0.5 → 0b00010_10000000000.
        assert_eq!(offset_code(2_500), 0x1400);
        assert_eq!(offset_code(-1_000), -2_048);
        assert_eq!(offset_code(20_000), i16::MAX);
        assert_eq!(soil_moisture(1_500, 1_000, 3_000), 2_500);
        assert_eq!(soil_moisture(500, 1_000, 3_000), 0);
        assert_eq!(soil_moisture(2_000, 3_000, 1_000), 5_000);
        assert_eq!(liquid_level_mm(3_000, 1_000, 4_700, 4_300, 10), Some(50));
        assert_eq!(liquid_level_mm(900, 1_000, 4_700, 4_300, 10), Some(0));
        assert_eq!(liquid_level_mm(1, 1, 4, 4, 10), None);
        let bad = MeasConfig {
            positive: Input::Cin3,
            negative: Negative::Input(Input::Cin2),
        };
        assert_eq!(bad.register(), None);
        let capdac = MeasConfig {
            positive: Input::Cin2,
            negative: Negative::Capdac(32),
        };
        assert_eq!(capdac.register(), None);
    }

    #[test]
    fn single_shot_with_capdac_and_soil_mapping() {
        let mut i2c = Mock::new(&[
            reg(0xFE, 0x5449),
            reg(0xFF, 0x1004),
            wr(0x09, (1 << 13) | (4 << 10) | (10 << 5)),
            wr(0x0E, 0x1400),
            wr(0x0C, 0x0440),
            reg(0x0C, 0x0440),
            reg(0x0C, 0x0444),
            reg(0x02, 0x0800),
            reg(0x03, 0x0000),
        ]);
        let mut s = Fdc1004::new(i2c.clone());
        s.probe().unwrap();
        let cfg = MeasConfig {
            positive: Input::Cin2,
            negative: Negative::Capdac(10),
        };
        s.configure(Slot::M2, &cfg).unwrap();
        s.set_offset(Input::Cin2, 2_500).unwrap();
        s.set_mapping(Mapping::SoilMoisture {
            slot: Slot::M2,
            dry_ff: 30_000,
            wet_ff: 40_000,
        });
        // Datasheet trigger example decoded with REPEAT = 0.
        assert_eq!(s.start_measurement(), Ok(11_000));
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        let m = s.read_measurement().unwrap();
        // 31.25 pF CAPDAC + 1 pF.
        assert_eq!(m.capacitance_ff, [None, Some(32_250), None, None]);
        assert_eq!(m.mapped, Some(Quantity::SoilMoisture(2_250)));
        i2c.done();
    }

    #[test]
    fn probe_rejects_other_device() {
        let mut i2c = Mock::new(&[reg(0xFE, 0x5449), reg(0xFF, 0x2114)]);
        let mut s = Fdc1004::new(i2c.clone());
        assert_eq!(s.probe(), Err(Error::UnexpectedId(0x5449_2114)));
        i2c.done();
    }

    #[test]
    fn async_level_uses_repeat_mode() {
        use zigbee_sensor::asynch::Sensor as _;
        let ground = |positive| MeasConfig {
            positive,
            negative: Negative::Ground,
        };
        let mut i2c = Mock::new(&[
            wr(0x08, 0x1C00),
            wr(0x09, 0x3C00),
            wr(0x0A, 0x5C00),
            wr(0x0C, 0x0DE0),
            reg(0x0C, 0x0DEE),
            reg(0x00, 0x1800),
            reg(0x01, 0x0000),
            reg(0x02, 0x2599),
            reg(0x03, 0x9A00),
            reg(0x04, 0x2266),
            reg(0x05, 0x6600),
            wr(0x0C, 0x0C00),
        ]);
        let mut s = asynch::Fdc1004::new(i2c.clone());
        s.set_rate(Rate::S400);
        block_on(s.configure(Slot::M1, &ground(Input::Cin1))).unwrap();
        block_on(s.configure(Slot::M2, &ground(Input::Cin2))).unwrap();
        block_on(s.configure(Slot::M3, &ground(Input::Cin3))).unwrap();
        s.set_mapping(Mapping::Level {
            level: Slot::M1,
            reference_liquid: Slot::M2,
            reference_environment: Slot::M3,
            empty_ff: 1_000,
            reference_height_mm: 10,
        });
        assert_eq!(block_on(s.start_measurement()), Ok(3 * 2_500 + 250));
        let m = block_on(s.read_measurement()).unwrap();
        assert_eq!(
            m.capacitance_ff,
            [Some(3_000), Some(4_700), Some(4_300), None]
        );
        assert_eq!(m.mapped, Some(Quantity::Distance(50)));
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
