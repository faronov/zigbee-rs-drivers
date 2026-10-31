//! Silicon Labs Si7210 I2C hall-effect magnetic position and temperature
//! sensor (Si7210-B-0x, all I2C variants).
//!
//! Reference: Silicon Labs "Si7210 I2C Hall Effect Magnetic Position and
//! Temperature Sensor Data Sheet" Rev. 1.3 (marked Not Recommended for New
//! Designs).
//!
//! * Address 0x30–0x33, fixed by part number ([`ADDRESSES`]).
//! * Each [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//!   wakes the part (bare address write), re-applies settings that sleep
//!   resets (scale coefficients, filter), and triggers one burst
//!   (`oneburst`). The wait is 11 µs + 8.8 µs per extra sample.
//! * `read_measurement` returns `NotReady` until the "fresh" bit is set,
//!   reads the field and optionally the temperature of the same burst,
//!   then enters the configured [`AfterRead`] state (default: the factory
//!   periodic sleep-timer loop that keeps the output pin working).
//! * Sleep reloads most registers from OTP. Only the output-switch
//!   settings survive (`Usestore`, set by [`Si7210::set_switch`]); this
//!   driver caches the scale coefficients and filter and rewrites them
//!   after every wake.
//!
//! Datasheet errata handled here: the Bop/Brp formulas quote 0.05 mT/bit
//! but the 13-bit comparator LSB is 0.005 mT ([`SWITCH_LSB_NT`]); the
//! tamper formula omits a ×32 factor visible in its own examples.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Factory I2C addresses (part-number dependent).
pub const ADDRESSES: [u8; 4] = [0x30, 0x31, 0x32, 0x33];

const REG_HREVID: u8 = 0xC0;
const REG_DSPSIGM: u8 = 0xC1;
const REG_DSPSIGL: u8 = 0xC2;
const REG_DSPSIGSEL: u8 = 0xC3;
const REG_POWER_CTRL: u8 = 0xC4;
const REG_CTRL1: u8 = 0xC6;
const REG_CTRL2: u8 = 0xC7;
const REG_SLTIME: u8 = 0xC8;
const REG_CTRL3: u8 = 0xC9;
const REG_CTRL4: u8 = 0xCD;
const REG_OTP_ADDR: u8 = 0xE1;
const REG_OTP_DATA: u8 = 0xE2;
const REG_OTP_CTRL: u8 = 0xE3;
/// a0–a5 live registers (note the gap at 0xCD).
const REG_COEFF: [u8; 6] = [0xCA, 0xCB, 0xCC, 0xCE, 0xCF, 0xD0];

const PC_USESTORE: u8 = 0x08;
const PC_ONEBURST: u8 = 0x04;
const PC_SLEEP: u8 = 0x01;
const FRESH: u8 = 0x80;
const OTP_READ_EN: u8 = 0x02;
const SLTIMEENA: u8 = 0x01;
const SLFAST: u8 = 0x02;

const OTP_TEMP_OFFSET: u8 = 0x1D;
const OTP_TEMP_GAIN: u8 = 0x1E;

/// Chip ID nibble in HREVID\[7:4\].
pub const CHIP_ID: u8 = 0x1;
/// Wake-up time after the bare address write.
pub const WAKE_US: WaitUs = 10;
/// Field change per 13-bit switch-comparator count on the 20 mT scale (nT).
pub const SWITCH_LSB_NT: u32 = 5_000;

/// Full-scale range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Scale {
    /// ±20.47 mT, 1.25 µT/LSB.
    #[default]
    Mt20,
    /// ±204.7 mT, 12.5 µT/LSB.
    Mt200,
}

impl Scale {
    /// Field per 15-bit LSB, nT.
    pub const fn lsb_nt(self) -> i32 {
        match self {
            Self::Mt20 => 1_250,
            Self::Mt200 => 12_500,
        }
    }

    /// Field per switch-comparator count, nT.
    pub const fn switch_lsb_nt(self) -> u32 {
        match self {
            Self::Mt20 => SWITCH_LSB_NT,
            Self::Mt200 => SWITCH_LSB_NT * 10,
        }
    }
}

/// Magnet temperature compensation of an OTP coefficient set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MagnetComp {
    /// No compensation.
    #[default]
    None,
    /// Neodymium, −0.12 %/°C.
    Neodymium,
    /// Ceramic (ferrite), −0.2 %/°C.
    Ceramic,
}

/// OTP address of the a0–a5 set for `scale`/`comp` (datasheet §4.1.7).
pub const fn otp_coefficients(scale: Scale, comp: MagnetComp) -> u8 {
    let base = match comp {
        MagnetComp::None => 0x21,
        MagnetComp::Neodymium => 0x2D,
        MagnetComp::Ceramic => 0x39,
    };
    match scale {
        Scale::Mt20 => base,
        Scale::Mt200 => base + 6,
    }
}

/// Digital averaging filter (CTRL4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Filter {
    /// Average a burst of 2^`log2_samples` samples (≤ 12).
    Fir {
        /// df_bw.
        log2_samples: u8,
    },
    /// IIR with weight 1/2^`log2_weight` (≤ 12) over bursts of
    /// 2^`log2_burst` samples (≤ 7).
    Iir {
        /// df_bw.
        log2_weight: u8,
        /// df_burstsize.
        log2_burst: u8,
    },
}

impl Filter {
    /// CTRL4 value, or `None` if a field is out of range.
    pub const fn ctrl4(self) -> Option<u8> {
        match self {
            Self::Fir { log2_samples } if log2_samples <= 12 => Some(log2_samples << 1),
            Self::Iir {
                log2_weight,
                log2_burst,
            } if log2_weight <= 12 && log2_burst <= 7 => {
                Some((log2_burst << 5) | (log2_weight << 1) | 1)
            }
            _ => None,
        }
    }
}

/// Samples per burst for a CTRL4 value.
pub const fn burst_samples(ctrl4: u8) -> u32 {
    if ctrl4 & 1 == 0 {
        1 << ((ctrl4 >> 1) & 0x0F)
    } else {
        1 << (ctrl4 >> 5)
    }
}

/// Conversion time for a burst, µs (11 µs + 8.8 µs per extra sample).
pub const fn conversion_us(samples: u32) -> WaitUs {
    (110 + 88 * (samples - 1)).div_ceil(10)
}

/// Field polarity used by the output switch (`sw_fieldpolsel`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Polarity {
    /// |B| compared with the threshold.
    #[default]
    Omnipolar,
    /// −B compared (south pole / negative field).
    Negative,
    /// +B compared (also used for latch mode).
    Positive,
}

/// `sw_op` value selecting latch mode (threshold 0, doubled hysteresis).
pub const LATCH: u8 = 127;
/// `sw_hyst` value for zero hysteresis.
pub const NO_HYSTERESIS: u8 = 63;

/// Switch centre point in comparator counts for `sw_op`.
pub const fn threshold_counts(sw_op: u8) -> u16 {
    if sw_op >= LATCH {
        0
    } else {
        (16 + (sw_op & 0x0F) as u16) << (sw_op >> 4)
    }
}

/// Hysteresis in comparator counts for `sw_hyst` (`latch` doubles it).
pub const fn hysteresis_counts(sw_hyst: u8, latch: bool) -> u16 {
    if sw_hyst >= NO_HYSTERESIS {
        return 0;
    }
    let h = (8 + (sw_hyst & 0x07) as u16) << (sw_hyst >> 3);
    if latch { h * 2 } else { h }
}

/// Nearest `sw_op` code for a centre point of `counts` (16…3840).
pub fn threshold_code(counts: u16) -> u8 {
    nearest(counts, 16, 8, threshold_counts)
}

/// Nearest `sw_hyst` code for a non-latch hysteresis of `counts`
/// (8…1792; 0 → [`NO_HYSTERESIS`]).
pub fn hysteresis_code(counts: u16) -> u8 {
    if counts == 0 {
        return NO_HYSTERESIS;
    }
    nearest(counts, 8, 8, |c| hysteresis_counts(c, false))
}

fn nearest(target: u16, mantissas: u8, exponents: u8, f: impl Fn(u8) -> u16) -> u8 {
    let mut best = 0;
    let mut best_err = u16::MAX;
    for e in 0..exponents {
        for m in 0..mantissas {
            let code = (e << if mantissas == 16 { 4 } else { 3 }) | m;
            let err = f(code).abs_diff(target);
            if err < best_err {
                best = code;
                best_err = err;
            }
        }
    }
    best
}

/// Output-pin switch configuration (CTRL1/CTRL2, kept across sleep).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Switch {
    /// `sw_op` 0–127 ([`threshold_code`], [`LATCH`]).
    pub threshold: u8,
    /// `sw_hyst` 0–63 ([`hysteresis_code`], [`NO_HYSTERESIS`]).
    pub hysteresis: u8,
    /// Output low (instead of high) when the field is strong.
    pub low_for_field: bool,
    /// Field polarity.
    pub polarity: Polarity,
}

impl Switch {
    /// Switch operating at `center_nt` ± `hysteresis_nt` for `scale`.
    pub fn from_nt(scale: Scale, center_nt: u32, hysteresis_nt: u32, polarity: Polarity) -> Self {
        let lsb = scale.switch_lsb_nt();
        let counts = |nt: u32| (nt.saturating_add(lsb / 2) / lsb).min(u16::MAX as u32) as u16;
        Self {
            threshold: threshold_code(counts(center_nt)),
            hysteresis: hysteresis_code(counts(hysteresis_nt)),
            low_for_field: false,
            polarity,
        }
    }

    fn registers(&self) -> [u8; 2] {
        let pol = match self.polarity {
            Polarity::Omnipolar => 0,
            Polarity::Negative => 1,
            Polarity::Positive => 2,
        };
        [
            ((self.low_for_field as u8) << 7) | (self.threshold & 0x7F),
            (pol << 6) | (self.hysteresis & 0x3F),
        ]
    }
}

/// State entered at the end of every `read_measurement`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AfterRead {
    /// Factory sleep-timer loop (`slTimeena` = 1, stop = 0): periodic
    /// measurements keep the output pin updated, ≈ 0.4 µA average.
    #[default]
    Periodic,
    /// Full sleep with the timer off (≈ 50 nA, output pin frozen).
    Sleep,
    /// Stay idle with stop set (360–1000 µA; fastest next conversion).
    Idle,
}

/// Field (and optional temperature) sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Flux density along the sensing axis, nT.
    pub field_nt: i32,
    /// Die temperature, 0.01 °C (when enabled).
    pub temperature: Option<i32>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        let ut = self.field_nt + if self.field_nt < 0 { -500 } else { 500 };
        f(Quantity::MagneticFlux(ut / 1_000));
        if let Some(t) = self.temperature {
            f(Quantity::Temperature(t));
        }
    }
}

/// Field in nT from DSPSIGM/DSPSIGL.
pub const fn field_nt(msb: u8, lsb: u8, scale: Scale) -> i32 {
    let raw = (((msb & 0x7F) as i32) << 8) | lsb as i32;
    (raw - 16_384) * scale.lsb_nt()
}

/// Temperature in 0.01 °C from DSPSIGM/DSPSIGL with dspsigsel = 1, OTP
/// offset/gain bytes (0x1D/0x1E) and supply voltage (datasheet §5).
pub const fn temperature_centi(msb: u8, lsb: u8, offset: i8, gain: i8, vdd_mv: u16) -> i32 {
    let v = (((msb & 0x7F) as i64) << 5) + ((lsb >> 3) as i64);
    // −3.83e-6·v² + 0.16094·v − 279.80, in 1e-6 × 0.01 °C.
    let raw = -383 * v * v + 16_094_000 * v - 27_980_000_000;
    let t =
        raw * (2_048 + gain as i64) / 2_048 + offset as i64 * 6_250_000 - 22_200 * vdd_mv as i64;
    let half = if t < 0 { -500_000 } else { 500_000 };
    ((t + half) / 1_000_000) as i32
}

macro_rules! api {
    ($($asyncness:ident)?; $($aw:tt)*) => {
        /// Create a driver for the part at `address` whose factory scale is
        /// `scale`. No bus traffic; call [`Self::init`] next.
        pub const fn new(i2c: I2C, address: u8, scale: Scale) -> Self {
            Self {
                i2c,
                address,
                scale,
                coefficients: None,
                ctrl4: None,
                samples: 1,
                usestore: false,
                temperature: false,
                after_read: AfterRead::Periodic,
                temp_offset: 0,
                temp_gain: 0,
                vdd_mv: 3_300,
            }
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        /// Also read the temperature of every burst.
        pub fn set_temperature(&mut self, enable: bool) {
            self.temperature = enable;
        }

        /// Supply voltage used for temperature correction (default 3300).
        pub fn set_vdd_mv(&mut self, mv: u16) {
            self.vdd_mv = mv;
        }

        /// Power state after each `read_measurement`.
        pub fn set_after_read(&mut self, state: AfterRead) {
            self.after_read = state;
        }

                /// Wake the part with a bare address write; wait [`WAKE_US`]
        /// (automatic at ≤ 400 kHz) before the next transaction.
        pub $($asyncness)? fn wake_up(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.i2c.write(self.address, &[]) $($aw)* .map_err(Error::Bus)?;
            Ok(WAKE_US)
        }

                /// Read one register.
        pub $($asyncness)? fn read_register(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
            let mut b = [0u8];
            self.i2c
                .write_read(self.address, &[reg], &mut b)
                $($aw)*
                .map_err(Error::Bus)?;
            Ok(b[0])
        }

                /// Write one register.
        pub $($asyncness)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
            self.i2c.write(self.address, &[reg, value]) $($aw)* .map_err(Error::Bus)
        }

                /// Read one OTP byte.
        pub $($asyncness)? fn read_otp(&mut self, addr: u8) -> Result<u8, Error<I2C::Error>> {
            self.write_register(REG_OTP_ADDR, addr) $($aw)* ?;
            self.write_register(REG_OTP_CTRL, OTP_READ_EN) $($aw)* ?;
            self.read_register(REG_OTP_DATA) $($aw)*
        }

                /// Wake the part and check the chip ID; returns HREVID.
        pub $($asyncness)? fn probe(&mut self) -> Result<u8, Error<I2C::Error>> {
            self.wake_up() $($aw)* ?;
            let id = self.read_register(REG_HREVID) $($aw)* ?;
            if id >> 4 != CHIP_ID {
                return Err(Error::UnexpectedId(id as u32));
            }
            Ok(id)
        }

                /// [`Self::probe`] and load the temperature correction from OTP.
        pub $($asyncness)? fn init(&mut self) -> Result<(), Error<I2C::Error>> {
            self.probe() $($aw)* ?;
            self.temp_offset = self.read_otp(OTP_TEMP_OFFSET) $($aw)* ? as i8;
            self.temp_gain = self.read_otp(OTP_TEMP_GAIN) $($aw)* ? as i8;
            Ok(())
        }

                /// Switch the full-scale range / magnet compensation by copying an
        /// OTP coefficient set (rewritten after every wake). With no
        /// averaging these sets read ≈ 0.1 mT high (datasheet §4.1.2).
        pub $($asyncness)? fn set_scale(&mut self, scale: Scale, comp: MagnetComp) -> Result<(), Error<I2C::Error>> {
            self.wake_up() $($aw)* ?;
            let base = otp_coefficients(scale, comp);
            let mut c = [0u8; 6];
            for (i, v) in c.iter_mut().enumerate() {
                *v = self.read_otp(base + i as u8) $($aw)* ?;
            }
            self.coefficients = Some(c);
            self.scale = scale;
            self.write_coefficients(&c) $($aw)*
        }

                $($asyncness)? fn write_coefficients(&mut self, c: &[u8; 6]) -> Result<(), Error<I2C::Error>> {
            for (reg, v) in REG_COEFF.iter().zip(c) {
                self.write_register(*reg, *v) $($aw)* ?;
            }
            Ok(())
        }

        /// Use `filter` for every burst (rewritten after every wake).
        /// `InvalidArgument` if out of range.
        pub fn set_filter(&mut self, filter: Filter) -> Result<(), Error<I2C::Error>> {
            let ctrl4 = filter.ctrl4().ok_or(Error::InvalidArgument)?;
            self.ctrl4 = Some(ctrl4);
            Ok(())
        }

        /// Program the output switch and set `Usestore` so it survives
        /// sleep. `InvalidArgument` if a code is out of range.
        pub $($asyncness)? fn set_switch(&mut self, switch: &Switch) -> Result<(), Error<I2C::Error>> {
            if switch.threshold > LATCH || switch.hysteresis > NO_HYSTERESIS {
                return Err(Error::InvalidArgument);
            }
            self.wake_up() $($aw)* ?;
            let [c1, c2] = switch.registers();
            self.write_register(REG_CTRL1, c1) $($aw)* ?;
            self.write_register(REG_CTRL2, c2) $($aw)* ?;
            self.usestore = true;
            self.power_ctrl(0x02) $($aw)*
        }

                /// Program the idle timer (`slTime`, `slFast`) used between samples
        /// when running from idle; not retained in sleep. The datasheet
        /// recommends leaving it alone while the sleep timer is used.
        pub $($asyncness)? fn set_idle_time(&mut self, sl_time: u8, sl_fast: bool) -> Result<(), Error<I2C::Error>> {
            self.write_register(REG_SLTIME, sl_time) $($aw)* ?;
            let c3 = self.read_register(REG_CTRL3) $($aw)* ?;
            self.write_register(REG_CTRL3, (c3 & !SLFAST) | if sl_fast { SLFAST } else { 0 }) $($aw)*
        }

                /// Write the low nibble of POWER_CTRL (Usestore kept), preserving
        /// the factory upper bits.
        $($asyncness)? fn power_ctrl(&mut self, bits: u8) -> Result<(), Error<I2C::Error>> {
            let pc = self.read_register(REG_POWER_CTRL) $($aw)* ?;
            let store = if self.usestore { PC_USESTORE } else { 0 };
            self.write_register(REG_POWER_CTRL, (pc & 0x70) | store | bits) $($aw)*
        }

                $($asyncness)? fn sl_timer(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
            let c3 = self.read_register(REG_CTRL3) $($aw)* ?;
            let new = if enable { c3 | SLTIMEENA } else { c3 & !SLTIMEENA };
            if new != c3 {
                self.write_register(REG_CTRL3, new) $($aw)* ?;
            }
            Ok(())
        }

                /// Enter `state` (see [`AfterRead`]).
        pub $($asyncness)? fn enter(&mut self, state: AfterRead) -> Result<(), Error<I2C::Error>> {
            match state {
                AfterRead::Periodic => {
                    self.sl_timer(true) $($aw)* ?;
                    self.power_ctrl(0) $($aw)*
                }
                AfterRead::Sleep => {
                    self.sl_timer(false) $($aw)* ?;
                    self.power_ctrl(PC_SLEEP) $($aw)*
                }
                AfterRead::Idle => Ok(()),
            }
        }

                $($asyncness)? fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.wake_up() $($aw)* ?;
            if let Some(c) = self.coefficients {
                self.write_coefficients(&c) $($aw)* ?;
            }
            let ctrl4 = match self.ctrl4 {
                Some(v) => {
                    self.write_register(REG_CTRL4, v) $($aw)* ?;
                    v
                }
                None => self.read_register(REG_CTRL4) $($aw)* ?,
            };
            self.samples = burst_samples(ctrl4);
            self.write_register(REG_DSPSIGSEL, 0) $($aw)* ?;
            self.power_ctrl(PC_ONEBURST) $($aw)* ?;
            Ok(conversion_us(self.samples))
        }

                $($asyncness)? fn read(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let m = self.read_register(REG_DSPSIGM) $($aw)* ?;
            if m & FRESH == 0 {
                return Err(Error::NotReady);
            }
            let l = self.read_register(REG_DSPSIGL) $($aw)* ?;
            let mut out = Measurement {
                field_nt: field_nt(m, l, self.scale),
                temperature: None,
            };
            if self.temperature {
                self.write_register(REG_DSPSIGSEL, 1) $($aw)* ?;
                let tm = self.read_register(REG_DSPSIGM) $($aw)* ?;
                let tl = self.read_register(REG_DSPSIGL) $($aw)* ?;
                self.write_register(REG_DSPSIGSEL, 0) $($aw)* ?;
                out.temperature = Some(temperature_centi(
                    tm,
                    tl,
                    self.temp_offset,
                    self.temp_gain,
                    self.vdd_mv,
                ));
            }
            self.enter(self.after_read) $($aw)* ?;
            Ok(out)
        }
    };
}

/// Blocking driver.
pub struct Si7210<I2C> {
    i2c: I2C,
    address: u8,
    scale: Scale,
    coefficients: Option<[u8; 6]>,
    ctrl4: Option<u8>,
    samples: u32,
    usestore: bool,
    temperature: bool,
    after_read: AfterRead,
    temp_offset: i8,
    temp_gain: i8,
    vdd_mv: u16,
}

impl<I2C: embedded_hal::i2c::I2c> Si7210<I2C> {
    api!(;);
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Si7210<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.start()
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        self.read()
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Si7210<I2C> {
    type BusError = I2C::Error;

    /// Full sleep, sleep timer off (≈ 50 nA).
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.enter(AfterRead::Sleep)
    }

    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.wake_up()
    }
}

/// Async driver (`embedded-hal-async` I2C).
pub mod asynch {
    use super::*;

    /// Async twin of [`super::Si7210`].
    pub struct Si7210<I2C> {
        i2c: I2C,
        address: u8,
        scale: Scale,
        coefficients: Option<[u8; 6]>,
        ctrl4: Option<u8>,
        samples: u32,
        usestore: bool,
        temperature: bool,
        after_read: AfterRead,
        temp_offset: i8,
        temp_gain: i8,
        vdd_mv: u16,
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Si7210<I2C> {
        api!(async; .await);
    }

    impl<I2C: embedded_hal_async::i2c::I2c> zigbee_sensor::asynch::Sensor for Si7210<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.start().await
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            self.read().await
        }
    }

    impl<I2C: embedded_hal_async::i2c::I2c> zigbee_sensor::asynch::LowPower for Si7210<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.enter(AfterRead::Sleep).await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.wake_up().await
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
    use std::{vec, vec::Vec};
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = 0x30;

    fn rd(reg: u8, v: u8) -> T {
        T::write_read(A, vec![reg], vec![v])
    }
    fn wr(reg: u8, v: u8) -> T {
        T::write(A, vec![reg, v])
    }

    #[test]
    fn field_and_temperature_formulas() {
        // 16384 = zero field; ±1 LSB = ±1.25 µT (20 mT) / 12.5 µT (200 mT).
        assert_eq!(field_nt(0xC0, 0x00, Scale::Mt20), 0);
        assert_eq!(field_nt(0x40, 0x01, Scale::Mt20), 1_250);
        assert_eq!(field_nt(0x3F, 0xFF, Scale::Mt200), -12_500);
        assert_eq!(field_nt(0x7F, 0xFF, Scale::Mt20), 16_383 * 1_250);
        // value = 2000: −15.32 + 321.88 − 279.80 = 26.76 °C, −0.222·3.3 V.
        assert_eq!(temperature_centi(0xBE, 0x80, 0, 0, 3_300), 2_603);
        // OTP offset 16/16 = +1 °C; gain −128/2048 = ×0.9375.
        assert_eq!(temperature_centi(0xBE, 0x80, 16, 0, 0), 2_776);
        assert_eq!(temperature_centi(0xBE, 0x80, 0, -128, 0), 2_509);
    }

    #[test]
    fn switch_codes_match_datasheet_ranges() {
        assert_eq!(threshold_counts(0), 16);
        assert_eq!(threshold_counts(0x0F), 31);
        assert_eq!(threshold_counts(0x7E), 3_840);
        assert_eq!(threshold_counts(LATCH), 0);
        assert_eq!(hysteresis_counts(0, false), 8);
        assert_eq!(hysteresis_counts(62, false), 1_792);
        assert_eq!(hysteresis_counts(62, true), 3_584);
        assert_eq!(hysteresis_counts(NO_HYSTERESIS, false), 0);
        // 19.2 mT ± 8.96 mT at 5 µT per count.
        let s = Switch::from_nt(Scale::Mt20, 19_200_000, 8_960_000, Polarity::Omnipolar);
        assert_eq!((s.threshold, s.hysteresis), (0x7E, 62));
        let s = Switch::from_nt(Scale::Mt20, 1_000_000, 0, Polarity::Positive);
        assert_eq!(threshold_counts(s.threshold), 200);
        assert_eq!(s.registers(), [s.threshold, 0x80 | NO_HYSTERESIS]);
        assert_eq!(conversion_us(1), 11);
        assert_eq!(conversion_us(4096), 36_047);
        assert_eq!(
            burst_samples(Filter::Fir { log2_samples: 3 }.ctrl4().unwrap()),
            8
        );
        let iir = Filter::Iir {
            log2_weight: 4,
            log2_burst: 2,
        }
        .ctrl4()
        .unwrap();
        assert_eq!((iir, burst_samples(iir)), (0x49, 4));
        assert_eq!(Filter::Fir { log2_samples: 13 }.ctrl4(), None);
    }

    fn start_tx(ctrl4: u8) -> Vec<T> {
        vec![
            T::write(A, vec![]),
            rd(REG_CTRL4, ctrl4),
            wr(REG_DSPSIGSEL, 0),
            rd(REG_POWER_CTRL, 0x02),
            wr(REG_POWER_CTRL, 0x04),
        ]
    }

    #[test]
    fn init_measure_with_temperature_then_periodic_sleep() {
        let mut tx = vec![
            T::write(A, vec![]),
            rd(REG_HREVID, 0x14),
            wr(REG_OTP_ADDR, 0x1D),
            wr(REG_OTP_CTRL, 0x02),
            rd(REG_OTP_DATA, 0x00),
            wr(REG_OTP_ADDR, 0x1E),
            wr(REG_OTP_CTRL, 0x02),
            rd(REG_OTP_DATA, 0x00),
        ];
        tx.extend(start_tx(0x02));
        tx.extend([
            rd(REG_DSPSIGM, 0x40),
            rd(REG_DSPSIGM, 0xC0),
            rd(REG_DSPSIGL, 0x08),
            wr(REG_DSPSIGSEL, 1),
            rd(REG_DSPSIGM, 0x3E),
            rd(REG_DSPSIGL, 0x80),
            wr(REG_DSPSIGSEL, 0),
            rd(REG_CTRL3, 0x00),
            wr(REG_CTRL3, 0x01),
            rd(REG_POWER_CTRL, 0x02),
            wr(REG_POWER_CTRL, 0x00),
        ]);
        let mut i2c = Mock::new(&tx);
        let mut s = Si7210::new(i2c.clone(), A, Scale::Mt20);
        s.init().unwrap();
        s.set_temperature(true);
        assert_eq!(s.start_measurement(), Ok(conversion_us(2)));
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        let m = s.read_measurement().unwrap();
        assert_eq!(m.field_nt, 10_000);
        assert_eq!(m.temperature, Some(2_603));
        let mut q = Vec::new();
        m.for_each(&mut |x| q.push(x));
        assert_eq!(
            q,
            [Quantity::MagneticFlux(10), Quantity::Temperature(2_603)]
        );
        i2c.done();
    }

    #[test]
    fn probe_rejects_wrong_chip() {
        let mut i2c = Mock::new(&[T::write(A, vec![]), rd(REG_HREVID, 0x24)]);
        let mut s = Si7210::new(i2c.clone(), A, Scale::Mt20);
        assert_eq!(s.probe(), Err(Error::UnexpectedId(0x24)));
        let fir = Filter::Fir { log2_samples: 13 };
        assert_eq!(s.set_filter(fir), Err(Error::InvalidArgument));
        let sw = Switch {
            threshold: LATCH + 1,
            hysteresis: 0,
            low_for_field: false,
            polarity: Polarity::Omnipolar,
        };
        assert_eq!(s.set_switch(&sw), Err(Error::InvalidArgument));
        i2c.done();
    }

    #[test]
    fn scale_switch_and_sleep_survive_wake() {
        let mut tx = vec![T::write(A, vec![])];
        for i in 0..6u8 {
            tx.extend([
                wr(REG_OTP_ADDR, 0x27 + i),
                wr(REG_OTP_CTRL, 0x02),
                rd(REG_OTP_DATA, 0x10 + i),
            ]);
        }
        let coeff = |tx: &mut Vec<T>| {
            for (i, r) in REG_COEFF.iter().enumerate() {
                tx.push(wr(*r, 0x10 + i as u8));
            }
        };
        coeff(&mut tx);
        tx.extend([
            T::write(A, vec![]),
            wr(REG_CTRL1, 0x80 | 0x20),
            wr(REG_CTRL2, 0x40 | 0x05),
            rd(REG_POWER_CTRL, 0x02),
            wr(REG_POWER_CTRL, 0x0A),
            // sleep
            rd(REG_CTRL3, 0x03),
            wr(REG_CTRL3, 0x02),
            rd(REG_POWER_CTRL, 0x0A),
            wr(REG_POWER_CTRL, 0x09),
            // measurement after sleep: coefficients and filter rewritten
            T::write(A, vec![]),
        ]);
        coeff(&mut tx);
        tx.extend([
            wr(REG_CTRL4, 0x06),
            wr(REG_DSPSIGSEL, 0),
            rd(REG_POWER_CTRL, 0x0A),
            wr(REG_POWER_CTRL, 0x0C),
            rd(REG_DSPSIGM, 0xBF),
            rd(REG_DSPSIGL, 0xFF),
        ]);
        let mut i2c = Mock::new(&tx);
        let mut s = Si7210::new(i2c.clone(), A, Scale::Mt20);
        s.set_scale(Scale::Mt200, MagnetComp::None).unwrap();
        let sw = Switch {
            threshold: 0x20,
            hysteresis: 5,
            low_for_field: true,
            polarity: Polarity::Negative,
        };
        s.set_switch(&sw).unwrap();
        s.sleep().unwrap();
        s.set_filter(Filter::Fir { log2_samples: 3 }).unwrap();
        s.set_after_read(AfterRead::Idle);
        assert_eq!(s.start_measurement(), Ok(conversion_us(8)));
        assert_eq!(s.read_measurement().unwrap().field_nt, -12_500);
        i2c.done();
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::Sensor as _;
        let mut tx = start_tx(0x00);
        tx.extend([
            rd(REG_DSPSIGM, 0xC0),
            rd(REG_DSPSIGL, 0x00),
            rd(REG_CTRL3, 0x01),
            rd(REG_POWER_CTRL, 0x02),
            wr(REG_POWER_CTRL, 0x00),
        ]);
        let mut i2c = Mock::new(&tx);
        let mut s = asynch::Si7210::new(i2c.clone(), A, Scale::Mt20);
        assert_eq!(block_on(s.start_measurement()), Ok(11));
        assert_eq!(block_on(s.read_measurement()).unwrap().field_nt, 0);
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
