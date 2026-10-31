//! CFSensor XGZP6847D and XGZP6897D calibrated gauge / differential
//! pressure sensors (I2C).
//!
//! The parts compensate on-chip; the host only scales the result. Two
//! incompatible digital interfaces exist, so there are two drivers:
//!
//! * [`Xgzp`]: the command protocol at address 0x6D used by the XGZP6897D
//!   and by XGZP6847D part numbers without a `C` after `6847D`.
//!   [`start_measurement`] writes `0x0A` to `CMD` (0x30), a combined
//!   temperature + pressure conversion, and returns 20 ms (the datasheet's
//!   alternative to polling `Sco`). The part idles by itself afterwards.
//!   Pressure is the signed 24-bit result divided by a range dependent
//!   factor K ([`k_for_full_scale`]); temperature is signed 16-bit / 256.
//! * [`XgzpC`]: the register protocol at address 0x58 of the
//!   `XGZP6847DC…` parts. One-shot mode measures either temperature or
//!   pressure, so a sample takes two conversions, like the BMP180:
//!   [`start_measurement`] starts the temperature conversion; the first
//!   [`read_measurement`] reads it, starts the pressure conversion and
//!   returns [`Error::Pending`] with the wait; the second call returns the
//!   sample. The `measure` helpers follow this phase. Pressure is `raw / 2^21 · (PMAX − PMIN)` with the range of the
//!   part; temperature uses two factory calibration registers read by
//!   [`XgzpC::init`].
//!
//! Pressure can be negative (vacuum and differential ranges), so
//! [`Measurement::pressure`] is signed and [`Readings`] reports it as
//! [`Quantity::GaugePressure`].
//!
//! References: CFSensor XGZP6897D datasheet V2.7 (2023-02), XGZP6847D
//! datasheet V2.8 (2024-09, 0x6D protocol) and V3.2 (2026-08, 0x58
//! protocol).
//!
//! [`start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Address of the command-protocol parts ([`Xgzp`]).
pub const ADDRESS: u8 = 0x6D;
/// Address of the `XGZP6847DC…` parts ([`XgzpC`]).
pub const ADDRESS_C: u8 = 0x58;

/// [`Xgzp`] combined conversion time: the datasheet allows a fixed 20 ms
/// wait instead of polling `Sco`.
pub const CONVERSION_US: WaitUs = 20_000;

/// [`XgzpC`] wait per one-shot conversion. The datasheet gives no
/// conversion time (5 ms response time in normal mode); 20 ms is the wait
/// used by the vendor sample code.
pub const PHASE_US: WaitUs = 20_000;

/// Largest [`XgzpC`] range span (PMAX − PMIN) in Pa for which the full
/// 24-bit output still fits the 0.01 Pa result.
pub const MAX_SPAN_PA: u32 = 5_000_000;

const REG_CMD: u8 = 0x30;
const CMD_COMBINED: u8 = 0x0A;
const SCO: u8 = 1 << 3;
const REG_DATA: u8 = 0x06;

const REG_ID: u8 = 0x00;
const REG_CTRL: u8 = 0x01;
const REG_CFG_OSR: u8 = 0x02;
const REG_P: u8 = 0x04;
const REG_T: u8 = 0x07;
const REG_TCAL: u8 = 0x20;
const ACTIVE: u8 = 1 << 0;
/// `ONE_PT`: set for pressure, clear for temperature ("measure the
/// temperature or pressure signal by clearing or setting").
const ONE_PT_PRESSURE: u8 = 1 << 2;
const MODE_MASK: u8 = 0b11;
const MODE_SLEEP: u8 = 0b00;
const MODE_ONE_SHOT: u8 = 0b10;

/// K factor for an [`Xgzp`] part from the datasheet tables, given the
/// largest absolute pressure of the range in Pa (−100…100, −100…0 and
/// 0…100 kPa all use 100 000, giving 64). Below 1 kPa the XGZP6897D table
/// gives 8192; 260 kPa to 1.5 MPa come from the XGZP6847D table. `None`
/// above 1.5 MPa.
pub const fn k_for_full_scale(pa: u32) -> Option<u16> {
    Some(match pa {
        0..1_000 => 8192,
        1_000..2_000 => 4096,
        2_000..=4_000 => 2048,
        4_001..=8_000 => 1024,
        8_001..=16_000 => 512,
        16_001..=32_000 => 256,
        32_001..=65_000 => 128,
        65_001..=131_000 => 64,
        131_001..=260_000 => 32,
        260_001..=500_000 => 16,
        500_001..1_000_000 => 8,
        1_000_000..=1_500_000 => 4,
        _ => return None,
    })
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Pressure in Pa (negative below the reference port pressure).
    pub pressure: i32,
    /// Pressure in 0.01 Pa, for low ranges.
    pub pressure_centi: i32,
}

impl Measurement {
    fn new(temperature: i32, pressure_centi: i32) -> Self {
        Self {
            temperature,
            pressure: div_round(pressure_centi, 100),
            pressure_centi,
        }
    }
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
        f(Quantity::GaugePressure(self.pressure));
    }
}

/// Division rounding half away from zero; `d > 0`.
const fn div_round(n: i32, d: i32) -> i32 {
    if n >= 0 {
        (n + d / 2) / d
    } else {
        (n - d / 2) / d
    }
}

/// Sign-extend a 24-bit big-endian value.
const fn s24(b: [u8; 3]) -> i32 {
    i32::from_be_bytes([b[0], b[1], b[2], 0]) >> 8
}

/// Decode an [`Xgzp`] data block (registers 0x06..=0x0A).
fn decode_k(raw: &[u8; 5], k: u16) -> Option<Measurement> {
    if k == 0 {
        return None;
    }
    let p = s24([raw[0], raw[1], raw[2]]);
    let t = i32::from(i16::from_be_bytes([raw[3], raw[4]]));
    Some(Measurement::new(
        (t * 100 + 128) >> 8,
        div_round(p * 100, i32::from(k)),
    ))
}

/// `XgzpC` pressure in 0.01 Pa: `raw / 2^21 · (pmax − pmin)`.
fn pressure_span(raw: [u8; 3], pmin: i32, pmax: i32) -> Option<i32> {
    let v = i64::from(s24(raw)) * (i64::from(pmax) - i64::from(pmin)) * 100;
    i32::try_from((v + (1 << 20)) >> 21).ok()
}

/// `2^(-r/10)` in Q16, `r = 0..9`.
const INV_POW2_TENTHS: [i64; 10] = [
    65536, 61147, 57052, 53232, 49667, 46341, 43238, 40342, 37641, 35120,
];

/// [`XgzpC`] temperature calibration (registers 0x20 and 0x21).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TempCalibration {
    offset: i32,
    shift_tenths: u8,
}

impl TempCalibration {
    /// Decode the registers: 0x20 bit 7 = sign, bits 6:0 = exponent of the
    /// offset `±2^n`; 0x21 = ten times the exponent of the divisor
    /// `2^(r/10)`. `None` for 0xFF (a failed read in the vendor code) or an
    /// offset beyond 2^16.
    pub const fn from_regs(eof: u8, shift: u8) -> Option<Self> {
        let exp = eof & 0x7F;
        if exp > 16 || shift == 0xFF {
            return None;
        }
        let magnitude = 1 << exp;
        Some(Self {
            offset: if eof & 0x80 != 0 {
                -magnitude
            } else {
                magnitude
            },
            shift_tenths: shift,
        })
    }

    /// Offset `Byte1` subtracted from the raw temperature.
    pub const fn offset(&self) -> i32 {
        self.offset
    }

    /// Temperature in 0.01 °C:
    /// `(raw − offset) / 2^(shift/10) + 25 °C`.
    pub fn temperature(&self, raw: i16) -> i32 {
        let d = i64::from(raw) - i64::from(self.offset);
        let shift = 16 + u32::from(self.shift_tenths / 10);
        let num = d * 100 * INV_POW2_TENTHS[usize::from(self.shift_tenths % 10)];
        // |num| < 2^41, so the result fits easily.
        ((num + (1 << (shift - 1))) >> shift) as i32 + 2_500
    }
}

/// [`XgzpC`] conversion phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Idle,
    Temperature,
    Pressure(i32),
}

macro_rules! xgzp_common {
    ($name:ident) => {
        impl<I2C> $name<I2C> {
            /// Create a driver at [`crate::ADDRESS`] for a part with factor
            /// `k` (see [`crate::k_for_full_scale`]); no bus traffic.
            pub const fn new(i2c: I2C, k: u16) -> Self {
                Self {
                    i2c,
                    address: crate::ADDRESS,
                    k,
                    started: false,
                }
            }

            /// Use another I2C address (custom parts).
            pub const fn with_address(mut self, address: u8) -> Self {
                self.address = address;
                self
            }

            /// Configured K factor.
            pub const fn k(&self) -> u16 {
                self.k
            }

            fn check_k<E>(&self) -> Result<(), Error<E>> {
                if self.k == 0 {
                    Err(Error::InvalidArgument)
                } else {
                    Ok(())
                }
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }
    };
}

macro_rules! xgzp_c_common {
    ($name:ident) => {
        impl<I2C> $name<I2C> {
            /// Create a driver at [`crate::ADDRESS_C`] for a part with range
            /// `pmin..pmax` in Pa (e.g. −40 000..40 000); no bus traffic.
            /// Call `init` before measuring.
            pub const fn new(i2c: I2C, pmin: i32, pmax: i32) -> Self {
                Self {
                    i2c,
                    address: crate::ADDRESS_C,
                    pmin,
                    pmax,
                    calibration: None,
                    cfg_osr: 0,
                    phase: Phase::Idle,
                }
            }

            /// Use another I2C address (custom parts).
            pub const fn with_address(mut self, address: u8) -> Self {
                self.address = address;
                self
            }

            /// Calibration read by `init`.
            pub const fn calibration(&self) -> Option<crate::TempCalibration> {
                self.calibration
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }

            /// `NotReady` before `init`, `InvalidArgument` for an empty,
            /// reversed or too wide (> [`crate::MAX_SPAN_PA`]) range.
            fn check_start<E>(&self) -> Result<(), Error<E>> {
                let span = i64::from(self.pmax) - i64::from(self.pmin);
                if span <= 0 || span > i64::from(crate::MAX_SPAN_PA) {
                    Err(Error::InvalidArgument)
                } else if self.calibration.is_none() {
                    Err(Error::NotReady)
                } else {
                    Ok(())
                }
            }

            fn check_id<E>(&self, id: u8) -> Result<(), Error<E>> {
                if id >> 1 == self.address {
                    Ok(())
                } else {
                    Err(Error::UnexpectedId(u32::from(id)))
                }
            }

            fn pressure_measurement<E>(
                &mut self,
                raw: [u8; 3],
                temperature: i32,
            ) -> Result<Measurement, Error<E>> {
                self.phase = Phase::Idle;
                let p = pressure_span(raw, self.pmin, self.pmax).ok_or(Error::InvalidData)?;
                Ok(Measurement::new(temperature, p))
            }
        }
    };
}

/// Blocking driver for the 0x6D command protocol (XGZP6897D, XGZP6847D).
#[derive(Debug)]
pub struct Xgzp<I2C> {
    i2c: I2C,
    address: u8,
    k: u16,
    started: bool,
}

xgzp_common!(Xgzp);

impl<I2C: embedded_hal::i2c::I2c> Xgzp<I2C> {
    fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut b = [0];
        self.i2c
            .write_read(self.address, &[reg], &mut b)
            .map_err(Error::bus)?;
        Ok(b[0])
    }

    /// `Sco` is set: a conversion is running.
    pub fn is_busy(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok(self.read_reg(REG_CMD)? & SCO != 0)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Xgzp<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// [`Error::InvalidArgument`] if K is 0.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.check_k()?;
        self.i2c
            .write(self.address, &[REG_CMD, CMD_COMBINED])
            .map_err(Error::bus)?;
        self.started = true;
        Ok(CONVERSION_US)
    }

    /// [`Error::NotReady`] if no conversion was started. Registers are read
    /// one at a time, as in the datasheet.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        if !self.started {
            return Err(Error::NotReady);
        }
        let mut raw = [0; 5];
        for (reg, b) in (REG_DATA..).zip(raw.iter_mut()) {
            *b = self.read_reg(reg)?;
        }
        self.started = false;
        decode_k(&raw, self.k).ok_or(Error::InvalidData)
    }
}

/// Blocking driver for the 0x58 register protocol (`XGZP6847DC…`).
#[derive(Debug)]
pub struct XgzpC<I2C> {
    i2c: I2C,
    address: u8,
    pmin: i32,
    pmax: i32,
    calibration: Option<TempCalibration>,
    cfg_osr: u8,
    phase: Phase,
}

xgzp_c_common!(XgzpC);

impl<I2C: embedded_hal::i2c::I2c> XgzpC<I2C> {
    fn read<const N: usize>(&mut self, reg: u8) -> Result<[u8; N], Error<I2C::Error>> {
        let mut b = [0; N];
        self.i2c
            .write_read(self.address, &[reg], &mut b)
            .map_err(Error::bus)?;
        Ok(b)
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[reg, value])
            .map_err(Error::bus)
    }

    fn trigger(&mut self, pressure: bool) -> Result<(), Error<I2C::Error>> {
        let ctrl = if pressure {
            ACTIVE | ONE_PT_PRESSURE
        } else {
            ACTIVE
        };
        self.write_reg(REG_CTRL, ctrl)?;
        self.write_reg(REG_CFG_OSR, self.cfg_osr | MODE_ONE_SHOT)
    }

    /// Check the ID register (it holds the I2C address in bits 7:1).
    pub fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
        let [id] = self.read(REG_ID)?;
        self.check_id(id)
    }

    /// Probe, read the temperature calibration and leave the power-on
    /// normal mode for sleep mode (keeping the factory oversampling).
    pub fn init(&mut self) -> Result<TempCalibration, Error<I2C::Error>> {
        self.probe()?;
        let [eof] = self.read(REG_TCAL)?;
        let [shift] = self.read(REG_TCAL + 1)?;
        let cal = TempCalibration::from_regs(eof, shift).ok_or(Error::InvalidData)?;
        let [cfg] = self.read(REG_CFG_OSR)?;
        self.cfg_osr = cfg & !MODE_MASK;
        self.write_reg(REG_CFG_OSR, self.cfg_osr | MODE_SLEEP)?;
        self.calibration = Some(cal);
        self.phase = Phase::Idle;
        Ok(cal)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for XgzpC<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Start the temperature conversion. [`Error::NotReady`] before `init`;
    /// [`Error::InvalidArgument`] if the range is reversed or wider than
    /// [`MAX_SPAN_PA`].
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.check_start()?;
        self.trigger(false)?;
        self.phase = Phase::Temperature;
        Ok(PHASE_US)
    }

    /// First call after a start: read the temperature, start the pressure
    /// conversion, return [`Error::Pending`]. Second call: the sample.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        match (self.phase, self.calibration) {
            (Phase::Temperature, Some(cal)) => {
                let raw = self.read::<2>(REG_T)?;
                let t = cal.temperature(i16::from_be_bytes(raw));
                self.trigger(true)?;
                self.phase = Phase::Pressure(t);
                Err(Error::Pending(PHASE_US))
            }
            (Phase::Pressure(t), _) => {
                let raw = self.read::<3>(REG_P)?;
                self.pressure_measurement(raw, t)
            }
            _ => Err(Error::NotReady),
        }
    }
}

/// Async drivers.
pub mod asynch {
    use super::{
        ACTIVE, CMD_COMBINED, CONVERSION_US, MODE_MASK, MODE_ONE_SHOT, MODE_SLEEP, Measurement,
        ONE_PT_PRESSURE, PHASE_US, Phase, REG_CFG_OSR, REG_CMD, REG_CTRL, REG_DATA, REG_ID, REG_P,
        REG_T, REG_TCAL, SCO, TempCalibration, decode_k, pressure_span,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Xgzp`].
    #[derive(Debug)]
    pub struct Xgzp<I2C> {
        i2c: I2C,
        address: u8,
        k: u16,
        started: bool,
    }

    xgzp_common!(Xgzp);

    impl<I2C: I2c> Xgzp<I2C> {
        async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
            let mut b = [0];
            self.i2c
                .write_read(self.address, &[reg], &mut b)
                .await
                .map_err(Error::bus)?;
            Ok(b[0])
        }

        /// See [`super::Xgzp::is_busy`].
        pub async fn is_busy(&mut self) -> Result<bool, Error<I2C::Error>> {
            Ok(self.read_reg(REG_CMD).await? & SCO != 0)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Xgzp<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.check_k()?;
            self.i2c
                .write(self.address, &[REG_CMD, CMD_COMBINED])
                .await
                .map_err(Error::bus)?;
            self.started = true;
            Ok(CONVERSION_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if !self.started {
                return Err(Error::NotReady);
            }
            let mut raw = [0; 5];
            for (reg, b) in (REG_DATA..).zip(raw.iter_mut()) {
                *b = self.read_reg(reg).await?;
            }
            self.started = false;
            decode_k(&raw, self.k).ok_or(Error::InvalidData)
        }
    }

    /// Async twin of [`super::XgzpC`].
    #[derive(Debug)]
    pub struct XgzpC<I2C> {
        i2c: I2C,
        address: u8,
        pmin: i32,
        pmax: i32,
        calibration: Option<TempCalibration>,
        cfg_osr: u8,
        phase: Phase,
    }

    xgzp_c_common!(XgzpC);

    impl<I2C: I2c> XgzpC<I2C> {
        async fn read<const N: usize>(&mut self, reg: u8) -> Result<[u8; N], Error<I2C::Error>> {
            let mut b = [0; N];
            self.i2c
                .write_read(self.address, &[reg], &mut b)
                .await
                .map_err(Error::bus)?;
            Ok(b)
        }

        async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[reg, value])
                .await
                .map_err(Error::bus)
        }

        async fn trigger(&mut self, pressure: bool) -> Result<(), Error<I2C::Error>> {
            let ctrl = if pressure {
                ACTIVE | ONE_PT_PRESSURE
            } else {
                ACTIVE
            };
            self.write_reg(REG_CTRL, ctrl).await?;
            self.write_reg(REG_CFG_OSR, self.cfg_osr | MODE_ONE_SHOT)
                .await
        }

        /// See [`super::XgzpC::probe`].
        pub async fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            let [id] = self.read(REG_ID).await?;
            self.check_id(id)
        }

        /// See [`super::XgzpC::init`].
        pub async fn init(&mut self) -> Result<TempCalibration, Error<I2C::Error>> {
            self.probe().await?;
            let [eof] = self.read(REG_TCAL).await?;
            let [shift] = self.read(REG_TCAL + 1).await?;
            let cal = TempCalibration::from_regs(eof, shift).ok_or(Error::InvalidData)?;
            let [cfg] = self.read(REG_CFG_OSR).await?;
            self.cfg_osr = cfg & !MODE_MASK;
            self.write_reg(REG_CFG_OSR, self.cfg_osr | MODE_SLEEP)
                .await?;
            self.calibration = Some(cal);
            self.phase = Phase::Idle;
            Ok(cal)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for XgzpC<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.check_start()?;
            self.trigger(false).await?;
            self.phase = Phase::Temperature;
            Ok(PHASE_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            match (self.phase, self.calibration) {
                (Phase::Temperature, Some(cal)) => {
                    let raw = self.read::<2>(REG_T).await?;
                    let t = cal.temperature(i16::from_be_bytes(raw));
                    self.trigger(true).await?;
                    self.phase = Phase::Pressure(t);
                    Err(Error::Pending(PHASE_US))
                }
                (Phase::Pressure(t), _) => {
                    let raw = self.read::<3>(REG_P).await?;
                    self.pressure_measurement(raw, t)
                }
                _ => Err(Error::NotReady),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    #[test]
    fn k_table() {
        assert_eq!(k_for_full_scale(500), Some(8192));
        assert_eq!(k_for_full_scale(1_000), Some(4096));
        assert_eq!(k_for_full_scale(2_000), Some(2048));
        assert_eq!(k_for_full_scale(4_000), Some(2048));
        assert_eq!(k_for_full_scale(4_001), Some(1024));
        assert_eq!(k_for_full_scale(40_000), Some(128));
        // Datasheet note: −100…100 kPa uses 64.
        assert_eq!(k_for_full_scale(100_000), Some(64));
        assert_eq!(k_for_full_scale(260_000), Some(32));
        assert_eq!(k_for_full_scale(300_000), Some(16));
        assert_eq!(k_for_full_scale(700_000), Some(8));
        assert_eq!(k_for_full_scale(1_500_000), Some(4));
        assert_eq!(k_for_full_scale(1_500_001), None);
    }

    #[test]
    fn decode_k_values() {
        // 100 000 / 64 = 1562.5 Pa; 0x1900 / 256 = 25 °C.
        let m = decode_k(&[0x01, 0x86, 0xA0, 0x19, 0x00], 64).unwrap();
        assert_eq!(m, Measurement::new(2_500, 156_250));
        assert_eq!(m.pressure, 1_563);
        // −64 / 64 = −1 Pa; −1 °C; negative gauge pressure is reported.
        let m = decode_k(&[0xFF, 0xFF, 0xC0, 0xFF, 0x00], 64).unwrap();
        assert_eq!(
            (m.pressure, m.pressure_centi, m.temperature),
            (-1, -100, -100)
        );
        let mut q = Vec::new();
        m.for_each(&mut |x| q.push(x));
        assert_eq!(
            q,
            [Quantity::Temperature(-100), Quantity::GaugePressure(-1)]
        );
        assert_eq!(decode_k(&[0; 5], 0), None);
    }

    #[test]
    fn span_pressure() {
        // 2^21 is full scale for 0…100 kPa.
        assert_eq!(pressure_span([0x20, 0, 0], 0, 100_000), Some(10_000_000));
        assert_eq!(pressure_span([0xE0, 0, 0], 0, 100_000), Some(-10_000_000));
        // 1 LSB at 200 kPa span = 9.54 cPa.
        assert_eq!(pressure_span([0, 0, 1], -100_000, 100_000), Some(10));
        assert_eq!(
            pressure_span([0x7F, 0xFF, 0xFF], 0, 1_000_000),
            Some(399_999_952)
        );
        assert_eq!(
            pressure_span([0x7F, 0xFF, 0xFF], -3_000_000, 3_000_000),
            None
        );
    }

    #[test]
    #[allow(clippy::float_arithmetic)]
    fn temperature_calibration() {
        // Datasheet example: 0x8D = −2^13, 0x46 = 70 → 2^7.
        let cal = TempCalibration::from_regs(0x8D, 0x46).unwrap();
        assert_eq!(cal.offset(), -8192);
        assert_eq!(cal.temperature(-8192), 2_500);
        assert_eq!(cal.temperature(-8064), 2_600);
        assert_eq!(cal.temperature(-5000), 4_994);
        assert_eq!(
            TempCalibration::from_regs(0x0C, 0x46).unwrap().offset(),
            4096
        );
        assert_eq!(TempCalibration::from_regs(0xFF, 0x46), None);
        assert_eq!(TempCalibration::from_regs(0x0D, 0xFF), None);
        // Fractional divisors as in the vendor code (pow(2, reg / 10.0)).
        for shift in [60u8, 65, 71, 79, 83] {
            let cal = TempCalibration::from_regs(0x0D, shift).unwrap();
            for raw in [i16::MIN, -20_000, 0, 8_192, 12_345, i16::MAX] {
                let f = (f64::from(raw) - 8192.0) / 2f64.powf(f64::from(shift) / 10.0) + 25.0;
                let got = f64::from(cal.temperature(raw)) / 100.0;
                assert!((got - f).abs() <= 0.006, "{shift} {raw}: {got} vs {f}");
            }
        }
    }

    #[test]
    fn xgzp_cycle() {
        let mut ex = vec![I2cTx::write(ADDRESS, vec![0x30, 0x0A])];
        ex.push(I2cTx::write_read(ADDRESS, vec![0x30], vec![0x0A]));
        for (reg, v) in [
            (0x06, 0x01),
            (0x07, 0x86),
            (0x08, 0xA0),
            (0x09, 0x19),
            (0x0A, 0x80),
        ] {
            ex.push(I2cTx::write_read(ADDRESS, vec![reg], vec![v]));
        }
        let mut i2c = I2cMock::new(&ex);
        let mut x = Xgzp::new(&mut i2c, 64);
        assert_eq!(x.read_measurement(), Err(Error::NotReady));
        assert_eq!(x.start_measurement().unwrap(), 20_000);
        assert!(x.is_busy().unwrap());
        let m = x.read_measurement().unwrap();
        assert_eq!((m.temperature, m.pressure), (2_550, 1_563));
        assert_eq!(x.read_measurement(), Err(Error::NotReady));
        i2c.done();
    }

    fn c_init(ex: &mut Vec<I2cTx>) {
        ex.extend([
            I2cTx::write_read(ADDRESS_C, vec![0x00], vec![0xB0]),
            I2cTx::write_read(ADDRESS_C, vec![0x20], vec![0x8D]),
            I2cTx::write_read(ADDRESS_C, vec![0x21], vec![0x46]),
            I2cTx::write_read(ADDRESS_C, vec![0x02], vec![0x55]),
            I2cTx::write(ADDRESS_C, vec![0x02, 0x54]),
        ]);
    }

    fn c_cycle(ex: &mut Vec<I2cTx>) {
        ex.extend([
            I2cTx::write(ADDRESS_C, vec![0x01, 0x01]),
            I2cTx::write(ADDRESS_C, vec![0x02, 0x56]),
            I2cTx::write_read(ADDRESS_C, vec![0x07], vec![0xE0, 0x80]),
            I2cTx::write(ADDRESS_C, vec![0x01, 0x05]),
            I2cTx::write(ADDRESS_C, vec![0x02, 0x56]),
            I2cTx::write_read(ADDRESS_C, vec![0x04], vec![0x10, 0x00, 0x00]),
        ]);
    }

    #[test]
    fn xgzp_c_two_phase() {
        let mut ex = vec![I2cTx::write_read(ADDRESS_C, vec![0x00], vec![0x6D])];
        c_init(&mut ex);
        c_cycle(&mut ex);
        let mut i2c = I2cMock::new(&ex);
        let mut x = XgzpC::new(&mut i2c, -40_000, 40_000);
        assert_eq!(x.probe(), Err(Error::UnexpectedId(0x6D)));
        assert_eq!(x.start_measurement(), Err(Error::NotReady));
        x.init().unwrap();
        assert_eq!(x.read_measurement(), Err(Error::NotReady));
        assert_eq!(x.start_measurement().unwrap(), PHASE_US);
        // 0xE080 = −8064 → 26 °C; pressure conversion started.
        assert_eq!(x.read_measurement(), Err(Error::Pending(PHASE_US)));
        // 2^20 / 2^21 · 80 kPa = 40 kPa.
        let m = x.read_measurement().unwrap();
        assert_eq!((m.temperature, m.pressure), (2_600, 40_000));
        assert_eq!(x.read_measurement(), Err(Error::NotReady));
        i2c.done();
    }

    #[test]
    fn invalid_arguments() {
        let mut i2c = I2cMock::new(&[]);
        assert_eq!(
            Xgzp::new(&mut i2c, 0).start_measurement(),
            Err(Error::InvalidArgument)
        );
        for (pmin, pmax) in [(0, 0), (40_000, -40_000), (-3_000_000, 3_000_000)] {
            assert_eq!(
                XgzpC::new(&mut i2c, pmin, pmax).start_measurement(),
                Err(Error::InvalidArgument)
            );
        }
        i2c.done();
    }

    struct RecordingDelay(u32);
    impl embedded_hal::delay::DelayNs for RecordingDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.0 += ns / 1_000;
        }
    }
    impl embedded_hal_async::delay::DelayNs for RecordingDelay {
        async fn delay_ns(&mut self, ns: u32) {
            self.0 += ns / 1_000;
        }
    }

    #[test]
    fn measure_follows_pending_phase() {
        let mut ex = Vec::new();
        c_init(&mut ex);
        c_cycle(&mut ex);
        let mut i2c = I2cMock::new(&ex);
        let mut x = XgzpC::new(&mut i2c, -40_000, 40_000);
        x.init().unwrap();
        let mut delay = RecordingDelay(0);
        let m = zigbee_sensor::blocking::measure(&mut x, &mut delay).unwrap();
        assert_eq!(m.pressure, 40_000);
        assert_eq!(delay.0, 2 * PHASE_US);
        i2c.done();

        let mut ex = Vec::new();
        c_init(&mut ex);
        c_cycle(&mut ex);
        let mut i2c = I2cMock::new(&ex);
        let mut x = asynch::XgzpC::new(&mut i2c, -40_000, 40_000);
        block_on(x.init()).unwrap();
        let mut delay = RecordingDelay(0);
        let m = block_on(zigbee_sensor::asynch::Sensor::measure(&mut x, &mut delay)).unwrap();
        assert_eq!((m.temperature, m.pressure), (2_600, 40_000));
        assert_eq!(delay.0, 2 * PHASE_US);
        i2c.done();
    }

    #[test]
    fn async_xgzp_measure() {
        use zigbee_sensor::asynch::Sensor as _;
        let mut ex = vec![I2cTx::write(ADDRESS, vec![0x30, 0x0A])];
        for (reg, v) in [
            (0x06, 0xFF),
            (0x07, 0xFF),
            (0x08, 0x00),
            (0x09, 0xF9),
            (0x0A, 0x80),
        ] {
            ex.push(I2cTx::write_read(ADDRESS, vec![reg], vec![v]));
        }
        let mut i2c = I2cMock::new(&ex);
        let mut x = asynch::Xgzp::new(&mut i2c, 128);
        let mut delay = RecordingDelay(0);
        let m = block_on(x.measure(&mut delay)).unwrap();
        // −256 / 128 = −2 Pa; 0xF980 / 256 = −6.5 °C.
        assert_eq!((m.pressure, m.temperature), (-2, -650));
        assert_eq!(delay.0, CONVERSION_US);
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
