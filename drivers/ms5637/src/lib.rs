//! TE Connectivity MS5637-02BA, MS5837-30BA and MS5837-02BA pressure and
//! temperature sensors over I2C (fixed address 0x76).
//!
//! Same command set as the MS5611 family with a 7-word PROM (CRC-4 in the
//! top nibble of word 0) and an extra OSR 8192. The parts have no status
//! register, so the driver is the same two-phase state machine as
//! `zigbee-ms5611` / `zigbee-bmp180`:
//!
//! 1. `start_measurement` starts the temperature conversion (D2) and
//!    returns its wait.
//! 2. The first `read_measurement` reads D2, starts the pressure
//!    conversion (D1) and returns [`Error::NotReady`];
//!    [`Ms5637::pending_wait_us`] then gives the D1 conversion time.
//! 3. The second `read_measurement` returns the compensated
//!    [`Measurement`].
//!
//! Reading before a conversion has finished returns 0 and corrupts it; the
//! driver maps a zero result to [`Error::InvalidData`] and goes back to
//! idle. Waits are the largest datasheet maximum conversion times of the
//! three parts (MS5837-30BA table). The part idles (≤ 0.1 µA) between
//! conversions. [`Ms5637::measure_full`] runs a whole cycle for bring-up.
//!
//! There is no ID register and the three parts share the address, so the
//! [`Variant`] is chosen by the application; it selects the compensation
//! constants. [`Ms5637::init`] reads the PROM and checks its CRC, which
//! doubles as a presence check. Send [`Ms5637::soft_reset`] once after
//! power-up and wait [`RESET_WAIT_US`].
//!
//! Compensation is the datasheet integer algorithm including the
//! second-order correction, in 64-bit arithmetic; divisions by 2^n are
//! arithmetic shifts (floor), which reproduces the datasheet example
//! values and avoids 64-bit division on Cortex-M0.
//!
//! References:
//! * TE "MS5637-02BA03 Low Voltage Barometric Pressure Sensor" datasheet,
//!   06/2017.
//! * TE "MS5837-02BA01" datasheet, REV A8, 12/2019.
//! * TE "MS5837-30BA" datasheet, REV C2, 12/2019.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed 7-bit I2C address of all three parts.
pub const ADDRESS: u8 = 0x76;

const CMD_RESET: u8 = 0x1E;
const CMD_ADC_READ: u8 = 0x00;
const CMD_CONVERT_D1: u8 = 0x40;
const CMD_CONVERT_D2: u8 = 0x50;
const CMD_PROM_READ: u8 = 0xA0;

/// PROM reload time after a reset (2.8 ms), with margin.
pub const RESET_WAIT_US: WaitUs = 3_000;

/// Part, selecting the compensation constants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// MS5637-02BA (300-1200 mbar).
    Ms5637,
    /// MS5837-30BA (0-30 bar, gel-filled; pressure resolution 10 Pa).
    Ms5837_30Ba,
    /// MS5837-02BA (300-1200 mbar, gel-filled).
    Ms5837_02Ba,
}

/// Oversampling ratio of one conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(missing_docs)]
pub enum Osr {
    X256 = 0,
    X512 = 1,
    X1024 = 2,
    X2048 = 3,
    X4096 = 4,
    X8192 = 5,
}

impl Osr {
    /// Maximum conversion time in µs (MS5837-30BA table, the largest of
    /// the three datasheets).
    pub const fn conversion_us(self) -> WaitUs {
        [600, 1_170, 2_280, 4_540, 9_040, 18_080][self as usize]
    }

    const fn cmd_offset(self) -> u8 {
        2 * self as u8
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Pressure (D1) oversampling.
    pub pressure: Osr,
    /// Temperature (D2) oversampling.
    pub temperature: Osr,
}

impl Config {
    /// Highest resolution for both conversions (2 × 18.08 ms).
    pub const HIGH_RES: Self = Self {
        pressure: Osr::X8192,
        temperature: Osr::X8192,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self::HIGH_RES
    }
}

/// One compensated sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Pressure in Pa.
    pub pressure: u32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
        f(Quantity::Pressure(self.pressure));
    }
}

/// Factory calibration C1..C6 (PROM words 1..6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub struct Calibration {
    pub c1: u16,
    pub c2: u16,
    pub c3: u16,
    pub c4: u16,
    pub c5: u16,
    pub c6: u16,
}

/// CRC-4 of the 7 PROM words (datasheet routine): the CRC itself (bits
/// 15:12 of word 0) is excluded and an eighth zero word is appended.
pub fn crc4(prom: &[u16; 7]) -> u8 {
    let mut rem: u16 = 0;
    for i in 0..16 {
        let word = match i >> 1 {
            0 => prom[0] & 0x0FFF,
            7 => 0,
            n => prom[n],
        };
        rem ^= if i % 2 == 1 { word & 0x00FF } else { word >> 8 };
        for _ in 0..8 {
            rem = if rem & 0x8000 != 0 {
                (rem << 1) ^ 0x3000
            } else {
                rem << 1
            };
        }
    }
    (rem >> 12) as u8
}

impl Calibration {
    /// Check the CRC of the 7 PROM words and extract C1..C6.
    pub fn from_prom<E>(prom: &[u16; 7]) -> Result<Self, Error<E>> {
        if crc4(prom) != (prom[0] >> 12) as u8 {
            return Err(Error::Crc);
        }
        Ok(Self {
            c1: prom[1],
            c2: prom[2],
            c3: prom[3],
            c4: prom[4],
            c5: prom[5],
            c6: prom[6],
        })
    }

    /// First- and second-order compensation of the raw pressure `d1` and
    /// temperature `d2` (datasheet "Pressure and temperature calculation"
    /// and "Second order temperature compensation").
    pub fn compensate(&self, variant: Variant, d1: u32, d2: u32) -> Measurement {
        let dt = i64::from(d2) - (i64::from(self.c5) << 8);
        let temp = 2000 + ((dt * i64::from(self.c6)) >> 23);
        let (c2_shift, c4_shift, c1_shift, c3_shift, p_shift) = match variant {
            Variant::Ms5837_30Ba => (16, 7, 15, 8, 13),
            Variant::Ms5637 | Variant::Ms5837_02Ba => (17, 6, 16, 7, 15),
        };
        let off = (i64::from(self.c2) << c2_shift) + ((i64::from(self.c4) * dt) >> c4_shift);
        let sens = (i64::from(self.c1) << c1_shift) + ((i64::from(self.c3) * dt) >> c3_shift);

        let low = (temp - 2000) * (temp - 2000);
        let very_low = (temp + 1500) * (temp + 1500);
        let dt2 = dt * dt;
        let (t2, off2, sens2) = match variant {
            Variant::Ms5637 if temp < 2000 => {
                let (mut o, mut s) = ((61 * low) >> 4, (29 * low) >> 4);
                if temp < -1500 {
                    o += 17 * very_low;
                    s += 9 * very_low;
                }
                ((3 * dt2) >> 33, o, s)
            }
            Variant::Ms5637 => ((5 * dt2) >> 38, 0, 0),
            Variant::Ms5837_02Ba if temp < 2000 => {
                ((11 * dt2) >> 35, (31 * low) >> 3, (63 * low) >> 5)
            }
            Variant::Ms5837_02Ba => (0, 0, 0),
            Variant::Ms5837_30Ba if temp < 2000 => {
                let (mut o, mut s) = ((3 * low) >> 1, (5 * low) >> 3);
                if temp < -1500 {
                    o += 7 * very_low;
                    s += 4 * very_low;
                }
                ((3 * dt2) >> 33, o, s)
            }
            Variant::Ms5837_30Ba => ((2 * dt2) >> 37, low >> 4, 0),
        };
        let temp = temp - t2;
        let off = off - off2;
        let sens = sens - sens2;
        let mut p = (((i64::from(d1) * sens) >> 21) - off) >> p_shift;
        if variant == Variant::Ms5837_30Ba {
            // 0.1 mbar -> Pa.
            p *= 10;
        }
        Measurement {
            temperature: temp.clamp(i32::MIN.into(), i32::MAX.into()) as i32,
            pressure: p.clamp(0, u32::MAX.into()) as u32,
        }
    }
}

fn adc24(b: &[u8; 3]) -> u32 {
    u32::from(b[0]) << 16 | u32::from(b[1]) << 8 | u32::from(b[2])
}

/// Where the two-phase cycle stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// No conversion started; `read_measurement` returns `NotReady`.
    Idle,
    /// Temperature (D2) conversion running.
    Temperature,
    /// Pressure (D1) conversion running (D2 captured).
    Pressure,
}

#[derive(Debug)]
struct State {
    variant: Variant,
    config: Config,
    calibration: Option<Calibration>,
    phase: Phase,
    d2: u32,
}

impl State {
    const fn new(variant: Variant) -> Self {
        Self {
            variant,
            config: Config::HIGH_RES,
            calibration: None,
            phase: Phase::Idle,
            d2: 0,
        }
    }

    const fn pending_wait_us(&self) -> WaitUs {
        match self.phase {
            Phase::Idle => 0,
            Phase::Temperature => self.config.temperature.conversion_us(),
            Phase::Pressure => self.config.pressure.conversion_us(),
        }
    }

    const fn start_cmd(&self) -> u8 {
        CMD_CONVERT_D2 + self.config.temperature.cmd_offset()
    }

    const fn pressure_cmd(&self) -> u8 {
        CMD_CONVERT_D1 + self.config.pressure.cmd_offset()
    }

    /// Consume an ADC result: `None` means "start the pressure phase".
    fn on_result<E>(&mut self, raw: &[u8; 3]) -> Result<Option<Measurement>, Error<E>> {
        let value = adc24(raw);
        if value == 0 {
            // Read too early: the running conversion is spoiled.
            self.phase = Phase::Idle;
            return Err(Error::InvalidData);
        }
        if self.phase == Phase::Temperature {
            self.d2 = value;
            self.phase = Phase::Pressure;
            return Ok(None);
        }
        self.phase = Phase::Idle;
        let c = self.calibration.as_ref().ok_or(Error::NotReady)?;
        Ok(Some(c.compensate(self.variant, value, self.d2)))
    }
}

fn prom_from_bytes(bytes: &[[u8; 2]; 7]) -> [u16; 7] {
    bytes.map(u16::from_be_bytes)
}

macro_rules! common {
    ($name:ident) => {
        impl<I2C> $name<I2C> {
            /// Create a driver; no bus traffic.
            pub const fn new(i2c: I2C, variant: Variant) -> Self {
                Self {
                    i2c,
                    state: State::new(variant),
                }
            }

            /// Select the oversampling used by the next cycle.
            pub const fn with_config(mut self, config: Config) -> Self {
                self.state.config = config;
                self
            }

            /// Change the oversampling (applies to the next command sent).
            pub fn set_config(&mut self, config: Config) {
                self.state.config = config;
            }

            /// Current configuration.
            pub const fn config(&self) -> &Config {
                &self.state.config
            }

            /// Variant selected at construction.
            pub const fn variant(&self) -> Variant {
                self.state.variant
            }

            /// Calibration read by `init` (`None` before).
            pub const fn calibration(&self) -> Option<&Calibration> {
                self.state.calibration.as_ref()
            }

            /// Current step of the two-phase cycle.
            pub const fn phase(&self) -> Phase {
                self.state.phase
            }

            /// Time to wait before the next `read_measurement`: the
            /// conversion currently running (0 when idle).
            pub const fn pending_wait_us(&self) -> WaitUs {
                self.state.pending_wait_us()
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Ms5637<I2C> {
    i2c: I2C,
    state: State,
}

common!(Ms5637);

impl<I2C: embedded_hal::i2c::I2c> Ms5637<I2C> {
    /// Reset (reloads the PROM). Wait [`RESET_WAIT_US`] before `init`.
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.state.calibration = None;
        self.state.phase = Phase::Idle;
        self.i2c.write(ADDRESS, &[CMD_RESET]).map_err(Error::bus)?;
        Ok(RESET_WAIT_US)
    }

    /// Read the 7 PROM words and check the CRC ([`Error::Crc`] on a
    /// mismatch). Called by `start_measurement`.
    pub fn init(&mut self) -> Result<Calibration, Error<I2C::Error>> {
        let mut bytes = [[0u8; 2]; 7];
        for (i, b) in (0u8..).zip(bytes.iter_mut()) {
            self.i2c
                .write_read(ADDRESS, &[CMD_PROM_READ + 2 * i], b)
                .map_err(Error::bus)?;
        }
        let calibration = Calibration::from_prom(&prom_from_bytes(&bytes))?;
        self.state.calibration = Some(calibration);
        Ok(calibration)
    }

    /// Run a complete cycle (temperature then pressure), blocking on
    /// `delay`. For tests and bring-up, not for a Zigbee event loop.
    pub fn measure_full<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Measurement, Error<I2C::Error>> {
        use zigbee_sensor::blocking::Sensor;
        let mut wait = self.start_measurement()?;
        for _ in 0..2 {
            delay.delay_us(wait);
            match self.read_measurement() {
                Err(Error::NotReady) => wait = self.pending_wait_us(),
                other => return other,
            }
        }
        Err(Error::Timeout)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Ms5637<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Start the temperature conversion (first phase).
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        if self.state.calibration.is_none() {
            self.init()?;
        }
        self.i2c
            .write(ADDRESS, &[self.state.start_cmd()])
            .map_err(Error::bus)?;
        self.state.phase = Phase::Temperature;
        Ok(self.state.pending_wait_us())
    }

    /// After the temperature phase: start the pressure conversion and
    /// return [`Error::NotReady`] (wait [`Ms5637::pending_wait_us`]).
    /// After the pressure phase: the measurement.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        if self.state.phase == Phase::Idle {
            return Err(Error::NotReady);
        }
        let mut raw = [0; 3];
        self.i2c
            .write_read(ADDRESS, &[CMD_ADC_READ], &mut raw)
            .map_err(Error::bus)?;
        match self.state.on_result(&raw)? {
            Some(m) => Ok(m),
            None => {
                self.i2c
                    .write(ADDRESS, &[self.state.pressure_cmd()])
                    .map_err(Error::bus)?;
                Err(Error::NotReady)
            }
        }
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Ms5637<I2C> {
    type BusError = I2C::Error;

    /// Forget a running cycle; the part idles by itself after each
    /// conversion (no command needed).
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.state.phase = Phase::Idle;
        Ok(())
    }

    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, CMD_ADC_READ, CMD_PROM_READ, CMD_RESET, Calibration, Config, Measurement, Phase,
        RESET_WAIT_US, State, Variant, prom_from_bytes,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Ms5637`].
    #[derive(Debug)]
    pub struct Ms5637<I2C> {
        i2c: I2C,
        state: State,
    }

    common!(Ms5637);

    impl<I2C: I2c> Ms5637<I2C> {
        /// See [`super::Ms5637::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.state.calibration = None;
            self.state.phase = Phase::Idle;
            self.i2c
                .write(ADDRESS, &[CMD_RESET])
                .await
                .map_err(Error::bus)?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Ms5637::init`].
        pub async fn init(&mut self) -> Result<Calibration, Error<I2C::Error>> {
            let mut bytes = [[0u8; 2]; 7];
            for (i, b) in (0u8..).zip(bytes.iter_mut()) {
                self.i2c
                    .write_read(ADDRESS, &[CMD_PROM_READ + 2 * i], b)
                    .await
                    .map_err(Error::bus)?;
            }
            let calibration = Calibration::from_prom(&prom_from_bytes(&bytes))?;
            self.state.calibration = Some(calibration);
            Ok(calibration)
        }

        /// See [`super::Ms5637::measure_full`].
        pub async fn measure_full<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<Measurement, Error<I2C::Error>> {
            use zigbee_sensor::asynch::Sensor;
            let mut wait = self.start_measurement().await?;
            for _ in 0..2 {
                delay.delay_us(wait).await;
                match self.read_measurement().await {
                    Err(Error::NotReady) => wait = self.pending_wait_us(),
                    other => return other,
                }
            }
            Err(Error::Timeout)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Ms5637<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.state.calibration.is_none() {
                self.init().await?;
            }
            self.i2c
                .write(ADDRESS, &[self.state.start_cmd()])
                .await
                .map_err(Error::bus)?;
            self.state.phase = Phase::Temperature;
            Ok(self.state.pending_wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if self.state.phase == Phase::Idle {
                return Err(Error::NotReady);
            }
            let mut raw = [0; 3];
            self.i2c
                .write_read(ADDRESS, &[CMD_ADC_READ], &mut raw)
                .await
                .map_err(Error::bus)?;
            match self.state.on_result(&raw)? {
                Some(m) => Ok(m),
                None => {
                    self.i2c
                        .write(ADDRESS, &[self.state.pressure_cmd()])
                        .await
                        .map_err(Error::bus)?;
                    Err(Error::NotReady)
                }
            }
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Ms5637<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.state.phase = Phase::Idle;
            Ok(())
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            Ok(0)
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
    use zigbee_sensor::blocking::{LowPower, Sensor};

    /// MS5637 / MS5837-02BA datasheet example coefficients.
    const CAL_A: Calibration = Calibration {
        c1: 46_372,
        c2: 43_981,
        c3: 29_059,
        c4: 27_842,
        c5: 31_553,
        c6: 28_165,
    };

    /// MS5837-30BA datasheet example coefficients.
    const CAL_30: Calibration = Calibration {
        c1: 34_982,
        c2: 36_352,
        c3: 20_328,
        c4: 22_354,
        c5: 26_646,
        c6: 26_146,
    };

    /// PROM with CRC 5 (datasheet routine) in word 0 bits 15:12.
    const PROM: [u16; 7] = [0x50C0, 46_372, 43_981, 29_059, 27_842, 31_553, 28_165];

    #[test]
    fn datasheet_examples() {
        let m = CAL_A.compensate(Variant::Ms5637, 6_465_444, 8_077_636);
        assert_eq!((m.temperature, m.pressure), (2_000, 110_002));
        let m = CAL_A.compensate(Variant::Ms5837_02Ba, 6_465_444, 8_077_636);
        assert_eq!((m.temperature, m.pressure), (2_000, 110_002));
        // 30BA example: TEMP 1981 (19.81 °C), P 39998 (3999.8 mbar).
        let m = CAL_30.compensate(Variant::Ms5837_30Ba, 4_958_179, 6_815_414);
        assert_eq!((m.temperature, m.pressure), (1_981, 399_980));
    }

    #[test]
    fn second_order_branches() {
        // Expected values from the datasheet algorithm evaluated with
        // floor (arithmetic shift) division.
        let a = (CAL_A, 6_465_444);
        let b = (CAL_30, 4_958_179);
        let cases = [
            (Variant::Ms5637, a, 8_500_000, 3_415, 113_417),
            (Variant::Ms5637, a, 7_600_000, 317, 106_002),
            (Variant::Ms5637, a, 6_000_000, -6_483, 86_609),
            (Variant::Ms5837_02Ba, a, 8_500_000, 3_418, 113_417),
            (Variant::Ms5837_02Ba, a, 7_600_000, 323, 105_969),
            (Variant::Ms5837_02Ba, a, 6_000_000, -6_357, 89_949),
            (Variant::Ms5837_30Ba, b, 8_500_000, 7_191, 429_000),
            (Variant::Ms5837_30Ba, b, 7_600_000, 4_418, 412_970),
            (Variant::Ms5837_30Ba, b, 6_000_000, -796, 387_120),
        ];
        for (v, (c, d1), d2, t, p) in cases {
            let m = c.compensate(v, d1, d2);
            assert_eq!((m.temperature, m.pressure), (t, p), "{v:?} D2 {d2}");
        }
    }

    #[test]
    fn crc_and_prom_decode() {
        assert_eq!(crc4(&PROM), 5);
        let p30 = [0x9340, 34_982, 36_352, 20_328, 22_354, 26_646, 26_146];
        assert_eq!(crc4(&p30), 9);
        assert_eq!(Calibration::from_prom::<()>(&p30), Ok(CAL_30));
        let mut bad = PROM;
        bad[2] ^= 0x0004;
        assert_eq!(Calibration::from_prom::<()>(&bad), Err(Error::Crc));
    }

    #[test]
    fn compensation_is_panic_free() {
        let max = Calibration {
            c1: u16::MAX,
            c2: u16::MAX,
            c3: u16::MAX,
            c4: u16::MAX,
            c5: u16::MAX,
            c6: u16::MAX,
        };
        let min = Calibration {
            c5: u16::MAX,
            c6: u16::MAX,
            ..Calibration::default()
        };
        let raws = [0, 1, 1 << 23, (1 << 24) - 1];
        for v in [Variant::Ms5637, Variant::Ms5837_30Ba, Variant::Ms5837_02Ba] {
            for c in [CAL_A, max, min, Calibration::default()] {
                for d1 in raws {
                    for d2 in raws {
                        let _ = c.compensate(v, d1, d2);
                    }
                }
            }
        }
    }

    fn prom_tx(prom: &[u16; 7]) -> Vec<I2cTx> {
        (0u8..)
            .zip(prom)
            .map(|(i, w)| I2cTx::write_read(ADDRESS, vec![0xA0 + 2 * i], w.to_be_bytes().to_vec()))
            .collect()
    }

    fn adc(v: u32) -> Vec<u8> {
        v.to_be_bytes()[1..].to_vec()
    }

    #[test]
    fn two_phase_cycle() {
        let mut ex = vec![I2cTx::write(ADDRESS, vec![0x1E])];
        ex.extend(prom_tx(&PROM));
        ex.extend([
            I2cTx::write(ADDRESS, vec![0x5A]),
            I2cTx::write_read(ADDRESS, vec![0x00], adc(8_077_636)),
            I2cTx::write(ADDRESS, vec![0x4A]),
            I2cTx::write_read(ADDRESS, vec![0x00], adc(6_465_444)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut ms = Ms5637::new(&mut i2c, Variant::Ms5637);
        assert_eq!(ms.read_measurement(), Err(Error::NotReady));
        assert_eq!(ms.soft_reset().unwrap(), RESET_WAIT_US);
        assert_eq!(ms.start_measurement().unwrap(), 18_080);
        assert_eq!(ms.read_measurement(), Err(Error::NotReady));
        assert_eq!(ms.phase(), Phase::Pressure);
        assert_eq!(ms.pending_wait_us(), 18_080);
        let m = ms.read_measurement().unwrap();
        assert_eq!((m.temperature, m.pressure), (2_000, 110_002));
        assert_eq!(ms.phase(), Phase::Idle);
        ms.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn early_read_and_bad_crc() {
        let mut bad = PROM;
        bad[0] ^= 0x1000;
        let mut ex = prom_tx(&bad);
        ex.extend(prom_tx(&PROM));
        ex.extend([
            I2cTx::write(ADDRESS, vec![0x50]),
            I2cTx::write_read(ADDRESS, vec![0x00], adc(0)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut ms = Ms5637::new(&mut i2c, Variant::Ms5837_02Ba).with_config(Config {
            pressure: Osr::X1024,
            temperature: Osr::X256,
        });
        assert_eq!(ms.start_measurement(), Err(Error::Crc));
        assert_eq!(ms.start_measurement().unwrap(), 600);
        assert_eq!(ms.read_measurement(), Err(Error::InvalidData));
        assert_eq!(ms.phase(), Phase::Idle);
        i2c.done();
    }

    #[test]
    fn async_measure_full_waits_both_phases() {
        struct RecordingDelay(Vec<u32>);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0.push(ns / 1_000);
            }
        }

        let p30 = [0x9340, 34_982, 36_352, 20_328, 22_354, 26_646, 26_146];
        let mut ex = prom_tx(&p30);
        ex.extend([
            I2cTx::write(ADDRESS, vec![0x56]),
            I2cTx::write_read(ADDRESS, vec![0x00], adc(6_815_414)),
            I2cTx::write(ADDRESS, vec![0x48]),
            I2cTx::write_read(ADDRESS, vec![0x00], adc(4_958_179)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut ms = asynch::Ms5637::new(&mut i2c, Variant::Ms5837_30Ba).with_config(Config {
            pressure: Osr::X4096,
            temperature: Osr::X2048,
        });
        let mut delay = RecordingDelay(Vec::new());
        let m = block_on(ms.measure_full(&mut delay)).unwrap();
        assert_eq!((m.temperature, m.pressure), (1_981, 399_980));
        assert_eq!(delay.0, [4_540, 9_040]);
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
