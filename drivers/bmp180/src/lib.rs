//! Bosch BMP180 and BMP085 pressure and temperature sensor (I2C).
//!
//! The part has no combined conversion: temperature and pressure are two
//! separate conversions, and the pressure compensation needs the
//! temperature of the same cycle. The driver is a small state machine:
//!
//! 1. `start_measurement` starts the temperature conversion and returns its
//!    wait (4.5 ms).
//! 2. The first `read_measurement` reads the raw temperature, starts the
//!    pressure conversion and returns [`Error::NotReady`];
//!    [`Bmp180::pending_wait_us`] then gives the pressure conversion time
//!    for the configured oversampling.
//! 3. The second `read_measurement` reads the raw pressure and returns the
//!    compensated [`Measurement`].
//!
//! Because of step 2 the generic `measure` helpers of `zigbee-sensor`
//! (start, one wait, one read) stop with `NotReady`; use
//! [`Bmp180::measure_full`] instead, or drive the three steps from the
//! application's scheduler. `read_measurement` also returns `NotReady`
//! while the conversion is still running (`ctrl_meas.sco`), without
//! changing state.
//!
//! The BMP180 enters standby (0.1 µA typ.) after every conversion; there
//! is no sleep command.
//!
//! Compensation uses the integer algorithm of the datasheet (32-bit).
//!
//! Reference: Bosch "BMP180 Digital pressure sensor" datasheet,
//! BST-BMP180-DS000-09, rev. 2.5 (sections 3.3-3.5, 5); the BMP085
//! (BST-BMP085-DS000-06) uses the same registers and algorithm.
//!
//! ```ignore
//! let mut bmp = Bmp180::new(i2c, ADDRESS).with_oversampling(Oversampling::Standard);
//! let mut wait_us = bmp.start_measurement()?;
//! let m = loop {
//!     // ... let the Zigbee stack run / sleep for wait_us ...
//!     match bmp.read_measurement() {
//!         Err(Error::NotReady) => wait_us = bmp.pending_wait_us(),
//!         other => break other?,
//!     }
//! };
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x77;

/// `id` register value of the BMP180 and BMP085.
pub const CHIP_ID: u8 = 0x55;

const REG_CALIB: u8 = 0xAA;
const REG_ID: u8 = 0xD0;
const REG_SOFT_RESET: u8 = 0xE0;
const REG_CTRL_MEAS: u8 = 0xF4;

const CMD_SOFT_RESET: u8 = 0xB6;
const CMD_TEMPERATURE: u8 = 0x2E;
const CMD_PRESSURE: u8 = 0x34;
const CTRL_SCO: u8 = 1 << 5;

const LEN_CALIB: usize = 22;
/// `ctrl_meas`, a reserved byte and `out_msb`/`out_lsb`/`out_xlsb`.
const LEN_FRAME: usize = 5;

/// Start-up time after power-on or soft reset (datasheet t_Startup 10 ms).
pub const RESET_WAIT_US: WaitUs = 10_000;

/// Maximum temperature conversion time (datasheet 3.3.1).
pub const TEMPERATURE_CONVERSION_US: WaitUs = 4_500;

/// Pressure oversampling (`oss`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Oversampling {
    /// 1 sample, 4.5 ms, 0.06 hPa RMS noise.
    UltraLowPower = 0,
    /// 2 samples, 7.5 ms, 0.05 hPa RMS noise.
    #[default]
    Standard = 1,
    /// 4 samples, 13.5 ms, 0.04 hPa RMS noise.
    HighResolution = 2,
    /// 8 samples, 25.5 ms, 0.03 hPa RMS noise.
    UltraHighResolution = 3,
}

impl Oversampling {
    /// Maximum pressure conversion time (datasheet 3.3.1).
    pub const fn conversion_us(self) -> WaitUs {
        match self {
            Self::UltraLowPower => 4_500,
            Self::Standard => 7_500,
            Self::HighResolution => 13_500,
            Self::UltraHighResolution => 25_500,
        }
    }

    const fn command(self) -> u8 {
        CMD_PRESSURE | ((self as u8) << 6)
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

/// Factory calibration (E²PROM 0xAA..0xBF, datasheet 3.4).
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    pub ac1: i16,
    pub ac2: i16,
    pub ac3: i16,
    pub ac4: u16,
    pub ac5: u16,
    pub ac6: u16,
    pub b1: i16,
    pub b2: i16,
    pub mb: i16,
    pub mc: i16,
    pub md: i16,
}

impl Calibration {
    /// Decode the 22 calibration bytes (big-endian words). `None` if a word
    /// reads 0x0000 or 0xFFFF, which the datasheet defines as a bad
    /// E²PROM read.
    pub fn parse(raw: &[u8; LEN_CALIB]) -> Option<Self> {
        let mut w = [0u16; LEN_CALIB / 2];
        for (word, bytes) in w.iter_mut().zip(raw.chunks_exact(2)) {
            *word = u16::from_be_bytes([bytes[0], bytes[1]]);
            if *word == 0 || *word == 0xFFFF {
                return None;
            }
        }
        let s = |i: usize| w[i] as i16;
        Some(Self {
            ac1: s(0),
            ac2: s(1),
            ac3: s(2),
            ac4: w[3],
            ac5: w[4],
            ac6: w[5],
            b1: s(6),
            b2: s(7),
            mb: s(8),
            mc: s(9),
            md: s(10),
        })
    }

    /// Datasheet 3.5 algorithm: raw temperature `ut` and raw pressure `up`
    /// (already shifted by `8 - oss`). `None` on a division by zero, which
    /// only invalid calibration can cause.
    pub fn compensate(&self, ut: u16, up: u32, oss: Oversampling) -> Option<Measurement> {
        let oss = oss as u32;
        let x1 = (i32::from(ut) - i32::from(self.ac6)).wrapping_mul(i32::from(self.ac5)) >> 15;
        let x2 = (i32::from(self.mc) << 11).checked_div(x1.wrapping_add(i32::from(self.md)))?;
        let b5 = x1.wrapping_add(x2);
        // B5 is in 1/16 of 0.1 °C.
        let temperature = b5.wrapping_mul(10).wrapping_add(8) >> 4;

        let b6 = b5.wrapping_sub(4_000);
        let b6_sq = b6.wrapping_mul(b6) >> 12;
        let x1 = i32::from(self.b2).wrapping_mul(b6_sq) >> 11;
        let x2 = i32::from(self.ac2).wrapping_mul(b6) >> 11;
        let x3 = x1.wrapping_add(x2);
        let b3 = ((i32::from(self.ac1).wrapping_mul(4).wrapping_add(x3) << oss) + 2) / 4;
        let x1 = i32::from(self.ac3).wrapping_mul(b6) >> 13;
        let x2 = i32::from(self.b1).wrapping_mul(b6_sq) >> 16;
        let x3 = (x1.wrapping_add(x2).wrapping_add(2)) >> 2;
        let b4 = (u32::from(self.ac4).wrapping_mul(x3.wrapping_add(32_768) as u32)) >> 15;
        let b7 = up.wrapping_sub(b3 as u32).wrapping_mul(50_000 >> oss);
        let p = if b7 < 0x8000_0000 {
            b7.wrapping_mul(2).checked_div(b4)?
        } else {
            b7.checked_div(b4)?.wrapping_mul(2)
        } as i32;
        let x1 = ((p >> 8).wrapping_mul(p >> 8)).wrapping_mul(3_038) >> 16;
        let x2 = (-7_357i32).wrapping_mul(p) >> 16;
        let pressure = p.wrapping_add((x1.wrapping_add(x2).wrapping_add(3_791)) >> 4);
        Some(Measurement {
            temperature,
            pressure: pressure.max(0) as u32,
        })
    }
}

/// Where the two-phase cycle stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// No conversion started; `read_measurement` returns `NotReady`.
    Idle,
    /// Temperature conversion running.
    Temperature,
    /// Pressure conversion running (raw temperature captured).
    Pressure,
}

enum Step {
    /// Raw temperature captured: write this `ctrl_meas` to start pressure.
    StartPressure(u8),
    Done(Measurement),
}

#[derive(Debug)]
struct State {
    oss: Oversampling,
    calibration: Option<Calibration>,
    phase: Phase,
    ut: u16,
}

impl State {
    const fn new() -> Self {
        Self {
            oss: Oversampling::Standard,
            calibration: None,
            phase: Phase::Idle,
            ut: 0,
        }
    }

    const fn pending_wait_us(&self) -> WaitUs {
        match self.phase {
            Phase::Idle => 0,
            Phase::Temperature => TEMPERATURE_CONVERSION_US,
            Phase::Pressure => self.oss.conversion_us(),
        }
    }

    fn calibrated<E>(raw: &[u8; LEN_CALIB]) -> Result<Calibration, Error<E>> {
        Calibration::parse(raw).ok_or(Error::InvalidData)
    }

    fn on_frame<E>(&mut self, frame: &[u8; LEN_FRAME]) -> Result<Step, Error<E>> {
        if self.phase == Phase::Idle || frame[0] & CTRL_SCO != 0 {
            return Err(Error::NotReady);
        }
        let [_, _, msb, lsb, xlsb] = *frame;
        if self.phase == Phase::Temperature {
            self.ut = u16::from_be_bytes([msb, lsb]);
            return Ok(Step::StartPressure(self.oss.command()));
        }
        self.phase = Phase::Idle;
        let up =
            (u32::from(msb) << 16 | u32::from(lsb) << 8 | u32::from(xlsb)) >> (8 - self.oss as u32);
        let calibration = self.calibration.as_ref().ok_or(Error::NotReady)?;
        calibration
            .compensate(self.ut, up, self.oss)
            .map(Step::Done)
            .ok_or(Error::InvalidData)
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                state: State::new(),
            }
        }

        /// Select the pressure oversampling (default `Standard`).
        pub const fn with_oversampling(mut self, oss: Oversampling) -> Self {
            self.state.oss = oss;
            self
        }

        /// Change the pressure oversampling for the next cycle.
        pub fn set_oversampling(&mut self, oss: Oversampling) {
            self.state.oss = oss;
        }

        /// Calibration read by `init` (`None` before).
        pub const fn calibration(&self) -> Option<&Calibration> {
            self.state.calibration.as_ref()
        }

        /// Current step of the two-phase cycle.
        pub const fn phase(&self) -> Phase {
            self.state.phase
        }

        /// Time to wait before the next `read_measurement`: the conversion
        /// currently running (0 when idle).
        pub const fn pending_wait_us(&self) -> WaitUs {
            self.state.pending_wait_us()
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Bmp180<I2C> {
    i2c: I2C,
    address: u8,
    state: State,
}

impl<I2C> Bmp180<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Bmp180<I2C> {
    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(self.address, &[reg], buf)
            .map_err(Error::bus)
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[reg, value])
            .map_err(Error::bus)
    }

    /// Check the `id` register.
    pub fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut id = [0];
        self.read(REG_ID, &mut id)?;
        if id[0] == CHIP_ID {
            Ok(())
        } else {
            Err(Error::UnexpectedId(id[0].into()))
        }
    }

    /// Soft reset. Wait [`RESET_WAIT_US`] before the next access.
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(REG_SOFT_RESET, CMD_SOFT_RESET)?;
        self.state.calibration = None;
        self.state.phase = Phase::Idle;
        Ok(RESET_WAIT_US)
    }

    /// Check the ID and read the calibration. Called by `start_measurement`
    /// if needed. [`Error::InvalidData`] if the E²PROM read is invalid.
    pub fn init(&mut self) -> Result<Calibration, Error<I2C::Error>> {
        self.probe()?;
        let mut raw = [0; LEN_CALIB];
        self.read(REG_CALIB, &mut raw)?;
        let calibration = State::calibrated(&raw)?;
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
        // Two phases, plus a few retries if a conversion is still running.
        for _ in 0..4 {
            delay.delay_us(wait);
            match self.read_measurement() {
                Err(Error::NotReady) => wait = self.pending_wait_us(),
                other => return other,
            }
        }
        Err(Error::Timeout)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Bmp180<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Start the temperature conversion (first phase).
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        if self.state.calibration.is_none() {
            self.init()?;
        }
        self.write(REG_CTRL_MEAS, CMD_TEMPERATURE)?;
        self.state.phase = Phase::Temperature;
        Ok(TEMPERATURE_CONVERSION_US)
    }

    /// After the temperature phase: start the pressure conversion and
    /// return [`Error::NotReady`] (wait [`Bmp180::pending_wait_us`]).
    /// After the pressure phase: the measurement.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        if self.state.phase == Phase::Idle {
            return Err(Error::NotReady);
        }
        let mut frame = [0; LEN_FRAME];
        self.read(REG_CTRL_MEAS, &mut frame)?;
        match self.state.on_frame(&frame)? {
            Step::StartPressure(cmd) => {
                self.write(REG_CTRL_MEAS, cmd)?;
                self.state.phase = Phase::Pressure;
                Err(Error::NotReady)
            }
            Step::Done(m) => Ok(m),
        }
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CHIP_ID, CMD_SOFT_RESET, CMD_TEMPERATURE, Calibration, LEN_CALIB, LEN_FRAME, Measurement,
        Oversampling, Phase, REG_CALIB, REG_CTRL_MEAS, REG_ID, REG_SOFT_RESET, RESET_WAIT_US,
        State, Step, TEMPERATURE_CONVERSION_US,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Bmp180`].
    #[derive(Debug)]
    pub struct Bmp180<I2C> {
        i2c: I2C,
        address: u8,
        state: State,
    }

    impl<I2C> Bmp180<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Bmp180<I2C> {
        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write_read(self.address, &[reg], buf)
                .await
                .map_err(Error::bus)
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[reg, value])
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Bmp180::probe`].
        pub async fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            let mut id = [0];
            self.read(REG_ID, &mut id).await?;
            if id[0] == CHIP_ID {
                Ok(())
            } else {
                Err(Error::UnexpectedId(id[0].into()))
            }
        }

        /// See [`super::Bmp180::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(REG_SOFT_RESET, CMD_SOFT_RESET).await?;
            self.state.calibration = None;
            self.state.phase = Phase::Idle;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Bmp180::init`].
        pub async fn init(&mut self) -> Result<Calibration, Error<I2C::Error>> {
            self.probe().await?;
            let mut raw = [0; LEN_CALIB];
            self.read(REG_CALIB, &mut raw).await?;
            let calibration = State::calibrated(&raw)?;
            self.state.calibration = Some(calibration);
            Ok(calibration)
        }

        /// Run a complete cycle, awaiting both conversions on `delay`.
        pub async fn measure_full<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<Measurement, Error<I2C::Error>> {
            use zigbee_sensor::asynch::Sensor;
            let mut wait = self.start_measurement().await?;
            for _ in 0..4 {
                delay.delay_us(wait).await;
                match self.read_measurement().await {
                    Err(Error::NotReady) => wait = self.pending_wait_us(),
                    other => return other,
                }
            }
            Err(Error::Timeout)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Bmp180<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.state.calibration.is_none() {
                self.init().await?;
            }
            self.write(REG_CTRL_MEAS, CMD_TEMPERATURE).await?;
            self.state.phase = Phase::Temperature;
            Ok(TEMPERATURE_CONVERSION_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if self.state.phase == Phase::Idle {
                return Err(Error::NotReady);
            }
            let mut frame = [0; LEN_FRAME];
            self.read(REG_CTRL_MEAS, &mut frame).await?;
            match self.state.on_frame(&frame)? {
                Step::StartPressure(cmd) => {
                    self.write(REG_CTRL_MEAS, cmd).await?;
                    self.state.phase = Phase::Pressure;
                    Err(Error::NotReady)
                }
                Step::Done(m) => Ok(m),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    /// Datasheet 3.5 example calibration.
    const EXAMPLE: Calibration = Calibration {
        ac1: 408,
        ac2: -72,
        ac3: -14_383,
        ac4: 32_741,
        ac5: 32_757,
        ac6: 23_153,
        b1: 6_190,
        b2: 4,
        mb: -32_768,
        mc: -8_711,
        md: 2_868,
    };

    fn calib_bytes(c: &Calibration) -> Vec<u8> {
        [
            c.ac1 as u16,
            c.ac2 as u16,
            c.ac3 as u16,
            c.ac4,
            c.ac5,
            c.ac6,
            c.b1 as u16,
            c.b2 as u16,
            c.mb as u16,
            c.mc as u16,
            c.md as u16,
        ]
        .iter()
        .flat_map(|w| w.to_be_bytes())
        .collect()
    }

    fn init_transactions() -> Vec<Transaction> {
        vec![
            Transaction::write_read(ADDRESS, vec![REG_ID], vec![CHIP_ID]),
            Transaction::write_read(ADDRESS, vec![REG_CALIB], calib_bytes(&EXAMPLE)),
        ]
    }

    #[test]
    fn datasheet_example() {
        // UT = 27898, UP = 23843, oss = 0 -> 15.0 °C, 69964 Pa.
        let m = EXAMPLE
            .compensate(27_898, 23_843, Oversampling::UltraLowPower)
            .unwrap();
        assert_eq!(
            m,
            Measurement {
                temperature: 1_500,
                pressure: 69_964
            }
        );
    }

    #[test]
    fn calibration_parse_and_invalid_words() {
        let raw: [u8; LEN_CALIB] = calib_bytes(&EXAMPLE).try_into().unwrap();
        assert_eq!(Calibration::parse(&raw), Some(EXAMPLE));
        let mut bad = raw;
        bad[6] = 0xFF;
        bad[7] = 0xFF;
        assert_eq!(Calibration::parse(&bad), None);
        bad[6] = 0;
        bad[7] = 0;
        assert_eq!(Calibration::parse(&bad), None);
    }

    #[test]
    fn compensation_is_panic_free() {
        for ut in (0..=u16::MAX).step_by(997) {
            for up in (0..=0x7_FFFF).step_by(7_919) {
                for oss in [
                    Oversampling::UltraLowPower,
                    Oversampling::UltraHighResolution,
                ] {
                    let _ = EXAMPLE.compensate(ut, up, oss);
                }
            }
        }
        let zero_md = Calibration {
            md: 0,
            ac6: 1_000,
            ..EXAMPLE
        };
        assert_eq!(zero_md.compensate(1_000, 0, Oversampling::Standard), None);
    }

    #[test]
    fn two_phase_cycle() {
        let mut ex = init_transactions();
        ex.extend([
            Transaction::write(ADDRESS, vec![REG_CTRL_MEAS, CMD_TEMPERATURE]),
            // Temperature still converting: no state change.
            Transaction::write_read(ADDRESS, vec![REG_CTRL_MEAS], vec![0x2E, 0, 0, 0, 0]),
            Transaction::write_read(ADDRESS, vec![REG_CTRL_MEAS], vec![0x0E, 0, 0x6C, 0xFA, 0]),
            Transaction::write(ADDRESS, vec![REG_CTRL_MEAS, 0x34]),
            Transaction::write_read(
                ADDRESS,
                vec![REG_CTRL_MEAS],
                vec![0x14, 0, 0x5D, 0x23, 0x00],
            ),
        ]);
        let mut i2c = Mock::new(&ex);
        let mut bmp = Bmp180::new(&mut i2c, ADDRESS).with_oversampling(Oversampling::UltraLowPower);

        assert_eq!(bmp.read_measurement(), Err(Error::NotReady));
        assert_eq!(bmp.start_measurement().unwrap(), 4_500);
        assert_eq!(bmp.read_measurement(), Err(Error::NotReady));
        assert_eq!(bmp.phase(), Phase::Temperature);
        assert_eq!(bmp.read_measurement(), Err(Error::NotReady));
        assert_eq!(bmp.phase(), Phase::Pressure);
        assert_eq!(bmp.pending_wait_us(), 4_500);
        assert_eq!(
            bmp.read_measurement().unwrap(),
            Measurement {
                temperature: 1_500,
                pressure: 69_964
            }
        );
        assert_eq!(bmp.phase(), Phase::Idle);
        i2c.done();
    }

    #[test]
    fn ultra_high_resolution_command_and_shift() {
        // oss 3: ctrl_meas 0xF4, UP = raw >> 5.
        let up = 23_843u32 << 3;
        let raw = (up << 5).to_be_bytes();
        let mut ex = init_transactions();
        ex.extend([
            Transaction::write(ADDRESS, vec![REG_CTRL_MEAS, CMD_TEMPERATURE]),
            Transaction::write_read(ADDRESS, vec![REG_CTRL_MEAS], vec![0x0E, 0, 0x6C, 0xFA, 0]),
            Transaction::write(ADDRESS, vec![REG_CTRL_MEAS, 0xF4]),
            Transaction::write_read(
                ADDRESS,
                vec![REG_CTRL_MEAS],
                vec![0x14, 0, raw[1], raw[2], raw[3]],
            ),
        ]);
        let mut i2c = Mock::new(&ex);
        let mut bmp =
            Bmp180::new(&mut i2c, ADDRESS).with_oversampling(Oversampling::UltraHighResolution);
        bmp.start_measurement().unwrap();
        assert_eq!(bmp.read_measurement(), Err(Error::NotReady));
        assert_eq!(bmp.pending_wait_us(), 25_500);
        let m = bmp.read_measurement().unwrap();
        assert_eq!(
            Some(m),
            EXAMPLE.compensate(27_898, up, Oversampling::UltraHighResolution)
        );
        i2c.done();
    }

    #[test]
    fn wrong_id_and_bad_calibration() {
        let mut bad = calib_bytes(&EXAMPLE);
        bad[0] = 0;
        bad[1] = 0;
        let ex = [
            Transaction::write_read(ADDRESS, vec![REG_ID], vec![0x58]),
            Transaction::write_read(ADDRESS, vec![REG_ID], vec![CHIP_ID]),
            Transaction::write_read(ADDRESS, vec![REG_CALIB], bad),
            Transaction::write(ADDRESS, vec![REG_SOFT_RESET, CMD_SOFT_RESET]),
        ];
        let mut i2c = Mock::new(&ex);
        let mut bmp = Bmp180::new(&mut i2c, ADDRESS);
        assert_eq!(bmp.start_measurement(), Err(Error::UnexpectedId(0x58)));
        assert_eq!(bmp.init(), Err(Error::InvalidData));
        assert_eq!(bmp.soft_reset().unwrap(), RESET_WAIT_US);
        i2c.done();
    }

    #[test]
    fn async_measure_full_awaits_both_conversions() {
        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let mut ex = init_transactions();
        ex.extend([
            Transaction::write(ADDRESS, vec![REG_CTRL_MEAS, CMD_TEMPERATURE]),
            Transaction::write_read(ADDRESS, vec![REG_CTRL_MEAS], vec![0x0E, 0, 0x6C, 0xFA, 0]),
            Transaction::write(ADDRESS, vec![REG_CTRL_MEAS, 0x74]),
            Transaction::write_read(
                ADDRESS,
                vec![REG_CTRL_MEAS],
                vec![0x14, 0, 0x5D, 0x23, 0x00],
            ),
        ]);
        let mut i2c = Mock::new(&ex);
        let mut bmp = asynch::Bmp180::new(&mut i2c, ADDRESS);
        let mut delay = RecordingDelay(0);
        let m = block_on(bmp.measure_full(&mut delay)).unwrap();
        assert_eq!(m.temperature, 1_500);
        assert_eq!(delay.0, 4_500 + 7_500);
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
