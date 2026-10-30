//! Goertek SPL06-001 and SPL07-003 pressure and temperature sensors over
//! I2C or SPI.
//!
//! Command mode measures one quantity per conversion and the pressure
//! compensation needs the temperature of the same cycle, so the driver is
//! a small two-phase state machine (same contract as `zigbee-bmp180`):
//!
//! 1. `start_measurement` starts a temperature conversion
//!    (`MEAS_CTRL = 010`) and returns its wait.
//! 2. The first `read_measurement` reads the raw temperature, starts the
//!    pressure conversion (`MEAS_CTRL = 001`) and returns
//!    [`Error::NotReady`]; [`Spl06::pending_wait_us`] then gives the
//!    pressure conversion time.
//! 3. The second `read_measurement` returns the compensated
//!    [`Measurement`].
//!
//! `read_measurement` also returns `NotReady`, without changing state,
//! while `TMP_RDY` / `PRS_RDY` is still clear. The generic `measure`
//! helpers (start, one wait, one read) stop at step 2; use
//! [`Spl06::measure_full`] or drive the steps from the application. The
//! part returns to standby (< 1 µA) after each conversion.
//!
//! Waits are the datasheet measurement time (table 8:
//! `2 ms + 1.6 ms · oversampling`) plus 10 %.
//!
//! The variant is detected from `ID` (0x10 SPL06-001, 0x11 SPL07-003):
//! * SPL06-001: 18 coefficient bytes; `TMP_CFG.TMP_EXT` is set (the
//!   datasheet asks for the external, MEMS temperature sensor).
//! * SPL07-003: 21 coefficient bytes with the extra `c31` and `c40`
//!   terms; `TMP_EXT` moved to `MEAS_CFG` bit 3 and is left at 0
//!   (internal sensor, as recommended).
//!
//! Compensation (datasheet 5.6 / 4.6) is evaluated in 64-bit fixed point
//! with the scaled raw values in Q24 (clamped to ±2, the normalised range
//! is about ±1); it stays within 1 Pa / 0.01 °C of the float formula.
//!
//! References:
//! * Goertek "SPL06-001 Digital pressure sensor" datasheet version 7.0,
//!   2018-01-26 (sections 5.6, 7, 8).
//! * Goermicro "SPL07-003" datasheet Ver1.0, 2023-02-24 (sections 4.3,
//!   4.6, 6, 7).
//!
//! ```ignore
//! let mut spl = Spl06::new_i2c(i2c, PRIMARY_ADDRESS);
//! let mut wait_us = spl.start_measurement()?;
//! let m = loop {
//!     // ... let the Zigbee stack run / sleep for wait_us ...
//!     match spl.read_measurement() {
//!         Err(Error::NotReady) => wait_us = spl.pending_wait_us(),
//!         other => break other?,
//!     }
//! };
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with SDO floating or high (default).
pub const PRIMARY_ADDRESS: u8 = 0x77;
/// I2C address with SDO pulled to GND.
pub const SECONDARY_ADDRESS: u8 = 0x76;

const REG_PSR: u8 = 0x00;
const REG_TMP: u8 = 0x03;
const REG_PRS_CFG: u8 = 0x06;
const REG_TMP_CFG: u8 = 0x07;
const REG_MEAS_CFG: u8 = 0x08;
const REG_CFG: u8 = 0x09;
const REG_RESET: u8 = 0x0C;
const REG_ID: u8 = 0x0D;
const REG_COEF: u8 = 0x10;

const CMD_SOFT_RESET: u8 = 0x09;
const MEAS_STANDBY: u8 = 0b000;
const MEAS_PRESSURE: u8 = 0b001;
const MEAS_TEMPERATURE: u8 = 0b010;
const MEAS_PRS_RDY: u8 = 1 << 4;
const MEAS_TMP_RDY: u8 = 1 << 5;
const MEAS_SENSOR_RDY: u8 = 1 << 6;
const MEAS_COEF_RDY: u8 = 1 << 7;
const CFG_P_SHIFT: u8 = 1 << 2;
const CFG_T_SHIFT: u8 = 1 << 3;
/// SPL06-001 `TMP_CFG.TMP_EXT` (external MEMS temperature sensor).
const TMP_CFG_EXT: u8 = 1 << 7;

/// Coefficient bytes of the SPL07-003 (0x10..=0x24); the SPL06-001 has
/// the first 18.
pub const LEN_COEF: usize = 21;

/// Time from power-up or soft reset until the coefficients are readable
/// (`TCoef_rdy`, both datasheets).
pub const RESET_WAIT_US: WaitUs = 40_000;

/// Register access over I2C or SPI.
pub trait Interface {
    /// Transport error.
    type Error;
    /// Burst-read consecutive registers starting at `reg`.
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write one register.
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
}

/// I2C transport.
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    /// The bus.
    pub i2c: I2C,
    /// 7-bit address ([`PRIMARY_ADDRESS`] or [`SECONDARY_ADDRESS`]).
    pub address: u8,
}

/// 4-wire SPI transport (mode 3, up to 10 MHz). Reads send the address
/// with bit 7 set and auto-increment (SPL07-003 datasheet 4.3.2).
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    /// The device (bus + chip select).
    pub spi: SPI,
}

/// Part, from the `ID` register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// SPL06-001 (`ID = 0x10`).
    Spl06,
    /// SPL07-003 (`ID = 0x11`), with the `c31` / `c40` coefficients.
    Spl07,
}

impl Variant {
    /// Decode the `ID` register.
    pub const fn from_id(id: u8) -> Option<Self> {
        match id {
            0x10 => Some(Self::Spl06),
            0x11 => Some(Self::Spl07),
            _ => None,
        }
    }

    /// Number of coefficient bytes.
    pub const fn coef_len(self) -> usize {
        match self {
            Self::Spl06 => 18,
            Self::Spl07 => LEN_COEF,
        }
    }

    /// `TMP_CFG` temperature-sensor selection bit.
    const fn tmp_cfg_ext(self) -> u8 {
        match self {
            Self::Spl06 => TMP_CFG_EXT,
            Self::Spl07 => 0,
        }
    }
}

impl<I2C: embedded_hal::i2c::I2c> Interface for I2cInterface<I2C> {
    type Error = I2C::Error;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[reg], buf)
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[reg, value])
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.spi
            .transaction(&mut [Operation::Write(&[reg | 0x80]), Operation::Read(buf)])
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg & 0x7F, value])
    }
}

/// Oversampling (`PM_PRC` / `TMP_PRC`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(missing_docs)]
pub enum Oversampling {
    X1 = 0,
    X2 = 1,
    X4 = 2,
    X8 = 3,
    X16 = 4,
    X32 = 5,
    X64 = 6,
    X128 = 7,
}

impl Oversampling {
    /// Compensation scale factor `kP` / `kT` (datasheet table 4):
    /// `(2·N − 1)·2^19`, or `·2^13` when the result shift is on (N > 8).
    pub const fn scale_factor(self) -> i32 {
        let n = 1i32 << self as u32;
        if self.needs_shift() {
            (2 * n - 1) << 13
        } else {
            (2 * n - 1) << 19
        }
    }

    /// Conversion time in µs: datasheet table 8 (`2 ms + 1.6 ms · N`)
    /// plus 10 %, rounded up.
    pub const fn conversion_us(self) -> WaitUs {
        let typ: WaitUs = 2_000 + (1_600 << self as u32);
        (typ * 11).div_ceil(10)
    }

    /// `CFG_REG` result shift must be on above 8 times.
    const fn needs_shift(self) -> bool {
        self as u8 > Self::X8 as u8
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Pressure oversampling.
    pub pressure: Oversampling,
    /// Temperature oversampling.
    pub temperature: Oversampling,
}

impl Config {
    /// "Standard" pressure precision (16×, 0.5 Pa RMS) with a single
    /// temperature conversion: 4 ms + 30.4 ms per cycle.
    pub const STANDARD: Self = Self {
        pressure: Oversampling::X16,
        temperature: Oversampling::X1,
    };

    /// `CFG_REG` value (result shifts; interrupts and FIFO off, 4-wire).
    pub const fn cfg_reg(&self) -> u8 {
        let mut v = 0;
        if self.pressure.needs_shift() {
            v |= CFG_P_SHIFT;
        }
        if self.temperature.needs_shift() {
            v |= CFG_T_SHIFT;
        }
        v
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::STANDARD
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

/// Calibration coefficients, sign-extended. `c31` and `c40` are 0 on the
/// SPL06-001.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub struct Calibration {
    pub c0: i16,
    pub c1: i16,
    pub c00: i32,
    pub c10: i32,
    pub c01: i16,
    pub c11: i16,
    pub c20: i16,
    pub c21: i16,
    pub c30: i16,
    pub c31: i16,
    pub c40: i16,
}

const fn sign_extend(v: u32, bits: u32) -> i32 {
    ((v << (32 - bits)) as i32) >> (32 - bits)
}

const Q: u32 = 24;
const ONE: i64 = 1 << Q;

/// `raw / k` in Q24, clamped to ±2.
fn scaled(raw: i32, k: i32) -> i64 {
    ((i64::from(raw) << Q) / i64::from(k)).clamp(-2 * ONE, 2 * ONE)
}

impl Calibration {
    /// Decode registers 0x10..=0x24 (zeros past 0x21 for the SPL06-001).
    pub fn parse(r: &[u8; LEN_COEF]) -> Self {
        let u = |i: usize| u32::from(r[i]);
        let be16 = |i: usize| i16::from_be_bytes([r[i], r[i + 1]]);
        Self {
            c0: sign_extend(u(0) << 4 | u(1) >> 4, 12) as i16,
            c1: sign_extend((u(1) & 0x0F) << 8 | u(2), 12) as i16,
            c00: sign_extend(u(3) << 12 | u(4) << 4 | u(5) >> 4, 20),
            c10: sign_extend((u(5) & 0x0F) << 16 | u(6) << 8 | u(7), 20),
            c01: be16(8),
            c11: be16(10),
            c20: be16(12),
            c21: be16(14),
            c30: be16(16),
            c31: sign_extend(u(18) << 4 | u(19) >> 4, 12) as i16,
            c40: sign_extend((u(19) & 0x0F) << 8 | u(20), 12) as i16,
        }
    }

    /// Temperature in 0.01 °C: `c0 / 2 + c1 · Traw / kT`.
    pub fn compensate_temperature(&self, t_raw: i32, kt: i32) -> i32 {
        let tsc = scaled(t_raw, kt);
        let frac = (100 * i64::from(self.c1) * tsc + (ONE >> 1)) >> Q;
        50 * i32::from(self.c0) + frac as i32
    }

    /// Pressure in Pa: `c00 + c10·P + c20·P² + c30·P³ + c40·P⁴
    /// + T·(c01 + c11·P + c21·P² + c31·P³)` with `P = Praw / kP`,
    /// `T = Traw / kT`.
    pub fn compensate_pressure(&self, p_raw: i32, kp: i32, t_raw: i32, kt: i32) -> u32 {
        let p = scaled(p_raw, kp);
        let t = scaled(t_raw, kt);
        let p2 = (p * p) >> Q;
        let p3 = (p2 * p) >> Q;
        let p4 = (p3 * p) >> Q;
        let sum = (i64::from(self.c00) << Q)
            + i64::from(self.c10) * p
            + i64::from(self.c20) * p2
            + i64::from(self.c30) * p3
            + i64::from(self.c01) * t
            + i64::from(self.c11) * ((t * p) >> Q)
            + i64::from(self.c21) * ((t * p2) >> Q)
            + i64::from(self.c40) * p4
            + i64::from(self.c31) * ((t * p3) >> Q);
        ((sum + (ONE >> 1)) >> Q).clamp(0, i64::from(u32::MAX)) as u32
    }
}

const fn raw24(b: &[u8; 3]) -> i32 {
    sign_extend((b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32, 24)
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

#[derive(Debug)]
struct State {
    config: Config,
    calibration: Option<Calibration>,
    phase: Phase,
    t_raw: i32,
}

impl State {
    const fn new() -> Self {
        Self {
            config: Config::STANDARD,
            calibration: None,
            phase: Phase::Idle,
            t_raw: 0,
        }
    }

    const fn pending_wait_us(&self) -> WaitUs {
        match self.phase {
            Phase::Idle => 0,
            Phase::Temperature => self.config.temperature.conversion_us(),
            Phase::Pressure => self.config.pressure.conversion_us(),
        }
    }

    /// `MEAS_CFG` ready bit and data register of the running phase.
    const fn pending(&self) -> Option<(u8, u8)> {
        match self.phase {
            Phase::Idle => None,
            Phase::Temperature => Some((MEAS_TMP_RDY, REG_TMP)),
            Phase::Pressure => Some((MEAS_PRS_RDY, REG_PSR)),
        }
    }

    /// Consume a raw result: `None` means "start the pressure phase".
    fn on_result<E>(&mut self, raw: &[u8; 3]) -> Result<Option<Measurement>, Error<E>> {
        if self.phase == Phase::Temperature {
            self.t_raw = raw24(raw);
            self.phase = Phase::Pressure;
            return Ok(None);
        }
        self.phase = Phase::Idle;
        let c = self.calibration.as_ref().ok_or(Error::NotReady)?;
        let kt = self.config.temperature.scale_factor();
        Ok(Some(Measurement {
            temperature: c.compensate_temperature(self.t_raw, kt),
            pressure: c.compensate_pressure(
                raw24(raw),
                self.config.pressure.scale_factor(),
                self.t_raw,
                kt,
            ),
        }))
    }
}

fn check_id<E>(id: u8) -> Result<Variant, Error<E>> {
    Variant::from_id(id).ok_or(Error::UnexpectedId(id.into()))
}

const fn coefficients_ready(meas_cfg: u8) -> bool {
    meas_cfg & (MEAS_COEF_RDY | MEAS_SENSOR_RDY) == MEAS_COEF_RDY | MEAS_SENSOR_RDY
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Create a driver over any transport; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    state: State::new(),
                    variant: Variant::Spl06,
                }
            }

            /// Select the configuration written by `init` / `set_config`.
            pub const fn with_config(mut self, config: Config) -> Self {
                self.state.config = config;
                self
            }

            /// Current configuration.
            pub const fn config(&self) -> &Config {
                &self.state.config
            }

            /// Calibration read by `init` (`None` before).
            pub const fn calibration(&self) -> Option<&Calibration> {
                self.state.calibration.as_ref()
            }

            /// Part detected by the last `probe` / `init` (SPL06-001
            /// before).
            pub const fn variant(&self) -> Variant {
                self.variant
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

            /// Give the transport back.
            pub fn release(self) -> IFACE {
                self.iface
            }
        }

        impl<I2C> $name<I2cInterface<I2C>> {
            /// Driver on an I2C bus.
            pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface { i2c, address })
            }
        }

        impl<SPI> $name<SpiInterface<SPI>> {
            /// Driver on an SPI device.
            pub const fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface { spi })
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Spl06<IFACE> {
    iface: IFACE,
    state: State,
    variant: Variant,
}

constructors!(Spl06);

impl<IFACE: Interface> Spl06<IFACE> {
    /// Check the `ID` register.
    pub fn probe(&mut self) -> Result<Variant, Error<IFACE::Error>> {
        let mut id = [0];
        self.read(REG_ID, &mut id)?;
        self.variant = check_id(id[0])?;
        Ok(self.variant)
    }

    /// Soft reset. Wait [`RESET_WAIT_US`], then call [`Self::init`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.write(REG_RESET, CMD_SOFT_RESET)?;
        self.state.calibration = None;
        self.state.phase = Phase::Idle;
        Ok(RESET_WAIT_US)
    }

    /// Check the ID, read the coefficients and write the configuration.
    /// [`Error::NotReady`] while `COEF_RDY` / `SENSOR_RDY` are clear
    /// (up to 40 ms after power-up). Called by `start_measurement`.
    pub fn init(&mut self) -> Result<Calibration, Error<IFACE::Error>> {
        self.probe()?;
        let mut b = [0];
        self.read(REG_MEAS_CFG, &mut b)?;
        if !coefficients_ready(b[0]) {
            return Err(Error::NotReady);
        }
        let mut raw = [0; LEN_COEF];
        let n = self.variant.coef_len();
        self.read(REG_COEF, &mut raw[..n])?;
        self.set_config(self.state.config)?;
        let calibration = Calibration::parse(&raw);
        self.state.calibration = Some(calibration);
        Ok(calibration)
    }

    /// Write a new configuration (device put in standby, cycle aborted).
    pub fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
        self.state.config = config;
        self.state.phase = Phase::Idle;
        self.write(REG_MEAS_CFG, MEAS_STANDBY)?;
        self.write(REG_PRS_CFG, config.pressure as u8)?;
        self.write(
            REG_TMP_CFG,
            self.variant.tmp_cfg_ext() | config.temperature as u8,
        )?;
        self.write(REG_CFG, config.cfg_reg())
    }

    /// Run a complete cycle (temperature then pressure), blocking on
    /// `delay`. For tests and bring-up, not for a Zigbee event loop.
    pub fn measure_full<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Measurement, Error<IFACE::Error>> {
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

    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
        self.iface.read_regs(reg, buf).map_err(Error::bus)
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
        self.iface.write_reg(reg, value).map_err(Error::bus)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Spl06<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    /// Start the temperature conversion (first phase).
    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        if self.state.calibration.is_none() {
            self.init()?;
        }
        self.write(REG_MEAS_CFG, MEAS_TEMPERATURE)?;
        self.state.phase = Phase::Temperature;
        Ok(self.state.pending_wait_us())
    }

    /// After the temperature phase: start the pressure conversion and
    /// return [`Error::NotReady`] (wait [`Spl06::pending_wait_us`]).
    /// After the pressure phase: the measurement.
    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        let (ready, reg) = self.state.pending().ok_or(Error::NotReady)?;
        let mut status = [0];
        self.read(REG_MEAS_CFG, &mut status)?;
        if status[0] & ready == 0 {
            return Err(Error::NotReady);
        }
        let mut raw = [0; 3];
        self.read(reg, &mut raw)?;
        match self.state.on_result(&raw)? {
            Some(m) => Ok(m),
            None => {
                self.write(REG_MEAS_CFG, MEAS_PRESSURE)?;
                Err(Error::NotReady)
            }
        }
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Spl06<IFACE> {
    type BusError = IFACE::Error;

    /// Standby (also aborts a running cycle). Command mode already
    /// returns to standby after each conversion.
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.state.phase = Phase::Idle;
        self.write(REG_MEAS_CFG, MEAS_STANDBY)
    }

    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_SOFT_RESET, Calibration, Config, I2cInterface, LEN_COEF, MEAS_PRESSURE, MEAS_STANDBY,
        MEAS_TEMPERATURE, Measurement, Phase, REG_CFG, REG_COEF, REG_ID, REG_MEAS_CFG, REG_PRS_CFG,
        REG_RESET, REG_TMP_CFG, RESET_WAIT_US, SpiInterface, State, Variant, check_id,
        coefficients_ready,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async register access.
    #[allow(async_fn_in_trait)]
    pub trait Interface {
        /// Transport error.
        type Error;
        /// Burst-read consecutive registers starting at `reg`.
        async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
        /// Write one register.
        async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Interface for I2cInterface<I2C> {
        type Error = I2C::Error;

        async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            self.i2c.write_read(self.address, &[reg], buf).await
        }

        async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
            self.i2c.write(self.address, &[reg, value]).await
        }
    }

    impl<SPI: embedded_hal_async::spi::SpiDevice> Interface for SpiInterface<SPI> {
        type Error = SPI::Error;

        async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            use embedded_hal_async::spi::Operation;
            self.spi
                .transaction(&mut [Operation::Write(&[reg | 0x80]), Operation::Read(buf)])
                .await
        }

        async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
            self.spi.write(&[reg & 0x7F, value]).await
        }
    }

    /// Async twin of [`super::Spl06`].
    #[derive(Debug)]
    pub struct Spl06<IFACE> {
        iface: IFACE,
        state: State,
        variant: Variant,
    }

    constructors!(Spl06);

    impl<IFACE: Interface> Spl06<IFACE> {
        /// See [`super::Spl06::probe`].
        pub async fn probe(&mut self) -> Result<Variant, Error<IFACE::Error>> {
            let mut id = [0];
            self.read(REG_ID, &mut id).await?;
            self.variant = check_id(id[0])?;
            Ok(self.variant)
        }

        /// See [`super::Spl06::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.write(REG_RESET, CMD_SOFT_RESET).await?;
            self.state.calibration = None;
            self.state.phase = Phase::Idle;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Spl06::init`].
        pub async fn init(&mut self) -> Result<Calibration, Error<IFACE::Error>> {
            self.probe().await?;
            let mut b = [0];
            self.read(REG_MEAS_CFG, &mut b).await?;
            if !coefficients_ready(b[0]) {
                return Err(Error::NotReady);
            }
            let mut raw = [0; LEN_COEF];
            let n = self.variant.coef_len();
            self.read(REG_COEF, &mut raw[..n]).await?;
            self.set_config(self.state.config).await?;
            let calibration = Calibration::parse(&raw);
            self.state.calibration = Some(calibration);
            Ok(calibration)
        }

        /// See [`super::Spl06::set_config`].
        pub async fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
            self.state.config = config;
            self.state.phase = Phase::Idle;
            self.write(REG_MEAS_CFG, MEAS_STANDBY).await?;
            self.write(REG_PRS_CFG, config.pressure as u8).await?;
            self.write(
                REG_TMP_CFG,
                self.variant.tmp_cfg_ext() | config.temperature as u8,
            )
            .await?;
            self.write(REG_CFG, config.cfg_reg()).await
        }

        /// See [`super::Spl06::measure_full`].
        pub async fn measure_full<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<Measurement, Error<IFACE::Error>> {
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

        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
            self.iface.read_regs(reg, buf).await.map_err(Error::bus)
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
            self.iface.write_reg(reg, value).await.map_err(Error::bus)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Spl06<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            if self.state.calibration.is_none() {
                self.init().await?;
            }
            self.write(REG_MEAS_CFG, MEAS_TEMPERATURE).await?;
            self.state.phase = Phase::Temperature;
            Ok(self.state.pending_wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            let (ready, reg) = self.state.pending().ok_or(Error::NotReady)?;
            let mut status = [0];
            self.read(REG_MEAS_CFG, &mut status).await?;
            if status[0] & ready == 0 {
                return Err(Error::NotReady);
            }
            let mut raw = [0; 3];
            self.read(reg, &mut raw).await?;
            match self.state.on_result(&raw)? {
                Some(m) => Ok(m),
                None => {
                    self.write(REG_MEAS_CFG, MEAS_PRESSURE).await?;
                    Err(Error::NotReady)
                }
            }
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Spl06<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.state.phase = Phase::Idle;
            self.write(REG_MEAS_CFG, MEAS_STANDBY).await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            Ok(0)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTx};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    /// Plausible SPL06-001 coefficients.
    const CAL06: Calibration = Calibration {
        c0: 204,
        c1: -261,
        c00: 80_469,
        c10: -54_015,
        c01: -2_463,
        c11: 1_225,
        c20: -11_000,
        c21: 158,
        c30: -1_170,
        c31: 0,
        c40: 0,
    };

    /// SPL07-003: the same plus the two 12-bit terms.
    const CAL07: Calibration = Calibration {
        c31: -312,
        c40: 1_105,
        ..CAL06
    };

    fn coef_bytes(c: &Calibration, variant: Variant) -> Vec<u8> {
        let c0 = c.c0 as u16 & 0xFFF;
        let c1 = c.c1 as u16 & 0xFFF;
        let c00 = c.c00 as u32 & 0xF_FFFF;
        let c10 = c.c10 as u32 & 0xF_FFFF;
        let mut v = vec![
            (c0 >> 4) as u8,
            ((c0 & 0xF) << 4 | c1 >> 8) as u8,
            c1 as u8,
            (c00 >> 12) as u8,
            (c00 >> 4) as u8,
            ((c00 & 0xF) << 4 | c10 >> 16) as u8,
            (c10 >> 8) as u8,
            c10 as u8,
        ];
        for x in [c.c01, c.c11, c.c20, c.c21, c.c30] {
            v.extend(x.to_be_bytes());
        }
        if variant == Variant::Spl07 {
            let c31 = c.c31 as u16 & 0xFFF;
            let c40 = c.c40 as u16 & 0xFFF;
            v.extend([
                (c31 >> 4) as u8,
                ((c31 & 0xF) << 4 | c40 >> 8) as u8,
                c40 as u8,
            ]);
        }
        v
    }

    fn raw_bytes(raw: i32) -> Vec<u8> {
        raw.to_be_bytes()[1..].to_vec()
    }

    /// SPL07-003 datasheet 4.6 in f64 (the SPL06-001 formula with
    /// c31 = c40 = 0).
    #[allow(clippy::float_arithmetic)]
    mod reference {
        use super::Calibration;

        pub fn temperature(c: &Calibration, t_raw: i32, kt: i32) -> f64 {
            let t = f64::from(t_raw) / f64::from(kt);
            f64::from(c.c0) * 0.5 + f64::from(c.c1) * t
        }

        pub fn pressure(c: &Calibration, p_raw: i32, kp: i32, t_raw: i32, kt: i32) -> f64 {
            let p = f64::from(p_raw) / f64::from(kp);
            let t = f64::from(t_raw) / f64::from(kt);
            let f = |x: i16| f64::from(x);
            f64::from(c.c00)
                + f64::from(c.c10) * p
                + f(c.c20) * p * p
                + f(c.c30) * p * p * p
                + f(c.c40) * p * p * p * p
                + t * (f(c.c01) + f(c.c11) * p + f(c.c21) * p * p + f(c.c31) * p * p * p)
        }
    }

    #[test]
    fn scale_factors_and_times_match_table() {
        use Oversampling::*;
        let k: Vec<i32> = [X1, X2, X4, X8, X16, X32, X64, X128]
            .iter()
            .map(|o| o.scale_factor())
            .collect();
        assert_eq!(
            k,
            [
                524_288, 1_572_864, 3_670_016, 7_864_320, 253_952, 516_096, 1_040_384, 2_088_960
            ]
        );
        // Table 8: 3.6 / 27.6 / 206.8 ms (+10 %).
        assert_eq!(X1.conversion_us(), 3_960);
        assert_eq!(X16.conversion_us(), 30_360);
        assert_eq!(X128.conversion_us(), 227_480);
        assert_eq!(Config::STANDARD.cfg_reg(), CFG_P_SHIFT);
    }

    #[test]
    fn coefficients_parse() {
        let mut raw = [0; LEN_COEF];
        raw[..18].copy_from_slice(&coef_bytes(&CAL06, Variant::Spl06));
        assert_eq!(Calibration::parse(&raw), CAL06);
        let extreme = Calibration {
            c0: -2_048,
            c1: 2_047,
            c00: -524_288,
            c10: 524_287,
            c31: -2_048,
            c40: 2_047,
            ..CAL07
        };
        for c in [CAL07, extreme] {
            let raw: [u8; LEN_COEF] = coef_bytes(&c, Variant::Spl07).try_into().unwrap();
            assert_eq!(Calibration::parse(&raw), c);
        }
    }

    #[test]
    #[allow(clippy::float_arithmetic)]
    fn matches_float_reference() {
        use Oversampling::*;
        for c in [CAL06, CAL07] {
            for (po, to) in [(X16, X1), (X1, X1), (X8, X128), (X128, X2)] {
                let (kp, kt) = (po.scale_factor(), to.scale_factor());
                for tsc in (-10..=10).map(|i| f64::from(i) / 10.0) {
                    let t_raw = (tsc * f64::from(kt)) as i32;
                    let rt = reference::temperature(&c, t_raw, kt);
                    let t = c.compensate_temperature(t_raw, kt);
                    assert!((f64::from(t) - rt * 100.0).abs() <= 0.51, "T {t} vs {rt}");
                    for psc in (-100..=100).map(|i| f64::from(i) / 100.0) {
                        let p_raw = (psc * f64::from(kp)) as i32;
                        let rp = reference::pressure(&c, p_raw, kp, t_raw, kt);
                        if rp < 0.0 {
                            continue;
                        }
                        let p = c.compensate_pressure(p_raw, kp, t_raw, kt);
                        assert!((f64::from(p) - rp).abs() <= 1.0, "P {p} vs {rp}");
                    }
                }
            }
        }
    }

    #[test]
    fn compensation_is_panic_free() {
        let extreme = Calibration {
            c0: i16::MIN,
            c1: i16::MIN,
            c00: i32::MIN,
            c10: i32::MIN,
            c01: i16::MIN,
            c11: i16::MIN,
            c20: i16::MIN,
            c21: i16::MIN,
            c30: i16::MIN,
            c31: i16::MIN,
            c40: i16::MIN,
        };
        let raws = [-(1 << 23), -1, 0, 1, (1 << 23) - 1];
        for c in [CAL07, extreme, Calibration::default()] {
            for k in [Oversampling::X1, Oversampling::X16].map(Oversampling::scale_factor) {
                for t in raws {
                    let _ = c.compensate_temperature(t, k);
                    for p in raws {
                        let _ = c.compensate_pressure(p, k, t, k);
                    }
                }
            }
        }
    }

    fn i2c_init(a: u8, variant: Variant, cal: &Calibration, config: Config) -> Vec<I2cTx> {
        let (id, ext) = match variant {
            Variant::Spl06 => (0x10, 0x80),
            Variant::Spl07 => (0x11, 0x00),
        };
        vec![
            I2cTx::write_read(a, vec![REG_ID], vec![id]),
            I2cTx::write_read(a, vec![REG_MEAS_CFG], vec![0xC0]),
            I2cTx::write_read(a, vec![REG_COEF], coef_bytes(cal, variant)),
            I2cTx::write(a, vec![REG_MEAS_CFG, 0x00]),
            I2cTx::write(a, vec![REG_PRS_CFG, config.pressure as u8]),
            I2cTx::write(a, vec![REG_TMP_CFG, ext | config.temperature as u8]),
            I2cTx::write(a, vec![REG_CFG, config.cfg_reg()]),
        ]
    }

    #[test]
    fn i2c_two_phase_cycle_spl06() {
        let a = PRIMARY_ADDRESS;
        let k = Config::STANDARD;
        // Tsc = 0.3, Psc = -0.36.
        let t_raw = 157_286;
        let p_raw = -91_423;
        let mut ex = i2c_init(a, Variant::Spl06, &CAL06, k);
        ex.extend([
            I2cTx::write(a, vec![REG_MEAS_CFG, 0x02]),
            I2cTx::write_read(a, vec![REG_MEAS_CFG], vec![0xC2]),
            I2cTx::write_read(a, vec![REG_MEAS_CFG], vec![0xE0]),
            I2cTx::write_read(a, vec![REG_TMP], raw_bytes(t_raw)),
            I2cTx::write(a, vec![REG_MEAS_CFG, 0x01]),
            I2cTx::write_read(a, vec![REG_MEAS_CFG], vec![0xD0]),
            I2cTx::write_read(a, vec![REG_PSR], raw_bytes(p_raw)),
            I2cTx::write(a, vec![REG_MEAS_CFG, 0x00]),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut spl = Spl06::new_i2c(&mut i2c, a);
        assert_eq!(spl.read_measurement(), Err(Error::NotReady));
        assert_eq!(spl.start_measurement().unwrap(), 3_960);
        assert_eq!(spl.variant(), Variant::Spl06);
        assert_eq!(spl.read_measurement(), Err(Error::NotReady));
        assert_eq!(spl.phase(), Phase::Temperature);
        assert_eq!(spl.read_measurement(), Err(Error::NotReady));
        assert_eq!(spl.phase(), Phase::Pressure);
        assert_eq!(spl.pending_wait_us(), 30_360);
        let m = spl.read_measurement().unwrap();
        assert_eq!(spl.phase(), Phase::Idle);
        assert_eq!(m.temperature, CAL06.compensate_temperature(t_raw, 524_288));
        assert_eq!(
            m.pressure,
            CAL06.compensate_pressure(p_raw, 253_952, t_raw, 524_288)
        );
        assert!((2_200..2_400).contains(&m.temperature), "{m:?}");
        assert!((95_000..105_000).contains(&m.pressure), "{m:?}");
        spl.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn coefficients_not_ready_and_wrong_id() {
        let a = SECONDARY_ADDRESS;
        let ex = [
            I2cTx::write_read(a, vec![REG_ID], vec![0x11]),
            I2cTx::write_read(a, vec![REG_MEAS_CFG], vec![0x40]),
            I2cTx::write_read(a, vec![REG_ID], vec![0x58]),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut spl = Spl06::new_i2c(&mut i2c, a);
        assert_eq!(spl.start_measurement(), Err(Error::NotReady));
        assert_eq!(spl.init(), Err(Error::UnexpectedId(0x58)));
        i2c.done();
    }

    #[test]
    fn spi_spl07_init_and_reset() {
        let rd = |reg: u8, v: Vec<u8>| {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![0x80 | reg]),
                SpiTx::read_vec(v),
                SpiTx::transaction_end(),
            ]
        };
        let wr = |reg: u8, v: u8| {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![reg, v]),
                SpiTx::transaction_end(),
            ]
        };
        let config = Config {
            pressure: Oversampling::X64,
            temperature: Oversampling::X32,
        };
        let mut ex = Vec::new();
        ex.extend(rd(REG_ID, vec![0x11]));
        ex.extend(rd(REG_MEAS_CFG, vec![0xC0]));
        ex.extend(rd(REG_COEF, coef_bytes(&CAL07, Variant::Spl07)));
        ex.extend(wr(REG_MEAS_CFG, 0x00));
        ex.extend(wr(REG_PRS_CFG, 0x06));
        ex.extend(wr(REG_TMP_CFG, 0x05));
        ex.extend(wr(REG_CFG, 0x0C));
        ex.extend(wr(REG_RESET, 0x09));
        let mut spi = SpiMock::new(&ex);
        let mut spl = Spl06::new_spi(&mut spi).with_config(config);
        assert_eq!(spl.init().unwrap(), CAL07);
        assert_eq!(spl.variant(), Variant::Spl07);
        assert_eq!(spl.soft_reset().unwrap(), RESET_WAIT_US);
        assert_eq!(spl.calibration(), None);
        spi.done();
    }

    #[test]
    fn async_spl07_measure_full_waits_both_phases() {
        struct RecordingDelay(Vec<u32>);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0.push(ns / 1_000);
            }
        }

        let a = PRIMARY_ADDRESS;
        let mut ex = i2c_init(a, Variant::Spl07, &CAL07, Config::STANDARD);
        ex.extend([
            I2cTx::write(a, vec![REG_MEAS_CFG, 0x02]),
            I2cTx::write_read(a, vec![REG_MEAS_CFG], vec![0xE0]),
            I2cTx::write_read(a, vec![REG_TMP], raw_bytes(150_000)),
            I2cTx::write(a, vec![REG_MEAS_CFG, 0x01]),
            I2cTx::write_read(a, vec![REG_MEAS_CFG], vec![0xD0]),
            I2cTx::write_read(a, vec![REG_PSR], raw_bytes(-90_000)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut spl = asynch::Spl06::new_i2c(&mut i2c, a);
        let mut delay = RecordingDelay(Vec::new());
        let m = block_on(spl.measure_full(&mut delay)).unwrap();
        assert_eq!(
            m.pressure,
            CAL07.compensate_pressure(-90_000, 253_952, 150_000, 524_288)
        );
        assert_eq!(delay.0, [3_960, 30_360]);
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
