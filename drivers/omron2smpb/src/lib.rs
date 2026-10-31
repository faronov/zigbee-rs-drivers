//! Omron 2SMPB-02E barometric pressure and temperature sensor over I2C or
//! SPI, forced mode.
//!
//! The part sleeps between conversions. [`start_measurement`] writes
//! `CTRL_MEAS` with the configured averaging and forced mode; the sensor
//! measures temperature and pressure once and drops back to sleep. The
//! returned wait is the datasheet's typical measurement time plus 25 %
//! (Omron gives only typical values); `read_measurement` also checks
//! `DEVICE_STAT.measure` and returns [`Error::NotReady`] while busy.
//!
//! # Initialisation
//!
//! Call [`init`] once after power-up (10 ms start-up time) or
//! [`soft_reset`] and wait the [`WaitUs`] it returns before starting a
//! measurement: the datasheet asks for at least 1 ms of bus silence after
//! the compensation coefficients are read. `start_measurement` returns
//! [`Error::NotReady`] until `init` has succeeded.
//!
//! # Compensation
//!
//! The datasheet formulas are in floating point:
//!
//! ```text
//! Tr = a0 + a1·Dt + a2·Dt²                      (Tr / 256 = °C)
//! Pr = b00 + bt1·Tr + bp1·Dp + b11·Dp·Tr + bt2·Tr² + bp2·Dp²
//!    + b12·Dp·Tr² + b21·Dp²·Tr + bp3·Dp³       (Pa)
//! K  = A + S·OTP / 32767                        (a1 … bp3)
//! ```
//!
//! This driver evaluates them in `i64` fixed point. Each coefficient `K`
//! is held as `K·2^q` with `q` chosen per coefficient (32 … 80) so that
//! its OTP resolution is kept; `A·2^q` and `S·2^q` are compile-time
//! constants, so only one integer division per coefficient happens, in
//! `init`. `Tr` is kept with 8 extra fractional bits and `Pr` is evaluated
//! as `c0 + Dp·(c1 + Dp·(c2 + Dp·c3))` with `c0 … c3` polynomials in `Tr`.
//! Host tests compare it with the float formula (in `f64`) over random OTP
//! sets: worst pressure difference 0.51 Pa (i.e. the rounding to 1 Pa),
//! temperature within 0.01 °C. Results with `|Tr|` ≥ 128 °C are rejected
//! as [`Error::InvalidData`]; within that bound no intermediate can
//! overflow for any register contents.
//!
//! Reference: Omron "2SMPB-02E Digital Barometric Pressure Sensor"
//! datasheet (English edition 2017-2018; Chinese edition 2021 for the
//! 1 ms note).
//!
//! [`start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`init`]: Omron2smpb::init
//! [`soft_reset`]: Omron2smpb::soft_reset

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with SDO tied to GND.
pub const ADDRESS_SDO_LOW: u8 = 0x70;
/// I2C address with SDO tied to VDDIO.
pub const ADDRESS_SDO_HIGH: u8 = 0x56;

/// `CHIP_ID` value.
pub const CHIP_ID: u8 = 0x5C;

/// Bus silence required after a soft reset (and power-up start-up time).
pub const RESET_WAIT_US: WaitUs = 10_000;
/// Bus silence required after reading the coefficients (returned by
/// `init`).
pub const INIT_WAIT_US: WaitUs = 1_000;

const REG_COE: u8 = 0xA0;
const REG_CHIP_ID: u8 = 0xD1;
const REG_RESET: u8 = 0xE0;
const REG_IIR_CNT: u8 = 0xF1;
const REG_DEVICE_STAT: u8 = 0xF3;
const REG_CTRL_MEAS: u8 = 0xF4;

const RESET_CMD: u8 = 0xE6;
const STAT_MEASURE: u8 = 1 << 3;
const MODE_FORCED: u8 = 0b01;

/// Register access over I2C or SPI.
pub trait Interface {
    /// Transport error.
    type Error;
    /// Burst-read consecutive registers starting at `reg` (I2C address).
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write one register (I2C address).
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
}

/// I2C transport (CSB high; up to 400 kHz, 3.4 MHz HS mode not used).
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    /// The bus.
    pub i2c: I2C,
    /// 7-bit address ([`ADDRESS_SDO_LOW`] or [`ADDRESS_SDO_HIGH`]).
    pub address: u8,
}

/// 4-wire SPI transport (mode 0 or 3, up to 10 MHz). The SPI address is
/// the I2C register address with bit 7 as the read flag.
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    /// The device (bus + chip select).
    pub spi: SPI,
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

/// Averaging count for one channel (`CTRL_MEAS`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Avg {
    /// 1 sample (22-bit result).
    X1 = 1,
    /// 2 samples (23-bit result).
    X2,
    /// 4 samples.
    X4,
    /// 8 samples.
    X8,
    /// 16 samples.
    X16,
    /// 32 samples.
    X32,
    /// 64 samples.
    X64,
}

impl Avg {
    const fn count(self) -> u32 {
        1 << (self as u32 - 1)
    }
}

/// IIR filter coefficient (`IIR_CNT`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Filter {
    /// Off (default).
    #[default]
    Off,
    /// N = 2.
    N2,
    /// N = 4.
    N4,
    /// N = 8.
    N8,
    /// N = 16.
    N16,
    /// N = 32.
    N32,
}

/// Measurement settings. The default is the datasheet "Standard" preset
/// (pressure ×8, temperature ×1: 10.6 ms typ., 2.6 Pa RMS).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Pressure averaging.
    pub pressure: Avg,
    /// Temperature averaging.
    pub temperature: Avg,
    /// IIR filter (written by `init`).
    pub filter: Filter,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            pressure: Avg::X8,
            temperature: Avg::X1,
            filter: Filter::Off,
        }
    }
}

impl Config {
    /// "High speed" preset: pressure ×2, temperature ×1 (5.5 ms typ.).
    pub const HIGH_SPEED: Self = Self::preset(Avg::X2, Avg::X1);
    /// "Low power" preset: pressure ×4, temperature ×1 (7.2 ms typ.).
    pub const LOW_POWER: Self = Self::preset(Avg::X4, Avg::X1);
    /// "High accuracy" preset: pressure ×16, temperature ×2 (18.3 ms typ.).
    pub const HIGH_ACCURACY: Self = Self::preset(Avg::X16, Avg::X2);
    /// "Ultra high accuracy" preset: pressure ×32, temperature ×4
    /// (33.7 ms typ.).
    pub const ULTRA_HIGH_ACCURACY: Self = Self::preset(Avg::X32, Avg::X4);

    const fn preset(pressure: Avg, temperature: Avg) -> Self {
        Self {
            pressure,
            temperature,
            filter: Filter::Off,
        }
    }

    const fn ctrl_meas(self) -> u8 {
        (self.temperature as u8) << 5 | (self.pressure as u8) << 2 | MODE_FORCED
    }

    /// Wait after starting a forced conversion: the datasheet's typical
    /// times fit `2.9 ms + 0.85 ms·Np + 0.9 ms·Nt` exactly for all five
    /// presets; this adds 25 % margin.
    pub const fn conversion_us(self) -> WaitUs {
        let typ = 2_900 + 850 * self.pressure.count() + 900 * self.temperature.count();
        typ * 5 / 4
    }
}

/// One sample.
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

/// `round(m / 10^e · 2^q)`, compile time only.
const fn fx(m: i64, e: u32, q: u32) -> i64 {
    let num = (m as i128) << q;
    let den = 10i128.pow(e);
    let r = if num >= 0 {
        (num + den / 2) / den
    } else {
        (num - den / 2) / den
    };
    r as i64
}

/// `(A·2^q, S·2^q)` of one conversion factor.
type Factor = (i64, i64);

// Datasheet conversion factors; the comment gives (A, S) and q.
const F_A1: Factor = (fx(-63, 4, 32), fx(43, 5, 32)); // -6.3e-3, 4.3e-4, Q32
const F_A2: Factor = (fx(-19, 12, 64), fx(12, 11, 64)); // -1.9e-11, 1.2e-10, Q64
const F_BT1: Factor = (fx(1, 1, 32), fx(91, 3, 32)); // 1.0e-1, 9.1e-2, Q32
const F_BT2: Factor = (fx(12, 9, 48), fx(12, 7, 48)); // 1.2e-8, 1.2e-6, Q48
const F_BP1: Factor = (fx(33, 3, 40), fx(19, 3, 40)); // 3.3e-2, 1.9e-2, Q40
const F_B11: Factor = (fx(21, 8, 56), fx(14, 8, 56)); // 2.1e-7, 1.4e-7, Q56
const F_BP2: Factor = (fx(-63, 11, 56), fx(35, 11, 56)); // -6.3e-10, 3.5e-10, Q56
const F_B12: Factor = (fx(29, 14, 64), fx(76, 14, 64)); // 2.9e-13, 7.6e-13, Q64
const F_B21: Factor = (fx(21, 16, 72), fx(12, 15, 72)); // 2.1e-15, 1.2e-14, Q72
const F_BP3: Factor = (fx(13, 17, 80), fx(79, 18, 80)); // 1.3e-16, 7.9e-17, Q80

/// `(A + S·otp/32767)·2^q`, rounded.
fn coefficient((a, s): Factor, otp: i16) -> i64 {
    let n = s * i64::from(otp);
    let d = if n >= 0 {
        (n + 16_383) / 32_767
    } else {
        (n - 16_383) / 32_767
    };
    a + d
}

/// Rounding arithmetic shift right.
const fn rs(x: i64, n: u32) -> i64 {
    (x + (1 << (n - 1))) >> n
}

/// Compensation coefficients in fixed point (see the crate docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    a0: i64,  // Q8 (Tr units)
    b00: i64, // Q8 (Pa)
    a1: i64,
    a2: i64,
    bt1: i64,
    bt2: i64,
    bp1: i64,
    b11: i64,
    bp2: i64,
    b12: i64,
    b21: i64,
    bp3: i64,
}

impl Calibration {
    /// Number of coefficient registers (`COE_b00_1` 0xA0 … `COE_b00_a0_ex`
    /// 0xB8).
    pub const LEN: usize = 25;

    /// Decode registers 0xA0..=0xB8.
    pub fn from_registers(r: &[u8; Self::LEN]) -> Self {
        let w = |i: usize| i16::from_be_bytes([r[i], r[i + 1]]);
        // 20-bit offsets: [19:12], [11:4] and a nibble of 0xB8; Q4.
        let s20 = |hi: u8, lo: u8, ex: u8| {
            let v = i32::from(hi) << 24 | i32::from(lo) << 16 | i32::from(ex & 0x0F) << 12;
            i64::from(v >> 12)
        };
        Self {
            a0: s20(r[0x12], r[0x13], r[0x18]) << 4,
            b00: s20(r[0x00], r[0x01], r[0x18] >> 4) << 4,
            bt1: coefficient(F_BT1, w(0x02)),
            bt2: coefficient(F_BT2, w(0x04)),
            bp1: coefficient(F_BP1, w(0x06)),
            b11: coefficient(F_B11, w(0x08)),
            bp2: coefficient(F_BP2, w(0x0A)),
            b12: coefficient(F_B12, w(0x0C)),
            b21: coefficient(F_B21, w(0x0E)),
            bp3: coefficient(F_BP3, w(0x10)),
            a1: coefficient(F_A1, w(0x14)),
            a2: coefficient(F_A2, w(0x16)),
        }
    }

    /// Compensate raw `TEMP_TXD` / `PRESS_TXD` values (24-bit, unsigned
    /// as read). [`Error::InvalidData`] if `|Tr|` ≥ 128 °C.
    pub fn compensate<E>(&self, t_raw: u32, p_raw: u32) -> Result<Measurement, Error<E>> {
        let dt = i64::from(t_raw & 0xFF_FFFF) - (1 << 23);
        let dp = i64::from(p_raw & 0xFF_FFFF) - (1 << 23);

        // Tr = a0 + Dt·(a1 + a2·Dt), Q8.
        let s = self.a1 + rs(self.a2 * dt, 32);
        let tr = self.a0 + rs(s * dt, 24);
        if tr.abs() >= 1 << 23 {
            return Err(Error::InvalidData);
        }
        let tr2 = rs(tr * tr, 16); // Tr², Q0

        let c0 = self.b00 + rs(self.bt1 * tr, 32) + rs(self.bt2 * tr2, 40); // Q8
        let c1 = self.bp1 + rs(self.b11 * tr, 24) + rs(self.b12 * tr2, 24); // Q40
        let c2 = self.bp2 + rs(self.b21 * tr, 24); // Q56
        let s2 = c2 + rs(self.bp3 * dp, 24); // Q56
        let s1 = c1 + rs(s2 * dp, 16); // Q40
        let pr = c0 + rs(s1 * dp, 32); // Q8

        Ok(Measurement {
            temperature: ((tr * 100 + (1 << 15)) >> 16) as i32,
            pressure: u32::try_from(rs(pr, 8).max(0)).map_err(|_| Error::InvalidData)?,
        })
    }
}

fn u24(b: &[u8]) -> u32 {
    u32::from_be_bytes([0, b[0], b[1], b[2]])
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Create a driver over any transport; no bus traffic.
            pub const fn new(iface: IFACE, config: Config) -> Self {
                Self {
                    iface,
                    config,
                    calibration: None,
                    pending: false,
                }
            }

            /// Current settings.
            pub const fn config(&self) -> Config {
                self.config
            }

            /// Coefficients read by `init` (`None` before).
            pub const fn calibration(&self) -> Option<&Calibration> {
                self.calibration.as_ref()
            }

            /// Give the transport back.
            pub fn release(self) -> IFACE {
                self.iface
            }
        }

        impl<I2C> $name<I2cInterface<I2C>> {
            /// Driver on an I2C bus.
            pub const fn new_i2c(i2c: I2C, address: u8, config: Config) -> Self {
                Self::new(I2cInterface { i2c, address }, config)
            }
        }

        impl<SPI> $name<SpiInterface<SPI>> {
            /// Driver on an SPI device (CSB low selects SPI after reset).
            pub const fn new_spi(spi: SPI, config: Config) -> Self {
                Self::new(SpiInterface { spi }, config)
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Omron2smpb<IFACE> {
    iface: IFACE,
    config: Config,
    calibration: Option<Calibration>,
    pending: bool,
}

constructors!(Omron2smpb);

impl<IFACE: Interface> Omron2smpb<IFACE> {
    /// Software reset. No bus traffic for [`RESET_WAIT_US`]; run `init`
    /// again afterwards.
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.calibration = None;
        self.pending = false;
        self.write(REG_RESET, RESET_CMD)?;
        Ok(RESET_WAIT_US)
    }

    /// Check `CHIP_ID`, write the IIR filter and read the coefficients.
    /// Wait the returned time before the next access.
    pub fn init(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        let mut id = [0];
        self.read(REG_CHIP_ID, &mut id)?;
        if id[0] != CHIP_ID {
            return Err(Error::UnexpectedId(id[0].into()));
        }
        self.write(REG_IIR_CNT, self.config.filter as u8)?;
        let mut coe = [0; Calibration::LEN];
        self.read(REG_COE, &mut coe)?;
        self.calibration = Some(Calibration::from_registers(&coe));
        Ok(INIT_WAIT_US)
    }

    /// Change the averaging (next start) and IIR filter (written now if
    /// initialised).
    pub fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
        self.config = config;
        if self.calibration.is_some() {
            self.write(REG_IIR_CNT, config.filter as u8)?;
        }
        Ok(())
    }

    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
        self.iface.read_regs(reg, buf).map_err(Error::bus)
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
        self.iface.write_reg(reg, value).map_err(Error::bus)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Omron2smpb<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    /// [`Error::NotReady`] if `init` has not run.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        if self.calibration.is_none() {
            return Err(Error::NotReady);
        }
        self.write(REG_CTRL_MEAS, self.config.ctrl_meas())?;
        self.pending = true;
        Ok(self.config.conversion_us())
    }

    /// [`Error::NotReady`] if nothing was started or the conversion is
    /// still running.
    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        let Some(cal) = self.calibration.filter(|_| self.pending) else {
            return Err(Error::NotReady);
        };
        // DEVICE_STAT, CTRL_MEAS, IO_SETUP, 0xF6, PRESS_TXD2..0, TEMP_TXD2..0.
        let mut buf = [0; 10];
        self.read(REG_DEVICE_STAT, &mut buf)?;
        if buf[0] & STAT_MEASURE != 0 {
            return Err(Error::NotReady);
        }
        self.pending = false;
        cal.compensate(u24(&buf[7..]), u24(&buf[4..7]))
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Omron2smpb<IFACE> {
    type BusError = IFACE::Error;

    /// Forced mode already returns to sleep; this writes sleep mode
    /// (aborting a running conversion).
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.pending = false;
        self.write(REG_CTRL_MEAS, 0)
    }

    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CHIP_ID, Calibration, Config, I2cInterface, INIT_WAIT_US, Measurement, REG_CHIP_ID,
        REG_COE, REG_CTRL_MEAS, REG_DEVICE_STAT, REG_IIR_CNT, REG_RESET, RESET_CMD, RESET_WAIT_US,
        STAT_MEASURE, SpiInterface, u24,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async register access.
    #[allow(async_fn_in_trait)]
    pub trait Interface {
        /// Transport error.
        type Error;
        /// Burst-read consecutive registers starting at `reg` (I2C address).
        async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
        /// Write one register (I2C address).
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

    /// Async twin of [`super::Omron2smpb`].
    #[derive(Debug)]
    pub struct Omron2smpb<IFACE> {
        iface: IFACE,
        config: Config,
        calibration: Option<Calibration>,
        pending: bool,
    }

    constructors!(Omron2smpb);

    impl<IFACE: Interface> Omron2smpb<IFACE> {
        /// See [`super::Omron2smpb::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.calibration = None;
            self.pending = false;
            self.write(REG_RESET, RESET_CMD).await?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Omron2smpb::init`].
        pub async fn init(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            let mut id = [0];
            self.read(REG_CHIP_ID, &mut id).await?;
            if id[0] != CHIP_ID {
                return Err(Error::UnexpectedId(id[0].into()));
            }
            self.write(REG_IIR_CNT, self.config.filter as u8).await?;
            let mut coe = [0; Calibration::LEN];
            self.read(REG_COE, &mut coe).await?;
            self.calibration = Some(Calibration::from_registers(&coe));
            Ok(INIT_WAIT_US)
        }

        /// See [`super::Omron2smpb::set_config`].
        pub async fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
            self.config = config;
            if self.calibration.is_some() {
                self.write(REG_IIR_CNT, config.filter as u8).await?;
            }
            Ok(())
        }

        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
            self.iface.read_regs(reg, buf).await.map_err(Error::bus)
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
            self.iface.write_reg(reg, value).await.map_err(Error::bus)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Omron2smpb<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            if self.calibration.is_none() {
                return Err(Error::NotReady);
            }
            self.write(REG_CTRL_MEAS, self.config.ctrl_meas()).await?;
            self.pending = true;
            Ok(self.config.conversion_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            let Some(cal) = self.calibration.filter(|_| self.pending) else {
                return Err(Error::NotReady);
            };
            let mut buf = [0; 10];
            self.read(REG_DEVICE_STAT, &mut buf).await?;
            if buf[0] & STAT_MEASURE != 0 {
                return Err(Error::NotReady);
            }
            self.pending = false;
            cal.compensate(u24(&buf[7..]), u24(&buf[4..7]))
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Omron2smpb<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.pending = false;
            self.write(REG_CTRL_MEAS, 0).await
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

    type E = core::convert::Infallible;

    /// Raw OTP words: a1, a2, bt1, bt2, bp1, b11, bp2, b12, b21, bp3 and
    /// the 20-bit offsets a0, b00.
    #[derive(Clone, Copy)]
    struct Otp {
        k: [i16; 10],
        a0: i32,
        b00: i32,
    }

    impl Otp {
        fn registers(&self) -> [u8; Calibration::LEN] {
            let [a1, a2, bt1, bt2, bp1, b11, bp2, b12, b21, bp3] = self.k;
            let mut r = [0; Calibration::LEN];
            for (i, v) in [
                (0x02, bt1),
                (0x04, bt2),
                (0x06, bp1),
                (0x08, b11),
                (0x0A, bp2),
                (0x0C, b12),
                (0x0E, b21),
                (0x10, bp3),
                (0x14, a1),
                (0x16, a2),
            ] {
                r[i..i + 2].copy_from_slice(&v.to_be_bytes());
            }
            let (a0, b00) = (self.a0 as u32, self.b00 as u32);
            r[0x00] = (b00 >> 12) as u8;
            r[0x01] = (b00 >> 4) as u8;
            r[0x12] = (a0 >> 12) as u8;
            r[0x13] = (a0 >> 4) as u8;
            r[0x18] = ((b00 & 0xF) << 4 | a0 & 0xF) as u8;
            r
        }

        fn calibration(&self) -> Calibration {
            Calibration::from_registers(&self.registers())
        }
    }

    /// Datasheet float formulas, in f64. Returns (Tr / 256 in °C, Pa).
    #[allow(clippy::float_arithmetic)]
    fn reference(o: &Otp, t_raw: u32, p_raw: u32) -> (f64, f64) {
        let factors = [
            (-6.3e-3, 4.3e-4),
            (-1.9e-11, 1.2e-10),
            (1.0e-1, 9.1e-2),
            (1.2e-8, 1.2e-6),
            (3.3e-2, 1.9e-2),
            (2.1e-7, 1.4e-7),
            (-6.3e-10, 3.5e-10),
            (2.9e-13, 7.6e-13),
            (2.1e-15, 1.2e-14),
            (1.3e-16, 7.9e-17),
        ];
        let mut k = [0.0; 10];
        for (i, (a, s)) in factors.iter().enumerate() {
            k[i] = a + s * f64::from(o.k[i]) / 32_767.0;
        }
        let [a1, a2, bt1, bt2, bp1, b11, bp2, b12, b21, bp3] = k;
        let dt = f64::from(t_raw) - 8_388_608.0;
        let dp = f64::from(p_raw) - 8_388_608.0;
        let tr = f64::from(o.a0) / 16.0 + a1 * dt + a2 * dt * dt;
        let pr = f64::from(o.b00) / 16.0
            + bt1 * tr
            + bp1 * dp
            + b11 * dp * tr
            + bt2 * tr * tr
            + bp2 * dp * dp
            + b12 * dp * tr * tr
            + b21 * dp * dp * tr
            + bp3 * dp * dp * dp;
        (tr / 256.0, pr)
    }

    struct Rng(u32);
    impl Rng {
        fn next(&mut self) -> u32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            self.0
        }
        fn otp(&mut self) -> Otp {
            let s20 = |v: u32| (v << 12) as i32 >> 12;
            Otp {
                k: core::array::from_fn(|_| self.next() as i16),
                a0: s20(self.next()),
                b00: s20(self.next()),
            }
        }
    }

    #[test]
    #[allow(clippy::float_arithmetic)]
    fn compensation_matches_float_reference() {
        let mut rng = Rng(0x1234_5678);
        let (mut worst_p, mut worst_t, mut checked) = (0.0f64, 0.0f64, 0);
        while checked < 20_000 {
            let o = rng.otp();
            let (t, p) = (rng.next() & 0xFF_FFFF, rng.next() & 0xFF_FFFF);
            let (rt, rp) = reference(&o, t, p);
            if !(-40.0..=85.0).contains(&rt) || !(30_000.0..=120_000.0).contains(&rp) {
                continue;
            }
            let m = o.calibration().compensate::<E>(t, p).unwrap();
            worst_p = worst_p.max((f64::from(m.pressure) - rp).abs());
            worst_t = worst_t.max((f64::from(m.temperature) - rt * 100.0).abs());
            checked += 1;
        }
        assert!(worst_p < 0.52, "pressure {worst_p}");
        assert!(worst_t < 0.51, "temperature {worst_t}");
    }

    #[test]
    fn extremes_do_not_overflow() {
        let edge = [i16::MIN, -1, 0, 1, i16::MAX];
        let raw = [0, 1 << 23, 0xFF_FFFF];
        let off = [-(1 << 19), 0, (1 << 19) - 1];
        let mut rng = Rng(42);
        for i in 0..20_000u32 {
            let mut o = rng.otp();
            for k in &mut o.k {
                if rng.next() % 2 == 0 {
                    *k = edge[rng.next() as usize % edge.len()];
                }
            }
            o.a0 = off[i as usize % 3];
            o.b00 = off[(i / 3) as usize % 3];
            let t = raw[rng.next() as usize % 3];
            let p = raw[rng.next() as usize % 3];
            let _ = o.calibration().compensate::<E>(t, p);
            let _ = o.calibration().compensate::<E>(rng.next(), rng.next());
        }
    }

    #[test]
    fn temperature_example_and_range() {
        // Tr = 6400 LSB → 25.00 °C (datasheet example): Dt = 0, a0 = 6400.
        let o = Otp {
            k: [0; 10],
            a0: 6_400 * 16,
            b00: 0,
        };
        let m = o.calibration().compensate::<E>(1 << 23, 1 << 23).unwrap();
        assert_eq!(m.temperature, 2_500);
        let hot = Otp {
            a0: 128 * 256 * 16,
            ..o
        };
        assert_eq!(
            hot.calibration().compensate::<E>(1 << 23, 1 << 23),
            Err(Error::InvalidData)
        );
    }

    #[test]
    fn conversion_times() {
        let typ = |c: Config| c.conversion_us() * 4 / 5;
        assert_eq!(typ(Config::HIGH_SPEED), 5_500);
        assert_eq!(typ(Config::LOW_POWER), 7_200);
        assert_eq!(typ(Config::default()), 10_600);
        assert_eq!(typ(Config::HIGH_ACCURACY), 18_300);
        assert_eq!(typ(Config::ULTRA_HIGH_ACCURACY), 33_700);
        assert_eq!(Config::default().ctrl_meas(), 0x31);
        assert_eq!(Config::ULTRA_HIGH_ACCURACY.ctrl_meas(), 0x79);
    }

    const OTP: Otp = Otp {
        k: [
            -1_200, 3_000, -15_000, 2_500, 10_000, -700, 8_000, -3_000, 1_500, -2_000,
        ],
        a0: -20_000,
        b00: 300_000,
    };

    fn data(stat: u8, t: u32, p: u32) -> Vec<u8> {
        let (t, p) = (t.to_be_bytes(), p.to_be_bytes());
        vec![stat, 0, 0, 0, p[1], p[2], p[3], t[1], t[2], t[3]]
    }

    fn i2c_init(a: u8, filter: u8) -> Vec<I2cTx> {
        vec![
            I2cTx::write_read(a, vec![REG_CHIP_ID], vec![CHIP_ID]),
            I2cTx::write(a, vec![REG_IIR_CNT, filter]),
            I2cTx::write_read(a, vec![REG_COE], OTP.registers().to_vec()),
        ]
    }

    /// Raw values giving about 25 °C and 100 kPa with [`OTP`].
    fn sample() -> (u32, u32) {
        let mut t = 1 << 23;
        let mut p = 1 << 23;
        // Coarse search is enough; the test compares against the reference.
        for step in [1 << 20, 1 << 16, 1 << 12, 1 << 8] {
            while reference(&OTP, t, p).0 < 25.0 {
                t -= step;
            }
            while reference(&OTP, t, p).1 < 100_000.0 {
                p += step;
            }
            t += step;
            p -= step;
        }
        (t, p)
    }

    #[test]
    #[allow(clippy::float_arithmetic)]
    fn i2c_cycle() {
        let a = ADDRESS_SDO_LOW;
        let (t, p) = sample();
        let mut ex = i2c_init(a, 3);
        ex.extend([
            I2cTx::write(a, vec![REG_CTRL_MEAS, 0x79]),
            I2cTx::write_read(a, vec![REG_DEVICE_STAT], data(0x08, 0, 0)),
            I2cTx::write_read(a, vec![REG_DEVICE_STAT], data(0x00, t, p)),
            I2cTx::write(a, vec![REG_CTRL_MEAS, 0]),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let config = Config {
            filter: Filter::N8,
            ..Config::ULTRA_HIGH_ACCURACY
        };
        let mut s = Omron2smpb::new_i2c(&mut i2c, a, config);
        assert_eq!(s.start_measurement(), Err(Error::NotReady));
        assert_eq!(s.init().unwrap(), INIT_WAIT_US);
        assert_eq!(s.calibration(), Some(&OTP.calibration()));
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        assert_eq!(s.start_measurement().unwrap(), 42_125);
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        let m = s.read_measurement().unwrap();
        let (rt, rp) = reference(&OTP, t, p);
        assert!((f64::from(m.pressure) - rp).abs() < 0.52);
        assert!((f64::from(m.temperature) - rt * 100.0).abs() < 0.51);
        assert!((99_000..=101_000).contains(&m.pressure), "{m:?}");
        assert!((2_400..=2_600).contains(&m.temperature), "{m:?}");
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        s.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn wrong_id_and_reset() {
        let a = ADDRESS_SDO_HIGH;
        let ex = [
            I2cTx::write_read(a, vec![REG_CHIP_ID], vec![0x58]),
            I2cTx::write(a, vec![REG_RESET, 0xE6]),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut s = Omron2smpb::new_i2c(&mut i2c, a, Config::default());
        assert_eq!(s.init(), Err(Error::UnexpectedId(0x58)));
        assert_eq!(s.soft_reset().unwrap(), RESET_WAIT_US);
        i2c.done();
    }

    #[test]
    fn spi_cycle() {
        let rd = |reg: u8, v: Vec<u8>| {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![reg]),
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
        let (t, p) = sample();
        let mut ex = Vec::new();
        ex.extend(rd(0xD1, vec![CHIP_ID]));
        ex.extend(wr(0x71, 0));
        ex.extend(rd(0xA0, OTP.registers().to_vec()));
        ex.extend(wr(0x74, 0x31));
        ex.extend(rd(0xF3, data(0, t, p)));
        let mut spi = SpiMock::new(&ex);
        let mut s = Omron2smpb::new_spi(&mut spi, Config::default());
        s.init().unwrap();
        assert_eq!(s.start_measurement().unwrap(), 13_250);
        let m = s.read_measurement().unwrap();
        assert_eq!(m, OTP.calibration().compensate::<E>(t, p).unwrap());
        spi.done();
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

        let a = ADDRESS_SDO_LOW;
        let (t, p) = sample();
        let mut ex = i2c_init(a, 0);
        ex.extend([
            I2cTx::write(a, vec![REG_CTRL_MEAS, 0x2D]),
            I2cTx::write_read(a, vec![REG_DEVICE_STAT], data(0, t, p)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut s = asynch::Omron2smpb::new_i2c(&mut i2c, a, Config::LOW_POWER);
        assert_eq!(block_on(s.init()).unwrap(), INIT_WAIT_US);
        let mut delay = RecordingDelay(0);
        let m = block_on(s.measure(&mut delay)).unwrap();
        assert_eq!(m, OTP.calibration().compensate::<E>(t, p).unwrap());
        assert_eq!(delay.0, 9_000);
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
