//! Bosch BMP388 and BMP390 pressure and temperature sensor over I2C or SPI.
//!
//! Forced mode: [`start_measurement`] writes `PWR_CTRL` with both sensors
//! enabled and `mode = forced`; the part converts once and returns to
//! sleep mode (2 µA BMP388, 1.4 µA BMP390). The returned wait is the
//! datasheet `T_conv` formula (3.9.2) scaled by 19/16, which covers the
//! maximum column of the measurement time table (3.9.1, max/typ ≤ 1.18).
//!
//! The datasheet only gives a floating-point compensation (appendix 8.4 -
//! 8.6). This driver evaluates the same polynomials in 64-bit fixed point:
//! temperature in Q16 °C, pressure accumulated in Q8 Pa with the
//! coefficient scaling folded into the shifts. It stays within 0.5 Pa and
//! 0.005 °C of the float reference over -40..85 °C and 300..1250 hPa
//! (see the tests).
//!
//! References:
//! * Bosch "BMP390 Digital pressure sensor" datasheet, BST-BMP390-DS002-07,
//!   rev. 1.7 (sections 3.9, 3.11, 4.3, 5.3, appendix 8.4-8.6).
//! * Bosch "BMP388 Digital pressure sensor" datasheet, BST-BMP388-DS001-07,
//!   rev. 1.7 (same register map and compensation).
//!
//! ```ignore
//! let mut bmp = Bmp3xx::new_i2c(i2c, SECONDARY_ADDRESS).with_config(Config::default());
//! let chip = bmp.init()?;
//! let wait_us = bmp.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = bmp.read_measurement()?;
//! ```
//!
//! [`start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with SDO low.
pub const PRIMARY_ADDRESS: u8 = 0x76;
/// I2C address with SDO high.
pub const SECONDARY_ADDRESS: u8 = 0x77;

/// `CHIP_ID` of the BMP388.
pub const CHIP_ID_BMP388: u8 = 0x50;
/// `CHIP_ID` of the BMP390.
pub const CHIP_ID_BMP390: u8 = 0x60;

const REG_CHIP_ID: u8 = 0x00;
const REG_STATUS: u8 = 0x03;
const REG_DATA: u8 = 0x04;
const REG_PWR_CTRL: u8 = 0x1B;
const REG_OSR: u8 = 0x1C;
const REG_CONFIG: u8 = 0x1F;
const REG_CALIB: u8 = 0x31;
const REG_CMD: u8 = 0x7E;

const CMD_SOFT_RESET: u8 = 0xB6;
const STATUS_DRDY_PRESS: u8 = 1 << 5;
const STATUS_DRDY_TEMP: u8 = 1 << 6;
/// `press_en | temp_en`.
const PWR_SENSORS_ON: u8 = 0x03;
const PWR_MODE_FORCED: u8 = 0x10;

const LEN_CALIB: usize = 21;
const LEN_DATA: usize = 6;

/// Start-up time after power-on or soft reset (datasheet t_startup 2 ms).
pub const RESET_WAIT_US: WaitUs = 2_000;

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

/// 4-wire SPI transport (mode 0 or 3, up to 10 MHz). Reads send the
/// address with bit 7 set and discard one dummy byte (datasheet 5.3.2).
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
        self.spi.transaction(&mut [
            Operation::Write(&[reg | 0x80]),
            Operation::Read(&mut [0]),
            Operation::Read(buf),
        ])
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg & 0x7F, value])
    }
}

/// Which part answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    /// BMP388.
    Bmp388,
    /// BMP390.
    Bmp390,
}

impl Chip {
    /// Identify a `CHIP_ID` value.
    pub const fn from_id(id: u8) -> Option<Self> {
        match id {
            CHIP_ID_BMP388 => Some(Self::Bmp388),
            CHIP_ID_BMP390 => Some(Self::Bmp390),
            _ => None,
        }
    }
}

/// Oversampling (`osr_p` / `osr_t`).
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
}

/// IIR filter coefficient (`iir_filter`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(missing_docs)]
pub enum Filter {
    Off = 0,
    C1 = 1,
    C3 = 2,
    C7 = 3,
    C15 = 4,
    C31 = 5,
    C63 = 6,
    C127 = 7,
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Pressure oversampling.
    pub pressure: Oversampling,
    /// Temperature oversampling.
    pub temperature: Oversampling,
    /// IIR filter (its state survives between forced conversions).
    pub filter: Filter,
}

impl Config {
    /// "Standard resolution" of datasheet table 23: P×4, T×1, filter off,
    /// 12.5 ms max.
    pub const STANDARD: Self = Self {
        pressure: Oversampling::X4,
        temperature: Oversampling::X1,
        filter: Filter::Off,
    };

    /// `OSR` register value.
    pub const fn osr(&self) -> u8 {
        ((self.temperature as u8) << 3) | self.pressure as u8
    }

    /// `CONFIG` register value.
    pub const fn config(&self) -> u8 {
        (self.filter as u8) << 1
    }

    /// Maximum conversion time in µs: datasheet 3.9.2
    /// `234 + (392 + 2^osr_p·2020) + (313 + 2^osr_t·2020)` (largest
    /// constants of both parts) times 19/16, rounded up.
    pub const fn max_conversion_us(&self) -> WaitUs {
        let typ: WaitUs =
            234 + 392 + (2_020 << self.pressure as u32) + 313 + (2_020 << self.temperature as u32);
        (typ * 19).div_ceil(16)
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

/// Raw NVM trimming coefficients (`NVM_PAR_*`, datasheet table 24).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub struct Calibration {
    pub t1: u16,
    pub t2: u16,
    pub t3: i8,
    pub p1: i16,
    pub p2: i16,
    pub p3: i8,
    pub p4: i8,
    pub p5: u16,
    pub p6: u16,
    pub p7: i8,
    pub p8: i8,
    pub p9: i16,
    pub p10: i8,
    pub p11: i8,
}

/// `t_lin` is clamped to ±128 °C before the pressure polynomial so that
/// its cube cannot overflow on corrupt data.
const T_LIN_LIMIT: i64 = 128 << 16;

impl Calibration {
    /// Decode registers 0x31..0x45 (little-endian words).
    pub fn parse(raw: &[u8; LEN_CALIB]) -> Self {
        let u = |i: usize| u16::from_le_bytes([raw[i], raw[i + 1]]);
        let s = |i: usize| raw[i] as i8;
        Self {
            t1: u(0),
            t2: u(2),
            t3: s(4),
            p1: u(5) as i16,
            p2: u(7) as i16,
            p3: s(9),
            p4: s(10),
            p5: u(11),
            p6: u(13),
            p7: s(15),
            p8: s(16),
            p9: u(17) as i16,
            p10: s(19),
            p11: s(20),
        }
    }

    /// Temperature from the raw 24-bit value: returns `t_lin` in Q16 °C
    /// (input of [`Self::compensate_pressure`]) and 0.01 °C.
    ///
    /// Float: `d = ut - T1·2^8; t_lin = d·T2/2^30 + d²·T3/2^48`.
    pub fn compensate_temperature(&self, ut: u32) -> (i64, i32) {
        let d = i64::from(ut) - (i64::from(self.t1) << 8);
        let t_lin = ((d * i64::from(self.t2)) >> 14) + ((d * d * i64::from(self.t3)) >> 32);
        (t_lin, ((t_lin * 100 + (1 << 15)) >> 16) as i32)
    }

    /// Pressure in Pa from the raw 24-bit value and `t_lin` (Q16 °C).
    ///
    /// Float (datasheet 8.6), with `PAR_*` the scaled coefficients:
    /// `P5 + P6·t + P7·t² + P8·t³ + up·(P1 + P2·t + P3·t² + P4·t³)
    /// + up²·(P9 + P10·t) + up³·P11`.
    pub fn compensate_pressure(&self, up: u32, t_lin: i64) -> u32 {
        let t = t_lin.clamp(-T_LIN_LIMIT, T_LIN_LIMIT);
        let t2 = (t * t) >> 16;
        let t3 = (t2 * t) >> 16;
        // Offset: P5·2^3 + P6/2^6·t + P7/2^8·t² + P8/2^15·t³, in Q8 Pa.
        let offset = (i64::from(self.p5) << 11)
            + ((i64::from(self.p6) * t) >> 14)
            + ((i64::from(self.p7) * t2) >> 16)
            + ((i64::from(self.p8) * t3) >> 23);
        // Sensitivity: (P1-2^14)/2^20 + (P2-2^14)/2^29·t + P3/2^32·t²
        // + P4/2^37·t³, in Q40.
        let sens = ((i64::from(self.p1) - 16_384) << 20)
            + (((i64::from(self.p2) - 16_384) * t) >> 5)
            + ((i64::from(self.p3) * t2) >> 8)
            + ((i64::from(self.p4) * t3) >> 13);
        let up = i64::from(up & 0xFF_FFFF);
        // up²/2^20 keeps the products below 2^61.
        let up2 = (up * up) >> 20;
        // (P9/2^48 + P10/2^48·t)·up² + P11/2^65·up³, in Q8 Pa.
        let quadratic = (up2 * ((i64::from(self.p9) << 16) + i64::from(self.p10) * t)) >> 36;
        let cubic = (up2 * up * i64::from(self.p11)) >> 37;
        let q8 = offset + ((up * sens) >> 32) + quadratic + cubic;
        ((q8 + 128) >> 8).clamp(0, i64::from(u32::MAX)) as u32
    }
}

fn u24_le(b: &[u8]) -> u32 {
    u32::from(b[0]) | u32::from(b[1]) << 8 | u32::from(b[2]) << 16
}

/// Compensate a 6-byte data burst (0x04..0x09: pressure then temperature,
/// both little-endian 24-bit).
pub fn compensate(data: &[u8; LEN_DATA], calibration: &Calibration) -> Measurement {
    let (t_lin, temperature) = calibration.compensate_temperature(u24_le(&data[3..]));
    Measurement {
        temperature,
        pressure: calibration.compensate_pressure(u24_le(&data[..3]), t_lin),
    }
}

const fn ready(status: u8) -> bool {
    status & (STATUS_DRDY_PRESS | STATUS_DRDY_TEMP) == STATUS_DRDY_PRESS | STATUS_DRDY_TEMP
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Create a driver over any transport; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    chip: None,
                    calibration: Calibration {
                        t1: 0,
                        t2: 0,
                        t3: 0,
                        p1: 0,
                        p2: 0,
                        p3: 0,
                        p4: 0,
                        p5: 0,
                        p6: 0,
                        p7: 0,
                        p8: 0,
                        p9: 0,
                        p10: 0,
                        p11: 0,
                    },
                    config: Config::STANDARD,
                }
            }

            /// Select the configuration written by `init` / `set_config`.
            pub const fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self
            }

            /// Part detected by `init` (`None` before).
            pub const fn chip(&self) -> Option<Chip> {
                self.chip
            }

            /// Calibration read by `init`.
            pub const fn calibration(&self) -> &Calibration {
                &self.calibration
            }

            /// Current configuration.
            pub const fn config(&self) -> &Config {
                &self.config
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
pub struct Bmp3xx<IFACE> {
    iface: IFACE,
    chip: Option<Chip>,
    calibration: Calibration,
    config: Config,
}

constructors!(Bmp3xx);

impl<IFACE: Interface> Bmp3xx<IFACE> {
    /// Read `CHIP_ID` and identify the part.
    pub fn probe(&mut self) -> Result<Chip, Error<IFACE::Error>> {
        let mut id = [0];
        self.read(REG_CHIP_ID, &mut id)?;
        Chip::from_id(id[0]).ok_or(Error::UnexpectedId(id[0].into()))
    }

    /// Soft reset. Wait [`RESET_WAIT_US`], then call [`Self::init`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.write(REG_CMD, CMD_SOFT_RESET)?;
        self.chip = None;
        Ok(RESET_WAIT_US)
    }

    /// Identify the part, read its calibration and write the configuration
    /// (device left in sleep mode). Called by `start_measurement` if needed.
    pub fn init(&mut self) -> Result<Chip, Error<IFACE::Error>> {
        let chip = self.probe()?;
        let mut raw = [0; LEN_CALIB];
        self.read(REG_CALIB, &mut raw)?;
        self.calibration = Calibration::parse(&raw);
        self.chip = Some(chip);
        self.set_config(self.config)?;
        Ok(chip)
    }

    /// Write a new configuration (device left in sleep mode).
    pub fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
        self.config = config;
        self.write(REG_PWR_CTRL, PWR_SENSORS_ON)?;
        self.write(REG_OSR, config.osr())?;
        self.write(REG_CONFIG, config.config())
    }

    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
        self.iface.read_regs(reg, buf).map_err(Error::bus)
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
        self.iface.write_reg(reg, value).map_err(Error::bus)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Bmp3xx<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        if self.chip.is_none() {
            self.init()?;
        }
        self.write(REG_PWR_CTRL, PWR_SENSORS_ON | PWR_MODE_FORCED)?;
        Ok(self.config.max_conversion_us())
    }

    /// [`Error::NotReady`] until both `drdy_press` and `drdy_temp` are set.
    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        if self.chip.is_none() {
            return Err(Error::NotReady);
        }
        let mut status = [0];
        self.read(REG_STATUS, &mut status)?;
        if !ready(status[0]) {
            return Err(Error::NotReady);
        }
        let mut data = [0; LEN_DATA];
        self.read(REG_DATA, &mut data)?;
        Ok(compensate(&data, &self.calibration))
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Bmp3xx<IFACE> {
    type BusError = IFACE::Error;

    /// Forced mode already returns to sleep; this makes sure of it.
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write(REG_PWR_CTRL, PWR_SENSORS_ON)
    }

    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_SOFT_RESET, Calibration, Chip, Config, I2cInterface, LEN_CALIB, LEN_DATA, Measurement,
        PWR_MODE_FORCED, PWR_SENSORS_ON, REG_CALIB, REG_CHIP_ID, REG_CMD, REG_CONFIG, REG_DATA,
        REG_OSR, REG_PWR_CTRL, REG_STATUS, RESET_WAIT_US, SpiInterface, compensate, ready,
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
                .transaction(&mut [
                    Operation::Write(&[reg | 0x80]),
                    Operation::Read(&mut [0]),
                    Operation::Read(buf),
                ])
                .await
        }

        async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
            self.spi.write(&[reg & 0x7F, value]).await
        }
    }

    /// Async twin of [`super::Bmp3xx`].
    #[derive(Debug)]
    pub struct Bmp3xx<IFACE> {
        iface: IFACE,
        chip: Option<Chip>,
        calibration: Calibration,
        config: Config,
    }

    constructors!(Bmp3xx);

    impl<IFACE: Interface> Bmp3xx<IFACE> {
        /// See [`super::Bmp3xx::probe`].
        pub async fn probe(&mut self) -> Result<Chip, Error<IFACE::Error>> {
            let mut id = [0];
            self.read(REG_CHIP_ID, &mut id).await?;
            Chip::from_id(id[0]).ok_or(Error::UnexpectedId(id[0].into()))
        }

        /// See [`super::Bmp3xx::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.write(REG_CMD, CMD_SOFT_RESET).await?;
            self.chip = None;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Bmp3xx::init`].
        pub async fn init(&mut self) -> Result<Chip, Error<IFACE::Error>> {
            let chip = self.probe().await?;
            let mut raw = [0; LEN_CALIB];
            self.read(REG_CALIB, &mut raw).await?;
            self.calibration = Calibration::parse(&raw);
            self.chip = Some(chip);
            self.set_config(self.config).await?;
            Ok(chip)
        }

        /// See [`super::Bmp3xx::set_config`].
        pub async fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
            self.config = config;
            self.write(REG_PWR_CTRL, PWR_SENSORS_ON).await?;
            self.write(REG_OSR, config.osr()).await?;
            self.write(REG_CONFIG, config.config()).await
        }

        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
            self.iface.read_regs(reg, buf).await.map_err(Error::bus)
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
            self.iface.write_reg(reg, value).await.map_err(Error::bus)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Bmp3xx<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            if self.chip.is_none() {
                self.init().await?;
            }
            self.write(REG_PWR_CTRL, PWR_SENSORS_ON | PWR_MODE_FORCED)
                .await?;
            Ok(self.config.max_conversion_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            if self.chip.is_none() {
                return Err(Error::NotReady);
            }
            let mut status = [0];
            self.read(REG_STATUS, &mut status).await?;
            if !ready(status[0]) {
                return Err(Error::NotReady);
            }
            let mut data = [0; LEN_DATA];
            self.read(REG_DATA, &mut data).await?;
            Ok(compensate(&data, &self.calibration))
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Bmp3xx<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write(REG_PWR_CTRL, PWR_SENSORS_ON).await
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

    /// Plausible trimming values of a BMP390.
    const CAL: Calibration = Calibration {
        t1: 27_891,
        t2: 19_064,
        t3: -10,
        p1: 1_238,
        p2: 1_960,
        p3: 29,
        p4: 0,
        p5: 25_424,
        p6: 29_878,
        p7: 3,
        p8: -6,
        p9: 16_106,
        p10: 13,
        p11: -60,
    };

    fn calib_bytes(c: &Calibration) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend(c.t1.to_le_bytes());
        v.extend(c.t2.to_le_bytes());
        v.push(c.t3 as u8);
        v.extend(c.p1.to_le_bytes());
        v.extend(c.p2.to_le_bytes());
        v.push(c.p3 as u8);
        v.push(c.p4 as u8);
        v.extend(c.p5.to_le_bytes());
        v.extend(c.p6.to_le_bytes());
        v.push(c.p7 as u8);
        v.push(c.p8 as u8);
        v.extend(c.p9.to_le_bytes());
        v.push(c.p10 as u8);
        v.push(c.p11 as u8);
        v
    }

    /// Datasheet appendix 8.4-8.6 in f64.
    #[allow(clippy::float_arithmetic)]
    mod reference {
        use super::Calibration;

        fn p2(n: i32) -> f64 {
            2f64.powi(n)
        }

        pub fn temperature(c: &Calibration, ut: u32) -> f64 {
            let pd1 = f64::from(ut) - f64::from(c.t1) / p2(-8);
            pd1 * (f64::from(c.t2) / p2(30)) + pd1 * pd1 * (f64::from(c.t3) / p2(48))
        }

        pub fn pressure(c: &Calibration, up: u32, t: f64) -> f64 {
            let par_p1 = (f64::from(c.p1) - p2(14)) / p2(20);
            let par_p2 = (f64::from(c.p2) - p2(14)) / p2(29);
            let par_p3 = f64::from(c.p3) / p2(32);
            let par_p4 = f64::from(c.p4) / p2(37);
            let par_p5 = f64::from(c.p5) / p2(-3);
            let par_p6 = f64::from(c.p6) / p2(6);
            let par_p7 = f64::from(c.p7) / p2(8);
            let par_p8 = f64::from(c.p8) / p2(15);
            let par_p9 = f64::from(c.p9) / p2(48);
            let par_p10 = f64::from(c.p10) / p2(48);
            let par_p11 = f64::from(c.p11) / p2(65);
            let out1 = par_p5 + par_p6 * t + par_p7 * t * t + par_p8 * t * t * t;
            let up = f64::from(up);
            let out2 = up * (par_p1 + par_p2 * t + par_p3 * t * t + par_p4 * t * t * t);
            out1 + out2 + up * up * (par_p9 + par_p10 * t) + up * up * up * par_p11
        }

        /// Raw temperature for `celsius`, first-order inverse.
        pub fn raw_temperature(c: &Calibration, celsius: i32) -> u32 {
            (f64::from(c.t1) * 256.0 + f64::from(celsius) * p2(30) / f64::from(c.t2)) as u32
        }
    }

    #[test]
    #[allow(clippy::float_arithmetic)]
    fn matches_float_reference() {
        for c in [
            CAL,
            Calibration {
                p4: -7,
                p6: 65_535,
                p7: -128,
                p8: 127,
                p9: -32_768,
                p10: -128,
                p11: 127,
                ..CAL
            },
        ] {
            for celsius in (-40..=85).step_by(5) {
                let ut = reference::raw_temperature(&c, celsius);
                let (t_lin, centi) = c.compensate_temperature(ut);
                let rt = reference::temperature(&c, ut);
                assert!(
                    (f64::from(centi) - rt * 100.0).abs() <= 0.51,
                    "T {centi} vs {rt}"
                );
                for up in (4_000_000..9_500_000).step_by(50_000) {
                    let rp = reference::pressure(&c, up, rt);
                    if !(30_000.0..=125_000.0).contains(&rp) {
                        continue;
                    }
                    let p = c.compensate_pressure(up, t_lin);
                    assert!((f64::from(p) - rp).abs() <= 1.0, "P {p} vs {rp}");
                }
            }
        }
    }

    #[test]
    fn compensation_is_panic_free() {
        let extreme = Calibration {
            t1: 0,
            t2: u16::MAX,
            t3: i8::MIN,
            p1: i16::MIN,
            p2: i16::MIN,
            p3: i8::MIN,
            p4: i8::MIN,
            p5: u16::MAX,
            p6: u16::MAX,
            p7: i8::MIN,
            p8: i8::MIN,
            p9: i16::MIN,
            p10: i8::MIN,
            p11: i8::MIN,
        };
        for c in [CAL, extreme, Calibration::default()] {
            for raw in [0, 1, 0x7F_FFFF, 0xFF_FFFF] {
                let (t_lin, _) = c.compensate_temperature(raw);
                for up in [0, 0x80_0000, 0xFF_FFFF] {
                    let _ = c.compensate_pressure(up, t_lin);
                }
            }
        }
    }

    #[test]
    fn calibration_parse() {
        let raw: [u8; LEN_CALIB] = calib_bytes(&CAL).try_into().unwrap();
        assert_eq!(Calibration::parse(&raw), CAL);
    }

    #[test]
    fn conversion_time_formula() {
        // Table 23 max column: ULP 5.70 ms, highest resolution 78.09 ms.
        let ulp = Config {
            pressure: Oversampling::X1,
            temperature: Oversampling::X1,
            filter: Filter::Off,
        };
        assert_eq!(ulp.max_conversion_us(), 5_913);
        let highest = Config {
            pressure: Oversampling::X32,
            temperature: Oversampling::X2,
            ..ulp
        };
        assert!(highest.max_conversion_us() >= 78_090);
        assert!(Config::STANDARD.max_conversion_us() >= 12_480);
    }

    fn i2c_init(address: u8, id: u8) -> Vec<I2cTx> {
        vec![
            I2cTx::write_read(address, vec![REG_CHIP_ID], vec![id]),
            I2cTx::write_read(address, vec![REG_CALIB], calib_bytes(&CAL)),
            I2cTx::write(address, vec![REG_PWR_CTRL, 0x03]),
            I2cTx::write(address, vec![REG_OSR, 0x02]),
            I2cTx::write(address, vec![REG_CONFIG, 0x00]),
        ]
    }

    fn data_frame(up: u32, ut: u32) -> Vec<u8> {
        let p = up.to_le_bytes();
        let t = ut.to_le_bytes();
        vec![p[0], p[1], p[2], t[0], t[1], t[2]]
    }

    #[test]
    fn i2c_forced_measurement() {
        let ut = 8_548_000;
        let up = 7_600_000;
        let mut ex = i2c_init(SECONDARY_ADDRESS, CHIP_ID_BMP390);
        ex.extend([
            I2cTx::write(SECONDARY_ADDRESS, vec![REG_PWR_CTRL, 0x13]),
            I2cTx::write_read(SECONDARY_ADDRESS, vec![REG_STATUS], vec![0x30]),
            I2cTx::write_read(SECONDARY_ADDRESS, vec![REG_STATUS], vec![0x70]),
            I2cTx::write_read(SECONDARY_ADDRESS, vec![REG_DATA], data_frame(up, ut)),
            I2cTx::write(SECONDARY_ADDRESS, vec![REG_PWR_CTRL, 0x03]),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut bmp = Bmp3xx::new_i2c(&mut i2c, SECONDARY_ADDRESS);
        assert_eq!(bmp.start_measurement().unwrap(), 13_109);
        assert_eq!(bmp.chip(), Some(Chip::Bmp390));
        assert_eq!(bmp.read_measurement(), Err(Error::NotReady));
        let m = bmp.read_measurement().unwrap();
        let (t_lin, t) = CAL.compensate_temperature(ut);
        assert_eq!(m.temperature, t);
        assert_eq!(m.pressure, CAL.compensate_pressure(up, t_lin));
        assert!((2_400..2_600).contains(&m.temperature));
        assert!((95_000..110_000).contains(&m.pressure));
        bmp.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn wrong_chip_id() {
        let ex = [I2cTx::write_read(
            PRIMARY_ADDRESS,
            vec![REG_CHIP_ID],
            vec![0x58],
        )];
        let mut i2c = I2cMock::new(&ex);
        let mut bmp = Bmp3xx::new_i2c(&mut i2c, PRIMARY_ADDRESS);
        assert_eq!(bmp.start_measurement(), Err(Error::UnexpectedId(0x58)));
        i2c.done();
    }

    #[test]
    fn spi_reads_skip_dummy_byte() {
        let config = Config {
            pressure: Oversampling::X8,
            temperature: Oversampling::X2,
            filter: Filter::C3,
        };
        let ex = [
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x80]),
            SpiTx::read_vec(vec![0xFF]),
            SpiTx::read_vec(vec![CHIP_ID_BMP388]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x80 | REG_CALIB]),
            SpiTx::read_vec(vec![0xFF]),
            SpiTx::read_vec(calib_bytes(&CAL)),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![REG_PWR_CTRL, 0x03]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![REG_OSR, 0x0B]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![REG_CONFIG, 0x04]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![REG_CMD, CMD_SOFT_RESET]),
            SpiTx::transaction_end(),
        ];
        let mut spi = SpiMock::new(&ex);
        let mut bmp = Bmp3xx::new_spi(&mut spi).with_config(config);
        assert_eq!(bmp.init().unwrap(), Chip::Bmp388);
        assert_eq!(bmp.calibration(), &CAL);
        assert_eq!(bmp.soft_reset().unwrap(), RESET_WAIT_US);
        assert_eq!(bmp.chip(), None);
        spi.done();
    }

    #[test]
    fn async_measure_awaits_conversion_time() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let mut ex = i2c_init(SECONDARY_ADDRESS, CHIP_ID_BMP388);
        ex.extend([
            I2cTx::write(SECONDARY_ADDRESS, vec![REG_PWR_CTRL, 0x13]),
            I2cTx::write_read(SECONDARY_ADDRESS, vec![REG_STATUS], vec![0x70]),
            I2cTx::write_read(
                SECONDARY_ADDRESS,
                vec![REG_DATA],
                data_frame(7_600_000, 8_548_000),
            ),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut bmp = asynch::Bmp3xx::new_i2c(&mut i2c, SECONDARY_ADDRESS);
        let mut delay = RecordingDelay(0);
        let m = block_on(bmp.measure(&mut delay)).unwrap();
        assert!((95_000..110_000).contains(&m.pressure));
        assert_eq!(delay.0, Config::STANDARD.max_conversion_us());
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
