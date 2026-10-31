//! Bosch BME280 (pressure, temperature, humidity) and BMP280 (pressure,
//! temperature) over I2C or SPI.
//!
//! The driver uses forced mode: [`start_measurement`] writes `ctrl_meas`
//! with `mode = 01`, the part converts once and drops back to sleep
//! (0.1 µA typ.) on its own. The returned wait is the datasheet maximum
//! measurement time for the configured oversampling.
//!
//! Compensation uses the integer formulas of the datasheets (32-bit for
//! temperature and humidity, 64-bit for pressure): temperature in 0.01 °C,
//! pressure in Pa, humidity in 0.01 %RH.
//!
//! References:
//! * Bosch "BME280 Combined humidity and pressure sensor" datasheet,
//!   BST-BME280-DS002, rev. 1.23 (sections 3.5, 4.2, 5.4, 8.2, 9).
//! * Bosch "BMP280 Digital Pressure Sensor" datasheet, BST-BMP280-DS001,
//!   rev. 1.26 (sections 3.8, 3.11, 3.12).
//!
//! ```ignore
//! let mut bme = Bme280::new_i2c(i2c, PRIMARY_ADDRESS).with_config(Config::default());
//! let chip = bme.init()?; // chip ID, calibration, configuration
//! let wait_us = bme.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = bme.read_measurement()?;
//! ```
//!
//! [`start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with SDO low.
pub const PRIMARY_ADDRESS: u8 = 0x76;
/// I2C address with SDO high.
pub const SECONDARY_ADDRESS: u8 = 0x77;

/// `chip_id` of the BME280.
pub const CHIP_ID_BME280: u8 = 0x60;
/// `chip_id` of the BMP280 (mass production parts).
pub const CHIP_ID_BMP280: u8 = 0x58;

const REG_CALIB_TP: u8 = 0x88;
const REG_CHIP_ID: u8 = 0xD0;
const REG_RESET: u8 = 0xE0;
const REG_CALIB_H: u8 = 0xE1;
const REG_CTRL_HUM: u8 = 0xF2;
const REG_STATUS: u8 = 0xF3;
const REG_CTRL_MEAS: u8 = 0xF4;
const REG_CONFIG: u8 = 0xF5;
const REG_DATA: u8 = 0xF7;

const CMD_SOFT_RESET: u8 = 0xB6;
const STATUS_MEASURING: u8 = 1 << 3;

/// Start-up time after power-on or soft reset (datasheet t_startup 2 ms).
pub const RESET_WAIT_US: WaitUs = 2_000;

const LEN_CALIB_TP: usize = 26;
const LEN_CALIB_H: usize = 7;

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

/// 4-wire SPI transport (mode 0 or 3, up to 10 MHz). Bit 7 of the control
/// byte selects read (1) or write (0).
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

/// Which part answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    /// BME280: pressure, temperature and humidity.
    Bme280,
    /// BMP280: pressure and temperature only.
    Bmp280,
}

impl Chip {
    /// Identify the part from its `chip_id` register.
    pub const fn from_id(id: u8) -> Option<Self> {
        match id {
            CHIP_ID_BME280 => Some(Self::Bme280),
            CHIP_ID_BMP280 => Some(Self::Bmp280),
            _ => None,
        }
    }

    /// The part has a humidity sensor.
    pub const fn has_humidity(self) -> bool {
        matches!(self, Self::Bme280)
    }
}

/// Oversampling of one channel (`osrs_t`, `osrs_p`, `osrs_h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Oversampling {
    /// Channel not measured (output 0x80000 / 0x8000).
    Skip = 0,
    /// ×1.
    X1 = 1,
    /// ×2.
    X2 = 2,
    /// ×4.
    X4 = 3,
    /// ×8.
    X8 = 4,
    /// ×16.
    X16 = 5,
}

impl Oversampling {
    /// Number of conversions averaged (0 when skipped).
    pub const fn factor(self) -> u32 {
        match self {
            Self::Skip => 0,
            _ => 1 << (self as u32 - 1),
        }
    }
}

/// IIR filter coefficient (`config.filter`). Smooths short pressure
/// disturbances; in forced mode it only helps when sampling frequently.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Filter {
    /// Filter off.
    Off = 0,
    /// Coefficient 2.
    C2 = 1,
    /// Coefficient 4.
    C4 = 2,
    /// Coefficient 8.
    C8 = 3,
    /// Coefficient 16.
    C16 = 4,
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Temperature oversampling. Must not be [`Oversampling::Skip`]:
    /// pressure and humidity need `t_fine` (`set_config` returns
    /// `InvalidArgument`).
    pub temperature: Oversampling,
    /// Pressure oversampling.
    pub pressure: Oversampling,
    /// Humidity oversampling (ignored on BMP280).
    pub humidity: Oversampling,
    /// IIR filter.
    pub filter: Filter,
}

impl Config {
    /// Bosch "weather monitoring" setting (datasheet 3.5.1): ×1 on every
    /// channel, filter off. 9.3 ms max on BME280.
    pub const WEATHER: Self = Self {
        temperature: Oversampling::X1,
        pressure: Oversampling::X1,
        humidity: Oversampling::X1,
        filter: Filter::Off,
    };

    /// `ctrl_meas` value (`osrs_t`, `osrs_p`, `mode`).
    pub const fn ctrl_meas(&self, forced: bool) -> u8 {
        ((self.temperature as u8) << 5) | ((self.pressure as u8) << 2) | forced as u8
    }

    /// `config` value (`t_sb` = 0, `filter`, SPI 3-wire off).
    pub const fn config(&self) -> u8 {
        (self.filter as u8) << 2
    }

    /// Maximum measurement time in µs (BME280 datasheet 9.1, BMP280 3.8.1):
    /// `1.25 + 2.3·osrs_t + (2.3·osrs_p + 0.575) + (2.3·osrs_h + 0.575)` ms,
    /// with the pressure/humidity terms dropped when the channel is skipped.
    pub const fn max_conversion_us(&self, chip: Chip) -> WaitUs {
        let mut us = 1_250 + 2_300 * self.temperature.factor();
        if self.pressure.factor() != 0 {
            us += 2_300 * self.pressure.factor() + 575;
        }
        if chip.has_humidity() && self.humidity.factor() != 0 {
            us += 2_300 * self.humidity.factor() + 575;
        }
        us
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::WEATHER
    }
}

/// Factory trimming coefficients (`dig_*`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub struct Calibration {
    pub t1: u16,
    pub t2: i16,
    pub t3: i16,
    pub p1: u16,
    pub p2: i16,
    pub p3: i16,
    pub p4: i16,
    pub p5: i16,
    pub p6: i16,
    pub p7: i16,
    pub p8: i16,
    pub p9: i16,
    pub h1: u8,
    pub h2: i16,
    pub h3: u8,
    pub h4: i16,
    pub h5: i16,
    pub h6: i8,
}

const fn le_u16(b: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([b[i], b[i + 1]])
}

const fn le_i16(b: &[u8], i: usize) -> i16 {
    i16::from_le_bytes([b[i], b[i + 1]])
}

impl Calibration {
    const ZERO: Self = Self {
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
        h1: 0,
        h2: 0,
        h3: 0,
        h4: 0,
        h5: 0,
        h6: 0,
    };

    /// Parse registers 0x88..=0xA1 (temperature, pressure and `dig_H1`)
    /// and, for the BME280, 0xE1..=0xE7 (datasheet table 16).
    pub fn parse(tp: &[u8; LEN_CALIB_TP], h: &[u8; LEN_CALIB_H]) -> Self {
        Self {
            t1: le_u16(tp, 0),
            t2: le_i16(tp, 2),
            t3: le_i16(tp, 4),
            p1: le_u16(tp, 6),
            p2: le_i16(tp, 8),
            p3: le_i16(tp, 10),
            p4: le_i16(tp, 12),
            p5: le_i16(tp, 14),
            p6: le_i16(tp, 16),
            p7: le_i16(tp, 18),
            p8: le_i16(tp, 20),
            p9: le_i16(tp, 22),
            h1: tp[25],
            h2: le_i16(h, 0),
            h3: h[2],
            // 12-bit signed values split across 0xE4..0xE6.
            h4: ((h[3] as i8 as i16) << 4) | (h[4] & 0x0F) as i16,
            h5: ((h[5] as i8 as i16) << 4) | (h[4] >> 4) as i16,
            h6: h[6] as i8,
        }
    }

    /// Temperature compensation (datasheet 4.2.3 / BMP280 3.11.3, 32-bit).
    /// Returns `(t_fine, 0.01 °C)`.
    pub fn compensate_temperature(&self, adc_t: u32) -> (i32, i32) {
        let adc_t = adc_t as i32;
        let t1 = i32::from(self.t1);
        let var1 = (((adc_t >> 3) - (t1 << 1)) * i32::from(self.t2)) >> 11;
        let d = (adc_t >> 4) - t1;
        let var2 = (((d * d) >> 12) * i32::from(self.t3)) >> 14;
        let t_fine = var1 + var2;
        (t_fine, (t_fine * 5 + 128) >> 8)
    }

    /// Pressure compensation in Pa, rounded (BME280 datasheet 4.2.3 /
    /// BMP280 3.11.3, 64-bit variant returning Q24.8 Pa).
    ///
    /// The datasheets also give a 32-bit variant; it deviates from this
    /// one by up to ~6 Pa (3 Pa on the datasheet example), half the
    /// sensor's relative accuracy, so the 64-bit one is used. It costs one
    /// 64-bit division per sample.
    pub fn compensate_pressure(&self, adc_p: u32, t_fine: i32) -> u32 {
        let x = i64::from(t_fine) - 128_000;
        let mut var2 = x * x * i64::from(self.p6);
        var2 += (x * i64::from(self.p5)) << 17;
        var2 += i64::from(self.p4) << 35;
        let mut var1 = ((x * x * i64::from(self.p3)) >> 8) + ((x * i64::from(self.p2)) << 12);
        var1 = (((1i64 << 47) + var1) * i64::from(self.p1)) >> 33;
        if var1 == 0 {
            return 0;
        }
        let p = 1_048_576 - i64::from(adc_p & 0xF_FFFF);
        let p = ((p << 31) - var2).wrapping_mul(3_125) / var1;
        let var1 = i64::from(self.p9).wrapping_mul((p >> 13).wrapping_mul(p >> 13)) >> 25;
        let var2 = i64::from(self.p8).wrapping_mul(p) >> 19;
        let q8 = ((p + var1 + var2) >> 8) + (i64::from(self.p7) << 4);
        ((q8 + 128) >> 8).clamp(0, i64::from(u32::MAX)) as u32
    }

    /// Humidity compensation (BME280 datasheet 4.2.3, 32-bit), in
    /// 0.01 %RH, 0..=10000.
    pub fn compensate_humidity(&self, adc_h: u16, t_fine: i32) -> u16 {
        let x = t_fine - 76_800;
        let a = ((i32::from(adc_h) << 14) - (i32::from(self.h4) << 20) - i32::from(self.h5) * x
            + 16_384)
            >> 15;
        let b = ((((((x * i32::from(self.h6)) >> 10)
            * (((x * i32::from(self.h3)) >> 11) + 32_768))
            >> 10)
            + 2_097_152)
            * i32::from(self.h2)
            + 8_192)
            >> 14;
        let mut v = a.wrapping_mul(b);
        v -= ((((v >> 15) * (v >> 15)) >> 7) * i32::from(self.h1)) >> 4;
        let q10 = (v.clamp(0, 419_430_400) >> 12) as u32; // %RH × 1024
        ((q10 * 100 + 512) / 1_024) as u16
    }
}

/// Raw ADC words of one conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Raw {
    /// 20-bit pressure.
    pub pressure: u32,
    /// 20-bit temperature.
    pub temperature: u32,
    /// 16-bit humidity (0 on BMP280).
    pub humidity: u16,
}

impl Raw {
    /// Parse the data registers 0xF7.. (6 bytes BMP280, 8 bytes BME280).
    pub fn parse(data: &[u8]) -> Self {
        let p20 = |i: usize| {
            (u32::from(data[i]) << 12) | (u32::from(data[i + 1]) << 4) | u32::from(data[i + 2] >> 4)
        };
        Self {
            pressure: p20(0),
            temperature: p20(3),
            humidity: if data.len() >= 8 {
                u16::from_be_bytes([data[6], data[7]])
            } else {
                0
            },
        }
    }
}

/// One compensated sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Pressure in Pa (0 if pressure was skipped).
    pub pressure: u32,
    /// Relative humidity in 0.01 %RH; `None` on BMP280 or when skipped.
    pub humidity: Option<u16>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
        if self.pressure != 0 {
            f(Quantity::Pressure(self.pressure));
        }
        if let Some(h) = self.humidity {
            f(Quantity::Humidity(h));
        }
    }
}

/// Compensate a raw sample for the given part and configuration.
pub fn compensate(raw: &Raw, cal: &Calibration, chip: Chip, config: &Config) -> Measurement {
    let (t_fine, temperature) = cal.compensate_temperature(raw.temperature);
    let pressure = if config.pressure == Oversampling::Skip {
        0
    } else {
        cal.compensate_pressure(raw.pressure, t_fine)
    };
    let humidity = (chip.has_humidity() && config.humidity != Oversampling::Skip)
        .then(|| cal.compensate_humidity(raw.humidity, t_fine));
    Measurement {
        temperature,
        pressure,
        humidity,
    }
}

fn data_len(chip: Chip) -> usize {
    if chip.has_humidity() { 8 } else { 6 }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Create a driver over any transport; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    chip: None,
                    calibration: Calibration::ZERO,
                    config: Config::WEATHER,
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
pub struct Bme280<IFACE> {
    iface: IFACE,
    chip: Option<Chip>,
    calibration: Calibration,
    config: Config,
}

constructors!(Bme280);

impl<IFACE: Interface> Bme280<IFACE> {
    /// Read `chip_id` and identify the part.
    pub fn probe(&mut self) -> Result<Chip, Error<IFACE::Error>> {
        let mut id = [0];
        self.iface
            .read_regs(REG_CHIP_ID, &mut id)
            .map_err(Error::bus)?;
        Chip::from_id(id[0]).ok_or(Error::UnexpectedId(id[0].into()))
    }

    /// Soft reset. Wait [`RESET_WAIT_US`], then call [`Self::init`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.iface
            .write_reg(REG_RESET, CMD_SOFT_RESET)
            .map_err(Error::bus)?;
        self.chip = None;
        Ok(RESET_WAIT_US)
    }

    /// Identify the part, read its calibration and write the configuration
    /// (device left in sleep mode). Called by `start_measurement` if needed.
    pub fn init(&mut self) -> Result<Chip, Error<IFACE::Error>> {
        let chip = self.probe()?;
        let mut tp = [0; LEN_CALIB_TP];
        let mut h = [0; LEN_CALIB_H];
        self.iface
            .read_regs(REG_CALIB_TP, &mut tp)
            .map_err(Error::bus)?;
        if chip.has_humidity() {
            self.iface
                .read_regs(REG_CALIB_H, &mut h)
                .map_err(Error::bus)?;
        }
        self.calibration = Calibration::parse(&tp, &h);
        self.chip = Some(chip);
        self.set_config(self.config)?;
        Ok(chip)
    }

    /// Write a new configuration (device left in sleep mode).
    /// [`Error::InvalidArgument`] if the temperature is skipped.
    pub fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
        if config.temperature == Oversampling::Skip {
            return Err(Error::InvalidArgument);
        }
        self.config = config;
        // Sleep first: `config` writes may be ignored in normal mode.
        self.write(REG_CTRL_MEAS, config.ctrl_meas(false))?;
        self.write(REG_CONFIG, config.config())?;
        // `ctrl_hum` takes effect with the next `ctrl_meas` write.
        if self.chip.is_some_and(Chip::has_humidity) {
            self.write(REG_CTRL_HUM, config.humidity as u8)?;
        }
        Ok(())
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
        self.iface.write_reg(reg, value).map_err(Error::bus)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Bme280<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        let chip = match self.chip {
            Some(chip) => chip,
            None => self.init()?,
        };
        self.write(REG_CTRL_MEAS, self.config.ctrl_meas(true))?;
        Ok(self.config.max_conversion_us(chip))
    }

    /// [`Error::NotReady`] while `status.measuring` is set.
    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        let chip = self.chip.ok_or(Error::NotReady)?;
        let mut status = [0];
        self.iface
            .read_regs(REG_STATUS, &mut status)
            .map_err(Error::bus)?;
        if status[0] & STATUS_MEASURING != 0 {
            return Err(Error::NotReady);
        }
        let mut data = [0; 8];
        let data = &mut data[..data_len(chip)];
        self.iface.read_regs(REG_DATA, data).map_err(Error::bus)?;
        Ok(compensate(
            &Raw::parse(data),
            &self.calibration,
            chip,
            &self.config,
        ))
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Bme280<IFACE> {
    type BusError = IFACE::Error;

    /// Forced mode already returns to sleep; this aborts nothing but makes
    /// sure the part is in sleep mode.
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write(REG_CTRL_MEAS, self.config.ctrl_meas(false))
    }

    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_SOFT_RESET, Calibration, Chip, Config, I2cInterface, LEN_CALIB_H, LEN_CALIB_TP,
        Measurement, REG_CALIB_H, REG_CALIB_TP, REG_CHIP_ID, REG_CONFIG, REG_CTRL_HUM,
        REG_CTRL_MEAS, REG_DATA, REG_RESET, REG_STATUS, RESET_WAIT_US, Raw, STATUS_MEASURING,
        SpiInterface, compensate, data_len,
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

    /// Async twin of [`super::Bme280`].
    #[derive(Debug)]
    pub struct Bme280<IFACE> {
        iface: IFACE,
        chip: Option<Chip>,
        calibration: Calibration,
        config: Config,
    }

    constructors!(Bme280);

    impl<IFACE: Interface> Bme280<IFACE> {
        /// See [`super::Bme280::probe`].
        pub async fn probe(&mut self) -> Result<Chip, Error<IFACE::Error>> {
            let mut id = [0];
            self.iface
                .read_regs(REG_CHIP_ID, &mut id)
                .await
                .map_err(Error::bus)?;
            Chip::from_id(id[0]).ok_or(Error::UnexpectedId(id[0].into()))
        }

        /// See [`super::Bme280::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.write(REG_RESET, CMD_SOFT_RESET).await?;
            self.chip = None;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Bme280::init`].
        pub async fn init(&mut self) -> Result<Chip, Error<IFACE::Error>> {
            let chip = self.probe().await?;
            let mut tp = [0; LEN_CALIB_TP];
            let mut h = [0; LEN_CALIB_H];
            self.iface
                .read_regs(REG_CALIB_TP, &mut tp)
                .await
                .map_err(Error::bus)?;
            if chip.has_humidity() {
                self.iface
                    .read_regs(REG_CALIB_H, &mut h)
                    .await
                    .map_err(Error::bus)?;
            }
            self.calibration = Calibration::parse(&tp, &h);
            self.chip = Some(chip);
            self.set_config(self.config).await?;
            Ok(chip)
        }

        /// See [`super::Bme280::set_config`].
        pub async fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
            if config.temperature == crate::Oversampling::Skip {
                return Err(Error::InvalidArgument);
            }
            self.config = config;
            self.write(REG_CTRL_MEAS, config.ctrl_meas(false)).await?;
            self.write(REG_CONFIG, config.config()).await?;
            if self.chip.is_some_and(Chip::has_humidity) {
                self.write(REG_CTRL_HUM, config.humidity as u8).await?;
            }
            Ok(())
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
            self.iface.write_reg(reg, value).await.map_err(Error::bus)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Bme280<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            let chip = match self.chip {
                Some(chip) => chip,
                None => self.init().await?,
            };
            self.write(REG_CTRL_MEAS, self.config.ctrl_meas(true))
                .await?;
            Ok(self.config.max_conversion_us(chip))
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            let chip = self.chip.ok_or(Error::NotReady)?;
            let mut status = [0];
            self.iface
                .read_regs(REG_STATUS, &mut status)
                .await
                .map_err(Error::bus)?;
            if status[0] & STATUS_MEASURING != 0 {
                return Err(Error::NotReady);
            }
            let mut data = [0; 8];
            let data = &mut data[..data_len(chip)];
            self.iface
                .read_regs(REG_DATA, data)
                .await
                .map_err(Error::bus)?;
            Ok(compensate(
                &Raw::parse(data),
                &self.calibration,
                chip,
                &self.config,
            ))
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Bme280<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write(REG_CTRL_MEAS, self.config.ctrl_meas(false))
                .await
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
    use zigbee_sensor::blocking::Sensor;

    /// Calibration of the BMP280 datasheet calculation example (3.12).
    fn example_tp() -> [u8; LEN_CALIB_TP] {
        let words: [i32; 12] = [
            27504, 26435, -1000, 36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000,
        ];
        let mut tp = [0u8; LEN_CALIB_TP];
        for (i, w) in words.iter().enumerate() {
            tp[2 * i..2 * i + 2].copy_from_slice(&(*w as u16).to_le_bytes());
        }
        tp[25] = 75; // dig_H1
        tp
    }

    /// Typical BME280 humidity trimming: H2=362, H3=0, H4=313, H5=50, H6=30.
    const EXAMPLE_H: [u8; LEN_CALIB_H] = [0x6A, 0x01, 0x00, 0x13, 0x29, 0x03, 0x1E];

    fn example_cal() -> Calibration {
        Calibration::parse(&example_tp(), &EXAMPLE_H)
    }

    /// adc_P = 415148, adc_T = 519888, adc_H = 0x6A00 as register bytes.
    const DATA: [u8; 8] = [0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00, 0x6A, 0x00];

    #[test]
    fn calibration_parsing() {
        let cal = example_cal();
        assert_eq!((cal.t1, cal.t2, cal.t3), (27504, 26435, -1000));
        assert_eq!((cal.p1, cal.p6, cal.p9), (36477, -7, 6000));
        assert_eq!(cal.h1, 75);
        assert_eq!(
            (cal.h2, cal.h3, cal.h4, cal.h5, cal.h6),
            (362, 0, 313, 50, 30)
        );
        // Negative 12-bit H4/H5.
        let cal = Calibration::parse(&example_tp(), &[0, 0, 0, 0xFF, 0xF8, 0xFF, 0]);
        assert_eq!((cal.h4, cal.h5), (-8, -1));
    }

    #[test]
    fn bmp280_datasheet_example() {
        // BMP280 datasheet 3.12: t_fine = 128422, T = 25.08 °C,
        // p = 100653.27 Pa (64-bit 25767236 / 256).
        let cal = example_cal();
        let raw = Raw::parse(&DATA);
        assert_eq!((raw.pressure, raw.temperature), (415_148, 519_888));
        let (t_fine, t) = cal.compensate_temperature(raw.temperature);
        assert_eq!(t_fine, 128_422);
        assert_eq!(t, 2_508);
        assert_eq!(cal.compensate_pressure(raw.pressure, t_fine), 100_653);
    }

    #[test]
    #[allow(clippy::float_arithmetic)]
    fn humidity_matches_double_formula() {
        // BME280 datasheet 8.1 double-precision formula as reference.
        let cal = example_cal();
        let (t_fine, _) = cal.compensate_temperature(519_888);
        for adc_h in [20_000u16, 27_136, 32_000, 40_000] {
            let x = f64::from(t_fine) - 76_800.0;
            let mut h = (f64::from(adc_h)
                - (f64::from(cal.h4) * 64.0 + f64::from(cal.h5) / 16_384.0 * x))
                * (f64::from(cal.h2) / 65_536.0
                    * (1.0
                        + f64::from(cal.h6) / 67_108_864.0
                            * x
                            * (1.0 + f64::from(cal.h3) / 67_108_864.0 * x)));
            h *= 1.0 - f64::from(cal.h1) * h / 524_288.0;
            let expected = (h.clamp(0.0, 100.0) * 100.0) as i32;
            let got = i32::from(cal.compensate_humidity(adc_h, t_fine));
            assert!((got - expected).abs() <= 2, "{adc_h}: {got} vs {expected}");
        }
        assert_eq!(cal.compensate_humidity(0, t_fine), 0);
        assert_eq!(cal.compensate_humidity(u16::MAX, t_fine), 10_000);
    }

    #[test]
    fn compensation_is_panic_free() {
        let cal = example_cal();
        for adc_t in (0..=0xF_FFFF).step_by(40_009) {
            let (t_fine, _) = cal.compensate_temperature(adc_t);
            for adc_p in (0..=0xF_FFFF).step_by(50_021) {
                let _ = cal.compensate_pressure(adc_p, t_fine);
            }
            for adc_h in (0..=0xFFFF).step_by(4_001) {
                let _ = cal.compensate_humidity(adc_h, t_fine);
            }
        }
        assert_eq!(Calibration::default().compensate_pressure(415_148, 0), 0);
    }

    #[test]
    fn conversion_time_formula() {
        let mut c = Config::WEATHER;
        // 1.25 + 2.3 + 2.875 + 2.875 = 9.3 ms (datasheet 9.2 weather monitoring).
        assert_eq!(c.max_conversion_us(Chip::Bme280), 9_300);
        // BMP280 "ultra low power" 6.4 ms (BMP280 table 13).
        assert_eq!(c.max_conversion_us(Chip::Bmp280), 6_425);
        c.temperature = Oversampling::X2;
        c.pressure = Oversampling::X16;
        // BMP280 "ultra high resolution" 43.2 ms.
        assert_eq!(c.max_conversion_us(Chip::Bmp280), 43_225);
        c.humidity = Oversampling::Skip;
        assert_eq!(c.max_conversion_us(Chip::Bme280), 43_225);
        assert_eq!(c.ctrl_meas(true), 0x55); // osrs_t 010, osrs_p 101, mode 01
        c.filter = Filter::C16;
        assert_eq!(c.config(), 0x10);
    }

    fn init_tx(addr: u8, id: u8) -> Vec<I2cTx> {
        let mut tx = vec![
            I2cTx::write_read(addr, vec![REG_CHIP_ID], vec![id]),
            I2cTx::write_read(addr, vec![REG_CALIB_TP], example_tp().to_vec()),
        ];
        if id == CHIP_ID_BME280 {
            tx.push(I2cTx::write_read(
                addr,
                vec![REG_CALIB_H],
                EXAMPLE_H.to_vec(),
            ));
        }
        tx.push(I2cTx::write(addr, vec![REG_CTRL_MEAS, 0x24]));
        tx.push(I2cTx::write(addr, vec![REG_CONFIG, 0x00]));
        if id == CHIP_ID_BME280 {
            tx.push(I2cTx::write(addr, vec![REG_CTRL_HUM, 0x01]));
        }
        tx
    }

    #[test]
    fn bme280_i2c_forced_measurement() {
        let a = PRIMARY_ADDRESS;
        let mut tx = init_tx(a, CHIP_ID_BME280);
        tx.extend([
            I2cTx::write(a, vec![REG_CTRL_MEAS, 0x25]),
            I2cTx::write_read(a, vec![REG_STATUS], vec![0x08]),
            I2cTx::write_read(a, vec![REG_STATUS], vec![0x00]),
            I2cTx::write_read(a, vec![REG_DATA], DATA.to_vec()),
        ]);
        let mut i2c = I2cMock::new(&tx);
        let mut bme = Bme280::new_i2c(&mut i2c, a);
        assert_eq!(bme.start_measurement().unwrap(), 9_300);
        assert_eq!(bme.chip(), Some(Chip::Bme280));
        assert_eq!(bme.read_measurement(), Err(Error::NotReady));
        let m = bme.read_measurement().unwrap();
        assert_eq!(m.temperature, 2_508);
        assert_eq!(m.pressure, 100_653);
        assert!(m.humidity.is_some());
        i2c.done();
    }

    #[test]
    fn bmp280_has_no_humidity() {
        let a = SECONDARY_ADDRESS;
        let mut tx = init_tx(a, CHIP_ID_BMP280);
        tx.extend([
            I2cTx::write(a, vec![REG_CTRL_MEAS, 0x25]),
            I2cTx::write_read(a, vec![REG_STATUS], vec![0x00]),
            I2cTx::write_read(a, vec![REG_DATA], DATA[..6].to_vec()),
        ]);
        let mut i2c = I2cMock::new(&tx);
        let mut bmp = Bme280::new_i2c(&mut i2c, a);
        assert_eq!(bmp.start_measurement().unwrap(), 6_425);
        let m = bmp.read_measurement().unwrap();
        assert_eq!(
            (m.temperature, m.pressure, m.humidity),
            (2_508, 100_653, None)
        );
        i2c.done();
    }

    #[test]
    fn unknown_chip_id() {
        let tx = [I2cTx::write_read(
            PRIMARY_ADDRESS,
            vec![REG_CHIP_ID],
            vec![0x61],
        )];
        let mut i2c = I2cMock::new(&tx);
        let mut bme = Bme280::new_i2c(&mut i2c, PRIMARY_ADDRESS);
        assert_eq!(bme.init(), Err(Error::UnexpectedId(0x61)));
        i2c.done();
    }

    #[test]
    fn skipped_temperature_is_invalid() {
        let mut i2c = I2cMock::new(&[]);
        let mut bme = Bme280::new_i2c(&mut i2c, PRIMARY_ADDRESS);
        let c = Config {
            temperature: Oversampling::Skip,
            ..Config::WEATHER
        };
        assert_eq!(bme.set_config(c), Err(Error::InvalidArgument));
        i2c.done();
    }

    #[test]
    fn spi_register_access_and_reset() {
        let tx = [
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0xD0]),
            SpiTx::read_vec(vec![0x58]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x60, 0xB6]),
            SpiTx::transaction_end(),
        ];
        let mut spi = SpiMock::new(&tx);
        let mut bmp = Bme280::new_spi(&mut spi);
        assert_eq!(bmp.probe(), Ok(Chip::Bmp280));
        assert_eq!(bmp.soft_reset(), Ok(RESET_WAIT_US));
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

        let a = PRIMARY_ADDRESS;
        let mut tx = init_tx(a, CHIP_ID_BME280);
        tx.extend([
            I2cTx::write(a, vec![REG_CTRL_MEAS, 0x25]),
            I2cTx::write_read(a, vec![REG_STATUS], vec![0x00]),
            I2cTx::write_read(a, vec![REG_DATA], DATA.to_vec()),
        ]);
        let mut i2c = I2cMock::new(&tx);
        let mut bme = asynch::Bme280::new_i2c(&mut i2c, a);
        let mut delay = RecordingDelay(0);
        let m = block_on(bme.measure(&mut delay)).unwrap();
        assert_eq!(m.pressure, 100_653);
        assert_eq!(delay.0, 9_300);
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
