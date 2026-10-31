//! Bosch BMP580 / BMP581 / BMP585 pressure and temperature sensor over I2C
//! or SPI.
//!
//! The part compensates internally: the data registers hold temperature in
//! 1/2^16 °C and pressure in 1/2^6 Pa, so no calibration is read.
//!
//! Forced mode: [`start_measurement`] writes `ODR_CONFIG.pwr_mode = forced`
//! with `deep_dis = 0` and `odr = 1 Hz`; the part converts once and falls
//! back to STANDBY and then, because the IIR filter stays bypassed and the
//! FIFO disabled, to DEEP STANDBY (0.55 µA). The datasheet specifies the
//! direct DEEP STANDBY → measurement transition (`tstartup_deep` 4 ms), so
//! no intermediate STANDBY step (and no driver-side delay) is needed. The
//! returned wait is `tstartup_deep + 1.05 · (tconv_t + tconv_p)` (typical
//! conversion times ±5 %, table 2).
//!
//! The IIR filter is not exposed: its registers are only writable in
//! STANDBY after a `tstandby` wait, it would block DEEP STANDBY, and it has
//! little use on one-shot samples taken minutes apart.
//!
//! References:
//! * Bosch "BMP581 Barometric pressure sensor" datasheet,
//!   BST-BMP581-DS004-13, rev. 1.13 (tables 2/9, sections 4.3, 4.6, 4.7,
//!   5.1, 5.5, chapter 7).
//! * Bosch "BMP585 Barometric pressure sensor" datasheet,
//!   BST-BMP585-DS003-02, rev. 1.2 (same register map, `CHIP_ID` 0x51).
//!
//! ```ignore
//! let mut bmp = Bmp5xx::new_i2c(i2c, PRIMARY_ADDRESS).with_config(Config::default());
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
pub const PRIMARY_ADDRESS: u8 = 0x46;
/// I2C address with SDO high.
pub const SECONDARY_ADDRESS: u8 = 0x47;

/// `CHIP_ID` of the BMP580 and BMP581.
pub const CHIP_ID_BMP581: u8 = 0x50;
/// `CHIP_ID` of the BMP585.
pub const CHIP_ID_BMP585: u8 = 0x51;

const REG_CHIP_ID: u8 = 0x01;
const REG_INT_SOURCE: u8 = 0x15;
const REG_TEMP_DATA: u8 = 0x1D;
const REG_INT_STATUS: u8 = 0x27;
const REG_STATUS: u8 = 0x28;
const REG_OSR_CONFIG: u8 = 0x36;
const REG_ODR_CONFIG: u8 = 0x37;
const REG_CMD: u8 = 0x7E;

const CMD_SOFT_RESET: u8 = 0xB6;
const INT_DRDY: u8 = 1 << 0;
const STATUS_NVM_RDY: u8 = 1 << 1;
const STATUS_NVM_ERR: u8 = 1 << 2;
const OSR_PRESS_EN: u8 = 1 << 6;
/// `deep_dis = 0`, `odr = 0x1C` (1 Hz, below the 5 Hz deep-standby limit).
const ODR_DEEP_STANDBY: u8 = 0x1C << 2;
const PWR_MODE_FORCED: u8 = 0b10;

const LEN_DATA: usize = 6;

/// Power-up / soft-reset time (datasheet t_powup, tsoft_res: 2 ms).
pub const RESET_WAIT_US: WaitUs = 2_000;
/// Mode change from DEEP STANDBY to start of measurement (tstartup_deep).
const STARTUP_US: u32 = 4_000;

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

/// 4-wire SPI transport (mode 0 or 3, up to 12 MHz). Reads send the
/// address with bit 7 set; there is no dummy byte (datasheet 5.5.3).
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
    /// BMP580 or BMP581 (same `CHIP_ID`).
    Bmp581,
    /// BMP585.
    Bmp585,
}

impl Chip {
    /// Identify a `CHIP_ID` value.
    pub const fn from_id(id: u8) -> Option<Self> {
        match id {
            CHIP_ID_BMP581 => Some(Self::Bmp581),
            CHIP_ID_BMP585 => Some(Self::Bmp585),
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
    X64 = 6,
    X128 = 7,
}

/// Typical `tconv_p` per OSR in µs (datasheet table 2).
const TCONV_P_US: [u32; 8] = [1_000, 1_700, 2_900, 5_400, 10_400, 20_400, 40_400, 80_400];
/// Typical `tconv_t` per OSR in µs (datasheet table 2).
const TCONV_T_US: [u32; 8] = [1_000, 1_100, 1_500, 2_100, 3_300, 5_800, 10_800, 20_800];

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Pressure oversampling.
    pub pressure: Oversampling,
    /// Temperature oversampling.
    pub temperature: Oversampling,
}

impl Config {
    /// "Standard resolution" of datasheet table 9: P×4, T×1.
    pub const STANDARD: Self = Self {
        pressure: Oversampling::X4,
        temperature: Oversampling::X1,
    };

    /// `OSR_CONFIG` register value (pressure enabled).
    pub const fn osr_config(&self) -> u8 {
        OSR_PRESS_EN | ((self.pressure as u8) << 3) | self.temperature as u8
    }

    /// Maximum time from the forced-mode write to data ready, in µs:
    /// `tstartup_deep + 1.05 · (tconv_p + tconv_t)`, rounded up.
    pub const fn max_conversion_us(&self) -> WaitUs {
        let conv = TCONV_P_US[self.pressure as usize] + TCONV_T_US[self.temperature as usize];
        STARTUP_US + (conv * 21).div_ceil(20)
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::STANDARD
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

/// Convert the 6-byte burst from `TEMP_DATA_XLSB` (0x1D..0x22: temperature
/// then pressure, both little-endian 24-bit) to 0.01 °C and Pa, rounded.
pub fn convert(data: &[u8; LEN_DATA]) -> Measurement {
    // Sign-extend the 24-bit temperature (1/2^16 °C).
    let t = i32::from_le_bytes([0, data[0], data[1], data[2]]) >> 8;
    let p = u32::from_le_bytes([data[3], data[4], data[5], 0]);
    Measurement {
        temperature: (t * 100 + (1 << 15)) >> 16,
        pressure: (p + (1 << 5)) >> 6,
    }
}

const fn nvm_ok(status: u8) -> bool {
    status & (STATUS_NVM_RDY | STATUS_NVM_ERR) == STATUS_NVM_RDY
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Create a driver over any transport; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    chip: None,
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
pub struct Bmp5xx<IFACE> {
    iface: IFACE,
    chip: Option<Chip>,
    config: Config,
}

constructors!(Bmp5xx);

impl<IFACE: Interface> Bmp5xx<IFACE> {
    /// Read `CHIP_ID` and identify the part.
    pub fn probe(&mut self) -> Result<Chip, Error<IFACE::Error>> {
        let mut id = [0];
        self.read(REG_CHIP_ID, &mut id)?;
        Chip::from_id(id[0]).ok_or(Error::UnexpectedId(id[0].into()))
    }

    /// Soft reset. Wait [`RESET_WAIT_US`], then call [`Self::init`].
    ///
    /// The part does not ACK the reset command over I2C, so the bus result
    /// of this write is ignored; `init` reports a dead bus.
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        let _ = self.iface.write_reg(REG_CMD, CMD_SOFT_RESET);
        self.chip = None;
        Ok(RESET_WAIT_US)
    }

    /// Identify the part, check the NVM boot status, enable the data-ready
    /// status flag and write the configuration (device left in deep
    /// standby). Called by `start_measurement` if needed.
    ///
    /// The first access is a discarded `CHIP_ID` read: after power-up or
    /// reset it switches the interface to SPI (datasheet 5.1) and is
    /// harmless over I2C. Returns [`Error::InvalidData`] if `STATUS`
    /// reports `nvm_rdy = 0` or `nvm_err = 1`.
    pub fn init(&mut self) -> Result<Chip, Error<IFACE::Error>> {
        let mut buf = [0];
        self.read(REG_CHIP_ID, &mut buf)?;
        let chip = self.probe()?;
        self.read(REG_STATUS, &mut buf)?;
        if !nvm_ok(buf[0]) {
            return Err(Error::InvalidData);
        }
        // Clear-on-read: drops the power-on-reset flag.
        self.read(REG_INT_STATUS, &mut buf)?;
        self.write(REG_INT_SOURCE, INT_DRDY)?;
        self.chip = Some(chip);
        self.set_config(self.config)?;
        Ok(chip)
    }

    /// Write a new configuration and request (deep) standby.
    pub fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
        self.config = config;
        self.write(REG_ODR_CONFIG, ODR_DEEP_STANDBY)?;
        self.write(REG_OSR_CONFIG, config.osr_config())
    }

    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
        self.iface.read_regs(reg, buf).map_err(Error::bus)
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
        self.iface.write_reg(reg, value).map_err(Error::bus)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Bmp5xx<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        if self.chip.is_none() {
            self.init()?;
        }
        self.write(REG_ODR_CONFIG, ODR_DEEP_STANDBY | PWR_MODE_FORCED)?;
        Ok(self.config.max_conversion_us())
    }

    /// [`Error::NotReady`] until `INT_STATUS.drdy_data_reg` is set (the
    /// flag clears on read, so each conversion is reported once).
    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        if self.chip.is_none() {
            return Err(Error::NotReady);
        }
        let mut status = [0];
        self.read(REG_INT_STATUS, &mut status)?;
        if status[0] & INT_DRDY == 0 {
            return Err(Error::NotReady);
        }
        let mut data = [0; LEN_DATA];
        self.read(REG_TEMP_DATA, &mut data)?;
        Ok(convert(&data))
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Bmp5xx<IFACE> {
    type BusError = IFACE::Error;

    /// Forced mode already returns to deep standby; this makes sure of it.
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write(REG_ODR_CONFIG, ODR_DEEP_STANDBY)
    }

    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_SOFT_RESET, Chip, Config, I2cInterface, INT_DRDY, LEN_DATA, Measurement,
        ODR_DEEP_STANDBY, PWR_MODE_FORCED, REG_CHIP_ID, REG_CMD, REG_INT_SOURCE, REG_INT_STATUS,
        REG_ODR_CONFIG, REG_OSR_CONFIG, REG_STATUS, REG_TEMP_DATA, RESET_WAIT_US, SpiInterface,
        convert, nvm_ok,
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

    /// Async twin of [`super::Bmp5xx`].
    #[derive(Debug)]
    pub struct Bmp5xx<IFACE> {
        iface: IFACE,
        chip: Option<Chip>,
        config: Config,
    }

    constructors!(Bmp5xx);

    impl<IFACE: Interface> Bmp5xx<IFACE> {
        /// See [`super::Bmp5xx::probe`].
        pub async fn probe(&mut self) -> Result<Chip, Error<IFACE::Error>> {
            let mut id = [0];
            self.read(REG_CHIP_ID, &mut id).await?;
            Chip::from_id(id[0]).ok_or(Error::UnexpectedId(id[0].into()))
        }

        /// See [`super::Bmp5xx::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            let _ = self.iface.write_reg(REG_CMD, CMD_SOFT_RESET).await;
            self.chip = None;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Bmp5xx::init`].
        pub async fn init(&mut self) -> Result<Chip, Error<IFACE::Error>> {
            let mut buf = [0];
            self.read(REG_CHIP_ID, &mut buf).await?;
            let chip = self.probe().await?;
            self.read(REG_STATUS, &mut buf).await?;
            if !nvm_ok(buf[0]) {
                return Err(Error::InvalidData);
            }
            self.read(REG_INT_STATUS, &mut buf).await?;
            self.write(REG_INT_SOURCE, INT_DRDY).await?;
            self.chip = Some(chip);
            self.set_config(self.config).await?;
            Ok(chip)
        }

        /// See [`super::Bmp5xx::set_config`].
        pub async fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
            self.config = config;
            self.write(REG_ODR_CONFIG, ODR_DEEP_STANDBY).await?;
            self.write(REG_OSR_CONFIG, config.osr_config()).await
        }

        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
            self.iface.read_regs(reg, buf).await.map_err(Error::bus)
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
            self.iface.write_reg(reg, value).await.map_err(Error::bus)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Bmp5xx<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            if self.chip.is_none() {
                self.init().await?;
            }
            self.write(REG_ODR_CONFIG, ODR_DEEP_STANDBY | PWR_MODE_FORCED)
                .await?;
            Ok(self.config.max_conversion_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            if self.chip.is_none() {
                return Err(Error::NotReady);
            }
            let mut status = [0];
            self.read(REG_INT_STATUS, &mut status).await?;
            if status[0] & INT_DRDY == 0 {
                return Err(Error::NotReady);
            }
            let mut data = [0; LEN_DATA];
            self.read(REG_TEMP_DATA, &mut data).await?;
            Ok(convert(&data))
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Bmp5xx<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write(REG_ODR_CONFIG, ODR_DEEP_STANDBY).await
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

    /// 25.00 °C (25 · 2^16) and 101 325 Pa (· 2^6), little-endian.
    fn frame(t_raw: i32, p_raw: u32) -> Vec<u8> {
        let t = t_raw.to_le_bytes();
        let p = p_raw.to_le_bytes();
        vec![t[0], t[1], t[2], p[0], p[1], p[2]]
    }

    #[test]
    fn conversion_rounds_to_units() {
        let d = |t, p| convert(&frame(t, p).try_into().unwrap());
        assert_eq!(
            d(25 << 16, 101_325 << 6),
            Measurement {
                temperature: 2_500,
                pressure: 101_325,
            }
        );
        // -12.345 °C, 99 999.5 Pa rounds up.
        let m = d(-809_042, 6_399_968);
        assert_eq!(m.temperature, -1_235);
        assert_eq!(m.pressure, 100_000);
        // Extremes of the 24-bit range do not overflow.
        assert_eq!(d(-(1 << 23), 0xFF_FFFF).temperature, -12_800);
        assert_eq!(d((1 << 23) - 1, 0).temperature, 12_800);
        assert_eq!(d(0, 0xFF_FFFF).pressure, 262_144);
    }

    #[test]
    fn conversion_time_formula() {
        let ulp = Config {
            pressure: Oversampling::X1,
            temperature: Oversampling::X1,
        };
        assert_eq!(ulp.max_conversion_us(), 4_000 + 2_100);
        assert_eq!(Config::STANDARD.max_conversion_us(), 4_000 + 4_095);
        let highest = Config {
            pressure: Oversampling::X128,
            temperature: Oversampling::X8,
        };
        assert_eq!(highest.max_conversion_us(), 4_000 + 86_625);
        assert_eq!(highest.osr_config(), 0x7B);
    }

    fn i2c_init(address: u8, id: u8, osr: u8) -> Vec<I2cTx> {
        vec![
            I2cTx::write_read(address, vec![REG_CHIP_ID], vec![id]),
            I2cTx::write_read(address, vec![REG_CHIP_ID], vec![id]),
            I2cTx::write_read(address, vec![REG_STATUS], vec![0x02]),
            I2cTx::write_read(address, vec![REG_INT_STATUS], vec![0x10]),
            I2cTx::write(address, vec![REG_INT_SOURCE, 0x01]),
            I2cTx::write(address, vec![REG_ODR_CONFIG, 0x70]),
            I2cTx::write(address, vec![REG_OSR_CONFIG, osr]),
        ]
    }

    #[test]
    fn i2c_forced_measurement() {
        let a = PRIMARY_ADDRESS;
        let mut ex = i2c_init(a, CHIP_ID_BMP581, 0x50);
        ex.extend([
            I2cTx::write(a, vec![REG_ODR_CONFIG, 0x72]),
            I2cTx::write_read(a, vec![REG_INT_STATUS], vec![0x00]),
            I2cTx::write_read(a, vec![REG_INT_STATUS], vec![0x01]),
            I2cTx::write_read(a, vec![REG_TEMP_DATA], frame(25 << 16, 101_325 << 6)),
            I2cTx::write(a, vec![REG_ODR_CONFIG, 0x70]),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut bmp = Bmp5xx::new_i2c(&mut i2c, a);
        assert_eq!(bmp.start_measurement().unwrap(), 8_095);
        assert_eq!(bmp.chip(), Some(Chip::Bmp581));
        assert_eq!(bmp.read_measurement(), Err(Error::NotReady));
        let m = bmp.read_measurement().unwrap();
        assert_eq!(m.temperature, 2_500);
        assert_eq!(m.pressure, 101_325);
        bmp.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn wrong_chip_id() {
        let a = SECONDARY_ADDRESS;
        let ex = [
            I2cTx::write_read(a, vec![REG_CHIP_ID], vec![0x60]),
            I2cTx::write_read(a, vec![REG_CHIP_ID], vec![0x60]),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut bmp = Bmp5xx::new_i2c(&mut i2c, a);
        assert_eq!(bmp.start_measurement(), Err(Error::UnexpectedId(0x60)));
        i2c.done();
    }

    #[test]
    fn nvm_error_is_invalid_data() {
        let a = PRIMARY_ADDRESS;
        let ex = [
            I2cTx::write_read(a, vec![REG_CHIP_ID], vec![CHIP_ID_BMP585]),
            I2cTx::write_read(a, vec![REG_CHIP_ID], vec![CHIP_ID_BMP585]),
            I2cTx::write_read(a, vec![REG_STATUS], vec![0x06]),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut bmp = Bmp5xx::new_i2c(&mut i2c, a);
        assert_eq!(bmp.init(), Err(Error::InvalidData));
        assert_eq!(bmp.chip(), None);
        i2c.done();
    }

    #[test]
    fn spi_init_dummy_read_and_unacked_reset() {
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
        let mut ex = Vec::new();
        ex.extend(rd(REG_CHIP_ID, vec![0x00]));
        ex.extend(rd(REG_CHIP_ID, vec![CHIP_ID_BMP585]));
        ex.extend(rd(REG_STATUS, vec![0x03]));
        ex.extend(rd(REG_INT_STATUS, vec![0x10]));
        ex.extend(wr(REG_INT_SOURCE, 0x01));
        ex.extend(wr(REG_ODR_CONFIG, 0x70));
        ex.extend(wr(REG_OSR_CONFIG, 0x40));
        ex.extend(wr(REG_CMD, CMD_SOFT_RESET));
        let mut spi = SpiMock::new(&ex);
        let config = Config {
            pressure: Oversampling::X1,
            temperature: Oversampling::X1,
        };
        let mut bmp = Bmp5xx::new_spi(&mut spi).with_config(config);
        assert_eq!(bmp.init().unwrap(), Chip::Bmp585);
        assert_eq!(bmp.soft_reset().unwrap(), RESET_WAIT_US);
        assert_eq!(bmp.chip(), None);
        spi.done();
    }

    #[test]
    fn i2c_reset_nack_is_ignored() {
        let a = PRIMARY_ADDRESS;
        let ex = [I2cTx::write(a, vec![REG_CMD, CMD_SOFT_RESET])
            .with_error(embedded_hal::i2c::ErrorKind::Other)];
        let mut i2c = I2cMock::new(&ex);
        let mut bmp = Bmp5xx::new_i2c(&mut i2c, a);
        assert_eq!(bmp.soft_reset().unwrap(), RESET_WAIT_US);
        i2c.done();
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

        let a = SECONDARY_ADDRESS;
        let mut ex = i2c_init(a, CHIP_ID_BMP581, 0x50);
        ex.extend([
            I2cTx::write(a, vec![REG_ODR_CONFIG, 0x72]),
            I2cTx::write_read(a, vec![REG_INT_STATUS], vec![0x01]),
            I2cTx::write_read(a, vec![REG_TEMP_DATA], frame(-(5 << 16), 90_000 << 6)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut bmp = asynch::Bmp5xx::new_i2c(&mut i2c, a);
        let mut delay = RecordingDelay(0);
        let m = block_on(bmp.measure(&mut delay)).unwrap();
        assert_eq!(m.temperature, -500);
        assert_eq!(m.pressure, 90_000);
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
