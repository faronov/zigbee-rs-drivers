//! ST LPS22DF and LPS28DFW absolute pressure and temperature sensors,
//! one-shot mode.
//!
//! Both parts report `WHO_AM_I = 0xB4` and share the register map, so the
//! [`Variant`] is chosen by the caller. The LPS28DFW (water-resistant,
//! I2C/I3C only) adds a second full scale (`FS_MODE`, up to 4060 hPa at
//! 2048 LSB/hPa); the LPS22DF (I2C/I3C/SPI) only has the 1260 hPa range.
//!
//! The part is kept in power-down (`ODR = 0000`, 0.9 µA typ.); each
//! [`start_measurement`] sets `ONESHOT`, the device acquires one averaged
//! pressure and temperature sample and returns to power-down by itself.
//!
//! ST does not specify a one-shot conversion time. The returned wait is one
//! period of the maximum ODR allowed for the selected averaging (datasheet
//! power-consumption table: AVG 512 → 25 Hz … AVG 4 → 500 Hz), which a
//! conversion cannot exceed. `read_measurement` checks `STATUS.P_DA` and
//! `T_DA` and returns [`Error::NotReady`] until both are set.
//!
//! References:
//! * ST "LPS22DF" datasheet, DS13316 Rev 3 (sections 4, 9; table 19).
//! * ST "LPS28DFW" datasheet, DS13317 Rev 1 (sections 4, 9; table 21).
//!
//! ```ignore
//! let mut lps = Lps2xdf::new_i2c(i2c, ADDRESS_SA0_HIGH, Variant::Lps22df);
//! let wait_us = lps.start_measurement()?; // probes WHO_AM_I first time
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = lps.read_measurement()?;
//! ```
//!
//! [`start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with SDO/SA0 tied to GND.
pub const ADDRESS_SA0_LOW: u8 = 0x5C;
/// I2C address with SDO/SA0 tied to VDD.
pub const ADDRESS_SA0_HIGH: u8 = 0x5D;

/// `WHO_AM_I` value of both parts.
pub const WHO_AM_I: u8 = 0xB4;

const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL_REG1: u8 = 0x10;
const REG_CTRL_REG2: u8 = 0x11;
const REG_STATUS: u8 = 0x27;

const CTRL2_FS_MODE: u8 = 1 << 6;
const CTRL2_SWRESET: u8 = 1 << 2;
const CTRL2_ONESHOT: u8 = 1 << 0;
const STATUS_DA: u8 = 0b11;

/// Reset wait. Not specified (`SWRESET` self-clears); 5 ms is a
/// conservative bound.
pub const RESET_WAIT_US: WaitUs = 5_000;

/// Register access over I2C or SPI.
pub trait Interface {
    /// Transport error.
    type Error;
    /// Burst-read consecutive registers starting at `reg`.
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write one register.
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
}

/// I2C transport. Burst reads rely on `CTRL_REG3.IF_ADD_INC` (enabled by
/// default).
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    /// The bus.
    pub i2c: I2C,
    /// 7-bit address ([`ADDRESS_SA0_LOW`] or [`ADDRESS_SA0_HIGH`]).
    pub address: u8,
}

/// 4-wire SPI transport (LPS22DF only; mode 0 or 3). Reads send the
/// address with bit 7 set.
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

/// Which part is fitted (both report the same `WHO_AM_I`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// LPS22DF, 260-1260 hPa.
    Lps22df,
    /// LPS28DFW, 260-1260 hPa or 260-4060 hPa.
    Lps28dfw,
}

/// Number of averaged samples (`CTRL_REG1.AVG`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Avg {
    /// 4 samples: 1.7 µA at 1 Hz.
    X4,
    /// 8 samples: 2.0 µA at 1 Hz.
    X8,
    /// 16 samples: 2.7 µA at 1 Hz (default).
    #[default]
    X16,
    /// 32 samples: 3.7 µA at 1 Hz.
    X32,
    /// 64 samples: 5.6 µA at 1 Hz.
    X64,
    /// 128 samples: 9.4 µA at 1 Hz.
    X128,
    /// 512 samples: 32.2 µA at 1 Hz.
    X512,
}

impl Avg {
    const fn bits(self) -> u8 {
        match self {
            Self::X4 => 0b000,
            Self::X8 => 0b001,
            Self::X16 => 0b010,
            Self::X32 => 0b011,
            Self::X64 => 0b100,
            Self::X128 => 0b101,
            Self::X512 => 0b111,
        }
    }

    /// Upper bound of a one-shot conversion: one period of the maximum ODR
    /// allowed with this averaging.
    pub const fn conversion_us(self) -> WaitUs {
        match self {
            Self::X4 => 2_000,
            Self::X8 => 2_500,
            Self::X16 => 3_334,
            Self::X32 => 5_000,
            Self::X64 => 10_000,
            Self::X128 => 13_334,
            Self::X512 => 40_000,
        }
    }
}

/// Pressure full scale (`CTRL_REG2.FS_MODE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FullScale {
    /// Mode 1: up to 1260 hPa, 4096 LSB/hPa (both parts).
    #[default]
    Hpa1260,
    /// Mode 2: up to 4060 hPa, 2048 LSB/hPa (LPS28DFW only).
    Hpa4060,
}

/// Measurement settings, written by `init`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Averaging.
    pub avg: Avg,
    /// Full scale.
    pub full_scale: FullScale,
}

impl Config {
    const fn ctrl2(self) -> u8 {
        match self.full_scale {
            FullScale::Hpa1260 => 0,
            FullScale::Hpa4060 => CTRL2_FS_MODE,
        }
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

/// Convert `PRESS_OUT_XL..TEMP_OUT_H` (5 bytes, little-endian) to units.
pub fn convert(data: &[u8; 5], full_scale: FullScale) -> Measurement {
    // Sign-extend the 24-bit pressure.
    let p_raw = i32::from_le_bytes([0, data[0], data[1], data[2]]) >> 8;
    let t_raw = i16::from_le_bytes([data[3], data[4]]);
    // Pa = raw · 100 / 4096 (or / 2048), rounded.
    let pa = match full_scale {
        FullScale::Hpa1260 => (p_raw * 25 + 512) >> 10,
        FullScale::Hpa4060 => (p_raw * 25 + 256) >> 9,
    };
    Measurement {
        temperature: t_raw.into(),
        pressure: pa.max(0) as u32,
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Create a driver over any transport; no bus traffic.
            pub const fn new(iface: IFACE, variant: Variant) -> Self {
                Self {
                    iface,
                    variant,
                    config: Config {
                        avg: Avg::X16,
                        full_scale: FullScale::Hpa1260,
                    },
                    ready: false,
                }
            }

            /// Select the settings applied by `init`.
            pub const fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self.ready = false;
                self
            }

            /// Current settings.
            pub const fn config(&self) -> Config {
                self.config
            }

            /// Configured part.
            pub const fn variant(&self) -> Variant {
                self.variant
            }

            /// Give the transport back.
            pub fn release(self) -> IFACE {
                self.iface
            }

            fn check_config(&self) -> bool {
                self.variant == Variant::Lps28dfw || self.config.full_scale == FullScale::Hpa1260
            }
        }

        impl<I2C> $name<I2cInterface<I2C>> {
            /// Driver on an I2C bus.
            pub const fn new_i2c(i2c: I2C, address: u8, variant: Variant) -> Self {
                Self::new(I2cInterface { i2c, address }, variant)
            }
        }

        impl<SPI> $name<SpiInterface<SPI>> {
            /// LPS22DF on an SPI device (the LPS28DFW has no SPI).
            pub const fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface { spi }, Variant::Lps22df)
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Lps2xdf<IFACE> {
    iface: IFACE,
    variant: Variant,
    config: Config,
    ready: bool,
}

constructors!(Lps2xdf);

impl<IFACE: Interface> Lps2xdf<IFACE> {
    /// Software reset of the volatile registers. Wait [`RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.ready = false;
        self.write(REG_CTRL_REG2, CTRL2_SWRESET)?;
        Ok(RESET_WAIT_US)
    }

    /// Check `WHO_AM_I` and write the configuration (power-down, AVG,
    /// full scale). Called by `start_measurement` if needed.
    ///
    /// [`Error::Unsupported`] if [`FullScale::Hpa4060`] is requested on an
    /// LPS22DF.
    pub fn init(&mut self) -> Result<(), Error<IFACE::Error>> {
        if !self.check_config() {
            return Err(Error::Unsupported);
        }
        let mut id = [0];
        self.read(REG_WHO_AM_I, &mut id)?;
        if id[0] != WHO_AM_I {
            return Err(Error::UnexpectedId(id[0].into()));
        }
        self.write(REG_CTRL_REG1, self.config.avg.bits())?;
        self.write(REG_CTRL_REG2, self.config.ctrl2())?;
        self.ready = true;
        Ok(())
    }

    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
        self.iface.read_regs(reg, buf).map_err(Error::bus)
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
        self.iface.write_reg(reg, value).map_err(Error::bus)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Lps2xdf<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        if !self.ready {
            self.init()?;
        }
        self.write(REG_CTRL_REG2, self.config.ctrl2() | CTRL2_ONESHOT)?;
        Ok(self.config.avg.conversion_us())
    }

    /// [`Error::NotReady`] until both data-available flags are set.
    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        if !self.ready {
            return Err(Error::NotReady);
        }
        let mut buf = [0; 6];
        self.read(REG_STATUS, &mut buf)?;
        if buf[0] & STATUS_DA != STATUS_DA {
            return Err(Error::NotReady);
        }
        Ok(convert(
            &[buf[1], buf[2], buf[3], buf[4], buf[5]],
            self.config.full_scale,
        ))
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Lps2xdf<IFACE> {
    type BusError = IFACE::Error;

    /// One-shot mode already returns to power-down; this forces `ODR = 0`.
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write(REG_CTRL_REG1, self.config.avg.bits())
    }

    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        Avg, CTRL2_ONESHOT, CTRL2_SWRESET, Config, FullScale, I2cInterface, Measurement,
        REG_CTRL_REG1, REG_CTRL_REG2, REG_STATUS, REG_WHO_AM_I, RESET_WAIT_US, STATUS_DA,
        SpiInterface, Variant, WHO_AM_I, convert,
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

    /// Async twin of [`super::Lps2xdf`].
    #[derive(Debug)]
    pub struct Lps2xdf<IFACE> {
        iface: IFACE,
        variant: Variant,
        config: Config,
        ready: bool,
    }

    constructors!(Lps2xdf);

    impl<IFACE: Interface> Lps2xdf<IFACE> {
        /// See [`super::Lps2xdf::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.ready = false;
            self.write(REG_CTRL_REG2, CTRL2_SWRESET).await?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Lps2xdf::init`].
        pub async fn init(&mut self) -> Result<(), Error<IFACE::Error>> {
            if !self.check_config() {
                return Err(Error::Unsupported);
            }
            let mut id = [0];
            self.read(REG_WHO_AM_I, &mut id).await?;
            if id[0] != WHO_AM_I {
                return Err(Error::UnexpectedId(id[0].into()));
            }
            self.write(REG_CTRL_REG1, self.config.avg.bits()).await?;
            self.write(REG_CTRL_REG2, self.config.ctrl2()).await?;
            self.ready = true;
            Ok(())
        }

        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
            self.iface.read_regs(reg, buf).await.map_err(Error::bus)
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
            self.iface.write_reg(reg, value).await.map_err(Error::bus)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Lps2xdf<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            if !self.ready {
                self.init().await?;
            }
            self.write(REG_CTRL_REG2, self.config.ctrl2() | CTRL2_ONESHOT)
                .await?;
            Ok(self.config.avg.conversion_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            if !self.ready {
                return Err(Error::NotReady);
            }
            let mut buf = [0; 6];
            self.read(REG_STATUS, &mut buf).await?;
            if buf[0] & STATUS_DA != STATUS_DA {
                return Err(Error::NotReady);
            }
            Ok(convert(
                &[buf[1], buf[2], buf[3], buf[4], buf[5]],
                self.config.full_scale,
            ))
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Lps2xdf<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write(REG_CTRL_REG1, self.config.avg.bits()).await
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

    fn frame(status: u8, p_raw: i32, t_raw: i16) -> Vec<u8> {
        let p = p_raw.to_le_bytes();
        let t = t_raw.to_le_bytes();
        vec![status, p[0], p[1], p[2], t[0], t[1]]
    }

    fn conv(p: i32, t: i16, fs: FullScale) -> Measurement {
        let f = frame(0, p, t);
        convert(&[f[1], f[2], f[3], f[4], f[5]], fs)
    }

    #[test]
    fn conversion() {
        // LPS22DF datasheet example: 0x3FF58D / 4096 = 1023.3 hPa;
        // 0x09C4 / 100 = 25.00 °C.
        assert_eq!(
            conv(0x3F_F58D, 0x09C4, FullScale::Hpa1260),
            Measurement {
                temperature: 2_500,
                pressure: 102_335,
            }
        );
        // LPS28DFW mode-2 example: 0x3FF58D / 2048 = 2046.7 hPa.
        assert_eq!(conv(0x3F_F58D, 0, FullScale::Hpa4060).pressure, 204_669);
        assert_eq!(conv(0x7F_FFFF, 0, FullScale::Hpa4060).pressure, 409_600);
        assert_eq!(conv(-1, -2_000, FullScale::Hpa1260).pressure, 0);
        assert_eq!(conv(-1, -2_000, FullScale::Hpa1260).temperature, -2_000);
    }

    #[test]
    fn i2c_one_shot_28dfw_mode2() {
        let a = ADDRESS_SA0_HIGH;
        let ex = [
            I2cTx::write_read(a, vec![REG_WHO_AM_I], vec![WHO_AM_I]),
            I2cTx::write(a, vec![REG_CTRL_REG1, 0x07]),
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x40]),
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x41]),
            I2cTx::write_read(a, vec![REG_STATUS], frame(0x02, 0, 0)),
            I2cTx::write_read(a, vec![REG_STATUS], frame(0x03, 6_144_000, 1_500)),
            I2cTx::write(a, vec![REG_CTRL_REG1, 0x07]),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut lps = Lps2xdf::new_i2c(&mut i2c, a, Variant::Lps28dfw).with_config(Config {
            avg: Avg::X512,
            full_scale: FullScale::Hpa4060,
        });
        assert_eq!(lps.read_measurement(), Err(Error::NotReady));
        assert_eq!(lps.start_measurement().unwrap(), 40_000);
        assert_eq!(lps.read_measurement(), Err(Error::NotReady));
        let m = lps.read_measurement().unwrap();
        assert_eq!((m.temperature, m.pressure), (1_500, 300_000));
        lps.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn invalid_config_and_wrong_id() {
        let mut i2c = I2cMock::new(&[]);
        let mut lps =
            Lps2xdf::new_i2c(&mut i2c, ADDRESS_SA0_LOW, Variant::Lps22df).with_config(Config {
                avg: Avg::X4,
                full_scale: FullScale::Hpa4060,
            });
        assert_eq!(lps.start_measurement(), Err(Error::Unsupported));
        i2c.done();

        let a = ADDRESS_SA0_LOW;
        let ex = [
            I2cTx::write_read(a, vec![REG_WHO_AM_I], vec![0xB3]),
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x04]),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut lps = Lps2xdf::new_i2c(&mut i2c, a, Variant::Lps22df);
        assert_eq!(lps.start_measurement(), Err(Error::UnexpectedId(0xB3)));
        assert_eq!(lps.soft_reset().unwrap(), RESET_WAIT_US);
        i2c.done();
    }

    #[test]
    fn spi_one_shot_22df() {
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
        ex.extend(rd(REG_WHO_AM_I, vec![WHO_AM_I]));
        ex.extend(wr(REG_CTRL_REG1, 0x02));
        ex.extend(wr(REG_CTRL_REG2, 0x00));
        ex.extend(wr(REG_CTRL_REG2, 0x01));
        ex.extend(rd(REG_STATUS, frame(0x33, 3_686_400, -500)));
        let mut spi = SpiMock::new(&ex);
        let mut lps = Lps2xdf::new_spi(&mut spi);
        assert_eq!(lps.variant(), Variant::Lps22df);
        assert_eq!(lps.start_measurement().unwrap(), 3_334);
        let m = lps.read_measurement().unwrap();
        assert_eq!((m.temperature, m.pressure), (-500, 90_000));
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

        let a = ADDRESS_SA0_LOW;
        let ex = [
            I2cTx::write_read(a, vec![REG_WHO_AM_I], vec![WHO_AM_I]),
            I2cTx::write(a, vec![REG_CTRL_REG1, 0x00]),
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x00]),
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x01]),
            I2cTx::write_read(a, vec![REG_STATUS], frame(0x03, 4_096_000, 2_000)),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut lps = asynch::Lps2xdf::new_i2c(&mut i2c, a, Variant::Lps22df).with_config(Config {
            avg: Avg::X4,
            ..Config::default()
        });
        let mut delay = RecordingDelay(0);
        let m = block_on(lps.measure(&mut delay)).unwrap();
        assert_eq!((m.temperature, m.pressure), (2_000, 100_000));
        assert_eq!(delay.0, 2_000);
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
