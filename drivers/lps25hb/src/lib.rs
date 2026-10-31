//! ST LPS25HB absolute pressure and temperature sensor over I2C or SPI,
//! one-shot mode.
//!
//! `init` turns the device on (`CTRL_REG1.PD = 1`) with `ODR = 000`; each
//! [`start_measurement`] sets `ONE_SHOT`, the device acquires one averaged
//! pressure and temperature sample and goes back to its internal
//! power-down (AN4672 section 5). [`LowPower::sleep`] additionally clears
//! `PD` (0.5 µA); the next start sets it again.
//!
//! ST does not specify a one-shot conversion time. The returned wait is one
//! period of the 25 Hz maximum ODR, which AN4672 allows with the highest
//! averaging (AVGP 512, AVGT 64). `read_measurement` checks `STATUS.P_DA`
//! and `T_DA` and returns [`Error::NotReady`] until both are set.
//!
//! Output: pressure is 24-bit two's complement at 4096 LSB/hPa. Temperature
//! is 16-bit two's complement at 480 LSB/°C with a +42.5 °C offset; the
//! offset is not printed in the LPS25HB datasheet and is taken from ST's
//! LPS25H documentation and ST's LPS25HB standard C driver.
//!
//! References:
//! * ST "LPS25HB" datasheet, DocID027112 Rev 4, August 2016 (sections 3,
//!   8, 9).
//! * ST AN4672 "LPS22HB/LPS25HB digital pressure sensors: hardware
//!   guidelines for system integration", DocID027655 Rev 3.
//!
//! ```ignore
//! let mut lps = Lps25hb::new_i2c(i2c, ADDRESS_SA0_HIGH);
//! let wait_us = lps.start_measurement()?; // probes WHO_AM_I first time
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = lps.read_measurement()?;
//! ```
//!
//! [`start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`LowPower::sleep`]: zigbee_sensor::blocking::LowPower::sleep

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with SDO/SA0 tied to GND.
pub const ADDRESS_SA0_LOW: u8 = 0x5C;
/// I2C address with SDO/SA0 tied to VDD.
pub const ADDRESS_SA0_HIGH: u8 = 0x5D;

/// `WHO_AM_I` value.
pub const WHO_AM_I: u8 = 0xBD;

const REG_WHO_AM_I: u8 = 0x0F;
const REG_RES_CONF: u8 = 0x10;
const REG_CTRL_REG1: u8 = 0x20;
const REG_CTRL_REG2: u8 = 0x21;
const REG_STATUS: u8 = 0x27;

const CTRL1_PD: u8 = 1 << 7;
const CTRL2_BOOT: u8 = 1 << 7;
const CTRL2_ONE_SHOT: u8 = 1 << 0;
const STATUS_DA: u8 = 0b11;

/// Reboot wait (memory reload, 2.2 ms typ.; rounded up).
pub const RESET_WAIT_US: WaitUs = 3_000;
/// Upper bound of a one-shot conversion: one period at the 25 Hz maximum
/// ODR.
pub const CONVERSION_US: WaitUs = 40_000;

/// Register access over I2C or SPI.
pub trait Interface {
    /// Transport error.
    type Error;
    /// Burst-read consecutive registers starting at `reg`.
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write one register.
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
}

/// I2C transport (up to 400 kHz). Reads set bit 7 of the sub-address to
/// auto-increment.
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    /// The bus.
    pub i2c: I2C,
    /// 7-bit address ([`ADDRESS_SA0_LOW`] or [`ADDRESS_SA0_HIGH`]).
    pub address: u8,
}

/// 4-wire SPI transport (mode 0 or 3, up to 10 MHz). Reads send the
/// address with bit 7 (read) and bit 6 (auto-increment) set.
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    /// The device (bus + chip select).
    pub spi: SPI,
}

impl<I2C: embedded_hal::i2c::I2c> Interface for I2cInterface<I2C> {
    type Error = I2C::Error;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[reg | 0x80], buf)
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
            .transaction(&mut [Operation::Write(&[reg | 0xC0]), Operation::Read(buf)])
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg & 0x3F, value])
    }
}

/// Pressure samples averaged per conversion (`RES_CONF.AVGP`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AvgP {
    /// 8 samples.
    X8,
    /// 32 samples.
    X32,
    /// 128 samples.
    X128,
    /// 512 samples (reset default).
    #[default]
    X512,
}

/// Temperature samples averaged per conversion (`RES_CONF.AVGT`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AvgT {
    /// 8 samples.
    X8,
    /// 16 samples.
    X16,
    /// 32 samples.
    X32,
    /// 64 samples (reset default).
    #[default]
    X64,
}

/// Resolution settings, written to `RES_CONF` by `init`.
///
/// The default (AVGP 512, AVGT 64, `RES_CONF = 0x0F`) is the datasheet
/// high-resolution mode, 25 µA at 1 Hz; [`Config::LOW_POWER`]
/// (`RES_CONF = 0x04`) is its low-resolution mode, 4 µA at 1 Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Pressure averaging.
    pub avg_p: AvgP,
    /// Temperature averaging.
    pub avg_t: AvgT,
}

impl Config {
    /// Datasheet low-resolution mode (AVGP 8, AVGT 16).
    pub const LOW_POWER: Self = Self {
        avg_p: AvgP::X8,
        avg_t: AvgT::X16,
    };

    const fn res_conf(self) -> u8 {
        (self.avg_t as u8) << 2 | self.avg_p as u8
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
pub fn convert(data: &[u8; 5]) -> Measurement {
    // Sign-extend the 24-bit pressure.
    let p_raw = i32::from_le_bytes([0, data[0], data[1], data[2]]) >> 8;
    let t_raw = i32::from(i16::from_le_bytes([data[3], data[4]]));
    // hPa · 4096 → Pa: · 25 / 1024, rounded.
    let pa = (p_raw * 25 + 512) >> 10;
    // 42.5 °C + raw / 480 in 0.01 °C = 4250 + raw · 5 / 24, rounded.
    let temperature = 4_250 + (t_raw * 5 + 12).div_euclid(24);
    Measurement {
        temperature,
        pressure: pa.max(0) as u32,
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Create a driver over any transport; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    config: Config {
                        avg_p: AvgP::X512,
                        avg_t: AvgT::X64,
                    },
                    ready: false,
                    active: false,
                }
            }

            /// Select the resolution applied by `init`.
            pub const fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self.ready = false;
                self
            }

            /// Current resolution.
            pub const fn config(&self) -> Config {
                self.config
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
pub struct Lps25hb<IFACE> {
    iface: IFACE,
    config: Config,
    ready: bool,
    active: bool,
}

constructors!(Lps25hb);

impl<IFACE: Interface> Lps25hb<IFACE> {
    /// Reload the trimming parameters (`BOOT`). Wait [`RESET_WAIT_US`].
    pub fn reboot(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.ready = false;
        self.write(REG_CTRL_REG2, CTRL2_BOOT)?;
        Ok(RESET_WAIT_US)
    }

    /// Check `WHO_AM_I`, write `RES_CONF` and turn the device on in
    /// one-shot mode. Called by `start_measurement` if needed.
    pub fn init(&mut self) -> Result<(), Error<IFACE::Error>> {
        let mut id = [0];
        self.read(REG_WHO_AM_I, &mut id)?;
        if id[0] != WHO_AM_I {
            return Err(Error::UnexpectedId(id[0].into()));
        }
        self.write(REG_CTRL_REG1, 0)?;
        self.write(REG_RES_CONF, self.config.res_conf())?;
        self.write(REG_CTRL_REG1, CTRL1_PD)?;
        self.active = true;
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

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Lps25hb<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        if !self.ready {
            self.init()?;
        } else if !self.active {
            self.write(REG_CTRL_REG1, CTRL1_PD)?;
            self.active = true;
        }
        self.write(REG_CTRL_REG2, CTRL2_ONE_SHOT)?;
        Ok(CONVERSION_US)
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
        Ok(convert(&[buf[1], buf[2], buf[3], buf[4], buf[5]]))
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Lps25hb<IFACE> {
    type BusError = IFACE::Error;

    /// Clear `PD` (0.5 µA). The next `start_measurement` sets it again.
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write(REG_CTRL_REG1, 0)?;
        self.active = false;
        Ok(())
    }

    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        AvgP, AvgT, CONVERSION_US, CTRL1_PD, CTRL2_BOOT, CTRL2_ONE_SHOT, Config, I2cInterface,
        Measurement, REG_CTRL_REG1, REG_CTRL_REG2, REG_RES_CONF, REG_STATUS, REG_WHO_AM_I,
        RESET_WAIT_US, STATUS_DA, SpiInterface, WHO_AM_I, convert,
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
            self.i2c.write_read(self.address, &[reg | 0x80], buf).await
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
                .transaction(&mut [Operation::Write(&[reg | 0xC0]), Operation::Read(buf)])
                .await
        }

        async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
            self.spi.write(&[reg & 0x3F, value]).await
        }
    }

    /// Async twin of [`super::Lps25hb`].
    #[derive(Debug)]
    pub struct Lps25hb<IFACE> {
        iface: IFACE,
        config: Config,
        ready: bool,
        active: bool,
    }

    constructors!(Lps25hb);

    impl<IFACE: Interface> Lps25hb<IFACE> {
        /// See [`super::Lps25hb::reboot`].
        pub async fn reboot(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.ready = false;
            self.write(REG_CTRL_REG2, CTRL2_BOOT).await?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Lps25hb::init`].
        pub async fn init(&mut self) -> Result<(), Error<IFACE::Error>> {
            let mut id = [0];
            self.read(REG_WHO_AM_I, &mut id).await?;
            if id[0] != WHO_AM_I {
                return Err(Error::UnexpectedId(id[0].into()));
            }
            self.write(REG_CTRL_REG1, 0).await?;
            self.write(REG_RES_CONF, self.config.res_conf()).await?;
            self.write(REG_CTRL_REG1, CTRL1_PD).await?;
            self.active = true;
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

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Lps25hb<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            if !self.ready {
                self.init().await?;
            } else if !self.active {
                self.write(REG_CTRL_REG1, CTRL1_PD).await?;
                self.active = true;
            }
            self.write(REG_CTRL_REG2, CTRL2_ONE_SHOT).await?;
            Ok(CONVERSION_US)
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
            Ok(convert(&[buf[1], buf[2], buf[3], buf[4], buf[5]]))
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Lps25hb<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write(REG_CTRL_REG1, 0).await?;
            self.active = false;
            Ok(())
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

    fn conv(p: i32, t: i16) -> Measurement {
        let f = frame(0, p, t);
        convert(&[f[1], f[2], f[3], f[4], f[5]])
    }

    #[test]
    fn conversion() {
        // Datasheet example: 0x3FF58D = 1023.3 hPa.
        assert_eq!(conv(0x3F_F58D, 0).pressure, 102_335);
        assert_eq!(conv(0, 0).temperature, 4_250);
        // 25.00 °C = 42.5 + raw / 480 → raw = -8400.
        assert_eq!(conv(0, -8_400).temperature, 2_500);
        // -10 °C → raw = -25200; 85 °C → raw = 20400.
        assert_eq!(conv(0, -25_200).temperature, -1_000);
        assert_eq!(conv(0, 20_400).temperature, 8_500);
        // Rounding: 1 LSB = 0.208 (0.01 °C), 3 LSB = 0.625.
        assert_eq!(conv(0, 1).temperature, 4_250);
        assert_eq!(conv(0, 3).temperature, 4_251);
        assert_eq!(conv(0, -3).temperature, 4_249);
        assert_eq!(conv(0, i16::MIN).temperature, 4_250 - 6_827);
        assert_eq!(conv(-2, 0).pressure, 0);
    }

    #[test]
    fn i2c_one_shot_and_sleep() {
        let a = ADDRESS_SA0_HIGH;
        let ex = [
            I2cTx::write_read(a, vec![0x80 | REG_WHO_AM_I], vec![WHO_AM_I]),
            I2cTx::write(a, vec![REG_CTRL_REG1, 0]),
            I2cTx::write(a, vec![REG_RES_CONF, 0x04]),
            I2cTx::write(a, vec![REG_CTRL_REG1, 0x80]),
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x01]),
            I2cTx::write_read(a, vec![0x80 | REG_STATUS], frame(0x02, 0, 0)),
            I2cTx::write_read(a, vec![0x80 | REG_STATUS], frame(0x03, 4_150_272, -8_400)),
            I2cTx::write(a, vec![REG_CTRL_REG1, 0]),
            I2cTx::write(a, vec![REG_CTRL_REG1, 0x80]),
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x01]),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut lps = Lps25hb::new_i2c(&mut i2c, a).with_config(Config::LOW_POWER);
        assert_eq!(lps.read_measurement(), Err(Error::NotReady));
        assert_eq!(lps.start_measurement().unwrap(), CONVERSION_US);
        assert_eq!(lps.read_measurement(), Err(Error::NotReady));
        let m = lps.read_measurement().unwrap();
        assert_eq!((m.temperature, m.pressure), (2_500, 101_325));
        lps.sleep().unwrap();
        lps.start_measurement().unwrap();
        i2c.done();
    }

    #[test]
    fn wrong_id_and_reboot() {
        let a = ADDRESS_SA0_LOW;
        let ex = [
            I2cTx::write_read(a, vec![0x80 | REG_WHO_AM_I], vec![0xB1]),
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x80]),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut lps = Lps25hb::new_i2c(&mut i2c, a);
        assert_eq!(lps.start_measurement(), Err(Error::UnexpectedId(0xB1)));
        assert_eq!(lps.reboot().unwrap(), RESET_WAIT_US);
        i2c.done();
    }

    #[test]
    fn spi_one_shot() {
        let rd = |reg: u8, v: Vec<u8>| {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![0xC0 | reg]),
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
        ex.extend(wr(REG_CTRL_REG1, 0));
        ex.extend(wr(REG_RES_CONF, 0x0F));
        ex.extend(wr(REG_CTRL_REG1, 0x80));
        ex.extend(wr(REG_CTRL_REG2, 0x01));
        ex.extend(rd(REG_STATUS, frame(0x03, 3_686_400, 0)));
        let mut spi = SpiMock::new(&ex);
        let mut lps = Lps25hb::new_spi(&mut spi);
        lps.start_measurement().unwrap();
        let m = lps.read_measurement().unwrap();
        assert_eq!((m.temperature, m.pressure), (4_250, 90_000));
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
            I2cTx::write_read(a, vec![0x80 | REG_WHO_AM_I], vec![WHO_AM_I]),
            I2cTx::write(a, vec![REG_CTRL_REG1, 0]),
            I2cTx::write(a, vec![REG_RES_CONF, 0x0F]),
            I2cTx::write(a, vec![REG_CTRL_REG1, 0x80]),
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x01]),
            I2cTx::write_read(a, vec![0x80 | REG_STATUS], frame(0x03, 4_096_000, -8_400)),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut lps = asynch::Lps25hb::new_i2c(&mut i2c, a);
        let mut delay = RecordingDelay(0);
        let m = block_on(lps.measure(&mut delay)).unwrap();
        assert_eq!((m.temperature, m.pressure), (2_500, 100_000));
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
