//! ST LPS22HH absolute pressure and temperature sensor over I2C or SPI,
//! one-shot mode.
//!
//! The part is kept in power-down (`ODR = 000`, 0.9 µA typ.); each
//! [`start_measurement`] sets `ONE_SHOT`, the device acquires one pressure
//! and temperature sample and drops back to power-down by itself.
//!
//! ST does not specify a one-shot conversion time. The returned wait is
//! one period of the fastest output data rate of the selected mode
//! (low-current 200 Hz → 5 ms, low-noise 75 Hz → 13.34 ms), which a
//! conversion cannot exceed. `read_measurement` checks `STATUS.P_DA` and
//! `T_DA` and returns [`Error::NotReady`] until both are set, so an early
//! read is harmless.
//!
//! The output is already compensated: pressure is 24-bit two's complement
//! at 4096 LSB/hPa, temperature 16-bit two's complement at 100 LSB/°C.
//!
//! References:
//! * ST "LPS22HH" datasheet, DocID030890 Rev 2, February 2019 (sections
//!   4, 6, 9).
//!
//! ```ignore
//! let mut lps = Lps22hh::new_i2c(i2c, ADDRESS_SA0_HIGH);
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

/// `WHO_AM_I` value.
pub const WHO_AM_I: u8 = 0xB3;

const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL_REG1: u8 = 0x10;
const REG_CTRL_REG2: u8 = 0x11;
const REG_STATUS: u8 = 0x27;

const CTRL2_IF_ADD_INC: u8 = 1 << 4;
const CTRL2_SWRESET: u8 = 1 << 2;
const CTRL2_LOW_NOISE_EN: u8 = 1 << 1;
const CTRL2_ONE_SHOT: u8 = 1 << 0;
const STATUS_DA: u8 = 0b11;

/// Reset / boot wait. Not specified by the datasheet (the `BOOT_ON` flag
/// exists instead); 5 ms is a conservative bound.
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

/// I2C transport (up to 400 kHz). Burst reads rely on `IF_ADD_INC`
/// (enabled by default and kept by this driver).
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    /// The bus.
    pub i2c: I2C,
    /// 7-bit address ([`ADDRESS_SA0_LOW`] or [`ADDRESS_SA0_HIGH`]).
    pub address: u8,
}

/// 4-wire SPI transport (mode 0 or 3, up to 10 MHz). Reads send the
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

/// Resolution mode (`CTRL_REG2.LOW_NOISE_EN`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// Low-current mode (default; 4 µA at 1 Hz, 4.5 Pa RMS noise).
    #[default]
    LowCurrent,
    /// Low-noise mode (12 µA at 1 Hz, 1.7 Pa RMS noise).
    LowNoise,
}

impl Mode {
    /// Upper bound of a one-shot conversion: one period at the maximum ODR
    /// of the mode (200 Hz low-current, 75 Hz low-noise).
    pub const fn conversion_us(self) -> WaitUs {
        match self {
            Self::LowCurrent => 5_000,
            Self::LowNoise => 13_334,
        }
    }

    const fn ctrl2(self) -> u8 {
        match self {
            Self::LowCurrent => CTRL2_IF_ADD_INC,
            Self::LowNoise => CTRL2_IF_ADD_INC | CTRL2_LOW_NOISE_EN,
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
pub fn convert(data: &[u8; 5]) -> Measurement {
    // Sign-extend the 24-bit pressure.
    let p_raw = i32::from_le_bytes([0, data[0], data[1], data[2]]) >> 8;
    let t_raw = i16::from_le_bytes([data[3], data[4]]);
    // hPa · 4096 → Pa: · 100 / 4096 = · 25 / 1024, rounded.
    let pa = (p_raw * 25 + 512) >> 10;
    Measurement {
        temperature: t_raw.into(),
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
                    mode: Mode::LowCurrent,
                    ready: false,
                }
            }

            /// Select the resolution mode applied by `init`.
            pub const fn with_mode(mut self, mode: Mode) -> Self {
                self.mode = mode;
                self
            }

            /// Current resolution mode.
            pub const fn mode(&self) -> Mode {
                self.mode
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
pub struct Lps22hh<IFACE> {
    iface: IFACE,
    mode: Mode,
    ready: bool,
}

constructors!(Lps22hh);

impl<IFACE: Interface> Lps22hh<IFACE> {
    /// Software reset of the control registers. Wait [`RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.ready = false;
        self.write(REG_CTRL_REG2, CTRL2_IF_ADD_INC | CTRL2_SWRESET)?;
        Ok(RESET_WAIT_US)
    }

    /// Check `WHO_AM_I`, select power-down / one-shot and write the
    /// resolution mode. Called by `start_measurement` if needed.
    pub fn init(&mut self) -> Result<(), Error<IFACE::Error>> {
        let mut id = [0];
        self.read(REG_WHO_AM_I, &mut id)?;
        if id[0] != WHO_AM_I {
            return Err(Error::UnexpectedId(id[0].into()));
        }
        self.write(REG_CTRL_REG1, 0)?;
        self.set_mode(self.mode)?;
        self.ready = true;
        Ok(())
    }

    /// Change the resolution mode (the device is in power-down between
    /// one-shot conversions, as the datasheet requires for `LOW_NOISE_EN`).
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<IFACE::Error>> {
        self.mode = mode;
        self.write(REG_CTRL_REG2, mode.ctrl2())
    }

    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
        self.iface.read_regs(reg, buf).map_err(Error::bus)
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
        self.iface.write_reg(reg, value).map_err(Error::bus)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Lps22hh<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        if !self.ready {
            self.init()?;
        }
        self.write(REG_CTRL_REG2, self.mode.ctrl2() | CTRL2_ONE_SHOT)?;
        Ok(self.mode.conversion_us())
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

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Lps22hh<IFACE> {
    type BusError = IFACE::Error;

    /// One-shot mode already returns to power-down; this forces `ODR = 0`.
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write(REG_CTRL_REG1, 0)
    }

    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CTRL2_IF_ADD_INC, CTRL2_ONE_SHOT, CTRL2_SWRESET, I2cInterface, Measurement, Mode,
        REG_CTRL_REG1, REG_CTRL_REG2, REG_STATUS, REG_WHO_AM_I, RESET_WAIT_US, STATUS_DA,
        SpiInterface, WHO_AM_I, convert,
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

    /// Async twin of [`super::Lps22hh`].
    #[derive(Debug)]
    pub struct Lps22hh<IFACE> {
        iface: IFACE,
        mode: Mode,
        ready: bool,
    }

    constructors!(Lps22hh);

    impl<IFACE: Interface> Lps22hh<IFACE> {
        /// See [`super::Lps22hh::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.ready = false;
            self.write(REG_CTRL_REG2, CTRL2_IF_ADD_INC | CTRL2_SWRESET)
                .await?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Lps22hh::init`].
        pub async fn init(&mut self) -> Result<(), Error<IFACE::Error>> {
            let mut id = [0];
            self.read(REG_WHO_AM_I, &mut id).await?;
            if id[0] != WHO_AM_I {
                return Err(Error::UnexpectedId(id[0].into()));
            }
            self.write(REG_CTRL_REG1, 0).await?;
            self.set_mode(self.mode).await?;
            self.ready = true;
            Ok(())
        }

        /// See [`super::Lps22hh::set_mode`].
        pub async fn set_mode(&mut self, mode: Mode) -> Result<(), Error<IFACE::Error>> {
            self.mode = mode;
            self.write(REG_CTRL_REG2, mode.ctrl2()).await
        }

        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
            self.iface.read_regs(reg, buf).await.map_err(Error::bus)
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
            self.iface.write_reg(reg, value).await.map_err(Error::bus)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Lps22hh<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            if !self.ready {
                self.init().await?;
            }
            self.write(REG_CTRL_REG2, self.mode.ctrl2() | CTRL2_ONE_SHOT)
                .await?;
            Ok(self.mode.conversion_us())
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

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Lps22hh<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write(REG_CTRL_REG1, 0).await
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

    #[test]
    fn conversion() {
        let c = |p: i32, t: i16| {
            let f = frame(0, p, t);
            convert(&[f[1], f[2], f[3], f[4], f[5]])
        };
        // Datasheet example: 0x3FF58D = 4 191 629 LSB = 1023.3 hPa.
        assert_eq!(
            c(0x3F_F58D, 2_534),
            Measurement {
                temperature: 2_534,
                pressure: 102_335,
            }
        );
        assert_eq!(c(101_325 * 4_096 / 100, -1_234).temperature, -1_234);
        assert_eq!(c(101_325 * 4_096 / 100, 0).pressure, 101_325);
        // Negative raw values (sign extension) clamp to 0 Pa.
        assert_eq!(c(-5, 0).pressure, 0);
        assert_eq!(c(0x7F_FFFF, 0).pressure, 204_800);
    }

    fn i2c_init(a: u8, ctrl2: u8) -> Vec<I2cTx> {
        vec![
            I2cTx::write_read(a, vec![REG_WHO_AM_I], vec![WHO_AM_I]),
            I2cTx::write(a, vec![REG_CTRL_REG1, 0]),
            I2cTx::write(a, vec![REG_CTRL_REG2, ctrl2]),
        ]
    }

    #[test]
    fn i2c_one_shot() {
        let a = ADDRESS_SA0_HIGH;
        let mut ex = i2c_init(a, 0x12);
        ex.extend([
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x13]),
            I2cTx::write_read(a, vec![REG_STATUS], frame(0x01, 0, 0)),
            I2cTx::write_read(a, vec![REG_STATUS], frame(0x03, 4_150_272, 2_100)),
            I2cTx::write(a, vec![REG_CTRL_REG1, 0]),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut lps = Lps22hh::new_i2c(&mut i2c, a).with_mode(Mode::LowNoise);
        assert_eq!(lps.read_measurement(), Err(Error::NotReady));
        assert_eq!(lps.start_measurement().unwrap(), 13_334);
        assert_eq!(lps.read_measurement(), Err(Error::NotReady));
        let m = lps.read_measurement().unwrap();
        assert_eq!((m.temperature, m.pressure), (2_100, 101_325));
        lps.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn wrong_id_and_reset() {
        let a = ADDRESS_SA0_LOW;
        let ex = [
            I2cTx::write_read(a, vec![REG_WHO_AM_I], vec![0xB1]),
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x14]),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut lps = Lps22hh::new_i2c(&mut i2c, a);
        assert_eq!(lps.start_measurement(), Err(Error::UnexpectedId(0xB1)));
        assert_eq!(lps.soft_reset().unwrap(), RESET_WAIT_US);
        i2c.done();
    }

    #[test]
    fn spi_one_shot() {
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
        ex.extend(wr(REG_CTRL_REG1, 0));
        ex.extend(wr(REG_CTRL_REG2, 0x10));
        ex.extend(wr(REG_CTRL_REG2, 0x11));
        ex.extend(rd(REG_STATUS, frame(0x33, 3_686_400, -500)));
        let mut spi = SpiMock::new(&ex);
        let mut lps = Lps22hh::new_spi(&mut spi);
        lps.start_measurement().unwrap();
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
        let mut ex = i2c_init(a, 0x10);
        ex.extend([
            I2cTx::write(a, vec![REG_CTRL_REG2, 0x11]),
            I2cTx::write_read(a, vec![REG_STATUS], frame(0x03, 4_096_000, 2_000)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut lps = asynch::Lps22hh::new_i2c(&mut i2c, a);
        let mut delay = RecordingDelay(0);
        let m = block_on(lps.measure(&mut delay)).unwrap();
        assert_eq!((m.temperature, m.pressure), (2_000, 100_000));
        assert_eq!(delay.0, 5_000);
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
