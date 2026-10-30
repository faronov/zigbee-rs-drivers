//! STMicroelectronics HTS221 temperature and humidity sensor (I2C or
//! 3-wire SPI).
//!
//! The raw outputs are converted with per-device calibration registers
//! (0x30..=0x3F) by linear interpolation between two factory points for
//! temperature and two for humidity (datasheet section 8). The driver keeps
//! the device in one-shot mode: `start_measurement` sets ONE_SHOT and
//! `read_measurement` checks the T_DA / H_DA status bits, then reads both
//! outputs and interpolates in integer arithmetic.
//!
//! The bus is abstracted by the small [`Interface`] trait with
//! [`I2cInterface`] and [`SpiInterface`] implementations (and async twins
//! in [`asynch`]).
//!
//! Reference: ST DocID026333 "HTS221" datasheet, Rev 4 (August 2016).
//!
//! ```ignore
//! let mut hts = Hts221::new_i2c(i2c);
//! hts.init()?;
//! let wait_us = hts.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = hts.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address (0xBE / 0xBF as 8-bit write / read).
pub const ADDRESS: u8 = 0x5F;
/// WHO_AM_I value.
pub const DEVICE_ID: u8 = 0xBC;

const REG_WHO_AM_I: u8 = 0x0F;
const REG_AV_CONF: u8 = 0x10;
const REG_CTRL1: u8 = 0x20;
const REG_CTRL2: u8 = 0x21;
const REG_STATUS: u8 = 0x27;
const REG_HUMIDITY_OUT_L: u8 = 0x28;
const REG_CALIB_0: u8 = 0x30;

const CTRL1_PD: u8 = 1 << 7;
const CTRL1_BDU: u8 = 1 << 2;
const CTRL2_BOOT: u8 = 1 << 7;
const CTRL2_HEATER: u8 = 1 << 1;
const CTRL2_ONE_SHOT: u8 = 1 << 0;
const STATUS_H_DA: u8 = 1 << 1;
const STATUS_T_DA: u8 = 1 << 0;

/// Wait after setting ONE_SHOT. The datasheet gives no conversion time;
/// this is one period of the fastest output data rate (12.5 Hz). Longer
/// averaging may need more, which `read_measurement` reports as
/// [`Error::NotReady`].
pub const ONE_SHOT_WAIT_US: WaitUs = 80_000;

/// Register access used by the driver.
pub trait Interface {
    /// Transport error.
    type Error;

    /// Read `buf.len()` consecutive registers starting at `reg`.
    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Write one register.
    fn write(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
}

/// I2C transport. Multi-byte reads set the sub-address MSB
/// (auto-increment).
pub struct I2cInterface<I2C> {
    i2c: I2C,
}

/// SPI transport (CS low selects SPI). The HTS221 uses 3-wire SPI with a
/// shared SDI/SDO line, so the [`SpiDevice`](embedded_hal::spi::SpiDevice)
/// must support half-duplex transfers on that pin. The first byte carries
/// the read bit (bit 7) and the auto-increment bit (bit 6).
pub struct SpiInterface<SPI> {
    spi: SPI,
}

const fn i2c_sub_address(reg: u8, len: usize) -> u8 {
    if len > 1 { reg | 0x80 } else { reg }
}

const fn spi_read_command(reg: u8, len: usize) -> u8 {
    0x80 | if len > 1 { 0x40 } else { 0 } | (reg & 0x3F)
}

impl<I2C: embedded_hal::i2c::I2c> Interface for I2cInterface<I2C> {
    type Error = I2C::Error;

    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c
            .write_read(ADDRESS, &[i2c_sub_address(reg, buf.len())], buf)
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(ADDRESS, &[reg, value])
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), SPI::Error> {
        use embedded_hal::spi::Operation;
        self.spi.transaction(&mut [
            Operation::Write(&[spi_read_command(reg, buf.len())]),
            Operation::Read(buf),
        ])
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), SPI::Error> {
        self.spi.write(&[reg & 0x3F, value])
    }
}

/// Number of internal averages (AV_CONF). The temperature uses half the
/// humidity count: index 0 = T 2 / RH 4 ... index 7 = T 256 / RH 512.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Averaging {
    /// T 2, RH 4.
    Avg0 = 0,
    /// T 4, RH 8.
    Avg1 = 1,
    /// T 8, RH 16.
    Avg2 = 2,
    /// T 16, RH 32 (default).
    #[default]
    Avg3 = 3,
    /// T 32, RH 64.
    Avg4 = 4,
    /// T 64, RH 128.
    Avg5 = 5,
    /// T 128, RH 256.
    Avg6 = 6,
    /// T 256, RH 512.
    Avg7 = 7,
}

/// Factory calibration (registers 0x30..=0x3F).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// H0_rH_x2: first humidity point in 0.5 %RH.
    pub h0_rh_x2: u8,
    /// H1_rH_x2: second humidity point in 0.5 %RH.
    pub h1_rh_x2: u8,
    /// T0_degC_x8 (10 bit): first temperature point in 0.125 °C.
    pub t0_degc_x8: u16,
    /// T1_degC_x8 (10 bit): second temperature point in 0.125 °C.
    pub t1_degc_x8: u16,
    /// H0_T0_OUT: raw humidity at the first point.
    pub h0_t0_out: i16,
    /// H1_T0_OUT: raw humidity at the second point.
    pub h1_t0_out: i16,
    /// T0_OUT: raw temperature at the first point.
    pub t0_out: i16,
    /// T1_OUT: raw temperature at the second point.
    pub t1_out: i16,
}

fn div_round(n: i32, d: i32) -> i32 {
    let (n, d) = if d < 0 { (-n, -d) } else { (n, d) };
    if n >= 0 {
        (n + d / 2) / d
    } else {
        (n - d / 2) / d
    }
}

impl Calibration {
    /// Decode the 16 calibration bytes (Table 19).
    pub fn from_registers(c: &[u8; 16]) -> Self {
        Self {
            h0_rh_x2: c[0],
            h1_rh_x2: c[1],
            t0_degc_x8: u16::from(c[2]) | (u16::from(c[5] & 0x03) << 8),
            t1_degc_x8: u16::from(c[3]) | (u16::from(c[5] & 0x0C) << 6),
            h0_t0_out: i16::from_le_bytes([c[6], c[7]]),
            h1_t0_out: i16::from_le_bytes([c[10], c[11]]),
            t0_out: i16::from_le_bytes([c[12], c[13]]),
            t1_out: i16::from_le_bytes([c[14], c[15]]),
        }
    }

    /// Temperature in 0.01 °C for a raw T_OUT; [`Error::InvalidData`] if
    /// the two calibration points coincide.
    pub fn temperature<E>(&self, t_out: i16) -> Result<i32, Error<E>> {
        let span = i32::from(self.t1_out) - i32::from(self.t0_out);
        if span == 0 {
            return Err(Error::InvalidData);
        }
        let t0 = i32::from(self.t0_degc_x8);
        let delta = i32::from(self.t1_degc_x8) - t0;
        // |Δraw| ≤ 65535 and |Δx8| ≤ 1023, so × 25 stays below 2^31.
        let step = (i32::from(t_out) - i32::from(self.t0_out)) * delta;
        Ok(div_round(t0 * 25, 2) + div_round(step * 25, span * 2))
    }

    /// Relative humidity in 0.01 %RH for a raw H_OUT, clipped to
    /// 0..=100 %RH; [`Error::InvalidData`] if the calibration points
    /// coincide.
    pub fn humidity<E>(&self, h_out: i16) -> Result<u16, Error<E>> {
        let span = i32::from(self.h1_t0_out) - i32::from(self.h0_t0_out);
        if span == 0 {
            return Err(Error::InvalidData);
        }
        let h0 = i32::from(self.h0_rh_x2);
        let delta = i32::from(self.h1_rh_x2) - h0;
        let step = (i32::from(h_out) - i32::from(self.h0_t0_out)) * delta;
        let rh = h0 * 50 + div_round(step * 50, span);
        Ok(rh.clamp(0, 10_000) as u16)
    }

    /// Convert both outputs.
    pub fn convert<E>(&self, h_out: i16, t_out: i16) -> Result<Measurement, Error<E>> {
        Ok(Measurement {
            temperature: self.temperature(t_out)?,
            humidity: self.humidity(h_out)?,
        })
    }
}

/// One temperature + humidity sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Relative humidity in 0.01 %RH.
    pub humidity: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
        f(Quantity::Humidity(self.humidity));
    }
}

fn check_id<E>(id: u8) -> Result<(), Error<E>> {
    if id == DEVICE_ID {
        Ok(())
    } else {
        Err(Error::UnexpectedId(u32::from(id)))
    }
}

fn decode_outputs(out: &[u8; 4]) -> (i16, i16) {
    (
        i16::from_le_bytes([out[0], out[1]]),
        i16::from_le_bytes([out[2], out[3]]),
    )
}

const fn av_conf(current: u8, averaging: Averaging) -> u8 {
    let a = averaging as u8;
    (current & 0xC0) | (a << 3) | a
}

macro_rules! constructors {
    () => {
        /// Create a driver over any [`Interface`]; no bus traffic.
        pub const fn new(iface: IF) -> Self {
            Self {
                iface,
                calibration: None,
            }
        }

        /// Use known calibration instead of reading it on first use.
        pub const fn with_calibration(mut self, calibration: Calibration) -> Self {
            self.calibration = Some(calibration);
            self
        }

        /// Give the interface back.
        pub fn release(self) -> IF {
            self.iface
        }
    };
}

/// Blocking driver.
pub struct Hts221<IF> {
    iface: IF,
    calibration: Option<Calibration>,
}

impl<IF> Hts221<IF> {
    constructors!();
}

impl<I2C> Hts221<I2cInterface<I2C>> {
    /// Driver on I2C at [`ADDRESS`].
    pub const fn new_i2c(i2c: I2C) -> Self {
        Self::new(I2cInterface { i2c })
    }
}

impl<SPI> Hts221<SpiInterface<SPI>> {
    /// Driver on 3-wire SPI.
    pub const fn new_spi(spi: SPI) -> Self {
        Self::new(SpiInterface { spi })
    }
}

impl<IF: Interface> Hts221<IF> {
    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IF::Error>> {
        self.iface.read(reg, buf).map_err(Error::bus)
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IF::Error>> {
        self.iface.write(reg, value).map_err(Error::bus)
    }

    /// Check WHO_AM_I (0xBC).
    pub fn verify_id(&mut self) -> Result<(), Error<IF::Error>> {
        let mut id = [0u8; 1];
        self.read(REG_WHO_AM_I, &mut id)?;
        check_id(id[0])
    }

    /// Read (and cache) the calibration registers.
    pub fn calibration(&mut self) -> Result<Calibration, Error<IF::Error>> {
        if let Some(calibration) = self.calibration {
            return Ok(calibration);
        }
        let mut c = [0u8; 16];
        self.read(REG_CALIB_0, &mut c)?;
        let calibration = Calibration::from_registers(&c);
        self.calibration = Some(calibration);
        Ok(calibration)
    }

    /// Verify the ID, read the calibration and power up in one-shot mode
    /// with block data update.
    pub fn init(&mut self) -> Result<(), Error<IF::Error>> {
        self.verify_id()?;
        self.calibration()?;
        self.write(REG_CTRL1, CTRL1_PD | CTRL1_BDU)
    }

    /// Set the number of internal averages (read-modify-write of AV_CONF).
    pub fn set_averaging(&mut self, averaging: Averaging) -> Result<(), Error<IF::Error>> {
        let mut reg = [0u8; 1];
        self.read(REG_AV_CONF, &mut reg)?;
        self.write(REG_AV_CONF, av_conf(reg[0], averaging))
    }

    /// Switch the heater (33 mA at 3.3 V). Do not read outputs while it
    /// is on.
    pub fn set_heater(&mut self, on: bool) -> Result<(), Error<IF::Error>> {
        self.write(REG_CTRL2, if on { CTRL2_HEATER } else { 0 })
    }

    /// Reload the trimming registers from flash (BOOT).
    pub fn reboot(&mut self) -> Result<(), Error<IF::Error>> {
        self.calibration = None;
        self.write(REG_CTRL2, CTRL2_BOOT)
    }
}

impl<IF: Interface> zigbee_sensor::blocking::Sensor for Hts221<IF> {
    type BusError = IF::Error;
    type Measurement = Measurement;

    /// Set ONE_SHOT (heater off). Requires [`Hts221::init`] (PD = 1).
    fn start_measurement(&mut self) -> Result<WaitUs, Error<IF::Error>> {
        self.write(REG_CTRL2, CTRL2_ONE_SHOT)?;
        Ok(ONE_SHOT_WAIT_US)
    }

    /// [`Error::NotReady`] until both T_DA and H_DA are set.
    fn read_measurement(&mut self) -> Result<Measurement, Error<IF::Error>> {
        let calibration = self.calibration()?;
        let mut status = [0u8; 1];
        self.read(REG_STATUS, &mut status)?;
        if status[0] & (STATUS_H_DA | STATUS_T_DA) != STATUS_H_DA | STATUS_T_DA {
            return Err(Error::NotReady);
        }
        let mut out = [0u8; 4];
        self.read(REG_HUMIDITY_OUT_L, &mut out)?;
        let (h, t) = decode_outputs(&out);
        calibration.convert(h, t)
    }
}

impl<IF: Interface> zigbee_sensor::blocking::LowPower for Hts221<IF> {
    type BusError = IF::Error;

    /// Power down (PD = 0).
    fn sleep(&mut self) -> Result<(), Error<IF::Error>> {
        self.write(REG_CTRL1, CTRL1_BDU)
    }

    /// Active mode (PD = 1); the datasheet gives no turn-on time.
    fn wake(&mut self) -> Result<WaitUs, Error<IF::Error>> {
        self.write(REG_CTRL1, CTRL1_PD | CTRL1_BDU)?;
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, Averaging, CTRL1_BDU, CTRL1_PD, CTRL2_BOOT, CTRL2_HEATER, CTRL2_ONE_SHOT,
        Calibration, I2cInterface, Measurement, ONE_SHOT_WAIT_US, REG_AV_CONF, REG_CALIB_0,
        REG_CTRL1, REG_CTRL2, REG_HUMIDITY_OUT_L, REG_STATUS, REG_WHO_AM_I, STATUS_H_DA,
        STATUS_T_DA, SpiInterface, av_conf, check_id, decode_outputs, i2c_sub_address,
        spi_read_command,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async register access; see [`super::Interface`].
    #[allow(async_fn_in_trait)]
    pub trait Interface {
        /// Transport error.
        type Error;

        /// Read `buf.len()` consecutive registers starting at `reg`.
        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

        /// Write one register.
        async fn write(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Interface for I2cInterface<I2C> {
        type Error = I2C::Error;

        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), I2C::Error> {
            self.i2c
                .write_read(ADDRESS, &[i2c_sub_address(reg, buf.len())], buf)
                .await
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
            self.i2c.write(ADDRESS, &[reg, value]).await
        }
    }

    impl<SPI: embedded_hal_async::spi::SpiDevice> Interface for SpiInterface<SPI> {
        type Error = SPI::Error;

        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), SPI::Error> {
            use embedded_hal_async::spi::Operation;
            self.spi
                .transaction(&mut [
                    Operation::Write(&[spi_read_command(reg, buf.len())]),
                    Operation::Read(buf),
                ])
                .await
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), SPI::Error> {
            self.spi.write(&[reg & 0x3F, value]).await
        }
    }

    /// Async twin of [`super::Hts221`].
    pub struct Hts221<IF> {
        iface: IF,
        calibration: Option<Calibration>,
    }

    impl<IF> Hts221<IF> {
        constructors!();
    }

    impl<I2C> Hts221<I2cInterface<I2C>> {
        /// Driver on I2C at [`ADDRESS`].
        pub const fn new_i2c(i2c: I2C) -> Self {
            Self::new(I2cInterface { i2c })
        }
    }

    impl<SPI> Hts221<SpiInterface<SPI>> {
        /// Driver on 3-wire SPI.
        pub const fn new_spi(spi: SPI) -> Self {
            Self::new(SpiInterface { spi })
        }
    }

    impl<IF: Interface> Hts221<IF> {
        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IF::Error>> {
            self.iface.read(reg, buf).await.map_err(Error::bus)
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IF::Error>> {
            self.iface.write(reg, value).await.map_err(Error::bus)
        }

        /// See [`super::Hts221::verify_id`].
        pub async fn verify_id(&mut self) -> Result<(), Error<IF::Error>> {
            let mut id = [0u8; 1];
            self.read(REG_WHO_AM_I, &mut id).await?;
            check_id(id[0])
        }

        /// See [`super::Hts221::calibration`].
        pub async fn calibration(&mut self) -> Result<Calibration, Error<IF::Error>> {
            if let Some(calibration) = self.calibration {
                return Ok(calibration);
            }
            let mut c = [0u8; 16];
            self.read(REG_CALIB_0, &mut c).await?;
            let calibration = Calibration::from_registers(&c);
            self.calibration = Some(calibration);
            Ok(calibration)
        }

        /// See [`super::Hts221::init`].
        pub async fn init(&mut self) -> Result<(), Error<IF::Error>> {
            self.verify_id().await?;
            self.calibration().await?;
            self.write(REG_CTRL1, CTRL1_PD | CTRL1_BDU).await
        }

        /// See [`super::Hts221::set_averaging`].
        pub async fn set_averaging(
            &mut self,
            averaging: Averaging,
        ) -> Result<(), Error<IF::Error>> {
            let mut reg = [0u8; 1];
            self.read(REG_AV_CONF, &mut reg).await?;
            self.write(REG_AV_CONF, av_conf(reg[0], averaging)).await
        }

        /// See [`super::Hts221::set_heater`].
        pub async fn set_heater(&mut self, on: bool) -> Result<(), Error<IF::Error>> {
            self.write(REG_CTRL2, if on { CTRL2_HEATER } else { 0 })
                .await
        }

        /// See [`super::Hts221::reboot`].
        pub async fn reboot(&mut self) -> Result<(), Error<IF::Error>> {
            self.calibration = None;
            self.write(REG_CTRL2, CTRL2_BOOT).await
        }
    }

    impl<IF: Interface> zigbee_sensor::asynch::Sensor for Hts221<IF> {
        type BusError = IF::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IF::Error>> {
            self.write(REG_CTRL2, CTRL2_ONE_SHOT).await?;
            Ok(ONE_SHOT_WAIT_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IF::Error>> {
            let calibration = self.calibration().await?;
            let mut status = [0u8; 1];
            self.read(REG_STATUS, &mut status).await?;
            if status[0] & (STATUS_H_DA | STATUS_T_DA) != STATUS_H_DA | STATUS_T_DA {
                return Err(Error::NotReady);
            }
            let mut out = [0u8; 4];
            self.read(REG_HUMIDITY_OUT_L, &mut out).await?;
            let (h, t) = decode_outputs(&out);
            calibration.convert(h, t)
        }
    }

    impl<IF: Interface> zigbee_sensor::asynch::LowPower for Hts221<IF> {
        type BusError = IF::Error;

        async fn sleep(&mut self) -> Result<(), Error<IF::Error>> {
            self.write(REG_CTRL1, CTRL1_BDU).await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<IF::Error>> {
            self.write(REG_CTRL1, CTRL1_PD | CTRL1_BDU).await?;
            Ok(0)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    // Calibration points of datasheet Figures 9 and 10: T0 = 10 °C at 300,
    // T1 = 20 °C at 500; H0 = 20 %RH at 0x4000, H1 = 40 %RH at 0x6000.
    const CALIB: [u8; 16] = [
        0x28, 0x50, 0x50, 0xA0, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x2C, 0x01, 0xF4,
        0x01,
    ];
    // H_OUT = 0x5000, T_OUT = 400.
    const OUTPUTS: [u8; 4] = [0x00, 0x50, 0x90, 0x01];
    const EXPECTED: Measurement = Measurement {
        temperature: 1_500,
        humidity: 3_000,
    };

    #[test]
    fn datasheet_interpolation_examples() {
        let c = Calibration::from_registers(&CALIB);
        // Figure 9: T_OUT = 400 → 15.0 °C; Figure 10: H_OUT = 0x5000 → 30 %RH.
        assert_eq!(c.convert::<()>(0x5000, 400), Ok(EXPECTED));
        assert_eq!(c.temperature::<()>(300), Ok(1_000));
        assert_eq!(c.temperature::<()>(100), Ok(0));
        assert_eq!(c.temperature::<()>(301), Ok(1_005));
        // Clipping below 0 and above 100 %RH.
        assert_eq!(c.humidity::<()>(0), Ok(0));
        assert_eq!(c.humidity::<()>(i16::MAX), Ok(6_000));
        let mut steep = CALIB;
        steep[1] = 200;
        assert_eq!(
            Calibration::from_registers(&steep).humidity::<()>(i16::MAX),
            Ok(10_000)
        );
        // 10-bit T0/T1: MSBs in register 0x35 (T0 bits 1:0, T1 bits 3:2).
        let mut wide = CALIB;
        wide[5] = 0x05;
        let w = Calibration::from_registers(&wide);
        assert_eq!((w.t0_degc_x8, w.t1_degc_x8), (0x150, 0x1A0));
        let mut flat = CALIB;
        flat[14] = 0x2C;
        flat[15] = 0x01;
        assert_eq!(
            Calibration::from_registers(&flat).temperature::<()>(0),
            Err(Error::InvalidData)
        );
    }

    #[test]
    fn i2c_init_one_shot_and_power_down() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x0F], vec![0xBC]),
            Transaction::write_read(ADDRESS, vec![0xB0], CALIB.to_vec()),
            Transaction::write(ADDRESS, vec![0x20, 0x84]),
            Transaction::write(ADDRESS, vec![0x21, 0x01]),
            Transaction::write_read(ADDRESS, vec![0x27], vec![0x01]),
            Transaction::write_read(ADDRESS, vec![0x27], vec![0x03]),
            Transaction::write_read(ADDRESS, vec![0xA8], OUTPUTS.to_vec()),
            Transaction::write_read(ADDRESS, vec![0x10], vec![0x1B]),
            Transaction::write(ADDRESS, vec![0x10, 0x00]),
            Transaction::write(ADDRESS, vec![0x21, 0x02]),
            Transaction::write(ADDRESS, vec![0x20, 0x04]),
            Transaction::write(ADDRESS, vec![0x20, 0x84]),
            Transaction::write_read(ADDRESS, vec![0x0F], vec![0xBD]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hts = Hts221::new_i2c(&mut i2c);
        hts.init().unwrap();
        assert_eq!(hts.start_measurement(), Ok(ONE_SHOT_WAIT_US));
        assert_eq!(hts.read_measurement(), Err(Error::NotReady));
        assert_eq!(hts.read_measurement(), Ok(EXPECTED));
        hts.set_averaging(Averaging::Avg0).unwrap();
        hts.set_heater(true).unwrap();
        hts.sleep().unwrap();
        assert_eq!(hts.wake(), Ok(0));
        assert_eq!(hts.verify_id(), Err(Error::UnexpectedId(0xBD)));
        i2c.done();
    }

    #[test]
    fn spi_read_and_write_framing() {
        use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(0x8F),
            SpiTransaction::read(0xBC),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(0xF0),
            SpiTransaction::read_vec(CALIB.to_vec()),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x20, 0x84]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut hts = Hts221::new_spi(&mut spi);
        hts.init().unwrap();
        spi.done();
    }

    #[test]
    fn async_measure_with_known_calibration() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(ADDRESS, vec![0x21, 0x01]),
            Transaction::write_read(ADDRESS, vec![0x27], vec![0x03]),
            Transaction::write_read(ADDRESS, vec![0xA8], OUTPUTS.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hts =
            asynch::Hts221::new_i2c(&mut i2c).with_calibration(Calibration::from_registers(&CALIB));
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(hts.measure(&mut delay)), Ok(EXPECTED));
        assert_eq!(delay.0, ONE_SHOT_WAIT_US);
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
