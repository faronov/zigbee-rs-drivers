//! HopeRF HP206C and HP203B barometric pressure, altitude and temperature
//! sensors (I2C).
//!
//! Both parts compute compensated results on-chip: after a conversion the
//! host reads 20-bit two's-complement values that are already in
//! 0.01 °C and 0.01 mbar (= Pa), so the driver does no maths beyond sign
//! extension. The parts are command based: [`start_measurement`] sends
//! `ADC_CVT` for the temperature + pressure channel at the configured
//! [`Osr`] and returns the datasheet conversion time; the part returns to
//! its sleep state (0.1 µA) by itself when the conversion ends, so there is
//! no sleep command. [`read_measurement`] first checks the `PA_RDY` and
//! `T_RDY` flags in `INT_SRC` and returns [`Error::NotReady`] if the
//! conversion has not finished (or none was started), then reads both
//! results with `READ_PT`, which also clears the flags.
//!
//! There is no ID register. On-chip compensation (`PARA.CMPS_EN`) is on
//! after power-up and soft reset and the driver never changes it.
//!
//! Reference: HopeRF HP206C datasheet V2.0 and HP203B datasheet V2.0.
//!
//! [`start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// HP206C address; HP203B with CSB high.
pub const ADDRESS: u8 = 0x76;
/// HP203B with CSB low.
pub const ADDRESS_CSB_LOW: u8 = 0x77;

/// Wait after soft reset or power-up. The datasheet gives about 400 µs for
/// the power-up sequence; `is_ready` reports the `DEV_RDY` flag.
pub const RESET_WAIT_US: WaitUs = 1_000;

const CMD_SOFT_RST: u8 = 0x06;
const CMD_ADC_CVT: u8 = 0x40;
const CMD_READ_PT: u8 = 0x10;
const CMD_READ_REG: u8 = 0x80;
const REG_INT_SRC: u8 = 0x0D;

const DEV_RDY: u8 = 1 << 6;
const PA_RDY: u8 = 1 << 5;
const T_RDY: u8 = 1 << 4;

/// Down-sampling (oversampling) rate of `ADC_CVT`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Osr {
    /// 131.1 ms.
    X4096,
    /// 65.6 ms.
    X2048,
    /// 32.8 ms (driver default).
    #[default]
    X1024,
    /// 16.4 ms.
    X512,
    /// 8.2 ms.
    X256,
    /// 4.1 ms.
    X128,
}

impl Osr {
    /// `ADC_CVT` for temperature + pressure (`CHNL = 00`).
    const fn command(self) -> u8 {
        CMD_ADC_CVT | (self as u8) << 2
    }

    /// Temperature + pressure conversion time (datasheet table 6).
    pub const fn conversion_us(self) -> WaitUs {
        match self {
            Self::X4096 => 131_100,
            Self::X2048 => 65_600,
            Self::X1024 => 32_800,
            Self::X512 => 16_400,
            Self::X256 => 8_200,
            Self::X128 => 4_100,
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

/// Sign-extend a 20-bit value stored in 3 bytes (upper 4 bits unused).
fn s20(b: &[u8]) -> i32 {
    let v = i32::from(b[0]) << 16 | i32::from(b[1]) << 8 | i32::from(b[2]);
    (v << 12) >> 12
}

/// Decode a `READ_PT` result (temperature then pressure, MSB first).
fn decode(raw: &[u8; 6]) -> Measurement {
    Measurement {
        temperature: s20(&raw[..3]),
        pressure: s20(&raw[3..]).max(0).unsigned_abs(),
    }
}

macro_rules! common {
    ($name:ident) => {
        impl<I2C> $name<I2C> {
            /// Create a driver; no bus traffic. Use [`crate::ADDRESS`] or
            /// [`crate::ADDRESS_CSB_LOW`].
            pub const fn new(i2c: I2C, address: u8) -> Self {
                Self {
                    i2c,
                    address,
                    osr: Osr::X1024,
                }
            }

            /// Select the oversampling rate.
            pub const fn with_osr(mut self, osr: Osr) -> Self {
                self.osr = osr;
                self
            }

            /// Change the oversampling rate (applies to the next start).
            pub fn set_osr(&mut self, osr: Osr) {
                self.osr = osr;
            }

            /// Current oversampling rate.
            pub const fn osr(&self) -> Osr {
                self.osr
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Hp206c<I2C> {
    i2c: I2C,
    address: u8,
    osr: Osr,
}

common!(Hp206c);

impl<I2C: embedded_hal::i2c::I2c> Hp206c<I2C> {
    fn int_src(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut b = [0];
        self.i2c
            .write_read(self.address, &[CMD_READ_REG | REG_INT_SRC], &mut b)
            .map_err(Error::bus)?;
        Ok(b[0])
    }

    /// Soft reset (registers to defaults, then sleep). Wait
    /// [`RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[CMD_SOFT_RST])
            .map_err(Error::bus)?;
        Ok(RESET_WAIT_US)
    }

    /// `DEV_RDY`: the part is idle and accepts commands.
    pub fn is_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok(self.int_src()? & DEV_RDY != 0)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Hp206c<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[self.osr.command()])
            .map_err(Error::bus)?;
        Ok(self.osr.conversion_us())
    }

    /// [`Error::NotReady`] unless both `PA_RDY` and `T_RDY` are set.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        if self.int_src()? & (PA_RDY | T_RDY) != PA_RDY | T_RDY {
            return Err(Error::NotReady);
        }
        let mut raw = [0; 6];
        self.i2c
            .write_read(self.address, &[CMD_READ_PT], &mut raw)
            .map_err(Error::bus)?;
        Ok(decode(&raw))
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Hp206c<I2C> {
    type BusError = I2C::Error;

    /// The part sleeps by itself after each conversion; nothing to send.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        Ok(())
    }

    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_READ_PT, CMD_READ_REG, CMD_SOFT_RST, DEV_RDY, Measurement, Osr, PA_RDY, REG_INT_SRC,
        T_RDY, decode,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Hp206c`].
    #[derive(Debug)]
    pub struct Hp206c<I2C> {
        i2c: I2C,
        address: u8,
        osr: Osr,
    }

    common!(Hp206c);

    impl<I2C: I2c> Hp206c<I2C> {
        async fn int_src(&mut self) -> Result<u8, Error<I2C::Error>> {
            let mut b = [0];
            self.i2c
                .write_read(self.address, &[CMD_READ_REG | REG_INT_SRC], &mut b)
                .await
                .map_err(Error::bus)?;
            Ok(b[0])
        }

        /// See [`super::Hp206c::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[CMD_SOFT_RST])
                .await
                .map_err(Error::bus)?;
            Ok(super::RESET_WAIT_US)
        }

        /// See [`super::Hp206c::is_ready`].
        pub async fn is_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
            Ok(self.int_src().await? & DEV_RDY != 0)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Hp206c<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[self.osr.command()])
                .await
                .map_err(Error::bus)?;
            Ok(self.osr.conversion_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if self.int_src().await? & (PA_RDY | T_RDY) != PA_RDY | T_RDY {
                return Err(Error::NotReady);
            }
            let mut raw = [0; 6];
            self.i2c
                .write_read(self.address, &[CMD_READ_PT], &mut raw)
                .await
                .map_err(Error::bus)?;
            Ok(decode(&raw))
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Hp206c<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            Ok(())
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            Ok(0)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    #[test]
    fn decode_datasheet_examples() {
        // Temperature 26.52 °C, pressure 1010.22 mbar.
        let m = decode(&[0x00, 0x0A, 0x5C, 0x01, 0x8A, 0x9E]);
        assert_eq!(m.temperature, 2_652);
        assert_eq!(m.pressure, 101_022);
        // -10.22 °C; unused upper nibble ignored; negative pressure clamps.
        let m = decode(&[0xFF, 0xFC, 0x02, 0xFF, 0xFF, 0xFF]);
        assert_eq!(m.temperature, -1_022);
        assert_eq!(m.pressure, 0);
        assert_eq!(decode(&[0xF0, 0x0A, 0x5C, 0, 0, 0]).temperature, 2_652);
    }

    #[test]
    fn osr_commands() {
        assert_eq!(Osr::X4096.command(), 0x40);
        assert_eq!(Osr::X1024.command(), 0x48);
        assert_eq!(Osr::X256.command(), 0x50);
        assert_eq!(Osr::X128.command(), 0x54);
    }

    #[test]
    fn i2c_cycle() {
        let ex = [
            I2cTx::write(ADDRESS_CSB_LOW, vec![0x06]),
            I2cTx::write_read(ADDRESS_CSB_LOW, vec![0x8D], vec![0x40]),
            I2cTx::write_read(ADDRESS_CSB_LOW, vec![0x8D], vec![0x40]),
            I2cTx::write(ADDRESS_CSB_LOW, vec![0x50]),
            I2cTx::write_read(ADDRESS_CSB_LOW, vec![0x8D], vec![0x10]),
            I2cTx::write_read(ADDRESS_CSB_LOW, vec![0x8D], vec![0x70]),
            I2cTx::write_read(
                ADDRESS_CSB_LOW,
                vec![0x10],
                vec![0x00, 0x0A, 0x5C, 0x01, 0x8A, 0x9E],
            ),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut hp = Hp206c::new(&mut i2c, ADDRESS_CSB_LOW).with_osr(Osr::X256);
        assert_eq!(hp.soft_reset().unwrap(), RESET_WAIT_US);
        assert!(hp.is_ready().unwrap());
        // Nothing converted yet.
        assert_eq!(hp.read_measurement(), Err(Error::NotReady));
        assert_eq!(hp.start_measurement().unwrap(), 8_200);
        // Only T_RDY: pressure still converting.
        assert_eq!(hp.read_measurement(), Err(Error::NotReady));
        let m = hp.read_measurement().unwrap();
        assert_eq!((m.temperature, m.pressure), (2_652, 101_022));
        hp.sleep().unwrap();
        assert_eq!(hp.wake().unwrap(), 0);
        i2c.done();
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

        let ex = [
            I2cTx::write(ADDRESS, vec![0x48]),
            I2cTx::write_read(ADDRESS, vec![0x8D], vec![0x70]),
            I2cTx::write_read(
                ADDRESS,
                vec![0x10],
                vec![0x0F, 0xFC, 0x02, 0x01, 0x8A, 0x9E],
            ),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut hp = asynch::Hp206c::new(&mut i2c, ADDRESS);
        let mut delay = RecordingDelay(0);
        let m = block_on(hp.measure(&mut delay)).unwrap();
        assert_eq!((m.temperature, m.pressure), (-1_022, 101_022));
        assert_eq!(delay.0, 32_800);
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
