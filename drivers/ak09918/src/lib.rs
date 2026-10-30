//! AKM AK09918 3-axis Hall-effect magnetometer over I2C.
//!
//! The part sits in power-down (≈1 µA); every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! starts one single measurement (≤8.2 ms) after which the chip powers
//! down again by itself. Optional continuous modes run at 10/20/50/100 Hz.
//! There is no interrupt pin and no threshold interrupt.
//!
//! The field is reported as [`Quantity::MagneticField`] in nT
//! (0.15 µT/LSB, ±4912 µT, factory sensitivity-adjusted).
//!
//! Reference: AKM "AK09918 3-axis Electronic Compass" datasheet
//! 016014242-E-00 (2016/11).
//!
//! ```ignore
//! let mut mag = Ak09918::new(i2c);
//! mag.probe()?;
//! let wait = mag.start_measurement()?;
//! delay_us(wait);
//! let m = mag.read_measurement()?; // m.field in nT
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x0C;

/// `WIA1`: AKM company ID.
pub const COMPANY_ID: u8 = 0x48;

/// `WIA2`: AK09918 device ID.
pub const DEVICE_ID: u8 = 0x0C;

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const WIA1: u8 = 0x00;
    pub const ST1: u8 = 0x10;
    pub const HXL: u8 = 0x11;
    pub const ST2: u8 = 0x18;
    pub const CNTL2: u8 = 0x31;
    pub const CNTL3: u8 = 0x32;
}

const MODE_POWER_DOWN: u8 = 0x00;
const MODE_SINGLE: u8 = 0x01;
const MODE_SELF_TEST: u8 = 0x10;
const ST1_DRDY: u8 = 0x01;
const ST2_HOFL: u8 = 0x08;
const CNTL3_SRST: u8 = 0x01;

/// Single measurement time, maximum (`TSM`).
pub const MEASUREMENT_US: WaitUs = 8_200;

/// Minimum time in power-down before another mode may be set (`Twait`);
/// also used after a soft reset (POR completion time is 100 µs).
pub const TWAIT_US: WaitUs = 100;

/// Sensitivity: 0.15 µT/LSB = 150 nT/LSB.
pub const NT_PER_LSB: i32 = 150;

/// Continuous measurement rate (`CNTL2.MODE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 10 Hz (continuous mode 1).
    #[default]
    Hz10 = 0x02,
    /// 20 Hz (continuous mode 2).
    Hz20 = 0x04,
    /// 50 Hz (continuous mode 3).
    Hz50 = 0x06,
    /// 100 Hz (continuous mode 4).
    Hz100 = 0x08,
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Measure continuously at this rate instead of single measurements
    /// (default `None`).
    pub continuous: Option<DataRate>,
}

impl Config {
    /// `CNTL2` value that starts a measurement.
    pub fn mode(&self) -> u8 {
        self.continuous.map_or(MODE_SINGLE, |r| r as u8)
    }
}

/// One magnetometer sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Flux density in nT.
    pub field: Vector3,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::MagneticField(self.field));
    }
}

/// Decode `HXL`..`ST2` (8 bytes: X, Y, Z little-endian, TMPS, ST2) to raw
/// counts. [`Error::InvalidData`] on magnetic sensor overflow
/// (|X| + |Y| + |Z| ≥ 4912 µT).
pub fn decode_raw<E>(b: &[u8; 8]) -> Result<[i16; 3], Error<E>> {
    if b[7] & ST2_HOFL != 0 {
        return Err(Error::InvalidData);
    }
    let axis = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
    Ok([axis(0), axis(2), axis(4)])
}

/// Decode `HXL`..`ST2` to nT.
pub fn decode<E>(b: &[u8; 8]) -> Result<Measurement, Error<E>> {
    let [x, y, z] = decode_raw(b)?;
    Ok(Measurement {
        field: Vector3 {
            x: i32::from(x) * NT_PER_LSB,
            y: i32::from(y) * NT_PER_LSB,
            z: i32::from(z) * NT_PER_LSB,
        },
    })
}

/// Self-test judgment (datasheet 9.4.4.2) on raw counts.
pub fn self_test_passed(raw: [i16; 3]) -> bool {
    let [x, y, z] = raw;
    (-200..=200).contains(&x) && (-200..=200).contains(&y) && (-1000..=-150).contains(&z)
}

macro_rules! driver {
    ($doc:literal, $I2c:path, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Ak09918<I2C> {
            i2c: I2C,
            config: Config,
            pending: bool,
        }

        impl<I2C> Ak09918<I2C> {
            /// Create a driver at [`ADDRESS`]; no bus traffic.
            pub fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    config: Config::default(),
                    pending: false,
                }
            }

            /// Set the configuration used by `configure()`.
            pub fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self
            }

            /// Current configuration.
            pub fn config(&self) -> &Config {
                &self.config
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: $I2c> Ak09918<I2C> {
            /// Read consecutive registers.
            pub $($async)? fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
                self.i2c.write_read(ADDRESS, &[reg], buf) $($aw)* .map_err(Error::bus)
            }

            /// Write one register.
            pub $($async)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c.write(ADDRESS, &[reg, value]) $($aw)* .map_err(Error::bus)
            }

            /// Check `WIA1`/`WIA2` (0x48, 0x0C).
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let mut id = [0u8; 2];
                self.read_registers(reg::WIA1, &mut id) $($aw)* ?;
                if id != [COMPANY_ID, DEVICE_ID] {
                    return Err(Error::UnexpectedId(u32::from(u16::from_be_bytes(id))));
                }
                Ok(())
            }

            /// Soft reset (`CNTL3.SRST`): registers to default, power-down.
            pub $($async)? fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.pending = false;
                self.write_register(reg::CNTL3, CNTL3_SRST) $($aw)* ?;
                Ok(TWAIT_US)
            }

            /// Enter power-down so the configuration can change. In
            /// continuous mode the next `start_measurement()` starts it;
            /// the returned [`TWAIT_US`] must pass before that.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(reg::CNTL2, MODE_POWER_DOWN) $($aw)* ?;
                self.pending = self.config.continuous.is_some();
                Ok(TWAIT_US)
            }

            /// Store a new configuration and apply it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I2C::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Power-down mode (≈1 µA); stops continuous mode until
            /// `wake()`. Wait [`TWAIT_US`] before the next measurement.
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_register(reg::CNTL2, MODE_POWER_DOWN) $($aw)*
            }

            /// Start self-test (internal field source, auto power-down
            /// afterwards). The chip must be in power-down for
            /// [`TWAIT_US`]; read the result with `read_self_test()`.
            pub $($async)? fn start_self_test(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(reg::CNTL2, MODE_SELF_TEST) $($aw)* ?;
                Ok(MEASUREMENT_US)
            }

            /// `true` if the self-test output is within the datasheet
            /// limits; [`Error::NotReady`] before data ready.
            pub $($async)? fn read_self_test(&mut self) -> Result<bool, Error<I2C::Error>> {
                let b = self.read_data() $($aw)* ?;
                Ok(self_test_passed(decode_raw(&b)?))
            }

            /// Read `ST1`, then `HXL`..`ST2` if data is ready. Reading
            /// `ST2` last releases the data protection.
            $($async)? fn read_data(&mut self) -> Result<[u8; 8], Error<I2C::Error>> {
                let mut st1 = [0u8];
                self.read_registers(reg::ST1, &mut st1) $($aw)* ?;
                if st1[0] & ST1_DRDY == 0 {
                    return Err(Error::NotReady);
                }
                let mut b = [0u8; 8];
                self.read_registers(reg::HXL, &mut b) $($aw)* ?;
                Ok(b)
            }
        }

        impl<I2C: $I2c> zigbee_sensor::$mode::Sensor for Ak09918<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Single mode: start one measurement (chip must be in
            /// power-down, i.e. the previous one finished) and return
            /// [`MEASUREMENT_US`]. Continuous mode: start it after
            /// `configure()`/`wake()`, else 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.config.continuous.is_some() && !self.pending {
                    return Ok(0);
                }
                self.write_register(reg::CNTL2, self.config.mode()) $($aw)* ?;
                self.pending = false;
                Ok(MEASUREMENT_US)
            }

            /// [`Error::NotReady`] until `ST1.DRDY`;
            /// [`Error::InvalidData`] on magnetic sensor overflow.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let b = self.read_data() $($aw)* ?;
                decode(&b)
            }
        }

        impl<I2C: $I2c> zigbee_sensor::$mode::LowPower for Ak09918<I2C> {
            type BusError = I2C::Error;

            /// [`Self::power_down`]; single mode is already powered down
            /// between measurements.
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.power_down() $($aw)*
            }

            /// No bus traffic: continuous mode restarts on the next
            /// `start_measurement()`. Returns [`TWAIT_US`].
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.pending = self.config.continuous.is_some();
                Ok(TWAIT_US)
            }
        }
    };
}

driver!("Blocking driver.", embedded_hal::i2c::I2c, blocking, [], []);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        "Async twin of [`super::Ak09918`].",
        embedded_hal_async::i2c::I2c,
        asynch,
        [async],
        [.await]
    );
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as I2cTx};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = ADDRESS;

    fn rd(reg: u8, data: &[u8]) -> I2cTx {
        I2cTx::write_read(A, vec![reg], data.to_vec())
    }
    fn wr(reg: u8, value: u8) -> I2cTx {
        I2cTx::write(A, vec![reg, value])
    }

    // X = +32752 (4912 µT), Y = -1, Z = -32752; TMPS dummy, ST2 normal.
    const FRAME: [u8; 8] = [0xF0, 0x7F, 0xFF, 0xFF, 0x10, 0x80, 0x00, 0x00];

    #[test]
    fn datasheet_data_format() {
        let m = decode::<()>(&FRAME).unwrap();
        assert_eq!(
            m.field,
            Vector3 {
                x: 4_912_800,
                y: -150,
                z: -4_912_800
            }
        );
        let mut ovf = FRAME;
        ovf[7] = ST2_HOFL;
        assert_eq!(decode::<()>(&ovf), Err(Error::InvalidData));
        assert!(self_test_passed([200, -200, -150]));
        assert!(!self_test_passed([0, 0, -149]));
        assert!(!self_test_passed([201, 0, -500]));
    }

    #[test]
    fn probe_and_reset() {
        let mut i2c = Mock::new(&[
            rd(0x00, &[0x48, 0x0C]),
            rd(0x00, &[0x48, 0x09]),
            wr(0x32, 0x01),
        ]);
        let mut mag = Ak09918::new(&mut i2c);
        mag.probe().unwrap();
        assert_eq!(mag.probe(), Err(Error::UnexpectedId(0x4809)));
        assert_eq!(mag.soft_reset().unwrap(), 100);
        i2c.done();
    }

    #[test]
    fn single_measurement_and_self_test() {
        let mut i2c = Mock::new(&[
            wr(0x31, 0x01),
            rd(0x10, &[0x00]),
            rd(0x10, &[0x01]),
            rd(0x11, &FRAME),
            wr(0x31, 0x10),
            rd(0x10, &[0x01]),
            // X = 10, Y = -20, Z = -400
            rd(0x11, &[0x0A, 0x00, 0xEC, 0xFF, 0x70, 0xFE, 0x00, 0x00]),
            wr(0x31, 0x00),
        ]);
        let mut mag = Ak09918::new(&mut i2c);
        assert_eq!(mag.start_measurement().unwrap(), 8_200);
        assert_eq!(mag.read_measurement(), Err(Error::NotReady));
        assert_eq!(mag.read_measurement().unwrap().field.y, -150);
        assert_eq!(mag.start_self_test().unwrap(), 8_200);
        assert!(mag.read_self_test().unwrap());
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 100);
        i2c.done();
    }

    #[test]
    fn continuous_mode_power_down_first() {
        let cfg = Config {
            continuous: Some(DataRate::Hz100),
        };
        let mut i2c = Mock::new(&[
            wr(0x31, 0x00),
            wr(0x31, 0x08),
            rd(0x10, &[0x03]),
            rd(0x11, &FRAME),
            wr(0x31, 0x00),
            wr(0x31, 0x08),
        ]);
        let mut mag = Ak09918::new(&mut i2c).with_config(cfg);
        assert_eq!(mag.configure().unwrap(), 100);
        assert_eq!(mag.start_measurement().unwrap(), 8_200);
        assert_eq!(mag.read_measurement().unwrap().field.x, 4_912_800);
        assert_eq!(mag.start_measurement().unwrap(), 0);
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 100);
        assert_eq!(mag.start_measurement().unwrap(), 8_200);
        i2c.done();
    }

    #[test]
    fn async_single_measurement() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut i2c = Mock::new(&[wr(0x31, 0x01), rd(0x10, &[0x01]), rd(0x11, &FRAME)]);
        let mut mag = asynch::Ak09918::new(&mut i2c);
        assert_eq!(block_on(mag.start_measurement()).unwrap(), 8_200);
        assert_eq!(
            block_on(mag.read_measurement()).unwrap().field.z,
            -4_912_800
        );
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
