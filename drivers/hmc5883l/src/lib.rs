//! Honeywell HMC5883L 3-axis magnetometer over I2C.
//!
//! The part idles at ≈2 µA; every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! triggers one single measurement (with the automatic set/reset offset
//! cancellation of the ASIC) and returns the measurement period. The
//! optional continuous mode runs at a fixed [`DataRate`]. The DRDY pin
//! signals data ready; there is no threshold interrupt.
//!
//! The field is reported as [`Quantity::MagneticField`] in nT for the
//! selected [`Gain`] (12-bit output, ±0.88 to ±8.1 gauss).
//!
//! Reference: Honeywell "3-Axis Digital Compass IC HMC5883L" datasheet
//! (Advanced Information, Rev. E, 2013).
//!
//! ```ignore
//! let mut mag = Hmc5883l::new(i2c);
//! mag.probe()?;
//! let wait = mag.configure()?; // primes the new gain
//! delay_us(wait);
//! let wait = mag.start_measurement()?;
//! delay_us(wait);
//! let m = mag.read_measurement()?; // m.field in nT
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x1E;

/// Identification registers A–C: ASCII "H43".
pub const ID: [u8; 3] = *b"H43";

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const CONFIG_A: u8 = 0x00;
    pub const CONFIG_B: u8 = 0x01;
    pub const MODE: u8 = 0x02;
    pub const DATA_X_MSB: u8 = 0x03;
    pub const STATUS: u8 = 0x09;
    pub const ID_A: u8 = 0x0A;
}

const MODE_CONTINUOUS: u8 = 0x00;
const MODE_SINGLE: u8 = 0x01;
const MODE_IDLE: u8 = 0x02;
const STATUS_RDY: u8 = 0x01;

/// Output value of an axis after an ADC overflow.
pub const OVERFLOW: i16 = -4096;

/// Measurement period in single mode without averaging (datasheet: 6 ms
/// from command to data ready).
pub const MEASUREMENT_US: WaitUs = 6_000;

/// Gain / range (`CRB.GN`). The first measurement after a change still
/// uses the previous gain; `configure()` takes care of that.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gain {
    /// ±0.88 gauss, 1370 LSB/gauss.
    Ga0_88 = 0,
    /// ±1.3 gauss, 1090 LSB/gauss (reset value).
    #[default]
    Ga1_3 = 1,
    /// ±1.9 gauss, 820 LSB/gauss.
    Ga1_9 = 2,
    /// ±2.5 gauss, 660 LSB/gauss.
    Ga2_5 = 3,
    /// ±4.0 gauss, 440 LSB/gauss.
    Ga4_0 = 4,
    /// ±4.7 gauss, 390 LSB/gauss.
    Ga4_7 = 5,
    /// ±5.6 gauss, 330 LSB/gauss.
    Ga5_6 = 6,
    /// ±8.1 gauss, 230 LSB/gauss.
    Ga8_1 = 7,
}

impl Gain {
    /// Sensitivity in LSB/gauss (datasheet Table 9).
    pub const fn lsb_per_gauss(self) -> i32 {
        match self {
            Self::Ga0_88 => 1370,
            Self::Ga1_3 => 1090,
            Self::Ga1_9 => 820,
            Self::Ga2_5 => 660,
            Self::Ga4_0 => 440,
            Self::Ga4_7 => 390,
            Self::Ga5_6 => 330,
            Self::Ga8_1 => 230,
        }
    }
}

/// Samples averaged per output (`CRA.MA`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Averaging {
    /// 1 sample (reset value).
    #[default]
    X1 = 0,
    /// 2 samples.
    X2 = 1,
    /// 4 samples.
    X4 = 2,
    /// 8 samples.
    X8 = 3,
}

impl Averaging {
    /// Number of samples.
    pub const fn samples(self) -> u32 {
        1 << self as u32
    }
}

/// Continuous-mode output rate (`CRA.DO`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 0.75 Hz.
    Hz0_75 = 0,
    /// 1.5 Hz.
    Hz1_5 = 1,
    /// 3 Hz.
    Hz3 = 2,
    /// 7.5 Hz.
    Hz7_5 = 3,
    /// 15 Hz (reset value).
    #[default]
    Hz15 = 4,
    /// 30 Hz.
    Hz30 = 5,
    /// 75 Hz.
    Hz75 = 6,
}

impl DataRate {
    /// Output period, rounded up.
    pub const fn period_us(self) -> WaitUs {
        match self {
            Self::Hz0_75 => 1_333_334,
            Self::Hz1_5 => 666_667,
            Self::Hz3 => 333_334,
            Self::Hz7_5 => 133_334,
            Self::Hz15 => 66_667,
            Self::Hz30 => 33_334,
            Self::Hz75 => 13_334,
        }
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Gain / range (default ±1.3 gauss).
    pub gain: Gain,
    /// Samples averaged per output (default 1).
    pub averaging: Averaging,
    /// Run continuously at this rate instead of single measurements
    /// (default `None`).
    pub continuous: Option<DataRate>,
}

impl Config {
    /// Configuration register A value (normal measurement, no bias).
    pub fn config_a(&self) -> u8 {
        let rate = self.continuous.unwrap_or_default();
        (self.averaging as u8) << 5 | (rate as u8) << 2
    }

    /// Configuration register B value.
    pub fn config_b(&self) -> u8 {
        (self.gain as u8) << 5
    }

    /// Single-measurement period: 6 ms per averaged sample. The datasheet
    /// only specifies the 1-sample case; scaling is conservative.
    pub fn measurement_us(&self) -> WaitUs {
        MEASUREMENT_US * self.averaging.samples()
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

/// Output count to nT (1 gauss = 100 000 nT), rounded to nearest.
pub const fn raw_to_nt(raw: i16, gain: Gain) -> i32 {
    let g = gain.lsb_per_gauss();
    let num = raw as i32 * 100_000;
    let r = if num >= 0 { num + g / 2 } else { num - g / 2 };
    r / g
}

/// Decode the data registers (X, Z, Y, big-endian) to nT.
/// [`Error::InvalidData`] if an axis overflowed (pick a larger range).
pub fn decode<E>(b: &[u8; 6], gain: Gain) -> Result<Measurement, Error<E>> {
    let axis = |i: usize| -> Result<i32, Error<E>> {
        let raw = i16::from_be_bytes([b[i], b[i + 1]]);
        if raw == OVERFLOW {
            return Err(Error::InvalidData);
        }
        Ok(raw_to_nt(raw, gain))
    };
    Ok(Measurement {
        field: Vector3 {
            x: axis(0)?,
            y: axis(4)?,
            z: axis(2)?,
        },
    })
}

macro_rules! driver {
    ($doc:literal, $I2c:path, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Hmc5883l<I2C> {
            i2c: I2C,
            config: Config,
            discard: bool,
        }

        impl<I2C> Hmc5883l<I2C> {
            /// Create a driver at [`ADDRESS`]; no bus traffic.
            pub fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    config: Config::default(),
                    discard: false,
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

        impl<I2C: $I2c> Hmc5883l<I2C> {
            /// Read consecutive registers.
            pub $($async)? fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
                self.i2c.write_read(ADDRESS, &[reg], buf) $($aw)* .map_err(Error::bus)
            }

            /// Write one register.
            pub $($async)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c.write(ADDRESS, &[reg, value]) $($aw)* .map_err(Error::bus)
            }

            /// Read identification registers A–C ("H43"). Many cheap
            /// "HMC5883L" modules carry a QMC5883L (address 0x0D) instead.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let mut id = [0u8; 3];
                self.read_registers(reg::ID_A, &mut id) $($aw)* ?;
                if id != ID {
                    return Err(Error::UnexpectedId(
                        u32::from(id[0]) << 16 | u32::from(id[1]) << 8 | u32::from(id[2]),
                    ));
                }
                Ok(())
            }

            /// Write the configuration. A gain change only applies from
            /// the second measurement, so in single mode a dummy
            /// measurement is started and the returned time must pass
            /// before `start_measurement()`. In continuous mode returns
            /// the time to the first sample with the new gain (3 periods).
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let (a, b) = (self.config.config_a(), self.config.config_b());
                self.write_register(reg::CONFIG_A, a) $($aw)* ?;
                self.write_register(reg::CONFIG_B, b) $($aw)* ?;
                if let Some(rate) = self.config.continuous {
                    self.discard = false;
                    self.write_register(reg::MODE, MODE_CONTINUOUS) $($aw)* ?;
                    Ok(3 * rate.period_us())
                } else {
                    self.discard = true;
                    self.write_register(reg::MODE, MODE_SINGLE) $($aw)* ?;
                    Ok(self.config.measurement_us())
                }
            }

            /// Store a new configuration and write it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I2C::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Idle mode (≈2 µA); stops continuous mode until `wake()`.
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_register(reg::MODE, MODE_IDLE) $($aw)*
            }
        }

        impl<I2C: $I2c> zigbee_sensor::$mode::Sensor for Hmc5883l<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Single mode: trigger one measurement (after reading out the
            /// dummy measurement of `configure()`) and return its period.
            /// Continuous mode: 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.config.continuous.is_some() {
                    return Ok(0);
                }
                if self.discard {
                    let mut b = [0u8; 6];
                    self.read_registers(reg::DATA_X_MSB, &mut b) $($aw)* ?;
                    self.discard = false;
                }
                self.write_register(reg::MODE, MODE_SINGLE) $($aw)* ?;
                Ok(self.config.measurement_us())
            }

            /// [`Error::NotReady`] until the status `RDY` bit is set;
            /// [`Error::InvalidData`] on an axis overflow.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut status = [0u8];
                self.read_registers(reg::STATUS, &mut status) $($aw)* ?;
                if status[0] & STATUS_RDY == 0 {
                    return Err(Error::NotReady);
                }
                let mut b = [0u8; 6];
                self.read_registers(reg::DATA_X_MSB, &mut b) $($aw)* ?;
                decode(&b, self.config.gain)
            }
        }

        impl<I2C: $I2c> zigbee_sensor::$mode::LowPower for Hmc5883l<I2C> {
            type BusError = I2C::Error;

            /// [`Self::power_down`].
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.power_down() $($aw)*
            }

            /// Restart continuous mode (first sample after 2 periods);
            /// nothing to do in single mode.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let Some(rate) = self.config.continuous else { return Ok(0) };
                self.write_register(reg::MODE, MODE_CONTINUOUS) $($aw)* ?;
                Ok(2 * rate.period_us())
            }
        }
    };
}

driver!("Blocking driver.", embedded_hal::i2c::I2c, blocking, [], []);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        "Async twin of [`super::Hmc5883l`].",
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

    // X = +1090, Z = -545, Y = 1 (register order X, Z, Y).
    const FRAME: [u8; 6] = [0x04, 0x42, 0xFD, 0xDF, 0x00, 0x01];

    #[test]
    fn datasheet_gain_table() {
        assert_eq!(raw_to_nt(1090, Gain::Ga1_3), 100_000);
        assert_eq!(raw_to_nt(-2048, Gain::Ga8_1), -890_435);
        assert_eq!(raw_to_nt(2047, Gain::Ga0_88), 149_416);
        assert_eq!(raw_to_nt(1, Gain::Ga1_3), 92);
        let m = decode::<()>(&FRAME, Gain::Ga1_3).unwrap();
        assert_eq!(
            m.field,
            Vector3 {
                x: 100_000,
                y: 92,
                z: -50_000
            }
        );
        let ovf = [0xF0, 0x00, 0, 0, 0, 0];
        assert_eq!(decode::<()>(&ovf, Gain::Ga1_3), Err(Error::InvalidData));
        assert_eq!(Config::default().config_a(), 0x10);
        assert_eq!(Config::default().config_b(), 0x20);
    }

    #[test]
    fn probe_identification() {
        let mut i2c = Mock::new(&[rd(0x0A, b"H43"), rd(0x0A, &[0, 0, 0])]);
        let mut mag = Hmc5883l::new(&mut i2c);
        mag.probe().unwrap();
        assert_eq!(mag.probe(), Err(Error::UnexpectedId(0)));
        i2c.done();
    }

    #[test]
    fn single_measurement_discards_gain_change_sample() {
        let cfg = Config {
            gain: Gain::Ga4_0,
            averaging: Averaging::X8,
            continuous: None,
        };
        let mut i2c = Mock::new(&[
            wr(reg::CONFIG_A, 0x70),
            wr(reg::CONFIG_B, 0x80),
            wr(reg::MODE, 0x01),
            rd(reg::DATA_X_MSB, &[0; 6]),
            wr(reg::MODE, 0x01),
            rd(reg::STATUS, &[0x00]),
            rd(reg::STATUS, &[0x01]),
            // X = 440 -> 1 gauss
            rd(reg::DATA_X_MSB, &[0x01, 0xB8, 0, 0, 0, 0]),
            wr(reg::MODE, 0x01),
            wr(reg::MODE, 0x02),
        ]);
        let mut mag = Hmc5883l::new(&mut i2c).with_config(cfg);
        assert_eq!(mag.configure().unwrap(), 48_000);
        assert_eq!(mag.start_measurement().unwrap(), 48_000);
        assert_eq!(mag.read_measurement(), Err(Error::NotReady));
        assert_eq!(mag.read_measurement().unwrap().field.x, 100_000);
        mag.start_measurement().unwrap();
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 0);
        i2c.done();
    }

    #[test]
    fn continuous_mode() {
        let cfg = Config {
            continuous: Some(DataRate::Hz75),
            ..Config::default()
        };
        let mut i2c = Mock::new(&[
            wr(reg::CONFIG_A, 0x18),
            wr(reg::CONFIG_B, 0x20),
            wr(reg::MODE, 0x00),
            rd(reg::STATUS, &[0x01]),
            rd(reg::DATA_X_MSB, &FRAME),
            wr(reg::MODE, 0x02),
            wr(reg::MODE, 0x00),
        ]);
        let mut mag = Hmc5883l::new(&mut i2c).with_config(cfg);
        assert_eq!(mag.configure().unwrap(), 40_002);
        assert_eq!(mag.start_measurement().unwrap(), 0);
        assert_eq!(mag.read_measurement().unwrap().field.z, -50_000);
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 26_668);
        i2c.done();
    }

    #[test]
    fn async_single_measurement() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut i2c = Mock::new(&[
            wr(reg::MODE, 0x01),
            rd(reg::STATUS, &[0x01]),
            rd(reg::DATA_X_MSB, &FRAME),
        ]);
        let mut mag = asynch::Hmc5883l::new(&mut i2c);
        assert_eq!(block_on(mag.start_measurement()).unwrap(), 6_000);
        assert_eq!(block_on(mag.read_measurement()).unwrap().field.x, 100_000);
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
