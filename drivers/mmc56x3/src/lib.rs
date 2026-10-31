//! MEMSIC MMC5603NJ / MMC5633NJL 3-axis AMR magnetometer over I2C.
//!
//! Both parts share the register map, I2C address 0x30 and product ID
//! 0x10. The control registers are write-only, so the driver keeps the
//! configuration and rewrites whole registers.
//!
//! By default every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! triggers one on-demand measurement with automatic SET/RESET (offset
//! and temperature-drift removal) and returns the measurement time `tTM`
//! of the selected [`Bandwidth`]; between measurements the part draws
//! ≈1 µA. [`Config::continuous`] runs the internal continuous mode
//! instead. The part has no interrupt pin, so there is no threshold
//! interrupt.
//!
//! The field is reported as [`Quantity::MagneticField`] in nT (20-bit
//! output, 16384 counts/gauss = 6.1 nT/LSB, ±30 gauss).
//!
//! Reference: MEMSIC "MMC5603NJ" datasheet Rev. B (formal release
//! 2022-01-17). MMC5633: same registers per the Linux `mmc5633` IIO
//! driver.
//!
//! ```ignore
//! let mut mag = Mmc56x3::new(i2c);
//! mag.probe()?;
//! mag.configure()?;
//! let wait = mag.start_measurement()?;
//! delay_us(wait);
//! let m = mag.read_measurement()?; // m.field in nT
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x30;

/// `Product_ID` value (MMC5603 and MMC5633).
pub const PRODUCT_ID: u8 = 0x10;

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const XOUT0: u8 = 0x00;
    pub const TOUT: u8 = 0x09;
    pub const STATUS1: u8 = 0x18;
    pub const ODR: u8 = 0x1A;
    pub const CTRL0: u8 = 0x1B;
    pub const CTRL1: u8 = 0x1C;
    pub const CTRL2: u8 = 0x1D;
    pub const PRODUCT_ID: u8 = 0x39;
}

const CTRL0_TAKE_MEAS_M: u8 = 0x01;
const CTRL0_DO_SET: u8 = 0x08;
const CTRL0_DO_RESET: u8 = 0x10;
const CTRL0_AUTO_SR_EN: u8 = 0x20;
const CTRL0_CMM_FREQ_EN: u8 = 0x80;
const CTRL1_SW_RESET: u8 = 0x80;
const CTRL2_EN_PRD_SET: u8 = 0x08;
const CTRL2_CMM_EN: u8 = 0x10;
const CTRL2_HPOWER: u8 = 0x80;
const STATUS1_MEAS_M_DONE: u8 = 0x40;

/// Output count at zero field (20-bit offset binary).
pub const NULL_FIELD: i32 = 1 << 19;

/// Wait after a SET or RESET before the next operation (`tSR`).
pub const SET_RESET_US: WaitUs = 1_000;

/// Power-on / software-reset time (OTP reload).
pub const RESET_US: WaitUs = 20_000;

/// Measurement bandwidth (`Internal Control 1.BW`): trades noise for
/// measurement time and energy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Bandwidth {
    /// 6.6 ms, 1.5 mG RMS noise, 3.4 mA at 100 Hz (reset value).
    #[default]
    Bw00 = 0,
    /// 3.5 ms, 2.0 mG RMS, 1.8 mA at 100 Hz.
    Bw01 = 1,
    /// 2.0 ms, 3.0 mG RMS, 1.3 mA at 100 Hz.
    Bw10 = 2,
    /// 1.2 ms, 4.0 mG RMS, 0.75 mA at 100 Hz.
    Bw11 = 3,
}

impl Bandwidth {
    /// Measurement time `tTM`.
    pub const fn measurement_us(self) -> WaitUs {
        match self {
            Self::Bw00 => 6_600,
            Self::Bw01 => 3_500,
            Self::Bw10 => 2_000,
            Self::Bw11 => 1_200,
        }
    }

    /// Highest continuous-mode `ODR` value for this bandwidth.
    pub const fn max_odr(self, auto_set_reset: bool) -> u8 {
        match (self, auto_set_reset) {
            (Self::Bw00, true) => 75,
            (Self::Bw00, false) | (Self::Bw01, true) => 150,
            _ => 255,
        }
    }
}

/// Automatic SET every N samples in continuous mode (`Prd_set`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PeriodicSet {
    /// Every sample.
    Every1 = 0,
    /// Every 25 samples.
    Every25 = 1,
    /// Every 75 samples.
    Every75 = 2,
    /// Every 100 samples.
    Every100 = 3,
    /// Every 250 samples.
    Every250 = 4,
    /// Every 500 samples.
    Every500 = 5,
    /// Every 1000 samples.
    Every1000 = 6,
    /// Every 2000 samples.
    Every2000 = 7,
}

/// Internal continuous measurement mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Continuous {
    /// Output data rate in Hz, 1 up to [`Bandwidth::max_odr`].
    pub odr_hz: u8,
    /// `hpower`: 1000 Hz; requires `odr_hz` = 255 and [`Bandwidth::Bw11`].
    pub high_power: bool,
    /// Do the automatic SET/RESET only every N samples (requires
    /// [`Config::auto_set_reset`]); `None` = every sample.
    pub periodic_set: Option<PeriodicSet>,
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Bandwidth (default [`Bandwidth::Bw00`]).
    pub bandwidth: Bandwidth,
    /// Automatic SET/RESET before each measurement (default on,
    /// recommended by MEMSIC); removes the bridge offset and its drift.
    pub auto_set_reset: bool,
    /// Run the internal continuous mode instead of on-demand
    /// measurements (default `None`).
    pub continuous: Option<Continuous>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            bandwidth: Bandwidth::Bw00,
            auto_set_reset: true,
            continuous: None,
        }
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

/// 20-bit offset-binary output count to nT, rounded to nearest
/// (1 gauss = 16384 counts = 100 000 nT).
pub const fn raw_to_nt(raw: u32) -> i32 {
    let d = (raw & 0xF_FFFF) as i64 - NULL_FIELD as i64;
    let num = d * 3125;
    let r = if num >= 0 { num + 256 } else { num - 256 };
    (r / 512) as i32
}

/// Decode `Xout0..Zout2` (registers 0x00–0x08) to nT.
pub fn decode(b: &[u8; 9]) -> Measurement {
    let axis = |i: usize| {
        let raw =
            (u32::from(b[2 * i]) << 12) | (u32::from(b[2 * i + 1]) << 4) | u32::from(b[6 + i] >> 4);
        raw_to_nt(raw)
    };
    Measurement {
        field: Vector3 {
            x: axis(0),
            y: axis(1),
            z: axis(2),
        },
    }
}

/// Manual offset removal from one measurement after SET and one after
/// RESET (both with `auto_set_reset` off): returns `(field, offset)` with
/// field = (set − reset) / 2 and offset = (set + reset) / 2. Store the
/// offset and subtract it from later measurements taken after a SET.
pub fn set_reset_offset(after_set: Vector3, after_reset: Vector3) -> (Vector3, Vector3) {
    let f = |s: i32, r: i32| ((s - r) / 2, (s + r) / 2);
    let (fx, ox) = f(after_set.x, after_reset.x);
    let (fy, oy) = f(after_set.y, after_reset.y);
    let (fz, oz) = f(after_set.z, after_reset.z);
    (
        Vector3 {
            x: fx,
            y: fy,
            z: fz,
        },
        Vector3 {
            x: ox,
            y: oy,
            z: oz,
        },
    )
}

/// `Internal Control 0` value that triggers one measurement.
pub fn ctrl0_measure(config: &Config) -> u8 {
    if config.auto_set_reset {
        CTRL0_TAKE_MEAS_M | CTRL0_AUTO_SR_EN
    } else {
        CTRL0_TAKE_MEAS_M
    }
}

/// `(ODR, Internal Control 0, Internal Control 2)` that start continuous
/// mode, or `None` if the settings are out of range.
pub fn continuous_regs(config: &Config, c: &Continuous) -> Option<(u8, u8, u8)> {
    if c.odr_hz == 0 || c.odr_hz > config.bandwidth.max_odr(config.auto_set_reset) {
        return None;
    }
    if c.high_power && (c.odr_hz != 255 || config.bandwidth != Bandwidth::Bw11) {
        return None;
    }
    let mut ctrl0 = CTRL0_CMM_FREQ_EN;
    if config.auto_set_reset {
        ctrl0 |= CTRL0_AUTO_SR_EN;
    }
    let mut ctrl2 = CTRL2_CMM_EN;
    if c.high_power {
        ctrl2 |= CTRL2_HPOWER;
    }
    if let Some(p) = c.periodic_set {
        if !config.auto_set_reset {
            return None;
        }
        ctrl2 |= CTRL2_EN_PRD_SET | p as u8;
    }
    Some((c.odr_hz, ctrl0, ctrl2))
}

/// Continuous-mode sample period, rounded up.
pub fn period_us(c: &Continuous) -> WaitUs {
    if c.high_power {
        1_000
    } else {
        1_000_000u32.div_ceil(u32::from(c.odr_hz.max(1)))
    }
}

macro_rules! driver {
    ($doc:literal, $I2c:path, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Mmc56x3<I2C> {
            i2c: I2C,
            config: Config,
        }

        impl<I2C> Mmc56x3<I2C> {
            /// Create a driver at [`ADDRESS`]; no bus traffic.
            pub fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    config: Config::default(),
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

        impl<I2C: $I2c> Mmc56x3<I2C> {
            /// Read consecutive registers.
            pub $($async)? fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
                self.i2c.write_read(ADDRESS, &[reg], buf) $($aw)* .map_err(Error::bus)
            }

            /// Write one register.
            pub $($async)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c.write(ADDRESS, &[reg, value]) $($aw)* .map_err(Error::bus)
            }

            /// Read `Product_ID` (0x10).
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let mut id = [0u8];
                self.read_registers(reg::PRODUCT_ID, &mut id) $($aw)* ?;
                if id[0] != PRODUCT_ID {
                    return Err(Error::UnexpectedId(u32::from(id[0])));
                }
                Ok(())
            }

            /// Software reset (reloads OTP); wait [`RESET_US`], then call
            /// `configure()`.
            pub $($async)? fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(reg::CTRL1, CTRL1_SW_RESET) $($aw)* ?;
                Ok(RESET_US)
            }

            /// Write the stored configuration. Returns 0 for on-demand
            /// measurements, or the sample period when continuous mode is
            /// started. [`Error::Unsupported`] for an invalid
            /// [`Continuous`] setting.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(reg::CTRL2, 0) $($aw)* ?;
                self.write_register(reg::CTRL1, self.config.bandwidth as u8) $($aw)* ?;
                self.start_continuous() $($aw)*
            }

            /// Store a new configuration and write it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I2C::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            $($async)? fn start_continuous(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let Some(c) = self.config.continuous else { return Ok(0) };
                let (odr, ctrl0, ctrl2) =
                    continuous_regs(&self.config, &c).ok_or(Error::Unsupported)?;
                self.write_register(reg::ODR, odr) $($aw)* ?;
                self.write_register(reg::CTRL0, ctrl0) $($aw)* ?;
                self.write_register(reg::CTRL2, ctrl2) $($aw)* ?;
                Ok(period_us(&c))
            }

            /// SET pulse (sensor magnetised in the positive direction);
            /// wait [`SET_RESET_US`]. For manual offset removal with
            /// `auto_set_reset` off, see [`set_reset_offset`].
            pub $($async)? fn do_set(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(reg::CTRL0, CTRL0_DO_SET) $($aw)* ?;
                Ok(SET_RESET_US)
            }

            /// RESET pulse (negative direction); wait [`SET_RESET_US`].
            pub $($async)? fn do_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(reg::CTRL0, CTRL0_DO_RESET) $($aw)* ?;
                Ok(SET_RESET_US)
            }

            /// Stop continuous mode (≈1 µA); on-demand mode already idles
            /// between measurements.
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_register(reg::CTRL2, 0) $($aw)*
            }
        }

        impl<I2C: $I2c> zigbee_sensor::$mode::Sensor for Mmc56x3<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// On demand: trigger one measurement and return `tTM`.
            /// Continuous mode: 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.config.continuous.is_some() {
                    return Ok(0);
                }
                self.write_register(reg::CTRL0, ctrl0_measure(&self.config)) $($aw)* ?;
                Ok(self.config.bandwidth.measurement_us())
            }

            /// [`Error::NotReady`] until `Status1.Meas_m_done` reports a
            /// new sample (cleared by reading the data).
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut status = [0u8];
                self.read_registers(reg::STATUS1, &mut status) $($aw)* ?;
                if status[0] & STATUS1_MEAS_M_DONE == 0 {
                    return Err(Error::NotReady);
                }
                let mut b = [0u8; 9];
                self.read_registers(reg::XOUT0, &mut b) $($aw)* ?;
                Ok(decode(&b))
            }
        }

        impl<I2C: $I2c> zigbee_sensor::$mode::LowPower for Mmc56x3<I2C> {
            type BusError = I2C::Error;

            /// [`Self::power_down`].
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.power_down() $($aw)*
            }

            /// Restart continuous mode if configured; nothing to do for
            /// on-demand measurements.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.start_continuous() $($aw)*
            }
        }
    };
}

driver!("Blocking driver.", embedded_hal::i2c::I2c, blocking, [], []);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        "Async twin of [`super::Mmc56x3`].",
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

    // X = +1 G, Y = -1 G, Z = null + 1 count.
    const FRAME: [u8; 9] = [0x84, 0x00, 0x7C, 0x00, 0x80, 0x00, 0x00, 0x00, 0x10];

    #[test]
    fn datasheet_scale_and_timing() {
        // 16384 counts/G, null field 524288.
        assert_eq!(raw_to_nt(524_288 + 16_384), 100_000);
        assert_eq!(raw_to_nt(524_288 - 16_384), -100_000);
        assert_eq!(raw_to_nt(524_289), 6);
        assert_eq!(raw_to_nt(0), -3_200_000);
        assert_eq!(raw_to_nt(0xF_FFFF), 3_199_994);
        assert_eq!(
            decode(&FRAME).field,
            Vector3 {
                x: 100_000,
                y: -100_000,
                z: 6
            }
        );
        assert_eq!(Bandwidth::Bw11.measurement_us(), 1_200);
        assert_eq!(Bandwidth::Bw00.max_odr(true), 75);
    }

    #[test]
    fn probe_product_id() {
        let mut i2c = Mock::new(&[rd(0x39, &[0x10]), rd(0x39, &[0x0C])]);
        let mut mag = Mmc56x3::new(&mut i2c);
        mag.probe().unwrap();
        assert_eq!(mag.probe(), Err(Error::UnexpectedId(0x0C)));
        i2c.done();
    }

    #[test]
    fn on_demand_measurement_with_auto_set_reset() {
        let cfg = Config {
            bandwidth: Bandwidth::Bw10,
            ..Config::default()
        };
        let mut i2c = Mock::new(&[
            wr(reg::CTRL2, 0x00),
            wr(reg::CTRL1, 0x02),
            wr(reg::CTRL0, 0x21),
            rd(reg::STATUS1, &[0x10]),
            rd(reg::STATUS1, &[0x50]),
            rd(reg::XOUT0, &FRAME),
            wr(reg::CTRL2, 0x00),
        ]);
        let mut mag = Mmc56x3::new(&mut i2c).with_config(cfg);
        assert_eq!(mag.configure().unwrap(), 0);
        assert_eq!(mag.start_measurement().unwrap(), 2_000);
        assert_eq!(mag.read_measurement(), Err(Error::NotReady));
        assert_eq!(mag.read_measurement().unwrap().field.x, 100_000);
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 0);
        i2c.done();
    }

    #[test]
    fn manual_set_reset_offset_removal() {
        let cfg = Config {
            auto_set_reset: false,
            ..Config::default()
        };
        let mut i2c = Mock::new(&[
            wr(reg::CTRL0, 0x08),
            wr(reg::CTRL0, 0x01),
            wr(reg::CTRL0, 0x10),
            wr(reg::CTRL0, 0x01),
        ]);
        let mut mag = Mmc56x3::new(&mut i2c).with_config(cfg);
        assert_eq!(mag.do_set().unwrap(), 1_000);
        assert_eq!(mag.start_measurement().unwrap(), 6_600);
        assert_eq!(mag.do_reset().unwrap(), 1_000);
        mag.start_measurement().unwrap();
        i2c.done();
        let s = Vector3 {
            x: 52_000,
            y: -10_000,
            z: 3_000,
        };
        let r = Vector3 {
            x: -48_000,
            y: 14_000,
            z: 1_000,
        };
        let (field, offset) = set_reset_offset(s, r);
        assert_eq!(
            field,
            Vector3 {
                x: 50_000,
                y: -12_000,
                z: 1_000
            }
        );
        assert_eq!(
            offset,
            Vector3 {
                x: 2_000,
                y: 2_000,
                z: 2_000
            }
        );
    }

    #[test]
    fn continuous_mode_and_validation() {
        let cfg = Config {
            bandwidth: Bandwidth::Bw01,
            continuous: Some(Continuous {
                odr_hz: 50,
                high_power: false,
                periodic_set: Some(PeriodicSet::Every100),
            }),
            ..Config::default()
        };
        let mut i2c = Mock::new(&[
            wr(reg::CTRL2, 0x00),
            wr(reg::CTRL1, 0x01),
            wr(reg::ODR, 50),
            wr(reg::CTRL0, 0xA0),
            wr(reg::CTRL2, 0x1B),
            wr(reg::CTRL2, 0x00),
            wr(reg::ODR, 50),
            wr(reg::CTRL0, 0xA0),
            wr(reg::CTRL2, 0x1B),
            // ODR 100 > 75 Hz limit of BW00 with auto SET/RESET
            wr(reg::CTRL2, 0x00),
            wr(reg::CTRL1, 0x00),
            wr(reg::CTRL1, CTRL1_SW_RESET),
        ]);
        let mut mag = Mmc56x3::new(&mut i2c).with_config(cfg);
        assert_eq!(mag.configure().unwrap(), 20_000);
        assert_eq!(mag.start_measurement().unwrap(), 0);
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 20_000);
        let bad = Config {
            continuous: Some(Continuous {
                odr_hz: 100,
                high_power: false,
                periodic_set: None,
            }),
            ..Config::default()
        };
        assert_eq!(mag.set_config(bad), Err(Error::Unsupported));
        assert_eq!(mag.soft_reset().unwrap(), 20_000);
        i2c.done();

        let hp = Continuous {
            odr_hz: 255,
            high_power: true,
            periodic_set: None,
        };
        let fast = Config {
            bandwidth: Bandwidth::Bw11,
            auto_set_reset: false,
            continuous: Some(hp),
        };
        assert_eq!(continuous_regs(&fast, &hp), Some((255, 0x80, 0x90)));
        assert_eq!(period_us(&hp), 1_000);
        assert_eq!(continuous_regs(&Config::default(), &hp), None);
    }

    #[test]
    fn async_on_demand_measurement() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut i2c = Mock::new(&[
            wr(reg::CTRL0, 0x21),
            rd(reg::STATUS1, &[0x40]),
            rd(reg::XOUT0, &FRAME),
        ]);
        let mut mag = asynch::Mmc56x3::new(&mut i2c);
        assert_eq!(block_on(mag.start_measurement()).unwrap(), 6_600);
        assert_eq!(block_on(mag.read_measurement()).unwrap().field.y, -100_000);
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
