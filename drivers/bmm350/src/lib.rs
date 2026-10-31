//! Bosch BMM350 3-axis TMR magnetometer over I2C.
//!
//! The part needs host-side compensation with factory coefficients stored
//! in its OTP: `configure()` reads them once (12 OTP words), terminates the
//! boot phase, runs the magnetic reset (bit reset + flux-guide reset) and
//! applies the settings. Like the other multi-step drivers, `configure()`
//! returns a wait; call [`continue_configure`](Bmm350::continue_configure)
//! (or `start_measurement()`) after each wait until it returns 0.
//!
//! By default every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! triggers one forced-mode conversion (with full CRST recharge), after
//! which the chip returns to suspend (1.8 µA) by itself. Normal mode
//! (continuous, 1.5625–400 Hz) is optional. The field is reported as
//! [`Quantity::MagneticField`] in nT (±2000 µT); the compensated die
//! temperature is available in [`Measurement::die_temperature`].
//! The BMM350 has no threshold interrupt, only data ready.
//!
//! References: Bosch Sensortec BMM350 datasheet BST-BMM350-DS001-27
//! (2023); compensation ported from the BMM350 SensorAPI v1.10.0
//! (BSD-3-Clause, see [`compensation`]).
//!
//! ```ignore
//! let mut mag = Bmm350::new(i2c, ADDRESS_ADSEL_LOW);
//! mag.probe()?;
//! let mut wait = mag.configure()?;
//! while wait > 0 {
//!     delay_us(wait);
//!     wait = mag.continue_configure()?;
//! }
//! delay_us(mag.start_measurement()?);
//! let m = mag.read_measurement()?; // m.field in nT
//! ```

#![no_std]

pub mod compensation;

pub use compensation::{Otp, Raw};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with ADSEL tied to GND.
pub const ADDRESS_ADSEL_LOW: u8 = 0x14;

/// I2C address with ADSEL tied to VDDIO.
pub const ADDRESS_ADSEL_HIGH: u8 = 0x15;

/// `CHIP_ID` value.
pub const CHIP_ID: u8 = 0x33;

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const CHIP_ID: u8 = 0x00;
    pub const PMU_CMD_AGGR_SET: u8 = 0x04;
    pub const PMU_CMD_AXIS_EN: u8 = 0x05;
    pub const PMU_CMD: u8 = 0x06;
    pub const INT_CTRL: u8 = 0x2E;
    pub const INT_STATUS: u8 = 0x30;
    pub const MAG_X_XLSB: u8 = 0x31;
    pub const OTP_CMD: u8 = 0x50;
    pub const OTP_DATA_MSB: u8 = 0x52;
    pub const OTP_STATUS: u8 = 0x55;
    pub const CMD: u8 = 0x7E;
}

/// Dummy bytes the BMM350 sends before the data of every I2C read.
pub const DUMMY_BYTES: usize = 2;

const PMU_SUS: u8 = 0x00;
const PMU_NM: u8 = 0x01;
const PMU_UPD_OAE: u8 = 0x02;
const PMU_FM: u8 = 0x03;
const PMU_FM_FAST: u8 = 0x04;
const PMU_FGR: u8 = 0x05;
const PMU_BR: u8 = 0x07;
const OTP_CMD_DIR_READ: u8 = 0x20;
const OTP_CMD_PWR_OFF: u8 = 0x80;
const OTP_STATUS_DONE: u8 = 0x01;
const OTP_STATUS_ERROR: u8 = 0xE0;
const CMD_SOFT_RESET: u8 = 0xB6;
const AXIS_EN_XYZ: u8 = 0x07;
const INT_DRDY_EN: u8 = 0x80;
const INT_OUTPUT_EN: u8 = 0x08;
const INT_PUSH_PULL: u8 = 0x04;
const INT_ACTIVE_HIGH: u8 = 0x02;
const INT_LATCHED: u8 = 0x01;
const INT_STATUS_DRDY: u8 = 0x04;
/// ODR code written in forced mode (25 Hz allows every averaging setting).
const FORCED_ODR: u8 = 0x06;
const OTP_MAX_POLLS: u8 = 10;

/// Start-up time from power-on to suspend (datasheet 2.5 ms, SensorAPI
/// 3 ms).
pub const POWER_ON_US: WaitUs = 3_000;
/// Soft reset time (SensorAPI).
pub const SOFT_RESET_US: WaitUs = 24_000;
/// Wait after each OTP read command (SensorAPI).
pub const OTP_READ_US: WaitUs = 300;
/// Bit reset time (SensorAPI).
pub const BIT_RESET_US: WaitUs = 14_000;
/// Flux-guide reset time (SensorAPI).
pub const FLUX_GUIDE_RESET_US: WaitUs = 18_000;
/// ODR/averaging update time (SensorAPI).
pub const UPDATE_US: WaitUs = 1_000;
/// Normal → suspend transition (SensorAPI).
pub const GOTO_SUSPEND_US: WaitUs = 6_000;
/// Suspend → normal start-up time (datasheet `Tsus2nm`).
pub const SUSPEND_TO_NORMAL_US: WaitUs = 70_000;

/// Number of internal samples averaged per output (`PMU_CMD_AGGR_SET.avg`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Averaging {
    /// 1 sample, "low power".
    None = 0,
    /// 2 samples, "regular power".
    #[default]
    Two = 1,
    /// 4 samples, "low noise".
    Four = 2,
    /// 8 samples, "ultra low noise".
    Eight = 3,
}

impl Averaging {
    /// Forced-mode conversion time from suspend (SensorAPI delays):
    /// `FM` 15/17/20/28 ms, `FM_FAST` 4/5/9/16 ms.
    pub fn forced_us(self, fast: bool) -> WaitUs {
        let (normal, fast_us) = match self {
            Averaging::None => (15_000, 4_000),
            Averaging::Two => (17_000, 5_000),
            Averaging::Four => (20_000, 9_000),
            Averaging::Eight => (28_000, 16_000),
        };
        if fast { fast_us } else { normal }
    }
}

/// Normal-mode output data rate (`PMU_CMD_AGGR_SET.odr`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataRate {
    /// 400 Hz (no averaging only).
    Hz400 = 0x02,
    /// 200 Hz (averaging ≤ 2).
    Hz200 = 0x03,
    /// 100 Hz (averaging ≤ 4).
    Hz100 = 0x04,
    /// 50 Hz.
    Hz50 = 0x05,
    /// 25 Hz.
    Hz25 = 0x06,
    /// 12.5 Hz.
    Hz12_5 = 0x07,
    /// 6.25 Hz.
    Hz6_25 = 0x08,
    /// 3.125 Hz.
    Hz3_125 = 0x09,
    /// 1.5625 Hz.
    Hz1_5625 = 0x0A,
}

impl DataRate {
    /// Highest averaging allowed at this rate (datasheet Table 5).
    pub fn max_averaging(self) -> Averaging {
        match self {
            DataRate::Hz400 => Averaging::None,
            DataRate::Hz200 => Averaging::Two,
            DataRate::Hz100 => Averaging::Four,
            _ => Averaging::Eight,
        }
    }
}

/// Data-ready output on the INT pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct IntPin {
    /// Active high (else active low).
    pub active_high: bool,
    /// Push-pull (else open drain).
    pub push_pull: bool,
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Noise/current trade-off (default 2 samples, "regular power").
    pub averaging: Averaging,
    /// Run in normal mode at this rate instead of forced (single)
    /// measurements (default `None`).
    pub continuous: Option<DataRate>,
    /// Use `FM_FAST` (fast CRST recharge) for forced measurements. Only
    /// allowed when triggering at ≥ 25 Hz (datasheet 5.1.4); default `FM`.
    pub fast_forced: bool,
    /// Route data ready (latched, cleared by `read_measurement()`) to the
    /// INT pin (default `None`). A-sample parts always drive the pin.
    pub int_pin: Option<IntPin>,
}

impl Config {
    /// `PMU_CMD_AGGR_SET` value, or `None` if the averaging is too high for
    /// the data rate.
    pub fn aggr_set(&self) -> Option<u8> {
        let odr = match self.continuous {
            Some(rate) if self.averaging as u8 > rate.max_averaging() as u8 => return None,
            Some(rate) => rate as u8,
            None => FORCED_ODR,
        };
        Some(((self.averaging as u8) << 4) | odr)
    }

    /// `INT_CTRL` value: data ready to `INT_STATUS`, latched.
    pub fn int_ctrl(&self) -> u8 {
        let pin = self.int_pin.map_or(0, |p| {
            INT_OUTPUT_EN
                | if p.push_pull { INT_PUSH_PULL } else { 0 }
                | if p.active_high { INT_ACTIVE_HIGH } else { 0 }
        });
        INT_DRDY_EN | INT_LATCHED | pin
    }

    /// Wait returned by `start_measurement()`: forced conversion time, or 0
    /// in normal mode.
    pub fn measurement_us(&self) -> WaitUs {
        match self.continuous {
            Some(_) => 0,
            None => self.averaging.forced_us(self.fast_forced),
        }
    }
}

/// One magnetometer sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Flux density in nT.
    pub field: Vector3,
    /// Compensated die temperature in 0.01 °C (not ambient temperature).
    pub die_temperature: i32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::MagneticField(self.field));
    }
}

/// Decode `MAG_X_XLSB`..`TEMP_MSB` (12 bytes, dummy bytes removed): four
/// 24-bit little-endian two's-complement values.
pub fn decode_raw(b: &[u8; 12]) -> Raw {
    let v = |i: usize| i32::from_le_bytes([0, b[i], b[i + 1], b[i + 2]]) >> 8;
    Raw {
        x: v(0),
        y: v(3),
        z: v(6),
        t: v(9),
    }
}

/// Decode and compensate a data frame. [`Error::InvalidData`] if the
/// compensation divides by zero.
pub fn decode<E>(otp: &Otp, b: &[u8; 12]) -> Result<Measurement, Error<E>> {
    let c = otp.compensate(decode_raw(b)).ok_or(Error::InvalidData)?;
    let nt = |v| compensation::scale_round(v, 1000);
    Ok(Measurement {
        field: Vector3 {
            x: nt(c.field[0]),
            y: nt(c.field[1]),
            z: nt(c.field[2]),
        },
        die_temperature: compensation::scale_round(c.temperature, 100),
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Unconfigured,
    Otp { index: u8, polls: u8 },
    BitReset,
    FluxGuideReset,
    Settings,
    Normal,
    Ready,
}

macro_rules! driver {
    ($doc:literal, $I2c:path, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Bmm350<I2C> {
            i2c: I2C,
            address: u8,
            config: Config,
            otp: Otp,
            otp_loaded: bool,
            step: Step,
            normal: bool,
        }

        impl<I2C> Bmm350<I2C> {
            /// Create a driver; no bus traffic. `address` is
            /// [`ADDRESS_ADSEL_LOW`] or [`ADDRESS_ADSEL_HIGH`].
            pub fn new(i2c: I2C, address: u8) -> Self {
                Self {
                    i2c,
                    address,
                    config: Config::default(),
                    otp: Otp::default(),
                    otp_loaded: false,
                    step: Step::Unconfigured,
                    normal: false,
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

            /// Factory OTP words, once loaded by `configure()`.
            pub fn otp(&self) -> Option<&Otp> {
                self.otp_loaded.then_some(&self.otp)
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: $I2c> Bmm350<I2C> {
            /// Read up to 12 consecutive registers, discarding the two
            /// leading dummy bytes. [`Error::Unsupported`] if `buf` is
            /// longer.
            pub $($async)? fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
                let mut tmp = [0u8; 12 + DUMMY_BYTES];
                let raw = tmp.get_mut(..buf.len() + DUMMY_BYTES).ok_or(Error::Unsupported)?;
                self.i2c.write_read(self.address, &[reg], raw) $($aw)* .map_err(Error::bus)?;
                buf.copy_from_slice(&raw[DUMMY_BYTES..]);
                Ok(())
            }

            /// Read one register.
            pub $($async)? fn read_register(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
                let mut b = [0u8];
                self.read_registers(reg, &mut b) $($aw)* ?;
                Ok(b[0])
            }

            /// Write one register.
            pub $($async)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c.write(self.address, &[reg, value]) $($aw)* .map_err(Error::bus)
            }

            /// Check `CHIP_ID` (0x33). Allow [`POWER_ON_US`] after power-on.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let id = self.read_register(reg::CHIP_ID) $($aw)* ?;
                if id != CHIP_ID {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(())
            }

            /// Soft reset: registers to default, suspend mode, new boot
            /// phase (`configure()` reads the OTP again).
            pub $($async)? fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(reg::CMD, CMD_SOFT_RESET) $($aw)* ?;
                self.otp_loaded = false;
                self.normal = false;
                self.step = Step::Unconfigured;
                Ok(SOFT_RESET_US)
            }

            /// Start applying the configuration (the first time: OTP read,
            /// boot termination and magnetic reset, ≈36 ms in total).
            /// Returns a wait; call [`Self::continue_configure`] after each
            /// wait until it returns 0. [`Error::Unsupported`] if the
            /// averaging is too high for the data rate.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if let Some(wait) = self.begin_configure() $($aw)* ? {
                    return Ok(wait);
                }
                self.advance() $($aw)*
            }

            $($async)? fn begin_configure(&mut self) -> Result<Option<WaitUs>, Error<I2C::Error>> {
                self.config.aggr_set().ok_or(Error::Unsupported)?;
                self.step = Step::Settings;
                if self.normal {
                    self.write_register(reg::PMU_CMD, PMU_SUS) $($aw)* ?;
                    self.normal = false;
                    return Ok(Some(GOTO_SUSPEND_US));
                }
                if !self.otp_loaded {
                    self.write_register(reg::OTP_CMD, OTP_CMD_DIR_READ | compensation::OTP_FIRST_WORD) $($aw)* ?;
                    self.step = Step::Otp { index: 0, polls: 0 };
                    return Ok(Some(OTP_READ_US));
                }
                Ok(None)
            }

            /// Perform the next configuration step. Returns the wait before
            /// the next call, or 0 when the sensor is ready.
            /// [`Error::InvalidData`] on an OTP read error,
            /// [`Error::Timeout`] if the OTP read never completes.
            pub $($async)? fn continue_configure(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.step == Step::Unconfigured {
                    return self.configure() $($aw)*;
                }
                self.advance() $($aw)*
            }

            $($async)? fn advance(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                match self.step {
                    Step::Otp { index, polls } => {
                        let status = self.read_register(reg::OTP_STATUS) $($aw)* ?;
                        if status & OTP_STATUS_ERROR != 0 {
                            self.step = Step::Unconfigured;
                            return Err(Error::InvalidData);
                        }
                        if status & OTP_STATUS_DONE == 0 {
                            if polls >= OTP_MAX_POLLS {
                                self.step = Step::Unconfigured;
                                return Err(Error::Timeout);
                            }
                            self.step = Step::Otp { index, polls: polls + 1 };
                            return Ok(OTP_READ_US);
                        }
                        let mut w = [0u8; 2];
                        self.read_registers(reg::OTP_DATA_MSB, &mut w) $($aw)* ?;
                        self.otp.0[usize::from(index)] = u16::from_be_bytes(w);
                        let next = index + 1;
                        if usize::from(next) < compensation::OTP_WORDS {
                            let cmd = OTP_CMD_DIR_READ | (compensation::OTP_FIRST_WORD + next);
                            self.write_register(reg::OTP_CMD, cmd) $($aw)* ?;
                            self.step = Step::Otp { index: next, polls: 0 };
                            return Ok(OTP_READ_US);
                        }
                        self.otp_loaded = true;
                        self.write_register(reg::OTP_CMD, OTP_CMD_PWR_OFF) $($aw)* ?;
                        self.step = Step::BitReset;
                        self.continue_configure_reset() $($aw)*
                    }
                    Step::BitReset | Step::FluxGuideReset => self.continue_configure_reset() $($aw)*,
                    Step::Settings => {
                        let aggr = self.config.aggr_set().ok_or(Error::Unsupported)?;
                        self.write_register(reg::PMU_CMD_AGGR_SET, aggr) $($aw)* ?;
                        self.write_register(reg::PMU_CMD_AXIS_EN, AXIS_EN_XYZ) $($aw)* ?;
                        self.write_register(reg::INT_CTRL, self.config.int_ctrl()) $($aw)* ?;
                        self.write_register(reg::PMU_CMD, PMU_UPD_OAE) $($aw)* ?;
                        self.step = if self.config.continuous.is_some() {
                            Step::Normal
                        } else {
                            Step::Ready
                        };
                        Ok(UPDATE_US)
                    }
                    Step::Normal => {
                        self.write_register(reg::PMU_CMD, PMU_NM) $($aw)* ?;
                        self.normal = true;
                        self.step = Step::Ready;
                        Ok(SUSPEND_TO_NORMAL_US)
                    }
                    Step::Unconfigured | Step::Ready => Ok(0),
                }
            }

            $($async)? fn continue_configure_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.step == Step::BitReset {
                    self.write_register(reg::PMU_CMD, PMU_BR) $($aw)* ?;
                    self.step = Step::FluxGuideReset;
                    Ok(BIT_RESET_US)
                } else {
                    self.write_register(reg::PMU_CMD, PMU_FGR) $($aw)* ?;
                    self.step = Step::Settings;
                    Ok(FLUX_GUIDE_RESET_US)
                }
            }

            /// Store a new configuration and start applying it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I2C::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Magnetic reset (bit reset + flux-guide reset), e.g. after
            /// exposure to a strong field while in suspend (datasheet
            /// 5.1.5). Continue with [`Self::continue_configure`]; the
            /// settings are rewritten afterwards.
            pub $($async)? fn magnetic_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if !self.otp_loaded {
                    return self.configure() $($aw)*;
                }
                self.step = Step::BitReset;
                if self.normal {
                    self.write_register(reg::PMU_CMD, PMU_SUS) $($aw)* ?;
                    self.normal = false;
                    return Ok(GOTO_SUSPEND_US);
                }
                self.continue_configure_reset() $($aw)*
            }

            /// Suspend mode (1.8 µA, settings kept). Takes
            /// [`GOTO_SUSPEND_US`] when leaving normal mode.
            pub $($async)? fn suspend(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_register(reg::PMU_CMD, PMU_SUS) $($aw)* ?;
                self.normal = false;
                Ok(())
            }
        }

        impl<I2C: $I2c> zigbee_sensor::$mode::Sensor for Bmm350<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// While configuring: perform the next configuration step and
            /// return its wait. Forced mode: trigger one conversion
            /// (suspend → forced → suspend) and return
            /// [`Config::measurement_us`]. Normal mode: 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.step != Step::Ready {
                    return self.continue_configure() $($aw)*;
                }
                if self.config.continuous.is_none() {
                    let cmd = if self.config.fast_forced { PMU_FM_FAST } else { PMU_FM };
                    self.write_register(reg::PMU_CMD, cmd) $($aw)* ?;
                }
                Ok(self.config.measurement_us())
            }

            /// [`Error::NotReady`] while configuring and until
            /// `INT_STATUS.drdy_data_reg` (cleared by this read); then one
            /// burst read of the data registers.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                if self.step != Step::Ready {
                    return Err(Error::NotReady);
                }
                let status = self.read_register(reg::INT_STATUS) $($aw)* ?;
                if status & INT_STATUS_DRDY == 0 {
                    return Err(Error::NotReady);
                }
                let mut b = [0u8; 12];
                self.read_registers(reg::MAG_X_XLSB, &mut b) $($aw)* ?;
                decode(&self.otp, &b)
            }
        }

        impl<I2C: $I2c> zigbee_sensor::$mode::LowPower for Bmm350<I2C> {
            type BusError = I2C::Error;

            /// [`Self::suspend`]; forced mode already returns to suspend
            /// after each conversion, so this only writes in normal mode.
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                if self.normal {
                    self.suspend() $($aw)*?;
                }
                Ok(())
            }

            /// Normal mode: restart it and return
            /// [`SUSPEND_TO_NORMAL_US`]; forced mode: no bus traffic, 0.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.step != Step::Ready || self.config.continuous.is_none() || self.normal {
                    return Ok(0);
                }
                self.write_register(reg::PMU_CMD, PMU_NM) $($aw)* ?;
                self.normal = true;
                Ok(SUSPEND_TO_NORMAL_US)
            }
        }
    };
}

driver!("Blocking driver.", embedded_hal::i2c::I2c, blocking, [], []);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        "Async twin of [`super::Bmm350`].",
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
    use std::vec::Vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = ADDRESS_ADSEL_LOW;
    const DUMMY: [u8; 2] = [0xAA, 0x55];

    const OTP_WORDS: [u16; 12] = [
        0x05F6, 0xFFE0, 0x009C, 0xFC35, 0xF907, 0xF00C, 0x08F4, 0xE020, 0xFA05, 0xF30A, 0x0000,
        0xFC00,
    ];

    fn rd(reg: u8, data: &[u8]) -> I2cTx {
        let mut v = Vec::from(DUMMY);
        v.extend_from_slice(data);
        I2cTx::write_read(A, std::vec![reg], v)
    }
    fn wr(reg: u8, value: u8) -> I2cTx {
        I2cTx::write(A, std::vec![reg, value])
    }

    fn put24(b: &mut [u8], v: i32) {
        b.copy_from_slice(&v.to_le_bytes()[..3]);
    }

    // X 4000, Y -7000, Z 60000, T 56553 (golden vector of `compensation`).
    fn frame() -> [u8; 12] {
        let mut b = [0u8; 12];
        put24(&mut b[0..3], 4000);
        put24(&mut b[3..6], -7000);
        put24(&mut b[6..9], 60000);
        put24(&mut b[9..12], 56553);
        b
    }

    /// OTP load + boot termination + magnetic reset.
    fn boot_tx(tx: &mut Vec<I2cTx>) {
        tx.push(wr(0x50, 0x2D));
        for (i, w) in OTP_WORDS.iter().enumerate() {
            if i == 0 {
                tx.push(rd(0x55, &[0x00]));
            }
            tx.push(rd(0x55, &[0x01]));
            tx.push(rd(0x52, &w.to_be_bytes()));
            if i < 11 {
                tx.push(wr(0x50, 0x2E + i as u8));
            }
        }
        tx.push(wr(0x50, 0x80));
        tx.push(wr(0x06, 0x07));
        tx.push(wr(0x06, 0x05));
    }

    fn configure_blocking<I: embedded_hal::i2c::I2c>(mag: &mut Bmm350<I>) -> Vec<WaitUs> {
        let mut waits = std::vec![mag.configure().unwrap()];
        while *waits.last().unwrap() != 0 {
            waits.push(mag.continue_configure().unwrap());
        }
        waits
    }

    #[test]
    fn raw_frame_layout() {
        let mut b = frame();
        assert_eq!(
            decode_raw(&b),
            Raw {
                x: 4000,
                y: -7000,
                z: 60000,
                t: 56553
            }
        );
        put24(&mut b[0..3], -8_388_608);
        assert_eq!(decode_raw(&b).x, -8_388_608);
        let m = decode::<()>(&Otp(OTP_WORDS), &frame()).unwrap();
        // -32973 / 65536 µT, -10028003 / 65536 µT, 31679488 / 65536 µT.
        assert_eq!(
            m.field,
            Vector3 {
                x: -503,
                y: -153_015,
                z: 483_391
            }
        );
        assert_eq!(m.die_temperature, 2825);
    }

    #[test]
    fn probe_and_soft_reset() {
        let mut i2c = Mock::new(&[rd(0x00, &[0x33]), rd(0x00, &[0x32]), wr(0x7E, 0xB6)]);
        let mut mag = Bmm350::new(&mut i2c, A);
        mag.probe().unwrap();
        assert_eq!(mag.probe(), Err(Error::UnexpectedId(0x32)));
        assert_eq!(mag.soft_reset().unwrap(), 24_000);
        i2c.done();
    }

    #[test]
    fn configure_then_forced_measurement() {
        let mut tx = Vec::new();
        boot_tx(&mut tx);
        tx.extend([
            wr(0x04, 0x16),
            wr(0x05, 0x07),
            wr(0x2E, 0x81),
            wr(0x06, 0x02),
            wr(0x06, 0x03),
            rd(0x30, &[0x00]),
            rd(0x30, &[0x04]),
            rd(0x31, &frame()),
            // Reconfigure: OTP is cached, no magnetic reset.
            wr(0x04, 0x36),
            wr(0x05, 0x07),
            wr(0x2E, 0x8F),
            wr(0x06, 0x02),
            wr(0x06, 0x04),
        ]);
        let mut i2c = Mock::new(&tx);
        let mut mag = Bmm350::new(&mut i2c, A);
        assert_eq!(mag.read_measurement(), Err(Error::NotReady));
        let waits = configure_blocking(&mut mag);
        let mut expected = std::vec![300; 13];
        expected.extend([14_000, 18_000, 1_000, 0]);
        assert_eq!(waits, expected);
        assert_eq!(mag.otp(), Some(&Otp(OTP_WORDS)));
        assert_eq!(mag.start_measurement().unwrap(), 17_000);
        assert_eq!(mag.read_measurement(), Err(Error::NotReady));
        assert_eq!(mag.read_measurement().unwrap().field.x, -503);
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 0);
        let cfg = Config {
            averaging: Averaging::Eight,
            fast_forced: true,
            int_pin: Some(IntPin {
                active_high: true,
                push_pull: true,
            }),
            ..Config::default()
        };
        assert_eq!(mag.set_config(cfg).unwrap(), 1_000);
        assert_eq!(mag.start_measurement().unwrap(), 16_000);
        i2c.done();
    }

    #[test]
    fn otp_errors() {
        let mut i2c = Mock::new(&[wr(0x50, 0x2D), rd(0x55, &[0x41])]);
        let mut mag = Bmm350::new(&mut i2c, A);
        assert_eq!(mag.configure().unwrap(), 300);
        assert_eq!(mag.continue_configure(), Err(Error::InvalidData));
        i2c.done();

        let mut tx = std::vec![wr(0x50, 0x2D)];
        tx.extend((0..=OTP_MAX_POLLS).map(|_| rd(0x55, &[0x00])));
        let mut i2c = Mock::new(&tx);
        let mut mag = Bmm350::new(&mut i2c, A);
        mag.configure().unwrap();
        for _ in 0..OTP_MAX_POLLS {
            assert_eq!(mag.continue_configure().unwrap(), 300);
        }
        assert_eq!(mag.continue_configure(), Err(Error::Timeout));
        i2c.done();
    }

    #[test]
    fn normal_mode_and_limits() {
        let bad = Config {
            averaging: Averaging::Four,
            continuous: Some(DataRate::Hz200),
            ..Config::default()
        };
        assert_eq!(bad.aggr_set(), None);
        let cfg = Config {
            continuous: Some(DataRate::Hz25),
            ..Config::default()
        };
        let mut tx = Vec::new();
        boot_tx(&mut tx);
        tx.extend([
            wr(0x04, 0x16),
            wr(0x05, 0x07),
            wr(0x2E, 0x81),
            wr(0x06, 0x02),
            wr(0x06, 0x01),
            rd(0x30, &[0x04]),
            rd(0x31, &frame()),
            wr(0x06, 0x00),
            wr(0x06, 0x01),
            // Magnetic reset from normal mode: suspend, BR, FGR, settings.
            wr(0x06, 0x00),
            wr(0x06, 0x07),
            wr(0x06, 0x05),
            wr(0x04, 0x16),
            wr(0x05, 0x07),
            wr(0x2E, 0x81),
            wr(0x06, 0x02),
            wr(0x06, 0x01),
        ]);
        let mut i2c = Mock::new(&tx);
        let mut mag = Bmm350::new(&mut i2c, A).with_config(cfg);
        assert_eq!(mag.set_config(bad), Err(Error::Unsupported));
        mag = mag.with_config(cfg);
        let waits = configure_blocking(&mut mag);
        assert_eq!(waits[waits.len() - 3..], [1_000, 70_000, 0]);
        assert_eq!(mag.start_measurement().unwrap(), 0);
        assert_eq!(mag.read_measurement().unwrap().die_temperature, 2825);
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 70_000);
        assert_eq!(mag.magnetic_reset().unwrap(), 6_000);
        let mut waits = std::vec![];
        loop {
            let w = mag.start_measurement().unwrap();
            if w == 0 {
                break;
            }
            waits.push(w);
        }
        assert_eq!(waits, [14_000, 18_000, 1_000, 70_000]);
        i2c.done();
    }

    #[test]
    fn async_forced_measurement() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut tx = Vec::new();
        boot_tx(&mut tx);
        tx.extend([
            wr(0x04, 0x06),
            wr(0x05, 0x07),
            wr(0x2E, 0x81),
            wr(0x06, 0x02),
            wr(0x06, 0x03),
            rd(0x30, &[0x04]),
            rd(0x31, &frame()),
        ]);
        let mut i2c = Mock::new(&tx);
        let cfg = Config {
            averaging: Averaging::None,
            ..Config::default()
        };
        let mut mag = asynch::Bmm350::new(&mut i2c, A).with_config(cfg);
        // start_measurement() drives the configuration steps.
        let mut waits = Vec::new();
        loop {
            let w = block_on(mag.start_measurement()).unwrap();
            waits.push(w);
            if w == 15_000 {
                break;
            }
        }
        assert_eq!(waits.len(), 13 + 3 + 1);
        assert_eq!(block_on(mag.read_measurement()).unwrap().field.z, 483_391);
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
