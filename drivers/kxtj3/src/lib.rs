//! Kionix KXTJ3-1057 three-axis accelerometer (I2C).
//!
//! The part samples continuously while operating (`PC1 = 1`); there is no
//! single-shot mode. [`PowerMode::LowPower`] gives 8-bit data at 10 µA
//! (ODR ≤ 200 Hz), [`PowerMode::HighResolution`] 12-bit (14-bit at ±8/16 g)
//! at 155 µA. [`Sensor::start_measurement`] leaves stand-by (returns
//! 1/ODR + 1 ms). With [`Config::data_ready`] set,
//! [`Sensor::read_measurement`] returns [`Error::NotReady`] until
//! `INT_SOURCE1.DRDY` is set. [`LowPower::sleep`] is stand-by (0.9 µA).
//!
//! The wake-up engine ([`WakeUpConfig`]) samples at its own rate (OWUF,
//! 0.781–100 Hz) and raises the push-pull INT pin on motion.
//!
//! Reference: Kionix "KXTJ3-1057 Specifications" Rev. 4.0 (05-Oct-2018).
//!
//! [`Sensor::start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
//! [`LowPower::sleep`]: zigbee_sensor::blocking::LowPower::sleep
//!
//! ```ignore
//! let mut acc = Kxtj3::new(i2c, ADDRESS_ADDR_LOW);
//! acc.probe()?;
//! acc.configure(Config { rate: DataRate::Hz0_781, ..Default::default() })?;
//! acc.configure_wake_up(&WakeUpConfig { threshold_mg: 100, ..Default::default() })?;
//! // ... sleep until INT, then:
//! let src = acc.read_interrupt_source()?; // reads INT_REL, releasing INT
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with ADDR to GND.
pub const ADDRESS_ADDR_LOW: u8 = 0x0E;
/// I2C address with ADDR to IO_VDD.
pub const ADDRESS_ADDR_HIGH: u8 = 0x0F;

/// WHO_AM_I value.
pub const WHO_AM_I: u8 = 0x35;

const REG_XOUT_L: u8 = 0x06;
const REG_WHO_AM_I: u8 = 0x0F;
const REG_INT_SOURCE1: u8 = 0x16;
const REG_INT_REL: u8 = 0x1A;
const REG_CTRL_REG1: u8 = 0x1B;
const REG_CTRL_REG2: u8 = 0x1D;
const REG_INT_CTRL_REG1: u8 = 0x1E;
const REG_INT_CTRL_REG2: u8 = 0x1F;
const REG_DATA_CTRL: u8 = 0x21;
const REG_WAKEUP_COUNTER: u8 = 0x29;
const REG_NA_COUNTER: u8 = 0x2A;
const REG_WAKEUP_THRESHOLD_H: u8 = 0x6A;
const REG_WAKEUP_THRESHOLD_L: u8 = 0x6B;

const CTRL1_PC1: u8 = 0x80;
const CTRL1_RES: u8 = 0x40;
const CTRL1_DRDYE: u8 = 0x20;
const CTRL1_WUFE: u8 = 0x02;
const INT_CTRL1_IEN: u8 = 0x20;
const INT_CTRL1_IEA: u8 = 0x10;
const INT_CTRL1_IEL: u8 = 0x08;
const INT_CTRL2_ULMODE: u8 = 0x80;
const SRC1_DRDY: u8 = 0x10;
const SRC1_WUFS: u8 = 0x02;

/// Performance mode (CTRL_REG1.RES).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PowerMode {
    /// 8-bit, 10 µA typ., ODR ≤ 200 Hz (faster rates select high
    /// resolution).
    #[default]
    LowPower,
    /// 12-bit (14-bit at ±8/16 g), 155 µA typ.
    HighResolution,
}

/// Full-scale range.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Range {
    /// ±2 g.
    #[default]
    G2,
    /// ±4 g.
    G4,
    /// ±8 g.
    G8,
    /// ±16 g.
    G16,
}

impl Range {
    const fn g(self) -> i32 {
        match self {
            Range::G2 => 2,
            Range::G4 => 4,
            Range::G8 => 8,
            Range::G16 => 16,
        }
    }
}

/// Output data rate (DATA_CTRL_REG).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DataRate {
    /// 0.781 Hz.
    Hz0_781 = 0x08,
    /// 1.563 Hz.
    Hz1_563 = 0x09,
    /// 3.125 Hz.
    Hz3_125 = 0x0A,
    /// 6.25 Hz.
    Hz6_25 = 0x0B,
    /// 12.5 Hz.
    #[default]
    Hz12_5 = 0x00,
    /// 25 Hz.
    Hz25 = 0x01,
    /// 50 Hz.
    Hz50 = 0x02,
    /// 100 Hz.
    Hz100 = 0x03,
    /// 200 Hz.
    Hz200 = 0x04,
    /// 400 Hz (high resolution only).
    Hz400 = 0x05,
    /// 800 Hz (high resolution only).
    Hz800 = 0x06,
    /// 1600 Hz (high resolution only).
    Hz1600 = 0x07,
}

impl DataRate {
    /// Sample period in µs.
    pub const fn period_us(self) -> u32 {
        match self {
            DataRate::Hz0_781 => 1_280_000,
            DataRate::Hz1_563 => 640_000,
            DataRate::Hz3_125 => 320_000,
            DataRate::Hz6_25 => 160_000,
            DataRate::Hz12_5 => 80_000,
            DataRate::Hz25 => 40_000,
            DataRate::Hz50 => 20_000,
            DataRate::Hz100 => 10_000,
            DataRate::Hz200 => 5_000,
            DataRate::Hz400 => 2_500,
            DataRate::Hz800 => 1_250,
            DataRate::Hz1600 => 625,
        }
    }

    const fn forces_high_resolution(self) -> bool {
        matches!(self, DataRate::Hz400 | DataRate::Hz800 | DataRate::Hz1600)
    }
}

/// Sampling configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Performance mode.
    pub mode: PowerMode,
    /// Full-scale range.
    pub range: Range,
    /// Output data rate.
    pub rate: DataRate,
    /// Enable DRDY reporting (DRDYE) so [`Sensor`] reads can return
    /// [`Error::NotReady`]. While the INT pin is enabled this also pulses
    /// it on every sample, so leave it off for motion wake-up.
    ///
    /// [`Sensor`]: zigbee_sensor::blocking::Sensor
    pub data_ready: bool,
}

impl Config {
    /// Low power, ±2 g, 12.5 Hz, no DRDY reporting.
    pub const DEFAULT: Self = Self {
        mode: PowerMode::LowPower,
        range: Range::G2,
        rate: DataRate::Hz12_5,
        data_ready: false,
    };

    const fn high_resolution(&self) -> bool {
        matches!(self.mode, PowerMode::HighResolution) || self.rate.forces_high_resolution()
    }

    /// Output resolution in bits.
    pub const fn resolution_bits(&self) -> u8 {
        if !self.high_resolution() {
            8
        } else if matches!(self.range, Range::G8 | Range::G16) {
            14
        } else {
            12
        }
    }

    // RES, GSEL1/GSEL0, EN16G. GSEL = 11 selects the 14-bit ±8/16 g mode.
    const fn ctrl1(&self) -> u8 {
        let hr = self.high_resolution();
        let g = match self.range {
            Range::G2 => 0x00,
            Range::G4 => 0x08,
            Range::G8 if hr => 0x18,
            Range::G8 => 0x10,
            Range::G16 if hr => 0x1C,
            Range::G16 => 0x04,
        };
        let res = if hr { CTRL1_RES } else { 0 };
        let drdy = if self.data_ready { CTRL1_DRDYE } else { 0 };
        res | drdy | g
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Wait after setting PC1: start-up time ~1/ODR (+1 ms margin).
pub const fn turn_on_us(rate: DataRate) -> WaitUs {
    rate.period_us() + 1_000
}

/// Wake-up engine sample rate (CTRL_REG2.OWUF).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum WakeUpRate {
    /// 0.781 Hz.
    Hz0_781 = 0,
    /// 1.563 Hz.
    Hz1_563 = 1,
    /// 3.125 Hz.
    Hz3_125 = 2,
    /// 6.25 Hz.
    #[default]
    Hz6_25 = 3,
    /// 12.5 Hz.
    Hz12_5 = 4,
    /// 25 Hz.
    Hz25 = 5,
    /// 50 Hz.
    Hz50 = 6,
    /// 100 Hz.
    Hz100 = 7,
}

/// INT pin behaviour (INT_CTRL_REG2.ULMODE, INT_CTRL_REG1.IEL).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum IntMode {
    /// Pin and status held until INT_REL is read.
    #[default]
    Latched,
    /// 30–50 µs pulse; status held until INT_REL is read.
    Pulsed,
    /// Pin and WUFS auto-clear after 1/OWUF.
    Unlatched,
}

/// Motion directions that may trigger the wake-up interrupt
/// (INT_CTRL_REG2 bits 5..0).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Directions(pub u8);

impl Directions {
    /// All six directions.
    pub const ALL: Self = Self(0x3F);
    /// X−.
    pub const X_NEG: Self = Self(0x20);
    /// X+.
    pub const X_POS: Self = Self(0x10);
    /// Y−.
    pub const Y_NEG: Self = Self(0x08);
    /// Y+.
    pub const Y_POS: Self = Self(0x04);
    /// Z−.
    pub const Z_NEG: Self = Self(0x02);
    /// Z+.
    pub const Z_POS: Self = Self(0x01);

    /// Union of two sets.
    pub const fn or(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// True if any direction of `other` is set.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 != 0
    }
}

/// Wake-up (motion detect) interrupt on the INT pin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WakeUpConfig {
    /// Change from the reference sample, 3.9 mg steps, range-independent
    /// (max 15996 mg).
    pub threshold_mg: u16,
    /// Engine sample rate; counters below count these periods.
    pub rate: WakeUpRate,
    /// WAKEUP_COUNTER: periods motion must last (1–255).
    pub duration: u8,
    /// NA_COUNTER: periods of no motion before the next event (1–255).
    pub non_activity: u8,
    /// Directions that trigger.
    pub directions: Directions,
    /// Pin behaviour.
    pub mode: IntMode,
    /// INT active high (reset value) instead of active low.
    pub active_high: bool,
}

impl Default for WakeUpConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 500,
            rate: WakeUpRate::Hz6_25,
            duration: 1,
            non_activity: 1,
            directions: Directions::ALL,
            mode: IntMode::Latched,
            active_high: true,
        }
    }
}

/// INT_SOURCE1 and INT_SOURCE2.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InterruptSource {
    /// INT_SOURCE1.
    pub source1: u8,
    /// INT_SOURCE2 (motion directions).
    pub source2: u8,
}

impl InterruptSource {
    /// Wake-up (WUFS).
    pub const fn wake_up(self) -> bool {
        self.source1 & SRC1_WUFS != 0
    }

    /// New data (DRDY).
    pub const fn data_ready(self) -> bool {
        self.source1 & SRC1_DRDY != 0
    }

    /// Directions that caused the wake-up.
    pub const fn directions(self) -> Directions {
        Directions(self.source2 & 0x3F)
    }
}

/// One acceleration sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Measurement {
    /// Acceleration in milli-g.
    pub acceleration: Vector3,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Acceleration(self.acceleration));
    }
}

/// Convert one left-justified output word to mg, discarding bits below
/// `bits` resolution (truncated toward zero).
pub const fn raw_to_mg(raw: i16, bits: u8, range: Range) -> i32 {
    let v = (raw as i32) & !((1 << (16 - bits)) - 1);
    v * 1000 * range.g() / 32768
}

/// Decode XOUT_L..ZOUT_H to milli-g.
pub fn decode(frame: &[u8; 6], bits: u8, range: Range) -> Vector3 {
    let axis = |i: usize| raw_to_mg(i16::from_le_bytes([frame[i], frame[i + 1]]), bits, range);
    Vector3 {
        x: axis(0),
        y: axis(2),
        z: axis(4),
    }
}

/// 12-bit WAKEUP_THRESHOLD value: 256 counts/g (rounded, at most 0xFFF).
pub fn wake_up_threshold_reg(threshold_mg: u16) -> u16 {
    ((u32::from(threshold_mg) * 256 + 500) / 1000).min(0xFFF) as u16
}

macro_rules! driver {
    ($I2c:path, $Sensor:path, $LowPower:path, [$($async:tt)?], [$($await:tt)*]) => {
        /// KXTJ3 driver.
        pub struct Kxtj3<I2C> {
            i2c: I2C,
            address: u8,
            config: Config,
            ctrl1: u8,
            active: bool,
        }

        impl<I2C> Kxtj3<I2C> {
            /// Driver on an I2C bus ([`ADDRESS_ADDR_LOW`] or
            /// [`ADDRESS_ADDR_HIGH`]); no bus traffic.
            pub const fn new(i2c: I2C, address: u8) -> Self {
                Self {
                    i2c,
                    address,
                    config: Config::DEFAULT,
                    ctrl1: 0,
                    active: false,
                }
            }

            /// Configuration last applied with [`Self::configure`].
            pub const fn config(&self) -> Config {
                self.config
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: $I2c> Kxtj3<I2C> {
            $($async)? fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
                self.i2c.write_read(self.address, &[reg], buf)$($await)*.map_err(Error::bus)
            }

            $($async)? fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c.write(self.address, &[reg, value])$($await)*.map_err(Error::bus)
            }

            /// Check WHO_AM_I. The part NACKs for 12–30 ms after power-up.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let mut id = [0u8];
                self.read_regs(REG_WHO_AM_I, &mut id)$($await)*?;
                if id[0] != WHO_AM_I {
                    return Err(Error::UnexpectedId(u32::from(id[0])));
                }
                Ok(())
            }

            /// Apply the configuration and start sampling; returns
            /// [`turn_on_us`]. A configured wake-up function stays enabled.
            pub $($async)? fn configure(&mut self, config: Config) -> Result<WaitUs, Error<I2C::Error>> {
                self.ctrl1 = (self.ctrl1 & CTRL1_WUFE) | config.ctrl1();
                self.config = config;
                self.write_reg(REG_CTRL_REG1, self.ctrl1)$($await)*?;
                self.active = false;
                self.write_reg(REG_DATA_CTRL, config.rate as u8)$($await)*?;
                self.power_up()$($await)*
            }

            $($async)? fn power_up(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_reg(REG_CTRL_REG1, self.ctrl1 | CTRL1_PC1)$($await)*?;
                self.active = true;
                Ok(turn_on_us(self.config.rate))
            }

            // Control registers are writable only with PC1 = 0.
            $($async)? fn standby_if_active(&mut self) -> Result<bool, Error<I2C::Error>> {
                let was = self.active;
                if was {
                    self.write_reg(REG_CTRL_REG1, self.ctrl1)$($await)*?;
                    self.active = false;
                }
                Ok(was)
            }

            $($async)? fn finish(&mut self, was_active: bool) -> Result<(), Error<I2C::Error>> {
                if was_active {
                    self.power_up()$($await)*?;
                } else {
                    self.write_reg(REG_CTRL_REG1, self.ctrl1)$($await)*?;
                }
                Ok(())
            }

            /// Enable the wake-up function and the INT pin. Fails with
            /// [`Error::Unsupported`] if a counter is zero.
            pub $($async)? fn configure_wake_up(&mut self, config: &WakeUpConfig) -> Result<(), Error<I2C::Error>> {
                if config.duration == 0 || config.non_activity == 0 {
                    return Err(Error::Unsupported);
                }
                let thr = wake_up_threshold_reg(config.threshold_mg);
                let was = self.standby_if_active()$($await)*?;
                self.write_reg(REG_CTRL_REG2, config.rate as u8)$($await)*?;
                self.write_reg(REG_WAKEUP_COUNTER, config.duration)$($await)*?;
                self.write_reg(REG_NA_COUNTER, config.non_activity)$($await)*?;
                self.write_reg(REG_WAKEUP_THRESHOLD_H, (thr >> 4) as u8)$($await)*?;
                self.write_reg(REG_WAKEUP_THRESHOLD_L, ((thr & 0x0F) << 4) as u8)$($await)*?;
                let ul = if config.mode == IntMode::Unlatched { INT_CTRL2_ULMODE } else { 0 };
                self.write_reg(REG_INT_CTRL_REG2, ul | (config.directions.0 & 0x3F))$($await)*?;
                let iel = if config.mode == IntMode::Pulsed { INT_CTRL1_IEL } else { 0 };
                let iea = if config.active_high { INT_CTRL1_IEA } else { 0 };
                self.write_reg(REG_INT_CTRL_REG1, INT_CTRL1_IEN | iea | iel)$($await)*?;
                self.ctrl1 |= CTRL1_WUFE;
                self.finish(was)$($await)*
            }

            /// Disable the wake-up function and the INT pin.
            pub $($async)? fn disable_wake_up(&mut self) -> Result<(), Error<I2C::Error>> {
                let was = self.standby_if_active()$($await)*?;
                self.write_reg(REG_INT_CTRL_REG1, INT_CTRL1_IEA)$($await)*?;
                self.ctrl1 &= !CTRL1_WUFE;
                self.finish(was)$($await)*
            }

            /// Read INT_SOURCE1/2, then INT_REL to clear them and release
            /// a latched INT pin.
            pub $($async)? fn read_interrupt_source(&mut self) -> Result<InterruptSource, Error<I2C::Error>> {
                let mut src = [0u8; 2];
                self.read_regs(REG_INT_SOURCE1, &mut src)$($await)*?;
                self.read_regs(REG_INT_REL, &mut [0u8])$($await)*?;
                Ok(InterruptSource { source1: src[0], source2: src[1] })
            }
        }

        impl<I2C: $I2c> $Sensor for Kxtj3<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Leave stand-by if needed (returns 1/ODR + 1 ms), else `0`.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.active {
                    return Ok(0);
                }
                self.power_up()$($await)*
            }

            /// With [`Config::data_ready`] checks INT_SOURCE1.DRDY first;
            /// reading the outputs clears it.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                if self.config.data_ready {
                    let mut src = [0u8];
                    self.read_regs(REG_INT_SOURCE1, &mut src)$($await)*?;
                    if src[0] & SRC1_DRDY == 0 {
                        return Err(Error::NotReady);
                    }
                }
                let mut frame = [0u8; 6];
                self.read_regs(REG_XOUT_L, &mut frame)$($await)*?;
                let bits = self.config.resolution_bits();
                Ok(Measurement { acceleration: decode(&frame, bits, self.config.range) })
            }
        }

        impl<I2C: $I2c> $LowPower for Kxtj3<I2C> {
            type BusError = I2C::Error;

            /// Stand-by (0.9 µA): no sampling and no wake-up detection.
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_reg(REG_CTRL_REG1, self.ctrl1)$($await)*?;
                self.active = false;
                Ok(())
            }

            /// Back to operating mode; returns 1/ODR + 1 ms.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.power_up()$($await)*
            }
        }
    };
}

driver!(
    embedded_hal::i2c::I2c,
    zigbee_sensor::blocking::Sensor,
    zigbee_sensor::blocking::LowPower,
    [],
    []
);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        embedded_hal_async::i2c::I2c,
        zigbee_sensor::asynch::Sensor,
        zigbee_sensor::asynch::LowPower,
        [async],
        [.await]
    );
}

#[cfg(test)]
mod tests;
