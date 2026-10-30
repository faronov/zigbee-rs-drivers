//! MEMSIC MC3419 / MC3479 16-bit three-axis accelerometers (I2C or SPI).
//!
//! The parts sample continuously in WAKE state; there is no single-shot or
//! low-power sampling mode. [`Sensor::start_measurement`] leaves STANDBY
//! (returns 1/ODR + 1 ms) and [`Sensor::read_measurement`] returns
//! [`Error::NotReady`] until `STATUS.NEW_DATA` is set. [`LowPower::sleep`]
//! is STANDBY (4 µA, no motion detection).
//!
//! For a motion wake-up keep the part in WAKE at a low rate with the
//! AnyMotion interrupt ([`MotionConfig`]) enabled (77 µA at 100 Hz).
//!
//! The two parts share the register map and CHIP_ID; they differ in the
//! sample-rate codes, selected with [`Part`].
//!
//! References: MEMSIC "MC3419 3-Axis Accelerometer" APS-048-0071 v1.1 and
//! "MC3479 3-Axis Accelerometer" APS-048-0072 v1.12.
//!
//! [`Sensor::start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
//! [`LowPower::sleep`]: zigbee_sensor::blocking::LowPower::sleep
//!
//! ```ignore
//! let mut acc = Mc3419::new_i2c(i2c, ADDRESS_A6_LOW);
//! acc.probe()?;
//! acc.configure(Config { rate: DataRate::Hz25, ..Default::default() })?;
//! acc.configure_motion(&MotionConfig { threshold_mg: 100, ..Default::default() })?;
//! // ... sleep until INTN1, then:
//! let src = acc.read_interrupt_source()?; // also clears the latch
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with DOUT_A6 low at power-up.
pub const ADDRESS_A6_LOW: u8 = 0x4C;
/// I2C address with DOUT_A6 high at power-up.
pub const ADDRESS_A6_HIGH: u8 = 0x6C;

/// CHIP_ID register value.
pub const CHIP_ID: u8 = 0xA4;

const REG_DEV_STAT: u8 = 0x05;
const REG_INTR_CTRL: u8 = 0x06;
const REG_MODE: u8 = 0x07;
const REG_SR: u8 = 0x08;
const REG_MOTION_CTRL: u8 = 0x09;
const REG_XOUT_L: u8 = 0x0D;
const REG_INTR_STAT: u8 = 0x14;
const REG_CHIP_ID: u8 = 0x18;
const REG_RANGE: u8 = 0x20;
const REG_COMM_CTRL: u8 = 0x31;
const REG_GPIO_CTRL: u8 = 0x33;
const REG_AM_THRESH_LSB: u8 = 0x43;
const REG_AM_THRESH_MSB: u8 = 0x44;
const REG_AM_DB: u8 = 0x45;

const MODE_STANDBY: u8 = 0x00;
const MODE_WAKE: u8 = 0x01;
const STATUS_NEW_DATA: u8 = 0x80;
const INTR_CTRL_AUTO_CLR: u8 = 0x40;
const MOTION_ANYM_EN: u8 = 0x04;
const COMM_INT_SWAP: u8 = 0x10;
const LPF_EN: u8 = 0x08;

const INT_TILT: u8 = 0x01;
const INT_FLIP: u8 = 0x02;
const INT_ANYM: u8 = 0x04;
const INT_SHAKE: u8 = 0x08;
const INT_TILT_35: u8 = 0x10;
const INT_FIFO: u8 = 0x20;
const INT_ACQ: u8 = 0x80;

/// Part variant (CHIP_ID does not distinguish them).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Part {
    /// MC3419: 25–1000 Hz.
    #[default]
    Mc3419,
    /// MC3479: 50–2000 Hz.
    Mc3479,
}

/// Full-scale range (RANGE register bits 6..4).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Range {
    /// ±2 g, 16384 LSB/g.
    #[default]
    G2 = 0,
    /// ±4 g, 8192 LSB/g.
    G4 = 1,
    /// ±8 g, 4096 LSB/g.
    G8 = 2,
    /// ±16 g, 2048 LSB/g.
    G16 = 3,
    /// ±12 g, 2730 LSB/g.
    G12 = 4,
}

impl Range {
    /// Sensitivity in LSB/g.
    pub const fn lsb_per_g(self) -> i32 {
        match self {
            Range::G2 => 16384,
            Range::G4 => 8192,
            Range::G8 => 4096,
            Range::G12 => 2730,
            Range::G16 => 2048,
        }
    }
}

/// Output data rate. Not every rate exists on both parts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DataRate {
    /// 25 Hz (MC3419 only).
    Hz25,
    /// 50 Hz.
    #[default]
    Hz50,
    /// 62.5 Hz (MC3419 only).
    Hz62_5,
    /// 100 Hz.
    Hz100,
    /// 125 Hz.
    Hz125,
    /// 200 Hz (MC3479 only).
    Hz200,
    /// 250 Hz.
    Hz250,
    /// 500 Hz.
    Hz500,
    /// 1000 Hz.
    Hz1000,
    /// 2000 Hz (MC3479 only).
    Hz2000,
}

impl DataRate {
    /// Sample period in µs.
    pub const fn period_us(self) -> u32 {
        match self {
            DataRate::Hz25 => 40_000,
            DataRate::Hz50 => 20_000,
            DataRate::Hz62_5 => 16_000,
            DataRate::Hz100 => 10_000,
            DataRate::Hz125 => 8_000,
            DataRate::Hz200 => 5_000,
            DataRate::Hz250 => 4_000,
            DataRate::Hz500 => 2_000,
            DataRate::Hz1000 => 1_000,
            DataRate::Hz2000 => 500,
        }
    }
}

/// SR register value for `rate`, or `None` if the part lacks it.
///
/// MC3419 uses table "Selection A" (I2C, or SPI at 4 MHz or less).
pub const fn sample_rate_reg(part: Part, rate: DataRate) -> Option<u8> {
    match part {
        Part::Mc3419 => match rate {
            DataRate::Hz25 => Some(0x10),
            DataRate::Hz50 => Some(0x11),
            DataRate::Hz62_5 => Some(0x12),
            DataRate::Hz100 => Some(0x13),
            DataRate::Hz125 => Some(0x14),
            DataRate::Hz250 => Some(0x15),
            DataRate::Hz500 => Some(0x16),
            DataRate::Hz1000 => Some(0x17),
            DataRate::Hz200 | DataRate::Hz2000 => None,
        },
        Part::Mc3479 => match rate {
            DataRate::Hz50 => Some(0x08),
            DataRate::Hz100 => Some(0x09),
            DataRate::Hz125 => Some(0x0A),
            DataRate::Hz200 => Some(0x0B),
            DataRate::Hz250 => Some(0x0C),
            DataRate::Hz500 => Some(0x0D),
            DataRate::Hz1000 => Some(0x0E),
            DataRate::Hz2000 => Some(0x0F),
            DataRate::Hz25 | DataRate::Hz62_5 => None,
        },
    }
}

/// Low-pass filter corner (RANGE register LPF bits).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LowPass {
    /// Fc = ODR / 4.255.
    Div4_255 = 1,
    /// Fc = ODR / 6.
    Div6 = 2,
    /// Fc = ODR / 12.
    Div12 = 3,
    /// Fc = ODR / 16.
    Div16 = 5,
}

/// Sampling and interrupt-pin configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Part variant (selects sample-rate codes).
    pub part: Part,
    /// Full-scale range.
    pub range: Range,
    /// Output data rate; also the AnyMotion evaluation rate.
    pub rate: DataRate,
    /// Optional low-pass filter.
    pub low_pass: Option<LowPass>,
    /// Drive both INTN pins push-pull instead of open-drain.
    pub int_push_pull: bool,
    /// Both INTN pins active high instead of active low.
    pub int_active_high: bool,
}

impl Config {
    /// Power-on defaults: MC3419, ±2 g, 50 Hz, no LPF, open-drain
    /// active-low INTN pins.
    pub const DEFAULT: Self = Self {
        part: Part::Mc3419,
        range: Range::G2,
        rate: DataRate::Hz50,
        low_pass: None,
        int_push_pull: false,
        int_active_high: false,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Wait after entering WAKE: start-up time 1/ODR + 1 ms.
pub const fn turn_on_us(rate: DataRate) -> WaitUs {
    rate.period_us() + 1_000
}

/// Interrupt pin.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum IntPin {
    /// INTN1.
    #[default]
    Int1,
    /// INTN2 (via INT1_INT2_REQ_SWAP).
    Int2,
}

/// AnyMotion interrupt: fires when an axis changes by more than the
/// threshold between consecutive samples.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotionConfig {
    /// Sample-to-sample change in mg.
    pub threshold_mg: u16,
    /// AM_DB: samples without motion before an auto-cleared interrupt
    /// drops (only used when `latch` is false).
    pub debounce: u8,
    /// Output pin. Data-ready and FIFO interrupts move with it.
    pub pin: IntPin,
    /// Keep the interrupt until [`InterruptSource`] is read and cleared;
    /// otherwise clear it when motion stops (AUTO_CLR_EN).
    pub latch: bool,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 250,
            debounce: 0,
            pin: IntPin::Int1,
            latch: true,
        }
    }
}

/// INTR_STAT bits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InterruptSource(pub u8);

impl InterruptSource {
    /// AnyMotion.
    pub const fn any_motion(self) -> bool {
        self.0 & INT_ANYM != 0
    }

    /// Shake.
    pub const fn shake(self) -> bool {
        self.0 & INT_SHAKE != 0
    }

    /// Tilt.
    pub const fn tilt(self) -> bool {
        self.0 & INT_TILT != 0
    }

    /// Flip.
    pub const fn flip(self) -> bool {
        self.0 & INT_FLIP != 0
    }

    /// Tilt-35.
    pub const fn tilt_35(self) -> bool {
        self.0 & INT_TILT_35 != 0
    }

    /// Any FIFO interrupt.
    pub const fn fifo(self) -> bool {
        self.0 & INT_FIFO != 0
    }

    /// New sample (ACQ_INT).
    pub const fn data_ready(self) -> bool {
        self.0 & INT_ACQ != 0
    }
}

/// Operating state from DEV_STAT.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Clocks stopped, registers writable.
    Standby,
    /// Sampling.
    Wake,
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

/// Convert one 16-bit sample to mg (truncated toward zero).
pub const fn raw_to_mg(raw: i16, range: Range) -> i32 {
    raw as i32 * 1000 / range.lsb_per_g()
}

/// Decode XOUT_EX_L..ZOUT_EX_H to milli-g.
pub fn decode(frame: &[u8; 6], range: Range) -> Vector3 {
    let axis = |i: usize| raw_to_mg(i16::from_le_bytes([frame[i], frame[i + 1]]), range);
    Vector3 {
        x: axis(0),
        y: axis(2),
        z: axis(4),
    }
}

/// 15-bit AnyMotion threshold in output LSBs (rounded).
pub fn any_motion_threshold_reg(threshold_mg: u16, range: Range) -> u16 {
    let lsb = (u32::from(threshold_mg) * range.lsb_per_g() as u32 + 500) / 1000;
    lsb.min(0x7FFF) as u16
}

macro_rules! driver {
    ($Iface:path, $Sensor:path, $LowPower:path, [$($async:tt)?], [$($await:tt)*]) => {
        /// MC3419 / MC3479 driver over a register [`Interface`].
        pub struct Mc3419<IFACE> {
            iface: IFACE,
            config: Config,
            active: bool,
        }

        impl<IFACE> Mc3419<IFACE> {
            /// Create a driver over a register interface; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    config: Config::DEFAULT,
                    active: false,
                }
            }

            /// Configuration last applied with [`Self::configure`].
            pub const fn config(&self) -> Config {
                self.config
            }

            /// Give the interface back.
            pub fn release(self) -> IFACE {
                self.iface
            }
        }

        impl<I2C> Mc3419<I2cInterface<I2C>> {
            /// Driver on an I2C bus ([`ADDRESS_A6_LOW`] or
            /// [`ADDRESS_A6_HIGH`]).
            pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }
        }

        impl<SPI> Mc3419<SpiInterface<SPI>> {
            /// Driver on an SPI device (4-wire; at most 4 MHz on MC3419,
            /// see [`sample_rate_reg`]).
            pub const fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }
        }

        impl<IFACE: $Iface> Mc3419<IFACE> {
            $($async)? fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
                self.iface.read_regs(reg, buf)$($await)*.map_err(Error::bus)
            }

            $($async)? fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
                self.iface.write_reg(reg, value)$($await)*.map_err(Error::bus)
            }

            /// Check CHIP_ID.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<IFACE::Error>> {
                let mut id = [0u8];
                self.read_regs(REG_CHIP_ID, &mut id)$($await)*?;
                if id[0] != CHIP_ID {
                    return Err(Error::UnexpectedId(u32::from(id[0])));
                }
                Ok(())
            }

            /// Apply the configuration and enter WAKE; returns
            /// [`turn_on_us`]. Fails with [`Error::Unsupported`] if the part
            /// lacks `config.rate`.
            pub $($async)? fn configure(&mut self, config: Config) -> Result<WaitUs, Error<IFACE::Error>> {
                let sr = sample_rate_reg(config.part, config.rate).ok_or(Error::Unsupported)?;
                self.write_reg(REG_MODE, MODE_STANDBY)$($await)*?;
                self.active = false;
                self.config = config;
                self.write_reg(REG_SR, sr)$($await)*?;
                let lpf = match config.low_pass {
                    Some(lp) => LPF_EN | lp as u8,
                    None => 0,
                };
                self.write_reg(REG_RANGE, (config.range as u8) << 4 | lpf)$($await)*?;
                let pin = (config.int_push_pull as u8) << 1 | config.int_active_high as u8;
                self.write_reg(REG_GPIO_CTRL, pin << 6 | pin << 2)$($await)*?;
                self.power_up()$($await)*
            }

            $($async)? fn power_up(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                self.write_reg(REG_MODE, MODE_WAKE)$($await)*?;
                self.active = true;
                Ok(turn_on_us(self.config.rate))
            }

            // Registers other than MODE and INTR_STAT are writable only in
            // STANDBY.
            $($async)? fn standby_if_active(&mut self) -> Result<bool, Error<IFACE::Error>> {
                let was = self.active;
                if was {
                    self.write_reg(REG_MODE, MODE_STANDBY)$($await)*?;
                    self.active = false;
                }
                Ok(was)
            }

            $($async)? fn restore(&mut self, was_active: bool) -> Result<(), Error<IFACE::Error>> {
                if was_active {
                    self.power_up()$($await)*?;
                }
                Ok(())
            }

            /// Enable the AnyMotion interrupt. The threshold is converted
            /// with the configured range, so call this after
            /// [`Self::configure`]. Detection runs only in WAKE state.
            pub $($async)? fn configure_motion(&mut self, config: &MotionConfig) -> Result<(), Error<IFACE::Error>> {
                let thr = any_motion_threshold_reg(config.threshold_mg, self.config.range);
                let was = self.standby_if_active()$($await)*?;
                self.write_reg(REG_AM_THRESH_LSB, thr as u8)$($await)*?;
                self.write_reg(REG_AM_THRESH_MSB, (thr >> 8) as u8)$($await)*?;
                self.write_reg(REG_AM_DB, config.debounce)$($await)*?;
                self.write_reg(REG_MOTION_CTRL, MOTION_ANYM_EN)$($await)*?;
                let swap = if config.pin == IntPin::Int2 { COMM_INT_SWAP } else { 0 };
                self.write_reg(REG_COMM_CTRL, swap)$($await)*?;
                let clr = if config.latch { 0 } else { INTR_CTRL_AUTO_CLR };
                self.write_reg(REG_INTR_CTRL, INT_ANYM | clr)$($await)*?;
                self.restore(was)$($await)*
            }

            /// Disable all motion and sample interrupts.
            pub $($async)? fn disable_motion(&mut self) -> Result<(), Error<IFACE::Error>> {
                let was = self.standby_if_active()$($await)*?;
                self.write_reg(REG_INTR_CTRL, 0)$($await)*?;
                self.write_reg(REG_MOTION_CTRL, 0)$($await)*?;
                self.restore(was)$($await)*
            }

            /// Read INTR_STAT and, if anything is pending, clear all
            /// interrupts (a write to INTR_STAT). Works only in WAKE state.
            pub $($async)? fn read_interrupt_source(&mut self) -> Result<InterruptSource, Error<IFACE::Error>> {
                let mut stat = [0u8];
                self.read_regs(REG_INTR_STAT, &mut stat)$($await)*?;
                if stat[0] != 0 {
                    self.write_reg(REG_INTR_STAT, 0)$($await)*?;
                }
                Ok(InterruptSource(stat[0]))
            }

            /// Operating state from DEV_STAT.
            pub $($async)? fn state(&mut self) -> Result<State, Error<IFACE::Error>> {
                let mut stat = [0u8];
                self.read_regs(REG_DEV_STAT, &mut stat)$($await)*?;
                Ok(if stat[0] & 0x03 == MODE_WAKE { State::Wake } else { State::Standby })
            }
        }

        impl<IFACE: $Iface> $Sensor for Mc3419<IFACE> {
            type BusError = IFACE::Error;
            type Measurement = Measurement;

            /// Enter WAKE if needed (returns 1/ODR + 1 ms), else `0`.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                if self.active {
                    return Ok(0);
                }
                self.power_up()$($await)*
            }

            /// Reads XOUT..ZOUT and STATUS in one burst.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
                let mut buf = [0u8; 7];
                self.read_regs(REG_XOUT_L, &mut buf)$($await)*?;
                if buf[6] & STATUS_NEW_DATA == 0 {
                    return Err(Error::NotReady);
                }
                let mut frame = [0u8; 6];
                frame.copy_from_slice(&buf[..6]);
                Ok(Measurement { acceleration: decode(&frame, self.config.range) })
            }
        }

        impl<IFACE: $Iface> $LowPower for Mc3419<IFACE> {
            type BusError = IFACE::Error;

            /// STANDBY (4 µA): no sampling and no motion detection.
            $($async)? fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_MODE, MODE_STANDBY)$($await)*?;
                self.active = false;
                Ok(())
            }

            /// Back to WAKE; returns 1/ODR + 1 ms.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                self.power_up()$($await)*
            }
        }
    };
}

driver!(
    Interface,
    zigbee_sensor::blocking::Sensor,
    zigbee_sensor::blocking::LowPower,
    [],
    []
);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        AsyncInterface,
        zigbee_sensor::asynch::Sensor,
        zigbee_sensor::asynch::LowPower,
        [async],
        [.await]
    );
}

#[cfg(test)]
mod tests;
