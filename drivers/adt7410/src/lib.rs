//! Analog Devices ADT7410 / ADT7420 ±0.5 / ±0.25 °C digital temperature
//! sensor (I2C) with INT and CT outputs.
//!
//! [`configure`](Adt7410::configure) puts the part in shutdown (2 µA typ.)
//! and every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! writes the one-shot mode; after the conversion the part shuts down by
//! itself.
//!
//! References: Analog Devices ADT7410 data sheet Rev. C and ADT7420 data
//! sheet Rev. A (identical register map and ID).
//!
//! ```ignore
//! let mut t = Adt7410::new(i2c, address(false, false));
//! t.probe()?; // ID register, manufacturer bits 0b11001
//! t.configure()?; // shutdown, 16-bit
//! let wait_us = t.start_measurement()?;
//! // ... sleep for wait_us ...
//! let m = t.read_measurement()?;
//! ```
//!
//! The blocking driver and its [`asynch`] twin are generated from one macro
//! body, so they issue identical bus transactions.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// ID register manufacturer bits (`ID[7:3]` = 0b11001); `ID[2:0]` is the
/// silicon revision (ADT7420: 0xCB).
pub const MANUFACTURER_ID: u8 = 0xC8;

/// One-shot / continuous conversion time: 240 ms typ., no maximum given,
/// so the driver waits 1.5 × typ.
pub const CONVERSION_US: WaitUs = 360_000;

/// 1 SPS mode conversion time: 60 ms typ., waited as 1.5 × typ.
pub const CONVERSION_1SPS_US: WaitUs = 90_000;

/// Time the part ignores the bus after a software reset.
pub const RESET_WAIT_US: WaitUs = 200;

const REG_TEMP: u8 = 0x00;
const REG_STATUS: u8 = 0x02;
const REG_CONFIG: u8 = 0x03;
const REG_T_HIGH: u8 = 0x04;
const REG_T_LOW: u8 = 0x06;
const REG_T_CRIT: u8 = 0x08;
const REG_T_HYST: u8 = 0x0A;
const REG_ID: u8 = 0x0B;
const CMD_RESET: u8 = 0x2F;

const MODE_CONTINUOUS: u8 = 0b00 << 5;
const MODE_ONE_SHOT: u8 = 0b01 << 5;
const MODE_1SPS: u8 = 0b10 << 5;
const MODE_SHUTDOWN: u8 = 0b11 << 5;

/// 7-bit address `0b10010 A1 A0` (0x48 … 0x4B).
pub const fn address(a1: bool, a0: bool) -> u8 {
    0x48 | ((a1 as u8) << 1) | a0 as u8
}

/// ADC resolution (configuration bit 7).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// Sign + 12 bit, 0.0625 °C; the three LSBs of the result carry the
    /// TLOW / THIGH / TCRIT flags (power-on default).
    Bits13,
    /// Sign + 15 bit, 0.0078 °C.
    #[default]
    Bits16,
}

/// Number of consecutive faults before INT / CT activate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FaultQueue {
    /// 1 fault (power-on default).
    #[default]
    One,
    /// 2 faults.
    Two,
    /// 3 faults.
    Three,
    /// 4 faults.
    Four,
}

/// INT / CT output behaviour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AlertMode {
    /// Latched until any register is read (power-on default).
    #[default]
    Interrupt,
    /// Follows the temperature with hysteresis; holds through shutdown.
    Comparator,
}

/// Mode used by [`LowPower::wake`](zigbee_sensor::blocking::LowPower::wake).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RunMode {
    /// Back-to-back conversions (210 µA typ.).
    #[default]
    Continuous,
    /// One 60 ms conversion per second.
    OneSps,
}

/// Configuration written by [`Adt7410::configure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// ADC resolution (driver default 16 bit; the chip powers up in 13 bit).
    pub resolution: Resolution,
    /// Fault queue for INT and CT.
    pub fault_queue: FaultQueue,
    /// INT / CT mode.
    pub alert_mode: AlertMode,
    /// INT active high instead of active low.
    pub int_active_high: bool,
    /// CT active high instead of active low.
    pub ct_active_high: bool,
    /// Mode entered by `wake`.
    pub run_mode: RunMode,
}

impl Config {
    /// Configuration register byte with operation mode bits `mode`
    /// (already shifted into bits 6:5).
    const fn register(self, mode: u8) -> u8 {
        let mut byte = mode | self.fault_queue as u8;
        if matches!(self.resolution, Resolution::Bits16) {
            byte |= 1 << 7;
        }
        if matches!(self.alert_mode, AlertMode::Comparator) {
            byte |= 1 << 4;
        }
        if self.int_active_high {
            byte |= 1 << 3;
        }
        if self.ct_active_high {
            byte |= 1 << 2;
        }
        byte
    }
}

/// Status register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    /// A new result has been written since the last temperature read.
    pub ready: bool,
    /// Above TCRIT.
    pub critical: bool,
    /// Above THIGH.
    pub high: bool,
    /// Below TLOW.
    pub low: bool,
}

impl Status {
    /// Decode the status byte (`RDY` is active low).
    pub const fn from_bits(bits: u8) -> Self {
        Self {
            ready: bits & 0x80 == 0,
            critical: bits & 0x40 != 0,
            high: bits & 0x20 != 0,
            low: bits & 0x10 != 0,
        }
    }
}

/// One temperature sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
    }
}

/// Decode the temperature register in 0.01 °C. In 13-bit mode the three
/// flag bits are dropped.
pub const fn decode_temperature(raw: u16, resolution: Resolution) -> i32 {
    match resolution {
        Resolution::Bits13 => {
            let counts = ((raw as i16) >> 3) as i32;
            (counts * 100 + 8) >> 4
        }
        Resolution::Bits16 => ((raw as i16 as i32) * 100 + 64) >> 7,
    }
}

/// Encode a setpoint in 0.01 °C (setpoints always use the 16-bit
/// 1/128 °C format), rounded and saturated to the register range.
pub const fn encode_setpoint(centi_celsius: i32) -> u16 {
    let counts = (centi_celsius * 128 + 50).div_euclid(100);
    let c = if counts < i16::MIN as i32 {
        i16::MIN as i32
    } else if counts > i16::MAX as i32 {
        i16::MAX as i32
    } else {
        counts
    };
    c as i16 as u16
}

/// Encode the hysteresis in 0.01 °C as whole degrees, 0 … 15 °C.
pub const fn encode_hysteresis(centi_celsius: i32) -> u8 {
    let c = (centi_celsius + 50).div_euclid(100);
    if c < 0 {
        0
    } else if c > 15 {
        15
    } else {
        c as u8
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver with the default [`Config`]; no bus traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                config: Config {
                    resolution: Resolution::Bits16,
                    fault_queue: FaultQueue::One,
                    alert_mode: AlertMode::Interrupt,
                    int_active_high: false,
                    ct_active_high: false,
                    run_mode: RunMode::Continuous,
                },
            }
        }

        /// Use `config` for [`Self::configure`] and later mode changes.
        pub const fn with_config(mut self, config: Config) -> Self {
            self.config = config;
            self
        }

        /// The active configuration.
        pub const fn config(&self) -> Config {
            self.config
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

macro_rules! driver_impl {
    ($i2c:path, $sensor:path, $low_power:path, $($async:ident)?; $($await:tt)*) => {
        impl<I2C: $i2c> Adt7410<I2C> {
            $($async)? fn write_config(&mut self, mode: u8) -> Result<(), Error<I2C::Error>> {
                let byte = self.config.register(mode);
                self.i2c
                    .write(self.address, &[REG_CONFIG, byte])
                    $($await)*
                    .map_err(Error::bus)
            }

            $($async)? fn read_reg(&mut self, register: u8) -> Result<u8, Error<I2C::Error>> {
                let mut buf = [0u8; 1];
                self.i2c
                    .write_read(self.address, &[register], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                Ok(buf[0])
            }

            $($async)? fn write_setpoint(&mut self, register: u8, centi: i32) -> Result<(), Error<I2C::Error>> {
                let [msb, lsb] = encode_setpoint(centi).to_be_bytes();
                self.i2c
                    .write(self.address, &[register, msb, lsb])
                    $($await)*
                    .map_err(Error::bus)
            }

            /// Check the manufacturer bits of the ID register; returns the
            /// silicon revision (`ID[2:0]`).
            pub $($async)? fn probe(&mut self) -> Result<u8, Error<I2C::Error>> {
                let id = self.read_reg(REG_ID) $($await)*?;
                if id & 0xF8 != MANUFACTURER_ID {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(id & 0x07)
            }

            /// Software reset (reloads power-on defaults, including 13-bit
            /// mode and continuous conversion). Returns the wait before the
            /// part acknowledges again; call [`Self::configure`] after it.
            pub $($async)? fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.i2c
                    .write(self.address, &[CMD_RESET])
                    $($await)*
                    .map_err(Error::bus)?;
                Ok(RESET_WAIT_US)
            }

            /// Write the configuration and enter shutdown.
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_config(MODE_SHUTDOWN) $($await)*
            }

            /// Change the configuration and write it (enters shutdown).
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program the INT window in 0.01 °C: active above `high` or
            /// below `low` (1/128 °C steps).
            pub $($async)? fn set_limits(&mut self, low: i32, high: i32) -> Result<(), Error<I2C::Error>> {
                self.write_setpoint(REG_T_HIGH, high) $($await)*?;
                self.write_setpoint(REG_T_LOW, low) $($await)*
            }

            /// Program the CT limit in 0.01 °C (power-on default 147 °C).
            pub $($async)? fn set_critical(&mut self, critical: i32) -> Result<(), Error<I2C::Error>> {
                self.write_setpoint(REG_T_CRIT, critical) $($await)*
            }

            /// Hysteresis for all limits in 0.01 °C, rounded to whole
            /// degrees, 0 … 15 °C (default 5 °C).
            pub $($async)? fn set_hysteresis(&mut self, hysteresis: i32) -> Result<(), Error<I2C::Error>> {
                let value = encode_hysteresis(hysteresis);
                self.i2c
                    .write(self.address, &[REG_T_HYST, value])
                    $($await)*
                    .map_err(Error::bus)
            }

            /// Read the status register; clears the limit flags and, in
            /// interrupt mode, releases INT / CT.
            pub $($async)? fn read_status(&mut self) -> Result<Status, Error<I2C::Error>> {
                Ok(Status::from_bits(self.read_reg(REG_STATUS) $($await)*?))
            }
        }

        impl<I2C: $i2c> $sensor for Adt7410<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot conversion; the part returns to shutdown afterwards.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_config(MODE_ONE_SHOT) $($await)*?;
                Ok(CONVERSION_US)
            }

            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(self.address, &[REG_TEMP], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                let temperature = decode_temperature(u16::from_be_bytes(buf), self.config.resolution);
                Ok(Measurement { temperature })
            }
        }

        impl<I2C: $i2c> $low_power for Adt7410<I2C> {
            type BusError = I2C::Error;

            /// Shutdown mode.
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_config(MODE_SHUTDOWN) $($await)*
            }

            /// Continuous or 1 SPS conversion per [`Config::run_mode`].
            /// Returns the wait until the first result.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let (mode, wait) = match self.config.run_mode {
                    RunMode::Continuous => (MODE_CONTINUOUS, CONVERSION_US),
                    RunMode::OneSps => (MODE_1SPS, CONVERSION_1SPS_US),
                };
                self.write_config(mode) $($await)*?;
                Ok(wait)
            }
        }
    };
}

/// Blocking driver.
pub struct Adt7410<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
}

impl<I2C> Adt7410<I2C> {
    constructors!();
}

/// Same register map and ID.
pub type Adt7420<I2C> = Adt7410<I2C>;

driver_impl!(
    embedded_hal::i2c::I2c,
    zigbee_sensor::blocking::Sensor,
    zigbee_sensor::blocking::LowPower,
    ;
);

/// Async driver.
pub mod asynch {
    use super::*;

    /// Async twin of [`super::Adt7410`].
    pub struct Adt7410<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
    }

    impl<I2C> Adt7410<I2C> {
        constructors!();
    }

    /// Same register map and ID.
    pub type Adt7420<I2C> = Adt7410<I2C>;

    driver_impl!(
        embedded_hal_async::i2c::I2c,
        zigbee_sensor::asynch::Sensor,
        zigbee_sensor::asynch::LowPower,
        async; .await
    );
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = 0x48;

    #[test]
    fn decodes_datasheet_formats() {
        use Resolution::*;
        // Table 5 (13-bit, value in bits 15:3; flag bits ignored).
        assert_eq!(decode_temperature(0x1C90 << 3, Bits13), -5_500);
        assert_eq!(decode_temperature(0x1FFF << 3, Bits13), -6);
        assert_eq!(decode_temperature((0x0190 << 3) | 0x7, Bits13), 2_500);
        assert_eq!(decode_temperature(0x0960 << 3, Bits13), 15_000);
        // 16-bit, 1/128 °C.
        assert_eq!(decode_temperature(0xE480, Bits16), -5_500);
        assert_eq!(decode_temperature(0x0C80, Bits16), 2_500);
        assert_eq!(decode_temperature(0x0001, Bits16), 1);
        assert_eq!(decode_temperature(0xFFFF, Bits16), -1);
    }

    #[test]
    fn encodes_setpoints_and_config() {
        // Power-on defaults: THIGH 64 °C, TLOW 10 °C, TCRIT 147 °C.
        assert_eq!(encode_setpoint(6_400), 0x2000);
        assert_eq!(encode_setpoint(1_000), 0x0500);
        assert_eq!(encode_setpoint(14_700), 0x4980);
        assert_eq!(encode_setpoint(-5_500), 0xE480);
        assert_eq!(encode_setpoint(100_000), 0x7FFF);
        assert_eq!(encode_hysteresis(500), 5);
        assert_eq!(encode_hysteresis(9_900), 15);
        assert_eq!(encode_hysteresis(-100), 0);
        assert_eq!(address(true, false), 0x4A);
        let c = Config {
            resolution: Resolution::Bits13,
            fault_queue: FaultQueue::Four,
            alert_mode: AlertMode::Comparator,
            int_active_high: true,
            ct_active_high: true,
            run_mode: RunMode::OneSps,
        };
        assert_eq!(c.register(MODE_SHUTDOWN), 0x7F);
        assert_eq!(Config::default().register(MODE_ONE_SHOT), 0xA0);
    }

    #[test]
    fn probe_configure_one_shot() {
        let expectations = [
            Transaction::write_read(A, vec![REG_ID], vec![0xCB]),
            Transaction::write_read(A, vec![REG_ID], vec![0x75]),
            Transaction::write(A, vec![CMD_RESET]),
            Transaction::write(A, vec![REG_CONFIG, 0xE0]),
            Transaction::write(A, vec![REG_CONFIG, 0xA0]),
            Transaction::write_read(A, vec![REG_STATUS], vec![0x60]),
            Transaction::write_read(A, vec![REG_TEMP], vec![0x0C, 0x80]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = Adt7410::new(&mut i2c, A);
        assert_eq!(t.probe(), Ok(3));
        assert_eq!(t.probe(), Err(Error::UnexpectedId(0x75)));
        assert_eq!(t.soft_reset().unwrap(), RESET_WAIT_US);
        t.configure().unwrap();
        assert_eq!(t.start_measurement().unwrap(), CONVERSION_US);
        assert_eq!(
            t.read_status().unwrap(),
            Status {
                ready: true,
                critical: true,
                high: true,
                low: false
            }
        );
        assert_eq!(t.read_measurement().unwrap().temperature, 2_500);
        i2c.done();
    }

    #[test]
    fn limits_and_power_modes() {
        let expectations = [
            Transaction::write(A, vec![REG_T_HIGH, 0x0F, 0x00]),
            Transaction::write(A, vec![REG_T_LOW, 0xFD, 0x80]),
            Transaction::write(A, vec![REG_T_CRIT, 0x28, 0x00]),
            Transaction::write(A, vec![REG_T_HYST, 0x02]),
            Transaction::write(A, vec![REG_CONFIG, 0x40]),
            Transaction::write(A, vec![REG_CONFIG, 0x60]),
            Transaction::write(A, vec![REG_CONFIG, 0x00]),
        ];
        let config = Config {
            resolution: Resolution::Bits13,
            run_mode: RunMode::OneSps,
            ..Config::default()
        };
        let mut i2c = Mock::new(&expectations);
        let mut t = Adt7410::new(&mut i2c, A).with_config(config);
        t.set_limits(-500, 3_000).unwrap();
        t.set_critical(8_000).unwrap();
        t.set_hysteresis(200).unwrap();
        assert_eq!(t.wake().unwrap(), CONVERSION_1SPS_US);
        t.sleep().unwrap();
        t.config = Config {
            run_mode: RunMode::Continuous,
            ..config
        };
        assert_eq!(t.wake().unwrap(), CONVERSION_US);
        i2c.done();
    }

    #[test]
    fn async_one_shot_13_bit() {
        use zigbee_sensor::asynch::Sensor as _;
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x20]),
            Transaction::write_read(A, vec![REG_TEMP], vec![0xF3, 0x85]),
        ];
        let config = Config {
            resolution: Resolution::Bits13,
            ..Config::default()
        };
        let mut i2c = Mock::new(&expectations);
        let mut t = asynch::Adt7420::new(&mut i2c, A).with_config(config);
        assert_eq!(block_on(t.start_measurement()).unwrap(), CONVERSION_US);
        // 0xF380 = −25 °C, plus two flag bits.
        assert_eq!(block_on(t.read_measurement()).unwrap().temperature, -2_500);
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
