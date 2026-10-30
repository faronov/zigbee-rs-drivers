//! Microchip TCN75A and MCP9800 / MCP9801 / MCP9802 / MCP9803 digital
//! temperature sensors (I2C / SMBus). All five share one register map.
//!
//! [`configure`](Tcn75a::configure) leaves the sensor in shutdown (0.1 µA
//! typ.); every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! sets the one-shot bit, after which the part returns to shutdown by
//! itself.
//!
//! References:
//! * Microchip "TCN75A 2-Wire Serial Temperature Sensor", DS21935D (2010).
//! * Microchip "MCP9800/1/2/3 2-Wire High-Accuracy Temperature Sensor",
//!   DS21909D (2010).
//!
//! Conversion time: both data sheets give 30 / 60 / 120 / 240 ms typical
//! for 9 / 10 / 11 / 12 bit; only the MCP980x data sheet specifies a
//! maximum (75 / 150 / 300 / 600 ms), which the driver uses for all parts.
//!
//! ```ignore
//! let mut t = Tcn75a::new(i2c, ADDRESS_DEFAULT);
//! t.configure()?; // shutdown, 9-bit
//! let wait_us = t.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = t.read_measurement()?;
//! ```
//!
//! The blocking driver and its [`asynch`] twin are generated from one macro
//! body, so they issue identical bus transactions.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Address with A2 = A1 = A0 = GND (TCN75A, MCP9801/03: 0x48..=0x4F by
/// pins; MCP9800/02: 0x48..=0x4F by ordering code A0..A7).
pub const ADDRESS_DEFAULT: u8 = 0x48;

/// MCP9800 / MCP9801 / MCP9802 / MCP9803 use the same driver.
pub type Mcp980x<I2C> = Tcn75a<I2C>;

const REG_TEMPERATURE: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;
const REG_T_HYST: u8 = 0x02;
const REG_T_SET: u8 = 0x03;

const CFG_ONE_SHOT: u8 = 1 << 7;
const CFG_POL: u8 = 1 << 2;
const CFG_INT: u8 = 1 << 1;
const CFG_SHUTDOWN: u8 = 1 << 0;

/// ADC resolution (bits 6:5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 9 bit, 0.5 °C (power-on default).
    #[default]
    Bits9,
    /// 10 bit, 0.25 °C.
    Bits10,
    /// 11 bit, 0.125 °C.
    Bits11,
    /// 12 bit, 0.0625 °C.
    Bits12,
}

impl Resolution {
    /// Maximum conversion time (MCP980x data sheet).
    pub const fn conversion_us(self) -> WaitUs {
        75_000 << (self as u32)
    }
}

/// Consecutive faults before ALERT changes (bits 4:3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FaultQueue {
    /// 1 fault (default).
    #[default]
    One,
    /// 2 faults.
    Two,
    /// 4 faults.
    Four,
    /// 6 faults.
    Six,
}

/// ALERT pin behaviour (COMP/INT bit).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AlertMode {
    /// Active above T_SET until below T_HYST; held through shutdown and
    /// released only in continuous mode.
    #[default]
    Comparator,
    /// Pulsed per crossing, cleared by any register read or by shutdown.
    Interrupt,
}

/// Configuration written by [`Tcn75a::configure`] and every one-shot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// ADC resolution (and conversion time).
    pub resolution: Resolution,
    /// ALERT fault queue.
    pub fault_queue: FaultQueue,
    /// ALERT comparator / interrupt mode.
    pub alert_mode: AlertMode,
    /// ALERT active high instead of active low.
    pub alert_active_high: bool,
}

impl Config {
    /// Configuration register byte for this configuration.
    pub const fn register(self, shutdown: bool, one_shot: bool) -> u8 {
        let mut byte = ((self.resolution as u8) << 5) | ((self.fault_queue as u8) << 3);
        if self.alert_active_high {
            byte |= CFG_POL;
        }
        if matches!(self.alert_mode, AlertMode::Interrupt) {
            byte |= CFG_INT;
        }
        if shutdown {
            byte |= CFG_SHUTDOWN;
        }
        if one_shot {
            byte |= CFG_ONE_SHOT;
        }
        byte
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

/// Decode the T_A register (register 5-2: 12-bit two's complement,
/// left-justified). Returns `None` if the always-zero low nibble is set
/// (e.g. `0xFFFF` from a missing device).
pub const fn decode_temperature(raw: u16) -> Option<i32> {
    if raw & 0x000F != 0 {
        return None;
    }
    let counts = (raw as i16 >> 4) as i32;
    Some((counts * 100 + 8) >> 4)
}

/// Encode a limit in 0.01 °C into the 9-bit T_HYST / T_SET format (0.5 °C,
/// bits 15:7), saturating.
pub const fn encode_limit(centi_celsius: i32) -> u16 {
    let counts = (centi_celsius * 2 + 50).div_euclid(100);
    let c = if counts < -256 {
        -256
    } else if counts > 255 {
        255
    } else {
        counts
    };
    (c << 7) as u16
}

macro_rules! constructors {
    () => {
        /// Create a driver with the default [`Config`]; no bus traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                config: Config {
                    resolution: Resolution::Bits9,
                    fault_queue: FaultQueue::One,
                    alert_mode: AlertMode::Comparator,
                    alert_active_high: false,
                },
                in_shutdown: false,
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
        impl<I2C: $i2c> Tcn75a<I2C> {
            $($async)? fn write_register(&mut self, register: u8, value: u16) -> Result<(), Error<I2C::Error>> {
                let [hi, lo] = value.to_be_bytes();
                self.i2c
                    .write(self.address, &[register, hi, lo])
                    $($await)*
                    .map_err(Error::bus)
            }

            $($async)? fn write_config(&mut self, byte: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(self.address, &[REG_CONFIG, byte])
                    $($await)*
                    .map_err(Error::bus)?;
                self.in_shutdown = byte & CFG_SHUTDOWN != 0;
                Ok(())
            }

            /// Read the 8-bit configuration register.
            pub $($async)? fn read_config(&mut self) -> Result<u8, Error<I2C::Error>> {
                let mut buf = [0u8; 1];
                self.i2c
                    .write_read(self.address, &[REG_CONFIG], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                Ok(buf[0])
            }

            /// `true` once a one-shot conversion has finished (bit 7
            /// cleared by the device).
            pub $($async)? fn conversion_done(&mut self) -> Result<bool, Error<I2C::Error>> {
                Ok(self.read_config() $($await)*? & CFG_ONE_SHOT == 0)
            }

            /// Write the configuration and enter shutdown. Call once after
            /// power-up: the part starts in continuous mode (200 µA).
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_config(self.config.register(true, false)) $($await)*
            }

            /// Change the configuration and write it (enters shutdown).
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program the ALERT limits in 0.01 °C (0.5 °C steps). ALERT
            /// asserts above `set` and, in comparator mode, releases below
            /// `hysteresis`.
            pub $($async)? fn set_limits(&mut self, hysteresis: i32, set: i32) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_T_HYST, encode_limit(hysteresis)) $($await)*?;
                self.write_register(REG_T_SET, encode_limit(set)) $($await)*
            }
        }

        impl<I2C: $i2c> $sensor for Tcn75a<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot conversion. The part ignores the one-shot bit
            /// unless already in shutdown, so a shutdown write is sent
            /// first if needed (only after power-up or `wake`).
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if !self.in_shutdown {
                    self.configure() $($await)*?;
                }
                self.write_config(self.config.register(true, true)) $($await)*?;
                Ok(self.config.resolution.conversion_us())
            }

            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(self.address, &[REG_TEMPERATURE], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                match decode_temperature(u16::from_be_bytes(buf)) {
                    Some(temperature) => Ok(Measurement { temperature }),
                    None => Err(Error::InvalidData),
                }
            }
        }

        impl<I2C: $i2c> $low_power for Tcn75a<I2C> {
            type BusError = I2C::Error;

            /// Shutdown (bit 0 = 1).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.configure() $($await)*
            }

            /// Continuous conversion. Returns the wait until the first
            /// result.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_config(self.config.register(false, false)) $($await)*?;
                Ok(self.config.resolution.conversion_us())
            }
        }
    };
}

/// Blocking driver.
pub struct Tcn75a<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
    in_shutdown: bool,
}

impl<I2C> Tcn75a<I2C> {
    constructors!();
}

driver_impl!(
    embedded_hal::i2c::I2c,
    zigbee_sensor::blocking::Sensor,
    zigbee_sensor::blocking::LowPower,
    ;
);

/// Async driver.
pub mod asynch {
    use super::*;

    /// Async twin of [`super::Tcn75a`].
    pub struct Tcn75a<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
        in_shutdown: bool,
    }

    impl<I2C> Tcn75a<I2C> {
        constructors!();
    }

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

    const A: u8 = ADDRESS_DEFAULT;

    #[test]
    fn decodes_register_format() {
        // Figure 5-3: +25.25 °C = 0x1940.
        assert_eq!(decode_temperature(0x1940), Some(2_525));
        assert_eq!(decode_temperature(0x7FF0), Some(12_794));
        assert_eq!(decode_temperature(0x0010), Some(6));
        assert_eq!(decode_temperature(0xFF80), Some(-50));
        assert_eq!(decode_temperature(0xC900), Some(-5_500));
        assert_eq!(decode_temperature(0xFFFF), None);
    }

    #[test]
    fn limits_config_and_waits() {
        // Power-up: T_HYST 75 °C = 0x4B00, T_SET 80 °C = 0x5000.
        assert_eq!(encode_limit(7_500), 0x4B00);
        assert_eq!(encode_limit(8_000), 0x5000);
        assert_eq!(encode_limit(-2_550), 0xE680); // rounds to -25.5
        assert_eq!(encode_limit(20_000), 0x7F80);
        assert_eq!(encode_limit(-30_000), 0x8000);
        assert_eq!(Config::default().register(false, false), 0x00);
        let c = Config {
            resolution: Resolution::Bits12,
            fault_queue: FaultQueue::Six,
            alert_mode: AlertMode::Interrupt,
            alert_active_high: true,
        };
        assert_eq!(c.register(true, true), 0xFF);
        assert_eq!(Resolution::Bits9.conversion_us(), 75_000);
        assert_eq!(Resolution::Bits12.conversion_us(), 600_000);
    }

    #[test]
    fn first_one_shot_enters_shutdown_first() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x01]),
            Transaction::write(A, vec![REG_CONFIG, 0x81]),
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x01]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x19, 0x40]),
            Transaction::write(A, vec![REG_CONFIG, 0x81]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0xFF, 0xFF]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = Tcn75a::new(&mut i2c, A);
        assert_eq!(t.start_measurement().unwrap(), 75_000);
        assert!(t.conversion_done().unwrap());
        assert_eq!(t.read_measurement().unwrap().temperature, 2_525);
        t.start_measurement().unwrap();
        assert_eq!(t.read_measurement(), Err(Error::InvalidData));
        i2c.done();
    }

    #[test]
    fn limits_sleep_wake() {
        let config = Config {
            resolution: Resolution::Bits11,
            alert_active_high: true,
            ..Config::default()
        };
        let expectations = [
            Transaction::write(A, vec![REG_T_HYST, 0x1E, 0x00]),
            Transaction::write(A, vec![REG_T_SET, 0x23, 0x80]),
            Transaction::write(A, vec![REG_CONFIG, 0x45]),
            Transaction::write(A, vec![REG_CONFIG, 0x44]),
            // After wake the next one-shot re-enters shutdown first.
            Transaction::write(A, vec![REG_CONFIG, 0x45]),
            Transaction::write(A, vec![REG_CONFIG, 0xC5]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = Mcp980x::new(&mut i2c, A).with_config(config);
        t.set_limits(3_000, 3_550).unwrap();
        t.sleep().unwrap();
        assert_eq!(t.wake().unwrap(), 300_000);
        assert_eq!(t.start_measurement().unwrap(), 300_000);
        i2c.done();
    }

    #[test]
    fn async_one_shot() {
        use zigbee_sensor::asynch::Sensor as _;
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x01]),
            Transaction::write(A, vec![REG_CONFIG, 0x81]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0xE7, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = asynch::Tcn75a::new(&mut i2c, A);
        assert_eq!(block_on(t.start_measurement()).unwrap(), 75_000);
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
