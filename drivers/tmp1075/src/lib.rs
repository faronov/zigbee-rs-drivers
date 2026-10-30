//! TI TMP1075 / TMP1075N LM75-compatible digital temperature sensor (I2C /
//! SMBus).
//!
//! [`configure`](Tmp1075::configure) leaves the sensor in shutdown
//! (0.37 µA typ., TMP1075N 0.15 µA); every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! sets the OS bit for one conversion, after which the part returns to
//! shutdown by itself.
//!
//! The configuration register is always accessed with single-byte writes
//! and reads (bits 15:8), which the TMP1075N requires and the TMP1075
//! accepts for LM75 compatibility.
//!
//! Reference: TI "TMP1075 Temperature Sensor With I2C and SMBus Interface in
//! Industry Standard LM75 Form Factor and Pinout", SBOS854F (June 2024).
//!
//! ```ignore
//! let mut tmp = Tmp1075::new(i2c, ADDRESS_DEFAULT);
//! tmp.probe()?;
//! tmp.configure()?;
//! let wait_us = tmp.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = tmp.read_measurement()?;
//! ```
//!
//! The blocking driver and its [`asynch`] twin are generated from one macro
//! body, so they issue identical bus transactions.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Address with A2 = A1 = A0 = GND (TMP1075: 0x40..=0x5F, TMP1075N:
/// 0x48..=0x4B).
pub const ADDRESS_DEFAULT: u8 = 0x48;

/// Value of the TMP1075 Device ID register (not present on TMP1075N).
pub const DEVICE_ID: u16 = 0x7500;

const REG_TEMPERATURE: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;
const REG_T_LOW: u8 = 0x02;
const REG_T_HIGH: u8 = 0x03;
const REG_DEVICE_ID: u8 = 0x0F;

const CFG_OS: u8 = 1 << 7;
const CFG_RATE_MASK: u8 = 0b11 << 5;
const CFG_POL: u8 = 1 << 2;
const CFG_TM: u8 = 1 << 1;
const CFG_SD: u8 = 1 << 0;

/// Orderable variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// TMP1075 (SOIC/VSSOP/WSON): 7 ms max one-shot, Device ID register.
    #[default]
    Tmp1075,
    /// TMP1075N (SOT-563): 15 ms max one-shot, fixed 250 ms rate, no ID.
    Tmp1075N,
}

impl Variant {
    /// Worst-case one-shot conversion time in µs.
    pub const fn conversion_us(self) -> WaitUs {
        match self {
            Self::Tmp1075 => 7_000,
            Self::Tmp1075N => 15_000,
        }
    }
}

/// Continuous-mode conversion period (R1:R0). Read-only (250 ms) on
/// TMP1075N.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConversionRate {
    /// 27.5 ms (default).
    #[default]
    Ms27_5,
    /// 55 ms.
    Ms55,
    /// 110 ms.
    Ms110,
    /// 220 ms.
    Ms220,
}

/// Consecutive faults before ALERT changes (F1:F0). TMP1075N counts
/// 1/2/4/6.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FaultQueue {
    /// 1 fault (default).
    #[default]
    One,
    /// 2 faults.
    Two,
    /// 3 faults (TMP1075N: 4).
    Three,
    /// 4 faults (TMP1075N: 6).
    Four,
}

/// ALERT pin behaviour (TM bit).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AlertMode {
    /// Active while `T >= T_HIGH`, released when `T < T_LOW`.
    #[default]
    Comparator,
    /// Latched until a register read or the SMBus alert response.
    Interrupt,
}

/// Configuration written by [`Tmp1075::configure`] and every one-shot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Period used in continuous mode.
    pub conversion_rate: ConversionRate,
    /// ALERT fault queue.
    pub fault_queue: FaultQueue,
    /// ALERT comparator / interrupt mode.
    pub alert_mode: AlertMode,
    /// ALERT active high (`POL = 1`) instead of active low.
    pub alert_active_high: bool,
}

impl Config {
    /// High byte of the configuration register (the byte the driver writes).
    pub const fn register(self, shutdown: bool, one_shot: bool) -> u8 {
        let mut byte = ((self.conversion_rate as u8) << 5) | ((self.fault_queue as u8) << 3);
        if self.alert_active_high {
            byte |= CFG_POL;
        }
        if matches!(self.alert_mode, AlertMode::Interrupt) {
            byte |= CFG_TM;
        }
        if shutdown {
            byte |= CFG_SD;
        }
        if one_shot {
            byte |= CFG_OS;
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

/// Decode the 12-bit left-justified temperature register (table 7-1).
/// Returns `None` if the always-zero low nibble is set (e.g. `0xFFFF` from a
/// missing device).
pub const fn decode_temperature(raw: u16) -> Option<i32> {
    if raw & 0x000F != 0 {
        return None;
    }
    let counts = (raw as i16 >> 4) as i32;
    Some((counts * 100 + 8) >> 4)
}

/// Encode a limit in 0.01 °C into the `LLIM`/`HLIM` format, saturating.
pub const fn encode_limit(centi_celsius: i32) -> u16 {
    let counts = (centi_celsius * 16 + 50).div_euclid(100);
    let c = if counts < -2048 {
        -2048
    } else if counts > 2047 {
        2047
    } else {
        counts
    };
    ((c << 4) as u16) & 0xFFF0
}

macro_rules! constructors {
    () => {
        /// Create a TMP1075 driver with the default [`Config`]; no bus
        /// traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                variant: Variant::Tmp1075,
                config: Config {
                    conversion_rate: ConversionRate::Ms27_5,
                    fault_queue: FaultQueue::One,
                    alert_mode: AlertMode::Comparator,
                    alert_active_high: false,
                },
            }
        }

        /// Select the orderable (conversion time and probe method).
        pub const fn with_variant(mut self, variant: Variant) -> Self {
            self.variant = variant;
            self
        }

        /// Use `config` for [`Self::configure`] and later mode changes.
        pub const fn with_config(mut self, config: Config) -> Self {
            self.config = config;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

macro_rules! driver_impl {
    ($i2c:path, $sensor:path, $low_power:path, $($async:ident)?; $($await:tt)*) => {
        impl<I2C: $i2c> Tmp1075<I2C> {
            $($async)? fn write_register(&mut self, register: u8, value: u16) -> Result<(), Error<I2C::Error>> {
                let [hi, lo] = value.to_be_bytes();
                self.i2c
                    .write(self.address, &[register, hi, lo])
                    $($await)*
                    .map_err(Error::bus)
            }

            $($async)? fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(self.address, &[register], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                Ok(u16::from_be_bytes(buf))
            }

            $($async)? fn write_config(&mut self, byte: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(self.address, &[REG_CONFIG, byte])
                    $($await)*
                    .map_err(Error::bus)
            }

            /// Read the high byte of the configuration register.
            pub $($async)? fn read_config(&mut self) -> Result<u8, Error<I2C::Error>> {
                let mut buf = [0u8; 1];
                self.i2c
                    .write_read(self.address, &[REG_CONFIG], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                Ok(buf[0])
            }

            /// TMP1075: check the Device ID register (0x7500). TMP1075N (no ID
            /// register): check that the read-only R1:R0 bits read 11.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                match self.variant {
                    Variant::Tmp1075 => {
                        let id = self.read_register(REG_DEVICE_ID) $($await)*?;
                        if id != DEVICE_ID {
                            return Err(Error::UnexpectedId(u32::from(id)));
                        }
                    }
                    Variant::Tmp1075N => {
                        let config = self.read_config() $($await)*?;
                        if config & CFG_RATE_MASK != CFG_RATE_MASK {
                            return Err(Error::UnexpectedId(u32::from(config)));
                        }
                    }
                }
                Ok(())
            }

            /// Write the configuration and enter shutdown. Call once after
            /// power-up: the part starts in continuous mode.
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_config(self.config.register(true, false)) $($await)*
            }

            /// Change the configuration and write it (enters shutdown).
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program the ALERT thresholds in 0.01 °C (0.0625 °C steps).
            /// In comparator mode `low` is the release point (hysteresis).
            pub $($async)? fn set_limits(&mut self, low: i32, high: i32) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_T_LOW, encode_limit(low)) $($await)*?;
                self.write_register(REG_T_HIGH, encode_limit(high)) $($await)*
            }
        }

        impl<I2C: $i2c> $sensor for Tmp1075<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot conversion from shutdown (OS = 1, SD = 1).
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_config(self.config.register(true, true)) $($await)*?;
                Ok(self.variant.conversion_us())
            }

            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let raw = self.read_register(REG_TEMPERATURE) $($await)*?;
                match decode_temperature(raw) {
                    Some(temperature) => Ok(Measurement { temperature }),
                    None => Err(Error::InvalidData),
                }
            }
        }

        impl<I2C: $i2c> $low_power for Tmp1075<I2C> {
            type BusError = I2C::Error;

            /// Shutdown (SD = 1).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.configure() $($await)*
            }

            /// Continuous conversion at the configured rate. Returns the
            /// wait until the first result.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_config(self.config.register(false, false)) $($await)*?;
                Ok(self.variant.conversion_us())
            }
        }
    };
}

/// Blocking driver.
pub struct Tmp1075<I2C> {
    i2c: I2C,
    address: u8,
    variant: Variant,
    config: Config,
}

impl<I2C> Tmp1075<I2C> {
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

    /// Async twin of [`super::Tmp1075`].
    pub struct Tmp1075<I2C> {
        i2c: I2C,
        address: u8,
        variant: Variant,
        config: Config,
    }

    impl<I2C> Tmp1075<I2C> {
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
    fn decodes_datasheet_table() {
        // SBOS854F table 7-1.
        assert_eq!(decode_temperature(0x7FF0), Some(12_794));
        assert_eq!(decode_temperature(0x6400), Some(10_000));
        assert_eq!(decode_temperature(0x1900), Some(2_500));
        assert_eq!(decode_temperature(0x0040), Some(25));
        assert_eq!(decode_temperature(0x0010), Some(6));
        assert_eq!(decode_temperature(0xFFF0), Some(-6));
        assert_eq!(decode_temperature(0xFFC0), Some(-25));
        assert_eq!(decode_temperature(0xE700), Some(-2_500));
        assert_eq!(decode_temperature(0xCE00), Some(-5_000));
        assert_eq!(decode_temperature(0xFFFF), None);
    }

    #[test]
    fn limits_and_config_byte() {
        // Reset values LLIM = 4B00h (75 °C), HLIM = 5000h (80 °C).
        assert_eq!(encode_limit(7_500), 0x4B00);
        assert_eq!(encode_limit(8_000), 0x5000);
        assert_eq!(encode_limit(-30_000), 0x8000);
        let c = Config {
            conversion_rate: ConversionRate::Ms220,
            fault_queue: FaultQueue::Three,
            alert_mode: AlertMode::Interrupt,
            alert_active_high: true,
        };
        assert_eq!(c.register(true, true), 0xF7);
        assert_eq!(Config::default().register(false, false), 0x00);
    }

    #[test]
    fn one_shot_then_read() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x01]),
            Transaction::write(A, vec![REG_CONFIG, 0x81]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x19, 0x00]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0xFF, 0xFF]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp1075::new(&mut i2c, A);
        tmp.configure().unwrap();
        assert_eq!(tmp.start_measurement().unwrap(), 7_000);
        assert_eq!(tmp.read_measurement().unwrap().temperature, 2_500);
        assert_eq!(tmp.read_measurement(), Err(Error::InvalidData));
        i2c.done();
    }

    #[test]
    fn probe_per_variant() {
        let expectations = [
            Transaction::write_read(A, vec![REG_DEVICE_ID], vec![0x75, 0x00]),
            Transaction::write_read(A, vec![REG_DEVICE_ID], vec![0x01, 0x17]),
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x60]),
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp1075::new(&mut i2c, A);
        assert_eq!(tmp.probe(), Ok(()));
        assert_eq!(tmp.probe(), Err(Error::UnexpectedId(0x0117)));
        let mut tmp = tmp.with_variant(Variant::Tmp1075N);
        assert_eq!(tmp.probe(), Ok(()));
        assert_eq!(tmp.probe(), Err(Error::UnexpectedId(0x00)));
        i2c.done();
    }

    #[test]
    fn limits_sleep_wake() {
        let expectations = [
            Transaction::write(A, vec![REG_T_LOW, 0x1E, 0x00]),
            Transaction::write(A, vec![REG_T_HIGH, 0x23, 0x80]),
            Transaction::write(A, vec![REG_CONFIG, 0x01]),
            Transaction::write(A, vec![REG_CONFIG, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp1075::new(&mut i2c, A).with_variant(Variant::Tmp1075N);
        tmp.set_limits(3_000, 3_550).unwrap();
        tmp.sleep().unwrap();
        assert_eq!(tmp.wake().unwrap(), 15_000);
        i2c.done();
    }

    #[test]
    fn async_one_shot() {
        use zigbee_sensor::asynch::Sensor as _;
        let config = Config {
            alert_mode: AlertMode::Interrupt,
            ..Config::default()
        };
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x83]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0xE7, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = asynch::Tmp1075::new(&mut i2c, A).with_config(config);
        assert_eq!(block_on(tmp.start_measurement()).unwrap(), 7_000);
        assert_eq!(
            block_on(tmp.read_measurement()).unwrap().temperature,
            -2_500
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
