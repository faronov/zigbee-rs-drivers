//! TI TMP102 / TMP112 digital temperature sensor (I2C / SMBus).
//!
//! Built for battery devices: [`configure`](Tmp102::configure) leaves the
//! sensor in shutdown (0.15 µA typ.) and every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! triggers a single conversion with the one-shot bit. After the conversion
//! the part returns to shutdown by itself.
//!
//! References:
//! * TI "TMP102 Low-Power Digital Temperature Sensor", SBOS397I (June 2024).
//! * TI "TMP112x High-Accuracy, Low-Power, Digital Temperature Sensors",
//!   SBOS473L (July 2024).
//!
//! Conversion time: the current data sheets specify 15 ms (TMP102) and
//! 11.25 ms (TMP112) maximum, but older silicon (SBOS397H and earlier) is
//! specified at 26 ms typ. / 35 ms max. The driver waits
//! [`CONVERSION_WAIT_US`] = 35 ms by default; use
//! [`with_conversion_wait_us`](Tmp102::with_conversion_wait_us) for new parts.
//!
//! ```ignore
//! let mut tmp = Tmp102::new(i2c, ADDRESS_GND);
//! tmp.configure()?; // shutdown, 12-bit, comparator alert
//! let wait_us = tmp.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = tmp.read_measurement()?;
//! ```
//!
//! The blocking driver and its [`asynch`] twin are generated from one macro
//! body, so they issue identical bus transactions.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with ADD0 tied to GND.
pub const ADDRESS_GND: u8 = 0x48;
/// I2C address with ADD0 tied to V+.
pub const ADDRESS_VCC: u8 = 0x49;
/// I2C address with ADD0 tied to SDA.
pub const ADDRESS_SDA: u8 = 0x4A;
/// I2C address with ADD0 tied to SCL.
pub const ADDRESS_SCL: u8 = 0x4B;

/// Default one-shot wait: 35 ms, the maximum of pre-2024 silicon.
pub const CONVERSION_WAIT_US: WaitUs = 35_000;

const REG_TEMPERATURE: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;
const REG_T_LOW: u8 = 0x02;
const REG_T_HIGH: u8 = 0x03;

// Configuration register, byte 1 (MSB) bits 15..8 and byte 2 bits 7..0.
const CFG_OS: u16 = 1 << 15;
const CFG_R_MASK: u16 = 0b11 << 13;
const CFG_SD: u16 = 1 << 8;
const CFG_TM: u16 = 1 << 9;
const CFG_POL: u16 = 1 << 10;
const CFG_AL: u16 = 1 << 5;
const CFG_EM: u16 = 1 << 4;

/// Continuous-mode conversion rate (CR1/CR0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConversionRate {
    /// 0.25 conversions per second.
    Hz0_25,
    /// 1 conversion per second.
    Hz1,
    /// 4 conversions per second (power-on default).
    #[default]
    Hz4,
    /// 8 conversions per second.
    Hz8,
}

impl ConversionRate {
    const fn bits(self) -> u16 {
        (self as u16) << 6
    }
}

/// Consecutive out-of-limit conversions needed to assert ALERT (F1/F0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FaultQueue {
    /// 1 fault (default).
    #[default]
    One,
    /// 2 consecutive faults.
    Two,
    /// 4 consecutive faults.
    Four,
    /// 6 consecutive faults.
    Six,
}

/// ALERT pin behaviour (TM bit).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AlertMode {
    /// Active from `T >= T_HIGH` until `T < T_LOW` (thermostat). Best for
    /// waking a sleepy device: the state survives shutdown.
    #[default]
    Comparator,
    /// Latched until any register is read; also cleared by entering
    /// shutdown, so it is of little use with one-shot conversions.
    Interrupt,
}

/// Configuration written by [`Tmp102::configure`] and every one-shot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Rate used in continuous mode (after [`LowPower::wake`](zigbee_sensor::blocking::LowPower::wake)).
    pub conversion_rate: ConversionRate,
    /// Extended mode: 13-bit format, range up to +150 °C.
    pub extended: bool,
    /// ALERT comparator / interrupt mode.
    pub alert_mode: AlertMode,
    /// ALERT active high (`POL = 1`) instead of active low.
    pub alert_active_high: bool,
    /// Fault queue length.
    pub fault_queue: FaultQueue,
}

impl Config {
    /// Configuration register word for this configuration.
    pub const fn register(self, shutdown: bool, one_shot: bool) -> u16 {
        let mut word = self.conversion_rate.bits() | ((self.fault_queue as u16) << 11);
        if self.extended {
            word |= CFG_EM;
        }
        if matches!(self.alert_mode, AlertMode::Interrupt) {
            word |= CFG_TM;
        }
        if self.alert_active_high {
            word |= CFG_POL;
        }
        if shutdown {
            word |= CFG_SD;
        }
        if one_shot {
            word |= CFG_OS;
        }
        word
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

/// Decode the temperature register (data sheet tables 6-2 / 6-3).
///
/// Bit 0 flags the 13-bit extended format. Returns `None` for `0xFFFF`,
/// which no valid conversion produces but a missing device on a bus with
/// pull-ups does.
pub const fn decode_temperature(raw: u16) -> Option<i32> {
    if raw == 0xFFFF {
        return None;
    }
    let counts = if raw & 1 != 0 {
        (raw as i16 >> 3) as i32
    } else {
        (raw as i16 >> 4) as i32
    };
    // 1 LSB = 0.0625 °C; round to the nearest 0.01 °C.
    Some((counts * 100 + 8) >> 4)
}

/// Encode a limit in 0.01 °C into the `T_LOW`/`T_HIGH` register format,
/// saturating at the register range.
pub const fn encode_limit(centi_celsius: i32, extended: bool) -> u16 {
    let counts = (centi_celsius * 16 + 50).div_euclid(100);
    if extended {
        let c = if counts < -4096 {
            -4096
        } else if counts > 4095 {
            4095
        } else {
            counts
        };
        ((c << 3) as u16) & 0xFFF8
    } else {
        let c = if counts < -2048 {
            -2048
        } else if counts > 2047 {
            2047
        } else {
            counts
        };
        ((c << 4) as u16) & 0xFFF0
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
                    conversion_rate: ConversionRate::Hz4,
                    extended: false,
                    alert_mode: AlertMode::Comparator,
                    alert_active_high: false,
                    fault_queue: FaultQueue::One,
                },
                conversion_wait_us: CONVERSION_WAIT_US,
            }
        }

        /// Use `config` for [`Self::configure`] and every later register
        /// write.
        pub const fn with_config(mut self, config: Config) -> Self {
            self.config = config;
            self
        }

        /// Override the one-shot wait (e.g. 15 ms for TMP102 rev. I
        /// silicon, 11.25 ms for current TMP112).
        pub const fn with_conversion_wait_us(mut self, wait_us: WaitUs) -> Self {
            self.conversion_wait_us = wait_us;
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
        impl<I2C: $i2c> Tmp102<I2C> {
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

            /// Check the read-only resolution bits (R1/R0 = 11). Returns
            /// [`Error::UnexpectedId`] with the configuration word otherwise.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let config = self.read_register(REG_CONFIG) $($await)*?;
                if config & CFG_R_MASK != CFG_R_MASK {
                    return Err(Error::UnexpectedId(u32::from(config)));
                }
                Ok(())
            }

            /// Write the configuration and enter shutdown. Call once after
            /// power-up: the part starts in continuous mode (~5 µA).
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(true, false)) $($await)*
            }

            /// Change the configuration and write it (device stays in shutdown).
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program the ALERT thresholds in 0.01 °C. In comparator mode
            /// ALERT asserts at `T >= high` and releases below `low`.
            pub $($async)? fn set_limits(&mut self, low: i32, high: i32) -> Result<(), Error<I2C::Error>> {
                let extended = self.config.extended;
                self.write_register(REG_T_LOW, encode_limit(low, extended)) $($await)*?;
                self.write_register(REG_T_HIGH, encode_limit(high, extended)) $($await)*
            }

            /// Read the raw configuration register.
            pub $($async)? fn read_config(&mut self) -> Result<u16, Error<I2C::Error>> {
                self.read_register(REG_CONFIG) $($await)*
            }

            /// `true` once a one-shot conversion has finished (OS reads 1).
            pub $($async)? fn conversion_done(&mut self) -> Result<bool, Error<I2C::Error>> {
                Ok(self.read_register(REG_CONFIG) $($await)*? & CFG_OS != 0)
            }

            /// `true` while the comparator alert condition is active (AL bit,
            /// corrected for the configured polarity).
            pub $($async)? fn alert_active(&mut self) -> Result<bool, Error<I2C::Error>> {
                let al = self.read_register(REG_CONFIG) $($await)*? & CFG_AL != 0;
                Ok(al == self.config.alert_active_high)
            }
        }

        impl<I2C: $i2c> $sensor for Tmp102<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot conversion from shutdown (OS = 1, SD = 1).
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(true, true)) $($await)*?;
                Ok(self.conversion_wait_us)
            }

            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let raw = self.read_register(REG_TEMPERATURE) $($await)*?;
                match decode_temperature(raw) {
                    Some(temperature) => Ok(Measurement { temperature }),
                    None => Err(Error::InvalidData),
                }
            }
        }

        impl<I2C: $i2c> $low_power for Tmp102<I2C> {
            type BusError = I2C::Error;

            /// Shutdown (SD = 1).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.configure() $($await)*
            }

            /// Continuous conversion at the configured rate. Returns the wait
            /// until the first result.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(false, false)) $($await)*?;
                Ok(self.conversion_wait_us)
            }
        }
    };
}

/// Blocking driver.
pub struct Tmp102<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
    conversion_wait_us: WaitUs,
}

impl<I2C> Tmp102<I2C> {
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

    /// Async twin of [`super::Tmp102`].
    pub struct Tmp102<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
        conversion_wait_us: WaitUs,
    }

    impl<I2C> Tmp102<I2C> {
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

    const A: u8 = ADDRESS_GND;

    #[test]
    fn decodes_12_bit_datasheet_table() {
        // SBOS397I table 6-2.
        assert_eq!(decode_temperature(0x7FF0), Some(12_794)); // 127.9375
        assert_eq!(decode_temperature(0x6400), Some(10_000));
        assert_eq!(decode_temperature(0x5000), Some(8_000));
        assert_eq!(decode_temperature(0x4B00), Some(7_500));
        assert_eq!(decode_temperature(0x3200), Some(5_000));
        assert_eq!(decode_temperature(0x1900), Some(2_500));
        assert_eq!(decode_temperature(0x0040), Some(25)); // 0.25
        assert_eq!(decode_temperature(0x0000), Some(0));
        assert_eq!(decode_temperature(0xFFC0), Some(-25));
        assert_eq!(decode_temperature(0xE700), Some(-2_500));
        assert_eq!(decode_temperature(0xC900), Some(-5_500));
        assert_eq!(decode_temperature(0x0010), Some(6)); // 0.0625
        assert_eq!(decode_temperature(0xFFF0), Some(-6));
    }

    #[test]
    fn decodes_13_bit_datasheet_table() {
        // SBOS397I table 6-3: 13-bit value left-aligned, bit 0 = EM flag.
        assert_eq!(decode_temperature(0x4B01), Some(15_000));
        assert_eq!(decode_temperature(0x3E81), Some(12_500));
        assert_eq!(decode_temperature(0x0C81), Some(2_500));
        assert_eq!(decode_temperature(0x0011), Some(13)); // 0.125
        assert_eq!(decode_temperature(0xF381), Some(-2_500));
        assert_eq!(decode_temperature(0xE481), Some(-5_500));
        assert_eq!(decode_temperature(0xFFFF), None);
    }

    #[test]
    fn limit_encoding_round_trips() {
        assert_eq!(encode_limit(8_000, false), 0x5000);
        assert_eq!(encode_limit(7_500, false), 0x4B00);
        assert_eq!(encode_limit(-2_500, false), 0xE700);
        assert_eq!(encode_limit(15_000, true), 0x4B00);
        assert_eq!(encode_limit(-5_500, true), 0xE480);
        // Saturates instead of wrapping.
        assert_eq!(encode_limit(20_000, false), 0x7FF0);
        assert_eq!(encode_limit(-30_000, true), 0x8000);
    }

    #[test]
    fn default_config_matches_power_on_value() {
        // Power-on 0x60A0 without the read-only R1/R0 and AL bits.
        assert_eq!(Config::default().register(false, false), 0x0080);
        assert_eq!(Config::default().register(true, true), 0x8180);
    }

    #[test]
    fn one_shot_then_read() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x81, 0x80]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x19, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp102::new(&mut i2c, A);
        assert_eq!(tmp.start_measurement().unwrap(), 35_000);
        assert_eq!(
            tmp.read_measurement().unwrap(),
            Measurement { temperature: 2_500 }
        );
        i2c.done();
    }

    #[test]
    fn configure_extended_interrupt_and_limits() {
        let config = Config {
            conversion_rate: ConversionRate::Hz1,
            extended: true,
            alert_mode: AlertMode::Interrupt,
            alert_active_high: true,
            fault_queue: FaultQueue::Four,
        };
        let expectations = [
            // OS=0 F=10 POL=1 TM=1 SD=1 | CR=01 EM=1.
            Transaction::write(A, vec![REG_CONFIG, 0x17, 0x50]),
            Transaction::write(A, vec![REG_T_LOW, 0x0C, 0x80]),
            Transaction::write(A, vec![REG_T_HIGH, 0x4B, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp102::new(&mut i2c, A)
            .with_config(config)
            .with_conversion_wait_us(15_000);
        tmp.configure().unwrap();
        tmp.set_limits(2_500, 15_000).unwrap();
        i2c.done();
    }

    #[test]
    fn probe_checks_resolution_bits() {
        let expectations = [
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x60, 0xA0]),
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x00, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp102::new(&mut i2c, A);
        assert_eq!(tmp.probe(), Ok(()));
        assert_eq!(tmp.probe(), Err(Error::UnexpectedId(0)));
        i2c.done();
    }

    #[test]
    fn floating_bus_is_invalid() {
        let expectations = [Transaction::write_read(
            A,
            vec![REG_TEMPERATURE],
            vec![0xFF, 0xFF],
        )];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp102::new(&mut i2c, A);
        assert_eq!(tmp.read_measurement(), Err(Error::InvalidData));
        i2c.done();
    }

    #[test]
    fn status_bits() {
        let expectations = [
            Transaction::write_read(A, vec![REG_CONFIG], vec![0xE1, 0x80]),
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x61, 0x80]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp102::new(&mut i2c, A);
        assert!(tmp.conversion_done().unwrap());
        // AL = 0 with active-low polarity: alert active.
        assert!(tmp.alert_active().unwrap());
        i2c.done();
    }

    #[test]
    fn sleep_and_wake() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x01, 0x80]),
            Transaction::write(A, vec![REG_CONFIG, 0x00, 0x80]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp102::new(&mut i2c, A);
        tmp.sleep().unwrap();
        assert_eq!(tmp.wake().unwrap(), CONVERSION_WAIT_US);
        i2c.done();
    }

    #[test]
    fn async_measure_awaits_conversion_time() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x81, 0x80]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0xE7, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = asynch::Tmp102::new(&mut i2c, A).with_conversion_wait_us(15_000);
        let mut delay = RecordingDelay(0);
        let m = block_on(tmp.measure(&mut delay)).unwrap();
        assert_eq!(m.temperature, -2_500);
        assert_eq!(delay.0, 15_000);
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
