//! Analog Devices (Maxim) MAX31875 low-power temperature sensor (I2C / SMBus).
//!
//! [`configure`](Max31875::configure) leaves the sensor in shutdown (0.5 µA
//! typ.) and every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! sets the one-shot bit; the part converts once and returns to shutdown.
//!
//! Reference: Maxim "MAX31875 Low-Power I2C Temperature Sensor in WLP
//! Package", 19-100110 Rev 8 (5/21).
//!
//! The conversion time depends on the supply voltage and the resolution.
//! The data sheet specifies the 10-bit maximum (50 / 70 / 150 ms for
//! VDD ≥ 3 V / 2.5–3 V / ≤ 2.5 V) and the time to the first result after
//! power-up (60 / 85 / 180 ms); every extra bit doubles the time. The
//! driver waits the 10-bit maximum scaled by resolution plus the
//! first-conversion overhead, for the [`Supply`] range in [`Config`].
//!
//! The WLP package has no ALERT pin: the over-temperature comparator is
//! only visible as the OT status bit
//! ([`over_temperature`](Max31875::over_temperature)), which entering
//! shutdown resets. It cannot wake the MCU.
//!
//! Optional SMBus Packet Error Checking (CRC-8, polynomial 0x07) is
//! supported through [`Config::pec`].
//!
//! ```ignore
//! let mut t = Max31875::new(i2c, address(0)); // MAX31875R0 = 0x48
//! t.configure()?; // shutdown, 10-bit
//! let wait_us = t.start_measurement()?;
//! // ... sleep for wait_us ...
//! let m = t.read_measurement()?;
//! ```
//!
//! The blocking driver and its [`asynch`] twin are generated from one macro
//! body, so they issue identical bus transactions.

#![no_std]

use zigbee_sensor::crc::crc8_msb;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address of the MAX31875R`option` ordering option (0..=7 →
/// 0x48..=0x4F); the address is factory-programmed.
pub const fn address(option: u8) -> u8 {
    0x48 | (option & 0x07)
}

const REG_TEMPERATURE: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;
const REG_T_HYST: u8 = 0x02;
const REG_T_OS: u8 = 0x03;

const CFG_OT_STATUS: u16 = 1 << 15;
const CFG_COMP_INT: u16 = 1 << 9;
const CFG_SHUTDOWN: u16 = 1 << 8;
const CFG_EXTENDED: u16 = 1 << 7;
const CFG_TIMEOUT_DISABLE: u16 = 1 << 4;
const CFG_PEC: u16 = 1 << 3;
const CFG_ONE_SHOT: u16 = 1 << 0;

/// Conversion resolution (D6:D5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 8 bit, 1 °C.
    Bits8,
    /// 9 bit, 0.5 °C.
    Bits9,
    /// 10 bit, 0.25 °C (power-on default, 35 ms typ.).
    #[default]
    Bits10,
    /// 12 bit, 0.0625 °C (140 ms typ.).
    Bits12,
}

/// Continuous-mode conversion rate (D2:D1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConversionRate {
    /// One conversion every 4 s (power-on default).
    #[default]
    Hz0_25,
    /// 1 conversion per second.
    Hz1,
    /// 4 conversions per second.
    Hz4,
    /// 8 conversions per second.
    Hz8,
}

/// Consecutive over-temperature results needed to set OT status (D12:D11).
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

/// OT status behaviour (D9).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AlertMode {
    /// Set above T_OS, cleared below T_HYST (default).
    #[default]
    Comparator,
    /// Set on each crossing, cleared by any register read.
    Interrupt,
}

/// Supply-voltage range; selects the guaranteed conversion time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Supply {
    /// VDD ≥ 3 V.
    Above3V,
    /// 2.5 V < VDD < 3 V.
    From2V5To3V,
    /// 1.6 V ≤ VDD ≤ 2.5 V (default, worst case: e.g. a sagging coin cell).
    #[default]
    Below2V5,
}

/// Configuration written by [`Max31875::configure`] and every one-shot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Conversion resolution.
    pub resolution: Resolution,
    /// Rate used in continuous mode (after [`LowPower::wake`](zigbee_sensor::blocking::LowPower::wake)).
    pub conversion_rate: ConversionRate,
    /// Extended data format: MSB weight 128 °C instead of 64 °C, range to
    /// +150 °C. Also applies to the limit registers.
    pub extended: bool,
    /// OT status comparator / interrupt mode.
    pub alert_mode: AlertMode,
    /// Fault queue length.
    pub fault_queue: FaultQueue,
    /// Disable the 30 ms SMBus timeout (saves 0.2 µA standby; allows SCL
    /// below 20 kHz).
    pub disable_timeout: bool,
    /// Append / check an SMBus PEC byte on every transfer.
    pub pec: bool,
    /// Supply range used for the conversion wait.
    pub supply: Supply,
}

impl Config {
    /// Configuration register word for this configuration.
    pub const fn register(self, shutdown: bool, one_shot: bool) -> u16 {
        let mut word = ((self.fault_queue as u16) << 11)
            | ((self.resolution as u16) << 5)
            | ((self.conversion_rate as u16) << 1);
        if matches!(self.alert_mode, AlertMode::Interrupt) {
            word |= CFG_COMP_INT;
        }
        if shutdown {
            word |= CFG_SHUTDOWN;
        }
        if self.extended {
            word |= CFG_EXTENDED;
        }
        if self.disable_timeout {
            word |= CFG_TIMEOUT_DISABLE;
        }
        if self.pec {
            word |= CFG_PEC;
        }
        if one_shot {
            word |= CFG_ONE_SHOT;
        }
        word
    }

    /// Guaranteed conversion wait for this resolution and supply range.
    pub const fn conversion_wait_us(self) -> WaitUs {
        // (10-bit max, first-conversion overhead) in µs.
        let (max_10, overhead) = match self.supply {
            Supply::Above3V => (50_000, 10_000),
            Supply::From2V5To3V => (70_000, 15_000),
            Supply::Below2V5 => (150_000, 30_000),
        };
        let scaled = match self.resolution {
            Resolution::Bits8 => max_10 / 4,
            Resolution::Bits9 => max_10 / 2,
            Resolution::Bits10 => max_10,
            Resolution::Bits12 => max_10 * 4,
        };
        scaled + overhead
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

/// Decode the temperature register (data sheet table 3).
///
/// Returns `None` if bits that are always zero in the selected format are
/// set (e.g. `0xFFFF` from a missing device).
pub const fn decode_temperature(raw: u16, extended: bool) -> Option<i32> {
    let (zero_mask, shift) = if extended { (0x0007, 3) } else { (0x000F, 4) };
    if raw & zero_mask != 0 {
        return None;
    }
    let counts = (raw as i16 >> shift) as i32;
    // 1 LSB = 0.0625 °C; round to the nearest 0.01 °C.
    Some((counts * 100 + 8) >> 4)
}

/// Encode a limit in 0.01 °C into the T_OS / T_HYST format, saturating.
pub const fn encode_limit(centi_celsius: i32, extended: bool) -> u16 {
    let counts = (centi_celsius * 16 + 50).div_euclid(100);
    let (min, max, shift) = if extended {
        (-4096, 4095, 3)
    } else {
        (-2048, 2047, 4)
    };
    let c = if counts < min {
        min
    } else if counts > max {
        max
    } else {
        counts
    };
    (c << shift) as u16
}

/// SMBus PEC (CRC-8, polynomial 0x07, init 0).
pub fn pec(bytes: &[u8]) -> u8 {
    crc8_msb(bytes, 0x07, 0x00)
}

macro_rules! constructors {
    () => {
        /// Create a driver with the default [`Config`]; no bus traffic.
        /// Assumes the part is in its power-on state (PEC disabled).
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                config: Config {
                    resolution: Resolution::Bits10,
                    conversion_rate: ConversionRate::Hz0_25,
                    extended: false,
                    alert_mode: AlertMode::Comparator,
                    fault_queue: FaultQueue::One,
                    disable_timeout: false,
                    pec: false,
                    supply: Supply::Below2V5,
                },
                pec_active: false,
            }
        }

        /// Use `config` for [`Self::configure`] and every later register
        /// write.
        pub const fn with_config(mut self, config: Config) -> Self {
            self.config = config;
            self
        }

        /// Declare that the device already has PEC enabled (e.g. after an
        /// MCU-only reset) so the next transfer carries a PEC byte.
        pub const fn with_pec_active(mut self, active: bool) -> Self {
            self.pec_active = active;
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
        impl<I2C: $i2c> Max31875<I2C> {
            $($async)? fn write_register(&mut self, register: u8, value: u16) -> Result<(), Error<I2C::Error>> {
                let [hi, lo] = value.to_be_bytes();
                let crc = pec(&[self.address << 1, register, hi, lo]);
                let frame = [register, hi, lo, crc];
                let len = if self.pec_active { 4 } else { 3 };
                self.i2c
                    .write(self.address, &frame[..len])
                    $($await)*
                    .map_err(Error::bus)
            }

            $($async)? fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
                let mut buf = [0u8; 3];
                let len = if self.pec_active { 3 } else { 2 };
                self.i2c
                    .write_read(self.address, &[register], &mut buf[..len])
                    $($await)*
                    .map_err(Error::bus)?;
                if self.pec_active {
                    let a = self.address << 1;
                    if pec(&[a, register, a | 1, buf[0], buf[1]]) != buf[2] {
                        return Err(Error::Crc);
                    }
                }
                Ok(u16::from_be_bytes([buf[0], buf[1]]))
            }

            $($async)? fn write_config(&mut self, word: u16) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_CONFIG, word) $($await)*?;
                self.pec_active = word & CFG_PEC != 0;
                Ok(())
            }

            /// Write the configuration and enter shutdown. Call once after
            /// power-up: the part starts converting every 4 s (~1.1 µA).
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_config(self.config.register(true, false)) $($await)*
            }

            /// Change the configuration and write it (device stays in
            /// shutdown). After toggling [`Config::extended`] the data
            /// format changes only after the next conversion.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program T_OS / T_HYST in 0.01 °C (OT status sets above
            /// `high`, clears below `hysteresis`; `hysteresis` above `high`
            /// is treated as `high`). Encoded in the current data format.
            pub $($async)? fn set_limits(&mut self, hysteresis: i32, high: i32) -> Result<(), Error<I2C::Error>> {
                let extended = self.config.extended;
                self.write_register(REG_T_HYST, encode_limit(hysteresis, extended)) $($await)*?;
                self.write_register(REG_T_OS, encode_limit(high, extended)) $($await)*
            }

            /// Read the raw configuration register.
            pub $($async)? fn read_config(&mut self) -> Result<u16, Error<I2C::Error>> {
                self.read_register(REG_CONFIG) $($await)*
            }

            /// `true` once a one-shot conversion has finished (D0 reads 0).
            pub $($async)? fn conversion_done(&mut self) -> Result<bool, Error<I2C::Error>> {
                Ok(self.read_register(REG_CONFIG) $($await)*? & CFG_ONE_SHOT == 0)
            }

            /// OT status bit (D15). Reset by entering shutdown, so mostly
            /// useful in continuous mode.
            pub $($async)? fn over_temperature(&mut self) -> Result<bool, Error<I2C::Error>> {
                Ok(self.read_register(REG_CONFIG) $($await)*? & CFG_OT_STATUS != 0)
            }
        }

        impl<I2C: $i2c> $sensor for Max31875<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot conversion from shutdown (D8 = 1, D0 = 1).
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_config(self.config.register(true, true)) $($await)*?;
                Ok(self.config.conversion_wait_us())
            }

            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let raw = self.read_register(REG_TEMPERATURE) $($await)*?;
                match decode_temperature(raw, self.config.extended) {
                    Some(temperature) => Ok(Measurement { temperature }),
                    None => Err(Error::InvalidData),
                }
            }
        }

        impl<I2C: $i2c> $low_power for Max31875<I2C> {
            type BusError = I2C::Error;

            /// Shutdown (D8 = 1).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.configure() $($await)*
            }

            /// Automatic conversions at the configured rate. Returns the
            /// wait until the first result.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_config(self.config.register(false, false)) $($await)*?;
                Ok(self.config.conversion_wait_us())
            }
        }
    };
}

/// Blocking driver.
pub struct Max31875<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
    pec_active: bool,
}

impl<I2C> Max31875<I2C> {
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

    /// Async twin of [`super::Max31875`].
    pub struct Max31875<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
        pec_active: bool,
    }

    impl<I2C> Max31875<I2C> {
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

    const A: u8 = 0x48;

    #[test]
    fn decodes_datasheet_formats() {
        // Table 3: normal format D14 = 64 °C, extended D14 = 128 °C.
        assert_eq!(decode_temperature(0x1900, false), Some(2_500));
        assert_eq!(decode_temperature(0x0010, false), Some(6)); // 0.0625
        assert_eq!(decode_temperature(0xFFF0, false), Some(-6));
        assert_eq!(decode_temperature(0xE700, false), Some(-2_500));
        assert_eq!(decode_temperature(0x7FF0, false), Some(12_794));
        assert_eq!(decode_temperature(0x4B00, true), Some(15_000));
        assert_eq!(decode_temperature(0x0C80, true), Some(2_500));
        assert_eq!(decode_temperature(0x0008, true), Some(6));
        assert_eq!(decode_temperature(0xFFFF, false), None);
        assert_eq!(decode_temperature(0x0008, false), None);
    }

    #[test]
    fn limits_and_power_on_values() {
        // POR: T_HYST 0x4B00 = 75 °C, T_OS 0x5000 = 80 °C, config 0x0040.
        assert_eq!(encode_limit(7_500, false), 0x4B00);
        assert_eq!(encode_limit(8_000, false), 0x5000);
        assert_eq!(encode_limit(15_000, true), 0x4B00);
        assert_eq!(encode_limit(20_000, false), 0x7FF0);
        assert_eq!(encode_limit(-30_000, true), 0x8000);
        assert_eq!(Config::default().register(false, false), 0x0040);
    }

    #[test]
    fn wait_scales_with_resolution_and_supply() {
        let c = Config::default();
        assert_eq!(c.conversion_wait_us(), 180_000);
        let fast = Config {
            supply: Supply::Above3V,
            resolution: Resolution::Bits8,
            ..c
        };
        assert_eq!(fast.conversion_wait_us(), 22_500);
        let fine = Config {
            supply: Supply::Above3V,
            resolution: Resolution::Bits12,
            ..c
        };
        assert_eq!(fine.conversion_wait_us(), 210_000);
    }

    #[test]
    fn one_shot_then_read() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x01, 0x41]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x19, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = Max31875::new(&mut i2c, A);
        assert_eq!(t.start_measurement().unwrap(), 180_000);
        assert_eq!(
            t.read_measurement().unwrap(),
            Measurement { temperature: 2_500 }
        );
        i2c.done();
    }

    #[test]
    fn configure_all_fields_and_limits() {
        let config = Config {
            resolution: Resolution::Bits12,
            conversion_rate: ConversionRate::Hz8,
            extended: true,
            alert_mode: AlertMode::Interrupt,
            fault_queue: FaultQueue::Six,
            disable_timeout: true,
            pec: false,
            supply: Supply::Above3V,
        };
        let expectations = [
            // FQ=11 C/I=1 SD=1 | EXT=1 RES=11 TO=1 PEC=0 CR=11.
            Transaction::write(A, vec![REG_CONFIG, 0x1B, 0xF6]),
            Transaction::write(A, vec![REG_T_HYST, 0x0C, 0x80]),
            Transaction::write(A, vec![REG_T_OS, 0x4B, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = Max31875::new(&mut i2c, A).with_config(config);
        t.configure().unwrap();
        t.set_limits(2_500, 15_000).unwrap();
        i2c.done();
    }

    #[test]
    fn pec_is_appended_and_checked_once_enabled() {
        let config = Config {
            pec: true,
            ..Config::default()
        };
        let a = A << 1;
        let one_shot = pec(&[a, REG_CONFIG, 0x01, 0x49]);
        let rd = pec(&[a, REG_TEMPERATURE, a | 1, 0x19, 0x00]);
        let expectations = [
            // The write that enables PEC carries none.
            Transaction::write(A, vec![REG_CONFIG, 0x01, 0x48]),
            Transaction::write(A, vec![REG_CONFIG, 0x01, 0x49, one_shot]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x19, 0x00, rd]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x19, 0x00, rd ^ 1]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = Max31875::new(&mut i2c, A).with_config(config);
        t.configure().unwrap();
        t.start_measurement().unwrap();
        assert_eq!(t.read_measurement().unwrap().temperature, 2_500);
        assert_eq!(t.read_measurement(), Err(Error::Crc));
        i2c.done();
    }

    #[test]
    fn pec_known_vector() {
        // SMBus CRC-8 check value for "123456789".
        assert_eq!(pec(b"123456789"), 0xF4);
    }

    #[test]
    fn status_sleep_and_wake() {
        let expectations = [
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x81, 0x40]),
            Transaction::write_read(A, vec![REG_CONFIG], vec![0x81, 0x41]),
            Transaction::write(A, vec![REG_CONFIG, 0x01, 0x40]),
            Transaction::write(A, vec![REG_CONFIG, 0x00, 0x40]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = Max31875::new(&mut i2c, A);
        assert!(t.over_temperature().unwrap());
        assert!(!t.conversion_done().unwrap());
        t.sleep().unwrap();
        assert_eq!(t.wake().unwrap(), 180_000);
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

        let config = Config {
            supply: Supply::Above3V,
            ..Config::default()
        };
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x01, 0x41]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0xE7, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut t = asynch::Max31875::new(&mut i2c, A).with_config(config);
        let mut delay = RecordingDelay(0);
        let m = block_on(t.measure(&mut delay)).unwrap();
        assert_eq!(m.temperature, -2_500);
        assert_eq!(delay.0, 60_000);
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
