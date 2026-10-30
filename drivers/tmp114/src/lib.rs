//! TI TMP114 ultra-thin 1.2 V digital temperature sensor (I2C).
//!
//! [`configure`](Tmp114::configure) leaves the sensor in shutdown
//! (0.16 µA typ.); every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! sets the one-shot bit. After the conversion the part sets `Mode = 1`
//! (shutdown) by itself.
//!
//! The TMP114 has no ALERT pin: limits and hysteresis only drive the flags
//! in the `Alert_Status` register. The optional write CRC (CRC_En) is not
//! used; the driver leaves it disabled.
//!
//! Reference: TI "TMP114 Ultra-Thin, 1.2-V to 1.8-V Supply, High Accuracy
//! Digital Temperature Sensor with I2C Interface", SNIS214E (July 2023).
//!
//! ```ignore
//! let mut tmp = Tmp114::new(i2c, ADDRESS_A);
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

/// TMP114A address.
pub const ADDRESS_A: u8 = 0x48;
/// TMP114B address.
pub const ADDRESS_B: u8 = 0x49;
/// TMP114C address.
pub const ADDRESS_C: u8 = 0x4A;
/// TMP114D address.
pub const ADDRESS_D: u8 = 0x4B;
/// TMP114ND address.
pub const ADDRESS_ND: u8 = 0x4C;
/// TMP114NC address.
pub const ADDRESS_NC: u8 = 0x4D;
/// TMP114NB address.
pub const ADDRESS_NB: u8 = 0x4E;
/// TMP114NA address.
pub const ADDRESS_NA: u8 = 0x4F;

/// Device ID (low 12 bits of the Device_ID register; bits 15:12 are the
/// silicon revision).
pub const DEVICE_ID: u16 = 0x114;

/// Wait after a software reset (data sheet: reset takes 1 ms).
pub const RESET_WAIT_US: WaitUs = 1_000;

const REG_TEMP_RESULT: u8 = 0x00;
const REG_ALERT_STATUS: u8 = 0x02;
const REG_CONFIG: u8 = 0x03;
const REG_T_LOW: u8 = 0x04;
const REG_T_HIGH: u8 = 0x05;
const REG_HYSTERESIS: u8 = 0x06;
const REG_DEVICE_ID: u8 = 0x0B;

const CFG_RESET: u16 = 1 << 8;
const CFG_AVG: u16 = 1 << 7;
const CFG_OS: u16 = 1 << 4;
const CFG_MODE_SHUTDOWN: u16 = 1 << 3;

/// ADC conversion time (ADC_Conv_Time). Shorter is lower energy but
/// noisier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AdcTime {
    /// 6.4 ms typ. (default, specified accuracy).
    #[default]
    Ms6_4,
    /// 3.5 ms typ.
    Ms3_5,
    /// 2.0 ms typ.
    Ms2_0,
    /// 1.2 ms typ.
    Ms1_2,
}

impl AdcTime {
    /// Worst-case single conversion time in µs. The data sheet specifies
    /// 7.5 ms max for the 6.4 ms setting; the shorter settings use the same
    /// +17 % margin, rounded up.
    pub const fn max_us(self) -> WaitUs {
        match self {
            Self::Ms6_4 => 7_500,
            Self::Ms3_5 => 4_100,
            Self::Ms2_0 => 2_400,
            Self::Ms1_2 => 1_500,
        }
    }
}

/// Continuous-mode conversion period (Conv_Period).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConversionPeriod {
    /// Back-to-back conversions (6.4 ms).
    Ms6_4,
    /// 31.25 ms / 32 Hz.
    Hz32,
    /// 62.5 ms / 16 Hz.
    Hz16,
    /// 125 ms / 8 Hz.
    Hz8,
    /// 250 ms / 4 Hz (default).
    #[default]
    Hz4,
    /// 500 ms / 2 Hz.
    Hz2,
    /// 1 s.
    Hz1,
    /// 2 s.
    Hz0_5,
}

/// Configuration written by [`Tmp114::configure`] and every one-shot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// ADC conversion time.
    pub adc_time: AdcTime,
    /// Average 8 conversions per result (also in one-shot).
    pub averaging: bool,
    /// Period used in continuous mode.
    pub period: ConversionPeriod,
}

impl Config {
    /// Configuration register word.
    pub const fn register(self, shutdown: bool, one_shot: bool) -> u16 {
        let mut word = ((self.adc_time as u16) << 9) | self.period as u16;
        if self.averaging {
            word |= CFG_AVG;
        }
        if shutdown {
            word |= CFG_MODE_SHUTDOWN;
        }
        if one_shot {
            word |= CFG_OS;
        }
        word
    }

    /// Worst-case time of one (possibly averaged) conversion in µs.
    pub const fn conversion_us(self) -> WaitUs {
        if self.averaging {
            self.adc_time.max_us() * 8
        } else {
            self.adc_time.max_us()
        }
    }
}

/// Alert_Status register. Reading it clears the `*_flag` bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AlertStatus(pub u16);

impl AlertStatus {
    /// A new result is available since the last status read.
    pub const fn data_ready(self) -> bool {
        self.0 & 1 != 0
    }
    /// Last result crossed below `T_LOW` (latched until read).
    pub const fn low_flag(self) -> bool {
        self.0 & (1 << 1) != 0
    }
    /// Last result crossed above `T_HIGH` (latched until read).
    pub const fn high_flag(self) -> bool {
        self.0 & (1 << 2) != 0
    }
    /// Temperature is below `T_LOW` (until above `T_LOW + hysteresis`).
    pub const fn low_status(self) -> bool {
        self.0 & (1 << 3) != 0
    }
    /// Temperature is above `T_HIGH` (until below `T_HIGH - hysteresis`).
    pub const fn high_status(self) -> bool {
        self.0 & (1 << 4) != 0
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

/// Decode Temp_Result (16-bit two's complement, 7.8125 m°C/LSB, table 8-1)
/// to 0.01 °C, rounded.
pub const fn decode_temperature(raw: u16) -> i32 {
    ((raw as i16 as i32) * 100 + 64) >> 7
}

/// Encode a limit in 0.01 °C (14-bit, 0.03125 °C/LSB, left-aligned in the
/// 16-bit word), saturating.
pub const fn encode_limit(centi_celsius: i32) -> u16 {
    let counts = (centi_celsius * 32 + 50).div_euclid(100);
    let c = if counts < -8192 {
        -8192
    } else if counts > 8191 {
        8191
    } else {
        counts
    };
    (c << 2) as u16
}

/// Encode a hysteresis in 0.01 °C (unsigned, 0.5 °C/LSB), saturating.
pub const fn encode_hysteresis(centi_celsius: u32) -> u8 {
    let counts = (centi_celsius + 25) / 50;
    if counts > 255 { 255 } else { counts as u8 }
}

macro_rules! constructors {
    () => {
        /// Create a driver with the default [`Config`]; no bus traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                config: Config {
                    adc_time: AdcTime::Ms6_4,
                    averaging: false,
                    period: ConversionPeriod::Hz4,
                },
            }
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
        impl<I2C: $i2c> Tmp114<I2C> {
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

            /// Check the Device_ID register (ID 0x114, any revision).
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let id = self.read_register(REG_DEVICE_ID) $($await)*?;
                if id & 0x0FFF != DEVICE_ID {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(())
            }

            /// Software reset. Wait [`RESET_WAIT_US`] before the next
            /// command; the part restarts in continuous mode.
            pub $($async)? fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(REG_CONFIG, CFG_RESET) $($await)*?;
                Ok(RESET_WAIT_US)
            }

            /// Write the configuration and enter shutdown. Call once after
            /// power-up: the part starts in continuous 4 Hz mode.
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(true, false)) $($await)*
            }

            /// Change the configuration and write it (enters shutdown).
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program the limits in 0.01 °C. Limits are ignored by the part
            /// if `high <= low`.
            pub $($async)? fn set_limits(&mut self, low: i32, high: i32) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_T_LOW, encode_limit(low)) $($await)*?;
                self.write_register(REG_T_HIGH, encode_limit(high)) $($await)*
            }

            /// Program the hysteresis of the high and low limits in 0.01 °C
            /// (0.5 °C steps, up to 127.5 °C).
            pub $($async)? fn set_hysteresis(&mut self, high: u32, low: u32) -> Result<(), Error<I2C::Error>> {
                let word = u16::from_be_bytes([encode_hysteresis(high), encode_hysteresis(low)]);
                self.write_register(REG_HYSTERESIS, word) $($await)*
            }

            /// Read (and clear the flags of) the Alert_Status register.
            pub $($async)? fn alert_status(&mut self) -> Result<AlertStatus, Error<I2C::Error>> {
                Ok(AlertStatus(self.read_register(REG_ALERT_STATUS) $($await)*?))
            }
        }

        impl<I2C: $i2c> $sensor for Tmp114<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot conversion (OS = 1, Mode = shutdown).
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(true, true)) $($await)*?;
                Ok(self.config.conversion_us())
            }

            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let raw = self.read_register(REG_TEMP_RESULT) $($await)*?;
                Ok(Measurement { temperature: decode_temperature(raw) })
            }
        }

        impl<I2C: $i2c> $low_power for Tmp114<I2C> {
            type BusError = I2C::Error;

            /// Shutdown (Mode = 1).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.configure() $($await)*
            }

            /// Continuous conversion at the configured period. Returns the
            /// wait until the first result.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(false, false)) $($await)*?;
                Ok(self.config.conversion_us())
            }
        }
    };
}

/// Blocking driver.
pub struct Tmp114<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
}

impl<I2C> Tmp114<I2C> {
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

    /// Async twin of [`super::Tmp114`].
    pub struct Tmp114<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
    }

    impl<I2C> Tmp114<I2C> {
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

    const A: u8 = ADDRESS_A;

    #[test]
    fn decodes_datasheet_table() {
        // SNIS214E table 8-1.
        assert_eq!(decode_temperature(0x3E80), 12_500);
        assert_eq!(decode_temperature(0x0C80), 2_500);
        assert_eq!(decode_temperature(0x0001), 1);
        assert_eq!(decode_temperature(0x0000), 0);
        assert_eq!(decode_temperature(0xFFFF), -1);
        assert_eq!(decode_temperature(0xF380), -2_500);
        assert_eq!(decode_temperature(0xEC00), -4_000);
    }

    #[test]
    fn limit_and_hysteresis_encoding() {
        // Reset values: T_LOW = F380h (-25 °C), T_HIGH = 2A80h (85 °C),
        // hysteresis 0Ah = 5 °C.
        assert_eq!(encode_limit(-2_500), 0xF380);
        assert_eq!(encode_limit(8_500), 0x2A80);
        assert_eq!(encode_limit(30_000), 0x7FFC);
        assert_eq!(encode_hysteresis(500), 0x0A);
        assert_eq!(encode_hysteresis(100_000), 0xFF);
    }

    #[test]
    fn config_word() {
        // Reset value 0004h: continuous, 4 Hz.
        assert_eq!(Config::default().register(false, false), 0x0004);
        let c = Config {
            adc_time: AdcTime::Ms1_2,
            averaging: true,
            period: ConversionPeriod::Hz0_5,
        };
        assert_eq!(c.register(true, true), 0x069F);
        assert_eq!(c.conversion_us(), 12_000);
    }

    #[test]
    fn probe_accepts_any_revision() {
        let expectations = [
            Transaction::write_read(A, vec![REG_DEVICE_ID], vec![0x11, 0x14]),
            Transaction::write_read(A, vec![REG_DEVICE_ID], vec![0x21, 0x14]),
            Transaction::write_read(A, vec![REG_DEVICE_ID], vec![0x01, 0x17]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp114::new(&mut i2c, A);
        assert_eq!(tmp.probe(), Ok(()));
        assert_eq!(tmp.probe(), Ok(()));
        assert_eq!(tmp.probe(), Err(Error::UnexpectedId(0x0117)));
        i2c.done();
    }

    #[test]
    fn one_shot_then_read() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x00, 0x1C]),
            Transaction::write_read(A, vec![REG_TEMP_RESULT], vec![0x0C, 0x80]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp114::new(&mut i2c, A);
        assert_eq!(tmp.start_measurement().unwrap(), 7_500);
        assert_eq!(tmp.read_measurement().unwrap().temperature, 2_500);
        i2c.done();
    }

    #[test]
    fn thresholds_status_reset_and_power() {
        let expectations = [
            Transaction::write(A, vec![REG_T_LOW, 0xF3, 0x80]),
            Transaction::write(A, vec![REG_T_HIGH, 0x2A, 0x80]),
            Transaction::write(A, vec![REG_HYSTERESIS, 0x04, 0x02]),
            Transaction::write_read(A, vec![REG_ALERT_STATUS], vec![0x00, 0x15]),
            Transaction::write(A, vec![REG_CONFIG, 0x01, 0x00]),
            Transaction::write(A, vec![REG_CONFIG, 0x00, 0x0C]),
            Transaction::write(A, vec![REG_CONFIG, 0x00, 0x04]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = Tmp114::new(&mut i2c, A);
        tmp.set_limits(-2_500, 8_500).unwrap();
        tmp.set_hysteresis(200, 100).unwrap();
        let status = tmp.alert_status().unwrap();
        assert!(status.data_ready() && status.high_flag() && status.high_status());
        assert!(!status.low_flag() && !status.low_status());
        assert_eq!(tmp.soft_reset().unwrap(), RESET_WAIT_US);
        tmp.sleep().unwrap();
        assert_eq!(tmp.wake().unwrap(), 7_500);
        i2c.done();
    }

    #[test]
    fn async_one_shot_with_averaging() {
        use zigbee_sensor::asynch::Sensor as _;
        let config = Config {
            averaging: true,
            ..Config::default()
        };
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x00, 0x9C]),
            Transaction::write_read(A, vec![REG_TEMP_RESULT], vec![0xEC, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tmp = asynch::Tmp114::new(&mut i2c, A).with_config(config);
        assert_eq!(block_on(tmp.start_measurement()).unwrap(), 60_000);
        assert_eq!(
            block_on(tmp.read_measurement()).unwrap().temperature,
            -4_000
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
