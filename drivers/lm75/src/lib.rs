//! LM75-family digital temperature sensors (I2C / SMBus): NXP LM75A, LM75B,
//! PCT2075, TI TMP75 and onsemi NCT75.
//!
//! All parts share the LM75 register map (temperature, configuration,
//! T_HYST/T_LOW, T_OS/T_HIGH) and differ in resolution and one-shot support:
//!
//! | Variant | Result | Limits | Single conversion |
//! |---|---|---|---|
//! | LM75A, LM75B, PCT2075 | 11-bit, 0.125 °C | 9-bit, 0.5 °C | emulated: leave shutdown, read, shut down |
//! | TMP75 | 9–12-bit (R1:R0) | 12-bit | OS bit in the configuration register |
//! | NCT75 | 12-bit, 0.0625 °C | 12-bit | one-shot mode (D5) + write to register 0x04 |
//!
//! [`configure`](Lm75::configure) leaves the sensor in shutdown. On the
//! parts without a one-shot,
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! returns the part to normal mode (a conversion starts immediately) and
//! [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//! shuts it down again after reading.
//!
//! References: NXP LM75A Rev. 04 (July 2007), NXP LM75B Rev. 6.1 (February
//! 2015), NXP PCT2075 Rev. 10.1 (September 2021), TI SBOS288M (TMPx75,
//! December 2020), onsemi NCT75/D Rev. 7 (June 2018).
//!
//! ```ignore
//! let mut lm = Lm75::new(i2c, ADDRESS_DEFAULT, Variant::Tmp75);
//! lm.configure()?;
//! let wait_us = lm.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = lm.read_measurement()?;
//! ```
//!
//! The blocking driver and its [`asynch`] twin are generated from one macro
//! body, so they issue identical bus transactions.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Address with A2 = A1 = A0 = GND (0x48..=0x4F with A2..A0). PCT2075 has
/// 27 addresses with tri-state pins; see its data sheet table 5.
pub const ADDRESS_DEFAULT: u8 = 0x48;

const REG_TEMPERATURE: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;
const REG_T_LOW: u8 = 0x02;
const REG_T_HIGH: u8 = 0x03;
/// PCT2075 Tidle, NCT75 one-shot trigger.
const REG_04: u8 = 0x04;

const CFG_SHUTDOWN: u8 = 1 << 0;
const CFG_INTERRUPT: u8 = 1 << 1;
const CFG_POL: u8 = 1 << 2;
const CFG_NCT75_ONE_SHOT_MODE: u8 = 1 << 5;
const CFG_TMP75_OS: u8 = 1 << 7;

/// Supported part.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// NXP LM75A: 11-bit, 100 ms typ. conversion, no one-shot.
    Lm75a,
    /// NXP LM75B: 11-bit, 10 ms typ. conversion, no one-shot.
    Lm75b,
    /// NXP PCT2075: 11-bit, 28 ms typ. conversion, Tidle sample period, no
    /// one-shot.
    Pct2075,
    /// TI TMP75: 9–12-bit, OS one-shot.
    Tmp75,
    /// onsemi NCT75: 12-bit, one-shot register.
    Nct75,
}

impl Variant {
    const fn result_mask(self) -> u16 {
        match self {
            Self::Lm75a | Self::Lm75b | Self::Pct2075 => 0xFFE0,
            Self::Tmp75 | Self::Nct75 => 0xFFF0,
        }
    }

    const fn limit_bits(self) -> u32 {
        match self {
            Self::Lm75a | Self::Lm75b | Self::Pct2075 => 9,
            Self::Tmp75 | Self::Nct75 => 12,
        }
    }

    const fn has_one_shot(self) -> bool {
        matches!(self, Self::Tmp75 | Self::Nct75)
    }

    /// Wait for one conversion at `resolution` (TMP75 only) in µs. TMP75:
    /// data sheet maximum. Other parts only give typical values; the driver
    /// uses typical × 1.5 (LM75A 100 ms, LM75B 10 ms, PCT2075 28 ms, NCT75
    /// 48.5 ms).
    pub const fn conversion_us(self, resolution: Resolution) -> WaitUs {
        match self {
            Self::Lm75a => 150_000,
            Self::Lm75b => 15_000,
            Self::Pct2075 => 42_000,
            Self::Nct75 => 73_000,
            Self::Tmp75 => match resolution {
                Resolution::Bits9 => 37_500,
                Resolution::Bits10 => 75_000,
                Resolution::Bits11 => 150_000,
                Resolution::Bits12 => 300_000,
            },
        }
    }
}

/// TMP75 converter resolution (R1:R0). Ignored by the other parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 0.5 °C, 27.5 ms typ. (TMP75 power-up default).
    Bits9,
    /// 0.25 °C, 55 ms typ.
    Bits10,
    /// 0.125 °C, 110 ms typ.
    Bits11,
    /// 0.0625 °C, 220 ms typ.
    #[default]
    Bits12,
}

/// Consecutive faults before OS/ALERT changes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FaultQueue {
    /// 1 fault (default).
    #[default]
    One,
    /// 2 faults.
    Two,
    /// 4 faults (TMP75: 3).
    Four,
    /// 6 faults (TMP75: 4).
    Six,
}

/// OS/ALERT pin behaviour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AlertMode {
    /// Active while `T > T_OS`, released below `T_HYST` (default).
    #[default]
    Comparator,
    /// Pulse-latched; cleared by any register read.
    Interrupt,
}

/// Configuration written by [`Lm75::configure`] and every mode change.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// TMP75 resolution.
    pub resolution: Resolution,
    /// OS/ALERT fault queue.
    pub fault_queue: FaultQueue,
    /// Comparator / interrupt mode.
    pub alert_mode: AlertMode,
    /// OS/ALERT active high instead of active low.
    pub alert_active_high: bool,
}

#[derive(Clone, Copy)]
enum Mode {
    Normal,
    Shutdown,
    OneShot,
}

impl Config {
    const fn register(self, variant: Variant, mode: Mode) -> u8 {
        let mut byte = (self.fault_queue as u8) << 3;
        if self.alert_active_high {
            byte |= CFG_POL;
        }
        if matches!(self.alert_mode, AlertMode::Interrupt) {
            byte |= CFG_INTERRUPT;
        }
        match variant {
            Variant::Tmp75 => {
                byte |= (self.resolution as u8) << 5;
                match mode {
                    Mode::Normal => {}
                    Mode::Shutdown => byte |= CFG_SHUTDOWN,
                    Mode::OneShot => byte |= CFG_SHUTDOWN | CFG_TMP75_OS,
                }
            }
            Variant::Nct75 => match mode {
                Mode::Normal => {}
                Mode::Shutdown | Mode::OneShot => byte |= CFG_NCT75_ONE_SHOT_MODE,
            },
            Variant::Lm75a | Variant::Lm75b | Variant::Pct2075 => match mode {
                Mode::Normal | Mode::OneShot => {}
                Mode::Shutdown => byte |= CFG_SHUTDOWN,
            },
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

/// Decode the left-justified temperature register to 0.01 °C, rounded.
/// The unused LSBs are masked (they are undefined on the 11-bit parts).
/// Returns `None` for a 12-bit part whose always-zero low nibble is set
/// (e.g. `0xFFFF` from a missing device).
pub const fn decode_temperature(variant: Variant, raw: u16) -> Option<i32> {
    let mask = variant.result_mask();
    if mask == 0xFFF0 && raw & 0x000F != 0 {
        return None;
    }
    let word = (raw & mask) as i16 as i32;
    Some((word * 100 + 128) >> 8)
}

/// Encode a limit in 0.01 °C into the variant's `T_HYST`/`T_OS` format
/// (9-bit 0.5 °C or 12-bit 0.0625 °C, left-justified), saturating.
pub const fn encode_limit(variant: Variant, centi_celsius: i32) -> u16 {
    let bits = variant.limit_bits();
    let scale = 1i32 << (bits - 8);
    let max = (1i32 << (bits - 1)) - 1;
    let counts = (centi_celsius * scale + 50).div_euclid(100);
    let c = if counts < -max - 1 {
        -max - 1
    } else if counts > max {
        max
    } else {
        counts
    };
    (c << (16 - bits)) as u16
}

macro_rules! constructors {
    () => {
        /// Create a driver with the default [`Config`]; no bus traffic.
        pub const fn new(i2c: I2C, address: u8, variant: Variant) -> Self {
            Self {
                i2c,
                address,
                variant,
                config: Config {
                    resolution: Resolution::Bits12,
                    fault_queue: FaultQueue::One,
                    alert_mode: AlertMode::Comparator,
                    alert_active_high: false,
                },
                conversion_wait_us: None,
                auto_shutdown: false,
            }
        }

        /// Use `config` for [`Self::configure`] and later mode changes.
        pub const fn with_config(mut self, config: Config) -> Self {
            self.config = config;
            self
        }

        /// Override the conversion wait returned by `start_measurement` and
        /// `wake` (e.g. for a second-source part).
        pub const fn with_conversion_wait_us(mut self, wait_us: WaitUs) -> Self {
            self.conversion_wait_us = Some(wait_us);
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        fn conversion_wait(&self) -> WaitUs {
            match self.conversion_wait_us {
                Some(us) => us,
                None => self.variant.conversion_us(self.config.resolution),
            }
        }
    };
}

macro_rules! driver_impl {
    ($i2c:path, $sensor:path, $low_power:path, $($async:ident)?; $($await:tt)*) => {
        impl<I2C: $i2c> Lm75<I2C> {
            $($async)? fn write_config(&mut self, mode: Mode) -> Result<(), Error<I2C::Error>> {
                let byte = self.config.register(self.variant, mode);
                self.i2c
                    .write(self.address, &[REG_CONFIG, byte])
                    $($await)*
                    .map_err(Error::bus)
            }

            $($async)? fn write_register(&mut self, register: u8, value: u16) -> Result<(), Error<I2C::Error>> {
                let [hi, lo] = value.to_be_bytes();
                self.i2c
                    .write(self.address, &[register, hi, lo])
                    $($await)*
                    .map_err(Error::bus)
            }

            /// Write the configuration and enter shutdown (NCT75: one-shot
            /// mode, which is powered down between conversions).
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                self.auto_shutdown = false;
                self.write_config(Mode::Shutdown) $($await)*
            }

            /// Change the configuration and write it (enters shutdown).
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program the thresholds in 0.01 °C: `hyst` (T_HYST / T_LOW) and
            /// `os` (T_OS / T_HIGH). The 11-bit parts round to 0.5 °C.
            pub $($async)? fn set_limits(&mut self, hyst: i32, os: i32) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_T_LOW, encode_limit(self.variant, hyst)) $($await)*?;
                self.write_register(REG_T_HIGH, encode_limit(self.variant, os)) $($await)*
            }

            /// PCT2075 only: continuous-mode sample period in 100 ms units
            /// (1..=31; 0 acts as 1). [`Error::Unsupported`] on other parts.
            pub $($async)? fn set_sample_period(&mut self, units_100ms: u8) -> Result<(), Error<I2C::Error>> {
                if !matches!(self.variant, Variant::Pct2075) {
                    return Err(Error::Unsupported);
                }
                self.i2c
                    .write(self.address, &[REG_04, units_100ms & 0x1F])
                    $($await)*
                    .map_err(Error::bus)
            }
        }

        impl<I2C: $i2c> $sensor for Lm75<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// TMP75: OS = 1 from shutdown. NCT75: write the one-shot
            /// register. Others: leave shutdown; `read_measurement` shuts
            /// the part down again.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                match self.variant {
                    Variant::Nct75 => {
                        self.i2c
                            .write(self.address, &[REG_04, 0x00])
                            $($await)*
                            .map_err(Error::bus)?;
                    }
                    _ => {
                        self.write_config(Mode::OneShot) $($await)*?;
                        self.auto_shutdown = !self.variant.has_one_shot();
                    }
                }
                Ok(self.conversion_wait())
            }

            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(self.address, &[REG_TEMPERATURE], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                if self.auto_shutdown {
                    self.configure() $($await)*?;
                }
                match decode_temperature(self.variant, u16::from_be_bytes(buf)) {
                    Some(temperature) => Ok(Measurement { temperature }),
                    None => Err(Error::InvalidData),
                }
            }
        }

        impl<I2C: $i2c> $low_power for Lm75<I2C> {
            type BusError = I2C::Error;

            /// Shutdown.
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.configure() $($await)*
            }

            /// Continuous conversion. Returns the wait until the first
            /// result.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.auto_shutdown = false;
                self.write_config(Mode::Normal) $($await)*?;
                Ok(self.conversion_wait())
            }
        }
    };
}

/// Blocking driver.
pub struct Lm75<I2C> {
    i2c: I2C,
    address: u8,
    variant: Variant,
    config: Config,
    conversion_wait_us: Option<WaitUs>,
    auto_shutdown: bool,
}

impl<I2C> Lm75<I2C> {
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

    /// Async twin of [`super::Lm75`].
    pub struct Lm75<I2C> {
        i2c: I2C,
        address: u8,
        variant: Variant,
        config: Config,
        conversion_wait_us: Option<WaitUs>,
        auto_shutdown: bool,
    }

    impl<I2C> Lm75<I2C> {
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
    fn decodes_11_bit_table() {
        // LM75B Rev. 6.1 table 10 (11-bit value << 5, low bits undefined).
        let v = Variant::Lm75b;
        assert_eq!(decode_temperature(v, 0x3F8 << 5), Some(12_700));
        assert_eq!(decode_temperature(v, 0x3F7 << 5), Some(12_688));
        assert_eq!(decode_temperature(v, 0x3E8 << 5), Some(12_500));
        assert_eq!(decode_temperature(v, (0x0C8 << 5) | 0x1F), Some(2_500));
        assert_eq!(decode_temperature(v, 0x001 << 5), Some(13));
        assert_eq!(decode_temperature(v, 0xFFE0), Some(-12));
        assert_eq!(decode_temperature(v, 0xC900), Some(-5_500));
    }

    #[test]
    fn decodes_12_bit_table() {
        // NCT75/D table 6 (12-bit value << 4).
        let v = Variant::Nct75;
        assert_eq!(decode_temperature(v, 0xC900), Some(-5_500));
        assert_eq!(decode_temperature(v, 0xE700), Some(-2_500));
        assert_eq!(decode_temperature(v, 0xFFF0), Some(-6));
        assert_eq!(decode_temperature(v, 0x0010), Some(6));
        assert_eq!(decode_temperature(v, 0x4B40), Some(7_525));
        assert_eq!(decode_temperature(v, 0x7D00), Some(12_500));
        assert_eq!(decode_temperature(v, 0xFFFF), None);
    }

    #[test]
    fn limits_per_variant() {
        // LM75B table 13: 0FAh = 125 °C, 032h = 25 °C (9-bit << 7).
        assert_eq!(encode_limit(Variant::Lm75b, 12_500), 0x0FA << 7);
        assert_eq!(encode_limit(Variant::Lm75b, 2_520), 0x032 << 7);
        assert_eq!(encode_limit(Variant::Pct2075, -100_000), 0x8000);
        assert_eq!(encode_limit(Variant::Tmp75, 7_525), 0x4B40);
        assert_eq!(encode_limit(Variant::Nct75, 30_000), 0x7FF0);
    }

    #[test]
    fn config_bytes() {
        let c = Config {
            resolution: Resolution::Bits10,
            fault_queue: FaultQueue::Six,
            alert_mode: AlertMode::Interrupt,
            alert_active_high: true,
        };
        assert_eq!(c.register(Variant::Tmp75, Mode::OneShot), 0xBF);
        assert_eq!(c.register(Variant::Nct75, Mode::Shutdown), 0x3E);
        assert_eq!(c.register(Variant::Lm75a, Mode::Shutdown), 0x1F);
        assert_eq!(c.register(Variant::Pct2075, Mode::OneShot), 0x1E);
    }

    #[test]
    fn tmp75_one_shot() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x61]),
            Transaction::write(A, vec![REG_CONFIG, 0xE1]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x19, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut lm = Lm75::new(&mut i2c, A, Variant::Tmp75);
        lm.configure().unwrap();
        assert_eq!(lm.start_measurement().unwrap(), 300_000);
        assert_eq!(lm.read_measurement().unwrap().temperature, 2_500);
        i2c.done();
    }

    #[test]
    fn nct75_one_shot_register() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x20]),
            Transaction::write(A, vec![REG_04, 0x00]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0xFF, 0xFF]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut lm = Lm75::new(&mut i2c, A, Variant::Nct75);
        lm.configure().unwrap();
        assert_eq!(lm.start_measurement().unwrap(), 73_000);
        assert_eq!(lm.read_measurement(), Err(Error::InvalidData));
        i2c.done();
    }

    #[test]
    fn lm75b_emulated_one_shot_and_power() {
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x00]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x19, 0x1F]),
            Transaction::write(A, vec![REG_CONFIG, 0x01]),
            Transaction::write(A, vec![REG_CONFIG, 0x00]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0x19, 0x00]),
            Transaction::write(A, vec![REG_CONFIG, 0x01]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut lm = Lm75::new(&mut i2c, A, Variant::Lm75b);
        assert_eq!(lm.start_measurement().unwrap(), 15_000);
        assert_eq!(lm.read_measurement().unwrap().temperature, 2_500);
        assert_eq!(lm.wake().unwrap(), 15_000);
        // Continuous mode: reading does not shut the part down.
        assert_eq!(lm.read_measurement().unwrap().temperature, 2_500);
        lm.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn pct2075_limits_and_tidle() {
        let expectations = [
            Transaction::write(A, vec![REG_T_LOW, 0x4B, 0x00]),
            Transaction::write(A, vec![REG_T_HIGH, 0x50, 0x00]),
            Transaction::write(A, vec![REG_04, 0x0A]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut lm = Lm75::new(&mut i2c, A, Variant::Pct2075).with_conversion_wait_us(30_000);
        lm.set_limits(7_500, 8_000).unwrap();
        lm.set_sample_period(10).unwrap();
        i2c.done();
        let mut i2c = Mock::new(&[]);
        let mut lm = Lm75::new(&mut i2c, A, Variant::Nct75);
        assert_eq!(lm.set_sample_period(10), Err(Error::Unsupported));
        i2c.done();
    }

    #[test]
    fn async_pct2075_emulated_one_shot() {
        use zigbee_sensor::asynch::Sensor as _;
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x00]),
            Transaction::write_read(A, vec![REG_TEMPERATURE], vec![0xE7, 0x00]),
            Transaction::write(A, vec![REG_CONFIG, 0x01]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut lm =
            asynch::Lm75::new(&mut i2c, A, Variant::Pct2075).with_conversion_wait_us(30_000);
        assert_eq!(block_on(lm.start_measurement()).unwrap(), 30_000);
        assert_eq!(block_on(lm.read_measurement()).unwrap().temperature, -2_500);
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
