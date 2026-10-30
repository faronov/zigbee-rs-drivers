//! Analog Devices (Maxim) MAX44009 ambient light sensor (I2C).
//!
//! 22-bit dynamic range (0.045 lx … 188 klx) reported as a 4-bit exponent
//! and 8-bit mantissa: `lux = 2^E · M · 0.045`. The part always measures
//! (there is no shutdown); the lowest-power state is the default mode,
//! one auto-ranged conversion every 800 ms at 0.65 µA. There is no
//! data-ready flag either, so `start_measurement` only (re)writes the
//! configuration and, the first time, returns one full conversion period.
//!
//! The two lux bytes are read with a repeated START between them, as the
//! datasheet requires for a consistent exponent/mantissa pair.
//!
//! Reference: Maxim MAX44009 datasheet (19-5861); register handling
//! cross-checked with Linux `iio/light/max44009.c`.
//!
//! ```ignore
//! let mut als = Max44009::new(i2c, ADDRESS_A0_LOW);
//! let m = als.measure(&mut delay)?;
//! ```

#![no_std]

use embedded_hal::i2c::{I2c, Operation};
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with A0 = GND.
pub const ADDRESS_A0_LOW: u8 = 0x4A;
/// I2C address with A0 = VCC.
pub const ADDRESS_A0_HIGH: u8 = 0x4B;

const REG_INT_STATUS: u8 = 0x00;
const REG_INT_EN: u8 = 0x01;
const REG_CONFIG: u8 = 0x02;
const REG_LUX_HIGH: u8 = 0x03;
const REG_LUX_LOW: u8 = 0x04;
const REG_UPPER_THRESHOLD: u8 = 0x05;
const REG_LOWER_THRESHOLD: u8 = 0x06;
const REG_THRESHOLD_TIMER: u8 = 0x07;

const CONFIG_CONT: u8 = 1 << 7;
const CONFIG_MANUAL: u8 = 1 << 6;

/// Milli-lux per mantissa LSB at exponent 0.
const MLX_PER_COUNT: u32 = 45;
/// Largest representable value (E = 14, M = 255).
pub const MAX_MILLILUX: u32 = (255 << 14) * MLX_PER_COUNT;

/// Manual-mode integration time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum IntegrationTime {
    #[default]
    Ms800,
    Ms400,
    Ms200,
    Ms100,
    Ms50,
    Ms25,
    Ms12_5,
    Ms6_25,
}

impl IntegrationTime {
    /// Integration time in microseconds.
    pub const fn us(self) -> u32 {
        800_000 >> self as u32
    }
}

/// Exponent / mantissa → milli-lux.
pub const fn to_millilux(exponent: u8, mantissa: u8) -> u32 {
    ((mantissa as u32) << exponent) * MLX_PER_COUNT
}

/// Milli-lux → threshold register byte (exponent in bits 7:4, the upper
/// mantissa nibble in bits 3:0; truncating).
pub const fn threshold_byte(millilux: u32) -> u8 {
    let mut raw = millilux / MLX_PER_COUNT;
    let mut exp = 0u8;
    while raw > 0xFF {
        raw >>= 1;
        exp += 1;
    }
    if exp > 14 {
        0xEF
    } else {
        (exp << 4) | (raw >> 4) as u8
    }
}

/// Threshold register byte → milli-lux (the upper threshold's low
/// mantissa nibble reads as all ones, the lower one's as zeros).
pub const fn threshold_millilux(byte: u8, upper: bool) -> u32 {
    let mantissa = (byte & 0x0F) << 4 | if upper { 0x0F } else { 0 };
    to_millilux(byte >> 4, mantissa)
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux ([`MAX_MILLILUX`] on overrange).
    pub illuminance: u32,
    /// Raw exponent (15 = overrange).
    pub exponent: u8,
    /// Raw 8-bit mantissa.
    pub mantissa: u8,
    /// Overrange (exponent 15).
    pub saturated: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// MAX44009 driver.
        pub struct Max44009<I2C> {
            i2c: I2C,
            address: u8,
            continuous: bool,
            manual: Option<IntegrationTime>,
            configured: bool,
        }

        impl<I2C> Max44009<I2C> {
            /// Create a driver (default mode: auto-range, 800 ms cycle);
            /// no bus traffic.
            pub const fn new(i2c: I2C, address: u8) -> Self {
                Self {
                    i2c,
                    address,
                    continuous: false,
                    manual: None,
                    configured: false,
                }
            }

            /// Convert back-to-back instead of once per 800 ms.
            pub const fn with_continuous(mut self, continuous: bool) -> Self {
                self.continuous = continuous;
                self
            }

            /// Fixed integration time (`None` = automatic range).
            pub const fn with_manual(mut self, integration: Option<IntegrationTime>) -> Self {
                self.manual = integration;
                self
            }

            /// Change the manual integration time for the next
            /// `start_measurement`.
            pub fn set_manual(&mut self, integration: Option<IntegrationTime>) {
                self.manual = integration;
                self.configured = false;
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }

            const fn config(&self) -> u8 {
                let cont = if self.continuous { CONFIG_CONT } else { 0 };
                match self.manual {
                    Some(it) => cont | CONFIG_MANUAL | it as u8,
                    None => cont,
                }
            }

            const fn period_us(&self) -> u32 {
                let t = match self.manual {
                    Some(it) => it.us(),
                    None => 800_000,
                };
                t + t / 10
            }
        }

        impl<I2C: I2c> Max44009<I2C> {
            $($async_)? fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(self.address, &[reg, value])
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn read(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
                let mut buf = [0u8; 1];
                self.i2c
                    .write_read(self.address, &[reg], &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                Ok(buf[0])
            }

            /// Presence check (the part has no ID register): reads `CONFIG`.
            pub $($async_)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                self.read(REG_CONFIG) $($await_)* ?;
                Ok(())
            }

            /// Configure the threshold interrupt (`None` disables it).
            /// Thresholds are in milli-lux (encoded with [`threshold_byte`]);
            /// `timer` is the threshold-timer in 100 ms steps.
            pub $($async_)? fn set_interrupt(
                &mut self,
                window: Option<(u32, u32)>,
                timer: u8,
            ) -> Result<(), Error<I2C::Error>> {
                let Some((low, high)) = window else {
                    return self.write(REG_INT_EN, 0) $($await_)*;
                };
                self.write(REG_UPPER_THRESHOLD, threshold_byte(high)) $($await_)* ?;
                self.write(REG_LOWER_THRESHOLD, threshold_byte(low)) $($await_)* ?;
                self.write(REG_THRESHOLD_TIMER, timer) $($await_)* ?;
                self.write(REG_INT_EN, 1) $($await_)*
            }

            /// Read (and so clear) the interrupt status.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<bool, Error<I2C::Error>> {
                Ok(self.read(REG_INT_STATUS) $($await_)* ? & 1 != 0)
            }
        }

        impl<I2C: I2c> Sensor for Max44009<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Write `CONFIG` if needed; returns one conversion period after
            /// a (re)configuration, otherwise 0 (the part is free-running).
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.configured {
                    return Ok(0);
                }
                self.write(REG_CONFIG, self.config()) $($await_)* ?;
                self.configured = true;
                Ok(self.period_us())
            }

            /// Read the latest exponent / mantissa (repeated START between
            /// the two bytes).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut hi = [0u8; 1];
                let mut lo = [0u8; 1];
                self.i2c
                    .transaction(
                        self.address,
                        &mut [
                            Operation::Write(&[REG_LUX_HIGH]),
                            Operation::Read(&mut hi),
                            Operation::Write(&[REG_LUX_LOW]),
                            Operation::Read(&mut lo),
                        ],
                    )
                    $($await_)*
                    .map_err(Error::bus)?;
                let exponent = hi[0] >> 4;
                let mantissa = (hi[0] << 4) | (lo[0] & 0x0F);
                let saturated = exponent == 0x0F;
                Ok(Measurement {
                    illuminance: if saturated {
                        MAX_MILLILUX
                    } else {
                        to_millilux(exponent, mantissa)
                    },
                    exponent,
                    mantissa,
                    saturated,
                })
            }
        }

        impl<I2C: I2c> LowPower for Max44009<I2C> {
            type BusError = I2C::Error;

            /// Return to the default mode (auto-range, one conversion per
            /// 800 ms, 0.65 µA); the configured mode is restored by the next
            /// `start_measurement`.
            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_CONFIG, 0) $($await_)* ?;
                self.configured = self.config() == 0;
                Ok(())
            }

            /// Nothing to do.
            $($async_)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                Ok(0)
            }
        }
    };
}

driver!(,);

/// Async driver.
pub mod asynch {
    use super::{
        CONFIG_CONT, CONFIG_MANUAL, IntegrationTime, MAX_MILLILUX, Measurement, REG_CONFIG,
        REG_INT_EN, REG_INT_STATUS, REG_LOWER_THRESHOLD, REG_LUX_HIGH, REG_LUX_LOW,
        REG_THRESHOLD_TIMER, REG_UPPER_THRESHOLD, threshold_byte, to_millilux,
    };
    use embedded_hal_async::i2c::{I2c, Operation};
    use zigbee_sensor::asynch::{LowPower, Sensor};
    use zigbee_sensor::{Error, WaitUs};

    driver!(async, .await);
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;

    const A: u8 = ADDRESS_A0_LOW;

    fn lux_read(a: u8, hi: u8, lo: u8) -> [Transaction; 6] {
        [
            Transaction::transaction_start(a),
            Transaction::write(a, vec![0x03]),
            Transaction::read(a, vec![hi]),
            Transaction::write(a, vec![0x04]),
            Transaction::read(a, vec![lo]),
            Transaction::transaction_end(a),
        ]
    }

    #[test]
    fn helpers() {
        assert_eq!(MAX_MILLILUX, 188_006_400);
        assert_eq!(to_millilux(0, 1), 45);
        assert_eq!(threshold_byte(100_000), 0x48);
        assert_eq!(threshold_byte(10), 0x00);
        assert_eq!(threshold_byte(u32::MAX), 0xEF);
        assert_eq!(threshold_millilux(0x48, false), 92_160);
        assert_eq!(threshold_millilux(0x48, true), 102_960);
        assert_eq!(IntegrationTime::Ms6_25.us(), 6_250);
    }

    #[test]
    fn default_mode_cycle() {
        let mut expectations = vec![
            Transaction::write_read(A, vec![0x02], vec![0x03]),
            Transaction::write(A, vec![0x02, 0x00]),
        ];
        expectations.extend(lux_read(A, 0x48, 0x0A));
        let mut i2c = Mock::new(&expectations);
        let mut als = Max44009::new(&mut i2c, A);
        als.probe().unwrap();
        assert_eq!(als.start_measurement().unwrap(), 880_000);
        assert_eq!(als.start_measurement().unwrap(), 0);
        let m = als.read_measurement().unwrap();
        assert_eq!((m.exponent, m.mantissa), (4, 0x8A));
        assert_eq!(m.illuminance, 99_360);
        assert!(!m.saturated);
        i2c.done();
    }

    #[test]
    fn manual_mode_and_interrupt() {
        let expectations = [
            Transaction::write(A, vec![0x05, 0x48]),
            Transaction::write(A, vec![0x06, 0x38]),
            Transaction::write(A, vec![0x07, 10]),
            Transaction::write(A, vec![0x01, 0x01]),
            Transaction::write(A, vec![0x02, 0xC3]),
            Transaction::write_read(A, vec![0x00], vec![0x01]),
            Transaction::write(A, vec![0x01, 0x00]),
            Transaction::write(A, vec![0x02, 0x00]),
            Transaction::write(A, vec![0x02, 0xC3]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut als = Max44009::new(&mut i2c, A)
            .with_continuous(true)
            .with_manual(Some(IntegrationTime::Ms100));
        als.set_interrupt(Some((50_000, 100_000)), 10).unwrap();
        assert_eq!(als.start_measurement().unwrap(), 110_000);
        assert!(als.take_interrupt().unwrap());
        als.set_interrupt(None, 0).unwrap();
        als.sleep().unwrap();
        assert_eq!(als.start_measurement().unwrap(), 110_000);
        i2c.done();
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let mut expectations = vec![Transaction::write(ADDRESS_A0_HIGH, vec![0x02, 0x80])];
        expectations.extend(lux_read(ADDRESS_A0_HIGH, 0xF0, 0x0F));
        let mut i2c = Mock::new(&expectations);
        let mut als = asynch::Max44009::new(&mut i2c, ADDRESS_A0_HIGH).with_continuous(true);
        let mut delay = RecordingDelay(0);
        let m = block_on(als.measure(&mut delay)).unwrap();
        assert!(m.saturated);
        assert_eq!(m.illuminance, MAX_MILLILUX);
        assert_eq!(delay.0, 880_000);
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
