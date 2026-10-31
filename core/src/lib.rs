//! Common API shared by every zigbee-rs sensor driver.
//!
//! A Zigbee node must never block its stack while a sensor converts. Every
//! driver therefore splits a measurement into two bus operations:
//!
//! 1. [`blocking::Sensor::start_measurement`] (or the async twin) triggers a
//!    conversion and returns how long the caller must wait, in microseconds.
//! 2. [`blocking::Sensor::read_measurement`] fetches and decodes the result.
//!    Multi-phase parts (temperature first, then pressure, ...) return
//!    [`Error::Pending`] with the wait for the phase they just started; the
//!    caller waits and calls `read_measurement` again.
//!
//! The wait in between belongs to the application: a timer in an event loop,
//! a `Timer::after(..).await` in an async executor, or a sleep period on a
//! sleepy end device. Drivers never call a delay during a measurement; the
//! only permitted delays are short, documented power-up/reset waits that
//! cannot be avoided.
//!
//! Measurements are reported in integer engineering units ([`Quantity`]) and
//! [`zcl`] converts them to the encodings of the matching ZCL attributes.

#![no_std]

pub mod adc;
pub mod crc;
mod quantity;
pub mod zcl;

pub use quantity::{Quantity, Readings, Vector3};

/// Error type shared by all drivers, generic over the bus error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// The I2C/SPI/UART/GPIO/ADC transport failed.
    Bus(E),
    /// A checksum (CRC, frame sum) did not match.
    Crc,
    /// The conversion has not finished yet; read again later.
    NotReady,
    /// One phase of a multi-phase measurement finished and the driver has
    /// started the next one. Wait the given time, then call
    /// `read_measurement` again. Not a failure.
    Pending(WaitUs),
    /// The chip/part ID read from the device is not the expected one.
    UnexpectedId(u32),
    /// The device returned data that cannot be decoded (bad frame, out of
    /// range raw value, sensor-reported fault).
    InvalidData,
    /// The device did not respond within the protocol time limit.
    Timeout,
    /// The requested configuration is not supported by this part.
    Unsupported,
    /// A caller-supplied argument is out of the range the part accepts.
    InvalidArgument,
}

impl<E> Error<E> {
    /// Map a transport error. Usable as `.map_err(Error::bus)`.
    pub fn bus(error: E) -> Self {
        Self::Bus(error)
    }
}

/// Microseconds the caller has to wait between starting a conversion and
/// reading it back.
pub type WaitUs = u32;

/// Upper bound on [`Error::Pending`] phases the `measure` helpers follow
/// before giving up with [`Error::Timeout`].
pub const MAX_PHASES: usize = 8;

/// Blocking (embedded-hal 1.0) driver traits.
pub mod blocking {
    use super::{Error, MAX_PHASES, Readings, WaitUs};

    /// A sensor that produces measurements on request.
    pub trait Sensor {
        /// Transport error type.
        type BusError;
        /// Decoded measurement.
        type Measurement: Readings;

        /// Trigger a conversion. Returns the time to wait before
        /// [`Self::read_measurement`]. Free-running sensors return `0`.
        fn start_measurement(&mut self) -> Result<WaitUs, Error<Self::BusError>>;

        /// Read the result of the last conversion. Returns
        /// [`Error::NotReady`] if the device reports that data is not ready,
        /// or [`Error::Pending`] when it started a further conversion phase.
        fn read_measurement(&mut self) -> Result<Self::Measurement, Error<Self::BusError>>;
    }

    /// A sensor with an explicit low-power state.
    pub trait LowPower {
        /// Transport error type.
        type BusError;

        /// Put the device into its lowest-power state that keeps its
        /// configuration.
        fn sleep(&mut self) -> Result<(), Error<Self::BusError>>;

        /// Leave the low-power state. Returns the time to wait before the
        /// device accepts commands again.
        fn wake(&mut self) -> Result<WaitUs, Error<Self::BusError>>;
    }

    /// Start, wait and read in one call.
    ///
    /// This blocks for the whole conversion time. It is intended for tests,
    /// tools and bring-up, not for a Zigbee event loop.
    pub fn measure<S, D>(
        sensor: &mut S,
        delay: &mut D,
    ) -> Result<S::Measurement, Error<S::BusError>>
    where
        S: Sensor,
        D: embedded_hal::delay::DelayNs,
    {
        let mut wait = sensor.start_measurement()?;
        for _ in 0..MAX_PHASES {
            if wait != 0 {
                delay.delay_us(wait);
            }
            match sensor.read_measurement() {
                Err(Error::Pending(next)) => wait = next,
                other => return other,
            }
        }
        Err(Error::Timeout)
    }
}

/// Async (embedded-hal-async 1.0) driver traits.
///
/// Named `asynch` because `async` is a keyword.
pub mod asynch {
    use super::{Error, MAX_PHASES, Readings, WaitUs};

    /// Async twin of [`crate::blocking::Sensor`].
    #[allow(async_fn_in_trait)]
    pub trait Sensor {
        /// Transport error type.
        type BusError;
        /// Decoded measurement.
        type Measurement: Readings;

        /// Trigger a conversion; see [`crate::blocking::Sensor::start_measurement`].
        async fn start_measurement(&mut self) -> Result<WaitUs, Error<Self::BusError>>;

        /// Read the last conversion; see [`crate::blocking::Sensor::read_measurement`].
        async fn read_measurement(&mut self) -> Result<Self::Measurement, Error<Self::BusError>>;

        /// Start, await the conversion time on `delay`, then read. The
        /// executor keeps running the Zigbee stack while this waits.
        async fn measure<D>(
            &mut self,
            delay: &mut D,
        ) -> Result<Self::Measurement, Error<Self::BusError>>
        where
            D: embedded_hal_async::delay::DelayNs,
        {
            let mut wait = self.start_measurement().await?;
            for _ in 0..MAX_PHASES {
                if wait != 0 {
                    delay.delay_us(wait).await;
                }
                match self.read_measurement().await {
                    Err(Error::Pending(next)) => wait = next,
                    other => return other,
                }
            }
            Err(Error::Timeout)
        }
    }

    /// Async twin of [`crate::blocking::LowPower`].
    #[allow(async_fn_in_trait)]
    pub trait LowPower {
        /// Transport error type.
        type BusError;

        /// See [`crate::blocking::LowPower::sleep`].
        async fn sleep(&mut self) -> Result<(), Error<Self::BusError>>;

        /// See [`crate::blocking::LowPower::wake`].
        async fn wake(&mut self) -> Result<WaitUs, Error<Self::BusError>>;
    }
}

#[cfg(test)]
mod tests {
    use super::{Error, MAX_PHASES, Quantity, Readings, WaitUs, blocking};

    struct Value(i32);
    impl Readings for Value {
        fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
            f(Quantity::Temperature(self.0));
        }
    }

    /// Returns `Pending(100)` `phases` times, then a value.
    struct TwoPhase {
        phases: usize,
    }
    impl blocking::Sensor for TwoPhase {
        type BusError = ();
        type Measurement = Value;
        fn start_measurement(&mut self) -> Result<WaitUs, Error<()>> {
            Ok(50)
        }
        fn read_measurement(&mut self) -> Result<Value, Error<()>> {
            if self.phases == 0 {
                return Ok(Value(42));
            }
            self.phases -= 1;
            Err(Error::Pending(100))
        }
    }

    #[derive(Default)]
    struct CountDelay(u64);
    impl embedded_hal::delay::DelayNs for CountDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.0 += u64::from(ns);
        }
    }

    #[test]
    fn measure_follows_pending_phases() {
        let mut delay = CountDelay::default();
        let value = blocking::measure(&mut TwoPhase { phases: 1 }, &mut delay).unwrap();
        assert_eq!(value.0, 42);
        assert_eq!(delay.0, 150_000);
    }

    #[test]
    fn measure_gives_up_after_max_phases() {
        let mut delay = CountDelay::default();
        let result = blocking::measure(&mut TwoPhase { phases: MAX_PHASES }, &mut delay);
        assert!(matches!(result, Err(Error::Timeout)));
    }
}
