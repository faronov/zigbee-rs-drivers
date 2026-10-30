//! HC-SR04 / RCWL-1601 (and pin-compatible clones) ultrasonic rangers.
//!
//! Protocol (Elecfreaks "Ultrasonic Ranging Module HC-SR04" datasheet): a
//! ≥ 10 µs high pulse on TRIG makes the module send a 40 kHz burst; ECHO
//! then goes high for the round-trip time of the sound. The datasheet
//! recommends ≥ 60 ms between measurements ([`MIN_CYCLE_US`]).
//!
//! Three ways to use it:
//!
//! * **Timer capture (recommended on a Zigbee node):** call
//!   [`HcSr04::trigger`], capture the ECHO pulse width with a timer
//!   input-capture peripheral, then convert it with [`echo_to_mm`].
//! * **Bit-banged blocking:** the `Sensor` implementation triggers and
//!   times the echo by polling the pin against a microsecond clock.
//! * **Async:** [`asynch::HcSr04`] awaits the echo edges.
//!
//! **Timing exception (driver guide):** this is a µs-timed protocol. The
//! trigger pulse uses `DelayNs` for 10 µs, and the bit-banged measurement
//! busy-polls ECHO for the whole echo pulse (up to the configured timeout,
//! default 30 ms ≈ 5 m). Use the timer-capture path if the radio stack
//! cannot tolerate that.
//!
//! Speed of sound: `c = 331.3 + 0.606·T` m/s (first-order dry-air
//! approximation, e.g. Engineering ToolBox "Speed of Sound in Air"); the
//! module datasheets assume a fixed 340 m/s ("µs / 58 = cm").

#![no_std]

use embedded_hal::digital::{InputPin, OutputPin};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Width of the trigger pulse in µs (datasheet: at least 10 µs).
pub const TRIGGER_PULSE_US: u32 = 10;
/// Minimum time between two measurements (datasheet recommendation).
pub const MIN_CYCLE_US: WaitUs = 60_000;
/// Default echo timeout: 30 ms ≈ 5.1 m at 20 °C, beyond the 4 m range.
pub const DEFAULT_TIMEOUT_US: u32 = 30_000;
/// Echo widths above this are rejected (overflow guard, ≈ 170 m).
pub const MAX_ECHO_US: u32 = 1_000_000;

/// Free-running microsecond clock (wrapping `u32`).
///
/// Implemented for any `FnMut() -> u32`, so a HAL timer can be passed as
/// `|| timer.now_us()`.
pub trait MicrosClock {
    /// Current time in µs.
    fn now_us(&mut self) -> u32;
}

impl<F: FnMut() -> u32> MicrosClock for F {
    fn now_us(&mut self) -> u32 {
        self()
    }
}

/// Speed of sound in 0.1 m/s for an air temperature in 0.01 °C.
pub fn speed_of_sound_dm_s(temperature: i32) -> u32 {
    // 331.3 m/s + 0.606 m/s/°C; in 0.1 m/s and 0.01 °C: 3313 + 606·T/10000.
    let t = temperature.clamp(-5_000, 10_000);
    (33_130_000 + 606 * t + 5_000) as u32 / 10_000
}

/// Convert an echo pulse width (µs, round trip) to a distance in mm at
/// the given air temperature (0.01 °C). Returns `None` above
/// [`MAX_ECHO_US`].
pub fn echo_to_mm(echo_us: u32, temperature: i32) -> Option<u32> {
    if echo_us > MAX_ECHO_US {
        return None;
    }
    // mm = echo[µs] × c[0.1 m/s] × 1e-4 / 2.
    Some((echo_us * speed_of_sound_dm_s(temperature) + 10_000) / 20_000)
}

/// One distance sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Distance in mm.
    pub distance: u32,
    /// Raw echo width in µs.
    pub echo_us: u32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Distance(self.distance));
    }
}

macro_rules! common {
    () => {
        /// Create a driver; no pin access. Temperature defaults to 20 °C.
        pub const fn new(trigger: TRIG, echo: ECHO, clock: CLK, delay: D) -> Self {
            Self {
                trigger,
                echo,
                clock,
                delay,
                temperature: 2_000,
                timeout_us: DEFAULT_TIMEOUT_US,
                last: None,
            }
        }

        /// Air temperature in 0.01 °C used for the speed of sound.
        pub const fn with_temperature(mut self, temperature: i32) -> Self {
            self.temperature = temperature;
            self
        }

        /// Update the air temperature (e.g. from a temperature sensor).
        pub fn set_temperature(&mut self, temperature: i32) {
            self.temperature = temperature;
        }

        /// Echo timeout in µs (bit-banged path only).
        pub const fn with_timeout_us(mut self, timeout_us: u32) -> Self {
            self.timeout_us = timeout_us;
            self
        }

        /// Give the pins, clock and delay back.
        pub fn release(self) -> (TRIG, ECHO, CLK, D) {
            (self.trigger, self.echo, self.clock, self.delay)
        }

        fn finish<E>(&mut self, echo_us: u32) -> Result<Measurement, Error<E>> {
            let distance = echo_to_mm(echo_us, self.temperature).ok_or(Error::InvalidData)?;
            let m = Measurement { distance, echo_us };
            self.last = Some(m);
            Ok(m)
        }
    };
}

/// Blocking driver (bit-banged echo timing).
pub struct HcSr04<TRIG, ECHO, CLK, D> {
    trigger: TRIG,
    echo: ECHO,
    clock: CLK,
    delay: D,
    temperature: i32,
    timeout_us: u32,
    last: Option<Measurement>,
}

impl<TRIG, ECHO, CLK, D> HcSr04<TRIG, ECHO, CLK, D> {
    common!();

    /// Convert an echo width captured by a timer to a measurement using
    /// the configured temperature; it becomes the next `read_measurement`.
    pub fn feed_echo<E>(&mut self, echo_us: u32) -> Result<Measurement, Error<E>> {
        self.finish(echo_us)
    }
}

impl<TRIG, ECHO, CLK, D, E> HcSr04<TRIG, ECHO, CLK, D>
where
    TRIG: OutputPin<Error = E>,
    ECHO: InputPin<Error = E>,
    CLK: MicrosClock,
    D: embedded_hal::delay::DelayNs,
{
    /// Send the 10 µs trigger pulse only (timer-capture use).
    pub fn trigger(&mut self) -> Result<(), Error<E>> {
        self.trigger.set_high().map_err(Error::bus)?;
        self.delay.delay_us(TRIGGER_PULSE_US);
        self.trigger.set_low().map_err(Error::bus)
    }

    fn wait_echo(&mut self, high: bool, start: u32) -> Result<u32, Error<E>> {
        loop {
            let now = self.clock.now_us();
            if self.echo.is_high().map_err(Error::bus)? == high {
                return Ok(now);
            }
            if now.wrapping_sub(start) > self.timeout_us {
                return Err(Error::Timeout);
            }
        }
    }

    /// Trigger and time the echo by polling (blocks up to twice the
    /// timeout). Returns [`Error::Timeout`] if ECHO never rises or stays
    /// high longer than the timeout (no target in range).
    pub fn measure_blocking(&mut self) -> Result<Measurement, Error<E>> {
        self.last = None;
        self.trigger()?;
        let start = self.clock.now_us();
        let rise = self.wait_echo(true, start)?;
        let fall = self.wait_echo(false, rise)?;
        self.finish(fall.wrapping_sub(rise))
    }
}

impl<TRIG, ECHO, CLK, D, E> zigbee_sensor::blocking::Sensor for HcSr04<TRIG, ECHO, CLK, D>
where
    TRIG: OutputPin<Error = E>,
    ECHO: InputPin<Error = E>,
    CLK: MicrosClock,
    D: embedded_hal::delay::DelayNs,
{
    type BusError = E;
    type Measurement = Measurement;

    /// Runs the whole bit-banged measurement (see the timing exception in
    /// the crate docs) and returns `0`.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<E>> {
        self.measure_blocking().map(|_| 0)
    }

    /// Last result of `start_measurement` / `feed_echo`, or `NotReady`.
    fn read_measurement(&mut self) -> Result<Measurement, Error<E>> {
        self.last.take().ok_or(Error::NotReady)
    }
}

/// Async driver: awaits the ECHO edges instead of polling.
pub mod asynch {
    use super::{DEFAULT_TIMEOUT_US, Measurement, MicrosClock, TRIGGER_PULSE_US, echo_to_mm};
    use embedded_hal::digital::{InputPin, OutputPin};
    use embedded_hal_async::{delay::DelayNs, digital::Wait};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::HcSr04`].
    ///
    /// The timeout is not applied while awaiting edges (no timer is
    /// available to the driver); wrap the call in your executor's timeout
    /// (e.g. `embassy_time::with_timeout`). The HC-SR04 ends its echo pulse
    /// on its own when no target is found.
    pub struct HcSr04<TRIG, ECHO, CLK, D> {
        trigger: TRIG,
        echo: ECHO,
        clock: CLK,
        delay: D,
        temperature: i32,
        timeout_us: u32,
        last: Option<Measurement>,
    }

    impl<TRIG, ECHO, CLK, D> HcSr04<TRIG, ECHO, CLK, D> {
        common!();

        /// See [`super::HcSr04::feed_echo`].
        pub fn feed_echo<E>(&mut self, echo_us: u32) -> Result<Measurement, Error<E>> {
            self.finish(echo_us)
        }
    }

    impl<TRIG, ECHO, CLK, D, E> HcSr04<TRIG, ECHO, CLK, D>
    where
        TRIG: OutputPin<Error = E>,
        ECHO: InputPin<Error = E> + Wait,
        CLK: MicrosClock,
        D: DelayNs,
    {
        /// See [`super::HcSr04::trigger`].
        pub async fn trigger(&mut self) -> Result<(), Error<E>> {
            self.trigger.set_high().map_err(Error::bus)?;
            self.delay.delay_us(TRIGGER_PULSE_US).await;
            self.trigger.set_low().map_err(Error::bus)
        }

        /// Trigger, await the echo edges and convert. An echo longer than
        /// the timeout is reported as [`Error::Timeout`].
        pub async fn measure(&mut self) -> Result<Measurement, Error<E>> {
            self.last = None;
            self.trigger().await?;
            self.echo.wait_for_high().await.map_err(Error::bus)?;
            let rise = self.clock.now_us();
            self.echo.wait_for_low().await.map_err(Error::bus)?;
            let width = self.clock.now_us().wrapping_sub(rise);
            if width > self.timeout_us {
                return Err(Error::Timeout);
            }
            self.finish(width)
        }
    }

    impl<TRIG, ECHO, CLK, D, E> zigbee_sensor::asynch::Sensor for HcSr04<TRIG, ECHO, CLK, D>
    where
        TRIG: OutputPin<Error = E>,
        ECHO: InputPin<Error = E> + Wait,
        CLK: MicrosClock,
        D: DelayNs,
    {
        type BusError = E;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<E>> {
            self.measure().await.map(|_| 0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<E>> {
            self.last.take().ok_or(Error::NotReady)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::digital::{Mock, State, Transaction};
    use std::{cell::Cell, rc::Rc};
    use zigbee_sensor::blocking::Sensor;

    #[test]
    fn speed_of_sound() {
        assert_eq!(speed_of_sound_dm_s(0), 3_313);
        assert_eq!(speed_of_sound_dm_s(2_000), 3_434);
        assert_eq!(speed_of_sound_dm_s(-1_000), 3_252);
    }

    #[test]
    fn echo_conversion() {
        // Datasheet rule of thumb at 340 m/s: 58 µs per cm → 580 µs ≈ 100 mm
        // (99.6 mm at 20 °C).
        assert_eq!(echo_to_mm(580, 2_000), Some(100));
        assert_eq!(echo_to_mm(5_831, 2_000), Some(1_001));
        // Same echo is shorter distance in cold air.
        assert_eq!(echo_to_mm(5_831, 0), Some(966));
        assert_eq!(echo_to_mm(MAX_ECHO_US + 1, 0), None);
    }

    fn fake_clock(step: u32) -> impl FnMut() -> u32 {
        let t = Rc::new(Cell::new(0u32));
        move || {
            let now = t.get();
            t.set(now + step);
            now
        }
    }

    #[test]
    fn bit_banged_measurement() {
        let trig_exp = [Transaction::set(State::High), Transaction::set(State::Low)];
        // Echo: low, low, high (rise), high, high, low (fall).
        let echo_exp = [
            Transaction::get(State::Low),
            Transaction::get(State::Low),
            Transaction::get(State::High),
            Transaction::get(State::High),
            Transaction::get(State::High),
            Transaction::get(State::Low),
        ];
        let mut trig = Mock::new(&trig_exp);
        let mut echo = Mock::new(&echo_exp);
        // Clock read once before the loop (t=0), then once per poll.
        let mut hc = HcSr04::new(
            trig.clone(),
            echo.clone(),
            fake_clock(1_000),
            NoopDelay::new(),
        );
        assert_eq!(hc.read_measurement(), Err(Error::NotReady));
        assert_eq!(hc.start_measurement().unwrap(), 0);
        let m = hc.read_measurement().unwrap();
        // rise sampled at t=3000, fall at t=6000.
        assert_eq!(m.echo_us, 3_000);
        assert_eq!(m.distance, 515);
        trig.done();
        echo.done();
    }

    #[test]
    fn no_echo_times_out() {
        let trig_exp = [Transaction::set(State::High), Transaction::set(State::Low)];
        let echo_exp: std::vec::Vec<_> = (0..4).map(|_| Transaction::get(State::Low)).collect();
        let mut trig = Mock::new(&trig_exp);
        let mut echo = Mock::new(&echo_exp);
        let mut hc = HcSr04::new(
            trig.clone(),
            echo.clone(),
            fake_clock(10_000),
            NoopDelay::new(),
        )
        .with_timeout_us(35_000);
        assert_eq!(hc.measure_blocking(), Err(Error::Timeout));
        trig.done();
        echo.done();
    }

    #[test]
    fn timer_capture_feed() {
        let mut hc = HcSr04::new((), (), fake_clock(1), ()).with_temperature(0);
        let m = hc.feed_echo::<()>(5_831).unwrap();
        assert_eq!(m.distance, 966);
    }

    #[test]
    fn async_measure() {
        let trig_exp = [Transaction::set(State::High), Transaction::set(State::Low)];
        let echo_exp = [
            Transaction::wait_for_state(State::High),
            Transaction::wait_for_state(State::Low),
        ];
        let mut trig = Mock::new(&trig_exp);
        let mut echo = Mock::new(&echo_exp);
        let mut hc = asynch::HcSr04::new(
            trig.clone(),
            echo.clone(),
            fake_clock(5_831),
            NoopDelay::new(),
        );
        let m = block_on(hc.measure()).unwrap();
        assert_eq!(m.distance, 1_001);
        trig.done();
        echo.done();
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
