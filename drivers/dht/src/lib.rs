//! Aosong DHT11 and DHT22 / AM2302 temperature and humidity sensors
//! (single-wire, bit-banged on one GPIO).
//!
//! The data line is an open-drain GPIO with a pull-up (4.7–10 kΩ); the pin
//! type must implement both `InputPin` and `OutputPin` (`set_high`
//! releases the line).
//!
//! * [`Dht::start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//!   pulls the line low and returns how long it must stay low (18 ms for
//!   DHT11, 1.1 ms for DHT22). The caller sleeps or awaits that time.
//! * [`Dht::read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//!   releases the line and bit-bangs the 40-bit answer (~5 ms). This is the
//!   one place that uses the `DelayNs` for µs polling: the caller must mask
//!   interrupts (or at least keep the radio from preempting) for the
//!   duration, otherwise bits are lost and [`Error::Crc`] / [`Error::Timeout`]
//!   is returned.
//! * Platforms with input capture / RMT can measure the 40 data-bit high
//!   times themselves and use [`decode_pulses`]; [`decode_frame`] decodes
//!   the 5 raw bytes.
//!
//! Wait at least 1 s after power-up and 2 s between reads; a faster rate
//! returns stale data or no response.
//!
//! The async twin awaits only the start pulse; the bit timing still uses a
//! blocking `embedded_hal::delay::DelayNs`, since async timers cannot
//! resolve microsecond pulses.
//!
//! References: Aosong "DHT11 Humidity & Temperature Sensor" manual,
//! Aosong "AM2302 / DHT22 Digital-output relative humidity & temperature
//! sensor/module" manual.
//!
//! ```ignore
//! let mut dht = Dht::new(pin, delay, Variant::Dht22);
//! let wait_us = dht.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = critical_section::with(|_| dht.read_measurement())?;
//! ```

#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Wait after power-up before the first start pulse.
pub const POWER_UP_US: WaitUs = 1_000_000;
/// Minimum interval between two reads.
pub const MIN_INTERVAL_US: WaitUs = 2_000_000;
/// High time (µs) above which a data bit is `1` (`0` ≈ 26 µs, `1` ≈ 70 µs).
pub const BIT_THRESHOLD_US: u16 = 48;

/// Polling ticks (≥ 1 µs each) before a level change is declared missing.
/// The longest expected level is the 200 µs maximum response delay.
const TIMEOUT_TICKS: u32 = 500;

/// Supported parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// DHT11: integer + tenths bytes, 0–50 °C (newer parts −20–60 °C).
    Dht11,
    /// DHT22 / AM2302 / AM2303: 16-bit 0.1 units, sign-magnitude temperature.
    Dht22,
}

impl Variant {
    /// Time the host keeps the line low to start a measurement.
    pub const fn start_low_us(self) -> WaitUs {
        match self {
            Self::Dht11 => 18_000,
            Self::Dht22 => 1_100,
        }
    }
}

/// One temperature + humidity sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Relative humidity in 0.01 %RH.
    pub humidity: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
        f(Quantity::Humidity(self.humidity));
    }
}

/// Whether the fifth byte is the low 8 bits of the sum of the first four.
pub fn checksum_ok(frame: &[u8; 5]) -> bool {
    frame[..4].iter().fold(0u8, |sum, b| sum.wrapping_add(*b)) == frame[4]
}

/// Decode the 5 received bytes (checksum included).
///
/// DHT11: `RH int, RH tenths, T int, T tenths (bit 7 = negative), sum`.
/// DHT22: `RH × 10 (u16), T × 10 (bit 15 = negative), sum`.
/// Humidity above 100 % returns [`Error::InvalidData`].
pub fn decode_frame<E>(variant: Variant, frame: &[u8; 5]) -> Result<Measurement, Error<E>> {
    if !checksum_ok(frame) {
        return Err(Error::Crc);
    }
    let (humidity, magnitude, negative) = match variant {
        Variant::Dht11 => (
            u32::from(frame[0]) * 100 + u32::from(frame[1]) * 10,
            i32::from(frame[2]) * 100 + i32::from(frame[3] & 0x7F) * 10,
            frame[3] & 0x80 != 0,
        ),
        Variant::Dht22 => (
            u32::from(u16::from_be_bytes([frame[0], frame[1]])) * 10,
            i32::from(u16::from_be_bytes([frame[2] & 0x7F, frame[3]])) * 10,
            frame[2] & 0x80 != 0,
        ),
    };
    if humidity > 10_000 {
        return Err(Error::InvalidData);
    }
    Ok(Measurement {
        temperature: if negative { -magnitude } else { magnitude },
        humidity: humidity as u16,
    })
}

/// Decode the high times (µs) of the 40 data bits, MSB first, e.g. from an
/// input-capture or RMT peripheral.
pub fn decode_pulses<E>(variant: Variant, high_us: &[u16; 40]) -> Result<Measurement, Error<E>> {
    let mut frame = [0u8; 5];
    for (i, &t) in high_us.iter().enumerate() {
        if t > BIT_THRESHOLD_US {
            frame[i / 8] |= 0x80 >> (i % 8);
        }
    }
    decode_frame(variant, &frame)
}

/// Count polling ticks while the line stays at `level`.
fn pulse<P: InputPin, D: DelayNs>(
    pin: &mut P,
    delay: &mut D,
    level: bool,
) -> Result<u32, Error<P::Error>> {
    let mut ticks = 0;
    while pin.is_high().map_err(Error::bus)? == level {
        if ticks >= TIMEOUT_TICKS {
            return Err(Error::Timeout);
        }
        delay.delay_us(1);
        ticks += 1;
    }
    Ok(ticks)
}

/// Release the line after the start pulse and receive the 40-bit frame.
///
/// Each bit is a ~50 µs low followed by a 26 µs (`0`) or 70 µs (`1`) high;
/// comparing the two counts makes the decision independent of the actual
/// polling period.
pub fn read_frame<P, D>(pin: &mut P, delay: &mut D) -> Result<[u8; 5], Error<P::Error>>
where
    P: InputPin + OutputPin,
    D: DelayNs,
{
    pin.set_high().map_err(Error::bus)?;
    // Host release (20–200 µs), then the 80 µs low / 80 µs high response.
    pulse(pin, delay, true)?;
    pulse(pin, delay, false)?;
    pulse(pin, delay, true)?;
    let mut frame = [0u8; 5];
    for i in 0..40 {
        let low = pulse(pin, delay, false)?;
        let high = pulse(pin, delay, true)?;
        if high > low {
            frame[i / 8] |= 0x80 >> (i % 8);
        }
    }
    Ok(frame)
}

macro_rules! constructors {
    () => {
        /// Create a driver. The pin must be configured as open-drain output
        /// with the line released (high); no pin activity here.
        pub const fn new(pin: P, delay: D, variant: Variant) -> Self {
            Self {
                pin,
                delay,
                variant,
            }
        }

        /// Give the pin and delay back.
        pub fn release(self) -> (P, D) {
            (self.pin, self.delay)
        }
    };
}

/// Blocking driver.
pub struct Dht<P, D> {
    pin: P,
    delay: D,
    variant: Variant,
}

impl<P, D> Dht<P, D> {
    constructors!();
}

impl<P: InputPin + OutputPin, D: DelayNs> zigbee_sensor::blocking::Sensor for Dht<P, D> {
    type BusError = P::Error;
    type Measurement = Measurement;

    /// Pull the line low; keep it low for the returned time.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<P::Error>> {
        self.pin.set_low().map_err(Error::bus)?;
        Ok(self.variant.start_low_us())
    }

    /// Release the line and receive the answer (~5 ms, timing critical).
    fn read_measurement(&mut self) -> Result<Measurement, Error<P::Error>> {
        let frame = read_frame(&mut self.pin, &mut self.delay)?;
        decode_frame(self.variant, &frame)
    }
}

/// Async driver.
pub mod asynch {
    use super::{Measurement, Variant, decode_frame, read_frame};
    use embedded_hal::delay::DelayNs;
    use embedded_hal::digital::{InputPin, OutputPin};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Dht`]. `D` is a *blocking* delay used for the
    /// µs bit timing; the start pulse is awaited by
    /// [`measure`](zigbee_sensor::asynch::Sensor::measure).
    pub struct Dht<P, D> {
        pin: P,
        delay: D,
        variant: Variant,
    }

    impl<P, D> Dht<P, D> {
        constructors!();
    }

    impl<P: InputPin + OutputPin, D: DelayNs> zigbee_sensor::asynch::Sensor for Dht<P, D> {
        type BusError = P::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<P::Error>> {
            self.pin.set_low().map_err(Error::bus)?;
            Ok(self.variant.start_low_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<P::Error>> {
            let frame = read_frame(&mut self.pin, &mut self.delay)?;
            decode_frame(self.variant, &frame)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use core::cell::{Cell, RefCell};
    use core::convert::Infallible;
    use std::rc::Rc;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    /// Simulated line: levels as `(high, µs)` segments starting when the
    /// host releases the line; time advances only through `FakeDelay`.
    #[derive(Default)]
    struct Line {
        now: Cell<u32>,
        released_at: Cell<Option<u32>>,
        waveform: RefCell<Vec<(bool, u32)>>,
        set_low_calls: Cell<u32>,
    }

    impl Line {
        fn level(&self) -> bool {
            let Some(start) = self.released_at.get() else {
                return false;
            };
            let mut t = self.now.get() - start;
            for &(level, len) in self.waveform.borrow().iter() {
                if t < len {
                    return level;
                }
                t -= len;
            }
            true
        }
    }

    struct FakePin(Rc<Line>);
    struct FakeDelay(Rc<Line>);

    impl embedded_hal::digital::ErrorType for FakePin {
        type Error = Infallible;
    }
    impl InputPin for FakePin {
        fn is_high(&mut self) -> Result<bool, Infallible> {
            Ok(self.0.level())
        }
        fn is_low(&mut self) -> Result<bool, Infallible> {
            Ok(!self.0.level())
        }
    }
    impl OutputPin for FakePin {
        fn set_low(&mut self) -> Result<(), Infallible> {
            self.0.set_low_calls.set(self.0.set_low_calls.get() + 1);
            self.0.released_at.set(None);
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Infallible> {
            self.0.released_at.set(Some(self.0.now.get()));
            Ok(())
        }
    }
    impl DelayNs for FakeDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.0.now.set(self.0.now.get() + ns.div_ceil(1_000));
        }
    }

    /// Sensor answer per the datasheet timing diagrams.
    fn answer(frame: [u8; 5]) -> Vec<(bool, u32)> {
        let mut w = std::vec![(true, 30), (false, 80), (true, 80)];
        for i in 0..40 {
            let one = frame[i / 8] & (0x80 >> (i % 8)) != 0;
            w.push((false, 50));
            w.push((true, if one { 70 } else { 26 }));
        }
        w.push((false, 50));
        w
    }

    fn setup(waveform: Vec<(bool, u32)>) -> (Rc<Line>, FakePin, FakeDelay) {
        let line = Rc::new(Line::default());
        *line.waveform.borrow_mut() = waveform;
        (line.clone(), FakePin(line.clone()), FakeDelay(line))
    }

    // AM2302 manual example: 65.2 %RH, 35.1 °C, checksum 0xEE.
    const DHT22_FRAME: [u8; 5] = [0x02, 0x8C, 0x01, 0x5F, 0xEE];

    #[test]
    fn decode_datasheet_examples() {
        assert_eq!(
            decode_frame::<()>(Variant::Dht22, &DHT22_FRAME),
            Ok(Measurement {
                temperature: 3_510,
                humidity: 6_520
            })
        );
        // AM2302 manual: 1000 0000 0110 0101 = −10.1 °C.
        let negative = [0x02, 0x8C, 0x80, 0x65, 0x73];
        assert_eq!(
            decode_frame::<()>(Variant::Dht22, &negative).map(|m| m.temperature),
            Ok(-1_010)
        );
        // DHT11 manual: 0x21 0x00 0x1A 0x00 0x3B = 33 %RH, 26 °C.
        assert_eq!(
            decode_frame::<()>(Variant::Dht11, &[0x21, 0x00, 0x1A, 0x00, 0x3B]),
            Ok(Measurement {
                temperature: 2_600,
                humidity: 3_300
            })
        );
        // Newer DHT11: tenths byte bit 7 = negative → −16.1 °C.
        assert_eq!(
            decode_frame::<()>(Variant::Dht11, &[0x21, 0x00, 0x10, 0x81, 0xB2])
                .map(|m| m.temperature),
            Ok(-1_610)
        );
    }

    #[test]
    fn checksum_and_range_errors() {
        let mut bad = DHT22_FRAME;
        bad[4] ^= 1;
        assert!(!checksum_ok(&bad));
        assert_eq!(decode_frame::<()>(Variant::Dht22, &bad), Err(Error::Crc));
        // 0x03E9 = 100.1 %RH.
        let over = [0x03, 0xE9, 0x00, 0x00, 0xEC];
        assert_eq!(
            decode_frame::<()>(Variant::Dht22, &over),
            Err(Error::InvalidData)
        );
    }

    #[test]
    fn decode_from_pulse_widths() {
        let mut highs = [26u16; 40];
        for (i, h) in highs.iter_mut().enumerate() {
            if DHT22_FRAME[i / 8] & (0x80 >> (i % 8)) != 0 {
                *h = 70;
            }
        }
        assert_eq!(
            decode_pulses::<()>(Variant::Dht22, &highs).map(|m| m.humidity),
            Ok(6_520)
        );
    }

    #[test]
    fn bit_banged_read() {
        let (line, pin, delay) = setup(answer(DHT22_FRAME));
        let mut dht = Dht::new(pin, delay, Variant::Dht22);
        assert_eq!(dht.start_measurement(), Ok(1_100));
        assert_eq!(line.set_low_calls.get(), 1);
        assert_eq!(
            dht.read_measurement(),
            Ok(Measurement {
                temperature: 3_510,
                humidity: 6_520
            })
        );
        // The whole answer takes under 5 ms.
        assert!(line.now.get() < 5_000);
    }

    #[test]
    fn no_response_times_out() {
        let (_line, pin, delay) = setup(std::vec![(true, 10_000)]);
        let mut dht = Dht::new(pin, delay, Variant::Dht11);
        assert_eq!(dht.start_measurement(), Ok(18_000));
        assert_eq!(dht.read_measurement(), Err(Error::Timeout));
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

        let (_line, pin, delay) = setup(answer([0x21, 0x00, 0x1A, 0x00, 0x3B]));
        let mut dht = asynch::Dht::new(pin, delay, Variant::Dht11);
        let mut wait = RecordingDelay(0);
        let m = block_on(dht.measure(&mut wait)).unwrap();
        assert_eq!(m.temperature, 2_600);
        assert_eq!(wait.0, 18_000);
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
