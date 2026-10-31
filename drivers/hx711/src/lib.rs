//! Avia Semiconductor HX711 24-bit ADC for load cells / weigh scales.
//!
//! Two-wire bit-banged interface: PD_SCK (output) and DOUT (input).
//! Reference: Avia Semiconductor "HX711 24-Bit Analog-to-Digital Converter
//! (ADC) for Weigh Scales" datasheet (no revision printed).
//!
//! * DOUT low means a conversion is ready; while it is high
//!   `read_measurement` returns [`Error::NotReady`]. The part converts
//!   continuously at 10 SPS (RATE = 0) or 80 SPS (RATE = 1), so
//!   `start_measurement` returns `0`.
//! * A read clocks 24 data bits (MSB first, two's complement) plus 1, 2 or
//!   3 extra pulses that select channel/gain of the **next** conversion
//!   ([`Gain`]).
//! * PD_SCK held high for more than 60 µs powers the chip down
//!   ([`LowPower::sleep`](zigbee_sensor::blocking::LowPower::sleep)). After
//!   power-up, reset, or a channel/gain change the output needs 400 ms
//!   (10 SPS) / 50 ms (80 SPS) to settle; the driver discards the first
//!   conversion after such a change and returns the settling time.
//!
//! **Timing exception (driver guide):** a read takes 25–27 PD_SCK pulses
//! of 1 µs high / 1 µs low using `DelayNs` (≈ 55 µs total). PD_SCK must
//! not stay high longer than 50 µs, so an interrupt that preempts the read
//! for > 50 µs can power the chip down and corrupt the sample; run
//! `read_measurement` with interrupts masked or at high priority if that
//! is a risk. The async twin awaits DOUT going low but performs the bit
//! transfer with a blocking `DelayNs` for the same reason.
//!
//! Mass conversion is integer: `mg = (raw − offset) × scale_q16 / 2¹⁶`
//! ([`Calibration`]).

#![no_std]

use embedded_hal::digital::{InputPin, OutputPin};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// PD_SCK high/low time used for each bit (datasheet T3/T4 typ. 1 µs).
pub const BIT_HALF_PERIOD_US: u32 = 1;

/// Input channel and gain selected for the next conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gain {
    /// Channel A, gain 128 (±20 mV full scale at 5 V AVDD). Power-on default.
    #[default]
    A128,
    /// Channel B, gain 32 (±80 mV).
    B32,
    /// Channel A, gain 64 (±40 mV).
    A64,
}

impl Gain {
    /// Total PD_SCK pulses of a read that selects this gain next
    /// (datasheet Table 3: 25, 26 or 27).
    pub const fn pulses(self) -> u8 {
        match self {
            Self::A128 => 25,
            Self::B32 => 26,
            Self::A64 => 27,
        }
    }
}

/// Output data rate, set by the RATE pin (not software controllable).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Rate {
    /// RATE low: 10 SPS, 400 ms settling.
    #[default]
    Sps10,
    /// RATE high: 80 SPS, 50 ms settling.
    Sps80,
}

impl Rate {
    /// Settling time after power-up, reset or channel/gain change.
    pub const fn settling_us(self) -> WaitUs {
        match self {
            Self::Sps10 => 400_000,
            Self::Sps80 => 50_000,
        }
    }

    /// Conversion period.
    pub const fn period_us(self) -> WaitUs {
        match self {
            Self::Sps10 => 100_000,
            Self::Sps80 => 12_500,
        }
    }
}

/// Sign-extend a 24-bit two's complement sample (0x800000 = −8 388 608).
pub const fn sign_extend_24(raw: u32) -> i32 {
    ((raw << 8) as i32) >> 8
}

/// Tare offset and scale. `mg = (raw − offset) × scale_q16 / 65536`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// Raw reading with an empty scale.
    pub offset: i32,
    /// Milligrams per count, Q16.16 fixed point (may be negative if the
    /// load cell is wired inverted).
    pub scale_q16: i32,
}

impl Default for Calibration {
    /// No offset, 1 mg per count.
    fn default() -> Self {
        Self {
            offset: 0,
            scale_q16: 1 << 16,
        }
    }
}

impl Calibration {
    /// Calibration from a tare reading and a reading with a known mass.
    /// Returns `None` if both readings are equal or the scale overflows.
    pub fn from_reference(offset: i32, raw_with_mass: i32, mass_mg: i32) -> Option<Self> {
        let counts = i64::from(raw_with_mass) - i64::from(offset);
        if counts == 0 {
            return None;
        }
        // One-off 64-bit division at calibration time only.
        let scale = (i64::from(mass_mg) << 16) / counts;
        Some(Self {
            offset,
            scale_q16: i32::try_from(scale).ok()?,
        })
    }

    /// Convert a raw sample to milligrams (saturating).
    pub fn mass_mg(&self, raw: i32) -> i32 {
        let counts = i64::from(raw) - i64::from(self.offset);
        let mg = (counts * i64::from(self.scale_q16)) >> 16;
        mg.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
    }
}

/// One conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Signed 24-bit ADC value.
    pub raw: i32,
    /// Calibrated mass in mg.
    pub mass: i32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Mass(self.mass));
    }
}

macro_rules! common {
    () => {
        /// Create a driver; no pin access. Assumes the power-on default
        /// gain (A128) and 10 SPS.
        pub const fn new(sck: SCK, dout: DOUT, delay: D) -> Self {
            Self {
                sck,
                dout,
                delay,
                gain: Gain::A128,
                rate: Rate::Sps10,
                calibration: Calibration {
                    offset: 0,
                    scale_q16: 1 << 16,
                },
                discard_next: false,
            }
        }

        /// Declare the RATE pin setting (affects returned wait times).
        pub const fn with_rate(mut self, rate: Rate) -> Self {
            self.rate = rate;
            self
        }

        /// Set tare offset and scale.
        pub const fn with_calibration(mut self, calibration: Calibration) -> Self {
            self.calibration = calibration;
            self
        }

        /// Replace the calibration.
        pub fn set_calibration(&mut self, calibration: Calibration) {
            self.calibration = calibration;
        }

        /// Current calibration.
        pub const fn calibration(&self) -> Calibration {
            self.calibration
        }

        /// Use `raw` (an empty-scale reading) as the tare offset.
        pub fn tare(&mut self, raw: i32) {
            self.calibration.offset = raw;
        }

        /// Select channel/gain. It takes effect after the next read (the
        /// pulses of that read program it); that read's sample is
        /// discarded. Returns the settling time to wait before reads give
        /// valid data again.
        pub fn set_gain(&mut self, gain: Gain) -> WaitUs {
            if gain != self.gain {
                self.gain = gain;
                self.discard_next = true;
            }
            self.rate.settling_us()
        }

        /// Give the pins and delay back.
        pub fn release(self) -> (SCK, DOUT, D) {
            (self.sck, self.dout, self.delay)
        }

        fn after_power_up(&mut self) -> WaitUs {
            // The chip restarts on A128; a different gain needs one read to
            // reprogram it and that sample is at the wrong gain.
            self.discard_next = self.gain != Gain::A128;
            self.rate.settling_us()
        }
    };
}

macro_rules! bit_bang {
    () => {
        /// DOUT is low: a conversion is ready.
        pub fn is_ready(&mut self) -> Result<bool, Error<E>> {
            self.dout.is_low().map_err(Error::bus)
        }

        /// Clock out one conversion if ready, else [`Error::NotReady`].
        /// Returns the raw signed sample (no discard handling).
        pub fn read_raw(&mut self) -> Result<i32, Error<E>> {
            if !self.is_ready()? {
                return Err(Error::NotReady);
            }
            let mut value = 0u32;
            for pulse in 0..self.gain.pulses() {
                self.sck.set_high().map_err(Error::bus)?;
                self.delay.delay_us(BIT_HALF_PERIOD_US);
                if pulse < 24 {
                    let bit = self.dout.is_high().map_err(Error::bus)?;
                    value = (value << 1) | u32::from(bit);
                }
                self.sck.set_low().map_err(Error::bus)?;
                self.delay.delay_us(BIT_HALF_PERIOD_US);
            }
            Ok(sign_extend_24(value))
        }

        fn read_converted(&mut self) -> Result<Measurement, Error<E>> {
            let raw = self.read_raw()?;
            if self.discard_next {
                self.discard_next = false;
                return Err(Error::NotReady);
            }
            Ok(Measurement {
                raw,
                mass: self.calibration.mass_mg(raw),
            })
        }

        fn power_down(&mut self) -> Result<(), Error<E>> {
            // Held high > 60 µs by the caller's sleep period.
            self.sck.set_high().map_err(Error::bus)
        }

        fn power_up(&mut self) -> Result<WaitUs, Error<E>> {
            self.sck.set_low().map_err(Error::bus)?;
            Ok(self.after_power_up())
        }
    };
}

/// Blocking driver.
pub struct Hx711<SCK, DOUT, D> {
    sck: SCK,
    dout: DOUT,
    delay: D,
    gain: Gain,
    rate: Rate,
    calibration: Calibration,
    discard_next: bool,
}

impl<SCK, DOUT, D> Hx711<SCK, DOUT, D> {
    common!();
}

impl<SCK, DOUT, D, E> Hx711<SCK, DOUT, D>
where
    SCK: OutputPin<Error = E>,
    DOUT: InputPin<Error = E>,
    D: embedded_hal::delay::DelayNs,
{
    bit_bang!();
}

impl<SCK, DOUT, D, E> zigbee_sensor::blocking::Sensor for Hx711<SCK, DOUT, D>
where
    SCK: OutputPin<Error = E>,
    DOUT: InputPin<Error = E>,
    D: embedded_hal::delay::DelayNs,
{
    type BusError = E;
    type Measurement = Measurement;

    /// Free-running: returns `0`.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<E>> {
        Ok(0)
    }

    /// `NotReady` while DOUT is high or when the sample is discarded after
    /// a gain change / wake-up.
    fn read_measurement(&mut self) -> Result<Measurement, Error<E>> {
        self.read_converted()
    }
}

impl<SCK, DOUT, D, E> zigbee_sensor::blocking::LowPower for Hx711<SCK, DOUT, D>
where
    SCK: OutputPin<Error = E>,
    DOUT: InputPin<Error = E>,
    D: embedded_hal::delay::DelayNs,
{
    type BusError = E;

    /// PD_SCK high; the chip powers down after 60 µs (≈ 0.5 µA).
    fn sleep(&mut self) -> Result<(), Error<E>> {
        self.power_down()
    }

    /// PD_SCK low; the chip resets. Returns the settling time.
    fn wake(&mut self) -> Result<WaitUs, Error<E>> {
        self.power_up()
    }
}

/// Async driver: awaits DOUT readiness; the bit transfer stays blocking.
pub mod asynch {
    use super::{BIT_HALF_PERIOD_US, Calibration, Gain, Measurement, Rate, sign_extend_24};
    use embedded_hal::digital::{InputPin, OutputPin};
    use embedded_hal_async::digital::Wait;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Hx711`]. `D` is a **blocking**
    /// `embedded_hal::delay::DelayNs` used for the 1 µs bit timing.
    pub struct Hx711<SCK, DOUT, D> {
        sck: SCK,
        dout: DOUT,
        delay: D,
        gain: Gain,
        rate: Rate,
        calibration: Calibration,
        discard_next: bool,
    }

    impl<SCK, DOUT, D> Hx711<SCK, DOUT, D> {
        common!();
    }

    impl<SCK, DOUT, D, E> Hx711<SCK, DOUT, D>
    where
        SCK: OutputPin<Error = E>,
        DOUT: InputPin<Error = E> + Wait,
        D: embedded_hal::delay::DelayNs,
    {
        bit_bang!();

        /// Wait until DOUT goes low (conversion ready).
        pub async fn wait_ready(&mut self) -> Result<(), Error<E>> {
            self.dout.wait_for_low().await.map_err(Error::bus)
        }
    }

    impl<SCK, DOUT, D, E> zigbee_sensor::asynch::Sensor for Hx711<SCK, DOUT, D>
    where
        SCK: OutputPin<Error = E>,
        DOUT: InputPin<Error = E> + Wait,
        D: embedded_hal::delay::DelayNs,
    {
        type BusError = E;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<E>> {
            Ok(0)
        }

        /// Awaits DOUT low, then reads (`NotReady` only for a discarded
        /// sample).
        async fn read_measurement(&mut self) -> Result<Measurement, Error<E>> {
            self.wait_ready().await?;
            self.read_converted()
        }
    }

    impl<SCK, DOUT, D, E> zigbee_sensor::asynch::LowPower for Hx711<SCK, DOUT, D>
    where
        SCK: OutputPin<Error = E>,
        DOUT: InputPin<Error = E> + Wait,
        D: embedded_hal::delay::DelayNs,
    {
        type BusError = E;

        async fn sleep(&mut self) -> Result<(), Error<E>> {
            self.power_down()
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<E>> {
            self.power_up()
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::digital::{Mock, State, Transaction};
    use std::vec::Vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    fn sck_pulses(n: u8) -> Vec<Transaction> {
        (0..n)
            .flat_map(|_| [Transaction::set(State::High), Transaction::set(State::Low)])
            .collect()
    }

    fn dout_word(word: u32, ready_check: bool) -> Vec<Transaction> {
        let mut v = Vec::new();
        if ready_check {
            v.push(Transaction::get(State::Low));
        }
        for i in (0..24).rev() {
            v.push(Transaction::get(if word >> i & 1 != 0 {
                State::High
            } else {
                State::Low
            }));
        }
        v
    }

    #[test]
    fn sign_extension_matches_datasheet_coding() {
        assert_eq!(sign_extend_24(0x7F_FFFF), 8_388_607);
        assert_eq!(sign_extend_24(0x80_0000), -8_388_608);
        assert_eq!(sign_extend_24(0xFF_FFFF), -1);
        assert_eq!(sign_extend_24(0x00_0001), 1);
    }

    #[test]
    fn calibration_integer_math() {
        // Tare 8000 counts, 100 g reference reads 8000 + 42000 counts.
        let cal = Calibration::from_reference(8_000, 50_000, 100_000).unwrap();
        assert_eq!(cal.scale_q16, (100_000i64 * 65_536 / 42_000) as i32);
        assert_eq!(cal.mass_mg(8_000), 0);
        assert_eq!(cal.mass_mg(50_000), 99_999); // Q16 truncation
        assert_eq!(cal.mass_mg(29_000), 49_999);
        assert!(cal.mass_mg(7_000) < 0);
        assert_eq!(Calibration::from_reference(5, 5, 1), None);
    }

    #[test]
    fn not_ready_when_dout_high() {
        let mut sck = Mock::new(&[]);
        let mut dout = Mock::new(&[Transaction::get(State::High)]);
        let mut hx = Hx711::new(sck.clone(), dout.clone(), NoopDelay::new());
        assert_eq!(hx.start_measurement(), Ok(0));
        assert_eq!(hx.read_measurement(), Err(Error::NotReady));
        sck.done();
        dout.done();
    }

    #[test]
    fn reads_24_bits_and_25_pulses() {
        let mut sck = Mock::new(&sck_pulses(25));
        let mut dout = Mock::new(&dout_word(0xFF_FFF6, true));
        let mut hx =
            Hx711::new(sck.clone(), dout.clone(), NoopDelay::new()).with_calibration(Calibration {
                offset: -10,
                scale_q16: 1 << 16,
            });
        assert_eq!(
            hx.read_measurement().unwrap(),
            Measurement { raw: -10, mass: 0 }
        );
        sck.done();
        dout.done();
    }

    #[test]
    fn gain_change_programs_pulses_and_discards_sample() {
        let mut exp = sck_pulses(27);
        exp.extend(sck_pulses(27));
        let mut sck = Mock::new(&exp);
        let mut d = dout_word(0x12_3456, true);
        d.extend(dout_word(0x00_0100, true));
        let mut dout = Mock::new(&d);
        let mut hx = Hx711::new(sck.clone(), dout.clone(), NoopDelay::new()).with_rate(Rate::Sps80);
        assert_eq!(hx.set_gain(Gain::A64), 50_000);
        assert_eq!(hx.read_measurement(), Err(Error::NotReady));
        assert_eq!(hx.read_measurement().unwrap().raw, 0x100);
        sck.done();
        dout.done();
    }

    #[test]
    fn power_down_and_wake() {
        let mut sck = Mock::new(&[Transaction::set(State::High), Transaction::set(State::Low)]);
        let mut dout = Mock::new(&[]);
        let mut hx = Hx711::new(sck.clone(), dout.clone(), NoopDelay::new());
        hx.sleep().unwrap();
        assert_eq!(hx.wake().unwrap(), 400_000);
        sck.done();
        dout.done();
    }

    #[test]
    fn async_waits_for_dout_low() {
        use zigbee_sensor::asynch::Sensor as _;
        let mut sck = Mock::new(&sck_pulses(25));
        let mut d = std::vec![Transaction::wait_for_state(State::Low)];
        d.extend(dout_word(0x00_2710, true));
        let mut dout = Mock::new(&d);
        let mut hx = asynch::Hx711::new(sck.clone(), dout.clone(), NoopDelay::new())
            .with_calibration(Calibration {
                offset: 0,
                scale_q16: 3 << 15, // 1.5 mg per count
            });
        assert_eq!(
            block_on(hx.read_measurement()).unwrap(),
            Measurement {
                raw: 10_000,
                mass: 15_000
            }
        );
        sck.done();
        dout.done();
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
