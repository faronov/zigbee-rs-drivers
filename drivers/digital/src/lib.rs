//! Debounced GPIO detectors: PIR motion modules, reed/contact switches,
//! water-leak probes, hall switches and vibration (tilt/ball) switches.
//!
//! The core is a pure state machine, [`Detector`], fed with
//! `(level, timestamp_ms)` pairs by the application (from a GPIO interrupt
//! and from a timer). It owns no timer: after every call the application
//! asks [`Detector::next_deadline`] when it must call [`Detector::poll`]
//! again, and schedules that itself. All times are `u32` milliseconds of a
//! free-running clock and may wrap; intervals must stay below 2³¹ ms.
//!
//! Two stages:
//!
//! 1. **Debounce**: an input change is accepted once the input has been
//!    stable for `debounce_ms` (reed switches bounce for ~1 ms, ball
//!    vibration switches for several ms; a few tens of ms is typical).
//! 2. **Hold** (PIR, vibration): the output stays active until the input
//!    has been inactive for `hold_ms`. This turns short PIR pulses into a
//!    ZCL occupancy that clears after a period without motion.
//!
//! [`PinDetector`] wraps a [`Detector`] around an `embedded_hal`
//! [`InputPin`]; its async twin in
//! [`asynch`] can also await the next edge. Their `Sensor` implementations
//! return the instantaneous (not debounced) pin state, for nodes that only
//! poll a contact when they wake up.
//!
//! [`Reading`] emits `Quantity::Occupancy` for PIR (ZCL 0x0406),
//! `Quantity::Contact` for reed contacts and hall switches (`true` =
//! closed / magnet present) and `Quantity::Alarm` for water leak and
//! vibration (usually reported via an IAS Zone ZoneStatus bit).
//!
//! ```ignore
//! let mut pir = PinDetector::new(pin, Config::pir(60_000));
//! pir.init(now_ms())?;
//! // in the GPIO interrupt / timer handler:
//! if let Some(occupied) = pir.sample(now_ms())? { report(occupied); }
//! if let Some(at) = pir.detector().next_deadline() { schedule_poll(at); }
//! // when the scheduled time arrives:
//! if let Some(occupied) = pir.detector_mut().poll(now_ms()) { report(occupied); }
//! ```

#![no_std]

use embedded_hal::digital::InputPin;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Electrical level that means "active" (motion, contact closed, water,
/// magnet present, vibration).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ActiveLevel {
    /// Active when the pin reads high (PIR modules such as HC-SR501/AM312).
    #[default]
    High,
    /// Active when the pin reads low (switch to GND with a pull-up).
    Low,
}

impl ActiveLevel {
    /// Map an electrical level (`true` = high) to the logical state.
    pub const fn is_active(self, high: bool) -> bool {
        match self {
            Self::High => high,
            Self::Low => !high,
        }
    }
}

/// What the input represents; selects the reported quantity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// PIR / microwave motion output: `Quantity::Occupancy`.
    Occupancy,
    /// Reed or mechanical contact: active = closed; `Quantity::Contact`.
    Contact,
    /// Water-leak probe: active = water detected; `Quantity::Alarm`.
    WaterLeak,
    /// Digital hall switch: active = magnet present; `Quantity::Contact`
    /// (closed when the magnet is near, as in door sensors).
    HallSwitch,
    /// Vibration / tilt switch: active = vibration detected;
    /// `Quantity::Alarm`.
    Vibration,
}

/// Detector configuration. Builder methods need no hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Meaning of the input.
    pub kind: Kind,
    /// Active electrical level.
    pub active_level: ActiveLevel,
    /// Time the input must stay at a new level before it is accepted.
    pub debounce_ms: u32,
    /// Time the output stays active after the input went inactive.
    pub hold_ms: u32,
}

impl Config {
    /// Generic configuration: no debounce, no hold.
    pub const fn new(kind: Kind, active_level: ActiveLevel) -> Self {
        Self {
            kind,
            active_level,
            debounce_ms: 0,
            hold_ms: 0,
        }
    }

    /// PIR motion module (active high, 50 ms glitch filter) holding
    /// occupancy for `hold_ms` after the last motion pulse.
    pub const fn pir(hold_ms: u32) -> Self {
        Self::new(Kind::Occupancy, ActiveLevel::High)
            .with_debounce_ms(50)
            .with_hold_ms(hold_ms)
    }

    /// Reed / door contact to GND with pull-up (active low = closed),
    /// 20 ms debounce.
    pub const fn contact() -> Self {
        Self::new(Kind::Contact, ActiveLevel::Low).with_debounce_ms(20)
    }

    /// Water-leak probe pulling the input low when wet, 500 ms debounce
    /// against splashes.
    pub const fn water_leak() -> Self {
        Self::new(Kind::WaterLeak, ActiveLevel::Low).with_debounce_ms(500)
    }

    /// Open-drain hall switch (active low when a magnet is present),
    /// 10 ms debounce.
    pub const fn hall_switch() -> Self {
        Self::new(Kind::HallSwitch, ActiveLevel::Low).with_debounce_ms(10)
    }

    /// Ball/spring vibration switch to GND (active low), no debounce (the
    /// bounces *are* the signal) and an alarm hold of `hold_ms`.
    pub const fn vibration(hold_ms: u32) -> Self {
        Self::new(Kind::Vibration, ActiveLevel::Low).with_hold_ms(hold_ms)
    }

    /// Set the active level.
    pub const fn with_active_level(mut self, level: ActiveLevel) -> Self {
        self.active_level = level;
        self
    }

    /// Set the debounce time.
    pub const fn with_debounce_ms(mut self, ms: u32) -> Self {
        self.debounce_ms = ms;
        self
    }

    /// Set the hold (occupancy clear) time.
    pub const fn with_hold_ms(mut self, ms: u32) -> Self {
        self.hold_ms = ms;
        self
    }
}

/// Output of a detector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reading {
    /// Meaning of `active`.
    pub kind: Kind,
    /// Occupied / closed / wet / magnet / vibration.
    pub active: bool,
}

impl Readings for Reading {
    /// Emits `Occupancy`, `Contact` or `Alarm` depending on [`Kind`].
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(match self.kind {
            Kind::Occupancy => Quantity::Occupancy(self.active),
            Kind::Contact | Kind::HallSwitch => Quantity::Contact(self.active),
            Kind::WaterLeak | Kind::Vibration => Quantity::Alarm(self.active),
        });
    }
}

/// `true` when `now` is at or after `deadline` on a wrapping clock.
const fn reached(now: u32, deadline: u32) -> bool {
    (now.wrapping_sub(deadline) as i32) >= 0
}

/// Pure debounce + hold state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Detector {
    config: Config,
    /// Debounced logical input.
    input: bool,
    /// Logical level waiting for the debounce time, if any.
    candidate: Option<(bool, u32)>,
    /// Reported output.
    output: bool,
    /// Time the debounced input went inactive while the output is held.
    released_at: Option<u32>,
}

impl Detector {
    /// Create an inactive detector.
    pub const fn new(config: Config) -> Self {
        Self {
            config,
            input: false,
            candidate: None,
            output: false,
            released_at: None,
        }
    }

    /// Configuration in use.
    pub const fn config(&self) -> &Config {
        &self.config
    }

    /// Set the initial electrical level (`true` = high) without producing
    /// an event, e.g. at boot.
    pub fn init(&mut self, high: bool) {
        let active = self.config.active_level.is_active(high);
        self.input = active;
        self.output = active;
        self.candidate = None;
        self.released_at = None;
    }

    /// Current output: occupied / closed / wet / ...
    pub const fn is_active(&self) -> bool {
        self.output
    }

    /// Current output as a [`Reading`].
    pub const fn reading(&self) -> Reading {
        Reading {
            kind: self.config.kind,
            active: self.output,
        }
    }

    /// Feed an electrical level (`true` = high) sampled at `now_ms`, e.g.
    /// from the pin-change interrupt. Returns the new output if it changed.
    pub fn update(&mut self, high: bool, now_ms: u32) -> Option<bool> {
        let active = self.config.active_level.is_active(high);
        if active == self.input {
            // Bounced back before the debounce time: drop the candidate.
            self.candidate = None;
        } else if self.candidate.is_none_or(|(level, _)| level != active) {
            self.candidate = Some((active, now_ms));
        }
        self.poll(now_ms)
    }

    /// Advance time without a new sample. Returns the new output if it
    /// changed (debounce elapsed or hold expired).
    pub fn poll(&mut self, now_ms: u32) -> Option<bool> {
        if let Some((level, since)) = self.candidate {
            if reached(now_ms, since.wrapping_add(self.config.debounce_ms)) {
                self.candidate = None;
                self.input = level;
                // The input changed at `since`; hold timing starts there.
                self.released_at = if level { None } else { Some(since) };
            }
        }

        let output = if self.input {
            true
        } else {
            match self.released_at {
                Some(at) if self.output => {
                    if reached(now_ms, at.wrapping_add(self.config.hold_ms)) {
                        self.released_at = None;
                        false
                    } else {
                        true
                    }
                }
                _ => false,
            }
        };

        if output != self.output {
            self.output = output;
            Some(output)
        } else {
            None
        }
    }

    /// Time at which [`Self::poll`] must be called next, if any timer is
    /// running (pending debounce or occupancy hold).
    pub fn next_deadline(&self) -> Option<u32> {
        let debounce = self
            .candidate
            .map(|(_, since)| since.wrapping_add(self.config.debounce_ms));
        let hold = match self.released_at {
            Some(at) if self.output && !self.input => Some(at.wrapping_add(self.config.hold_ms)),
            _ => None,
        };
        match (debounce, hold) {
            (Some(a), Some(b)) => Some(if reached(a, b) { b } else { a }),
            (a, b) => a.or(b),
        }
    }
}

macro_rules! pin_common {
    () => {
        /// Wrap `pin`; no pin access.
        pub const fn new(pin: P, config: Config) -> Self {
            Self {
                pin,
                detector: Detector::new(config),
            }
        }

        /// The state machine (for [`Detector::next_deadline`]).
        pub const fn detector(&self) -> &Detector {
            &self.detector
        }

        /// The state machine (for [`Detector::poll`]).
        pub fn detector_mut(&mut self) -> &mut Detector {
            &mut self.detector
        }

        /// Give the pin back.
        pub fn release(self) -> P {
            self.pin
        }
    };
}

macro_rules! pin_sampling {
    () => {
        /// Read the pin and set it as the initial state (no event).
        pub fn init(&mut self, _now_ms: u32) -> Result<(), Error<P::Error>> {
            let high = self.pin.is_high().map_err(Error::bus)?;
            self.detector.init(high);
            Ok(())
        }

        /// Read the pin and feed it to the detector at `now_ms`. Returns
        /// the new output if it changed.
        pub fn sample(&mut self, now_ms: u32) -> Result<Option<bool>, Error<P::Error>> {
            let high = self.pin.is_high().map_err(Error::bus)?;
            Ok(self.detector.update(high, now_ms))
        }

        /// Instantaneous logical pin state (not debounced, no hold).
        pub fn read_instant(&mut self) -> Result<Reading, Error<P::Error>> {
            let high = self.pin.is_high().map_err(Error::bus)?;
            let config = self.detector.config();
            Ok(Reading {
                kind: config.kind,
                active: config.active_level.is_active(high),
            })
        }
    };
}

/// [`Detector`] reading an `embedded_hal` input pin.
pub struct PinDetector<P> {
    pin: P,
    detector: Detector,
}

impl<P> PinDetector<P> {
    pin_common!();
}

impl<P: InputPin> PinDetector<P> {
    pin_sampling!();
}

impl<P: InputPin> zigbee_sensor::blocking::Sensor for PinDetector<P> {
    type BusError = P::Error;
    type Measurement = Reading;

    /// The pin is always readable: returns `0`.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<P::Error>> {
        Ok(0)
    }

    /// Instantaneous pin state; see [`PinDetector::read_instant`].
    fn read_measurement(&mut self) -> Result<Reading, Error<P::Error>> {
        self.read_instant()
    }
}

/// Async twin: same sampling plus edge waiting.
pub mod asynch {
    use super::{Config, Detector, Reading};
    use embedded_hal::digital::InputPin;
    use embedded_hal_async::digital::Wait;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::PinDetector`].
    pub struct PinDetector<P> {
        pin: P,
        detector: Detector,
    }

    impl<P> PinDetector<P> {
        pin_common!();
    }

    impl<P: InputPin> PinDetector<P> {
        pin_sampling!();
    }

    impl<P: InputPin + Wait> PinDetector<P> {
        /// Wait for any edge on the pin. Afterwards call
        /// [`Self::sample`] with the current time. Combine with a timer on
        /// [`Detector::next_deadline`] (e.g. `select`) to also catch the
        /// debounce/hold expiry.
        pub async fn wait_for_edge(&mut self) -> Result<(), Error<P::Error>> {
            self.pin.wait_for_any_edge().await.map_err(Error::bus)
        }
    }

    impl<P: InputPin> zigbee_sensor::asynch::Sensor for PinDetector<P> {
        type BusError = P::Error;
        type Measurement = Reading;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<P::Error>> {
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Reading, Error<P::Error>> {
            self.read_instant()
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::digital::{Edge, Mock, State, Transaction};
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    #[test]
    fn contact_debounce_ignores_bounces() {
        let mut d = Detector::new(Config::contact());
        d.init(true); // pull-up: open
        assert!(!d.is_active());
        // Closing contact bounces for 3 ms.
        assert_eq!(d.update(false, 100), None);
        assert_eq!(d.update(true, 101), None);
        assert_eq!(d.next_deadline(), None);
        assert_eq!(d.update(false, 103), None);
        assert_eq!(d.next_deadline(), Some(123));
        assert_eq!(d.poll(122), None);
        assert_eq!(d.poll(123), Some(true));
        assert_eq!(d.next_deadline(), None);
        // Opening.
        assert_eq!(d.update(true, 500), None);
        assert_eq!(d.update(true, 510), None); // same level again keeps start
        assert_eq!(d.update(true, 520), Some(false));
    }

    #[test]
    fn pir_holds_occupancy_until_timeout() {
        let mut d = Detector::new(Config::pir(60_000));
        d.init(false);
        assert_eq!(d.update(true, 1_000), None);
        assert_eq!(d.poll(1_050), Some(true));
        assert!(d.reading().active);
        // Pulse ends after 2 s; occupancy is held 60 s from the falling edge.
        assert_eq!(d.update(false, 3_000), None);
        assert_eq!(d.poll(3_050), None);
        assert_eq!(d.next_deadline(), Some(63_000));
        // Re-trigger at 30 s restarts the hold.
        assert_eq!(d.update(true, 30_000), None);
        assert_eq!(d.poll(30_050), None);
        assert_eq!(d.next_deadline(), None);
        assert_eq!(d.update(false, 31_000), None);
        assert_eq!(d.poll(31_050), None);
        assert_eq!(d.next_deadline(), Some(91_000));
        assert_eq!(d.poll(90_999), None);
        assert_eq!(d.poll(91_000), Some(false));
        assert_eq!(d.next_deadline(), None);
    }

    #[test]
    fn short_glitch_is_rejected_and_clock_wraps() {
        let mut d = Detector::new(Config::pir(1_000));
        d.init(false);
        let t0 = u32::MAX - 10;
        assert_eq!(d.update(true, t0), None);
        assert_eq!(d.update(false, t0.wrapping_add(20)), None); // 20 ms glitch
        assert_eq!(d.poll(t0.wrapping_add(100)), None);
        assert!(!d.is_active());
        assert_eq!(d.update(true, t0.wrapping_add(200)), None);
        assert_eq!(d.poll(t0.wrapping_add(250)), Some(true));
    }

    #[test]
    fn deadline_is_earliest_timer() {
        let mut d = Detector::new(Config::pir(100).with_debounce_ms(50));
        d.init(false);
        d.update(true, 0);
        d.poll(50);
        d.update(false, 60);
        d.poll(110); // input now inactive, released at 60
        assert_eq!(d.next_deadline(), Some(160));
        d.update(true, 130); // debounce deadline 180, hold 160
        assert_eq!(d.next_deadline(), Some(160));
    }

    #[test]
    fn readings_per_kind() {
        let mut out = Vec::new();
        for (kind, active) in [
            (Kind::Occupancy, true),
            (Kind::Contact, false),
            (Kind::HallSwitch, true),
            (Kind::WaterLeak, true),
            (Kind::Vibration, false),
        ] {
            Reading { kind, active }.for_each(&mut |q| out.push(q));
        }
        assert_eq!(
            out,
            [
                Quantity::Occupancy(true),
                Quantity::Contact(false),
                Quantity::Contact(true),
                Quantity::Alarm(true),
                Quantity::Alarm(false),
            ]
        );
    }

    #[test]
    fn pin_detector_samples_pin() {
        let expectations = [
            Transaction::get(State::High),
            Transaction::get(State::Low),
            Transaction::get(State::Low),
            Transaction::get(State::Low),
        ];
        let mut pin = Mock::new(&expectations);
        let mut contact = PinDetector::new(pin.clone(), Config::contact());
        contact.init(0).unwrap();
        assert_eq!(contact.sample(10).unwrap(), None);
        assert_eq!(contact.sample(30).unwrap(), Some(true));
        assert_eq!(
            contact.read_measurement().unwrap(),
            Reading {
                kind: Kind::Contact,
                active: true
            }
        );
        pin.done();
    }

    #[test]
    fn async_waits_for_edge() {
        use zigbee_sensor::asynch::Sensor as _;
        let expectations = [
            Transaction::wait_for_edge(Edge::Any),
            Transaction::get(State::High),
            Transaction::get(State::High),
        ];
        let mut pin = Mock::new(&expectations);
        let mut pir = asynch::PinDetector::new(pin.clone(), Config::pir(1_000).with_debounce_ms(0));
        block_on(pir.wait_for_edge()).unwrap();
        assert_eq!(pir.sample(5).unwrap(), Some(true));
        let r = block_on(pir.read_measurement()).unwrap();
        assert!(r.active);
        pin.done();
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
