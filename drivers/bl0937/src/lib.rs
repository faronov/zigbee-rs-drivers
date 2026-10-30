//! Belling BL0937 and HLW8012 pulse-output energy metering ICs.
//!
//! These chips have no bus: CF outputs pulses at a frequency proportional
//! to active power, CF1 at a frequency proportional to RMS current
//! (SEL = 0) or RMS voltage (SEL = 1). The platform counts pulses with a
//! timer/counter or edge interrupts; this driver drives SEL and converts
//! the counts with integer math.
//!
//! Datasheet formulas (pin voltages V(V), V(I) in V):
//!
//! ```text
//! BL0937 (V1.02):  F_CF  = 1721506·V(V)·V(I)/Vref²   Vref = 1.218 V
//!                  F_CFU = 15397·V(V)/Vref
//!                  F_CFI = 94638·V(I)/Vref
//! HLW8012 (REV 1.3): F_CF  = 48·V1·V2/Vref²·fosc/128  Vref = 2.43 V,
//!                  F_CFI = 24·V1/Vref·fosc/512        fosc = 3.579 MHz
//!                  F_CFU = 2·V2/Vref·fosc/512
//! ```
//!
//! (The BL0937 V1.02 legend swaps "V(V)" and "V(I)"; V(V) is the voltage
//! channel VP, V(I) the shunt voltage IP-IN.)
//!
//! Measurement cycle:
//!
//! 1. [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//!    drives SEL for the CF1 channel of this cycle ([`Bl0937::channel`];
//!    current and voltage alternate by default) and returns the counting
//!    window.
//! 2. The platform counts CF and CF1 during the window and hands over
//!    [`Pulses`] with [`Bl0937::set_cf`] / [`Bl0937::set_cf1`]. Count
//!    whole periods: `count` = rising edges − 1 between the first and
//!    the last edge, `duration_us` = time between them. This also skips
//!    the partial CF1 period after a SEL change (≤ 10 µs switching).
//!    Fewer than two edges → `Pulses::NONE` (zero / anti-creep).
//! 3. [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//!    consumes the windows, converts and, after a CF1 window, switches
//!    SEL to the next channel right away so a continuously counting
//!    platform can deliver windows before calling `measure()`.
//!    [`Error::NotReady`] if no CF window was supplied.
//!
//! Energy is the running total of CF pulses: pass every CF pulse once via
//! [`Bl0937::add_energy_pulses`] (e.g. from a hardware counter); the CF
//! window does not add energy by itself.
//!
//! The chips draw ~3 mA and have no sleep mode, so `LowPower` is not
//! implemented.

#![no_std]

use embedded_hal::digital::OutputPin;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Default counting window: 1 s (BL0937 reference: ~1.7 W per CF Hz).
pub const DEFAULT_WINDOW_US: u32 = 1_000_000;

/// Pulses counted over a window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Pulses {
    /// Number of whole periods.
    pub count: u32,
    /// Duration of those periods, µs.
    pub duration_us: u32,
}

impl Pulses {
    /// No pulses: frequency 0.
    pub const NONE: Self = Self {
        count: 0,
        duration_us: 0,
    };

    /// `count` periods in `duration_us`.
    pub const fn new(count: u32, duration_us: u32) -> Self {
        Self { count, duration_us }
    }

    /// One period of `period_us` (period-capture timers).
    pub const fn period(period_us: u32) -> Self {
        Self::new(1, period_us)
    }

    /// Frequency in mHz, 0 without pulses.
    pub const fn frequency_mhz(&self) -> u32 {
        scale(self.count, 1_000_000, self.duration_us) as u32
    }
}

/// `count · factor · 1000 / duration_us`, saturating; 0 for an empty window.
const fn scale(count: u32, factor: u32, duration_us: u32) -> i32 {
    if duration_us == 0 {
        return 0;
    }
    let v = (count as u64 * factor as u64).saturating_mul(1000) / duration_us as u64;
    if v > i32::MAX as u64 {
        i32::MAX
    } else {
        v as i32
    }
}

/// Supported chips (nominal transfer constants).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    /// Belling BL0937, Vref 1.218 V, 3.3 V supply.
    Bl0937,
    /// HLW8012, Vref 2.43 V, 5 V supply, fosc 3.579 MHz.
    Hlw8012,
}

impl Chip {
    /// (power, current, voltage) constants in 0.001 units and Vref in µV.
    const fn constants(self) -> (u64, u64, u64, u64) {
        match self {
            Chip::Bl0937 => (1_721_506_000, 94_638_000, 15_397_000, 1_218_000),
            // 48·fosc/128, 24·fosc/512, 2·fosc/512 with fosc = 3 579 000 Hz.
            Chip::Hlw8012 => (1_342_125_000, 167_765_625, 13_980_469, 2_430_000),
        }
    }
}

/// Conversion factors per Hz of output frequency.
///
/// Fields are public: after measuring a known load, scale a factor by
/// `true / reported`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// Active power per CF Hz, µW (= µJ per CF pulse).
    pub power_uw_per_hz: u32,
    /// RMS current per CF1 Hz (SEL = 0), nA.
    pub current_na_per_hz: u32,
    /// RMS voltage per CF1 Hz (SEL = 1), µV.
    pub voltage_uv_per_hz: u32,
}

const fn div_round(n: u128, d: u128) -> u128 {
    (n + d / 2) / d
}

const fn fit_u32(v: u128) -> Option<u32> {
    if v == 0 || v > u32::MAX as u128 {
        None
    } else {
        Some(v as u32)
    }
}

impl Calibration {
    /// BL0937 datasheet application: 1 mΩ shunt, 6 × 330 kΩ / 1 kΩ
    /// divider (≈ 1.707 W, 12.87 mA, 156.7 mV per Hz).
    pub const BL0937_REFERENCE: Self =
        match Self::from_circuit(Chip::Bl0937, 1_000, 1_980_000, 1_000) {
            Some(c) => c,
            None => panic!("reference calibration"),
        };

    /// HLW8012 datasheet application: 1 mΩ shunt, 6 × 470 kΩ / 1 kΩ
    /// divider (≈ 12.41 W, 14.48 mA, 490.3 mV per Hz).
    pub const HLW8012_REFERENCE: Self =
        match Self::from_circuit(Chip::Hlw8012, 1_000, 2_820_000, 1_000) {
            Some(c) => c,
            None => panic!("reference calibration"),
        };

    /// Nominal factors for a shunt of `shunt_uohm` µΩ and a line divider
    /// `r_top_ohm` / `r_bottom_ohm`. 128-bit math: evaluate it in a
    /// `const`. `None` if an input is zero or a factor rounds to 0 /
    /// exceeds `u32`.
    pub const fn from_circuit(
        chip: Chip,
        shunt_uohm: u32,
        r_top_ohm: u32,
        r_bottom_ohm: u32,
    ) -> Option<Self> {
        if shunt_uohm == 0 || r_bottom_ohm == 0 {
            return None;
        }
        let (kp, ki, ku, vref) = chip.constants();
        let (kp, ki, ku, vref) = (kp as u128, ki as u128, ku as u128, vref as u128);
        let r = shunt_uohm as u128;
        let rb = r_bottom_ohm as u128;
        let total = r_top_ohm as u128 + rb;
        // P = F·Vref²/(K·Rshunt·ratio), ratio = Rb/(Rt+Rb)
        let power = div_round(vref * vref * total * 1000, kp * r * rb);
        // I = F·Vref/(K·Rshunt)
        let current = div_round(vref * 1_000_000_000_000, ki * r);
        // V = F·Vref/(K·ratio)
        let voltage = div_round(vref * total * 1000, ku * rb);
        match (fit_u32(power), fit_u32(current), fit_u32(voltage)) {
            (Some(power_uw_per_hz), Some(current_na_per_hz), Some(voltage_uv_per_hz)) => {
                Some(Self {
                    power_uw_per_hz,
                    current_na_per_hz,
                    voltage_uv_per_hz,
                })
            }
            _ => None,
        }
    }

    /// CF pulses → mW.
    pub const fn power_mw(&self, cf: Pulses) -> i32 {
        scale(cf.count, self.power_uw_per_hz, cf.duration_us)
    }

    /// CF1 pulses (SEL = 0) → µA.
    pub const fn current_ua(&self, cf1: Pulses) -> i32 {
        scale(cf1.count, self.current_na_per_hz, cf1.duration_us)
    }

    /// CF1 pulses (SEL = 1) → mV.
    pub const fn voltage_mv(&self, cf1: Pulses) -> i32 {
        scale(cf1.count, self.voltage_uv_per_hz, cf1.duration_us)
    }

    /// Total CF pulses → mWh (one u64 division).
    pub const fn energy_mwh(&self, pulses: u64) -> u64 {
        pulses.saturating_mul(self.power_uw_per_hz as u64) / 3_600_000
    }
}

/// Power factor in 0.001 from P / (V·I), clamped to 0..=1000; `None`
/// without voltage or current. V and I come from different windows, so
/// this is only meaningful for steady loads.
pub fn power_factor(power_mw: i32, voltage_mv: i32, current_ua: i32) -> Option<i16> {
    let apparent = voltage_mv as i64 * current_ua as i64;
    if apparent <= 0 {
        return None;
    }
    let pf = power_mw as i64 * 1_000_000_000 / apparent;
    Some(pf.clamp(0, 1000) as i16)
}

/// CF1 channel selected by SEL.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    /// SEL low: RMS current.
    Current,
    /// SEL high: RMS voltage.
    Voltage,
}

impl Channel {
    const fn other(self) -> Self {
        match self {
            Channel::Current => Channel::Voltage,
            Channel::Voltage => Channel::Current,
        }
    }
}

/// CF1 channel schedule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cf1Mode {
    /// Alternate current and voltage every cycle (default).
    Alternate,
    /// Always current (e.g. voltage assumed constant).
    CurrentOnly,
    /// Always voltage.
    VoltageOnly,
}

/// One reading. Voltage and current are the latest values of their
/// channels (`None` until measured once).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// RMS line voltage, mV.
    pub voltage: Option<i32>,
    /// RMS current, µA.
    pub current: Option<i32>,
    /// Active power, mW (always ≥ 0).
    pub power: i32,
    /// Accumulated energy, mWh.
    pub energy: u64,
    /// Power factor, 0.001.
    pub power_factor: Option<i16>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if let Some(v) = self.voltage {
            f(Quantity::Voltage(v));
        }
        if let Some(v) = self.current {
            f(Quantity::Current(v));
        }
        f(Quantity::Power(self.power));
        f(Quantity::Energy(self.energy));
        if let Some(v) = self.power_factor {
            f(Quantity::PowerFactor(v));
        }
    }
}

/// Bus-independent state shared by both drivers.
#[derive(Debug)]
struct State {
    calibration: Calibration,
    mode: Cf1Mode,
    window_us: u32,
    channel: Channel,
    cf: Option<Pulses>,
    cf1: Option<Pulses>,
    voltage: Option<i32>,
    current: Option<i32>,
    energy_pulses: u64,
}

impl State {
    const fn new(calibration: Calibration) -> Self {
        Self {
            calibration,
            mode: Cf1Mode::Alternate,
            window_us: DEFAULT_WINDOW_US,
            channel: Channel::Current,
            cf: None,
            cf1: None,
            voltage: None,
            current: None,
            energy_pulses: 0,
        }
    }

    /// Channel of the cycle being started.
    fn start(&mut self) -> Channel {
        self.channel = match self.mode {
            Cf1Mode::Alternate => self.channel,
            Cf1Mode::CurrentOnly => Channel::Current,
            Cf1Mode::VoltageOnly => Channel::Voltage,
        };
        self.channel
    }

    /// Convert the supplied windows; `true` if SEL must switch to the new
    /// [`Self::channel`].
    fn finish<E>(&mut self) -> Result<(Measurement, bool), Error<E>> {
        let cf = self.cf.take().ok_or(Error::NotReady)?;
        let cal = &self.calibration;
        let mut switched = false;
        if let Some(cf1) = self.cf1.take() {
            match self.channel {
                Channel::Current => self.current = Some(cal.current_ua(cf1)),
                Channel::Voltage => self.voltage = Some(cal.voltage_mv(cf1)),
            }
            if self.mode == Cf1Mode::Alternate {
                self.channel = self.channel.other();
                switched = true;
            }
        }
        let power = cal.power_mw(cf);
        let m = Measurement {
            voltage: self.voltage,
            current: self.current,
            power,
            energy: cal.energy_mwh(self.energy_pulses),
            power_factor: match (self.voltage, self.current) {
                (Some(v), Some(i)) => power_factor(power, v, i),
                _ => None,
            },
        };
        Ok((m, switched))
    }
}

fn drive<P: OutputPin>(sel: &mut P, channel: Channel) -> Result<(), Error<P::Error>> {
    match channel {
        Channel::Current => sel.set_low(),
        Channel::Voltage => sel.set_high(),
    }
    .map_err(Error::bus)
}

macro_rules! driver {
    ($name:ident) => {
        impl<SEL> $name<SEL> {
            /// Driver with the default 1 s window and alternating CF1
            /// channels; no pin activity.
            pub const fn new(sel: SEL, calibration: Calibration) -> Self {
                Self {
                    sel,
                    state: State::new(calibration),
                }
            }

            /// Counting window returned by `start_measurement`.
            pub const fn with_window_us(mut self, window_us: u32) -> Self {
                self.state.window_us = window_us;
                self
            }

            /// CF1 channel schedule.
            pub const fn with_cf1_mode(mut self, mode: Cf1Mode) -> Self {
                self.state.mode = mode;
                self
            }

            /// Active calibration.
            pub const fn calibration(&self) -> &Calibration {
                &self.state.calibration
            }

            /// CF1 channel of the current cycle.
            pub const fn channel(&self) -> Channel {
                self.state.channel
            }

            /// CF pulses counted in this cycle's window.
            pub fn set_cf(&mut self, cf: Pulses) {
                self.state.cf = Some(cf);
            }

            /// CF1 pulses counted in this cycle's window (channel
            /// [`Self::channel`]).
            pub fn set_cf1(&mut self, cf1: Pulses) {
                self.state.cf1 = Some(cf1);
            }

            /// Add CF pulses to the energy total.
            pub fn add_energy_pulses(&mut self, pulses: u32) {
                self.state.energy_pulses = self.state.energy_pulses.saturating_add(pulses as u64);
            }

            /// Energy total in CF pulses (persist it across resets).
            pub const fn energy_pulses(&self) -> u64 {
                self.state.energy_pulses
            }

            /// Restore an energy total.
            pub fn set_energy_pulses(&mut self, pulses: u64) {
                self.state.energy_pulses = pulses;
            }

            /// Give the SEL pin back.
            pub fn release(self) -> SEL {
                self.sel
            }
        }
    };
}

/// Blocking driver (SEL control + conversion).
#[derive(Debug)]
pub struct Bl0937<SEL> {
    sel: SEL,
    state: State,
}

driver!(Bl0937);

impl<SEL: OutputPin> zigbee_sensor::blocking::Sensor for Bl0937<SEL> {
    type BusError = SEL::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<SEL::Error>> {
        let channel = self.state.start();
        drive(&mut self.sel, channel)?;
        Ok(self.state.window_us)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<SEL::Error>> {
        let (m, switched) = self.state.finish()?;
        if switched {
            drive(&mut self.sel, self.state.channel)?;
        }
        Ok(m)
    }
}

/// Async driver (the SEL pin is still a blocking `OutputPin`).
pub mod asynch {
    use super::{Calibration, Cf1Mode, Channel, Measurement, Pulses, State, drive};
    use embedded_hal::digital::OutputPin;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Bl0937`].
    #[derive(Debug)]
    pub struct Bl0937<SEL> {
        sel: SEL,
        state: State,
    }

    driver!(Bl0937);

    impl<SEL: OutputPin> zigbee_sensor::asynch::Sensor for Bl0937<SEL> {
        type BusError = SEL::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<SEL::Error>> {
            let channel = self.state.start();
            drive(&mut self.sel, channel)?;
            Ok(self.state.window_us)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<SEL::Error>> {
            let (m, switched) = self.state.finish()?;
            if switched {
                drive(&mut self.sel, self.state.channel)?;
            }
            Ok(m)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::digital::{
        Mock as PinMock, State as PinState, Transaction as PinT,
    };
    use zigbee_sensor::blocking::Sensor;

    // BL0937 reference circuit at 230 V, 1 A, cos φ = 1:
    // V(V) = 230·1000/1 981 000 = 0.116103 V, V(I) = 0.001 V.
    // F_CF = 1721506·0.116103·0.001/1.218² = 134.73 Hz,
    // F_CFU = 15397·0.116103/1.218 = 1467.7 Hz, F_CFI = 94638·0.001/1.218 = 77.70 Hz.

    #[test]
    fn datasheet_frequencies_convert_back() {
        let c = Calibration::BL0937_REFERENCE;
        assert_eq!(
            c,
            Calibration {
                power_uw_per_hz: 1_707_145,
                current_na_per_hz: 12_870_094,
                voltage_uv_per_hz: 156_710,
            }
        );
        // 1347 pulses in 10 s, 777 in 10 s, 14677 in 10 s.
        assert_eq!(c.power_mw(Pulses::new(1_347, 10_000_000)), 229_952);
        assert_eq!(c.current_ua(Pulses::new(777, 10_000_000)), 1_000_006);
        assert_eq!(c.voltage_mv(Pulses::new(14_677, 10_000_000)), 230_003);
        // Period capture: one CF period of 7.4223 ms.
        assert_eq!(c.power_mw(Pulses::period(7_422)), 230_011);
        assert_eq!(c.power_mw(Pulses::NONE), 0);
        // 1 kWh = 3.6e9 µJ / 1.707145 J per pulse ≈ 2 108 785 pulses.
        assert_eq!(c.energy_mwh(2_108_785), 1_000_000);
        assert_eq!(Pulses::new(1_347, 10_000_000).frequency_mhz(), 134_700);

        let h = Calibration::HLW8012_REFERENCE;
        // HLW8012 at 230 V, 1 A: V2 = 0.0815314 V, V1 = 0.001 V.
        // F_CF = 1342125·V1·V2/2.43² = 18.531 Hz, F_CFI = 69.04 Hz,
        // F_CFU = 469.07 Hz.
        assert_eq!(h.power_mw(Pulses::new(18_531, 1_000_000_000)), 229_996);
        assert_eq!(h.current_ua(Pulses::new(6_904, 100_000_000)), 1_000_009);
        assert_eq!(h.voltage_mv(Pulses::new(46_907, 100_000_000)), 229_998);
        assert_eq!(Calibration::from_circuit(Chip::Bl0937, 0, 1, 1), None);
        assert_eq!(power_factor(115_000, 230_000, 1_000_000), Some(500));
        assert_eq!(power_factor(1, 230_000, 0), None);
    }

    #[test]
    fn alternates_sel_and_keeps_latest_values() {
        let expectations = [
            PinT::set(PinState::Low),
            PinT::set(PinState::High),
            PinT::set(PinState::High),
            PinT::set(PinState::Low),
            PinT::set(PinState::Low),
        ];
        let mut sel = PinMock::new(&expectations);
        let mut meter =
            Bl0937::new(sel.clone(), Calibration::BL0937_REFERENCE).with_window_us(2_000_000);
        assert_eq!(meter.read_measurement(), Err(Error::NotReady));

        assert_eq!(meter.start_measurement().unwrap(), 2_000_000);
        assert_eq!(meter.channel(), Channel::Current);
        meter.set_cf(Pulses::new(269, 1_996_000));
        meter.set_cf1(Pulses::new(155, 1_995_000));
        meter.add_energy_pulses(270);
        let m = meter.read_measurement().unwrap();
        assert_eq!(m.voltage, None);
        assert_eq!(m.current, Some(999_932));
        assert_eq!(m.power, 230_071);
        assert_eq!(m.energy, 128);
        assert_eq!(m.power_factor, None);

        meter.start_measurement().unwrap();
        assert_eq!(meter.channel(), Channel::Voltage);
        meter.set_cf(Pulses::new(269, 1_996_000));
        meter.set_cf1(Pulses::new(2_935, 1_999_900));
        let m = meter.read_measurement().unwrap();
        assert_eq!(m.voltage, Some(229_983));
        assert_eq!(m.current, Some(999_932));
        assert_eq!(m.power_factor, Some(1000));

        // A cycle without a CF window is not ready and keeps the channel.
        meter.start_measurement().unwrap();
        assert_eq!(meter.read_measurement(), Err(Error::NotReady));
        assert_eq!(meter.channel(), Channel::Current);
        assert_eq!(meter.energy_pulses(), 270);
        meter.release();
        sel.done();
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

        let expectations = [PinT::set(PinState::High)];
        let mut sel = PinMock::new(&expectations);
        let mut meter = asynch::Bl0937::new(sel.clone(), Calibration::HLW8012_REFERENCE)
            .with_cf1_mode(Cf1Mode::VoltageOnly)
            .with_window_us(500_000);
        // A continuously counting platform delivers the windows first.
        meter.set_cf(Pulses::NONE);
        meter.set_cf1(Pulses::new(234, 498_900));
        let mut delay = RecordingDelay(0);
        let m = block_on(meter.measure(&mut delay)).unwrap();
        assert_eq!(
            m,
            Measurement {
                voltage: Some(229_979),
                current: None,
                power: 0,
                energy: 0,
                power_factor: None,
            }
        );
        assert_eq!(delay.0, 500_000);
        meter.release();
        sel.done();
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
