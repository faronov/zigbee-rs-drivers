//! Battery state from a voltage measured by an ADC through an optional
//! resistor divider.
//!
//! Pure crate: the only "bus" is [`zigbee_sensor::adc::Millivolts`] /
//! [`zigbee_sensor::adc::AsyncMillivolts`] supplied by the platform HAL.
//! The pin voltage is scaled by the [`Divider`] and mapped to a state of
//! charge with a piecewise-linear [`Curve`]. Presets: [`CR2032`],
//! [`CR2450`], [`ALKALINE_2X`] (2 × AA / AAA), [`LI_ION_1S`] (Li-ion /
//! LiPo), [`LIFEPO4_1S`] and [`NIMH_2X`]. The curves are typical
//! open-circuit / light-load shapes from manufacturers' discharge graphs —
//! approximate; tune them (or build a [`Curve`]) for the actual load.
//!
//! Load compensation: a cell sags under load by `I_load × R_internal`
//! (a CR2032 has 10-40 Ω, more when cold or near empty, so a 10 mA radio
//! burst costs 100-400 mV). Prefer sampling while the radio is idle; if the
//! sample has to be taken under a known load, add the expected drop with
//! [`Battery::with_compensation_mv`]. It only shifts the curve lookup; the
//! reported voltage stays the measured one.
//!
//! The ZCL Power Configuration attributes come from the core helpers:
//!
//! ```
//! use zigbee_battery::{Battery, CR2032, Divider};
//! use zigbee_sensor::{blocking::Sensor, zcl};
//!
//! struct Adc; // platform ADC channel, already calibrated to mV
//! impl zigbee_sensor::adc::Millivolts for Adc {
//!     type Error = ();
//!     fn read_mv(&mut self) -> Result<u32, ()> {
//!         Ok(1_400) // 2.8 V behind a 1:1 divider
//!     }
//! }
//!
//! let mut battery = Battery::new(Adc, CR2032).with_divider(Divider::new(100_000, 100_000));
//! let m = battery.read_measurement().unwrap();
//! assert_eq!(zcl::battery_voltage(m.voltage as u32), 28); // 0x0020, 100 mV
//! assert_eq!(zcl::battery_percentage(m.percent), 140); // 0x0021, 70 %
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Resistor divider between the battery and the ADC pin.
///
/// `battery = pin × (r_top + r_bottom) / r_bottom`, computed in `u64`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Divider {
    /// Resistor from the battery to the pin (Ω); 0 = direct connection.
    pub r_top_ohm: u32,
    /// Resistor from the pin to ground (Ω).
    pub r_bottom_ohm: u32,
}

impl Divider {
    /// Battery connected straight to the ADC (or measured as VDD).
    pub const NONE: Self = Self {
        r_top_ohm: 0,
        r_bottom_ohm: 1,
    };

    /// Divider from its resistors; a zero `r_bottom_ohm` means direct.
    pub const fn new(r_top_ohm: u32, r_bottom_ohm: u32) -> Self {
        if r_bottom_ohm == 0 {
            Self::NONE
        } else {
            Self {
                r_top_ohm,
                r_bottom_ohm,
            }
        }
    }

    /// Pin voltage → battery voltage (mV, rounded, saturating at
    /// `u32::MAX`).
    pub const fn battery_mv(&self, pin_mv: u32) -> u32 {
        let num = pin_mv as u64 * (self.r_top_ohm as u64 + self.r_bottom_ohm as u64);
        let mv = (num + self.r_bottom_ohm as u64 / 2) / self.r_bottom_ohm as u64;
        if mv > u32::MAX as u64 {
            u32::MAX
        } else {
            mv as u32
        }
    }
}

impl Default for Divider {
    fn default() -> Self {
        Self::NONE
    }
}

/// One curve point: battery voltage and remaining charge in 0.5 % units.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Point {
    /// Battery voltage (mV).
    pub mv: u16,
    /// Remaining charge, 0..=200 (0.5 %).
    pub half_percent: u8,
}

const fn p(mv: u16, percent: u8) -> Point {
    Point {
        mv,
        half_percent: percent * 2,
    }
}

/// Piecewise-linear discharge curve, points ordered by strictly falling
/// voltage and non-increasing charge. Above the first point → its charge,
/// below the last → its charge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Curve {
    points: &'static [Point],
}

impl Curve {
    /// Validate and wrap a table; `None` when empty, unordered or above
    /// 200 half-percent.
    pub const fn new(points: &'static [Point]) -> Option<Self> {
        if points.is_empty() {
            return None;
        }
        let mut i = 0;
        while i < points.len() {
            if points[i].half_percent > 200 {
                return None;
            }
            if i > 0
                && (points[i].mv >= points[i - 1].mv
                    || points[i].half_percent > points[i - 1].half_percent)
            {
                return None;
            }
            i += 1;
        }
        Some(Self { points })
    }

    /// The table.
    pub const fn points(&self) -> &'static [Point] {
        self.points
    }

    /// Battery voltage → remaining charge (0.5 % units, rounded).
    pub const fn half_percent(&self, mv: u32) -> u8 {
        let pts = self.points;
        if mv >= pts[0].mv as u32 {
            return pts[0].half_percent;
        }
        let mut i = 1;
        while i < pts.len() {
            let (hi, lo) = (pts[i - 1], pts[i]);
            if mv > lo.mv as u32 {
                let span_mv = (hi.mv - lo.mv) as u32;
                let span_hp = (hi.half_percent - lo.half_percent) as u32;
                let hp = ((mv - lo.mv as u32) * span_hp + span_mv / 2) / span_mv;
                return lo.half_percent + hp as u8;
            }
            i += 1;
        }
        pts[pts.len() - 1].half_percent
    }
}

/// CR2032 lithium coin cell (Li-MnO₂, 3.0 V nominal, light load / at rest;
/// long plateau around 2.9 V, steep knee below 2.6 V, cut-off 2.0 V).
pub const CR2032: Curve = Curve {
    points: &[
        p(3_000, 100),
        p(2_900, 90),
        p(2_800, 70),
        p(2_700, 40),
        p(2_600, 20),
        p(2_500, 10),
        p(2_300, 5),
        p(2_000, 0),
    ],
};

/// CR2450 lithium coin cell: same chemistry and voltage profile as
/// [`CR2032`] (larger capacity, lower internal resistance).
pub const CR2450: Curve = CR2032;

/// Two alkaline AA or AAA cells in series (1.55 V → 0.9 V per cell under a
/// light load; AA and AAA share the voltage profile).
pub const ALKALINE_2X: Curve = Curve {
    points: &[
        p(3_100, 100),
        p(2_900, 85),
        p(2_700, 65),
        p(2_500, 40),
        p(2_300, 20),
        p(2_100, 8),
        p(1_900, 2),
        p(1_800, 0),
    ],
};

/// Single Li-ion / LiPo cell (4.2 V full, rest voltage; 3.3 V empty).
pub const LI_ION_1S: Curve = Curve {
    points: &[
        p(4_200, 100),
        p(4_110, 90),
        p(4_020, 80),
        p(3_950, 70),
        p(3_870, 60),
        p(3_840, 50),
        p(3_800, 40),
        p(3_770, 30),
        p(3_730, 20),
        p(3_690, 10),
        p(3_610, 5),
        p(3_300, 0),
    ],
};

/// Single LiFePO₄ cell (rest voltage; very flat 3.2-3.35 V plateau, so the
/// mid-range estimate is coarse; 2.5 V empty).
pub const LIFEPO4_1S: Curve = Curve {
    points: &[
        p(3_500, 100),
        p(3_350, 90),
        p(3_320, 70),
        p(3_300, 50),
        p(3_270, 30),
        p(3_200, 15),
        p(3_000, 5),
        p(2_500, 0),
    ],
};

/// Two NiMH cells in series (1.35 V → 1.0 V per cell under a light load).
pub const NIMH_2X: Curve = Curve {
    points: &[
        p(2_700, 100),
        p(2_560, 90),
        p(2_500, 75),
        p(2_440, 50),
        p(2_400, 35),
        p(2_300, 20),
        p(2_200, 10),
        p(2_000, 0),
    ],
};

/// One battery reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Battery voltage in mV (after the divider, before compensation).
    pub voltage: i32,
    /// Remaining charge in 0.5 % units (0..=200).
    pub percent: u8,
}

impl Readings for Measurement {
    /// Voltage, then BatteryPercent.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Voltage(self.voltage));
        f(Quantity::BatteryPercent(self.percent));
    }
}

/// Convert an ADC pin voltage: divider, compensation, curve.
pub const fn evaluate(
    pin_mv: u32,
    divider: &Divider,
    curve: &Curve,
    compensation_mv: u16,
) -> Measurement {
    let mv = divider.battery_mv(pin_mv);
    Measurement {
        voltage: if mv > i32::MAX as u32 {
            i32::MAX
        } else {
            mv as i32
        },
        percent: curve.half_percent(mv.saturating_add(compensation_mv as u32)),
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<ADC> $name<ADC> {
            /// Battery on an ADC channel with a discharge curve; direct
            /// connection, no compensation.
            pub const fn new(adc: ADC, curve: Curve) -> Self {
                Self {
                    adc,
                    curve,
                    divider: Divider::NONE,
                    compensation_mv: 0,
                    settle_us: 0,
                }
            }

            /// Resistor divider in front of the ADC.
            pub const fn with_divider(mut self, divider: Divider) -> Self {
                self.divider = divider;
                self
            }

            /// Voltage added before the curve lookup (expected sag under
            /// the load present while sampling).
            pub const fn with_compensation_mv(mut self, mv: u16) -> Self {
                self.compensation_mv = mv;
                self
            }

            /// Wait returned by `start_measurement` (e.g. settling of a
            /// switched or high-impedance divider); default 0.
            pub const fn with_settle_us(mut self, us: WaitUs) -> Self {
                self.settle_us = us;
                self
            }

            /// Replace the discharge curve.
            pub fn set_curve(&mut self, curve: Curve) {
                self.curve = curve;
            }

            /// Active curve.
            pub const fn curve(&self) -> &Curve {
                &self.curve
            }

            /// Give the ADC channel back.
            pub fn release(self) -> ADC {
                self.adc
            }
        }
    };
}

/// Blocking battery monitor.
#[derive(Debug)]
pub struct Battery<ADC> {
    adc: ADC,
    curve: Curve,
    divider: Divider,
    compensation_mv: u16,
    settle_us: WaitUs,
}

constructors!(Battery);

impl<ADC: zigbee_sensor::adc::Millivolts> zigbee_sensor::blocking::Sensor for Battery<ADC> {
    type BusError = ADC::Error;
    type Measurement = Measurement;

    /// Returns the configured settle time (default 0).
    fn start_measurement(&mut self) -> Result<WaitUs, Error<ADC::Error>> {
        Ok(self.settle_us)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<ADC::Error>> {
        let pin = self.adc.read_mv().map_err(Error::bus)?;
        Ok(evaluate(
            pin,
            &self.divider,
            &self.curve,
            self.compensation_mv,
        ))
    }
}

/// Async battery monitor.
pub mod asynch {
    use super::{Curve, Divider, Measurement, evaluate};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Battery`].
    #[derive(Debug)]
    pub struct Battery<ADC> {
        adc: ADC,
        curve: Curve,
        divider: Divider,
        compensation_mv: u16,
        settle_us: WaitUs,
    }

    constructors!(Battery);

    impl<ADC: zigbee_sensor::adc::AsyncMillivolts> zigbee_sensor::asynch::Sensor for Battery<ADC> {
        type BusError = ADC::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<ADC::Error>> {
            Ok(self.settle_us)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<ADC::Error>> {
            let pin = self.adc.read_mv().await.map_err(Error::bus)?;
            Ok(evaluate(
                pin,
                &self.divider,
                &self.curve,
                self.compensation_mv,
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    struct FakeAdc(Result<u32, u8>);

    impl zigbee_sensor::adc::Millivolts for FakeAdc {
        type Error = u8;
        fn read_mv(&mut self) -> Result<u32, u8> {
            self.0
        }
    }

    impl zigbee_sensor::adc::AsyncMillivolts for FakeAdc {
        type Error = u8;
        async fn read_mv(&mut self) -> Result<u32, u8> {
            self.0
        }
    }

    #[test]
    fn presets_are_valid_and_span_full_range() {
        for curve in [CR2032, CR2450, ALKALINE_2X, LI_ION_1S, LIFEPO4_1S, NIMH_2X] {
            assert_eq!(Curve::new(curve.points()), Some(curve));
            assert_eq!(curve.half_percent(5_000), 200);
            assert_eq!(curve.half_percent(0), 0);
        }
        static UNORDERED: [Point; 2] = [p(3_000, 50), p(3_100, 0)];
        static RISING: [Point; 2] = [p(3_000, 50), p(2_900, 60)];
        static OVER: [Point; 1] = [Point {
            mv: 3_000,
            half_percent: 201,
        }];
        assert_eq!(Curve::new(&UNORDERED), None);
        assert_eq!(Curve::new(&RISING), None);
        assert_eq!(Curve::new(&OVER), None);
        assert_eq!(Curve::new(&[]), None);
    }

    #[test]
    fn interpolation() {
        // Exact points.
        assert_eq!(CR2032.half_percent(2_800), 140);
        assert_eq!(LI_ION_1S.half_percent(3_840), 100);
        // 2.75 V: halfway between 70 % and 40 % → 55 %.
        assert_eq!(CR2032.half_percent(2_750), 110);
        // 2.733 V: 40 % + 33/100 · 30 % = 49.9 % → 100 half-percent.
        assert_eq!(CR2032.half_percent(2_733), 100);
        assert_eq!(ALKALINE_2X.half_percent(2_400), 60);
        assert_eq!(NIMH_2X.half_percent(2_470), 125);
        assert_eq!(LIFEPO4_1S.half_percent(3_310), 120);
    }

    #[test]
    fn divider_compensation_and_errors() {
        assert_eq!(Divider::NONE.battery_mv(3_000), 3_000);
        assert_eq!(Divider::new(1_000_000, 0), Divider::NONE);
        // 1 MΩ / 330 kΩ: 1.0 V at the pin → 4.030 V.
        assert_eq!(Divider::new(1_000_000, 330_000).battery_mv(1_000), 4_030);
        assert_eq!(Divider::new(u32::MAX, 1).battery_mv(u32::MAX), u32::MAX);

        let mut battery = Battery::new(FakeAdc(Ok(2_700)), CR2032).with_settle_us(500);
        assert_eq!(battery.start_measurement().unwrap(), 500);
        let m = battery.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                voltage: 2_700,
                percent: 80
            }
        );
        let mut quantities = Vec::new();
        m.for_each(&mut |q| quantities.push(q));
        assert_eq!(
            quantities,
            [Quantity::Voltage(2_700), Quantity::BatteryPercent(80)]
        );

        // Sampled during a radio burst: +100 mV compensation.
        let mut battery = Battery::new(FakeAdc(Ok(2_700)), CR2032).with_compensation_mv(100);
        let m = battery.read_measurement().unwrap();
        assert_eq!((m.voltage, m.percent), (2_700, 140));

        battery.set_curve(LI_ION_1S);
        assert_eq!(battery.read_measurement().unwrap().percent, 0);
        let mut failing = Battery::new(FakeAdc(Err(7)), CR2032);
        assert_eq!(failing.read_measurement(), Err(Error::Bus(7)));
        assert!(battery.release().0.is_ok());
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

        // 2 × AA behind a 1:1 divider: 1.25 V at the pin → 2.5 V → 40 %.
        let mut battery = asynch::Battery::new(FakeAdc(Ok(1_250)), ALKALINE_2X)
            .with_divider(Divider::new(470_000, 470_000))
            .with_settle_us(1_000);
        let mut delay = RecordingDelay(0);
        let m = block_on(battery.measure(&mut delay)).unwrap();
        assert_eq!(
            m,
            Measurement {
                voltage: 2_500,
                percent: 80
            }
        );
        assert_eq!(delay.0, 1_000);
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
