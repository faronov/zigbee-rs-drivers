//! Capacitive soil-moisture probes with an analog output ("Capacitive Soil
//! Moisture Sensor v1.2/v2.0" TLC555/NE555 boards, DFRobot SEN0193 and
//! similar).
//!
//! The probe's output voltage falls as the soil gets wetter. The driver
//! reads it through [`zigbee_sensor::adc::Millivolts`] and maps it linearly
//! between two calibration points (probe in air / dry soil and probe in
//! water) to `Quantity::SoilMoisture` in 0.01 %.
//!
//! These boards draw ~5 mA continuously. On battery nodes, supply the
//! probe from a GPIO or a load switch: `start_measurement` enables the
//! supply and returns the settling time, `read_measurement` samples and
//! switches it off again.
//!
//! No vendor datasheet defines the curve; calibrate every probe in its
//! soil. The default calibration (dry 2 800 mV, wet 1 200 mV) is a typical
//! starting point for v1.2 boards at 3.3 V.

#![no_std]

use core::convert::Infallible;
use embedded_hal::digital::OutputPin;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Default supply settling time when a power pin is used: the 555
/// oscillator starts at once, but the output RC filter needs ~ 5 τ.
pub const DEFAULT_SETTLE_US: WaitUs = 100_000;

/// Two-point calibration in mV. `dry_mv` may be above or below `wet_mv`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// Output in air or completely dry soil (0 %).
    pub dry_mv: u32,
    /// Output in water or saturated soil (100 %).
    pub wet_mv: u32,
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            dry_mv: 2_800,
            wet_mv: 1_200,
        }
    }
}

impl Calibration {
    /// Map a probe voltage to moisture in 0.01 % (0..=10000, clamped).
    /// Returns 0 if both points are equal.
    pub fn moisture(&self, mv: u32) -> u16 {
        let (span, pos) = if self.dry_mv >= self.wet_mv {
            (self.dry_mv - self.wet_mv, self.dry_mv.saturating_sub(mv))
        } else {
            (self.wet_mv - self.dry_mv, mv.saturating_sub(self.dry_mv))
        };
        if span == 0 {
            return 0;
        }
        let (mut span, mut pos) = (span, pos.min(span));
        // Keep pos × 10000 within u32 (only matters above ~400 V).
        while span > 400_000 {
            span >>= 1;
            pos >>= 1;
        }
        ((pos * 10_000 + span / 2) / span) as u16
    }
}

/// Error of either the ADC or the power pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusError<A, P> {
    /// ADC read failed.
    Adc(A),
    /// Power pin could not be switched.
    Pin(P),
}

/// Placeholder for "probe permanently powered".
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NoPin;

impl embedded_hal::digital::ErrorType for NoPin {
    type Error = Infallible;
}

impl OutputPin for NoPin {
    fn set_low(&mut self) -> Result<(), Infallible> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Infallible> {
        Ok(())
    }
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Probe output in mV.
    pub millivolts: u32,
    /// Soil moisture in 0.01 %.
    pub moisture: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::SoilMoisture(self.moisture));
    }
}

macro_rules! common {
    ($name:ident) => {
        impl<A> $name<A, NoPin> {
            /// Probe permanently powered; no hardware access.
            pub const fn new(adc: A, calibration: Calibration) -> Self {
                Self {
                    adc,
                    power: NoPin,
                    settle_us: 0,
                    calibration,
                }
            }

            /// Switch the probe supply with `pin` (high = on) and wait
            /// `settle_us` after switching it on.
            pub fn with_power_pin<P: OutputPin>(self, pin: P, settle_us: WaitUs) -> $name<A, P> {
                $name {
                    adc: self.adc,
                    power: pin,
                    settle_us,
                    calibration: self.calibration,
                }
            }
        }

        impl<A, P> $name<A, P> {
            /// Replace the calibration.
            pub fn set_calibration(&mut self, calibration: Calibration) {
                self.calibration = calibration;
            }

            /// Give the ADC channel and power pin back.
            pub fn release(self) -> (A, P) {
                (self.adc, self.power)
            }
        }

        impl<A, P: OutputPin> $name<A, P> {
            fn power_on<AE>(&mut self) -> Result<WaitUs, Error<BusError<AE, P::Error>>> {
                self.power
                    .set_high()
                    .map_err(|e| Error::Bus(BusError::Pin(e)))?;
                Ok(self.settle_us)
            }

            fn finish<AE>(
                &mut self,
                sample: Result<u32, AE>,
            ) -> Result<Measurement, Error<BusError<AE, P::Error>>> {
                let off = self.power.set_low();
                let millivolts = sample.map_err(|e| Error::Bus(BusError::Adc(e)))?;
                off.map_err(|e| Error::Bus(BusError::Pin(e)))?;
                Ok(Measurement {
                    millivolts,
                    moisture: self.calibration.moisture(millivolts),
                })
            }
        }
    };
}

/// Blocking driver over [`zigbee_sensor::adc::Millivolts`].
pub struct SoilProbe<A, P> {
    adc: A,
    power: P,
    settle_us: WaitUs,
    calibration: Calibration,
}

common!(SoilProbe);

impl<A, P> zigbee_sensor::blocking::Sensor for SoilProbe<A, P>
where
    A: zigbee_sensor::adc::Millivolts,
    P: OutputPin,
{
    type BusError = BusError<A::Error, P::Error>;
    type Measurement = Measurement;

    /// Power the probe; returns the settling time (0 without power pin).
    fn start_measurement(&mut self) -> Result<WaitUs, Error<Self::BusError>> {
        self.power_on()
    }

    /// Sample, switch the probe off and convert.
    fn read_measurement(&mut self) -> Result<Measurement, Error<Self::BusError>> {
        let sample = self.adc.read_mv();
        self.finish(sample)
    }
}

/// Async driver over [`zigbee_sensor::adc::AsyncMillivolts`].
pub mod asynch {
    use super::{BusError, Calibration, Measurement, NoPin};
    use embedded_hal::digital::OutputPin;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::SoilProbe`].
    pub struct SoilProbe<A, P> {
        adc: A,
        power: P,
        settle_us: WaitUs,
        calibration: Calibration,
    }

    common!(SoilProbe);

    impl<A, P> zigbee_sensor::asynch::Sensor for SoilProbe<A, P>
    where
        A: zigbee_sensor::adc::AsyncMillivolts,
        P: OutputPin,
    {
        type BusError = BusError<A::Error, P::Error>;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<Self::BusError>> {
            self.power_on()
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<Self::BusError>> {
            let sample = self.adc.read_mv().await;
            self.finish(sample)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::digital::{Mock, State, Transaction};
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
    fn linear_mapping_and_clamping() {
        let cal = Calibration {
            dry_mv: 2_800,
            wet_mv: 1_200,
        };
        assert_eq!(cal.moisture(2_800), 0);
        assert_eq!(cal.moisture(3_100), 0);
        assert_eq!(cal.moisture(1_200), 10_000);
        assert_eq!(cal.moisture(900), 10_000);
        assert_eq!(cal.moisture(2_000), 5_000);
        assert_eq!(cal.moisture(2_401), 2_494);
        // Inverted probe (voltage rises with moisture).
        let inv = Calibration {
            dry_mv: 500,
            wet_mv: 1_500,
        };
        assert_eq!(inv.moisture(750), 2_500);
        assert_eq!(
            Calibration {
                dry_mv: 1,
                wet_mv: 1
            }
            .moisture(1),
            0
        );
    }

    #[test]
    fn unpowered_probe() {
        let mut p = SoilProbe::new(FakeAdc(Ok(2_000)), Calibration::default());
        assert_eq!(p.start_measurement(), Ok(0));
        assert_eq!(
            p.read_measurement().unwrap(),
            Measurement {
                millivolts: 2_000,
                moisture: 5_000
            }
        );
    }

    #[test]
    fn power_pin_switched_around_sample() {
        let mut pin = Mock::new(&[Transaction::set(State::High), Transaction::set(State::Low)]);
        let mut p = SoilProbe::new(FakeAdc(Ok(1_200)), Calibration::default())
            .with_power_pin(pin.clone(), 20_000);
        assert_eq!(p.start_measurement(), Ok(20_000));
        assert_eq!(p.read_measurement().unwrap().moisture, 10_000);
        pin.done();
    }

    #[test]
    fn adc_error_still_powers_off() {
        let mut pin = Mock::new(&[Transaction::set(State::High), Transaction::set(State::Low)]);
        let mut p = SoilProbe::new(FakeAdc(Err(7)), Calibration::default())
            .with_power_pin(pin.clone(), DEFAULT_SETTLE_US);
        assert_eq!(p.start_measurement(), Ok(DEFAULT_SETTLE_US));
        assert_eq!(p.read_measurement(), Err(Error::Bus(BusError::Adc(7))));
        pin.done();
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::Sensor as _;
        struct Delay(u32);
        impl embedded_hal_async::delay::DelayNs for Delay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }
        let mut pin = Mock::new(&[Transaction::set(State::High), Transaction::set(State::Low)]);
        let mut p = asynch::SoilProbe::new(FakeAdc(Ok(2_400)), Calibration::default())
            .with_power_pin(pin.clone(), 30_000);
        let mut delay = Delay(0);
        let m = block_on(p.measure(&mut delay)).unwrap();
        assert_eq!(m.moisture, 2_500);
        assert_eq!(delay.0, 30_000);
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
