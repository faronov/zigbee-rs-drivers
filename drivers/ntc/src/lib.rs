//! NTC thermistor read through an ADC voltage divider.
//!
//! The thermistor and a fixed series resistor form a divider; the ADC
//! channel (any [`Millivolts`] / [`AsyncMillivolts`] implementation)
//! samples the middle node. The driver converts the voltage to resistance
//! and then to temperature with one of three integer-only models:
//!
//! * [`Model::Beta`]: `1/T = 1/T0 + ln(R/R0)/β` (datasheet R25 and β).
//! * [`Model::SteinhartHart`]: `1/T = A + B·ln R + C·(ln R)³`.
//! * [`Model::Table`]: a user `(Ω, 0.01 °C)` table with linear
//!   interpolation (e.g. the manufacturer R/T table).
//!
//! The logarithm is a fixed-point [`ln_q16`]; no floating point is used.
//!
//! # Supply
//!
//! `Divider::supply_mv` is the divider supply. For a **ratiometric** setup
//! (divider powered from the ADC reference) set it to the voltage the ADC
//! reports at full scale, so reference drift cancels. If the divider runs
//! from a battery rail measured on another channel, call
//! [`Ntc::measure_with_supply`] with that reading.
//!
//! # Power
//!
//! The divider draws `Vs / (Rs + R)` continuously (≈ 165 µA at 3.3 V and
//! 25 °C with 10 kΩ / 10 kΩ). Battery devices should feed it from a GPIO
//! and switch it on just before the reading (allow for the ADC input
//! filter's RC settling); the driver does not touch any pin.
//! [`Sensor::start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! returns 0 because the ADC read is immediate.

#![no_std]

mod math;

pub use math::ln_q16;
use zigbee_sensor::adc::{AsyncMillivolts, Millivolts};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Where the thermistor sits in the divider.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Topology {
    /// Series resistor from supply to the ADC node, NTC from the node to
    /// ground: the voltage falls as temperature rises.
    NtcToGround,
    /// NTC from supply to the ADC node, series resistor to ground: the
    /// voltage rises with temperature.
    NtcToSupply,
}

/// Divider wiring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Divider {
    /// Thermistor position.
    pub topology: Topology,
    /// Series (fixed) resistor in Ω.
    pub series_ohm: u32,
    /// Divider supply in mV, or the ADC full-scale reading when
    /// ratiometric.
    pub supply_mv: u32,
}

impl Divider {
    /// Thermistor resistance in mΩ for a node voltage `mv` and supply
    /// `supply_mv`. `None` when the node sits at a rail (open or shorted
    /// thermistor).
    pub fn resistance_mohm(&self, mv: u32, supply_mv: u32) -> Option<u64> {
        if mv == 0 || mv >= supply_mv {
            return None;
        }
        let rs = i128::from(self.series_ohm) * 1000;
        let (num, den) = match self.topology {
            Topology::NtcToGround => (rs * i128::from(mv), supply_mv - mv),
            Topology::NtcToSupply => (rs * i128::from(supply_mv - mv), mv),
        };
        u64::try_from(math::div_round(num, i128::from(den))).ok()
    }
}

/// Resistance-to-temperature model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Model {
    /// Beta (B-parameter) equation.
    Beta {
        /// Nominal resistance R0 in Ω at `t0_centi`.
        r0_ohm: u32,
        /// Reference temperature in 0.01 °C (usually 2500).
        t0_centi: i32,
        /// β in K (e.g. 3950 for B25/50 = 3950).
        beta: u32,
    },
    /// Steinhart-Hart equation with R in Ω. Coefficients are scaled by
    /// 1e15: A = 1.009249522e-3 is written `1_009_249_522_000`.
    SteinhartHart {
        /// A × 1e15.
        a: i64,
        /// B × 1e15.
        b: i64,
        /// C × 1e15.
        c: i64,
    },
    /// `(resistance Ω, temperature 0.01 °C)` points sorted by resistance
    /// (either direction), interpolated linearly in resistance. Readings
    /// outside the table are [`Error::InvalidData`]. Use a dense table
    /// (≤ 5 °C steps) because R(T) is strongly non-linear.
    Table(&'static [(u32, i32)]),
}

impl Model {
    /// Common 10 kΩ at 25 °C, β = 3950 thermistor ("10k B3950").
    pub const B3950_10K: Model = Model::Beta {
        r0_ohm: 10_000,
        t0_centi: 2_500,
        beta: 3_950,
    };

    /// Temperature in 0.01 °C for a resistance in mΩ; `None` if out of the
    /// model's range.
    pub fn temperature(&self, r_mohm: u64) -> Option<i32> {
        match *self {
            Model::Beta {
                r0_ohm,
                t0_centi,
                beta,
            } => math::beta(r_mohm, r0_ohm, t0_centi, beta),
            Model::SteinhartHart { a, b, c } => math::steinhart_hart(r_mohm, a, b, c),
            Model::Table(t) => math::table(r_mohm, t),
        }
    }
}

/// One reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Thermistor resistance in mΩ.
    pub resistance_mohm: u64,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
    }
}

/// Convert a node voltage: rail voltages or out-of-model resistances are
/// [`Error::InvalidData`].
pub fn convert<E>(
    divider: &Divider,
    model: &Model,
    mv: u32,
    supply_mv: u32,
) -> Result<Measurement, Error<E>> {
    let resistance_mohm = divider
        .resistance_mohm(mv, supply_mv)
        .ok_or(Error::InvalidData)?;
    let temperature = model
        .temperature(resistance_mohm)
        .ok_or(Error::InvalidData)?;
    Ok(Measurement {
        temperature,
        resistance_mohm,
    })
}

fn average(sum: u64, n: u8) -> u32 {
    let n = u64::from(n);
    ((sum + n / 2) / n) as u32
}

macro_rules! constructors {
    () => {
        /// Create a driver; no ADC access.
        pub const fn new(adc: ADC, divider: Divider, model: Model) -> Self {
            Self {
                adc,
                divider,
                model,
                samples: 1,
            }
        }

        /// Average `n` ADC samples per reading (0 is treated as 1).
        pub const fn with_samples(mut self, n: u8) -> Self {
            self.samples = if n == 0 { 1 } else { n };
            self
        }

        /// Divider configuration.
        pub const fn divider(&self) -> &Divider {
            &self.divider
        }

        /// Conversion model.
        pub const fn model(&self) -> &Model {
            &self.model
        }

        /// Give the ADC channel back.
        pub fn release(self) -> ADC {
            self.adc
        }
    };
}

/// Blocking driver.
pub struct Ntc<ADC> {
    adc: ADC,
    divider: Divider,
    model: Model,
    samples: u8,
}

impl<ADC> Ntc<ADC> {
    constructors!();
}

impl<ADC: Millivolts> Ntc<ADC> {
    /// Averaged node voltage in mV.
    pub fn read_mv(&mut self) -> Result<u32, Error<ADC::Error>> {
        let mut sum = 0u64;
        for _ in 0..self.samples {
            sum += u64::from(self.adc.read_mv().map_err(Error::bus)?);
        }
        Ok(average(sum, self.samples))
    }

    /// Measure with an explicitly supplied divider voltage (e.g. a battery
    /// rail read on another channel).
    pub fn measure_with_supply(
        &mut self,
        supply_mv: u32,
    ) -> Result<Measurement, Error<ADC::Error>> {
        let mv = self.read_mv()?;
        convert(&self.divider, &self.model, mv, supply_mv)
    }
}

impl<ADC: Millivolts> zigbee_sensor::blocking::Sensor for Ntc<ADC> {
    type BusError = ADC::Error;
    type Measurement = Measurement;

    /// Nothing to start; returns 0.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<ADC::Error>> {
        Ok(0)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<ADC::Error>> {
        self.measure_with_supply(self.divider.supply_mv)
    }
}

/// Async driver.
pub mod asynch {
    use super::{AsyncMillivolts, Divider, Measurement, Model, average, convert};
    use zigbee_sensor::{Error, WaitUs};

    /// Async driver; same API as [`super::Ntc`].
    pub struct Ntc<ADC> {
        adc: ADC,
        divider: Divider,
        model: Model,
        samples: u8,
    }

    impl<ADC> Ntc<ADC> {
        constructors!();
    }

    impl<ADC: AsyncMillivolts> Ntc<ADC> {
        /// See [`super::Ntc::read_mv`].
        pub async fn read_mv(&mut self) -> Result<u32, Error<ADC::Error>> {
            let mut sum = 0u64;
            for _ in 0..self.samples {
                sum += u64::from(self.adc.read_mv().await.map_err(Error::bus)?);
            }
            Ok(average(sum, self.samples))
        }

        /// See [`super::Ntc::measure_with_supply`].
        pub async fn measure_with_supply(
            &mut self,
            supply_mv: u32,
        ) -> Result<Measurement, Error<ADC::Error>> {
            let mv = self.read_mv().await?;
            convert(&self.divider, &self.model, mv, supply_mv)
        }
    }

    impl<ADC: AsyncMillivolts> zigbee_sensor::asynch::Sensor for Ntc<ADC> {
        type BusError = ADC::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<ADC::Error>> {
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<ADC::Error>> {
            self.measure_with_supply(self.divider.supply_mv).await
        }
    }
}

#[cfg(test)]
mod tests;
