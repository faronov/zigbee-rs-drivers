//! Physical quantities reported by drivers, in integer units.

/// Three-axis value (accelerometer, gyroscope, magnetometer).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Vector3 {
    /// X axis.
    pub x: i32,
    /// Y axis.
    pub y: i32,
    /// Z axis.
    pub z: i32,
}

/// One measured value with its unit encoded in the variant.
///
/// Units are chosen so that every useful range fits an integer without
/// floating point and maps directly onto the ZCL attribute (see
/// [`crate::zcl`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Quantity {
    /// Temperature in 0.01 °C (ZCL 0x0402 MeasuredValue).
    Temperature(i32),
    /// Remote (object) temperature seen by an IR thermometer, in 0.01 °C.
    /// `Temperature` from the same part is its die/ambient temperature.
    ObjectTemperature(i32),
    /// Relative humidity in 0.01 %RH (ZCL 0x0405).
    Humidity(u16),
    /// Absolute pressure in Pa (ZCL 0x0403).
    Pressure(u32),
    /// Illuminance in milli-lux (ZCL 0x0400).
    Illuminance(u32),
    /// UV index in 0.01 units.
    UvIndex(u16),
    /// Carbon dioxide in ppm (ZCL 0x040D).
    Co2(u32),
    /// Total volatile organic compounds in ppb.
    Tvoc(u32),
    /// Equivalent CO2 estimated from VOCs, in ppm.
    Eco2(u32),
    /// Sensirion/ScioSense VOC index (1..=500).
    VocIndex(u16),
    /// Sensirion NOx index (1..=500).
    NoxIndex(u16),
    /// Device air-quality index (e.g. UBA AQI 1..=5).
    AirQualityIndex(u16),
    /// Relative air-quality index against the device's own baseline
    /// (e.g. ScioSense AQI-S, 0..=500, 100 = average).
    RelativeAirQualityIndex(u16),
    /// Formaldehyde in ppb (ZCL 0x042B).
    Formaldehyde(u32),
    /// PM1.0 mass concentration in 0.1 µg/m³.
    Pm1(u32),
    /// PM2.5 mass concentration in 0.1 µg/m³ (ZCL 0x042A).
    Pm25(u32),
    /// PM4.0 mass concentration in 0.1 µg/m³.
    Pm4(u32),
    /// PM10 mass concentration in 0.1 µg/m³.
    Pm10(u32),
    /// Number concentration of particles up to `size_nm` (Sensirion NC
    /// bins), in 0.01 #/cm³ (= particles per 0.1 L).
    ParticlesBelow {
        /// Upper particle diameter bound in nm (e.g. 2500 for NC2.5).
        size_nm: u16,
        /// Concentration in 0.01 #/cm³.
        count: u32,
    },
    /// Number concentration of particles at or above `size_nm`
    /// (Plantower/Honeywell ">0.3 µm" style bins), in 0.01 #/cm³
    /// (= particles per 0.1 L).
    ParticlesAbove {
        /// Lower particle diameter bound in nm (e.g. 300 for >0.3 µm).
        size_nm: u16,
        /// Concentration in 0.01 #/cm³.
        count: u32,
    },
    /// Typical (number-weighted) particle size in nm.
    TypicalParticleSize(u32),
    /// Gas-sensor heater resistance in Ω (raw BME680-style output).
    GasResistance(u32),
    /// Voltage in mV.
    Voltage(i32),
    /// Current in µA.
    Current(i32),
    /// Active power in mW.
    Power(i32),
    /// Accumulated energy in mWh.
    Energy(u64),
    /// Power factor in 0.001 units (-1000..=1000).
    PowerFactor(i16),
    /// AC frequency in mHz.
    Frequency(u32),
    /// Battery state of charge in 0.5 % units, 0..=200 (ZCL 0x0001 0x0021).
    BatteryPercent(u8),
    /// Distance in mm.
    Distance(u32),
    /// Presence/occupancy detected (ZCL 0x0406).
    Occupancy(bool),
    /// Mass in mg.
    Mass(i32),
    /// Soil moisture / water content in 0.01 % (ZCL 0x0408).
    SoilMoisture(u16),
    /// Acceleration in milli-g.
    Acceleration(Vector3),
    /// Angular rate in milli-degrees per second.
    AngularRate(Vector3),
    /// Magnetic flux density in nT.
    MagneticField(Vector3),
    /// Single-axis magnetic flux density in µT (hall sensors).
    MagneticFlux(i32),
    /// Unscaled proximity/reflectance counts.
    Proximity(u32),
}

/// A decoded measurement that can enumerate its quantities.
///
/// Lets generic application code publish any driver's output to ZCL
/// attributes without knowing the concrete measurement type.
pub trait Readings {
    /// Call `f` once for every quantity contained in this measurement.
    fn for_each(&self, f: &mut dyn FnMut(Quantity));
}

impl Readings for Quantity {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(*self);
    }
}
