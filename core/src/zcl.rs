//! Conversions from [`crate::Quantity`] units to ZCL attribute encodings.

/// Temperature Measurement (0x0402) MeasuredValue invalid marker.
pub const TEMPERATURE_INVALID: i16 = i16::MIN;
/// Relative Humidity (0x0405) MeasuredValue invalid marker.
pub const HUMIDITY_INVALID: u16 = 0xFFFF;
/// Illuminance (0x0400) MeasuredValue meaning "too low to be measured".
pub const ILLUMINANCE_TOO_LOW: u16 = 0;

/// Temperature Measurement MeasuredValue (int16, 0.01 °C). Values outside
/// the encodable range -273.15..=327.67 °C saturate.
pub fn temperature(centi_celsius: i32) -> i16 {
    centi_celsius.clamp(-27_315, i32::from(i16::MAX)) as i16
}

/// Relative Humidity MeasuredValue (uint16, 0.01 %RH), clamped to 100 %.
pub fn humidity(centi_percent: u16) -> u16 {
    centi_percent.min(10_000)
}

/// Pressure Measurement MeasuredValue (int16, 0.1 kPa), rounded.
pub fn pressure(pascal: u32) -> i16 {
    ((pascal + 50) / 100).min(i16::MAX as u32) as i16
}

/// Illuminance Measurement MeasuredValue, `10000 × log10(lux) + 1`,
/// rounded to the nearest step, computed without floating point.
///
/// Zero encodes as [`ILLUMINANCE_TOO_LOW`]; anything above zero but below
/// 1 lx clamps to 1, values above the range clamp to 0xFFFE.
pub fn illuminance(milli_lux: u32) -> u16 {
    if milli_lux == 0 {
        return ILLUMINANCE_TOO_LOW;
    }
    if milli_lux < 1000 {
        return 1;
    }

    // log2(milli_lux) in Q16: integer part from the MSB position, fraction
    // from 16 rounds of squaring the normalised mantissa (Q1.31 in [1, 2)).
    let integer = 31 - milli_lux.leading_zeros();
    let mut mantissa = (u64::from(milli_lux) << 31) >> integer;
    let mut fraction = 0u32;
    for bit in (0..16).rev() {
        mantissa = (mantissa * mantissa) >> 31;
        if mantissa >= 1 << 32 {
            fraction |= 1 << bit;
            mantissa >>= 1;
        }
    }
    let log2_q16 = u64::from((integer << 16) | fraction);

    // 10000 × log10(x) = log2(x) × 3010.299957.
    const DENOMINATOR: u64 = 65_536 * 1_000_000;
    let scaled = (log2_q16 * 3_010_299_957 + DENOMINATOR / 2) / DENOMINATOR;
    // lux = milli_lux / 1000 subtracts 30000; the encoding adds 1.
    scaled.saturating_sub(29_999).clamp(1, 0xFFFE) as u16
}

/// Soil Moisture / Leaf Wetness MeasuredValue (uint16, 0.01 %).
pub fn soil_moisture(centi_percent: u16) -> u16 {
    centi_percent.min(10_000)
}

/// Power Configuration BatteryVoltage (uint8, 100 mV), rounded.
pub fn battery_voltage(millivolts: u32) -> u8 {
    ((millivolts + 50) / 100).min(0xFE) as u8
}

/// Power Configuration BatteryPercentageRemaining (uint8, 0.5 %).
pub fn battery_percentage(half_percent: u8) -> u8 {
    half_percent.min(200)
}

/// Concentration clusters (CO2 0x040D, PM2.5 0x042A, formaldehyde 0x042B)
/// use single-precision floats; these are the only float helpers.
#[allow(clippy::float_arithmetic)]
pub mod concentration {
    /// CO2 / formaldehyde MeasuredValue: fraction of 1 (ppm × 1e-6).
    pub fn ppm_fraction(ppm: u32) -> f32 {
        ppm as f32 * 1e-6
    }

    /// Fraction of 1 from parts per billion (ppb × 1e-9).
    pub fn ppb_fraction(ppb: u32) -> f32 {
        ppb as f32 * 1e-9
    }

    /// PM2.5 MeasuredValue in µg/m³ from 0.1 µg/m³ units.
    pub fn micrograms(deci_ug_m3: u32) -> f32 {
        deci_ug_m3 as f32 / 10.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn illuminance_matches_log10() {
        assert_eq!(illuminance(0), 0);
        assert_eq!(illuminance(500), 1);
        assert_eq!(illuminance(1_000), 1);
        assert_eq!(illuminance(10_000), 10_001);
        assert_eq!(illuminance(1_000_000), 30_001);
        assert_eq!(illuminance(100_000_000), 50_001);
        assert_eq!(illuminance(u32::MAX), 0xFFFE);
    }

    #[test]
    fn scalar_encodings() {
        assert_eq!(temperature(2_345), 2_345);
        assert_eq!(temperature(-40_000), -27_315);
        assert_eq!(temperature(100_000), i16::MAX);
        assert_eq!(humidity(12_000), 10_000);
        assert_eq!(pressure(101_325), 1_013);
        assert_eq!(battery_voltage(3_049), 30);
        assert_eq!(battery_voltage(3_050), 31);
    }
}
