//! Platinum RTD resistance → temperature (IEC 60751 Callendar-Van Dusen),
//! integer only.
//!
//! `R(T) = R0 · (1 + A·T + B·T²)` for T ≥ 0 °C and
//! `R(T) = R0 · (1 + A·T + B·T² + C·(T − 100)·T³)` for T < 0 °C with
//! A = 3.9083e-3, B = −5.775e-7, C = −4.183e-12.
//!
//! * T ≥ 0: the quadratic is inverted exactly with an integer square root;
//!   error < 0.001 °C from 0 to 850 °C.
//! * T < 0: no closed form; linear interpolation in a 5 °C table of
//!   R/R0 generated from the equation; error < 0.003 °C from −200 to 0 °C.
//!   The table extends to −210 °C so readings slightly below the IEC range
//!   still convert.

/// R/R0 in ppm at −210, −205, …, 0 °C.
const NEGATIVE: [u32; 43] = [
    141_780, 163_538, 185_201, 206_772, 228_255, 249_651, 270_964, 292_197, 313_350, 334_429,
    355_433, 376_367, 397_232, 418_030, 438_764, 459_436, 480_048, 500_601, 521_098, 541_540,
    561_930, 582_269, 602_558, 622_800, 642_996, 663_147, 683_254, 703_320, 723_345, 743_331,
    763_278, 783_189, 803_063, 822_902, 842_707, 862_478, 882_217, 901_923, 921_599, 941_244,
    960_859, 980_444, 1_000_000,
];
const NEGATIVE_START_CENTI: i32 = -21_000;
const STEP_CENTI: i32 = 500;

/// Floor of the square root.
pub(crate) const fn isqrt(n: u64) -> u64 {
    let mut rem = n;
    let mut root = 0u64;
    let mut bit = 1u64 << 62;
    while bit > n {
        bit >>= 2;
    }
    while bit != 0 {
        if rem >= root + bit {
            rem -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    root
}

/// Temperature in 0.01 °C for a resistance ratio R/R0 in ppm (1 000 000 =
/// 0 °C). `None` below −210 °C or beyond the top of the quadratic.
pub const fn temperature_from_ratio(ratio_ppm: u32) -> Option<i32> {
    if ratio_ppm >= 1_000_000 {
        // T = (A − √(A² + 4B·q)) / (2|B|) with q = R/R0 − 1, scaled by 1e9:
        // A·1e9 = 3_908_300, A²·1e18 = 15_274_808_890_000,
        // 4|B|·1e12 = 2_310_000, 1 / (2|B|·1e9) °C → ×100/1155 centi.
        let q = (ratio_ppm - 1_000_000) as u64;
        let d = 15_274_808_890_000u64;
        let sub = 2_310_000 * q;
        if sub > d {
            return None;
        }
        let s = isqrt(d - sub);
        let num = (3_908_300 - s) * 20;
        Some(((num + 115) / 231) as i32)
    } else {
        if ratio_ppm < NEGATIVE[0] {
            return None;
        }
        let mut i = 0;
        while NEGATIVE[i + 1] <= ratio_ppm {
            i += 1;
        }
        let lo = NEGATIVE[i];
        let span = (NEGATIVE[i + 1] - lo) as i32;
        let frac = (ratio_ppm - lo) as i32 * STEP_CENTI;
        Some(NEGATIVE_START_CENTI + i as i32 * STEP_CENTI + (frac + span / 2) / span)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn isqrt_exact() {
        for n in [
            0u64,
            1,
            2,
            3,
            4,
            15,
            16,
            17,
            1 << 40,
            15_274_808_890_000,
            u64::MAX,
        ] {
            let r = isqrt(n);
            assert!(r * r <= n);
            assert!((r + 1).checked_mul(r + 1).is_none_or(|sq| sq > n), "{n}");
        }
    }

    #[test]
    fn iec_60751_pt100_table() {
        // IEC 60751 PT100 reference values (Ω, rounded to 0.01 Ω, i.e.
        // ±0.013 °C), as R/R0 ppm = R · 10_000.
        for (ohm_centi, centi) in [
            (1_852u32, -20_000),
            (6_026, -10_000),
            (8_031, -5_000),
            (9_609, -1_000),
            (10_000, 0),
            (10_390, 1_000),
            (13_851, 10_000),
            (17_586, 20_000),
            (24_709, 40_000),
            (31_371, 60_000),
            (39_048, 85_000),
        ] {
            let t = temperature_from_ratio(ohm_centi * 100).unwrap();
            assert!((t - centi).abs() <= 2, "{ohm_centi}: {t} vs {centi}");
        }
    }

    #[test]
    fn exact_equation_points() {
        // Ratios computed from the CVD equation itself.
        assert_eq!(temperature_from_ratio(1_000_000), Some(0));
        assert_eq!(temperature_from_ratio(1_385_055), Some(10_000));
        assert_eq!(temperature_from_ratio(3_904_811), Some(85_000));
        assert_eq!(temperature_from_ratio(185_201), Some(-20_000));
        assert_eq!(temperature_from_ratio(602_558), Some(-10_000));
        // −37.5 °C: 852_596 ppm.
        assert_eq!(temperature_from_ratio(852_596), Some(-3_750));
        assert_eq!(temperature_from_ratio(141_779), None);
        assert_eq!(temperature_from_ratio(u32::MAX), None);
    }
}
