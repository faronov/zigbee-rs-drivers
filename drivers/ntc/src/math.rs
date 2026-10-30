//! Integer math: natural logarithm and the thermistor models.

/// ln 2 in Q32.
const LN2_Q32: u64 = 2_977_044_472;
/// 0 °C in 0.01 K.
const ZERO_C_CENTI_K: i128 = 27_315;

/// Natural logarithm of `x` in Q16 fixed point (`ln(x) · 65536`), `None`
/// for `x = 0`.
///
/// `log2` is computed bit by bit by repeatedly squaring the normalised
/// mantissa (20 fractional bits), then scaled by ln 2. Absolute error is
/// below 2 / 65536 over the whole `u64` range.
pub fn ln_q16(x: u64) -> Option<i64> {
    if x == 0 {
        return None;
    }
    let n = 63 - x.leading_zeros();
    // Mantissa in [1, 2) as Q30.
    let mut m = if n >= 30 {
        x >> (n - 30)
    } else {
        x << (30 - n)
    };
    let mut frac = 0u64;
    for bit in (0..20).rev() {
        m = (m * m) >> 30;
        if m >= 1 << 31 {
            m >>= 1;
            frac |= 1 << bit;
        }
    }
    let log2_q20 = (u64::from(n) << 20) | frac;
    Some(((log2_q20 * LN2_Q32 + (1 << 35)) >> 36) as i64)
}

/// `n / d` rounded half away from zero; `d != 0`.
pub(crate) fn div_round(n: i128, d: i128) -> i128 {
    let q = n / d;
    let r = n % d;
    if 2 * r.abs() >= d.abs() {
        if (n < 0) != (d < 0) { q - 1 } else { q + 1 }
    } else {
        q
    }
}

fn centi_k_to_centi_c(t: i128) -> Option<i32> {
    i32::try_from(t - ZERO_C_CENTI_K).ok()
}

/// Beta model: `1/T = 1/T0 + ln(R/R0)/B`.
pub(crate) fn beta(r_mohm: u64, r0_ohm: u32, t0_centi: i32, beta: u32) -> Option<i32> {
    let l = i128::from(ln_q16(r_mohm)? - ln_q16(u64::from(r0_ohm) * 1000)?);
    let t0 = i128::from(t0_centi) + ZERO_C_CENTI_K;
    if t0 <= 0 || beta == 0 {
        return None;
    }
    // T [0.01 K] = T0·B·655.36e4 / (B·655.36e4 + T0·L), with T0 in 0.01 K
    // and L in Q16.
    let b = i128::from(beta) * 6_553_600;
    let den = b + t0 * l;
    if den <= 0 {
        return None;
    }
    centi_k_to_centi_c(div_round(t0 * b, den))
}

/// Steinhart-Hart: `1/T = A + B·ln R + C·(ln R)³`, coefficients in
/// 1e-15 units, R in Ω.
pub(crate) fn steinhart_hart(r_mohm: u64, a: i64, b: i64, c: i64) -> Option<i32> {
    let l = i128::from(ln_q16(r_mohm)? - ln_q16(1000)?);
    // 1/T in 1e-15 K⁻¹, Q48.
    let inv = (i128::from(a) << 48) + ((i128::from(b) * l) << 32) + i128::from(c) * l * l * l;
    if inv <= 0 {
        return None;
    }
    centi_k_to_centi_c(div_round(100_000_000_000_000_000 << 48, inv))
}

/// Linear interpolation in a `(Ω, 0.01 °C)` table sorted by resistance
/// (ascending or descending).
pub(crate) fn table(r_mohm: u64, table: &[(u32, i32)]) -> Option<i32> {
    let r = i128::from(r_mohm);
    for w in table.windows(2) {
        let (r1, t1) = (i128::from(w[0].0) * 1000, i128::from(w[0].1));
        let (r2, t2) = (i128::from(w[1].0) * 1000, i128::from(w[1].1));
        if r >= r1.min(r2) && r <= r1.max(r2) {
            if r1 == r2 {
                return i32::try_from(t1).ok();
            }
            return i32::try_from(t1 + div_round((t2 - t1) * (r - r1), r2 - r1)).ok();
        }
    }
    None
}
