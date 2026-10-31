//! Offset, sensitivity, temperature and cross-axis compensation of the raw
//! BMM350 output with the factory OTP coefficients.
//!
//! Integer port of `update_mag_off_sens`, `read_out_raw_data` and
//! `bmm350_get_compensated_mag_xyz_temp_data_fixed` (`BMM350_USE_FIXED_POINT`
//! variant, A48.16 fixed point) from Bosch Sensortec's BMM350 SensorAPI
//! v1.10.0 (<https://github.com/boschsensortec/BMM350_SensorAPI>,
//! `bmm350.c`). Original notice:
//!
//! Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
//!
//! BSD-3-Clause
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions are met:
//!
//! 1. Redistributions of source code must retain the above copyright
//!    notice, this list of conditions and the following disclaimer.
//!
//! 2. Redistributions in binary form must reproduce the above copyright
//!    notice, this list of conditions and the following disclaimer in the
//!    documentation and/or other materials provided with the distribution.
//!
//! 3. Neither the name of the copyright holder nor the names of its
//!    contributors may be used to endorse or promote products derived from
//!    this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//! "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//! LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//! FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//! COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//! INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//! (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//! SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//! HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
//! STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
//! IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//! POSSIBILITY OF SUCH DAMAGE.

/// First OTP word holding compensation data (temperature offset/sensitivity).
pub const OTP_FIRST_WORD: u8 = 0x0D;

/// Number of OTP words used (0x0D..=0x18).
pub const OTP_WORDS: usize = 12;

/// Factory OTP words 0x0D..=0x18 as read through `OTP_DATA_MSB/LSB`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Otp(pub [u16; OTP_WORDS]);

/// Uncompensated 24-bit two's-complement samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Raw {
    /// X axis.
    pub x: i32,
    /// Y axis.
    pub y: i32,
    /// Z axis.
    pub z: i32,
    /// Temperature.
    pub t: i32,
}

/// Compensated field (A48.16 fixed point, µT) and die temperature
/// (A48.16, °C).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Compensated {
    /// X, Y, Z in µT × 65536.
    pub field: [i64; 3],
    /// Temperature in °C × 65536.
    pub temperature: i64,
}

const ONE: i64 = 1 << 16;

/// A48.16 multiply, truncating toward zero like `fixed_mul_A48_16`.
fn mul(a: i64, b: i64) -> i64 {
    a.saturating_mul(b) / ONE
}

/// A48.16 divide, truncating toward zero like `fixed_div`; `None` on a zero
/// divisor.
fn div(a: i64, b: i64) -> Option<i64> {
    a.saturating_mul(ONE).checked_div(b)
}

fn int(v: i32) -> i64 {
    i64::from(v) * ONE
}

fn s8(v: u16) -> i32 {
    i32::from(v as u8 as i8)
}

fn s12(v: u16) -> i32 {
    i32::from(((v << 4) as i16) >> 4)
}

fn lo(v: u16) -> u16 {
    v & 0x00FF
}

fn hi(v: u16) -> u16 {
    v >> 8
}

struct Coefficients {
    offset: [i64; 3],
    sens: [i64; 3],
    tco: [i64; 3],
    tcs: [i64; 3],
    t_offs: i64,
    t_sens: i64,
    t0: i64,
    cross_x_y: i64,
    cross_y_x: i64,
    cross_z_x: i64,
    cross_z_y: i64,
}

impl Otp {
    fn word(&self, addr: u8) -> u16 {
        self.0[usize::from(addr - OTP_FIRST_WORD)]
    }

    fn coefficients(&self) -> Coefficients {
        let w = |a| self.word(a);
        let off_x = w(0x0E) & 0x0FFF;
        let off_y = ((w(0x0E) & 0xF000) >> 4) + lo(w(0x0F));
        let off_z = (w(0x0F) & 0x0F00) + lo(w(0x10));
        // OTP scale factors in A48.16: 1/256, 1/512, 1/32, 1/16384, 1/800
        // (rounded to 82), 1/5 (0x3333).
        let sens = |v| mul(int(s8(v)), 0x100);
        let tco = |v| mul(int(s8(v)), 0x800);
        let tcs = |v| mul(int(s8(v)), 4);
        let cross = |v| mul(int(s8(v)), 0x52);
        Coefficients {
            offset: [int(s12(off_x)), int(s12(off_y)), int(s12(off_z))],
            sens: [sens(hi(w(0x10))), sens(lo(w(0x11))), sens(hi(w(0x11)))],
            tco: [tco(lo(w(0x12))), tco(lo(w(0x13))), tco(lo(w(0x14)))],
            tcs: [tcs(hi(w(0x12))), tcs(hi(w(0x13))), tcs(hi(w(0x14)))],
            t_offs: mul(int(s8(lo(w(0x0D)))), 0x3333),
            t_sens: mul(int(s8(hi(w(0x0D)))), 0x80),
            t0: mul(int(i32::from(w(0x18) as i16)), 0x80) + int(23),
            cross_x_y: cross(lo(w(0x15))),
            cross_y_x: cross(hi(w(0x15))),
            cross_z_x: cross(lo(w(0x16))),
            cross_z_y: cross(hi(w(0x16))),
        }
    }

    /// Compensate one raw sample. `None` if a divisor becomes zero
    /// (only possible for nonsensical raw temperatures).
    pub fn compensate(&self, raw: Raw) -> Option<Compensated> {
        let c = self.coefficients();
        // Raw LSB → µT / °C: 0.007069979 (X/Y), 0.007174964 (Z),
        // 0.000981282 (T), coefficients scaled by 4.
        let mut o = [
            mul(int(raw.x), 0x73D) >> 2,
            mul(int(raw.y), 0x73D) >> 2,
            mul(int(raw.z), 0x759) >> 2,
        ];
        let t = (mul(int(raw.t), 0x101) >> 2) - 0x0019_7D70;
        let t = mul(ONE + c.t_sens, t) + c.t_offs;
        let dt = t - c.t0;
        for (i, v) in o.iter_mut().enumerate() {
            let mut x = mul(*v, ONE + c.sens[i]) + c.offset[i];
            x += mul(c.tco[i], dt);
            *v = div(x, ONE + mul(c.tcs[i], dt))?;
        }
        let [x, y, z] = o;
        let denom = ONE - mul(c.cross_y_x, c.cross_x_y);
        let cx = div(x - mul(c.cross_x_y, y), denom)?;
        let cy = div(y - mul(c.cross_y_x, x), denom)?;
        let z1 = mul(c.cross_y_x, c.cross_z_y);
        let z2 = mul(c.cross_x_y, c.cross_z_x);
        let cz = z + div(mul(x, z1 - c.cross_z_x) - mul(y, c.cross_z_y - z2), denom)?;
        Some(Compensated {
            field: [cx, cy, cz],
            temperature: t,
        })
    }
}

/// Round an A48.16 value × `scale` to the nearest integer (half away from
/// zero), saturating to `i32`.
pub fn scale_round(v: i64, scale: i64) -> i32 {
    let p = v.saturating_mul(scale);
    let r = if p >= 0 { p + ONE / 2 } else { p - ONE / 2 } / ONE;
    r.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    // OTP words 0x0D..=0x18 with mostly negative coefficients.
    const OTP: Otp = Otp([
        0x05F6, 0xFFE0, 0x009C, 0xFC35, 0xF907, 0xF00C, 0x08F4, 0xE020, 0xFA05, 0xF30A, 0x0000,
        0xFC00,
    ]);

    // Golden values from Bosch SensorAPI v1.10.0 built with
    // BMM350_USE_FIXED_POINT (`bmm350_init` + `..._fixed` on a fake bus).
    #[test]
    fn matches_sensor_api_fixed_point() {
        let cases = [
            (
                Raw {
                    x: 4000,
                    y: -7000,
                    z: 60000,
                    t: 56553,
                },
                [-32_973, -10_028_003, 31_679_488],
                1_851_118,
            ),
            (
                Raw {
                    x: -1_000_000,
                    y: 2_000_000,
                    z: -8_000_000,
                    t: 20000,
                },
                [-452_192_850, 956_156_083, -3_440_685_964],
                -520_346,
            ),
            (
                Raw::default(),
                [-3_110_802, -5_509_663, 204_301],
                -1_817_895,
            ),
            (
                Raw {
                    x: 8_388_607,
                    y: -8_388_608,
                    z: 123_456,
                    t: -100_000,
                },
                [3_364_824_163, -4_280_516_271, -72_714_141],
                -8_305_639,
            ),
        ];
        for (raw, field, temperature) in cases {
            let c = OTP.compensate(raw).unwrap();
            assert_eq!(c.field, field, "{raw:?}");
            assert_eq!(c.temperature, temperature, "{raw:?}");
        }
    }

    #[test]
    fn rounding() {
        // -32973 / 65536 µT = -503.1 nT; 1851118 / 65536 °C = 28.25 °C.
        assert_eq!(scale_round(-32_973, 1000), -503);
        assert_eq!(scale_round(1_851_118, 100), 2825);
        assert_eq!(scale_round(ONE / 2, 1), 1);
        assert_eq!(scale_round(-ONE / 2, 1), -1);
    }
}
