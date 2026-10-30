//! Temperature/sensitivity compensation of the raw BMM150 output with the
//! factory trim registers.
//!
//! Integer port of `compensate_x`, `compensate_y` and `compensate_z`
//! (fixed-point variant) and `read_trim_registers` from Bosch Sensortec's
//! BMM150 SensorAPI v2.0.0 (<https://github.com/boschsensortec/BMM150_SensorAPI>,
//! `bmm150.c`). Unlike v2.0.0 the final `/ 16` is not applied, keeping the
//! 1/16 µT resolution of the earlier API versions. Original notice:
//!
//! Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
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

/// X/Y raw output on overflow.
pub const OVERFLOW_XY: i16 = -4096;

/// Z raw output on overflow.
pub const OVERFLOW_Z: i16 = -16384;

/// Factory trim values (NVM, registers 0x5D–0x71).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub struct Trim {
    pub x1: i8,
    pub y1: i8,
    pub x2: i8,
    pub y2: i8,
    pub z1: u16,
    pub z2: i16,
    pub z3: i16,
    pub z4: i16,
    pub xy1: u8,
    pub xy2: i8,
    pub xyz1: u16,
}

impl Trim {
    /// Build from the three trim blocks: 0x5D–0x5E (`x1y1`), 0x62–0x65
    /// (`z4x2y2`) and 0x68–0x71 (`z2_xy1`).
    pub fn from_regs(x1y1: &[u8; 2], z4x2y2: &[u8; 4], z2_xy1: &[u8; 10]) -> Self {
        let le = |l: u8, m: u8| u16::from_le_bytes([l, m]);
        Self {
            x1: x1y1[0] as i8,
            y1: x1y1[1] as i8,
            x2: z4x2y2[2] as i8,
            y2: z4x2y2[3] as i8,
            z1: le(z2_xy1[2], z2_xy1[3]),
            z2: le(z2_xy1[0], z2_xy1[1]) as i16,
            z3: le(z2_xy1[6], z2_xy1[7]) as i16,
            z4: le(z4x2y2[0], z4x2y2[1]) as i16,
            xy1: z2_xy1[9],
            xy2: z2_xy1[8] as i8,
            xyz1: le(z2_xy1[4], z2_xy1[5] & 0x7F),
        }
    }

    /// X axis in 1/16 µT; `None` on overflow or missing trim/RHALL data.
    pub fn compensate_x(&self, raw: i16, rhall: u16) -> Option<i32> {
        self.compensate_xy(raw, rhall, self.x1, self.x2)
    }

    /// Y axis in 1/16 µT; `None` on overflow or missing trim/RHALL data.
    pub fn compensate_y(&self, raw: i16, rhall: u16) -> Option<i32> {
        self.compensate_xy(raw, rhall, self.y1, self.y2)
    }

    // X and Y use the same equations with their own dig_*1 / dig_*2.
    fn compensate_xy(&self, raw: i16, rhall: u16, d1: i8, d2: i8) -> Option<i32> {
        if raw == OVERFLOW_XY {
            return None;
        }
        let r0 = if rhall != 0 { rhall } else { self.xyz1 };
        if r0 == 0 {
            return None;
        }
        let c1 = i64::from(self.xyz1) * 16384 / i64::from(r0);
        let c = i64::from((c1 as u16).wrapping_sub(0x4000) as i16);
        let c4 = i64::from(self.xy2) * (c * c / 128);
        let c6 = c * (i64::from(self.xy1) * 128);
        let c7 = (c4 + c6) / 512 + 0x10_0000;
        let c9 = c7 * (i64::from(d2) + 0xA0) / 4096;
        let v = (i64::from(raw) * c9 / 8192) as i16;
        Some(i32::from(v) + i32::from(d1) * 8)
    }

    /// Z axis in 1/16 µT (saturated to ±32767); `None` on overflow or
    /// missing trim/RHALL data.
    pub fn compensate_z(&self, raw: i16, rhall: u16) -> Option<i32> {
        if raw == OVERFLOW_Z || self.z1 == 0 || self.z2 == 0 || self.xyz1 == 0 || rhall == 0 {
            return None;
        }
        let r = i64::from(rhall as i16);
        let z0 = i64::from((rhall as i16).wrapping_sub(self.xyz1 as i16));
        let z1 = i64::from(self.z3) * z0 / 4;
        let z2 = (i64::from(raw) - i64::from(self.z4)) * 32768;
        let z3 = i64::from(self.z1) * (r * 2);
        let z4 = i64::from(((z3 + 32768) / 65536) as i16);
        let den = i64::from(self.z2) + z4;
        if den == 0 {
            return None;
        }
        Some(((z2 - z1) / den).clamp(-32767, 32767) as i32)
    }
}
