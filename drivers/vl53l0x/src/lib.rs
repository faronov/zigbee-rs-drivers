//! ST VL53L0X time-of-flight ranging sensor (up to ≈2 m).
//!
//! The VL53L0X has no public register map; ST only ships the VL53L0X API
//! (STSW-IMG005). This driver re-implements the minimal path of that API
//! (stm32duino `VL53L0X` port, BSD-3-Clause): `DataInit`, `StaticInit`
//! with reference SPADs taken from NVM, the default tuning settings,
//! `PerformRefCalibration` and single-shot ranging. The tuning table,
//! register sequences and timing arithmetic follow those sources; their
//! notices are kept below.
//!
//! * Initialisation is split so the driver never waits:
//!   [`Vl53l0x::begin_init`] runs `DataInit`/`StaticInit` and starts the
//!   VHV calibration, [`Vl53l0x::finish_init`] returns `NotReady` until
//!   the VHV and phase calibrations have both finished.
//! * `Sensor::start_measurement` starts one ranging and returns the
//!   timing budget plus 10 %; `read_measurement` returns `NotReady` until
//!   the interrupt status is set, then reads and clears it.
//! * Only API range status 0 (valid) is reported as `Quantity::Distance`.

// Portions (initialisation sequences, reference SPAD selection, timing
// budget arithmetic, range-status mapping) derived from the stm32duino
// VL53L0X driver class:
//
// COPYRIGHT(c) 2016 STMicroelectronics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   1. Redistributions of source code must retain the above copyright notice,
//      this list of conditions and the following disclaimer.
//   2. Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//   3. Neither the name of STMicroelectronics nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Default 7-bit address.
pub const DEFAULT_ADDRESS: u8 = 0x29;
/// Model ID register (0xC0) value.
pub const MODEL_ID: u8 = 0xEE;
/// Wait hint between [`Vl53l0x::finish_init`] calls. The calibration
/// durations are not documented; this is an estimate.
pub const CAL_WAIT_US: WaitUs = 5_000;
/// Smallest timing budget accepted by the API.
pub const MIN_BUDGET_US: u32 = 20_000;
/// Budget assumed before [`Vl53l0x::begin_init`] has read the real one
/// (≈33 ms with the default tuning).
pub const DEFAULT_BUDGET_US: u32 = 33_000;

const REG_SYSRANGE_START: u8 = 0x00;
const REG_SEQUENCE_CONFIG: u8 = 0x01;
const REG_INTERRUPT_CONFIG_GPIO: u8 = 0x0A;
const REG_INTERRUPT_CLEAR: u8 = 0x0B;
const REG_RESULT_INTERRUPT_STATUS: u8 = 0x13;
const REG_RESULT_RANGE_STATUS: u8 = 0x14;
const REG_FINAL_RANGE_MIN_COUNT_RATE: u8 = 0x44;
const REG_MSRC_TIMEOUT: u8 = 0x46;
const REG_DYNAMIC_SPAD_NUM_REQUESTED: u8 = 0x4E;
const REG_DYNAMIC_SPAD_START_OFFSET: u8 = 0x4F;
const REG_PRE_RANGE_VCSEL: u8 = 0x50;
const REG_PRE_RANGE_TIMEOUT: u8 = 0x51;
const REG_MSRC_CONFIG_CONTROL: u8 = 0x60;
const REG_FINAL_RANGE_VCSEL: u8 = 0x70;
const REG_FINAL_RANGE_TIMEOUT: u8 = 0x71;
const REG_GPIO_HV_MUX_ACTIVE_HIGH: u8 = 0x84;
const REG_I2C_MODE: u8 = 0x88;
const REG_VHV_CONFIG_PAD: u8 = 0x89;
const REG_I2C_ADDRESS: u8 = 0x8A;
const REG_SPAD_ENABLES_REF_0: u8 = 0xB0;
const REG_REF_EN_START_SELECT: u8 = 0xB6;
const REG_MODEL_ID: u8 = 0xC0;

const SEQ_TCC: u8 = 0x10;
const SEQ_DSS: u8 = 0x08;
const SEQ_MSRC: u8 = 0x04;
const SEQ_PRE_RANGE: u8 = 0x40;
const SEQ_FINAL_RANGE: u8 = 0x80;

const START_OVERHEAD_US: u32 = 1910;
const END_OVERHEAD_US: u32 = 960;
const MSRC_OVERHEAD_US: u32 = 660;
const TCC_OVERHEAD_US: u32 = 590;
const DSS_OVERHEAD_US: u32 = 690;
const PRE_RANGE_OVERHEAD_US: u32 = 660;
const FINAL_RANGE_OVERHEAD_US: u32 = 550;

/// API `VL53L0X_DEFAULT_MAX_LOOP`: bound for the NVM strobe poll.
const STROBE_POLLS: u16 = 2000;
/// First reference SPAD index (`GLOBAL_CONFIG_REF_EN_START_SELECT`).
const REF_START_SELECT: u8 = 0xB4;
const SPAD_MAP_LEN: usize = 6;

// DefaultTuningSettings (update 02/11/2015_v36), from vl53l0x_tuning.h.
// Every API entry is a single-byte write, so it is stored as
// (register, value) pairs.
//
// Copyright © 2016, STMicroelectronics International N.V.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of STMicroelectronics nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
// IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
const TUNING: [(u8, u8); 80] = [
    (0xFF, 0x01),
    (0x00, 0x00),
    (0xFF, 0x00),
    (0x09, 0x00),
    (0x10, 0x00),
    (0x11, 0x00),
    (0x24, 0x01),
    (0x25, 0xFF),
    (0x75, 0x00),
    (0xFF, 0x01),
    (0x4E, 0x2C),
    (0x48, 0x00),
    (0x30, 0x20),
    (0xFF, 0x00),
    (0x30, 0x09),
    (0x54, 0x00),
    (0x31, 0x04),
    (0x32, 0x03),
    (0x40, 0x83),
    (0x46, 0x25),
    (0x60, 0x00),
    (0x27, 0x00),
    (0x50, 0x06),
    (0x51, 0x00),
    (0x52, 0x96),
    (0x56, 0x08),
    (0x57, 0x30),
    (0x61, 0x00),
    (0x62, 0x00),
    (0x64, 0x00),
    (0x65, 0x00),
    (0x66, 0xA0),
    (0xFF, 0x01),
    (0x22, 0x32),
    (0x47, 0x14),
    (0x49, 0xFF),
    (0x4A, 0x00),
    (0xFF, 0x00),
    (0x7A, 0x0A),
    (0x7B, 0x00),
    (0x78, 0x21),
    (0xFF, 0x01),
    (0x23, 0x34),
    (0x42, 0x00),
    (0x44, 0xFF),
    (0x45, 0x26),
    (0x46, 0x05),
    (0x40, 0x40),
    (0x0E, 0x06),
    (0x20, 0x1A),
    (0x43, 0x40),
    (0xFF, 0x00),
    (0x34, 0x03),
    (0x35, 0x44),
    (0xFF, 0x01),
    (0x31, 0x04),
    (0x4B, 0x09),
    (0x4C, 0x05),
    (0x4D, 0x04),
    (0xFF, 0x00),
    (0x44, 0x00),
    (0x45, 0x20),
    (0x47, 0x08),
    (0x48, 0x28),
    (0x67, 0x00),
    (0x70, 0x04),
    (0x71, 0x01),
    (0x72, 0xFE),
    (0x76, 0x00),
    (0x77, 0x00),
    (0xFF, 0x01),
    (0x0D, 0x01),
    (0xFF, 0x00),
    (0x80, 0x01),
    (0x01, 0xF8),
    (0xFF, 0x01),
    (0x8E, 0x01),
    (0x00, 0x01),
    (0xFF, 0x00),
    (0x80, 0x00),
];

/// Decode a 16-bit sequence-step timeout register (LSB × 2^MSB + 1),
/// in macro clocks.
pub const fn decode_timeout(reg: u16) -> u32 {
    (((reg & 0xFF) as u32) << (reg >> 8)) + 1
}

/// Encode a timeout in macro clocks into the register format.
pub const fn encode_timeout(mclks: u32) -> u16 {
    if mclks == 0 {
        return 0;
    }
    let mut ls = mclks - 1;
    let mut ms = 0u16;
    while ls > 0xFF {
        ls >>= 1;
        ms += 1;
    }
    (ms << 8) | ls as u16
}

/// Macro period for a VCSEL period in PCLKs, ns.
const fn macro_period_ns(vcsel_pclks: u8) -> u64 {
    (2304 * vcsel_pclks as u64 * 1655 + 500) / 1000
}

const fn mclks_to_us(mclks: u32, vcsel_pclks: u8) -> u32 {
    ((mclks as u64 * macro_period_ns(vcsel_pclks) + 500) / 1000) as u32
}

const fn us_to_mclks(us: u32, vcsel_pclks: u8) -> u32 {
    let ns = macro_period_ns(vcsel_pclks);
    if ns == 0 {
        return 0;
    }
    ((us as u64 * 1000 + ns / 2) / ns) as u32
}

/// Sequence-step configuration and timeouts, as read from the device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    /// `SYSTEM_SEQUENCE_CONFIG` (TCC 0x10, DSS 0x08, MSRC 0x04, pre-range
    /// 0x40, final range 0x80).
    pub sequence: u8,
    /// Pre-range VCSEL period, PCLKs.
    pub pre_vcsel_pclks: u8,
    /// Final-range VCSEL period, PCLKs.
    pub final_vcsel_pclks: u8,
    /// MSRC/DSS/TCC timeout, macro clocks.
    pub msrc_mclks: u32,
    /// Pre-range timeout, macro clocks.
    pub pre_mclks: u32,
    /// Final-range register timeout, macro clocks (includes the pre-range
    /// timeout when the pre-range step is enabled).
    pub final_mclks: u32,
}

impl Timing {
    /// Build from raw registers 0x01, 0x50, 0x70, 0x46, 0x51 and 0x71.
    pub const fn from_registers(
        sequence: u8,
        pre_vcsel: u8,
        final_vcsel: u8,
        msrc: u8,
        pre: u16,
        final_range: u16,
    ) -> Self {
        Self {
            sequence,
            pre_vcsel_pclks: (pre_vcsel + 1) << 1,
            final_vcsel_pclks: (final_vcsel + 1) << 1,
            msrc_mclks: msrc as u32 + 1,
            pre_mclks: decode_timeout(pre),
            final_mclks: decode_timeout(final_range),
        }
    }

    const fn has(&self, step: u8) -> bool {
        self.sequence & step != 0
    }

    const fn msrc_us(&self) -> u32 {
        mclks_to_us(self.msrc_mclks, self.pre_vcsel_pclks)
    }

    const fn pre_us(&self) -> u32 {
        mclks_to_us(self.pre_mclks, self.pre_vcsel_pclks)
    }

    /// Measurement timing budget, µs (API
    /// `GetMeasurementTimingBudgetMicroSeconds`).
    pub const fn budget_us(&self) -> u32 {
        let msrc = self.msrc_us();
        let mut budget = START_OVERHEAD_US + END_OVERHEAD_US;
        if self.has(SEQ_TCC) {
            budget += msrc + TCC_OVERHEAD_US;
        }
        if self.has(SEQ_DSS) {
            budget += 2 * (msrc + DSS_OVERHEAD_US);
        } else if self.has(SEQ_MSRC) {
            budget += msrc + MSRC_OVERHEAD_US;
        }
        if self.has(SEQ_PRE_RANGE) {
            budget += self.pre_us() + PRE_RANGE_OVERHEAD_US;
        }
        if self.has(SEQ_FINAL_RANGE) {
            let pre = if self.has(SEQ_PRE_RANGE) {
                self.pre_mclks
            } else {
                0
            };
            budget += mclks_to_us(self.final_mclks.saturating_sub(pre), self.final_vcsel_pclks)
                + FINAL_RANGE_OVERHEAD_US;
        }
        budget
    }

    /// Final-range timeout register value that gives `budget_us` (API
    /// `SetMeasurementTimingBudgetMicroSeconds`); `None` if the budget is
    /// below [`MIN_BUDGET_US`] or too short for the enabled steps.
    pub const fn final_timeout_for(&self, budget_us: u32) -> Option<u16> {
        if budget_us < MIN_BUDGET_US || !self.has(SEQ_FINAL_RANGE) {
            return None;
        }
        let msrc = self.msrc_us();
        let mut rem = budget_us - START_OVERHEAD_US - END_OVERHEAD_US;
        let mut steps = [0u32; 4];
        if self.has(SEQ_TCC) {
            steps[0] = msrc + TCC_OVERHEAD_US;
        }
        if self.has(SEQ_DSS) {
            steps[1] = 2 * (msrc + DSS_OVERHEAD_US);
        } else if self.has(SEQ_MSRC) {
            steps[1] = msrc + MSRC_OVERHEAD_US;
        }
        if self.has(SEQ_PRE_RANGE) {
            steps[2] = self.pre_us() + PRE_RANGE_OVERHEAD_US;
        }
        steps[3] = FINAL_RANGE_OVERHEAD_US;
        let mut i = 0;
        while i < steps.len() {
            if steps[i] != 0 {
                if rem <= steps[i] {
                    return None;
                }
                rem -= steps[i];
            }
            i += 1;
        }
        let mut mclks = us_to_mclks(rem, self.final_vcsel_pclks);
        if self.has(SEQ_PRE_RANGE) {
            mclks += self.pre_mclks;
        }
        Some(encode_timeout(mclks))
    }
}

/// Reference SPAD data stored in the device NVM.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpadInfo {
    /// Number of reference SPADs to enable.
    pub count: u8,
    /// Use aperture SPADs (otherwise non-aperture).
    pub aperture: bool,
    /// Good-SPAD bitmap (bit n of byte k is SPAD 8k + n).
    pub good_map: [u8; SPAD_MAP_LEN],
}

const fn is_aperture(index: u32) -> bool {
    // API refArrayQuadrants {10, 5, 0, 5}: quadrant 2 has no aperture.
    (index >> 6) & 3 != 2
}

impl SpadInfo {
    /// Decode the NVM words read with strobe selects 0x6B, 0x24 and 0x25.
    pub const fn from_nvm(w6b: u32, w24: u32, w25: u32) -> Self {
        let a = w24.to_be_bytes();
        let b = w25.to_be_bytes();
        Self {
            count: ((w6b >> 8) & 0x7F) as u8,
            aperture: (w6b >> 15) & 1 == 1,
            good_map: [a[0], a[1], a[2], a[3], b[0], b[1]],
        }
    }

    /// NVM data usable without ST's reference SPAD management (at most
    /// 32 aperture or 12 non-aperture SPADs).
    pub const fn is_valid(&self) -> bool {
        if self.aperture {
            self.count <= 32
        } else {
            self.count <= 12
        }
    }

    /// Reference SPAD enable map (API `set_reference_spads`): the first
    /// `count` good SPADs of the requested type, starting at the first
    /// SPAD of that type. `None` if they run out or the type changes.
    pub fn reference_map(&self) -> Option<[u8; SPAD_MAP_LEN]> {
        let start = REF_START_SELECT as u32;
        let size = (SPAD_MAP_LEN * 8) as u32;
        let mut current = 0;
        if self.aperture {
            while !is_aperture(start + current) && current < 44 {
                current += 1;
            }
        }
        let mut map = [0u8; SPAD_MAP_LEN];
        for _ in 0..self.count {
            let next =
                (current..size).find(|&i| (self.good_map[(i / 8) as usize] >> (i % 8)) & 1 == 1)?;
            if is_aperture(start + next) != self.aperture {
                return None;
            }
            map[(next / 8) as usize] |= 1 << (next % 8);
            current = next + 1;
        }
        Some(map)
    }
}

/// API range status from the device range status (register 0x14 bits
/// 6:3): 0 valid, 2 signal fail, 3 minimum range, 4 phase fail, 5
/// hardware fail, 255 none. The host-side sigma check (status 1) is not
/// implemented.
pub const fn range_status(device: u8) -> u8 {
    match (device & 0x78) >> 3 {
        1..=3 => 5,
        6 | 9 => 4,
        8 | 10 => 3,
        4 => 2,
        11 => 0,
        _ => 255,
    }
}

/// One ranging result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// API range status (see [`range_status`]).
    pub range_status: u8,
    /// Raw device range status register.
    pub device_status: u8,
    /// Distance, mm.
    pub distance_mm: u16,
    /// Return signal rate, kcps.
    pub signal_kcps: u32,
    /// Ambient rate, kcps.
    pub ambient_kcps: u32,
    /// Effective return SPAD count, 8.8 fixed point.
    pub spad_count_q8: u16,
}

impl Measurement {
    /// Decode the 12-byte result block at 0x14.
    pub fn decode(raw: &[u8; 12]) -> Self {
        let word = |i: usize| u16::from_be_bytes([raw[i], raw[i + 1]]);
        // Rates are MCPS in 9.7 fixed point.
        let kcps = |v: u16| v as u32 * 125 / 16;
        Self {
            range_status: range_status(raw[0]),
            device_status: raw[0],
            distance_mm: word(10),
            signal_kcps: kcps(word(6)),
            ambient_kcps: kcps(word(8)),
            spad_count_q8: word(2),
        }
    }

    /// Range status 0.
    pub const fn is_valid(&self) -> bool {
        self.range_status == 0
    }
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if self.is_valid() {
            f(Quantity::Distance(self.distance_mm as u32));
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InitPhase {
    Idle,
    Vhv,
    Phase,
    Done,
}

macro_rules! api {
    ($($asyncness:ident)?; $($aw:tt)*) => {
        /// Create a driver at [`DEFAULT_ADDRESS`]; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self::with_address(i2c, DEFAULT_ADDRESS)
        }

        /// Create a driver at another 7-bit address (after `set_address`).
        pub const fn with_address(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                stop_variable: 0,
                sequence: 0,
                budget_us: DEFAULT_BUDGET_US,
                phase: InitPhase::Idle,
            }
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        /// Current 7-bit address.
        pub const fn address(&self) -> u8 {
            self.address
        }

        /// Timing budget, µs (read during init or set by
        /// [`Self::set_timing_budget`]).
        pub const fn timing_budget_us(&self) -> u32 {
            self.budget_us
        }

        $($asyncness)? fn write(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<I2C::Error>> {
            let mut buf = [0u8; 1 + SPAD_MAP_LEN];
            buf[0] = reg;
            buf[1..1 + data.len()].copy_from_slice(data);
            self.i2c.write(self.address, &buf[..1 + data.len()]) $($aw)* .map_err(Error::Bus)
        }

        $($asyncness)? fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c.write_read(self.address, &[reg], buf) $($aw)* .map_err(Error::Bus)
        }

        $($asyncness)? fn wr8(&mut self, reg: u8, v: u8) -> Result<(), Error<I2C::Error>> {
            self.write(reg, &[v]) $($aw)*
        }

        $($asyncness)? fn wr16(&mut self, reg: u8, v: u16) -> Result<(), Error<I2C::Error>> {
            self.write(reg, &v.to_be_bytes()) $($aw)*
        }

        $($asyncness)? fn wr_seq(&mut self, seq: &[(u8, u8)]) -> Result<(), Error<I2C::Error>> {
            for &(reg, v) in seq {
                self.wr8(reg, v) $($aw)* ?;
            }
            Ok(())
        }

        $($asyncness)? fn rd8(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
            let mut b = [0u8];
            self.read(reg, &mut b) $($aw)* ?;
            Ok(b[0])
        }

        $($asyncness)? fn rd16(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
            let mut b = [0u8; 2];
            self.read(reg, &mut b) $($aw)* ?;
            Ok(u16::from_be_bytes(b))
        }

        $($asyncness)? fn rd32(&mut self, reg: u8) -> Result<u32, Error<I2C::Error>> {
            let mut b = [0u8; 4];
            self.read(reg, &mut b) $($aw)* ?;
            Ok(u32::from_be_bytes(b))
        }

        /// Check the model ID (0xC0 = 0xEE).
        pub $($asyncness)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            let id = self.rd8(REG_MODEL_ID) $($aw)* ?;
            if id != MODEL_ID {
                return Err(Error::UnexpectedId(id as u32));
            }
            Ok(())
        }

        /// Change the 7-bit address (volatile; lost on power-down or
        /// XSHUT).
        pub $($asyncness)? fn set_address(&mut self, address: u8) -> Result<(), Error<I2C::Error>> {
            self.wr8(REG_I2C_ADDRESS, address & 0x7F) $($aw)* ?;
            self.address = address & 0x7F;
            Ok(())
        }

        /// Switch the I/O pads to 2.8 V levels (API `USE_I2C_2V8`). The
        /// default is 1.8 V; boards that pull SDA/SCL to 2.8–3.3 V
        /// usually need this. Call before [`Self::begin_init`].
        pub $($asyncness)? fn enable_io_2v8(&mut self) -> Result<(), Error<I2C::Error>> {
            let v = self.rd8(REG_VHV_CONFIG_PAD) $($aw)* ?;
            self.wr8(REG_VHV_CONFIG_PAD, v | 0x01) $($aw)*
        }

        /// API `DataInit` and `StaticInit`, then start the VHV
        /// calibration. Call [`Self::finish_init`] after the returned
        /// wait. Returns `Unsupported` if the NVM reference SPAD data is
        /// unusable (ST's SPAD management is not implemented),
        /// `Timeout` if the NVM read does not complete, and
        /// `InvalidData` if the SPAD map does not read back.
        pub $($asyncness)? fn begin_init(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.phase = InitPhase::Idle;
            // DataInit.
            self.wr8(REG_I2C_MODE, 0x00) $($aw)* ?;
            self.wr_seq(&[(0x80, 0x01), (0xFF, 0x01), (0x00, 0x00)]) $($aw)* ?;
            self.stop_variable = self.rd8(0x91) $($aw)* ?;
            self.wr_seq(&[(0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)]) $($aw)* ?;
            // Net effect of the API limit checks: MSRC and pre-range signal
            // rate checks disabled, final-range minimum rate 0.25 MCPS.
            let v = self.rd8(REG_MSRC_CONFIG_CONTROL) $($aw)* ?;
            self.wr8(REG_MSRC_CONFIG_CONTROL, (v & 0xEE) | 0x12) $($aw)* ?;
            self.wr16(REG_FINAL_RANGE_MIN_COUNT_RATE, 32) $($aw)* ?;
            self.wr8(REG_SEQUENCE_CONFIG, 0xFF) $($aw)* ?;

            // StaticInit.
            let info = self.read_spad_info() $($aw)* ?;
            self.set_reference_spads(&info) $($aw)* ?;
            self.wr_seq(&TUNING) $($aw)* ?;
            // GPIO: new sample ready, active low.
            self.wr8(REG_INTERRUPT_CONFIG_GPIO, 0x04) $($aw)* ?;
            let v = self.rd8(REG_GPIO_HV_MUX_ACTIVE_HIGH) $($aw)* ?;
            self.wr8(REG_GPIO_HV_MUX_ACTIVE_HIGH, v & 0xEF) $($aw)* ?;
            self.clear_interrupt() $($aw)* ?;
            // Disable TCC and MSRC, keeping the timing budget.
            let timing = self.read_timing() $($aw)* ?;
            let budget = timing.budget_us();
            let sequence = timing.sequence & !(SEQ_TCC | SEQ_MSRC);
            let reg = Timing { sequence, ..timing }
                .final_timeout_for(budget)
                .ok_or(Error::InvalidData)?;
            self.wr8(REG_SEQUENCE_CONFIG, sequence) $($aw)* ?;
            self.wr16(REG_FINAL_RANGE_TIMEOUT, reg) $($aw)* ?;
            self.sequence = sequence;
            self.budget_us = budget;

            // PerformRefCalibration: VHV first.
            self.wr8(REG_SEQUENCE_CONFIG, 0x01) $($aw)* ?;
            self.wr8(REG_SYSRANGE_START, 0x41) $($aw)* ?;
            self.phase = InitPhase::Vhv;
            Ok(CAL_WAIT_US)
        }

        /// `NotReady` until the VHV and then the phase calibration have
        /// finished (call again after [`CAL_WAIT_US`]); `Ok` once the
        /// sensor is ready for ranging.
        pub $($asyncness)? fn finish_init(&mut self) -> Result<(), Error<I2C::Error>> {
            match self.phase {
                InitPhase::Idle => Err(Error::InvalidData),
                InitPhase::Done => Ok(()),
                InitPhase::Vhv | InitPhase::Phase => {
                    if !self.data_ready() $($aw)* ? {
                        return Err(Error::NotReady);
                    }
                    self.clear_interrupt() $($aw)* ?;
                    self.wr8(REG_SYSRANGE_START, 0x00) $($aw)* ?;
                    if self.phase == InitPhase::Vhv {
                        self.wr8(REG_SEQUENCE_CONFIG, 0x02) $($aw)* ?;
                        self.wr8(REG_SYSRANGE_START, 0x01) $($aw)* ?;
                        self.phase = InitPhase::Phase;
                        return Err(Error::NotReady);
                    }
                    let sequence = self.sequence;
                    self.wr8(REG_SEQUENCE_CONFIG, sequence) $($aw)* ?;
                    self.phase = InitPhase::Done;
                    Ok(())
                }
            }
        }

        $($asyncness)? fn strobe(&mut self) -> Result<bool, Error<I2C::Error>> {
            self.wr8(0x83, 0x00) $($aw)* ?;
            let mut done = false;
            for _ in 0..STROBE_POLLS {
                if self.rd8(0x83) $($aw)* ? != 0 {
                    done = true;
                    break;
                }
            }
            self.wr8(0x83, 0x01) $($aw)* ?;
            Ok(done)
        }

        $($asyncness)? fn read_spad_info(&mut self) -> Result<SpadInfo, Error<I2C::Error>> {
            self.wr_seq(&[(0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x06)]) $($aw)* ?;
            let v = self.rd8(0x83) $($aw)* ?;
            self.wr8(0x83, v | 0x04) $($aw)* ?;
            self.wr_seq(&[(0xFF, 0x07), (0x81, 0x01), (0x80, 0x01)]) $($aw)* ?;
            let mut words = [0u32; 3];
            let mut done = true;
            for (word, select) in words.iter_mut().zip([0x6B, 0x24, 0x25]) {
                self.wr8(0x94, select) $($aw)* ?;
                if !self.strobe() $($aw)* ? {
                    done = false;
                    break;
                }
                *word = self.rd32(0x90) $($aw)* ?;
            }
            // Leave the NVM access mode even after a timeout.
            self.wr_seq(&[(0x81, 0x00), (0xFF, 0x06)]) $($aw)* ?;
            let v = self.rd8(0x83) $($aw)* ?;
            self.wr8(0x83, v & 0xFB) $($aw)* ?;
            self.wr_seq(&[(0xFF, 0x01), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)]) $($aw)* ?;
            if !done {
                return Err(Error::Timeout);
            }
            Ok(SpadInfo::from_nvm(words[0], words[1], words[2]))
        }

        $($asyncness)? fn set_reference_spads(&mut self, info: &SpadInfo) -> Result<(), Error<I2C::Error>> {
            if !info.is_valid() {
                return Err(Error::Unsupported);
            }
            let map = info.reference_map().ok_or(Error::InvalidData)?;
            self.wr_seq(&[
                (0xFF, 0x01),
                (REG_DYNAMIC_SPAD_START_OFFSET, 0x00),
                (REG_DYNAMIC_SPAD_NUM_REQUESTED, 0x2C),
                (0xFF, 0x00),
                (REG_REF_EN_START_SELECT, REF_START_SELECT),
            ]) $($aw)* ?;
            self.write(REG_SPAD_ENABLES_REF_0, &map) $($aw)* ?;
            let mut check = [0u8; SPAD_MAP_LEN];
            self.read(REG_SPAD_ENABLES_REF_0, &mut check) $($aw)* ?;
            if check != map {
                return Err(Error::InvalidData);
            }
            Ok(())
        }

        /// Read the sequence steps and timeouts.
        pub $($asyncness)? fn read_timing(&mut self) -> Result<Timing, Error<I2C::Error>> {
            let sequence = self.rd8(REG_SEQUENCE_CONFIG) $($aw)* ?;
            let pre_vcsel = self.rd8(REG_PRE_RANGE_VCSEL) $($aw)* ?;
            let final_vcsel = self.rd8(REG_FINAL_RANGE_VCSEL) $($aw)* ?;
            let msrc = self.rd8(REG_MSRC_TIMEOUT) $($aw)* ?;
            let pre = self.rd16(REG_PRE_RANGE_TIMEOUT) $($aw)* ?;
            let final_range = self.rd16(REG_FINAL_RANGE_TIMEOUT) $($aw)* ?;
            Ok(Timing::from_registers(sequence, pre_vcsel, final_vcsel, msrc, pre, final_range))
        }

        /// Set the measurement timing budget (≥ [`MIN_BUDGET_US`]; ≈33 ms
        /// after init, 200 ms for high accuracy). `InvalidArgument` if it
        /// is too short for the enabled sequence steps.
        pub $($asyncness)? fn set_timing_budget(&mut self, budget_us: u32) -> Result<(), Error<I2C::Error>> {
            if budget_us < MIN_BUDGET_US {
                return Err(Error::InvalidArgument);
            }
            let timing = self.read_timing() $($aw)* ?;
            let reg = timing.final_timeout_for(budget_us).ok_or(Error::InvalidArgument)?;
            self.wr16(REG_FINAL_RANGE_TIMEOUT, reg) $($aw)* ?;
            self.budget_us = budget_us;
            Ok(())
        }

        /// Interrupt status set (a ranging or calibration has finished).
        pub $($asyncness)? fn data_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
            Ok(self.rd8(REG_RESULT_INTERRUPT_STATUS) $($aw)* ? & 0x07 != 0)
        }

        /// API `ClearInterruptMask`: up to three attempts, `Timeout` if the
        /// status stays set.
        pub $($asyncness)? fn clear_interrupt(&mut self) -> Result<(), Error<I2C::Error>> {
            for _ in 0..3 {
                self.wr8(REG_INTERRUPT_CLEAR, 0x01) $($aw)* ?;
                self.wr8(REG_INTERRUPT_CLEAR, 0x00) $($aw)* ?;
                if !self.data_ready() $($aw)* ? {
                    return Ok(());
                }
            }
            Err(Error::Timeout)
        }

        /// Start one single-shot ranging (API `StartMeasurement`).
        pub $($asyncness)? fn start_ranging(&mut self) -> Result<(), Error<I2C::Error>> {
            let stop = self.stop_variable;
            self.wr_seq(&[
                (0x80, 0x01),
                (0xFF, 0x01),
                (0x00, 0x00),
                (0x91, stop),
                (0x00, 0x01),
                (0xFF, 0x00),
                (0x80, 0x00),
                (REG_SYSRANGE_START, 0x01),
            ]) $($aw)*
        }

        /// `NotReady` until the ranging has finished; then read the result
        /// and clear the interrupt.
        pub $($asyncness)? fn read_result(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if !self.data_ready() $($aw)* ? {
                return Err(Error::NotReady);
            }
            let mut raw = [0u8; 12];
            self.read(REG_RESULT_RANGE_STATUS, &mut raw) $($aw)* ?;
            self.clear_interrupt() $($aw)* ?;
            Ok(Measurement::decode(&raw))
        }

        $($asyncness)? fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.start_ranging() $($aw)* ?;
            Ok(self.budget_us + self.budget_us / 10)
        }
    };
}

/// Blocking driver.
pub struct Vl53l0x<I2C> {
    i2c: I2C,
    address: u8,
    stop_variable: u8,
    sequence: u8,
    budget_us: u32,
    phase: InitPhase,
}

impl<I2C: embedded_hal::i2c::I2c> Vl53l0x<I2C> {
    api!(;);
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Vl53l0x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Start one ranging; wait the timing budget plus 10 %.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.start()
    }

    /// `NotReady` until the result is ready; then read it.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        self.read_result()
    }
}

/// Async driver (`embedded-hal-async` I2C).
pub mod asynch {
    use super::*;

    /// Async twin of [`super::Vl53l0x`].
    pub struct Vl53l0x<I2C> {
        i2c: I2C,
        address: u8,
        stop_variable: u8,
        sequence: u8,
        budget_us: u32,
        phase: InitPhase,
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Vl53l0x<I2C> {
        api!(async; .await);
    }

    impl<I2C: embedded_hal_async::i2c::I2c> zigbee_sensor::asynch::Sensor for Vl53l0x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.start().await
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            self.read_result().await
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    const A: u8 = DEFAULT_ADDRESS;

    fn rd(reg: u8, data: &[u8]) -> T {
        T::write_read(A, vec![reg], data.to_vec())
    }

    fn wr(reg: u8, data: &[u8]) -> T {
        let mut v = vec![reg];
        v.extend_from_slice(data);
        T::write(A, v)
    }

    fn wrs(seq: &[(u8, u8)]) -> Vec<T> {
        seq.iter().map(|&(r, v)| wr(r, &[v])).collect()
    }

    fn clear() -> [T; 3] {
        [wr(0x0B, &[1]), wr(0x0B, &[0]), rd(0x13, &[0])]
    }

    fn timing_reads(sequence: u8, final_reg: [u8; 2]) -> [T; 6] {
        [
            rd(0x01, &[sequence]),
            rd(0x50, &[0x06]),
            rd(0x70, &[0x04]),
            rd(0x46, &[0x25]),
            rd(0x51, &[0x00, 0x96]),
            rd(0x71, &final_reg),
        ]
    }

    #[test]
    fn timing_math() {
        assert_eq!(decode_timeout(0x01FE), 509);
        assert_eq!(encode_timeout(509), 0x01FE);
        assert_eq!(encode_timeout(578), 0x0290);
        assert_eq!(encode_timeout(0), 0);
        // Registers after the default tuning settings.
        let t = Timing::from_registers(0xF8, 0x06, 0x04, 0x25, 0x0096, 0x01FE);
        assert_eq!((t.pre_vcsel_pclks, t.final_vcsel_pclks), (14, 10));
        assert_eq!(t.budget_us(), 33_849);
        let e8 = Timing {
            sequence: 0xE8,
            ..t
        };
        assert_eq!(e8.final_timeout_for(33_849), Some(0x0290));
        assert_eq!(e8.final_timeout_for(20_000), Some(0x00D5));
        assert_eq!(e8.final_timeout_for(19_999), None);
        assert_eq!(t.final_timeout_for(20_000), None);
        assert_eq!(range_status(11 << 3), 0);
        assert_eq!(range_status(4 << 3), 2);
        assert_eq!(range_status(8 << 3), 3);
        assert_eq!(range_status(9 << 3), 4);
        assert_eq!(range_status(2 << 3), 5);
        assert_eq!(range_status(0), 255);
    }

    #[test]
    fn reference_spad_selection() {
        let info = SpadInfo::from_nvm(0x0000_0500, 0xB5FF_FFFF, 0xFF0F_0000);
        assert_eq!(info.count, 5);
        assert!(!info.aperture && info.is_valid());
        assert_eq!(info.good_map, [0xB5, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F]);
        assert_eq!(info.reference_map(), Some([0xB5, 0, 0, 0, 0, 0]));
        // Aperture SPADs start at index 12.
        let ap = SpadInfo {
            count: 3,
            aperture: true,
            good_map: [0xFF, 0xA0, 0x01, 0, 0, 0],
        };
        assert_eq!(ap.reference_map(), Some([0, 0xA0, 0x01, 0, 0, 0]));
        // The third good SPAD (12) is an aperture SPAD.
        let mixed = SpadInfo {
            count: 3,
            aperture: false,
            good_map: [0x03, 0x10, 0, 0, 0, 0],
        };
        assert_eq!(mixed.reference_map(), None);
        let short = SpadInfo {
            count: 4,
            aperture: true,
            good_map: [0, 0xA0, 0x01, 0, 0, 0],
        };
        assert_eq!(short.reference_map(), None);
        assert!(!SpadInfo { count: 13, ..info }.is_valid());
    }

    #[test]
    fn init_sequence() {
        let mut e: Vec<T> = vec![wr(0x88, &[0])];
        e.extend(wrs(&[(0x80, 1), (0xFF, 1), (0x00, 0)]));
        e.push(rd(0x91, &[0x3C]));
        e.extend(wrs(&[(0x00, 1), (0xFF, 0), (0x80, 0)]));
        e.extend([
            rd(0x60, &[0x01]),
            wr(0x60, &[0x12]),
            wr(0x44, &[0x00, 0x20]),
            wr(0x01, &[0xFF]),
        ]);
        e.extend(wrs(&[(0x80, 1), (0xFF, 1), (0x00, 0), (0xFF, 6)]));
        e.extend([rd(0x83, &[0x01]), wr(0x83, &[0x05])]);
        e.extend(wrs(&[(0xFF, 7), (0x81, 1), (0x80, 1)]));
        let nvm: [(u8, [u8; 4]); 3] = [
            (0x6B, [0x00, 0x00, 0x05, 0x00]),
            (0x24, [0xB5, 0xFF, 0xFF, 0xFF]),
            (0x25, [0xFF, 0x0F, 0x00, 0x00]),
        ];
        for (i, (select, word)) in nvm.iter().enumerate() {
            e.extend([wr(0x94, &[*select]), wr(0x83, &[0])]);
            if i == 0 {
                e.push(rd(0x83, &[0]));
            }
            e.extend([rd(0x83, &[1]), wr(0x83, &[1]), rd(0x90, word)]);
        }
        e.extend(wrs(&[(0x81, 0), (0xFF, 6)]));
        e.extend([rd(0x83, &[0x05]), wr(0x83, &[0x01])]);
        e.extend(wrs(&[(0xFF, 1), (0x00, 1), (0xFF, 0), (0x80, 0)]));
        e.extend(wrs(&[
            (0xFF, 1),
            (0x4F, 0),
            (0x4E, 0x2C),
            (0xFF, 0),
            (0xB6, 0xB4),
        ]));
        let map = [0xB5, 0, 0, 0, 0, 0];
        e.extend([wr(0xB0, &map), rd(0xB0, &map)]);
        e.extend(wrs(&TUNING));
        e.extend([wr(0x0A, &[0x04]), rd(0x84, &[0x11]), wr(0x84, &[0x01])]);
        e.extend(clear());
        e.extend(timing_reads(0xF8, [0x01, 0xFE]));
        e.extend([wr(0x01, &[0xE8]), wr(0x71, &[0x02, 0x90])]);
        e.extend([wr(0x01, &[0x01]), wr(0x00, &[0x41])]);
        // VHV: not ready, then done; phase calibration started.
        e.push(rd(0x13, &[0]));
        e.push(rd(0x13, &[0x04]));
        e.extend(clear());
        e.extend([wr(0x00, &[0]), wr(0x01, &[0x02]), wr(0x00, &[0x01])]);
        e.push(rd(0x13, &[0x04]));
        e.extend(clear());
        e.extend([wr(0x00, &[0]), wr(0x01, &[0xE8])]);

        let mut i2c = Mock::new(&e);
        let mut s = Vl53l0x::new(i2c.clone());
        assert_eq!(s.finish_init(), Err(Error::InvalidData));
        assert_eq!(s.begin_init(), Ok(CAL_WAIT_US));
        assert_eq!(s.timing_budget_us(), 33_849);
        assert_eq!(s.finish_init(), Err(Error::NotReady));
        assert_eq!(s.finish_init(), Err(Error::NotReady));
        s.finish_init().unwrap();
        s.finish_init().unwrap();
        i2c.done();
    }

    #[test]
    fn single_shot() {
        let mut raw = [0u8; 12];
        raw[0] = 11 << 3;
        raw[2..4].copy_from_slice(&[0x0A, 0x00]);
        raw[6..8].copy_from_slice(&[0x01, 0x00]);
        raw[8..10].copy_from_slice(&[0x00, 0x40]);
        raw[10..12].copy_from_slice(&[0x01, 0x2C]);
        let mut e: Vec<T> = timing_reads(0xE8, [0x02, 0x90]).to_vec();
        e.push(wr(0x71, &[0x00, 0xD5]));
        e.extend(wrs(&[
            (0x80, 1),
            (0xFF, 1),
            (0x00, 0),
            (0x91, 0),
            (0x00, 1),
            (0xFF, 0),
            (0x80, 0),
            (0x00, 1),
        ]));
        e.extend([rd(0x13, &[0]), rd(0x13, &[0x04]), rd(0x14, &raw)]);
        e.extend(clear());
        let mut i2c = Mock::new(&e);
        let mut s = Vl53l0x::new(i2c.clone());
        assert_eq!(s.set_timing_budget(19_000), Err(Error::InvalidArgument));
        s.set_timing_budget(20_000).unwrap();
        assert_eq!(s.start_measurement(), Ok(22_000));
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        let m = s.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                range_status: 0,
                device_status: 0x58,
                distance_mm: 300,
                signal_kcps: 2000,
                ambient_kcps: 500,
                spad_count_q8: 0x0A00,
            }
        );
        let mut q = None;
        m.for_each(&mut |x| q = Some(x));
        assert_eq!(q, Some(Quantity::Distance(300)));
        i2c.done();
    }

    #[test]
    fn async_probe_and_address() {
        let mut i2c = Mock::new(&[
            rd(0xC0, &[0xAA]),
            rd(0xC0, &[0xEE]),
            T::write(A, vec![0x8A, 0x30]),
            T::write_read(0x30, vec![0x89], vec![0x00]),
            T::write(0x30, vec![0x89, 0x01]),
        ]);
        let mut s = asynch::Vl53l0x::new(i2c.clone());
        assert_eq!(block_on(s.probe()), Err(Error::UnexpectedId(0xAA)));
        block_on(s.probe()).unwrap();
        block_on(s.set_address(0x30)).unwrap();
        assert_eq!(s.address(), 0x30);
        block_on(s.enable_io_2v8()).unwrap();
        i2c.done();
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
