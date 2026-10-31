//! ST VL53L1X and VL53L4CD time-of-flight ranging sensors.
//!
//! Follows ST's Ultra Lite Drivers (ULD): the VL53L1X ULD (STSW-IMG009,
//! stm32duino `VL53L1X` port) and the VL53L4CD ULD (STSW-IMG026,
//! stm32duino `VL53L4CD` port), both BSD-3-Clause. The default
//! configuration blob, register indices, timing tables and range-status
//! mapping are taken from those sources; their notice is kept below.
//!
//! * [`Variant`] selects the blob, the model ID and the variant-specific
//!   features: distance mode and ROI exist only on the VL53L1X; the
//!   VL53L4CD timing budget is computed from its oscillator.
//! * Initialisation is split so the driver never waits:
//!   [`Vl53l1x::begin_init`] (`NotReady` until the firmware has booted)
//!   loads the blob and starts the VHV calibration ranging,
//!   [`Vl53l1x::finish_init`] (`NotReady` until that ranging ends)
//!   completes it.
//! * `Sensor::start_measurement` starts ranging; `read_measurement`
//!   returns `NotReady` until data is ready, reads the result, clears the
//!   interrupt and stops ranging. As in the ULD, the first VL53L4CD result
//!   after a start is discarded, so its wait covers two periods.
//! * Only range status 0 (valid) is reported as `Quantity::Distance`;
//!   the status and rates are in [`Measurement`].

// Portions (default configuration, register map, timing tables, status
// mapping) derived from ST's VL53L1X and VL53L4CD Ultra Lite Drivers:
//
// COPYRIGHT(c) 2018 STMicroelectronics
// COPYRIGHT(c) 2021 STMicroelectronics
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
/// VL53L1X model ID (0x010F word).
pub const MODEL_ID_VL53L1X: u16 = 0xEACC;
/// VL53L4CD model ID (0x010F word).
pub const MODEL_ID_VL53L4CD: u16 = 0xEBAA;
/// Wait hint after [`Vl53l1x::begin_init`]: the VHV calibration ranging
/// runs with the blob's ≈100 ms timing budget.
pub const INIT_WAIT_US: WaitUs = 100_000;

const REG_I2C_ADDRESS: u16 = 0x0001;
const REG_OSC_FREQUENCY: u16 = 0x0006;
const REG_VHV_LOOP_BOUND: u16 = 0x0008;
const REG_VHV_INIT: u16 = 0x000B;
const REG_XTALK_PLANE_OFFSET: u16 = 0x0016;
const REG_XTALK_X_GRADIENT: u16 = 0x0018;
const REG_XTALK_Y_GRADIENT: u16 = 0x001A;
const REG_RANGE_OFFSET: u16 = 0x001E;
const REG_INNER_OFFSET: u16 = 0x0020;
const REG_OUTER_OFFSET: u16 = 0x0022;
const REG_L4CD_0024: u16 = 0x0024;
const REG_CONFIG_START: u16 = 0x002D;
const REG_GPIO_HV_MUX_CTRL: u16 = 0x0030;
const REG_GPIO_TIO_HV_STATUS: u16 = 0x0031;
const REG_INTERRUPT_CONFIG: u16 = 0x0046;
const REG_PHASECAL_TIMEOUT: u16 = 0x004B;
const REG_TIMEOUT_A: u16 = 0x005E;
const REG_VCSEL_PERIOD_A: u16 = 0x0060;
const REG_TIMEOUT_B: u16 = 0x0061;
const REG_VCSEL_PERIOD_B: u16 = 0x0063;
const REG_SIGMA_THRESH: u16 = 0x0064;
const REG_MIN_COUNT_RATE: u16 = 0x0066;
const REG_VALID_PHASE_HIGH: u16 = 0x0069;
const REG_INTERMEASUREMENT: u16 = 0x006C;
const REG_THRESH_HIGH: u16 = 0x0072;
const REG_THRESH_LOW: u16 = 0x0074;
const REG_WOI_SD0: u16 = 0x0078;
const REG_INITIAL_PHASE_SD0: u16 = 0x007A;
const REG_ROI_CENTRE: u16 = 0x007F;
const REG_ROI_SIZE: u16 = 0x0080;
const REG_INTERRUPT_CLEAR: u16 = 0x0086;
const REG_MODE_START: u16 = 0x0087;
const REG_RESULT_STATUS: u16 = 0x0089;
const REG_OSC_CALIBRATE: u16 = 0x00DE;
const REG_SYSTEM_STATUS: u16 = 0x00E5;
const REG_MODEL_ID: u16 = 0x010F;
const REG_MODE_ROI_CENTRE_SPAD: u16 = 0x013E;

const CONFIG_LEN: usize = 91;
const RESULT_LEN: usize = 17;

/// VL53L4CD default configuration for 0x2D..=0x87 (VL53L4CD ULD).
const DEFAULT_CONFIG: [u8; CONFIG_LEN] = [
    0x12, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x14, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x07, 0x01, 0xF1, 0x05, 0x00, 0xA0, 0x00, 0x80, 0x08, 0x38, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
];

/// Bytes where the VL53L1X ULD blob differs from [`DEFAULT_CONFIG`].
const VL53L1X_CONFIG_PATCH: [(u8, u8); 12] = [
    (0x2D, 0x00),
    (0x30, 0x01),
    (0x4B, 0x0A),
    (0x60, 0x0F),
    (0x63, 0x0D),
    (0x64, 0x01),
    (0x65, 0x68),
    (0x69, 0xB8),
    (0x78, 0x0F),
    (0x79, 0x0D),
    (0x7A, 0x0E),
    (0x7B, 0x0E),
];

/// Hardware result status (low 5 bits) to ULD range status.
const STATUS_MAP: [u8; 24] = [
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11,
    12,
];

/// Supported part.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// VL53L1X (up to 4 m, distance modes, ROI).
    Vl53l1x,
    /// VL53L4CD (up to 1.3 m).
    Vl53l4cd,
}

impl Variant {
    /// Expected model ID.
    pub const fn model_id(self) -> u16 {
        match self {
            Self::Vl53l1x => MODEL_ID_VL53L1X,
            Self::Vl53l4cd => MODEL_ID_VL53L4CD,
        }
    }

    /// Default configuration written to 0x2D..=0x87 by `begin_init`.
    pub const fn default_config(self) -> [u8; CONFIG_LEN] {
        let mut c = DEFAULT_CONFIG;
        if let Self::Vl53l1x = self {
            let mut i = 0;
            while i < VL53L1X_CONFIG_PATCH.len() {
                let (reg, v) = VL53L1X_CONFIG_PATCH[i];
                c[(reg - REG_CONFIG_START as u8) as usize] = v;
                i += 1;
            }
        }
        c
    }
}

/// VL53L1X distance mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DistanceMode {
    /// Up to 1.3 m, better ambient immunity.
    Short,
    /// Up to 4 m (default after init).
    #[default]
    Long,
}

/// Detection window of the distance thresholds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Window {
    /// Distance below `low`.
    Below = 0,
    /// Distance above `high`.
    Above = 1,
    /// Distance below `low` or above `high`.
    Outside = 2,
    /// Distance between `low` and `high`.
    Inside = 3,
}

/// ULD range status from the RESULT__RANGE_STATUS register (0 = valid).
pub const fn range_status(raw: u8) -> u8 {
    let r = (raw & 0x1F) as usize;
    if r < STATUS_MAP.len() {
        STATUS_MAP[r]
    } else {
        255
    }
}

/// VL53L1X RANGE_CONFIG timeouts (A, B) for a timing budget; `None` if
/// the budget is not one of 15 (short only), 20, 33, 50, 100, 200 or
/// 500 ms.
pub const fn vl53l1x_timeouts(mode: DistanceMode, budget_ms: u16) -> Option<(u16, u16)> {
    use DistanceMode::{Long, Short};
    Some(match (mode, budget_ms) {
        (Short, 15) => (0x001D, 0x0027),
        (Short, 20) => (0x0051, 0x006E),
        (Short, 33) => (0x00D6, 0x006E),
        (Short, 50) => (0x01AE, 0x01E8),
        (Short, 100) => (0x02E1, 0x0388),
        (Short, 200) => (0x03E1, 0x0496),
        (Short, 500) => (0x0591, 0x05C1),
        (Long, 20) => (0x001E, 0x0022),
        (Long, 33) => (0x0060, 0x006E),
        (Long, 50) => (0x00AD, 0x00C6),
        (Long, 100) => (0x01CC, 0x01EA),
        (Long, 200) => (0x02D9, 0x02F8),
        (Long, 500) => (0x048F, 0x04A4),
        _ => return None,
    })
}

const fn encode_timeout(budget_us_q12: u32, macro_x: u32) -> Option<u16> {
    let div = macro_x >> 6;
    if div == 0 {
        return None;
    }
    let mut ls = (budget_us_q12.wrapping_add(div >> 1) / div).wrapping_sub(1);
    let mut ms = 0u16;
    while ls & 0xFFFF_FF00 != 0 {
        ls >>= 1;
        ms += 1;
    }
    Some((ms << 8) | (ls & 0xFF) as u16)
}

/// VL53L4CD RANGE_CONFIG timeouts (A, B) from the oscillator register
/// (0x0006), a 10–200 ms budget and the inter-measurement period
/// (0 = continuous, otherwise it must exceed the budget).
pub const fn vl53l4cd_timeouts(osc: u16, budget_ms: u32, inter_ms: u32) -> Option<(u16, u16)> {
    if osc == 0 || budget_ms < 10 || budget_ms > 200 {
        return None;
    }
    let macro_us = 2304u32.wrapping_mul(0x4000_0000 / osc as u32) >> 6;
    let budget_us = budget_ms * 1000;
    let budget_us = if inter_ms == 0 {
        budget_us - 2500
    } else if inter_ms > budget_ms {
        (budget_us - 4300) / 2
    } else {
        return None;
    };
    let q12 = budget_us << 12;
    let (Some(a), Some(b)) = (
        encode_timeout(q12, macro_us.wrapping_mul(16)),
        encode_timeout(q12, macro_us.wrapping_mul(12)),
    ) else {
        return None;
    };
    Some((a, b))
}

/// INTERMEASUREMENT register value: `clock_pll × ms × factor`, with the
/// ULD factors 1.075 (VL53L1X) and 1.055 (VL53L4CD) in integer form.
pub const fn intermeasurement(variant: Variant, osc_calibrate: u16, ms: u32) -> u32 {
    let factor = match variant {
        Variant::Vl53l1x => 1075,
        Variant::Vl53l4cd => 1055,
    };
    ((osc_calibrate & 0x3FF) as u64 * ms as u64 * factor / 1000) as u32
}

/// One ranging result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// ULD range status: 0 valid, 1 sigma, 2 signal, 3 below minimum
    /// range, 4 phase out of limits, 5 hardware fail, 6 no wrap-around
    /// check, 7 wrap-around, 9–13 other, 255 unknown/no SPADs.
    pub range_status: u8,
    /// Distance, mm.
    pub distance_mm: u16,
    /// Return signal rate, kcps.
    pub signal_kcps: u32,
    /// Ambient rate, kcps.
    pub ambient_kcps: u32,
    /// Number of enabled SPADs.
    pub spad_count: u16,
    /// Estimated sigma, mm (VL53L4CD only).
    pub sigma_mm: Option<u16>,
}

impl Measurement {
    /// Decode the 0x0089..=0x0099 result block.
    pub fn decode(variant: Variant, raw: &[u8; RESULT_LEN]) -> Self {
        let word = |i: usize| u16::from_be_bytes([raw[i], raw[i + 1]]);
        let spad_count = word(3) >> 8;
        let mut range_status = range_status(raw[0]);
        let (signal, sigma_mm) = match variant {
            Variant::Vl53l1x => (word(15), None),
            Variant::Vl53l4cd => {
                if spad_count == 0 {
                    range_status = 255;
                }
                (word(5), Some(word(9) / 4))
            }
        };
        Self {
            range_status,
            distance_mm: word(13),
            signal_kcps: signal as u32 * 8,
            ambient_kcps: word(7) as u32 * 8,
            spad_count,
            sigma_mm,
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

macro_rules! api {
    ($($asyncness:ident)?; $($aw:tt)*) => {
        /// Create a driver at [`DEFAULT_ADDRESS`]; no bus traffic.
        pub const fn new(i2c: I2C, variant: Variant) -> Self {
            Self::with_address(i2c, variant, DEFAULT_ADDRESS)
        }

        /// Create a driver at another 7-bit address (after `set_address`).
        pub const fn with_address(i2c: I2C, variant: Variant, address: u8) -> Self {
            let budget_ms = match variant {
                Variant::Vl53l1x => 100,
                Variant::Vl53l4cd => 50,
            };
            Self {
                i2c,
                address,
                variant,
                mode: DistanceMode::Long,
                budget_ms,
                inter_ms: 0,
                discard: false,
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

        $($asyncness)? fn write(&mut self, index: u16, data: &[u8]) -> Result<(), Error<I2C::Error>> {
            let mut buf = [0u8; 2 + CONFIG_LEN];
            buf[..2].copy_from_slice(&index.to_be_bytes());
            buf[2..2 + data.len()].copy_from_slice(data);
            self.i2c.write(self.address, &buf[..2 + data.len()]) $($aw)* .map_err(Error::Bus)
        }

        $($asyncness)? fn read(&mut self, index: u16, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c.write_read(self.address, &index.to_be_bytes(), buf) $($aw)* .map_err(Error::Bus)
        }

        $($asyncness)? fn wr8(&mut self, index: u16, v: u8) -> Result<(), Error<I2C::Error>> {
            self.write(index, &[v]) $($aw)*
        }

        $($asyncness)? fn wr16(&mut self, index: u16, v: u16) -> Result<(), Error<I2C::Error>> {
            self.write(index, &v.to_be_bytes()) $($aw)*
        }

        $($asyncness)? fn rd8(&mut self, index: u16) -> Result<u8, Error<I2C::Error>> {
            let mut b = [0u8];
            self.read(index, &mut b) $($aw)* ?;
            Ok(b[0])
        }

        $($asyncness)? fn rd16(&mut self, index: u16) -> Result<u16, Error<I2C::Error>> {
            let mut b = [0u8; 2];
            self.read(index, &mut b) $($aw)* ?;
            Ok(u16::from_be_bytes(b))
        }

        /// Check the model ID of the configured variant.
        pub $($asyncness)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            let id = self.rd16(REG_MODEL_ID) $($aw)* ?;
            if id != self.variant.model_id() {
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

        /// `NotReady` until the firmware has booted (ULD waits up to 1 s);
        /// then load the default configuration and start the VHV
        /// calibration ranging. Call [`Self::finish_init`] after
        /// [`INIT_WAIT_US`].
        pub $($asyncness)? fn begin_init(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.rd8(REG_SYSTEM_STATUS) $($aw)* ? != 3 {
                return Err(Error::NotReady);
            }
            let config = self.variant.default_config();
            self.write(REG_CONFIG_START, &config) $($aw)* ?;
            self.wr8(REG_MODE_START, 0x40) $($aw)* ?;
            self.discard = false;
            Ok(INIT_WAIT_US)
        }

        /// `NotReady` until the calibration ranging has finished; then
        /// stop it and apply the ULD post-init settings (VL53L1X: long
        /// mode, ≈100 ms; VL53L4CD: 50 ms continuous).
        pub $($asyncness)? fn finish_init(&mut self) -> Result<(), Error<I2C::Error>> {
            if !self.data_ready() $($aw)* ? {
                return Err(Error::NotReady);
            }
            self.clear_interrupt() $($aw)* ?;
            self.stop_ranging() $($aw)* ?;
            self.wr8(REG_VHV_LOOP_BOUND, 0x09) $($aw)* ?;
            self.wr8(REG_VHV_INIT, 0) $($aw)* ?;
            self.mode = DistanceMode::Long;
            self.inter_ms = 0;
            match self.variant {
                Variant::Vl53l1x => self.budget_ms = 100,
                Variant::Vl53l4cd => {
                    self.wr16(REG_L4CD_0024, 0x0500) $($aw)* ?;
                    self.set_timing(50, 0) $($aw)* ?;
                }
            }
            Ok(())
        }

        /// New result available (GPIO1 status at the configured polarity).
        pub $($asyncness)? fn data_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
            let active_high = self.rd8(REG_GPIO_HV_MUX_CTRL) $($aw)* ? & 0x10 == 0;
            let status = self.rd8(REG_GPIO_TIO_HV_STATUS) $($aw)* ? & 1 == 1;
            Ok(status == active_high)
        }

        /// Acknowledge the result so the next one can be latched.
        pub $($asyncness)? fn clear_interrupt(&mut self) -> Result<(), Error<I2C::Error>> {
            self.wr8(REG_INTERRUPT_CLEAR, 0x01) $($aw)*
        }

        /// Start ranging (continuous or timed/autonomous).
        pub $($asyncness)? fn start_ranging(&mut self) -> Result<(), Error<I2C::Error>> {
            let continuous = self.variant == Variant::Vl53l4cd && self.inter_ms == 0;
            self.wr8(REG_MODE_START, if continuous { 0x21 } else { 0x40 }) $($aw)* ?;
            self.discard = self.variant == Variant::Vl53l4cd;
            Ok(())
        }

        /// Stop ranging (software standby).
        pub $($asyncness)? fn stop_ranging(&mut self) -> Result<(), Error<I2C::Error>> {
            self.wr8(REG_MODE_START, 0x00) $($aw)*
        }

        /// While ranging: `NotReady` until a result is ready, then read it
        /// and clear the interrupt. The first VL53L4CD result after
        /// `start_ranging` is dropped (ULD behaviour) and yields `NotReady`.
        pub $($asyncness)? fn read_result(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if !self.data_ready() $($aw)* ? {
                return Err(Error::NotReady);
            }
            if self.discard {
                self.discard = false;
                self.clear_interrupt() $($aw)* ?;
                return Err(Error::NotReady);
            }
            let mut raw = [0u8; RESULT_LEN];
            self.read(REG_RESULT_STATUS, &mut raw) $($aw)* ?;
            self.clear_interrupt() $($aw)* ?;
            Ok(Measurement::decode(self.variant, &raw))
        }

        /// VL53L1X distance mode; re-applies the timing budget.
        /// `Unsupported` on the VL53L4CD, `InvalidData` if the current
        /// budget (15 ms) does not exist in long mode.
        pub $($asyncness)? fn set_distance_mode(&mut self, mode: DistanceMode) -> Result<(), Error<I2C::Error>> {
            if self.variant != Variant::Vl53l1x {
                return Err(Error::Unsupported);
            }
            let (a, b) = vl53l1x_timeouts(mode, self.budget_ms).ok_or(Error::InvalidData)?;
            let (phasecal, vcsel_a, vcsel_b, phase_high, woi, initial) = match mode {
                DistanceMode::Short => (0x14, 0x07, 0x05, 0x38, 0x0705, 0x0606),
                DistanceMode::Long => (0x0A, 0x0F, 0x0D, 0xB8, 0x0F0D, 0x0E0E),
            };
            self.wr8(REG_PHASECAL_TIMEOUT, phasecal) $($aw)* ?;
            self.wr8(REG_VCSEL_PERIOD_A, vcsel_a) $($aw)* ?;
            self.wr8(REG_VCSEL_PERIOD_B, vcsel_b) $($aw)* ?;
            self.wr8(REG_VALID_PHASE_HIGH, phase_high) $($aw)* ?;
            self.wr16(REG_WOI_SD0, woi) $($aw)* ?;
            self.wr16(REG_INITIAL_PHASE_SD0, initial) $($aw)* ?;
            self.wr16(REG_TIMEOUT_A, a) $($aw)* ?;
            self.wr16(REG_TIMEOUT_B, b) $($aw)* ?;
            self.mode = mode;
            Ok(())
        }

        /// Timing budget and inter-measurement period (call while
        /// stopped). `inter_ms = 0` ranges back-to-back (VL53L4CD
        /// continuous mode); otherwise it must exceed the budget on the
        /// VL53L4CD and be at least the budget on the VL53L1X.
        /// VL53L1X budgets: 15 (short), 20, 33, 50, 100, 200, 500 ms;
        /// VL53L4CD: 10–200 ms. `InvalidData` otherwise.
        pub $($asyncness)? fn set_timing(&mut self, budget_ms: u16, inter_ms: u32) -> Result<(), Error<I2C::Error>> {
            let (a, b) = match self.variant {
                Variant::Vl53l1x => {
                    if inter_ms != 0 && inter_ms < budget_ms as u32 {
                        return Err(Error::InvalidData);
                    }
                    vl53l1x_timeouts(self.mode, budget_ms).ok_or(Error::InvalidData)?
                }
                Variant::Vl53l4cd => {
                    let osc = self.rd16(REG_OSC_FREQUENCY) $($aw)* ?;
                    vl53l4cd_timeouts(osc, budget_ms as u32, inter_ms).ok_or(Error::InvalidData)?
                }
            };
            let period = match (self.variant, inter_ms) {
                (Variant::Vl53l1x, 0) => budget_ms as u32,
                (Variant::Vl53l4cd, 0) => 0,
                (_, ms) => ms,
            };
            let im = if period == 0 {
                0
            } else {
                let clock = self.rd16(REG_OSC_CALIBRATE) $($aw)* ?;
                intermeasurement(self.variant, clock, period)
            };
            self.write(REG_INTERMEASUREMENT, &im.to_be_bytes()) $($aw)* ?;
            self.wr16(REG_TIMEOUT_A, a) $($aw)* ?;
            self.wr16(REG_TIMEOUT_B, b) $($aw)* ?;
            self.budget_ms = budget_ms;
            self.inter_ms = inter_ms;
            Ok(())
        }

        /// Range offset correction, mm.
        pub $($asyncness)? fn set_offset(&mut self, mm: i16) -> Result<(), Error<I2C::Error>> {
            self.wr16(REG_RANGE_OFFSET, (mm as u16).wrapping_mul(4)) $($aw)* ?;
            self.wr16(REG_INNER_OFFSET, 0) $($aw)* ?;
            self.wr16(REG_OUTER_OFFSET, 0) $($aw)*
        }

        /// Crosstalk correction, counts per second.
        pub $($asyncness)? fn set_xtalk(&mut self, cps: u16) -> Result<(), Error<I2C::Error>> {
            self.wr16(REG_XTALK_X_GRADIENT, 0) $($aw)* ?;
            self.wr16(REG_XTALK_Y_GRADIENT, 0) $($aw)* ?;
            self.wr16(REG_XTALK_PLANE_OFFSET, (((cps as u32) << 9) / 1000) as u16) $($aw)*
        }

        /// Raise the interrupt (data ready) only for distances in
        /// `window`. Replaces the new-sample-ready interrupt.
        pub $($asyncness)? fn set_thresholds(&mut self, low_mm: u16, high_mm: u16, window: Window) -> Result<(), Error<I2C::Error>> {
            let config = match self.variant {
                Variant::Vl53l1x => (self.rd8(REG_INTERRUPT_CONFIG) $($aw)* ? & 0x40) | window as u8,
                Variant::Vl53l4cd => window as u8,
            };
            self.wr8(REG_INTERRUPT_CONFIG, config) $($aw)* ?;
            self.wr16(REG_THRESH_HIGH, high_mm) $($aw)* ?;
            self.wr16(REG_THRESH_LOW, low_mm) $($aw)*
        }

        /// Minimum return signal rate for a valid result, kcps.
        pub $($asyncness)? fn set_signal_threshold(&mut self, kcps: u16) -> Result<(), Error<I2C::Error>> {
            self.wr16(REG_MIN_COUNT_RATE, kcps >> 3) $($aw)*
        }

        /// Maximum sigma for a valid result, mm (≤ 16383).
        pub $($asyncness)? fn set_sigma_threshold(&mut self, mm: u16) -> Result<(), Error<I2C::Error>> {
            if mm > 0x3FFF {
                return Err(Error::InvalidData);
            }
            self.wr16(REG_SIGMA_THRESH, mm << 2) $($aw)*
        }

        /// VL53L1X region of interest, `width` × `height` SPADs (4–16,
        /// larger values clamp to 16) around the optical centre.
        /// `Unsupported` on the VL53L4CD.
        pub $($asyncness)? fn set_roi(&mut self, width: u8, height: u8) -> Result<(), Error<I2C::Error>> {
            if self.variant != Variant::Vl53l1x {
                return Err(Error::Unsupported);
            }
            if width < 4 || height < 4 {
                return Err(Error::InvalidData);
            }
            let (w, h) = (width.min(16), height.min(16));
            let mut centre = self.rd8(REG_MODE_ROI_CENTRE_SPAD) $($aw)* ?;
            if w > 10 || h > 10 {
                centre = 199;
            }
            self.wr8(REG_ROI_CENTRE, centre) $($aw)* ?;
            self.wr8(REG_ROI_SIZE, ((h - 1) << 4) | (w - 1)) $($aw)*
        }

        fn period_us(&self) -> WaitUs {
            let ms = match self.variant {
                Variant::Vl53l1x => (self.budget_ms as u32).max(self.inter_ms),
                Variant::Vl53l4cd if self.inter_ms == 0 => self.budget_ms as u32,
                Variant::Vl53l4cd => self.inter_ms,
            };
            ms * 1000
        }

        $($asyncness)? fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.start_ranging() $($aw)* ?;
            let n = if self.discard { 2 } else { 1 };
            let us = self.period_us() * n;
            Ok(us + us / 10)
        }

        $($asyncness)? fn read_once(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let m = self.read_result() $($aw)* ?;
            self.stop_ranging() $($aw)* ?;
            Ok(m)
        }
    };
}

/// Blocking driver.
pub struct Vl53l1x<I2C> {
    i2c: I2C,
    address: u8,
    variant: Variant,
    mode: DistanceMode,
    budget_ms: u16,
    inter_ms: u32,
    discard: bool,
}

impl<I2C: embedded_hal::i2c::I2c> Vl53l1x<I2C> {
    api!(;);
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Vl53l1x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Start ranging; wait one ranging period (two on the VL53L4CD)
    /// plus 10 %.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.start()
    }

    /// `NotReady` until a result is ready; then read it and stop ranging.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        self.read_once()
    }
}

/// Async driver (`embedded-hal-async` I2C).
pub mod asynch {
    use super::*;

    /// Async twin of [`super::Vl53l1x`].
    pub struct Vl53l1x<I2C> {
        i2c: I2C,
        address: u8,
        variant: Variant,
        mode: DistanceMode,
        budget_ms: u16,
        inter_ms: u32,
        discard: bool,
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Vl53l1x<I2C> {
        api!(async; .await);
    }

    impl<I2C: embedded_hal_async::i2c::I2c> zigbee_sensor::asynch::Sensor for Vl53l1x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.start().await
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            self.read_once().await
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

    fn rd(index: u16, data: &[u8]) -> T {
        T::write_read(A, index.to_be_bytes().to_vec(), data.to_vec())
    }

    fn wr(index: u16, data: &[u8]) -> T {
        let mut v = index.to_be_bytes().to_vec();
        v.extend_from_slice(data);
        T::write(A, v)
    }

    fn ready(polarity_reg: u8, status: u8) -> [T; 2] {
        [rd(0x30, &[polarity_reg]), rd(0x31, &[status])]
    }

    #[test]
    fn conversions() {
        assert_eq!(range_status(0x09), 0);
        assert_eq!(range_status(0xE9), 0);
        assert_eq!(range_status(0x03), 5);
        assert_eq!(range_status(0x13), 6);
        assert_eq!(range_status(0x1F), 255);
        // Reference values from ST's C code for osc = 0x0BB8.
        assert_eq!(vl53l4cd_timeouts(0x0BB8, 50, 0), Some((0x003B, 0x0050)));
        assert_eq!(vl53l4cd_timeouts(0x0BB8, 200, 1000), Some((0x007B, 0x00A5)));
        assert_eq!(vl53l4cd_timeouts(0x0BB8, 9, 0), None);
        assert_eq!(vl53l4cd_timeouts(0x0BB8, 50, 50), None);
        assert_eq!(vl53l4cd_timeouts(0, 50, 0), None);
        assert_eq!(intermeasurement(Variant::Vl53l4cd, 0xFDA5, 1000), 444_155);
        assert_eq!(intermeasurement(Variant::Vl53l1x, 0x01A5, 200), 90_515);
        assert_eq!(vl53l1x_timeouts(DistanceMode::Long, 15), None);
        let l1x = Variant::Vl53l1x.default_config();
        let l4cd = Variant::Vl53l4cd.default_config();
        assert_eq!((l1x[0], l1x[0x4B - 0x2D], l1x[90]), (0x00, 0x0A, 0x00));
        assert_eq!((l4cd[0], l4cd[0x4B - 0x2D]), (0x12, 0x14));
        assert_eq!(
            l1x.iter().zip(l4cd.iter()).filter(|(a, b)| a != b).count(),
            12
        );
    }

    #[test]
    fn vl53l1x_init_sequence() {
        let blob = Variant::Vl53l1x.default_config().to_vec();
        let mut expect = vec![
            rd(0xE5, &[2]),
            rd(0xE5, &[3]),
            wr(0x2D, &blob),
            wr(0x87, &[0x40]),
        ];
        expect.extend(ready(0x01, 0x00));
        expect.extend(ready(0x01, 0x01));
        expect.extend([
            wr(0x86, &[1]),
            wr(0x87, &[0]),
            wr(0x08, &[0x09]),
            wr(0x0B, &[0]),
        ]);
        let mut i2c = Mock::new(&expect);
        let mut s = Vl53l1x::new(i2c.clone(), Variant::Vl53l1x);
        assert_eq!(s.begin_init(), Err(Error::NotReady));
        assert_eq!(s.begin_init(), Ok(INIT_WAIT_US));
        assert_eq!(s.finish_init(), Err(Error::NotReady));
        s.finish_init().unwrap();
        i2c.done();
    }

    #[test]
    fn vl53l4cd_measurement_discards_first_result() {
        let mut raw = [0u8; RESULT_LEN];
        raw[0] = 0x09;
        raw[3..5].copy_from_slice(&[0x0C, 0x00]);
        raw[5..7].copy_from_slice(&[0x01, 0x00]);
        raw[7..9].copy_from_slice(&[0x00, 0x10]);
        raw[9..11].copy_from_slice(&[0x00, 0x14]);
        raw[13..15].copy_from_slice(&[0x01, 0x2C]);
        let mut expect: Vec<T> = vec![
            rd(0x06, &[0x0B, 0xB8]),
            wr(0x6C, &[0, 0, 0, 0]),
            wr(0x5E, &[0x00, 0x3B]),
            wr(0x61, &[0x00, 0x50]),
            wr(0x87, &[0x21]),
        ];
        // Active-low interrupt (bit 4 set): status 0 means ready.
        expect.extend(ready(0x11, 0x01));
        expect.extend(ready(0x11, 0x00));
        expect.push(wr(0x86, &[1]));
        expect.extend(ready(0x11, 0x00));
        expect.extend([rd(0x89, &raw), wr(0x86, &[1]), wr(0x87, &[0])]);
        let mut i2c = Mock::new(&expect);
        let mut s = Vl53l1x::new(i2c.clone(), Variant::Vl53l4cd);
        s.set_timing(50, 0).unwrap();
        assert_eq!(s.start_measurement(), Ok(110_000));
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        let m = s.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                range_status: 0,
                distance_mm: 300,
                signal_kcps: 2048,
                ambient_kcps: 128,
                spad_count: 12,
                sigma_mm: Some(5),
            }
        );
        let mut q = None;
        m.for_each(&mut |x| q = Some(x));
        assert_eq!(q, Some(Quantity::Distance(300)));
        i2c.done();
    }

    #[test]
    fn vl53l1x_short_mode_timing_and_roi() {
        let mut i2c = Mock::new(&[
            wr(0x4B, &[0x14]),
            wr(0x60, &[0x07]),
            wr(0x63, &[0x05]),
            wr(0x69, &[0x38]),
            wr(0x78, &[0x07, 0x05]),
            wr(0x7A, &[0x06, 0x06]),
            wr(0x5E, &[0x02, 0xE1]),
            wr(0x61, &[0x03, 0x88]),
            rd(0xDE, &[0x01, 0xA5]),
            wr(0x6C, &[0x00, 0x00, 0x1A, 0x84]),
            wr(0x5E, &[0x00, 0x1D]),
            wr(0x61, &[0x00, 0x27]),
            rd(0x013E, &[0x91]),
            wr(0x7F, &[199]),
            wr(0x80, &[0x3F]),
            wr(0x87, &[0x40]),
        ]);
        let mut s = Vl53l1x::new(i2c.clone(), Variant::Vl53l1x);
        s.set_distance_mode(DistanceMode::Short).unwrap();
        s.set_timing(15, 0).unwrap();
        assert_eq!(
            s.set_distance_mode(DistanceMode::Long),
            Err(Error::InvalidData)
        );
        assert_eq!(s.set_timing(50, 20), Err(Error::InvalidData));
        assert_eq!(s.set_roi(3, 8), Err(Error::InvalidData));
        s.set_roi(16, 4).unwrap();
        assert_eq!(s.start_measurement(), Ok(16_500));
        i2c.done();
    }

    #[test]
    fn async_probe_and_l4cd_limits() {
        let mut i2c = Mock::new(&[rd(0x010F, &[0xEA, 0xCC]), rd(0x010F, &[0xEB, 0xAA])]);
        let mut s = asynch::Vl53l1x::new(i2c.clone(), Variant::Vl53l4cd);
        assert_eq!(block_on(s.probe()), Err(Error::UnexpectedId(0xEACC)));
        block_on(s.probe()).unwrap();
        assert_eq!(
            block_on(s.set_distance_mode(DistanceMode::Short)),
            Err(Error::Unsupported)
        );
        assert_eq!(block_on(s.set_roi(8, 8)), Err(Error::Unsupported));
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
