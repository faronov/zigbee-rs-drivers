//! Sensirion Gas Index Algorithm (VOC and NOx index), fixed point.
//!
//! Rust port of `sensirion_gas_index_algorithm_fixpoint` (algorithm version
//! 3.2, library 3.2.2) from <https://github.com/Sensirion/gas-index-algorithm>.
//! It uses the same 16.16 fixed-point arithmetic (originally from libfixmath)
//! and produces bit-identical results; constants are the precomputed `F16()`
//! values of the C sources. The algorithm expects one raw sample per second.
//!
//! Copyright (c) 2022, Sensirion AG
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions are met:
//!
//! * Redistributions of source code must retain the above copyright notice,
//!   this list of conditions and the following disclaimer.
//!
//! * Redistributions in binary form must reproduce the above copyright notice,
//!   this list of conditions and the following disclaimer in the documentation
//!   and/or other materials provided with the distribution.
//!
//! * Neither the name of Sensirion AG nor the names of its
//!   contributors may be used to endorse or promote products derived from
//!   this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//! POSSIBILITY OF SUCH DAMAGE.

use core::ops::{Add, Div, Mul, Neg, Sub};

/// 16.16 fixed-point number with libfixmath `fix16_t` semantics: wrapping
/// addition, rounded multiplication/division returning `0x80000000` on
/// overflow.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Fix(i32);

const fn int(n: i32) -> Fix {
    Fix(n * 0x1_0000)
}

impl Fix {
    const OVERFLOW: Fix = Fix(i32::MIN);

    fn to_int(self) -> i32 {
        if self.0 >= 0 {
            self.0 >> 16
        } else {
            (self.0.wrapping_neg() >> 16).wrapping_neg()
        }
    }

    fn sqrt(self) -> Fix {
        let mut num = self.0 as u32;
        let mut result: u32 = 0;
        let mut bit: u32 = 1 << 30;
        while bit > num {
            bit >>= 2;
        }
        for n in 0..2 {
            while bit != 0 {
                if num >= result + bit {
                    num -= result + bit;
                    result = (result >> 1) + bit;
                } else {
                    result >>= 1;
                }
                bit >>= 2;
            }
            if n == 0 {
                if num > 65_535 {
                    num -= result;
                    num = (num << 16).wrapping_sub(0x8000);
                    result = (result << 16) + 0x8000;
                } else {
                    num <<= 16;
                    result <<= 16;
                }
                bit = 1 << 14;
            }
        }
        if num > result {
            result += 1;
        }
        Fix(result as i32)
    }

    fn exp(self) -> Fix {
        // exp(±x) for x = 1, 1/8, 1/64, 1/512.
        const POS: [Fix; 4] = [Fix(178_145), Fix(74_262), Fix(66_568), Fix(65_664)];
        const NEG: [Fix; 4] = [Fix(24_109), Fix(57_835), Fix(64_520), Fix(65_408)];
        let mut x = self;
        if x >= Fix(681_391) {
            // 10.3972
            return Fix(i32::MAX);
        }
        if x <= Fix(-772_243) {
            // -11.7835
            return Fix(0);
        }
        let table = if x < Fix(0) {
            x = -x;
            &NEG
        } else {
            &POS
        };
        let mut res = ONE;
        let mut arg = ONE;
        for &value in table {
            while x >= arg {
                res = res * value;
                x = x - arg;
            }
            arg = Fix(arg.0 >> 3);
        }
        res
    }
}

impl Add for Fix {
    type Output = Fix;
    fn add(self, rhs: Fix) -> Fix {
        Fix(self.0.wrapping_add(rhs.0))
    }
}

impl Sub for Fix {
    type Output = Fix;
    fn sub(self, rhs: Fix) -> Fix {
        Fix(self.0.wrapping_sub(rhs.0))
    }
}

impl Neg for Fix {
    type Output = Fix;
    fn neg(self) -> Fix {
        Fix(self.0.wrapping_neg())
    }
}

impl Mul for Fix {
    type Output = Fix;
    fn mul(self, rhs: Fix) -> Fix {
        let product = u64::from(self.0.unsigned_abs()) * u64::from(rhs.0.unsigned_abs());
        if product >> 47 != 0 {
            return Fix::OVERFLOW;
        }
        let result = ((product + 0x8000) >> 16) as u32 as i32;
        if (self.0 < 0) != (rhs.0 < 0) {
            Fix(result.wrapping_neg())
        } else {
            Fix(result)
        }
    }
}

impl Div for Fix {
    type Output = Fix;
    fn div(self, rhs: Fix) -> Fix {
        if rhs.0 == 0 {
            return Fix::OVERFLOW;
        }
        let mut remainder = self.0.unsigned_abs();
        let mut divider = rhs.0.unsigned_abs();
        let mut quotient: u32 = 0;
        let mut bit: u32 = 0x1_0000;
        while divider < remainder {
            divider <<= 1;
            bit <<= 1;
        }
        if bit == 0 {
            return Fix::OVERFLOW;
        }
        if divider & 0x8000_0000 != 0 {
            if remainder >= divider {
                quotient |= bit;
                remainder -= divider;
            }
            divider >>= 1;
            bit >>= 1;
        }
        while bit != 0 && remainder != 0 {
            if remainder >= divider {
                quotient |= bit;
                remainder -= divider;
            }
            remainder <<= 1;
            bit >>= 1;
        }
        if remainder >= divider {
            quotient = quotient.wrapping_add(1);
        }
        let result = quotient as i32;
        if (self.0 < 0) != (rhs.0 < 0) {
            if result == i32::MIN {
                return Fix::OVERFLOW;
            }
            Fix(-result)
        } else {
            Fix(result)
        }
    }
}

const ZERO: Fix = Fix(0);
const ONE: Fix = int(1);
const HALF: Fix = Fix(32_768);
const SAMPLING_INTERVAL: Fix = ONE; // 1 s
const INITIAL_BLACKOUT: Fix = int(45);
const INDEX_GAIN: Fix = int(230);
const SRAW_STD_INITIAL: Fix = int(50);
const SRAW_STD_BONUS_VOC: Fix = int(220);
const SRAW_STD_NOX: Fix = int(2000);
const TAU_HOURS: Fix = int(12);
const INIT_DURATION_MEAN_VOC: Fix = Fix(176_947_200); // 3600 * 0.75
const INIT_DURATION_MEAN_NOX: Fix = Fix(1_120_665_600); // 3600 * 4.75
const INIT_TRANSITION: Fix = Fix(655); // 0.01
const INIT_DURATION_VARIANCE_VOC: Fix = Fix(342_097_920); // 3600 * 1.45
const INIT_DURATION_VARIANCE_NOX: Fix = Fix(1_344_798_720); // 3600 * 5.70
const GATING_THRESHOLD_VOC: Fix = int(340);
const GATING_THRESHOLD_NOX: Fix = int(30);
const GATING_THRESHOLD_INITIAL: Fix = int(510);
const GATING_THRESHOLD_TRANSITION: Fix = Fix(5_898); // 0.09
const GATING_VOC_MAX_DURATION_MINUTES: Fix = int(180);
const GATING_NOX_MAX_DURATION_MINUTES: Fix = int(720);
const GATING_MAX_RATIO: Fix = Fix(19_661); // 0.3
const ONE_PLUS_GATING_MAX_RATIO: Fix = Fix(85_197); // 1.3
const SIGMOID_L: Fix = int(500);
const SIGMOID_K_VOC: Fix = Fix(-426); // -0.0065
const SIGMOID_X0_VOC: Fix = int(213);
const SIGMOID_K_NOX: Fix = Fix(-662); // -0.0101
const SIGMOID_X0_NOX: Fix = int(614);
const VOC_INDEX_OFFSET_DEFAULT: Fix = int(100);
const NOX_INDEX_OFFSET_DEFAULT: Fix = int(1);
const LP_TAU_FAST: Fix = int(20);
const LP_TAU_SLOW_MINUS_FAST: Fix = int(480);
const LP_ALPHA: Fix = Fix(-13_107); // -0.2
const LP_A1: Fix = Fix(3_121); // 1 / (20 + 1)
const LP_A2: Fix = Fix(131); // 1 / (500 + 1)
const VOC_SRAW_MINIMUM: i32 = 20_000;
const NOX_SRAW_MINIMUM: i32 = 10_000;
const PERSISTENCE_UPTIME_GAMMA: Fix = int(3 * 3600);
const GAMMA_SCALING: Fix = int(64);
const ADDITIONAL_GAMMA_MEAN_SCALING: Fix = int(8);
const GAMMA_MEAN_NUMERATOR: Fix = Fix(9_321); // 8 * 64 / 3600
const GAMMA_VARIANCE_NUMERATOR: Fix = Fix(1_165); // 64 / 3600
const SAMPLING_INTERVAL_HOURS: Fix = Fix(18); // 1 / 3600
const GAMMA_INITIAL_MEAN_VOC: Fix = Fix(1_597_830); // 8 * 64 / (20 + 1)
const GAMMA_INITIAL_MEAN_NOX: Fix = Fix(27_939); // 8 * 64 / (1200 + 1)
const GAMMA_INITIAL_VARIANCE: Fix = Fix(1_677); // 64 / (2500 + 1)
const UPTIME_LIMIT: Fix = int(32_767 - 1);
const SAMPLING_INTERVAL_MINUTES: Fix = Fix(1_092); // 1 / 60
const SIGMOID_LIMIT: Fix = int(50);
const NOX_SHIFT_SCALE: Fix = Fix(65_667); // 500 / 499

/// Which index the algorithm computes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Algorithm {
    /// VOC index (average conditions = 100).
    Voc,
    /// NOx index (average conditions = 1).
    Nox,
}

/// Tuning parameters (see `GasIndexAlgorithm_set_tuning_parameters`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TuningParameters {
    /// Index for average conditions, 1–250 (VOC 100, NOx 1).
    pub index_offset: i32,
    /// Offset learning time constant, 1–1000 h (12).
    pub learning_time_offset_hours: i32,
    /// Gain learning time constant, 1–1000 h (12; not used for NOx).
    pub learning_time_gain_hours: i32,
    /// Maximum gating duration, 0–3000 min (VOC 180, NOx 720).
    pub gating_max_duration_minutes: i32,
    /// Initial standard deviation estimate, 10–5000 (50; not used for NOx).
    pub std_initial: i32,
    /// Gain factor, 1–1000 (230).
    pub gain_factor: i32,
}

impl TuningParameters {
    fn in_range(&self) -> bool {
        (1..=250).contains(&self.index_offset)
            && (1..=1000).contains(&self.learning_time_offset_hours)
            && (1..=1000).contains(&self.learning_time_gain_hours)
            && (0..=3000).contains(&self.gating_max_duration_minutes)
            && (10..=5000).contains(&self.std_initial)
            && (1..=1000).contains(&self.gain_factor)
    }
}

/// A tuning parameter is outside the documented range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidTuning;

/// Gas index algorithm state (about 130 bytes).
#[derive(Debug, Clone)]
pub struct GasIndexAlgorithm {
    algorithm: Algorithm,
    index_offset: Fix,
    sraw_minimum: i32,
    gating_max_duration_minutes: Fix,
    init_duration_mean: Fix,
    init_duration_variance: Fix,
    gating_threshold: Fix,
    index_gain: Fix,
    tau_mean_hours: Fix,
    tau_variance_hours: Fix,
    sraw_std_initial: Fix,
    uptime: Fix,
    sraw: Fix,
    gas_index: Fix,
    // Mean/variance estimator.
    mve_initialized: bool,
    mve_mean: Fix,
    mve_sraw_offset: Fix,
    mve_std: Fix,
    mve_gamma_mean_base: Fix,
    mve_gamma_variance_base: Fix,
    mve_gamma_initial_mean: Fix,
    mve_gamma_initial_variance: Fix,
    mve_gamma_mean: Fix,
    mve_gamma_variance: Fix,
    mve_uptime_gamma: Fix,
    mve_uptime_gating: Fix,
    mve_gating_duration_minutes: Fix,
    // MOX model.
    mox_sraw_std: Fix,
    mox_sraw_mean: Fix,
    // Adaptive low-pass filter.
    lp_initialized: bool,
    lp_x1: Fix,
    lp_x2: Fix,
    lp_x3: Fix,
}

impl GasIndexAlgorithm {
    /// Algorithm with default parameters (`GasIndexAlgorithm_init`).
    pub fn new(algorithm: Algorithm) -> Self {
        let voc = algorithm == Algorithm::Voc;
        let mut a = Self {
            algorithm,
            index_offset: if voc {
                VOC_INDEX_OFFSET_DEFAULT
            } else {
                NOX_INDEX_OFFSET_DEFAULT
            },
            sraw_minimum: if voc {
                VOC_SRAW_MINIMUM
            } else {
                NOX_SRAW_MINIMUM
            },
            gating_max_duration_minutes: if voc {
                GATING_VOC_MAX_DURATION_MINUTES
            } else {
                GATING_NOX_MAX_DURATION_MINUTES
            },
            init_duration_mean: if voc {
                INIT_DURATION_MEAN_VOC
            } else {
                INIT_DURATION_MEAN_NOX
            },
            init_duration_variance: if voc {
                INIT_DURATION_VARIANCE_VOC
            } else {
                INIT_DURATION_VARIANCE_NOX
            },
            gating_threshold: if voc {
                GATING_THRESHOLD_VOC
            } else {
                GATING_THRESHOLD_NOX
            },
            index_gain: INDEX_GAIN,
            tau_mean_hours: TAU_HOURS,
            tau_variance_hours: TAU_HOURS,
            sraw_std_initial: SRAW_STD_INITIAL,
            uptime: ZERO,
            sraw: ZERO,
            gas_index: ZERO,
            mve_initialized: false,
            mve_mean: ZERO,
            mve_sraw_offset: ZERO,
            mve_std: ZERO,
            mve_gamma_mean_base: ZERO,
            mve_gamma_variance_base: ZERO,
            mve_gamma_initial_mean: ZERO,
            mve_gamma_initial_variance: ZERO,
            mve_gamma_mean: ZERO,
            mve_gamma_variance: ZERO,
            mve_uptime_gamma: ZERO,
            mve_uptime_gating: ZERO,
            mve_gating_duration_minutes: ZERO,
            mox_sraw_std: ZERO,
            mox_sraw_mean: ZERO,
            lp_initialized: false,
            lp_x1: ZERO,
            lp_x2: ZERO,
            lp_x3: ZERO,
        };
        a.reset();
        a
    }

    /// Reset the learned state, keeping tuning parameters. Call after a
    /// measurement interruption.
    pub fn reset(&mut self) {
        self.uptime = ZERO;
        self.sraw = ZERO;
        self.gas_index = ZERO;
        self.init_instances();
    }

    /// Learned state `(mean, std)` for persistence (VOC only, after at least
    /// 3 h of operation).
    pub fn states(&self) -> (i32, i32) {
        (self.mve_get_mean().0, self.mve_std.0)
    }

    /// Restore a state from [`states`](Self::states) after an interruption
    /// of at most 10 minutes (VOC only); call after `new`/`reset` and any
    /// tuning.
    pub fn set_states(&mut self, state0: i32, state1: i32) {
        self.mve_mean = Fix(state0);
        self.mve_std = Fix(state1);
        self.mve_uptime_gamma = PERSISTENCE_UPTIME_GAMMA;
        self.mve_initialized = true;
        self.mox_sraw_std = self.mve_std;
        self.mox_sraw_mean = self.mve_get_mean();
        self.sraw = Fix(state0);
    }

    /// Current tuning parameters.
    pub fn tuning_parameters(&self) -> TuningParameters {
        TuningParameters {
            index_offset: self.index_offset.to_int(),
            learning_time_offset_hours: self.tau_mean_hours.to_int(),
            learning_time_gain_hours: self.tau_variance_hours.to_int(),
            gating_max_duration_minutes: self.gating_max_duration_minutes.to_int(),
            std_initial: self.sraw_std_initial.to_int(),
            gain_factor: self.index_gain.to_int(),
        }
    }

    /// Apply tuning parameters (re-initialises the estimators, keeps the
    /// uptime).
    pub fn set_tuning_parameters(&mut self, p: TuningParameters) -> Result<(), InvalidTuning> {
        if !p.in_range() {
            return Err(InvalidTuning);
        }
        self.index_offset = int(p.index_offset);
        self.tau_mean_hours = int(p.learning_time_offset_hours);
        self.tau_variance_hours = int(p.learning_time_gain_hours);
        self.gating_max_duration_minutes = int(p.gating_max_duration_minutes);
        self.sraw_std_initial = int(p.std_initial);
        self.index_gain = int(p.gain_factor);
        self.init_instances();
        Ok(())
    }

    /// Process one raw signal (SRAW_VOC or SRAW_NOX) sampled at 1 Hz; returns
    /// the index: 0 during the initial 45 s blackout, then 1–500.
    pub fn process(&mut self, sraw: u16) -> u16 {
        if self.uptime <= INITIAL_BLACKOUT {
            self.uptime = self.uptime + SAMPLING_INTERVAL;
        } else {
            let sraw = i32::from(sraw);
            if sraw > 0 && sraw < 65_000 {
                let sraw = sraw.clamp(self.sraw_minimum + 1, self.sraw_minimum + 32_767);
                self.sraw = int(sraw - self.sraw_minimum);
            }
            self.gas_index = if self.algorithm == Algorithm::Voc || self.mve_initialized {
                let modelled = self.mox_process(self.sraw);
                self.sigmoid_scaled_process(modelled)
            } else {
                self.index_offset
            };
            self.gas_index = self.lowpass_process(self.gas_index);
            if self.gas_index < HALF {
                self.gas_index = HALF;
            }
            if self.sraw > ZERO {
                self.mve_process(self.sraw);
                self.mox_sraw_std = self.mve_std;
                self.mox_sraw_mean = self.mve_get_mean();
            }
        }
        (self.gas_index + HALF)
            .to_int()
            .clamp(0, i32::from(u16::MAX)) as u16
    }

    fn init_instances(&mut self) {
        self.mve_set_parameters();
        self.mox_sraw_std = self.mve_std;
        self.mox_sraw_mean = self.mve_get_mean();
        self.lp_initialized = false;
    }

    fn mve_set_parameters(&mut self) {
        self.mve_initialized = false;
        self.mve_mean = ZERO;
        self.mve_sraw_offset = ZERO;
        self.mve_std = self.sraw_std_initial;
        self.mve_gamma_mean_base =
            GAMMA_MEAN_NUMERATOR / (self.tau_mean_hours + SAMPLING_INTERVAL_HOURS);
        self.mve_gamma_variance_base =
            GAMMA_VARIANCE_NUMERATOR / (self.tau_variance_hours + SAMPLING_INTERVAL_HOURS);
        self.mve_gamma_initial_mean = match self.algorithm {
            Algorithm::Voc => GAMMA_INITIAL_MEAN_VOC,
            Algorithm::Nox => GAMMA_INITIAL_MEAN_NOX,
        };
        self.mve_gamma_initial_variance = GAMMA_INITIAL_VARIANCE;
        self.mve_gamma_mean = ZERO;
        self.mve_gamma_variance = ZERO;
        self.mve_uptime_gamma = ZERO;
        self.mve_uptime_gating = ZERO;
        self.mve_gating_duration_minutes = ZERO;
    }

    fn mve_get_mean(&self) -> Fix {
        self.mve_mean + self.mve_sraw_offset
    }

    fn mve_sigmoid(sample: Fix, x0: Fix, k: Fix) -> Fix {
        let x = k * (sample - x0);
        if x < -SIGMOID_LIMIT {
            ONE
        } else if x > SIGMOID_LIMIT {
            ZERO
        } else {
            ONE / (ONE + x.exp())
        }
    }

    fn mve_calculate_gamma(&mut self) {
        if self.mve_uptime_gamma < UPTIME_LIMIT {
            self.mve_uptime_gamma = self.mve_uptime_gamma + SAMPLING_INTERVAL;
        }
        if self.mve_uptime_gating < UPTIME_LIMIT {
            self.mve_uptime_gating = self.mve_uptime_gating + SAMPLING_INTERVAL;
        }
        let (x0, k) = (self.init_duration_mean, INIT_TRANSITION);
        let sigmoid_gamma_mean = Self::mve_sigmoid(self.mve_uptime_gamma, x0, k);
        let gamma_mean = self.mve_gamma_mean_base
            + (self.mve_gamma_initial_mean - self.mve_gamma_mean_base) * sigmoid_gamma_mean;
        let gating_threshold_mean = self.gating_threshold
            + (GATING_THRESHOLD_INITIAL - self.gating_threshold)
                * Self::mve_sigmoid(self.mve_uptime_gating, x0, k);
        let sigmoid_gating_mean = Self::mve_sigmoid(
            self.gas_index,
            gating_threshold_mean,
            GATING_THRESHOLD_TRANSITION,
        );
        self.mve_gamma_mean = sigmoid_gating_mean * gamma_mean;

        let (x0, k) = (self.init_duration_variance, INIT_TRANSITION);
        let sigmoid_gamma_variance = Self::mve_sigmoid(self.mve_uptime_gamma, x0, k);
        let gamma_variance = self.mve_gamma_variance_base
            + (self.mve_gamma_initial_variance - self.mve_gamma_variance_base)
                * (sigmoid_gamma_variance - sigmoid_gamma_mean);
        let gating_threshold_variance = self.gating_threshold
            + (GATING_THRESHOLD_INITIAL - self.gating_threshold)
                * Self::mve_sigmoid(self.mve_uptime_gating, x0, k);
        let sigmoid_gating_variance = Self::mve_sigmoid(
            self.gas_index,
            gating_threshold_variance,
            GATING_THRESHOLD_TRANSITION,
        );
        self.mve_gamma_variance = sigmoid_gating_variance * gamma_variance;

        self.mve_gating_duration_minutes = self.mve_gating_duration_minutes
            + SAMPLING_INTERVAL_MINUTES
                * ((ONE - sigmoid_gating_mean) * ONE_PLUS_GATING_MAX_RATIO - GATING_MAX_RATIO);
        if self.mve_gating_duration_minutes < ZERO {
            self.mve_gating_duration_minutes = ZERO;
        }
        if self.mve_gating_duration_minutes > self.gating_max_duration_minutes {
            self.mve_uptime_gating = ZERO;
        }
    }

    fn mve_process(&mut self, sraw: Fix) {
        if !self.mve_initialized {
            self.mve_initialized = true;
            self.mve_sraw_offset = sraw;
            self.mve_mean = ZERO;
            return;
        }
        if self.mve_mean >= int(100) || self.mve_mean <= int(-100) {
            self.mve_sraw_offset = self.mve_sraw_offset + self.mve_mean;
            self.mve_mean = ZERO;
        }
        let sraw = sraw - self.mve_sraw_offset;
        self.mve_calculate_gamma();
        let delta_sgp = (sraw - self.mve_mean) / GAMMA_SCALING;
        let c = if delta_sgp < ZERO {
            self.mve_std - delta_sgp
        } else {
            self.mve_std + delta_sgp
        };
        let mut additional_scaling = ONE;
        if c > int(1440) {
            let ratio = c / int(1440);
            additional_scaling = ratio * ratio;
        }
        let std = self.mve_std;
        self.mve_std = (additional_scaling * (GAMMA_SCALING - self.mve_gamma_variance)).sqrt()
            * (std * (std / (GAMMA_SCALING * additional_scaling))
                + self.mve_gamma_variance * delta_sgp / additional_scaling * delta_sgp)
                .sqrt();
        self.mve_mean =
            self.mve_mean + self.mve_gamma_mean * delta_sgp / ADDITIONAL_GAMMA_MEAN_SCALING;
    }

    fn mox_process(&self, sraw: Fix) -> Fix {
        match self.algorithm {
            Algorithm::Nox => (sraw - self.mox_sraw_mean) / SRAW_STD_NOX * self.index_gain,
            Algorithm::Voc => {
                (sraw - self.mox_sraw_mean) / -(self.mox_sraw_std + SRAW_STD_BONUS_VOC)
                    * self.index_gain
            }
        }
    }

    fn sigmoid_scaled_process(&self, sample: Fix) -> Fix {
        let (x0, k, offset_default) = match self.algorithm {
            Algorithm::Voc => (SIGMOID_X0_VOC, SIGMOID_K_VOC, VOC_INDEX_OFFSET_DEFAULT),
            Algorithm::Nox => (SIGMOID_X0_NOX, SIGMOID_K_NOX, NOX_INDEX_OFFSET_DEFAULT),
        };
        let x = k * (sample - x0);
        if x < -SIGMOID_LIMIT {
            SIGMOID_L
        } else if x > SIGMOID_LIMIT {
            ZERO
        } else if sample >= ZERO {
            let shift = if offset_default == ONE {
                NOX_SHIFT_SCALE * (ONE - self.index_offset)
            } else {
                (SIGMOID_L - int(5) * self.index_offset) / int(4)
            };
            (SIGMOID_L + shift) / (ONE + x.exp()) - shift
        } else {
            self.index_offset / offset_default * (SIGMOID_L / (ONE + x.exp()))
        }
    }

    fn lowpass_process(&mut self, sample: Fix) -> Fix {
        if !self.lp_initialized {
            self.lp_x1 = sample;
            self.lp_x2 = sample;
            self.lp_x3 = sample;
            self.lp_initialized = true;
        }
        self.lp_x1 = (ONE - LP_A1) * self.lp_x1 + LP_A1 * sample;
        self.lp_x2 = (ONE - LP_A2) * self.lp_x2 + LP_A2 * sample;
        let mut abs_delta = self.lp_x1 - self.lp_x2;
        if abs_delta < ZERO {
            abs_delta = -abs_delta;
        }
        let f1 = (LP_ALPHA * abs_delta).exp();
        let tau_a = LP_TAU_SLOW_MINUS_FAST * f1 + LP_TAU_FAST;
        let a3 = SAMPLING_INTERVAL / (SAMPLING_INTERVAL + tau_a);
        self.lp_x3 = (ONE - a3) * self.lp_x3 + a3 * sample;
        self.lp_x3
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Reference values produced by the C fixpoint implementation (3.2.2)
    // for the same deterministic input sequences.
    struct Lcg(u32);

    impl Lcg {
        fn noise(&mut self) -> i32 {
            self.0 = self.0.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            ((self.0 >> 16) % 101) as i32 - 50
        }
    }

    fn run(algorithm: Algorithm) -> ([u16; 16], i64, (i32, i32)) {
        let mut a = GasIndexAlgorithm::new(algorithm);
        let mut lcg = Lcg(12_345);
        let mut spots = [0u16; 16];
        let mut sum = 0i64;
        for i in 0..4000 {
            let base = match algorithm {
                Algorithm::Voc if (1200..1500).contains(&i) => 27_000,
                Algorithm::Voc if (2500..2600).contains(&i) => 25_000,
                Algorithm::Voc => 30_000,
                Algorithm::Nox if (1200..1500).contains(&i) => 17_000,
                Algorithm::Nox => 15_000,
            };
            let index = a.process((base + lcg.noise()) as u16);
            sum += i64::from(index);
            if i % 250 == 249 {
                spots[i / 250] = index;
            }
        }
        (spots, sum, a.states())
    }

    #[test]
    fn fix16_primitives() {
        assert_eq!(int(3) * Fix(-32_768), Fix(-98_304));
        assert_eq!(int(1) / int(3), Fix(21_845));
        assert_eq!(int(-1) / int(3), Fix(-21_845));
        assert_eq!(int(1) / ZERO, Fix::OVERFLOW);
        assert_eq!(int(200) * int(200), Fix::OVERFLOW);
        assert_eq!(int(2).sqrt(), Fix(92_682));
        assert_eq!(ONE.exp(), Fix(178_145));
        assert_eq!(Fix(-98_304).to_int(), -1);
    }

    #[test]
    fn voc_matches_reference() {
        let (spots, sum, states) = run(Algorithm::Voc);
        assert_eq!(
            spots,
            [
                99, 102, 102, 102, 362, 100, 93, 96, 98, 100, 82, 98, 99, 100, 99, 99
            ]
        );
        assert_eq!(sum, 420_600);
        assert_eq!(states, (655_288_772, 12_243_268));
    }

    #[test]
    fn nox_matches_reference() {
        let (spots, sum, states) = run(Algorithm::Nox);
        assert_eq!(spots, [1, 1, 1, 1, 3, 6, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1]);
        assert_eq!(sum, 6_520);
        assert_eq!(states, (331_348_577, 10_453_296));
    }

    #[test]
    fn tuning_and_restored_states_match_reference() {
        let mut a = GasIndexAlgorithm::new(Algorithm::Voc);
        let tuning = TuningParameters {
            index_offset: 150,
            learning_time_offset_hours: 24,
            learning_time_gain_hours: 24,
            gating_max_duration_minutes: 60,
            std_initial: 100,
            gain_factor: 200,
        };
        a.set_tuning_parameters(tuning).unwrap();
        assert_eq!(a.tuning_parameters(), tuning);
        a.set_states(655_288_772, 12_243_268);
        let mut lcg = Lcg(999);
        let mut spots = [0u16; 6];
        let mut sum = 0i64;
        for i in 0..300 {
            let base = if (100..150).contains(&i) {
                28_500
            } else {
                30_000
            };
            let index = a.process((base + lcg.noise()) as u16);
            sum += i64::from(index);
            if i % 50 == 49 {
                spots[i / 50] = index;
            }
        }
        assert_eq!(spots, [154, 153, 455, 198, 154, 151]);
        assert_eq!(sum, 55_815);
        let bad = TuningParameters {
            index_offset: 0,
            ..tuning
        };
        assert_eq!(a.set_tuning_parameters(bad), Err(InvalidTuning));
    }
}
