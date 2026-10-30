//! Silicon Labs Si1145/46/47 UV index, ambient light and proximity sensor
//! (I2C).
//!
//! The part runs in forced conversion mode (`MEAS_RATE = 0`): the sequencer
//! stays in standby (≈ 150 nA at 1.8 V) and wakes only for a `*_FORCE`
//! command. `start_measurement` clears `RESPONSE` with a `NOP` and sends
//! `ALS_FORCE`, `PS_FORCE` or `PSALS_FORCE` for the enabled channels.
//! `read_measurement` checks the `RESPONSE` counter and burst-reads the data
//! registers.
//!
//! Commands follow the datasheet handshake: `NOP`, poll `RESPONSE` until it
//! reads 0, send the command, poll until it reads non-zero. The driver has no
//! delay source, so the polls are bounded ([`DEFAULT_POLLS`],
//! [`Si1145::with_poll_limit`]) and return [`Error::Timeout`] when exceeded.
//!
//! UV index: `AUX_DATA` holds 100 × UVI once `EN_UV` is set and `UCOEF` holds
//! the datasheet defaults (0x29, 0x89, 0x02, 0x00).
//!
//! Illuminance: the datasheet gives no lux formula, only the visible
//! photodiode response (0.282 counts/lx in sunlight at gain 0). The reported
//! value is an uncompensated estimate, see [`LuxCal`].
//!
//! Reference: Silicon Labs Si1145/46/47 datasheet, Rev. 1.1.
//!
//! ```ignore
//! let mut s = Si1145::new(i2c);
//! s.probe()?;
//! delay.delay_us(s.reset()?);
//! s.init()?;
//! let m = s.measure(&mut delay)?;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address (default; can be changed with `BUSADDR`, not used).
pub const ADDRESS: u8 = 0x60;

/// Default bound on `RESPONSE` polls per command handshake step.
pub const DEFAULT_POLLS: u16 = 200;

/// Start-up time after power-on or a `RESET` command.
pub const STARTUP_US: u32 = 25_000;

const REG_PART_ID: u8 = 0x00;
const REG_INT_CFG: u8 = 0x03;
const REG_IRQ_ENABLE: u8 = 0x04;
const REG_HW_KEY: u8 = 0x07;
const REG_MEAS_RATE0: u8 = 0x08;
const REG_MEAS_RATE1: u8 = 0x09;
// MEAS_RATE0 on parts with SEQ_ID = 0x01 (datasheet errata).
const REG_MEAS_RATE0_ALT: u8 = 0x0A;
const REG_PS_LED21: u8 = 0x0F;
const REG_UCOEF0: u8 = 0x13;
const REG_PARAM_WR: u8 = 0x17;
const REG_COMMAND: u8 = 0x18;
const REG_RESPONSE: u8 = 0x20;
const REG_IRQ_STATUS: u8 = 0x21;
const REG_ALS_VIS_DATA0: u8 = 0x22;

const HW_KEY: u8 = 0x17;
const UCOEF: [u8; 4] = [0x29, 0x89, 0x02, 0x00];

const CMD_NOP: u8 = 0x00;
const CMD_RESET: u8 = 0x01;
const CMD_PS_FORCE: u8 = 0x05;
const CMD_ALS_FORCE: u8 = 0x06;
const CMD_PSALS_FORCE: u8 = 0x07;
const CMD_PARAM_SET: u8 = 0xA0;

const RESP_INVALID: u8 = 0x80;

const PARAM_CHLIST: u8 = 0x01;
const PARAM_PS_ADC_COUNTER: u8 = 0x0A;
const PARAM_PS_ADC_GAIN: u8 = 0x0B;
const PARAM_PS_ADC_MISC: u8 = 0x0C;
const PARAM_ALS_VIS_ADC_COUNTER: u8 = 0x10;
const PARAM_ALS_IR_ADC_COUNTER: u8 = 0x1D;

const EN_UV: u8 = 1 << 7;
const EN_ALS_IR: u8 = 1 << 5;
const EN_ALS_VIS: u8 = 1 << 4;
const EN_PS1: u8 = 1 << 0;

const RANGE_HIGH: u8 = 1 << 5;
const PS_ADC_MODE_NORMAL: u8 = 1 << 2;

const INT_OE: u8 = 1 << 0;
const ALS_IE: u8 = 1 << 0;
const PS1_IE: u8 = 1 << 2;

/// `take_interrupt` flag: ALS group (VIS/IR/UV) conversion complete.
pub const INT_ALS: u8 = 0b11;
/// `take_interrupt` flag: PS1 conversion complete.
pub const INT_PS1: u8 = 1 << 2;
/// `take_interrupt` flag: command interrupt.
pub const INT_CMD: u8 = 1 << 5;

/// Part number from `PART_ID`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum Part {
    Si1145,
    Si1146,
    Si1147,
}

impl Part {
    const fn from_id(id: u8) -> Option<Self> {
        match id {
            0x45 => Some(Self::Si1145),
            0x46 => Some(Self::Si1146),
            0x47 => Some(Self::Si1147),
            _ => None,
        }
    }
}

/// ALS ADC settings (visible or IR channel).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Adc {
    /// `ALS_*_ADC_GAIN` 0..=7: integration time 25.6 µs × 2^gain.
    pub gain: u8,
    /// High signal range (`*_RANGE`): sensitivity divided by 14.5, for
    /// direct sunlight.
    pub high_range: bool,
}

impl Adc {
    const LADDER: u8 = 9;

    const fn misc(self) -> u8 {
        if self.high_range { RANGE_HIGH } else { 0 }
    }

    // Auto-range ladder position: 0 = high range gain 0, 1..=8 = normal
    // range gain 0..=7.
    const fn step(self) -> u8 {
        if self.high_range { 0 } else { self.gain + 1 }
    }

    const fn from_step(step: u8) -> Self {
        if step == 0 {
            Self {
                gain: 0,
                high_range: true,
            }
        } else {
            Self {
                gain: step - 1,
                high_range: false,
            }
        }
    }

    /// Extra conversion time over gain 0, in microseconds.
    pub const fn extra_us(self) -> u32 {
        ((1u32 << (self.gain & 7)) - 1) * 256 / 10
    }
}

/// Proximity (PS1, LED1, large IR photodiode) settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Proximity {
    /// `LED1_I` code 1..=15 (5.6 mA … 359 mA, datasheet Table 2).
    pub led_current: u8,
    /// `PS_ADC_GAIN` 0..=5: LED pulse width and integration × 2^gain.
    pub gain: u8,
    /// High signal range (`PS_RANGE`).
    pub high_range: bool,
}

impl Default for Proximity {
    /// 22.4 mA, gain 0, normal range.
    fn default() -> Self {
        Self {
            led_current: 3,
            gain: 0,
            high_range: false,
        }
    }
}

/// Visible-channel calibration for the lux estimate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LuxCal {
    /// Visible response in counts per 1000 lx at gain 0, normal range.
    pub counts_per_klx: u32,
    /// Dark reading subtracted from the visible count.
    pub dark: u16,
}

impl Default for LuxCal {
    /// 282 counts/klx (datasheet, sunlight) and a 256-count dark offset.
    fn default() -> Self {
        Self {
            counts_per_klx: 282,
            dark: 256,
        }
    }
}

/// Visible counts to milli-lux: `(vis − dark) · 10⁶ / (counts_per_klx ·
/// 2^gain)`, × 14.5 in high range. No IR compensation.
pub const fn to_millilux(vis: u16, adc: Adc, cal: LuxCal) -> u32 {
    if cal.counts_per_klx == 0 {
        return 0;
    }
    let net = vis.saturating_sub(cal.dark) as u64;
    let k = if adc.high_range { 29 } else { 2 };
    let den = (2 * cal.counts_per_klx as u64) << (adc.gain & 7);
    let mlx = net * 1_000_000 * k / den;
    if mlx > u32::MAX as u64 {
        u32::MAX
    } else {
        mlx as u32
    }
}

/// Pick the ALS ADC setting for the next sample: step down above 90 % of
/// full scale (or on overflow), step up while the reading would stay below
/// 50 %. The ladder is high range, then normal range gain 0..=7.
pub const fn suggest_adc(current: Adc, counts: u16) -> Adc {
    let step = current.step();
    let counts = counts as u32;
    if counts * 10 > 65_535 * 9 {
        return Adc::from_step(step.saturating_sub(1));
    }
    if step + 1 >= Adc::LADDER {
        return current;
    }
    // high range → normal gain 0 is ×14.5, every other step ×2.
    let next = if step == 0 {
        counts * 29 / 2
    } else {
        counts * 2
    };
    if next < 32_768 {
        Adc::from_step(step + 1)
    } else {
        current
    }
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Estimated illuminance in milli-lux, if the visible channel is on.
    pub illuminance: Option<u32>,
    /// UV index in 0.01 units, if UV is enabled.
    pub uv_index: Option<u16>,
    /// Raw visible count.
    pub visible: u16,
    /// Raw IR count.
    pub ir: u16,
    /// Raw PS1 count, if proximity is enabled.
    pub proximity: Option<u16>,
    /// An ADC overflow was reported or a channel read 0xFFFF.
    pub saturated: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if let Some(lux) = self.illuminance {
            f(Quantity::Illuminance(lux));
        }
        if let Some(uv) = self.uv_index {
            f(Quantity::UvIndex(uv));
        }
        if let Some(ps) = self.proximity {
            f(Quantity::Proximity(ps as u32));
        }
    }
}

const fn recovery(gain: u8) -> u8 {
    // Recommended *_ADC_REC: one's complement of the gain.
    (!gain & 7) << 4
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// Si1145/46/47 driver.
        pub struct Si1145<I2C> {
            i2c: I2C,
            uv: bool,
            visible: Option<Adc>,
            ir: Option<Adc>,
            proximity: Option<Proximity>,
            cal: LuxCal,
            interrupt: bool,
            polls: u16,
        }

        impl<I2C> Si1145<I2C> {
            /// Create a driver (UV, visible and IR at gain 0, no
            /// proximity); no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    uv: true,
                    visible: Some(Adc { gain: 0, high_range: false }),
                    ir: Some(Adc { gain: 0, high_range: false }),
                    proximity: None,
                    cal: LuxCal { counts_per_klx: 282, dark: 256 },
                    interrupt: false,
                    polls: DEFAULT_POLLS,
                }
            }

            /// Enable the UV index channel (applied by `init`).
            pub const fn with_uv(mut self, uv: bool) -> Self {
                self.uv = uv;
                self
            }

            /// Visible channel settings, `None` disables it (applied by
            /// `init`).
            pub const fn with_visible(mut self, adc: Option<Adc>) -> Self {
                self.visible = adc;
                self
            }

            /// IR channel settings, `None` disables it (applied by `init`).
            pub const fn with_ir(mut self, adc: Option<Adc>) -> Self {
                self.ir = adc;
                self
            }

            /// Proximity settings, `None` disables PS1 (applied by `init`).
            pub const fn with_proximity(mut self, proximity: Option<Proximity>) -> Self {
                self.proximity = proximity;
                self
            }

            /// Lux estimate calibration.
            pub const fn with_lux_cal(mut self, cal: LuxCal) -> Self {
                self.cal = cal;
                self
            }

            /// Drive INT low when a forced conversion completes (applied by
            /// `init`); clear with `take_interrupt`.
            pub const fn with_interrupt(mut self, enable: bool) -> Self {
                self.interrupt = enable;
                self
            }

            /// Bound on `RESPONSE` polls per handshake step.
            pub const fn with_poll_limit(mut self, polls: u16) -> Self {
                self.polls = polls;
                self
            }

            /// Current visible channel settings.
            pub const fn visible(&self) -> Option<Adc> {
                self.visible
            }

            /// Conversion time for the enabled channels plus 10 % and
            /// 1 ms for wake-up: 285 µs for the ALS group, 155 µs for PS1
            /// (datasheet Table 2), plus 25.6 µs × (2^gain − 1) per channel.
            pub const fn conversion_us(&self) -> u32 {
                let mut t = 0;
                if self.uv || self.visible.is_some() || self.ir.is_some() {
                    t += 285;
                }
                if let Some(a) = self.visible {
                    t += a.extra_us();
                }
                if let Some(a) = self.ir {
                    t += a.extra_us();
                }
                if let Some(p) = self.proximity {
                    t += 155 + Adc { gain: p.gain, high_range: false }.extra_us();
                }
                t + t / 10 + 1_000
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }

            const fn chlist(&self) -> u8 {
                let mut v = 0;
                if self.uv {
                    v |= EN_UV;
                }
                if self.visible.is_some() {
                    v |= EN_ALS_VIS;
                }
                if self.ir.is_some() {
                    v |= EN_ALS_IR;
                }
                if self.proximity.is_some() {
                    v |= EN_PS1;
                }
                v
            }

            const fn force_command(&self) -> Option<u8> {
                let als = self.uv || self.visible.is_some() || self.ir.is_some();
                match (als, self.proximity.is_some()) {
                    (true, true) => Some(CMD_PSALS_FORCE),
                    (true, false) => Some(CMD_ALS_FORCE),
                    (false, true) => Some(CMD_PS_FORCE),
                    (false, false) => None,
                }
            }
        }

        impl<I2C: I2c> Si1145<I2C> {
            $($async_)? fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
                self.i2c.write(ADDRESS, bytes) $($await_)* .map_err(Error::bus)
            }

            $($async_)? fn read(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
                let mut buf = [0u8];
                self.i2c
                    .write_read(ADDRESS, &[reg], &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                Ok(buf[0])
            }

            // Poll RESPONSE until it is zero (`zero`) or non-zero.
            $($async_)? fn wait_response(&mut self, zero: bool) -> Result<u8, Error<I2C::Error>> {
                for _ in 0..self.polls {
                    let r = self.read(REG_RESPONSE) $($await_)* ?;
                    if zero == (r == 0) {
                        return Ok(r);
                    }
                }
                Err(Error::Timeout)
            }

            // NOP + wait for RESPONSE = 0, so the next command is accepted
            // and its completion is visible as a non-zero RESPONSE.
            $($async_)? fn clear_response(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(&[REG_COMMAND, CMD_NOP]) $($await_)* ?;
                self.wait_response(true) $($await_)* ?;
                Ok(())
            }

            /// Write a parameter RAM entry (`PARAM_SET`).
            pub $($async_)? fn set_param(&mut self, param: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.clear_response() $($await_)* ?;
                self.write(&[REG_PARAM_WR, value, CMD_PARAM_SET | (param & 0x1F)]) $($await_)* ?;
                let r = self.wait_response(false) $($await_)* ?;
                if r == RESP_INVALID {
                    return Err(Error::InvalidData);
                }
                Ok(())
            }

            /// Read `PART_ID`.
            pub $($async_)? fn probe(&mut self) -> Result<Part, Error<I2C::Error>> {
                let id = self.read(REG_PART_ID) $($await_)* ?;
                match Part::from_id(id) {
                    Some(p) => Ok(p),
                    None => Err(Error::UnexpectedId(id as u32)),
                }
            }

            /// Stop autonomous mode, mask interrupts and send `RESET`.
            /// Wait the returned time, then call `init`.
            pub $($async_)? fn reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                for reg in [
                    REG_MEAS_RATE0,
                    REG_MEAS_RATE1,
                    REG_MEAS_RATE0_ALT,
                    REG_IRQ_ENABLE,
                    REG_INT_CFG,
                ] {
                    self.write(&[reg, 0]) $($await_)* ?;
                }
                self.write(&[REG_IRQ_STATUS, 0xFF]) $($await_)* ?;
                self.write(&[REG_COMMAND, CMD_RESET]) $($await_)* ?;
                Ok(STARTUP_US)
            }

            /// Unlock the sequencer (`HW_KEY`), load the UV coefficients,
            /// interrupt enables and LED current, and write the channel
            /// parameters. Required after power-on or `reset`.
            pub $($async_)? fn init(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(&[REG_HW_KEY, HW_KEY]) $($await_)* ?;
                self.write(&[REG_UCOEF0, UCOEF[0], UCOEF[1], UCOEF[2], UCOEF[3]]) $($await_)* ?;
                let led = match self.proximity {
                    Some(p) => p.led_current & 0x0F,
                    None => 0,
                };
                self.write(&[REG_PS_LED21, led]) $($await_)* ?;
                let (oe, ie) = if self.interrupt {
                    let mut ie = 0;
                    if self.uv || self.visible.is_some() || self.ir.is_some() {
                        ie |= ALS_IE;
                    }
                    if self.proximity.is_some() {
                        ie |= PS1_IE;
                    }
                    (INT_OE, ie)
                } else {
                    (0, 0)
                };
                self.write(&[REG_INT_CFG, oe]) $($await_)* ?;
                self.write(&[REG_IRQ_ENABLE, ie]) $($await_)* ?;
                let chlist = self.chlist();
                self.set_param(PARAM_CHLIST, chlist) $($await_)* ?;
                if let Some(a) = self.visible {
                    self.set_adc(PARAM_ALS_VIS_ADC_COUNTER, a) $($await_)* ?;
                }
                if let Some(a) = self.ir {
                    self.set_adc(PARAM_ALS_IR_ADC_COUNTER, a) $($await_)* ?;
                }
                if let Some(p) = self.proximity {
                    let gain = p.gain.min(5);
                    self.set_param(PARAM_PS_ADC_COUNTER, recovery(gain)) $($await_)* ?;
                    self.set_param(PARAM_PS_ADC_GAIN, gain) $($await_)* ?;
                    let misc = PS_ADC_MODE_NORMAL | if p.high_range { RANGE_HIGH } else { 0 };
                    self.set_param(PARAM_PS_ADC_MISC, misc) $($await_)* ?;
                }
                Ok(())
            }

            // COUNTER, GAIN and MISC sit at consecutive offsets.
            $($async_)? fn set_adc(&mut self, counter: u8, adc: Adc) -> Result<(), Error<I2C::Error>> {
                let gain = adc.gain & 7;
                self.set_param(counter, recovery(gain)) $($await_)* ?;
                self.set_param(counter + 1, gain) $($await_)* ?;
                self.set_param(counter + 2, adc.misc()) $($await_)*
            }

            /// Change the visible channel gain/range now (e.g. from
            /// [`crate::suggest_adc`]); the channel must be enabled.
            pub $($async_)? fn set_visible(&mut self, adc: Adc) -> Result<(), Error<I2C::Error>> {
                if self.visible.is_none() {
                    return Err(Error::Unsupported);
                }
                self.set_adc(PARAM_ALS_VIS_ADC_COUNTER, adc) $($await_)* ?;
                self.visible = Some(adc);
                Ok(())
            }

            /// Change the IR channel gain/range now; the channel must be
            /// enabled.
            pub $($async_)? fn set_ir(&mut self, adc: Adc) -> Result<(), Error<I2C::Error>> {
                if self.ir.is_none() {
                    return Err(Error::Unsupported);
                }
                self.set_adc(PARAM_ALS_IR_ADC_COUNTER, adc) $($await_)* ?;
                self.ir = Some(adc);
                Ok(())
            }

            /// Read and clear `IRQ_STATUS`; returns the `INT_*` bits.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<u8, Error<I2C::Error>> {
                let v = self.read(REG_IRQ_STATUS) $($await_)* ?;
                if v != 0 {
                    self.write(&[REG_IRQ_STATUS, v]) $($await_)* ?;
                }
                Ok(v)
            }
        }

        impl<I2C: I2c> Sensor for Si1145<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Clear `RESPONSE` and send the forced conversion command for
            /// the enabled channels.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let Some(cmd) = self.force_command() else {
                    return Err(Error::Unsupported);
                };
                self.clear_response() $($await_)* ?;
                self.write(&[REG_COMMAND, cmd]) $($await_)* ?;
                Ok(self.conversion_us())
            }

            /// Check `RESPONSE` (0 = still converting) and read the data
            /// registers.
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let r = self.read(REG_RESPONSE) $($await_)* ?;
                if r == 0 {
                    return Err(Error::NotReady);
                }
                if r == RESP_INVALID {
                    return Err(Error::InvalidData);
                }
                let mut buf = [0u8; 12];
                self.i2c
                    .write_read(ADDRESS, &[REG_ALS_VIS_DATA0], &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                let word = |i: usize| u16::from_le_bytes([buf[i], buf[i + 1]]);
                let mut m = Measurement {
                    illuminance: None,
                    uv_index: None,
                    visible: 0,
                    ir: 0,
                    proximity: None,
                    saturated: r & 0xF0 != 0,
                };
                if let Some(a) = self.visible {
                    m.visible = word(0);
                    m.illuminance = Some(to_millilux(m.visible, a, self.cal));
                    m.saturated |= m.visible == u16::MAX;
                }
                if self.ir.is_some() {
                    m.ir = word(2);
                    m.saturated |= m.ir == u16::MAX;
                }
                if self.proximity.is_some() {
                    let ps = word(4);
                    m.proximity = Some(ps);
                    m.saturated |= ps == u16::MAX;
                }
                if self.uv {
                    m.uv_index = Some(word(10));
                }
                Ok(m)
            }
        }

        impl<I2C: I2c> LowPower for Si1145<I2C> {
            type BusError = I2C::Error;

            /// In forced mode the sequencer returns to standby after every
            /// conversion; nothing to do.
            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                Ok(())
            }

            /// Nothing to do; each forced command wakes the sequencer.
            $($async_)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                Ok(0)
            }
        }
    };
}

driver!(,);

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, ALS_IE, Adc, CMD_ALS_FORCE, CMD_NOP, CMD_PARAM_SET, CMD_PS_FORCE, CMD_PSALS_FORCE,
        CMD_RESET, DEFAULT_POLLS, EN_ALS_IR, EN_ALS_VIS, EN_PS1, EN_UV, HW_KEY, INT_OE, LuxCal,
        Measurement, PARAM_ALS_IR_ADC_COUNTER, PARAM_ALS_VIS_ADC_COUNTER, PARAM_CHLIST,
        PARAM_PS_ADC_COUNTER, PARAM_PS_ADC_GAIN, PARAM_PS_ADC_MISC, PS_ADC_MODE_NORMAL, PS1_IE,
        Part, Proximity, RANGE_HIGH, REG_ALS_VIS_DATA0, REG_COMMAND, REG_HW_KEY, REG_INT_CFG,
        REG_IRQ_ENABLE, REG_IRQ_STATUS, REG_MEAS_RATE0, REG_MEAS_RATE0_ALT, REG_MEAS_RATE1,
        REG_PARAM_WR, REG_PART_ID, REG_PS_LED21, REG_RESPONSE, REG_UCOEF0, RESP_INVALID,
        STARTUP_US, UCOEF, recovery, to_millilux,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::asynch::{LowPower, Sensor};
    use zigbee_sensor::{Error, WaitUs};

    driver!(async, .await);
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use std::vec::Vec;

    fn nop() -> [Transaction; 2] {
        [
            Transaction::write(ADDRESS, vec![0x18, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x20], vec![0x00]),
        ]
    }

    fn param(p: u8, v: u8) -> Vec<Transaction> {
        let mut t = nop().to_vec();
        t.push(Transaction::write(ADDRESS, vec![0x17, v, 0xA0 | p]));
        t.push(Transaction::write_read(ADDRESS, vec![0x20], vec![0x01]));
        t
    }

    #[test]
    fn helpers() {
        let g0 = Adc::default();
        let cal = LuxCal::default();
        assert_eq!(to_millilux(256 + 282, g0, cal), 1_000_000);
        assert_eq!(to_millilux(100, g0, cal), 0);
        let hi = Adc {
            gain: 0,
            high_range: true,
        };
        assert_eq!(to_millilux(256 + 282, hi, cal), 14_500_000);
        let g7 = Adc {
            gain: 7,
            high_range: false,
        };
        assert_eq!(to_millilux(256 + 282, g7, cal), 7_812);
        assert_eq!(g7.extra_us(), 3_251);
        assert_eq!(recovery(0), 0x70);
        assert_eq!(recovery(7), 0x00);

        assert_eq!(suggest_adc(g0, 60_000), hi);
        assert_eq!(suggest_adc(hi, 60_000), hi);
        assert_eq!(suggest_adc(hi, 2_000), g0);
        assert_eq!(suggest_adc(hi, 3_000), hi);
        assert_eq!(
            suggest_adc(g0, 10_000),
            Adc {
                gain: 1,
                high_range: false
            }
        );
        assert_eq!(suggest_adc(g0, 20_000), g0);
        assert_eq!(suggest_adc(g7, 10), g7);

        let s = Si1145::new(());
        assert_eq!(s.conversion_us(), 1_313);
        let s = s
            .with_proximity(Some(Proximity::default()))
            .with_ir(Some(g7));
        assert_eq!(s.conversion_us(), 5_060);
    }

    #[test]
    fn init_and_one_shot() {
        let mut e = vec![
            Transaction::write_read(ADDRESS, vec![0x00], vec![0x46]),
            Transaction::write(ADDRESS, vec![0x08, 0]),
            Transaction::write(ADDRESS, vec![0x09, 0]),
            Transaction::write(ADDRESS, vec![0x0A, 0]),
            Transaction::write(ADDRESS, vec![0x04, 0]),
            Transaction::write(ADDRESS, vec![0x03, 0]),
            Transaction::write(ADDRESS, vec![0x21, 0xFF]),
            Transaction::write(ADDRESS, vec![0x18, 0x01]),
            Transaction::write(ADDRESS, vec![0x07, 0x17]),
            Transaction::write(ADDRESS, vec![0x13, 0x29, 0x89, 0x02, 0x00]),
            Transaction::write(ADDRESS, vec![0x0F, 0x03]),
            Transaction::write(ADDRESS, vec![0x03, 0x01]),
            Transaction::write(ADDRESS, vec![0x04, 0x05]),
        ];
        e.extend(param(0x01, 0xB1));
        e.extend(param(0x10, 0x70));
        e.extend(param(0x11, 0x00));
        e.extend(param(0x12, 0x00));
        e.extend(param(0x1D, 0x40));
        e.extend(param(0x1E, 0x03));
        e.extend(param(0x1F, 0x20));
        e.extend(param(0x0A, 0x70));
        e.extend(param(0x0B, 0x00));
        e.extend(param(0x0C, 0x04));
        // start: NOP with one busy poll, then PSALS_FORCE.
        e.extend([
            Transaction::write(ADDRESS, vec![0x18, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x20], vec![0x8C]),
            Transaction::write_read(ADDRESS, vec![0x20], vec![0x00]),
            Transaction::write(ADDRESS, vec![0x18, 0x07]),
            Transaction::write_read(ADDRESS, vec![0x20], vec![0x00]),
            Transaction::write_read(ADDRESS, vec![0x20], vec![0x01]),
            Transaction::write_read(
                ADDRESS,
                vec![0x22],
                vec![0x1A, 0x02, 0x00, 0x10, 0x34, 0x12, 0, 0, 0, 0, 0x2C, 0x01],
            ),
            Transaction::write_read(ADDRESS, vec![0x21], vec![0x05]),
            Transaction::write(ADDRESS, vec![0x21, 0x05]),
        ]);
        let mut i2c = Mock::new(&e);
        let ir = Adc {
            gain: 3,
            high_range: true,
        };
        let mut s = Si1145::new(&mut i2c)
            .with_ir(Some(ir))
            .with_proximity(Some(Proximity::default()))
            .with_interrupt(true);
        assert_eq!(s.probe().unwrap(), Part::Si1146);
        assert_eq!(s.reset().unwrap(), STARTUP_US);
        s.init().unwrap();
        assert_eq!(s.start_measurement().unwrap(), 1_680);
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        let m = s.read_measurement().unwrap();
        assert_eq!(m.visible, 538);
        assert_eq!(m.illuminance, Some(1_000_000));
        assert_eq!(m.ir, 0x1000);
        assert_eq!(m.proximity, Some(0x1234));
        assert_eq!(m.uv_index, Some(300));
        assert!(!m.saturated);
        assert_eq!(s.take_interrupt().unwrap(), 0b101);
        i2c.done();
    }

    #[test]
    fn errors() {
        let mut e = vec![Transaction::write_read(ADDRESS, vec![0x00], vec![0x41])];
        // set_visible: invalid setting reported.
        e.extend(nop());
        e.push(Transaction::write(ADDRESS, vec![0x17, 0x60, 0xB0]));
        e.push(Transaction::write_read(ADDRESS, vec![0x20], vec![0x80]));
        // NOP never clears within the poll limit.
        e.push(Transaction::write(ADDRESS, vec![0x18, 0x00]));
        e.extend((0..3).map(|_| Transaction::write_read(ADDRESS, vec![0x20], vec![0x80])));
        // overflow on the visible channel.
        e.push(Transaction::write_read(ADDRESS, vec![0x20], vec![0x8C]));
        e.push(Transaction::write_read(
            ADDRESS,
            vec![0x22],
            vec![0xFF, 0xFF, 0x10, 0x01, 0, 0, 0, 0, 0, 0, 0x05, 0x00],
        ));
        let mut i2c = Mock::new(&e);
        let mut s = Si1145::new(&mut i2c).with_poll_limit(3);
        assert_eq!(s.probe(), Err(Error::UnexpectedId(0x41)));
        let g1 = Adc {
            gain: 1,
            high_range: false,
        };
        assert_eq!(s.set_visible(g1), Err(Error::InvalidData));
        assert_eq!(s.start_measurement(), Err(Error::Timeout));
        let m = s.read_measurement().unwrap();
        assert!(m.saturated);
        assert_eq!(m.uv_index, Some(5));
        assert_eq!(m.proximity, None);
        let mut s = s.with_uv(false).with_visible(None).with_ir(None);
        assert_eq!(s.start_measurement(), Err(Error::Unsupported));
        assert_eq!(s.set_visible(g1), Err(Error::Unsupported));
        i2c.done();
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let mut e = nop().to_vec();
        e.extend([
            Transaction::write(ADDRESS, vec![0x18, 0x06]),
            Transaction::write_read(ADDRESS, vec![0x20], vec![0x01]),
            Transaction::write_read(
                ADDRESS,
                vec![0x22],
                vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x34, 0x04],
            ),
        ]);
        let mut i2c = Mock::new(&e);
        let mut s = asynch::Si1145::new(&mut i2c)
            .with_visible(None)
            .with_ir(None);
        let mut delay = RecordingDelay(0);
        let m = block_on(s.measure(&mut delay)).unwrap();
        assert_eq!(m.uv_index, Some(1076));
        assert_eq!(m.illuminance, None);
        assert_eq!(delay.0, 1_313);
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
