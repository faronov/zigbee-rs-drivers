//! HLW8032 (and its CSE7759 twin) single-phase energy metering IC, UART.
//!
//! The chip is transmit-only: it streams a 24-byte frame at 4800 bit/s,
//! 8E1 (even parity — configure the UART accordingly) every ~50 ms. A
//! frame carries, per quantity, a factory-calibrated *parameter* register
//! and a measured *cycle* register; the value is `parameter / cycle`
//! scaled by the board's voltage divider and shunt ([`Calibration`]).
//! It also carries a 16-bit PF pulse counter plus an overflow toggle bit
//! for energy.
//!
//! ```text
//! byte  0      1     2..5  5..8  8..11  11..14  14..17  17..20  20      21..23  23
//!       State  0x5A  VP    V     IP     I       PP      P       Update  PF      checksum
//! ```
//!
//! Multi-byte fields are big-endian; the checksum is the low byte of the
//! sum of bytes 2..=22.
//!
//! The driver needs [`embedded_io::ReadReady`] so it never blocks on a
//! silent line:
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! drains stale bytes and returns the time for one complete frame to
//! arrive (110 ms);
//! [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//! parses what has arrived (newest valid frame wins) and returns
//! [`Error::NotReady`] without one. Reading without a preceding start also
//! works.
//!
//! The chip has no power-down mode, so `LowPower` is not implemented.
//!
//! Reference: HLW Technology "HLW8032 Energy Metering IC" datasheet REV 1.5
//! (UART interface, register description, application calculations).
//!
//! ```text
//! let mut meter = Hlw8032::new(uart).with_calibration(Calibration::REFERENCE);
//! let wait_us = meter.start_measurement()?;
//! // ... wait_us later:
//! let m = meter.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Frame length.
pub const FRAME_LEN: usize = 24;
/// Second byte of every frame (Check REG).
pub const CHECK: u8 = 0x5A;
/// State REG: error correction normal, no overflow.
pub const STATE_OK: u8 = 0x55;
/// State REG: error correction failed, parameter registers unusable.
pub const STATE_FAILED: u8 = 0xAA;
/// Duration of one frame at 4800 bit/s 8E1 (11 bits per byte), µs.
pub const FRAME_US: u32 = 55_000;
/// Wait returned by `start_measurement`: the rest of a frame in progress
/// plus one complete frame.
pub const MEASUREMENT_US: WaitUs = 2 * FRAME_US;
/// UART bytes discarded at most per `start_measurement`.
const MAX_DRAIN_BYTES: usize = 256;
/// UART bytes parsed at most per `read_measurement` call (4 frames).
const MAX_READ_BYTES: usize = 4 * FRAME_LEN;

/// State REG bits (valid when the high nibble is 0xF).
pub mod state {
    /// Voltage REG overflow: voltage ≈ 0.
    pub const VOLTAGE_OVERFLOW: u8 = 1 << 3;
    /// Current REG overflow: current ≈ 0.
    pub const CURRENT_OVERFLOW: u8 = 1 << 2;
    /// Power REG overflow: power ≈ 0.
    pub const POWER_OVERFLOW: u8 = 1 << 1;
    /// Parameter registers unusable.
    pub const PARAMETERS_UNUSABLE: u8 = 1 << 0;
}

/// Data Update REG bits.
pub mod update {
    /// Toggles each time the 16-bit PF REG overflows.
    pub const PF_CARRY: u8 = 1 << 7;
    /// Voltage REG updated.
    pub const VOLTAGE: u8 = 1 << 6;
    /// Current REG updated.
    pub const CURRENT: u8 = 1 << 5;
    /// Power REG updated.
    pub const POWER: u8 = 1 << 4;
}

/// Low byte of the sum of `bytes`.
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |a, b| a.wrapping_add(*b))
}

const fn is_state(b: u8) -> bool {
    b == STATE_OK || b == STATE_FAILED || b & 0xF0 == 0xF0
}

const fn be24(b: &[u8]) -> u32 {
    (b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32
}

/// Raw frame registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Raw {
    /// State REG.
    pub state: u8,
    /// Voltage Parameter REG.
    pub voltage_param: u32,
    /// Voltage REG.
    pub voltage: u32,
    /// Current Parameter REG.
    pub current_param: u32,
    /// Current REG.
    pub current: u32,
    /// Power Parameter REG.
    pub power_param: u32,
    /// Power REG.
    pub power: u32,
    /// Data Update REG, see [`update`].
    pub data_update: u8,
    /// PF REG (pulse count modulo 2^16).
    pub pf: u16,
}

/// Decode a frame: [`Error::InvalidData`] without the 0x5A check byte or a
/// valid State REG, [`Error::Crc`] on a checksum mismatch.
pub fn decode_frame<E>(f: &[u8; FRAME_LEN]) -> Result<Raw, Error<E>> {
    if f[1] != CHECK || !is_state(f[0]) {
        return Err(Error::InvalidData);
    }
    if checksum(&f[2..23]) != f[23] {
        return Err(Error::Crc);
    }
    Ok(Raw {
        state: f[0],
        voltage_param: be24(&f[2..]),
        voltage: be24(&f[5..]),
        current_param: be24(&f[8..]),
        current: be24(&f[11..]),
        power_param: be24(&f[14..]),
        power: be24(&f[17..]),
        data_update: f[20],
        pf: u16::from_be_bytes([f[21], f[22]]),
    })
}

/// Board coefficients in parts per million.
///
/// Datasheet: `V = VP / V · Kv`, `I = IP / I · Ki`, `P = PP / P · Kv · Ki`
/// with the voltage coefficient `Kv = R_top / (R_bottom · 1000)` (1.88 for
/// 4 × 470 kΩ / 1 kΩ) and the current coefficient `Ki = 1 / (R_shunt[Ω] ·
/// 1000)` (1 for 1 mΩ). Fields are public: after measuring a known load,
/// scale one by `true / reported`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// Kv · 10^6.
    pub voltage_ppm: u32,
    /// Ki · 10^6.
    pub current_ppm: u32,
    /// Kv · Ki · 10^6 (power and energy).
    pub power_ppm: u32,
}

impl Calibration {
    /// Datasheet application circuit: 1 mΩ shunt, 4 × 470 kΩ / 1 kΩ.
    pub const REFERENCE: Self = match Self::from_circuit(1_000, 1_880_000, 1_000) {
        Some(c) => c,
        None => panic!("reference calibration"),
    };

    /// Coefficients for a shunt of `shunt_uohm` µΩ and a line divider
    /// `r_top_ohm` / `r_bottom_ohm`. `None` if an input is zero or a
    /// coefficient rounds to 0 / exceeds `u32` (shunt ≥ 233 µΩ,
    /// divider ≤ 4294 · 1000).
    pub const fn from_circuit(shunt_uohm: u32, r_top_ohm: u32, r_bottom_ohm: u32) -> Option<Self> {
        if shunt_uohm == 0 || r_bottom_ohm == 0 {
            return None;
        }
        let voltage = r_top_ohm as u64 * 1_000 / r_bottom_ohm as u64;
        let current = 1_000_000_000 / shunt_uohm as u64;
        let power = voltage * current / 1_000_000;
        if voltage == 0
            || voltage > u32::MAX as u64
            || current > u32::MAX as u64
            || power == 0
            || power > u32::MAX as u64
        {
            return None;
        }
        Some(Self {
            voltage_ppm: voltage as u32,
            current_ppm: current as u32,
            power_ppm: power as u32,
        })
    }

    /// Voltage in mV from parameter and cycle registers (0 for a zero
    /// cycle; saturating).
    pub const fn voltage_mv(&self, param: u32, cycle: u32) -> i32 {
        ratio(param, self.voltage_ppm, cycle as u64 * 1_000)
    }

    /// Current in µA.
    pub const fn current_ua(&self, param: u32, cycle: u32) -> i32 {
        ratio(param, self.current_ppm, cycle as u64)
    }

    /// Active power in mW.
    pub const fn power_mw(&self, param: u32, cycle: u32) -> i32 {
        ratio(param, self.power_ppm, cycle as u64 * 1_000)
    }

    /// Energy of one PF pulse in nWh: `PP · Kv · Ki / 3.6` (PP µJ at
    /// unity coefficients; ≈ 1.44 mWh for PP = 5.2 · 10^6).
    pub const fn pulse_nwh(&self, power_param: u32) -> u64 {
        power_param as u64 * self.power_ppm as u64 / 3_600_000
    }

    /// Energy in mWh of `pulses` PF pulses (saturating).
    pub const fn energy_mwh(&self, power_param: u32, pulses: u64) -> u64 {
        pulses.saturating_mul(self.pulse_nwh(power_param)) / 1_000_000
    }
}

const fn ratio(param: u32, ppm: u32, den: u64) -> i32 {
    if den == 0 {
        return 0;
    }
    // 24-bit param · 32-bit ppm < 2^56.
    let v = param as u64 * ppm as u64 / den;
    if v > i32::MAX as u64 {
        i32::MAX
    } else {
        v as i32
    }
}

/// Power factor in 0.001 from active power and RMS values, clamped to
/// 0..=1000; `None` without voltage or current.
pub fn power_factor(power_mw: i32, voltage_mv: i32, current_ua: i32) -> Option<i16> {
    let apparent = voltage_mv as i64 * current_ua as i64;
    if apparent <= 0 {
        return None;
    }
    let pf = power_mw as i64 * 1_000_000_000 / apparent;
    Some(pf.clamp(0, 1000) as i16)
}

/// Total PF pulses from the 16-bit PF REG and its overflow toggle bit.
///
/// The first frame becomes the starting total (pulses since power-up,
/// assuming no overflow yet); restore a saved total with
/// [`Self::set_total`]. A toggled carry bit with a count not below the
/// previous one means the count was sampled just before the wrap: it
/// counts as 0. A smaller count without a toggle means the chip
/// restarted. At most one wrap can be detected between frames, so read
/// at least every 65 535 pulses (≈ 90 s at 3.6 kW on the reference
/// board).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PulseCounter {
    last: Option<(u16, bool)>,
    total: u64,
    restored: bool,
}

impl PulseCounter {
    /// Empty counter.
    pub const fn new() -> Self {
        Self {
            last: None,
            total: 0,
            restored: false,
        }
    }

    /// Feed PF REG and the carry bit; returns the total.
    pub fn update(&mut self, count: u16, carry: bool) -> u64 {
        let (delta, count) = match self.last {
            None if self.restored => (0, count),
            None => (count as u32, count),
            Some((last, last_carry)) if carry != last_carry => {
                if count < last {
                    (count as u32 + 0x1_0000 - last as u32, count)
                } else {
                    (0x1_0000 - last as u32, 0)
                }
            }
            Some((last, _)) if count >= last => ((count - last) as u32, count),
            Some(_) => (count as u32, count),
        };
        self.last = Some((count, carry));
        self.total = self.total.saturating_add(delta as u64);
        self.total
    }

    /// Total pulses so far.
    pub const fn total(&self) -> u64 {
        self.total
    }

    /// Replace the total (e.g. restored from NV); the next frame is only
    /// a new baseline.
    pub fn set_total(&mut self, pulses: u64) {
        self.total = pulses;
        self.last = None;
        self.restored = true;
    }
}

/// One reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// RMS line voltage, mV.
    pub voltage: i32,
    /// RMS current, µA.
    pub current: i32,
    /// Active power, mW.
    pub power: i32,
    /// Accumulated energy, mWh.
    pub energy: u64,
    /// Power factor, 0.001 (computed).
    pub power_factor: Option<i16>,
    /// State REG.
    pub state: u8,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Voltage(self.voltage));
        f(Quantity::Current(self.current));
        f(Quantity::Power(self.power));
        f(Quantity::Energy(self.energy));
        if let Some(v) = self.power_factor {
            f(Quantity::PowerFactor(v));
        }
    }
}

/// Conversion state: calibration, last complete values, pulse total.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Converter {
    /// Board calibration.
    pub calibration: Calibration,
    /// PF pulse total (persist `total()` to keep energy over resets).
    pub pulses: PulseCounter,
    held: [Option<i32>; 3],
}

impl Converter {
    /// Fresh state.
    pub const fn new(calibration: Calibration) -> Self {
        Self {
            calibration,
            pulses: PulseCounter::new(),
            held: [None; 3],
        }
    }

    /// Convert one frame. [`Error::InvalidData`] when the State REG marks
    /// the parameter registers unusable (0xAA or bit 0).
    ///
    /// An overflowed register reads 0. A register without its update bit
    /// holds an incomplete (growing) cycle: the last complete value is
    /// kept, capped by the incomplete cycle's value, so it decays to 0
    /// when the load disappears.
    pub fn convert<E>(&mut self, raw: &Raw) -> Result<Measurement, Error<E>> {
        let s = raw.state;
        if s == STATE_FAILED || (s & 0xF0 == 0xF0 && s & state::PARAMETERS_UNUSABLE != 0) {
            return Err(Error::InvalidData);
        }
        let overflow = |bit: u8| s & 0xF0 == 0xF0 && s & bit != 0;
        let c = &self.calibration;
        let candidates = [
            (
                state::VOLTAGE_OVERFLOW,
                update::VOLTAGE,
                c.voltage_mv(raw.voltage_param, raw.voltage),
            ),
            (
                state::CURRENT_OVERFLOW,
                update::CURRENT,
                c.current_ua(raw.current_param, raw.current),
            ),
            (
                state::POWER_OVERFLOW,
                update::POWER,
                c.power_mw(raw.power_param, raw.power),
            ),
        ];
        let mut out = [0i32; 3];
        for ((held, out), (ovf, upd, value)) in self.held.iter_mut().zip(&mut out).zip(candidates) {
            *out = if overflow(ovf) {
                *held = Some(0);
                0
            } else if raw.data_update & upd != 0 {
                *held = Some(value);
                value
            } else {
                held.map_or(value, |h| h.min(value))
            };
        }
        let [voltage, current, power] = out;
        let pulses = self
            .pulses
            .update(raw.pf, raw.data_update & update::PF_CARRY != 0);
        Ok(Measurement {
            voltage,
            current,
            power,
            energy: c.energy_mwh(raw.power_param, pulses),
            power_factor: power_factor(power, voltage, current),
            state: s,
        })
    }
}

/// Streaming frame assembler (syncs on State + 0x5A).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Parser {
    buf: [u8; FRAME_LEN],
    len: usize,
}

impl Parser {
    const fn new() -> Self {
        Self {
            buf: [0; FRAME_LEN],
            len: 0,
        }
    }

    fn feed(&mut self, b: u8) -> Option<[u8; FRAME_LEN]> {
        match self.len {
            0 if !is_state(b) => return None,
            1 if b != CHECK => {
                self.len = usize::from(is_state(b));
                self.buf[0] = b;
                return None;
            }
            _ => {}
        }
        self.buf[self.len] = b;
        self.len += 1;
        if self.len < FRAME_LEN {
            return None;
        }
        self.len = 0;
        Some(self.buf)
    }
}

/// Result of parsing a batch of bytes.
#[derive(Default)]
struct Batch {
    frame: Option<Raw>,
    crc: bool,
}

impl Batch {
    fn push(&mut self, f: &[u8; FRAME_LEN]) {
        match decode_frame::<()>(f) {
            Ok(raw) => self.frame = Some(raw),
            Err(_) => self.crc = true,
        }
    }

    fn finish<E>(self, conv: &mut Converter) -> Result<Measurement, Error<E>> {
        match self.frame {
            Some(raw) => conv.convert(&raw),
            None if self.crc => Err(Error::Crc),
            None => Err(Error::NotReady),
        }
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<U> $name<U> {
            /// Driver with [`Calibration::REFERENCE`]; no bus traffic.
            pub const fn new(uart: U) -> Self {
                Self {
                    uart,
                    parser: Parser::new(),
                    converter: Converter::new(Calibration::REFERENCE),
                }
            }

            /// Board calibration.
            pub const fn with_calibration(mut self, calibration: Calibration) -> Self {
                self.converter.calibration = calibration;
                self
            }

            /// Active calibration.
            pub const fn calibration(&self) -> &Calibration {
                &self.converter.calibration
            }

            /// PF pulse counter (persist `total()` to keep energy over
            /// resets, restore with `set_total`).
            pub fn pulse_counter(&mut self) -> &mut PulseCounter {
                &mut self.converter.pulses
            }

            /// Give the UART back.
            pub fn release(self) -> U {
                self.uart
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Hlw8032<U> {
    uart: U,
    parser: Parser,
    converter: Converter,
}

constructors!(Hlw8032);

impl<U> zigbee_sensor::blocking::Sensor for Hlw8032<U>
where
    U: embedded_io::Read + embedded_io::ReadReady,
{
    type BusError = U::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        let mut buf = [0u8; 32];
        let mut left = MAX_DRAIN_BYTES;
        while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
            let n = self
                .uart
                .read(&mut buf[..left.min(32)])
                .map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            left -= n;
        }
        self.parser = Parser::new();
        Ok(MEASUREMENT_US)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
        let mut buf = [0u8; FRAME_LEN];
        let mut left = MAX_READ_BYTES;
        let mut batch = Batch::default();
        while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
            let n = self
                .uart
                .read(&mut buf[..left.min(FRAME_LEN)])
                .map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            left -= n;
            for b in &buf[..n] {
                if let Some(f) = self.parser.feed(*b) {
                    batch.push(&f);
                }
            }
        }
        batch.finish(&mut self.converter)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        Batch, Calibration, Converter, FRAME_LEN, MAX_DRAIN_BYTES, MAX_READ_BYTES, MEASUREMENT_US,
        Measurement, Parser, PulseCounter,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Hlw8032`].
    #[derive(Debug)]
    pub struct Hlw8032<U> {
        uart: U,
        parser: Parser,
        converter: Converter,
    }

    constructors!(Hlw8032);

    impl<U> zigbee_sensor::asynch::Sensor for Hlw8032<U>
    where
        U: embedded_io_async::Read + embedded_io_async::ReadReady,
    {
        type BusError = U::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
            let mut buf = [0u8; 32];
            let mut left = MAX_DRAIN_BYTES;
            while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
                let n = self
                    .uart
                    .read(&mut buf[..left.min(32)])
                    .await
                    .map_err(Error::bus)?;
                if n == 0 {
                    break;
                }
                left -= n;
            }
            self.parser = Parser::new();
            Ok(MEASUREMENT_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
            let mut buf = [0u8; FRAME_LEN];
            let mut left = MAX_READ_BYTES;
            let mut batch = Batch::default();
            while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
                let n = self
                    .uart
                    .read(&mut buf[..left.min(FRAME_LEN)])
                    .await
                    .map_err(Error::bus)?;
                if n == 0 {
                    break;
                }
                left -= n;
                for b in &buf[..n] {
                    if let Some(f) = self.parser.feed(*b) {
                        batch.push(&f);
                    }
                }
            }
            batch.finish(&mut self.converter)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::collections::VecDeque;
    use zigbee_sensor::blocking::Sensor;

    /// `later` arrives once the line was seen idle (after a drain).
    #[derive(Default)]
    struct FakeUart {
        rx: VecDeque<u8>,
        later: std::vec::Vec<u8>,
    }

    impl embedded_io::ErrorType for FakeUart {
        type Error = core::convert::Infallible;
    }
    impl embedded_io::ReadReady for FakeUart {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            if self.rx.is_empty() {
                self.rx.extend(core::mem::take(&mut self.later));
                return Ok(false);
            }
            Ok(true)
        }
    }
    impl embedded_io::Read for FakeUart {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.rx.len());
            for b in &mut buf[..n] {
                *b = self.rx.pop_front().unwrap();
            }
            Ok(n)
        }
    }
    impl embedded_io_async::Read for FakeUart {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            embedded_io::Read::read(self, buf)
        }
    }

    /// Frame with factory parameters for a 1 mΩ / 1 MΩ circuit at
    /// ≈ 233.5 V, 4.75 A, 1103 W; PF REG 0x8176.
    const FRAME: [u8; FRAME_LEN] = [
        0x55, 0x5A, 0x02, 0xE9, 0x50, 0x00, 0x03, 0x31, 0x00, 0x3E, 0x9E, 0x00, 0x0D, 0x30, 0x4F,
        0x44, 0xF8, 0x00, 0x12, 0x65, 0xF1, 0x81, 0x76, 0x72,
    ];

    fn one_megohm() -> Calibration {
        Calibration::from_circuit(1_000, 1_000_000, 1_000).unwrap()
    }

    fn frame(state: u8, update: u8, pf: u16) -> [u8; FRAME_LEN] {
        let mut f = FRAME;
        f[0] = state;
        f[20] = update;
        f[21..23].copy_from_slice(&pf.to_be_bytes());
        f[23] = checksum(&f[2..23]);
        f
    }

    #[test]
    fn decode_and_convert() {
        let raw = decode_frame::<()>(&FRAME).unwrap();
        assert_eq!(raw.voltage_param, 190_800);
        assert_eq!(raw.voltage, 817);
        assert_eq!(raw.power_param, 5_195_000);
        assert_eq!(raw.pf, 0x8176);
        let mut bad = FRAME;
        bad[10] ^= 1;
        assert_eq!(decode_frame::<()>(&bad), Err(Error::Crc));
        bad[1] = 0x00;
        assert_eq!(decode_frame::<()>(&bad), Err(Error::InvalidData));

        let r = Calibration::REFERENCE;
        assert_eq!(
            (r.voltage_ppm, r.current_ppm, r.power_ppm),
            (1_880_000, 1_000_000, 1_880_000)
        );
        assert_eq!(r.voltage_mv(190_800, 817), 439_050);
        assert_eq!(Calibration::from_circuit(0, 1, 1), None);

        let mut c = Converter::new(one_megohm());
        let m = c.convert::<()>(&raw).unwrap();
        assert_eq!(
            m,
            Measurement {
                voltage: 233_537,
                current: 4_748_222,
                power: 1_103_206,
                // 0x8176 pulses × 1.443055 mWh.
                energy: 47_825,
                power_factor: Some(994),
                state: 0x55,
            }
        );
        assert_eq!(c.calibration.pulse_nwh(raw.power_param), 1_443_055);
    }

    #[test]
    fn state_and_update_flags() {
        let mut c = Converter::new(one_megohm());
        let full = decode_frame::<()>(&FRAME).unwrap();
        c.convert::<()>(&full).unwrap();
        // Parameters unusable.
        for s in [STATE_FAILED, 0xF1] {
            let raw = Raw { state: s, ..full };
            assert_eq!(c.convert::<()>(&raw), Err(Error::InvalidData));
        }
        // Current and power overflow (≈ 0).
        let raw = Raw {
            state: 0xF6,
            ..full
        };
        let m = c.convert::<()>(&raw).unwrap();
        assert_eq!((m.voltage, m.current, m.power), (233_537, 0, 0));
        assert_eq!(m.power_factor, None);
        // Incomplete power cycle: held value (0 now) capped by the partial.
        let raw = Raw {
            data_update: update::VOLTAGE | update::CURRENT,
            power: 1_000_000,
            ..full
        };
        assert_eq!(c.convert::<()>(&raw).unwrap().power, 0);
        // Complete cycle, then the load drops: 1103 W held while the
        // partial cycle implies more; the partial caps it once smaller.
        assert_eq!(c.convert::<()>(&full).unwrap().power, 1_103_206);
        let partial = Raw {
            power: 4_000,
            ..raw
        };
        assert_eq!(c.convert::<()>(&partial).unwrap().power, 1_103_206);
        let partial = Raw {
            power: 1_000_000,
            ..raw
        };
        assert_eq!(c.convert::<()>(&partial).unwrap().power, 5_195);
    }

    #[test]
    fn pulse_counter_wraps() {
        let mut p = PulseCounter::new();
        assert_eq!(p.update(0xFFF0, false), 0xFFF0);
        assert_eq!(p.update(0xFFF8, false), 0xFFF8);
        // Carry toggled, count sampled before the wrap: counts as 0.
        assert_eq!(p.update(0xFFFA, true), 0x1_0000);
        assert_eq!(p.update(0x0005, true), 0x1_0005);
        // Carry toggled with the wrapped count.
        assert_eq!(p.update(0xFFFF, true), 0x1_FFFF);
        assert_eq!(p.update(0x0003, false), 0x2_0003);
        // Smaller count without a toggle: chip restart.
        assert_eq!(p.update(0x0001, false), 0x2_0004);
        p.set_total(100);
        assert_eq!(p.update(0x0500, true), 100);
        assert_eq!(p.update(0x0510, true), 116);
    }

    #[test]
    fn uart_stream() {
        let mut meter = Hlw8032::new(FakeUart::default()).with_calibration(one_megohm());
        assert_eq!(meter.read_measurement(), Err(Error::NotReady));
        meter.uart.rx.extend([0x00, 0x5A, 0x12]); // stale, drained
        assert_eq!(meter.start_measurement(), Ok(110_000));
        assert!(meter.uart.rx.is_empty());
        // Noise and a false State byte before the frame, split delivery.
        meter.uart.rx.extend([0x13, 0xF2, 0x55]);
        meter.uart.rx.extend(&FRAME[..10]);
        assert_eq!(meter.read_measurement(), Err(Error::NotReady));
        meter.uart.rx.extend(&FRAME[10..]);
        assert_eq!(meter.read_measurement().unwrap().voltage, 233_537);
        // Several frames: the newest valid one wins.
        let mut bad = frame(0x55, 0xF1, 0x8180);
        bad[23] ^= 1;
        meter.uart.rx.extend(frame(0x55, 0xF1, 0x8177));
        meter.uart.rx.extend(frame(0x55, 0xF1, 0x8178));
        meter.uart.rx.extend(bad);
        let m = meter.read_measurement().unwrap();
        assert_eq!(meter.pulse_counter().total(), 0x8178);
        assert_eq!(m.energy, 47_828);
        meter.uart.rx.extend(bad);
        assert_eq!(meter.read_measurement(), Err(Error::Crc));
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

        let mut uart = FakeUart::default();
        uart.rx.extend(&FRAME[..7]); // stale, drained
        uart.later = frame(0x55, 0x71, 0).to_vec();
        let mut meter = asynch::Hlw8032::new(uart);
        let mut delay = RecordingDelay(0);
        let m = block_on(meter.measure(&mut delay)).unwrap();
        assert_eq!(delay.0, 110_000);
        assert_eq!(
            (m.voltage, m.current, m.power, m.energy),
            (439_050, 4_748_222, 2_074_028, 0)
        );
        assert_eq!(block_on(meter.measure(&mut delay)), Err(Error::NotReady));
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
