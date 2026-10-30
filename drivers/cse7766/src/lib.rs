//! Chipsea CSE7766 single-phase energy metering IC, UART (Sonoff POW R2,
//! Sonoff S31 and similar plugs).
//!
//! The chip is transmit-only (RI is reserved): it streams a 24-byte frame
//! at 4800 bit/s, 8E1 (even parity — configure the UART accordingly)
//! continuously, 54.9 ms per frame. A frame carries, per quantity, the
//! factory calibration *coefficient* and the measured signal *cycle* in
//! µs; the value is `coefficient / cycle` scaled by the board's shunt and
//! divider ([`Calibration`]). It also carries a 16-bit CF pulse counter
//! plus an overflow toggle bit for energy.
//!
//! ```text
//! byte  0        1     2..5   5..8    8..11  11..14  14..17  17..20  20   21..23  23
//!       header1  0x5A  Vcoef  Vcycle  Icoef  Icycle  Pcoef   Pcycle  Adj  CF      checksum
//! ```
//!
//! Multi-byte fields are big-endian; the checksum is the low byte of the
//! sum of bytes 2..=22. The frame layout matches the HLW8032.
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
//! Reference: Chipsea "CSE7766 Single-phase Multi-function Metering Chip
//! with UART" datasheet Rev1.2 (sections 3.4 UART, 3.5 calibration,
//! 3.6 energy, 3.7 flow charts).
//!
//! ```text
//! let mut meter = Cse7766::new(uart); // factory calibration: 1 mΩ, 1 MΩ
//! let wait_us = meter.start_measurement()?;
//! // ... wait_us later:
//! let m = meter.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Frame length.
pub const FRAME_LEN: usize = 24;
/// Packet header 2.
pub const HEADER2: u8 = 0x5A;
/// Packet header 1: chip calibrated, all cycles in range.
pub const HEADER_OK: u8 = 0x55;
/// Packet header 1: chip not calibrated, coefficients unusable.
pub const HEADER_UNCALIBRATED: u8 = 0xAA;
/// Duration of one frame at 4800 bit/s 8E1, µs (datasheet: 54.912 ms).
pub const FRAME_US: u32 = 55_000;
/// Wait returned by `start_measurement`: the rest of a frame in progress
/// plus one complete frame.
pub const MEASUREMENT_US: WaitUs = 2 * FRAME_US;
/// UART bytes discarded at most per `start_measurement`.
const MAX_DRAIN_BYTES: usize = 256;
/// UART bytes parsed at most per `read_measurement` call (4 frames).
const MAX_READ_BYTES: usize = 4 * FRAME_LEN;

/// Packet header 1 bits (valid when the high nibble is 0xF).
pub mod header {
    /// Voltage cycle out of range (abnormal circuit or chip damage).
    pub const VOLTAGE_RANGE: u8 = 1 << 3;
    /// Current cycle out of range (abnormal circuit or chip damage).
    pub const CURRENT_RANGE: u8 = 1 << 2;
    /// Power cycle out of range: active power is 0.
    pub const POWER_RANGE: u8 = 1 << 1;
    /// Coefficient storage abnormal.
    pub const COEFFICIENT_ERROR: u8 = 1 << 0;
}

/// Adj byte bits (bits 2:0 = calibration storage slot).
pub mod adj {
    /// Toggles each time the 16-bit CF counter overflows.
    pub const CF_CARRY: u8 = 1 << 7;
    /// Voltage cycle complete (0: cycle > 1 s, partial count).
    pub const VOLTAGE: u8 = 1 << 6;
    /// Current cycle complete.
    pub const CURRENT: u8 = 1 << 5;
    /// Power cycle complete.
    pub const POWER: u8 = 1 << 4;
}

/// Low byte of the sum of `bytes`.
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |a, b| a.wrapping_add(*b))
}

const fn is_header(b: u8) -> bool {
    b == HEADER_OK || b == HEADER_UNCALIBRATED || b & 0xF0 == 0xF0
}

const fn be24(b: &[u8]) -> u32 {
    (b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32
}

/// Raw frame fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Raw {
    /// Packet header 1.
    pub header: u8,
    /// Voltage coefficient.
    pub voltage_coef: u32,
    /// Voltage cycle, µs.
    pub voltage_cycle: u32,
    /// Current coefficient.
    pub current_coef: u32,
    /// Current cycle, µs.
    pub current_cycle: u32,
    /// Power coefficient.
    pub power_coef: u32,
    /// Power cycle, µs.
    pub power_cycle: u32,
    /// Adj, see [`adj`].
    pub adj: u8,
    /// CF pulse count modulo 2^16 (CFm:CFl).
    pub cf: u16,
}

/// Decode a frame: [`Error::InvalidData`] without header 2 or a valid
/// header 1, [`Error::Crc`] on a checksum mismatch.
pub fn decode_frame<E>(f: &[u8; FRAME_LEN]) -> Result<Raw, Error<E>> {
    if f[1] != HEADER2 || !is_header(f[0]) {
        return Err(Error::InvalidData);
    }
    if checksum(&f[2..23]) != f[23] {
        return Err(Error::Crc);
    }
    Ok(Raw {
        header: f[0],
        voltage_coef: be24(&f[2..]),
        voltage_cycle: be24(&f[5..]),
        current_coef: be24(&f[8..]),
        current_cycle: be24(&f[11..]),
        power_coef: be24(&f[14..]),
        power_cycle: be24(&f[17..]),
        adj: f[20],
        cf: u16::from_be_bytes([f[21], f[22]]),
    })
}

/// Board scale factors relative to the factory calibration circuit
/// (1 mΩ shunt, 1 MΩ / 1 kΩ divider), in parts per million.
///
/// `V = Vcoef / Vcycle · kv`, `I = Icoef / Icycle · ki`,
/// `P = Pcoef / Pcycle · kv · ki` with `kv = V2R` (top resistor in MΩ
/// over a 1 kΩ bottom) and `ki = 1 / V1R` (shunt in mΩ). Fields are
/// public: after measuring a known load, scale one by `true / reported`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// kv · 10^6.
    pub voltage_ppm: u32,
    /// ki · 10^6.
    pub current_ppm: u32,
    /// kv · ki · 10^6 (power and energy).
    pub power_ppm: u32,
}

impl Calibration {
    /// Factory calibration circuit: 1 mΩ shunt, 1 MΩ / 1 kΩ (unity).
    pub const FACTORY: Self = Self {
        voltage_ppm: 1_000_000,
        current_ppm: 1_000_000,
        power_ppm: 1_000_000,
    };

    /// Factors for a shunt of `shunt_uohm` µΩ and a line divider
    /// `r_top_ohm` / `r_bottom_ohm`. `None` if an input is zero or a
    /// factor rounds to 0 / exceeds `u32` (shunt ≥ 233 µΩ,
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

    /// Voltage in mV from coefficient and cycle (0 for a zero cycle;
    /// saturating).
    pub const fn voltage_mv(&self, coef: u32, cycle: u32) -> i32 {
        ratio(coef, self.voltage_ppm, cycle as u64 * 1_000)
    }

    /// Current in µA.
    pub const fn current_ua(&self, coef: u32, cycle: u32) -> i32 {
        ratio(coef, self.current_ppm, cycle as u64)
    }

    /// Active power in mW.
    pub const fn power_mw(&self, coef: u32, cycle: u32) -> i32 {
        ratio(coef, self.power_ppm, cycle as u64 * 1_000)
    }

    /// Energy of one CF pulse in nWh: `Pcoef · kv · ki / 3.6` (the
    /// datasheet's 1000 W pulse rate is `10^9 / Pcoef` Hz at unity).
    pub const fn pulse_nwh(&self, power_coef: u32) -> u64 {
        power_coef as u64 * self.power_ppm as u64 / 3_600_000
    }

    /// Energy in mWh of `pulses` CF pulses (saturating).
    pub const fn energy_mwh(&self, power_coef: u32, pulses: u64) -> u64 {
        pulses.saturating_mul(self.pulse_nwh(power_coef)) / 1_000_000
    }
}

const fn ratio(coef: u32, ppm: u32, den: u64) -> i32 {
    if den == 0 {
        return 0;
    }
    // 24-bit coefficient · 32-bit ppm < 2^56.
    let v = coef as u64 * ppm as u64 / den;
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

/// Total CF pulses from the 16-bit counter and the Adj.7 overflow toggle.
///
/// The first frame becomes the starting total (pulses since power-up,
/// assuming no overflow yet); restore a saved total with
/// [`Self::set_total`]. The chip samples CF ~50 ms before Adj.7, so a
/// toggled Adj.7 with a count not below the previous one counts as 0
/// (datasheet 3.6.2). A smaller count without a toggle means the chip
/// restarted. At most one wrap can be detected between frames, so read
/// at least every 65 535 pulses (≈ 90 s at 3.6 kW with the factory
/// circuit).
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

    /// Feed CF and Adj.7; returns the total. Frames with CFl (low byte)
    /// = 0 are invalid for energy and ignored (datasheet 3.6.2).
    pub fn update(&mut self, count: u16, carry: bool) -> u64 {
        if count & 0xFF == 0 {
            return self.total;
        }
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
    /// Packet header 1.
    pub header: u8,
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
    /// CF pulse total (persist `total()` to keep energy over resets).
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

    /// Convert one frame (datasheet 3.7 flow charts).
    /// [`Error::InvalidData`] when the chip is not calibrated (0xAA), the
    /// coefficient storage is abnormal or the voltage / current cycle is
    /// out of range (abnormal circuit or chip damage). A power cycle out
    /// of range reads 0 W.
    ///
    /// A cycle without its Adj bit is incomplete (> 1 s, still growing):
    /// the last complete value is kept, capped by the incomplete cycle's
    /// value, so it decays to 0 when the load disappears.
    pub fn convert<E>(&mut self, raw: &Raw) -> Result<Measurement, Error<E>> {
        let h = raw.header;
        let flags = if h & 0xF0 == 0xF0 { h & 0x0F } else { 0 };
        let fatal = header::VOLTAGE_RANGE | header::CURRENT_RANGE | header::COEFFICIENT_ERROR;
        if h == HEADER_UNCALIBRATED || flags & fatal != 0 {
            return Err(Error::InvalidData);
        }
        let c = &self.calibration;
        let candidates = [
            (
                false,
                adj::VOLTAGE,
                c.voltage_mv(raw.voltage_coef, raw.voltage_cycle),
            ),
            (
                false,
                adj::CURRENT,
                c.current_ua(raw.current_coef, raw.current_cycle),
            ),
            (
                flags & header::POWER_RANGE != 0,
                adj::POWER,
                c.power_mw(raw.power_coef, raw.power_cycle),
            ),
        ];
        let mut out = [0i32; 3];
        for ((held, out), (zero, bit, value)) in self.held.iter_mut().zip(&mut out).zip(candidates)
        {
            *out = if zero {
                *held = Some(0);
                0
            } else if raw.adj & bit != 0 {
                *held = Some(value);
                value
            } else {
                held.map_or(value, |h| h.min(value))
            };
        }
        let [voltage, current, power] = out;
        let pulses = self.pulses.update(raw.cf, raw.adj & adj::CF_CARRY != 0);
        Ok(Measurement {
            voltage,
            current,
            power,
            energy: c.energy_mwh(raw.power_coef, pulses),
            power_factor: power_factor(power, voltage, current),
            header: h,
        })
    }
}

/// Streaming frame assembler (syncs on header 1 + 0x5A).
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
            0 if !is_header(b) => return None,
            1 if b != HEADER2 => {
                self.len = usize::from(is_header(b));
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
            /// Driver with [`Calibration::FACTORY`]; no bus traffic.
            pub const fn new(uart: U) -> Self {
                Self {
                    uart,
                    parser: Parser::new(),
                    converter: Converter::new(Calibration::FACTORY),
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

            /// CF pulse counter (persist `total()` to keep energy over
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
pub struct Cse7766<U> {
    uart: U,
    parser: Parser,
    converter: Converter,
}

constructors!(Cse7766);

impl<U> zigbee_sensor::blocking::Sensor for Cse7766<U>
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

    /// Async twin of [`super::Cse7766`].
    #[derive(Debug)]
    pub struct Cse7766<U> {
        uart: U,
        parser: Parser,
        converter: Converter,
    }

    constructors!(Cse7766);

    impl<U> zigbee_sensor::asynch::Sensor for Cse7766<U>
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

    /// Frame of a factory-circuit plug at ≈ 233.5 V, 4.75 A, 1103 W;
    /// CF = 0x8176.
    const FRAME: [u8; FRAME_LEN] = [
        0x55, 0x5A, 0x02, 0xE9, 0x50, 0x00, 0x03, 0x31, 0x00, 0x3E, 0x9E, 0x00, 0x0D, 0x30, 0x4F,
        0x44, 0xF8, 0x00, 0x12, 0x65, 0xF1, 0x81, 0x76, 0x72,
    ];

    const EXPECTED: Measurement = Measurement {
        voltage: 233_537,
        current: 4_748_222,
        power: 1_103_206,
        // 0x8176 pulses × 1.443055 mWh.
        energy: 47_825,
        power_factor: Some(994),
        header: 0x55,
    };

    fn frame(header: u8, adj: u8, cf: u16) -> [u8; FRAME_LEN] {
        let mut f = FRAME;
        f[0] = header;
        f[20] = adj;
        f[21..23].copy_from_slice(&cf.to_be_bytes());
        f[23] = checksum(&f[2..23]);
        f
    }

    #[test]
    fn decode_and_convert() {
        let raw = decode_frame::<()>(&FRAME).unwrap();
        assert_eq!(
            (raw.voltage_coef, raw.voltage_cycle, raw.power_coef, raw.cf),
            (190_800, 817, 5_195_000, 0x8176)
        );
        let mut bad = FRAME;
        bad[10] ^= 1;
        assert_eq!(decode_frame::<()>(&bad), Err(Error::Crc));
        bad[0] = 0x12;
        assert_eq!(decode_frame::<()>(&bad), Err(Error::InvalidData));

        assert_eq!(
            Calibration::from_circuit(1_000, 1_000_000, 1_000),
            Some(Calibration::FACTORY)
        );
        let c = Calibration::from_circuit(2_000, 2_000_000, 1_000).unwrap();
        assert_eq!(
            (c.voltage_ppm, c.current_ppm, c.power_ppm),
            (2_000_000, 500_000, 1_000_000)
        );
        assert_eq!(c.voltage_mv(190_800, 817), 467_074);
        assert_eq!(c.current_ua(16_030, 3_376), 2_374_111);
        assert_eq!(Calibration::from_circuit(1, 0, 1), None);

        let mut conv = Converter::new(Calibration::FACTORY);
        assert_eq!(conv.convert::<()>(&raw), Ok(EXPECTED));
        assert_eq!(conv.calibration.pulse_nwh(raw.power_coef), 1_443_055);
    }

    #[test]
    fn header_and_adj_flags() {
        let mut c = Converter::new(Calibration::FACTORY);
        let full = decode_frame::<()>(&FRAME).unwrap();
        for h in [HEADER_UNCALIBRATED, 0xF8, 0xF4, 0xF1] {
            let raw = Raw { header: h, ..full };
            assert_eq!(c.convert::<()>(&raw), Err(Error::InvalidData));
        }
        // Power cycle out of range: 0 W.
        let raw = Raw {
            header: 0xF2,
            ..full
        };
        let m = c.convert::<()>(&raw).unwrap();
        assert_eq!((m.voltage, m.current, m.power), (233_537, 4_748_222, 0));
        assert_eq!(m.power_factor, Some(0));
        // Complete cycle, then incomplete ones: the held 1103 W is capped
        // once the growing cycle implies less.
        assert_eq!(c.convert::<()>(&full).unwrap().power, 1_103_206);
        let partial = Raw {
            adj: adj::CF_CARRY | adj::VOLTAGE | adj::CURRENT,
            power_cycle: 4_000,
            ..full
        };
        assert_eq!(c.convert::<()>(&partial).unwrap().power, 1_103_206);
        let partial = Raw {
            power_cycle: 1_000_000,
            ..partial
        };
        assert_eq!(c.convert::<()>(&partial).unwrap().power, 5_195);
    }

    #[test]
    fn pulse_counter_rules() {
        let mut p = PulseCounter::new();
        assert_eq!(p.update(0xFFF0, false), 0xFFF0);
        // CFl = 0: ignored.
        assert_eq!(p.update(0xFF00, true), 0xFFF0);
        // Adj.7 toggled, CF sampled before the wrap: counts as 0.
        assert_eq!(p.update(0xFFFA, true), 0x1_0000);
        assert_eq!(p.update(0x0005, true), 0x1_0005);
        // Adj.7 toggled with the wrapped count.
        assert_eq!(p.update(0xFFFF, true), 0x1_FFFF);
        assert_eq!(p.update(0x0003, false), 0x2_0003);
        // Smaller count without a toggle: chip restart.
        assert_eq!(p.update(0x0001, false), 0x2_0004);
        p.set_total(100);
        assert_eq!(p.update(0x0501, true), 100);
        assert_eq!(p.update(0x0511, true), 116);
    }

    #[test]
    fn uart_stream() {
        let mut meter = Cse7766::new(FakeUart::default());
        assert_eq!(meter.read_measurement(), Err(Error::NotReady));
        meter.uart.rx.extend([0x00, 0x5A, 0x12]); // stale, drained
        assert_eq!(meter.start_measurement(), Ok(110_000));
        assert!(meter.uart.rx.is_empty());
        // Noise and a false header byte before the frame, split delivery.
        meter.uart.rx.extend([0x13, 0xF2, 0x55]);
        meter.uart.rx.extend(&FRAME[..10]);
        assert_eq!(meter.read_measurement(), Err(Error::NotReady));
        meter.uart.rx.extend(&FRAME[10..]);
        assert_eq!(meter.read_measurement(), Ok(EXPECTED));
        // Several frames: the newest valid one wins.
        let mut bad = frame(0x55, 0xF1, 0x8180);
        bad[23] ^= 1;
        meter.uart.rx.extend(frame(0x55, 0xF1, 0x8177));
        meter.uart.rx.extend(frame(0x55, 0xF1, 0x8178));
        meter.uart.rx.extend(bad);
        assert_eq!(meter.read_measurement().unwrap().energy, 47_828);
        assert_eq!(meter.pulse_counter().total(), 0x8178);
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
        uart.later = FRAME.to_vec();
        let mut meter = asynch::Cse7766::new(uart);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(meter.measure(&mut delay)), Ok(EXPECTED));
        assert_eq!(delay.0, 110_000);
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
