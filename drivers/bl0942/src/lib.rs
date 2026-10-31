//! Belling BL0942 single-phase energy metering IC over UART or SPI.
//!
//! The BL0942 measures the shunt voltage (IP/IN) and the divided line
//! voltage (VP) and reports RMS current, RMS voltage, active power, an
//! energy pulse counter (CF_CNT) and the line period. It knows nothing
//! about the shunt or divider, so a [`Calibration`] converts its counts to
//! µA / mV / mW / mWh in integer fixed point.
//!
//! * UART (SEL = 0): 8N1, 4800 or 9600 bit/s by the SCLK_BPS pin (19200 /
//!   38400 via MODE), half duplex, chip address 0-3 by pins A2/A1
//!   (SSOP10 is always 0). One "full packet" request returns all
//!   measurement registers in 23 bytes (~48 ms at 4800 bit/s). The driver
//!   needs [`embedded_io::ReadReady`] so it never blocks on a silent chip:
//!   [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//!   sends the request and returns the transfer time,
//!   [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//!   parses what has arrived and returns [`Error::NotReady`] otherwise.
//! * SPI (SEL = 1): mode 1 (CPOL 0, CPHA 1), ≤ 900 kHz, 6-byte frames with
//!   a checksum. There is no full packet: `read_measurement` reads seven
//!   registers.
//!
//! RMS and power registers refresh every 400 ms (800 ms with
//! `MODE.RMS_UPDATE_SEL`). The chip draws ~3 mA and has no power-down
//! mode, so `LowPower` is not implemented.
//!
//! Reference: Belling "BL0942 calibration-free metering IC" datasheet
//! V1.06 (sections 1.5 registers, 2.x formulas, 3.1 SPI, 3.2 UART).
//!
//! ```text
//! let iface = UartInterface::new(uart, 0);
//! let mut meter = Bl0942::new(iface).with_calibration(Calibration::REFERENCE);
//! let wait_us = meter.start_measurement()?;
//! // ... wait_us later:
//! let m = meter.read_measurement()?; // NotReady until all 23 bytes arrived
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Register addresses (datasheet section 1.5).
pub mod reg {
    /// Current RMS (24-bit unsigned).
    pub const I_RMS: u8 = 0x03;
    /// Voltage RMS (24-bit unsigned).
    pub const V_RMS: u8 = 0x04;
    /// Fast current RMS (24-bit unsigned).
    pub const I_FAST_RMS: u8 = 0x05;
    /// Active power (24-bit two's complement).
    pub const WATT: u8 = 0x06;
    /// Active energy pulse counter (24-bit unsigned).
    pub const CF_CNT: u8 = 0x07;
    /// Line period in 1 µs units (16-bit; 0x4E20 = 50 Hz).
    pub const FREQ: u8 = 0x08;
    /// Status (bit 0 CF_REVP_F, bit 1 CREEP_F, bits 8/9 zero-cross faults).
    pub const STATUS: u8 = 0x09;
    /// Current RMS offset.
    pub const I_RMSOS: u8 = 0x12;
    /// Active power no-load (creep) threshold.
    pub const WA_CREEP: u8 = 0x14;
    /// Fast RMS over-current threshold.
    pub const I_FAST_RMS_TH: u8 = 0x15;
    /// Fast RMS refresh period.
    pub const I_FAST_RMS_CYC: u8 = 0x16;
    /// Line period refresh cycles.
    pub const FREQ_CYC: u8 = 0x17;
    /// CF/ZX/over-current output pin function.
    pub const OT_FUNX: u8 = 0x18;
    /// Mode register, see [`super::mode`].
    pub const MODE: u8 = 0x19;
    /// Current channel gain: 0 = ×1, 1 = ×4, 2 = ×16 (default), 3 = ×24.
    pub const GAIN_CR: u8 = 0x1A;
    /// Write [`super::SOFT_RESET_KEY`] to reset the chip.
    pub const SOFT_RESET: u8 = 0x1C;
    /// Write [`super::UNLOCK_KEY`] to allow writes to user registers.
    pub const USR_WRPROT: u8 = 0x1D;
}

/// MODE register bits (default 0x87).
pub mod mode {
    /// Power-on value.
    pub const DEFAULT: u32 = 0x87;
    /// CF pulse output and energy accumulation enable.
    pub const CF_EN: u32 = 1 << 2;
    /// RMS refresh 800 ms instead of 400 ms.
    pub const RMS_UPDATE_800MS: u32 = 1 << 3;
    /// Fast RMS from AC component only.
    pub const FAST_RMS_AC: u32 = 1 << 4;
    /// 60 Hz line (fast RMS / zero-cross timing).
    pub const AC_FREQ_60HZ: u32 = 1 << 5;
    /// Clear CF_CNT after each read (the driver expects this off).
    pub const CF_CNT_CLR: u32 = 1 << 6;
    /// Absolute (not signed) energy accumulation.
    pub const CF_CNT_ABS: u32 = 1 << 7;
    /// UART rate field: 19200 bit/s.
    pub const UART_19200: u32 = 0b10 << 8;
    /// UART rate field: 38400 bit/s.
    pub const UART_38400: u32 = 0b11 << 8;
}

/// Value for [`reg::USR_WRPROT`] that unlocks user registers.
pub const UNLOCK_KEY: u32 = 0x55;
/// Value for [`reg::SOFT_RESET`].
pub const SOFT_RESET_KEY: u32 = 0x5A_5A5A;
/// Pseudo-register that requests the UART full packet.
pub const FULL_PACKET: u8 = 0xAA;
/// Full packet response length.
pub const PACKET_LEN: usize = 23;
/// First byte of the full packet response.
pub const PACKET_HEAD: u8 = 0x55;
/// Read command byte (UART: `| address`).
pub const CMD_READ: u8 = 0x58;
/// Write command byte (UART: `| address`).
pub const CMD_WRITE: u8 = 0xA8;
/// Delay from the end of the request to the first response byte (t2).
pub const RESPONSE_DELAY_US: u32 = 150;
/// UART bytes discarded at most per request (stale input).
const MAX_DRAIN_BYTES: usize = 64;
/// UART bytes parsed at most per `read_measurement` call.
const MAX_READ_BYTES: usize = 64;

/// Checksum of all UART/SPI frames: bit-inverted 8-bit sum of command,
/// register (or packet) bytes and data.
pub fn checksum(bytes: &[u8]) -> u8 {
    !bytes.iter().fold(0u8, |a, b| a.wrapping_add(*b))
}

/// Raw measurement registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Raw {
    /// I_RMS.
    pub i_rms: u32,
    /// V_RMS.
    pub v_rms: u32,
    /// I_FAST_RMS.
    pub i_fast_rms: u32,
    /// WATT, sign-extended.
    pub watt: i32,
    /// CF_CNT.
    pub cf_cnt: u32,
    /// FREQ (line period in µs).
    pub freq: u16,
    /// STATUS (the UART packet carries only bits 7:0).
    pub status: u16,
}

const fn sign_extend_24(v: u32) -> i32 {
    ((v << 8) as i32) >> 8
}

const fn le24(b: &[u8]) -> u32 {
    b[0] as u32 | (b[1] as u32) << 8 | (b[2] as u32) << 16
}

/// Decode a UART full packet (section 3.2.6). `command` is the read
/// command byte that was sent (`0x58 | address`), part of the checksum.
pub fn decode_packet<E>(command: u8, p: &[u8; PACKET_LEN]) -> Result<Raw, Error<E>> {
    if p[0] != PACKET_HEAD {
        return Err(Error::InvalidData);
    }
    if checksum(&p[..22]).wrapping_sub(command) != p[22] {
        return Err(Error::Crc);
    }
    Ok(Raw {
        i_rms: le24(&p[1..]),
        v_rms: le24(&p[4..]),
        i_fast_rms: le24(&p[7..]),
        watt: sign_extend_24(le24(&p[10..])),
        cf_cnt: le24(&p[13..]),
        freq: u16::from_le_bytes([p[16], p[17]]),
        status: p[19] as u16,
    })
}

/// Fixed-point conversion factors for one board.
///
/// The defaults of the datasheet formulas (section 2, current gain 16):
/// `I_RMS = 305978·Ip/Vref`, `V_RMS = 73989·Vp/Vref`,
/// `WATT = 3537·Ip·Vp·cosφ/Vref²` (Ip, Vp pin voltages in mV, Vref in V)
/// and one CF_CNT count per `1638.4·256/WATT` seconds. Fields are public:
/// after measuring a known load, scale a factor by `true / reported`.
/// With a current gain other than 16, scale `current_q16`, `power_q16`
/// and `energy_q16` by `16 / gain`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// µA per I_RMS LSB, Q16.
    pub current_q16: u32,
    /// mV per V_RMS LSB, Q20.
    pub voltage_q20: u32,
    /// mW per WATT LSB, Q16.
    pub power_q16: u32,
    /// mWh per CF_CNT pulse, Q16.
    pub energy_q16: u32,
}

const fn div_round(n: u128, d: u128) -> u128 {
    (n + d / 2) / d
}

const fn fit_u32(v: u128) -> Option<u32> {
    if v == 0 || v > u32::MAX as u128 {
        None
    } else {
        Some(v as u32)
    }
}

impl Calibration {
    /// Datasheet reference circuit: 1 mΩ shunt, 5 × 390 kΩ / 510 Ω divider,
    /// Vref 1.218 V (≈ 3.98 µA, 0.063 mV, 1.60 mW and 186.9 mWh per LSB).
    pub const REFERENCE: Self = match Self::from_circuit(1_000, 1_950_000, 510, 1_218_000) {
        Some(c) => c,
        None => panic!("reference calibration"),
    };

    /// Factors for a shunt of `shunt_uohm` µΩ, a line divider
    /// `r_top_ohm` / `r_bottom_ohm` and `vref_uv` (nominal 1 218 000 µV).
    /// Uses 128-bit math: evaluate it in a `const` to keep it out of
    /// flash. `None` if an input is zero, `vref_uv` > 2 000 000 or a
    /// factor rounds to 0 / exceeds `u32`.
    pub const fn from_circuit(
        shunt_uohm: u32,
        r_top_ohm: u32,
        r_bottom_ohm: u32,
        vref_uv: u32,
    ) -> Option<Self> {
        if shunt_uohm == 0 || r_bottom_ohm == 0 || vref_uv == 0 || vref_uv > 2_000_000 {
            return None;
        }
        let r = shunt_uohm as u128;
        let rb = r_bottom_ohm as u128;
        let total = r_top_ohm as u128 + rb;
        let v = vref_uv as u128;
        // µA/LSB = Vref[µV]·1000 / (305978·R[µΩ])
        let current = div_round((v * 1000) << 16, 305_978 * r);
        // mV/LSB = (Rt+Rb)·Vref[µV] / (73989e6·Rb)
        let voltage = div_round((total * v) << 20, 73_989_000_000 * rb);
        // mW/LSB = (Rt+Rb)·Vref[µV]² / (3537e9·R[µΩ]·Rb)
        let power_den = 3_537_000_000_000 * r * rb;
        let power = div_round((total * v * v) << 16, power_den);
        // mWh/pulse = mW/LSB · 419430.4 s / 3600 = mW/LSB · 2^22 / 36000
        let energy = div_round((total * v * v) << 38, power_den * 36_000);
        match (
            fit_u32(current),
            fit_u32(voltage),
            fit_u32(power),
            fit_u32(energy),
        ) {
            (Some(current_q16), Some(voltage_q20), Some(power_q16), Some(energy_q16)) => {
                Some(Self {
                    current_q16,
                    voltage_q20,
                    power_q16,
                    energy_q16,
                })
            }
            _ => None,
        }
    }

    /// I_RMS → µA (saturating).
    pub const fn current_ua(&self, i_rms: u32) -> i32 {
        sat_i32((i_rms as u64 * self.current_q16 as u64) >> 16)
    }

    /// V_RMS → mV (saturating).
    pub const fn voltage_mv(&self, v_rms: u32) -> i32 {
        sat_i32((v_rms as u64 * self.voltage_q20 as u64) >> 20)
    }

    /// WATT → mW; exact for the full ±2^23 register range.
    pub const fn power_mw(&self, watt: i32) -> i32 {
        ((watt as i64 * self.power_q16 as i64) >> 16) as i32
    }

    /// CF pulses → mWh (saturating).
    pub const fn energy_mwh(&self, pulses: u64) -> u64 {
        pulses.saturating_mul(self.energy_q16 as u64) >> 16
    }
}

const fn sat_i32(v: u64) -> i32 {
    if v > i32::MAX as u64 {
        i32::MAX
    } else {
        v as i32
    }
}

/// FREQ (line period, µs) → mHz; `None` for 0.
pub const fn frequency_mhz(freq: u16) -> Option<u32> {
    if freq == 0 {
        None
    } else {
        Some(1_000_000_000 / freq as u32)
    }
}

/// Power factor in 0.001 from active power and RMS values, clamped to
/// ±1000; `None` without voltage or current. The BL0942 has no PF
/// register, and below a few percent of full scale this ratio is noisy.
pub fn power_factor(power_mw: i32, voltage_mv: i32, current_ua: i32) -> Option<i16> {
    let apparent = voltage_mv as i64 * current_ua as i64;
    if apparent <= 0 {
        return None;
    }
    let pf = power_mw as i64 * 1_000_000_000 / apparent;
    Some(pf.clamp(-1000, 1000) as i16)
}

/// Total energy pulses from the 24-bit wrapping CF_CNT register.
///
/// The first value seen becomes the starting total (pulses since chip
/// power-up); afterwards increments are added modulo 2^24. Restore a
/// saved total with [`Self::set_total`]. Requires `MODE.CF_CNT_CLR` off.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EnergyCounter {
    last: Option<u32>,
    total: u64,
    restored: bool,
}

impl EnergyCounter {
    /// Empty counter.
    pub const fn new() -> Self {
        Self {
            last: None,
            total: 0,
            restored: false,
        }
    }

    /// Feed a CF_CNT value; returns the total.
    pub fn update(&mut self, cf_cnt: u32) -> u64 {
        let cf_cnt = cf_cnt & 0xFF_FFFF;
        let delta = match self.last {
            Some(last) => cf_cnt.wrapping_sub(last) & 0xFF_FFFF,
            None if self.restored => 0,
            None => cf_cnt,
        };
        self.last = Some(cf_cnt);
        self.total = self.total.saturating_add(delta as u64);
        self.total
    }

    /// Total pulses so far.
    pub const fn total(&self) -> u64 {
        self.total
    }

    /// Replace the total (e.g. restored from NV); the next CF_CNT read is
    /// only a new baseline.
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
    /// Active power, mW (negative for reverse flow).
    pub power: i32,
    /// Accumulated energy, mWh.
    pub energy: u64,
    /// Line frequency, mHz.
    pub frequency: Option<u32>,
    /// Power factor, 0.001.
    pub power_factor: Option<i16>,
    /// STATUS register.
    pub status: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Voltage(self.voltage));
        f(Quantity::Current(self.current));
        f(Quantity::Power(self.power));
        f(Quantity::Energy(self.energy));
        if let Some(v) = self.frequency {
            f(Quantity::Frequency(v));
        }
        if let Some(v) = self.power_factor {
            f(Quantity::PowerFactor(v));
        }
    }
}

/// Convert raw registers, feeding CF_CNT into `energy`.
pub fn convert(cal: &Calibration, energy: &mut EnergyCounter, raw: &Raw) -> Measurement {
    let voltage = cal.voltage_mv(raw.v_rms);
    let current = cal.current_ua(raw.i_rms);
    let power = cal.power_mw(raw.watt);
    Measurement {
        voltage,
        current,
        power,
        energy: cal.energy_mwh(energy.update(raw.cf_cnt)),
        frequency: frequency_mhz(raw.freq),
        power_factor: power_factor(power, voltage, current),
        status: raw.status,
    }
}

/// Response of a completed request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Response {
    /// Full measurement set.
    Packet(Raw),
    /// One register value.
    Register(u32),
}

/// Register access over UART or SPI as request + poll.
pub trait Interface {
    /// Transport error.
    type Error;
    /// Start reading `reg` ([`FULL_PACKET`] = all measurement registers);
    /// returns the time until [`Self::poll`] can complete.
    fn request(&mut self, reg: u8) -> Result<WaitUs, Self::Error>;
    /// The response to the last request, `None` while incomplete.
    fn poll(&mut self) -> Result<Option<Response>, Error<Self::Error>>;
    /// Write a 24-bit register.
    fn write_reg(&mut self, reg: u8, value: u32) -> Result<(), Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    None,
    Packet,
    Register(u8),
}

/// UART response assembler.
#[derive(Debug)]
struct Parser {
    pending: Pending,
    buf: [u8; PACKET_LEN],
    len: usize,
}

impl Parser {
    const fn new() -> Self {
        Self {
            pending: Pending::None,
            buf: [0; PACKET_LEN],
            len: 0,
        }
    }

    fn start(&mut self, reg: u8) {
        self.len = 0;
        self.pending = if reg == FULL_PACKET {
            Pending::Packet
        } else {
            Pending::Register(reg)
        };
    }

    fn feed<E>(&mut self, command: u8, byte: u8) -> Option<Result<Response, Error<E>>> {
        let want = match self.pending {
            Pending::None => return None,
            Pending::Packet if self.len == 0 && byte != PACKET_HEAD => return None,
            Pending::Packet => PACKET_LEN,
            Pending::Register(_) => 4,
        };
        self.buf[self.len] = byte;
        self.len += 1;
        if self.len < want {
            return None;
        }
        let pending = self.pending;
        self.pending = Pending::None;
        self.len = 0;
        Some(match pending {
            Pending::Register(reg) => {
                let d = &self.buf[..4];
                if checksum(&[command, reg, d[0], d[1], d[2]]) == d[3] {
                    Ok(Response::Register(le24(d)))
                } else {
                    Err(Error::Crc)
                }
            }
            _ => {
                let mut p = [0; PACKET_LEN];
                p.copy_from_slice(&self.buf);
                decode_packet(command, &p).map(Response::Packet)
            }
        })
    }
}

/// UART transport (SEL = 0).
#[derive(Debug)]
pub struct UartInterface<U> {
    uart: U,
    address: u8,
    baud: u32,
    parser: Parser,
}

impl<U> UartInterface<U> {
    /// Chip address 0-3 (pins A2/A1; SSOP10 is 0) at 4800 bit/s.
    pub const fn new(uart: U, address: u8) -> Self {
        Self {
            uart,
            address: address & 3,
            baud: 4800,
            parser: Parser::new(),
        }
    }

    /// Baud rate used for the response time estimate.
    pub const fn with_baud(mut self, baud: u32) -> Self {
        self.baud = if baud == 0 { 4800 } else { baud };
        self
    }

    /// Give the UART back.
    pub fn release(self) -> U {
        self.uart
    }

    const fn read_command(&self) -> u8 {
        CMD_READ | self.address
    }

    fn write_frame(&self, reg: u8, value: u32) -> [u8; 6] {
        let [l, m, h, _] = value.to_le_bytes();
        let mut f = [CMD_WRITE | self.address, reg, l, m, h, 0];
        f[5] = checksum(&f[..5]);
        f
    }

    fn response_us(&self, reg: u8) -> WaitUs {
        let bytes = if reg == FULL_PACKET { PACKET_LEN } else { 4 } as u32;
        (bytes * 10_000_000).div_ceil(self.baud) + RESPONSE_DELAY_US
    }
}

impl<U> Interface for UartInterface<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type Error = U::Error;

    fn request(&mut self, reg: u8) -> Result<WaitUs, U::Error> {
        let mut buf = [0u8; 16];
        let mut left = MAX_DRAIN_BYTES;
        while left > 0 && self.uart.read_ready()? {
            let n = self.uart.read(&mut buf)?;
            if n == 0 {
                break;
            }
            left = left.saturating_sub(n);
        }
        self.parser.start(reg);
        self.uart.write_all(&[self.read_command(), reg])?;
        self.uart.flush()?;
        Ok(self.response_us(reg))
    }

    fn poll(&mut self) -> Result<Option<Response>, Error<U::Error>> {
        let command = self.read_command();
        let mut buf = [0u8; PACKET_LEN];
        let mut left = MAX_READ_BYTES;
        while left > 0
            && self.parser.pending != Pending::None
            && self.uart.read_ready().map_err(Error::bus)?
        {
            let want = left.min(PACKET_LEN - self.parser.len);
            let n = self.uart.read(&mut buf[..want]).map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            left -= n;
            for b in &buf[..n] {
                if let Some(r) = self.parser.feed(command, *b) {
                    return r.map(Some);
                }
            }
        }
        Ok(None)
    }

    fn write_reg(&mut self, reg: u8, value: u32) -> Result<(), U::Error> {
        self.uart.write_all(&self.write_frame(reg, value))?;
        self.uart.flush()
    }
}

/// SPI transport (SEL = 1): mode 1, ≤ 900 kHz.
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    /// The device (bus + chip select).
    pub spi: SPI,
    pending: Pending,
}

impl<SPI> SpiInterface<SPI> {
    /// Wrap an SPI device.
    pub const fn new(spi: SPI) -> Self {
        Self {
            spi,
            pending: Pending::None,
        }
    }
}

/// Registers read for a measurement over SPI.
const SPI_SNAPSHOT: [u8; 7] = [
    reg::I_RMS,
    reg::V_RMS,
    reg::I_FAST_RMS,
    reg::WATT,
    reg::CF_CNT,
    reg::FREQ,
    reg::STATUS,
];

fn spi_value<E>(reg: u8, r: &[u8; 4]) -> Result<u32, Error<E>> {
    if checksum(&[CMD_READ, reg, r[0], r[1], r[2]]) != r[3] {
        return Err(Error::Crc);
    }
    Ok(u32::from_be_bytes([0, r[0], r[1], r[2]]))
}

fn spi_write_frame(reg: u8, value: u32) -> [u8; 6] {
    let [_, h, m, l] = value.to_be_bytes();
    let mut f = [CMD_WRITE, reg, h, m, l, 0];
    f[5] = checksum(&f[..5]);
    f
}

fn raw_from_registers(v: &[u32; 7]) -> Raw {
    Raw {
        i_rms: v[0],
        v_rms: v[1],
        i_fast_rms: v[2],
        watt: sign_extend_24(v[3]),
        cf_cnt: v[4],
        freq: v[5] as u16,
        status: v[6] as u16,
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> SpiInterface<SPI> {
    fn read(&mut self, reg: u8) -> Result<u32, Error<SPI::Error>> {
        use embedded_hal::spi::Operation;
        let mut r = [0; 4];
        self.spi
            .transaction(&mut [Operation::Write(&[CMD_READ, reg]), Operation::Read(&mut r)])
            .map_err(Error::bus)?;
        spi_value(reg, &r)
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn request(&mut self, reg: u8) -> Result<WaitUs, SPI::Error> {
        self.pending = if reg == FULL_PACKET {
            Pending::Packet
        } else {
            Pending::Register(reg)
        };
        Ok(0)
    }

    fn poll(&mut self) -> Result<Option<Response>, Error<SPI::Error>> {
        let pending = core::mem::replace(&mut self.pending, Pending::None);
        Ok(match pending {
            Pending::None => None,
            Pending::Register(reg) => Some(Response::Register(self.read(reg)?)),
            Pending::Packet => {
                let mut v = [0; 7];
                for (out, reg) in v.iter_mut().zip(SPI_SNAPSHOT) {
                    *out = self.read(reg)?;
                }
                Some(Response::Packet(raw_from_registers(&v)))
            }
        })
    }

    fn write_reg(&mut self, reg: u8, value: u32) -> Result<(), SPI::Error> {
        self.spi.write(&spi_write_frame(reg, value))
    }
}

fn check_24bit<E>(value: u32) -> Result<(), Error<E>> {
    if value > 0xFF_FFFF {
        Err(Error::InvalidArgument)
    } else {
        Ok(())
    }
}

fn finish_packet<E>(r: Option<Response>) -> Result<Raw, Error<E>> {
    match r {
        Some(Response::Packet(raw)) => Ok(raw),
        Some(Response::Register(_)) => Err(Error::InvalidData),
        None => Err(Error::NotReady),
    }
}

fn finish_register<E>(r: Option<Response>) -> Result<u32, Error<E>> {
    match r {
        Some(Response::Register(v)) => Ok(v),
        Some(Response::Packet(_)) => Err(Error::InvalidData),
        None => Err(Error::NotReady),
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Driver with [`Calibration::REFERENCE`]; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    calibration: Calibration::REFERENCE,
                    energy: EnergyCounter::new(),
                }
            }

            /// Board calibration.
            pub const fn with_calibration(mut self, calibration: Calibration) -> Self {
                self.calibration = calibration;
                self
            }

            /// Active calibration.
            pub const fn calibration(&self) -> &Calibration {
                &self.calibration
            }

            /// Energy pulse counter (persist `total()` to keep energy over
            /// resets).
            pub fn energy_counter(&mut self) -> &mut EnergyCounter {
                &mut self.energy
            }

            /// Give the transport back.
            pub fn release(self) -> IFACE {
                self.iface
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Bl0942<IFACE> {
    iface: IFACE,
    calibration: Calibration,
    energy: EnergyCounter,
}

constructors!(Bl0942);

impl<IFACE: Interface> Bl0942<IFACE> {
    /// Start reading one register; fetch it with [`Self::read_requested`]
    /// after the returned time (immediately over SPI).
    pub fn request_register(&mut self, reg: u8) -> Result<WaitUs, Error<IFACE::Error>> {
        self.iface.request(reg).map_err(Error::bus)
    }

    /// Value of the register requested by [`Self::request_register`];
    /// [`Error::NotReady`] while the UART response is incomplete.
    pub fn read_requested(&mut self) -> Result<u32, Error<IFACE::Error>> {
        finish_register(self.iface.poll()?)
    }

    /// Write a 24-bit register (user registers need
    /// [`Self::unlock`] first); [`Error::InvalidArgument`] if `value`
    /// does not fit 24 bits.
    pub fn write_register(&mut self, reg: u8, value: u32) -> Result<(), Error<IFACE::Error>> {
        check_24bit(value)?;
        self.iface.write_reg(reg, value).map_err(Error::bus)
    }

    /// Allow writes to the user registers 0x12-0x1A.
    pub fn unlock(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write_register(reg::USR_WRPROT, UNLOCK_KEY)
    }

    /// Unlock and write MODE (see [`mode`]).
    pub fn set_mode(&mut self, mode: u32) -> Result<(), Error<IFACE::Error>> {
        self.unlock()?;
        self.write_register(reg::MODE, mode)
    }

    /// Software reset; registers return to defaults and CF_CNT restarts
    /// (call [`EnergyCounter::set_total`] to keep the total).
    pub fn soft_reset(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write_register(reg::SOFT_RESET, SOFT_RESET_KEY)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Bl0942<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.iface.request(FULL_PACKET).map_err(Error::bus)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        let raw = finish_packet(self.iface.poll()?)?;
        Ok(convert(&self.calibration, &mut self.energy, &raw))
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        Calibration, EnergyCounter, FULL_PACKET, MAX_DRAIN_BYTES, MAX_READ_BYTES, Measurement,
        PACKET_LEN, Pending, Response, SOFT_RESET_KEY, SPI_SNAPSHOT, SpiInterface, UNLOCK_KEY,
        UartInterface, check_24bit, convert, finish_packet, finish_register, raw_from_registers,
        reg, spi_value, spi_write_frame,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Interface`].
    #[allow(async_fn_in_trait)]
    pub trait Interface {
        /// Transport error.
        type Error;
        /// See [`super::Interface::request`].
        async fn request(&mut self, reg: u8) -> Result<WaitUs, Self::Error>;
        /// See [`super::Interface::poll`].
        async fn poll(&mut self) -> Result<Option<Response>, Error<Self::Error>>;
        /// See [`super::Interface::write_reg`].
        async fn write_reg(&mut self, reg: u8, value: u32) -> Result<(), Self::Error>;
    }

    impl<U> Interface for UartInterface<U>
    where
        U: embedded_io_async::Read + embedded_io_async::ReadReady + embedded_io_async::Write,
    {
        type Error = U::Error;

        async fn request(&mut self, reg: u8) -> Result<WaitUs, U::Error> {
            let mut buf = [0u8; 16];
            let mut left = MAX_DRAIN_BYTES;
            while left > 0 && self.uart.read_ready()? {
                let n = self.uart.read(&mut buf).await?;
                if n == 0 {
                    break;
                }
                left = left.saturating_sub(n);
            }
            self.parser.start(reg);
            self.uart.write_all(&[self.read_command(), reg]).await?;
            self.uart.flush().await?;
            Ok(self.response_us(reg))
        }

        async fn poll(&mut self) -> Result<Option<Response>, Error<U::Error>> {
            let command = self.read_command();
            let mut buf = [0u8; PACKET_LEN];
            let mut left = MAX_READ_BYTES;
            while left > 0
                && self.parser.pending != Pending::None
                && self.uart.read_ready().map_err(Error::bus)?
            {
                let want = left.min(PACKET_LEN - self.parser.len);
                let n = self.uart.read(&mut buf[..want]).await.map_err(Error::bus)?;
                if n == 0 {
                    break;
                }
                left -= n;
                for b in &buf[..n] {
                    if let Some(r) = self.parser.feed(command, *b) {
                        return r.map(Some);
                    }
                }
            }
            Ok(None)
        }

        async fn write_reg(&mut self, reg: u8, value: u32) -> Result<(), U::Error> {
            self.uart.write_all(&self.write_frame(reg, value)).await?;
            self.uart.flush().await
        }
    }

    impl<SPI: embedded_hal_async::spi::SpiDevice> SpiInterface<SPI> {
        async fn read_async(&mut self, reg: u8) -> Result<u32, Error<SPI::Error>> {
            use embedded_hal_async::spi::Operation;
            let mut r = [0; 4];
            self.spi
                .transaction(&mut [
                    Operation::Write(&[super::CMD_READ, reg]),
                    Operation::Read(&mut r),
                ])
                .await
                .map_err(Error::bus)?;
            spi_value(reg, &r)
        }
    }

    impl<SPI: embedded_hal_async::spi::SpiDevice> Interface for SpiInterface<SPI> {
        type Error = SPI::Error;

        async fn request(&mut self, reg: u8) -> Result<WaitUs, SPI::Error> {
            self.pending = if reg == FULL_PACKET {
                Pending::Packet
            } else {
                Pending::Register(reg)
            };
            Ok(0)
        }

        async fn poll(&mut self) -> Result<Option<Response>, Error<SPI::Error>> {
            let pending = core::mem::replace(&mut self.pending, Pending::None);
            Ok(match pending {
                Pending::None => None,
                Pending::Register(reg) => Some(Response::Register(self.read_async(reg).await?)),
                Pending::Packet => {
                    let mut v = [0; 7];
                    for (out, reg) in v.iter_mut().zip(SPI_SNAPSHOT) {
                        *out = self.read_async(reg).await?;
                    }
                    Some(Response::Packet(raw_from_registers(&v)))
                }
            })
        }

        async fn write_reg(&mut self, reg: u8, value: u32) -> Result<(), SPI::Error> {
            self.spi.write(&spi_write_frame(reg, value)).await
        }
    }

    /// Async twin of [`super::Bl0942`].
    #[derive(Debug)]
    pub struct Bl0942<IFACE> {
        iface: IFACE,
        calibration: Calibration,
        energy: EnergyCounter,
    }

    constructors!(Bl0942);

    impl<IFACE: Interface> Bl0942<IFACE> {
        /// See [`super::Bl0942::request_register`].
        pub async fn request_register(&mut self, reg: u8) -> Result<WaitUs, Error<IFACE::Error>> {
            self.iface.request(reg).await.map_err(Error::bus)
        }

        /// See [`super::Bl0942::read_requested`].
        pub async fn read_requested(&mut self) -> Result<u32, Error<IFACE::Error>> {
            finish_register(self.iface.poll().await?)
        }

        /// See [`super::Bl0942::write_register`].
        pub async fn write_register(
            &mut self,
            reg: u8,
            value: u32,
        ) -> Result<(), Error<IFACE::Error>> {
            check_24bit(value)?;
            self.iface.write_reg(reg, value).await.map_err(Error::bus)
        }

        /// See [`super::Bl0942::unlock`].
        pub async fn unlock(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write_register(reg::USR_WRPROT, UNLOCK_KEY).await
        }

        /// See [`super::Bl0942::set_mode`].
        pub async fn set_mode(&mut self, mode: u32) -> Result<(), Error<IFACE::Error>> {
            self.unlock().await?;
            self.write_register(reg::MODE, mode).await
        }

        /// See [`super::Bl0942::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write_register(reg::SOFT_RESET, SOFT_RESET_KEY).await
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Bl0942<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.iface.request(FULL_PACKET).await.map_err(Error::bus)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            let raw = finish_packet(self.iface.poll().await?)?;
            Ok(convert(&self.calibration, &mut self.energy, &raw))
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
    use std::collections::VecDeque;
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    /// In-memory UART: `rx` is what the chip sends, `tx` what we wrote.
    #[derive(Default)]
    struct FakeUart {
        rx: VecDeque<u8>,
        tx: Vec<u8>,
        on_write: Vec<u8>,
    }

    impl embedded_io::ErrorType for FakeUart {
        type Error = core::convert::Infallible;
    }
    impl embedded_io::ReadReady for FakeUart {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            Ok(!self.rx.is_empty())
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
    impl embedded_io::Write for FakeUart {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.tx.extend_from_slice(buf);
            let reply = core::mem::take(&mut self.on_write);
            self.rx.extend(reply);
            Ok(buf.len())
        }
        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }
    impl embedded_io_async::Read for FakeUart {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            embedded_io::Read::read(self, buf)
        }
    }
    impl embedded_io_async::Write for FakeUart {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            embedded_io::Write::write(self, buf)
        }
    }

    /// 230 V, 1 A, cos φ = 1 on the reference circuit: Ip = 1 mV,
    /// Vp = 230 V · 510 / 1 950 510 = 60.138 mV (section 2 formulas).
    const I_RMS_1A: u32 = 251_213;
    const V_RMS_230V: u32 = 3_653_168;
    const WATT_230W: i32 = 143_381;

    fn packet(command: u8, raw: &Raw) -> [u8; PACKET_LEN] {
        let mut p = [0u8; PACKET_LEN];
        p[0] = PACKET_HEAD;
        p[1..4].copy_from_slice(&raw.i_rms.to_le_bytes()[..3]);
        p[4..7].copy_from_slice(&raw.v_rms.to_le_bytes()[..3]);
        p[7..10].copy_from_slice(&raw.i_fast_rms.to_le_bytes()[..3]);
        p[10..13].copy_from_slice(&raw.watt.to_le_bytes()[..3]);
        p[13..16].copy_from_slice(&raw.cf_cnt.to_le_bytes()[..3]);
        p[16..18].copy_from_slice(&raw.freq.to_le_bytes());
        p[19] = raw.status as u8;
        p[22] = checksum(&p[..22]).wrapping_sub(command);
        p
    }

    const RAW: Raw = Raw {
        i_rms: I_RMS_1A,
        v_rms: V_RMS_230V,
        i_fast_rms: 1_000,
        watt: WATT_230W,
        cf_cnt: 1_000,
        freq: 0x4E20,
        status: 0,
    };

    const EXPECTED: Measurement = Measurement {
        voltage: 229_998,
        current: 999_999,
        power: 230_001,
        energy: 186_893,
        frequency: Some(50_000),
        power_factor: Some(1000),
        status: 0,
    };

    #[test]
    fn reference_calibration_matches_datasheet_formulas() {
        let c = Calibration::REFERENCE;
        assert_eq!(c.current_q16, 260_878);
        assert_eq!(c.voltage_q20, 66_017);
        assert_eq!(c.power_q16, 105_128);
        assert_eq!(c.energy_q16, 12_248_273);
        assert_eq!(c.current_ua(I_RMS_1A), 999_999);
        assert_eq!(c.voltage_mv(V_RMS_230V), 229_998);
        assert_eq!(c.power_mw(WATT_230W), 230_001);
        assert_eq!(c.power_mw(-WATT_230W), -230_002);
        // One pulse per 419430.4 s / WATT at 230 W = 186.9 mWh.
        assert_eq!(c.energy_mwh(1), 186);
        assert_eq!(Calibration::from_circuit(0, 1, 1, 1), None);
        assert_eq!(frequency_mhz(0x4E20), Some(50_000));
        assert_eq!(frequency_mhz(16_667), Some(59_998));
        assert_eq!(power_factor(115_000, 230_000, 1_000_000), Some(500));
        assert_eq!(power_factor(1, 0, 5), None);
        assert_eq!(power_factor(-300_000, 230_000, 1_000_000), Some(-1000));
    }

    #[test]
    fn energy_counter_wraps_at_24_bits() {
        let mut e = EnergyCounter::new();
        assert_eq!(e.update(0xFF_FFF0), 0xFF_FFF0);
        assert_eq!(e.update(0x10), 0xFF_FFF0 + 0x20);
        e.set_total(0);
        assert_eq!(e.update(7), 0);
        assert_eq!(e.update(9), 2);
    }

    #[test]
    fn uart_full_packet() {
        let mut meter = Bl0942::new(UartInterface::new(FakeUart::default(), 2));
        assert_eq!(meter.start_measurement().unwrap(), 48_067);
        assert_eq!(meter.read_measurement(), Err(Error::NotReady));
        let p = packet(0x5A, &RAW);
        let uart = &mut meter.iface.uart;
        assert_eq!(uart.tx, vec![0x5A, 0xAA]);
        uart.rx.extend([0x00, 0x13]); // noise before the header is skipped
        uart.rx.extend(&p[..10]);
        assert_eq!(meter.read_measurement(), Err(Error::NotReady));
        meter.iface.uart.rx.extend(&p[10..]);
        assert_eq!(meter.read_measurement().unwrap(), EXPECTED);

        // Corrupted checksum, then a stale reply drained by the next request.
        meter.start_measurement().unwrap();
        let mut bad = p;
        bad[22] ^= 1;
        meter.iface.uart.rx.extend(bad);
        assert_eq!(meter.read_measurement(), Err(Error::Crc));
        meter.iface.uart.rx.extend(bad);
        meter.start_measurement().unwrap();
        let mut next = RAW;
        next.cf_cnt = 1_010;
        meter.iface.uart.rx.extend(packet(0x5A, &next));
        assert_eq!(meter.read_measurement().unwrap().energy, 188_762);
    }

    #[test]
    fn uart_register_access() {
        let mut meter = Bl0942::new(UartInterface::new(FakeUart::default(), 0).with_baud(9600));
        assert_eq!(meter.request_register(reg::MODE).unwrap(), 4_317);
        meter.iface.uart.rx.extend([0x87, 0x00, 0x00]);
        assert_eq!(meter.read_requested(), Err(Error::NotReady));
        meter.iface.uart.rx.push_back(0x07);
        assert_eq!(meter.read_requested(), Ok(0x87));
        meter
            .set_mode(mode::DEFAULT | mode::RMS_UPDATE_800MS)
            .unwrap();
        meter.soft_reset().unwrap();
        assert_eq!(
            meter.write_register(reg::MODE, 0x0100_0000),
            Err(Error::InvalidArgument)
        );
        // Section 3.2.3: [A8|addr, reg, L, M, H, !(sum)].
        assert_eq!(
            meter.iface.uart.tx[2..],
            [
                0xA8, 0x1D, 0x55, 0x00, 0x00, 0xE5, 0xA8, 0x19, 0x8F, 0x00, 0x00, 0xAF, 0xA8, 0x1C,
                0x5A, 0x5A, 0x5A, 0x2D,
            ]
        );
    }

    fn spi_read(reg: u8, value: u32) -> [SpiT<u8>; 4] {
        let [_, h, m, l] = value.to_be_bytes();
        [
            SpiT::transaction_start(),
            SpiT::write_vec(vec![CMD_READ, reg]),
            SpiT::read_vec(vec![h, m, l, checksum(&[CMD_READ, reg, h, m, l])]),
            SpiT::transaction_end(),
        ]
    }

    fn spi_snapshot(raw: &Raw) -> Vec<SpiT<u8>> {
        let values = [
            raw.i_rms,
            raw.v_rms,
            raw.i_fast_rms,
            raw.watt as u32 & 0xFF_FFFF,
            raw.cf_cnt,
            raw.freq as u32,
            raw.status as u32,
        ];
        SPI_SNAPSHOT
            .iter()
            .zip(values)
            .flat_map(|(r, v)| spi_read(*r, v))
            .collect()
    }

    #[test]
    fn spi_frames() {
        let mut expectations = spi_snapshot(&RAW);
        expectations.extend([
            SpiT::transaction_start(),
            SpiT::write_vec(vec![0xA8, 0x1D, 0x00, 0x00, 0x55, 0xE5]),
            SpiT::transaction_end(),
        ]);
        expectations.extend(spi_read(reg::WATT, 0x80_0000));
        let mut bad = spi_read(reg::FREQ, 0x4E20);
        bad[2] = SpiT::read_vec(vec![0x00, 0x4E, 0x20, 0x00]);
        expectations.extend(bad);
        let mut spi = SpiMock::new(&expectations);
        let mut meter = Bl0942::new(SpiInterface::new(&mut spi));
        assert_eq!(meter.read_measurement(), Err(Error::NotReady));
        assert_eq!(meter.start_measurement().unwrap(), 0);
        assert_eq!(meter.read_measurement().unwrap(), EXPECTED);
        meter.unlock().unwrap();
        meter.request_register(reg::WATT).unwrap();
        assert_eq!(meter.read_requested(), Ok(0x80_0000));
        meter.request_register(reg::FREQ).unwrap();
        assert_eq!(meter.read_requested(), Err(Error::Crc));
        spi.done();
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

        let mut spi = SpiMock::new(&spi_snapshot(&RAW));
        let mut meter = asynch::Bl0942::new(SpiInterface::new(&mut spi));
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(meter.measure(&mut delay)).unwrap(), EXPECTED);
        assert_eq!(delay.0, 0);
        spi.done();

        let mut uart = FakeUart::default();
        uart.rx.extend([0x55; 5]); // stale bytes are drained by the request
        uart.on_write = packet(0x58, &RAW).to_vec();
        let mut meter = asynch::Bl0942::new(UartInterface::new(uart, 0));
        assert_eq!(block_on(meter.measure(&mut delay)).unwrap(), EXPECTED);
        assert_eq!(delay.0, 48_067);
        assert_eq!(meter.release().release().tx, vec![0x58, 0xAA]);
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
