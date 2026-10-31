//! Peacefair PZEM-004T v3.0 / PZEM-016 (AC) and PZEM-003 / PZEM-017 (DC)
//! energy monitor modules over Modbus RTU.
//!
//! The modules answer Modbus RTU requests at 9600 bit/s (AC: 8N1, DC:
//! 8N2) over TTL UART (PZEM-004T) or RS-485 (PZEM-016 / PZEM-017).
//! Supported functions: 0x04 read input registers (measurements), 0x03 /
//! 0x06 read / write holding registers (alarm thresholds, address, shunt
//! range) and 0x42 reset energy. Every frame ends with
//! [`zigbee_sensor::crc::crc16_modbus`], low byte first.
//!
//! The driver needs [`embedded_io::ReadReady`] so it never blocks on a
//! silent module:
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! drains stale bytes, sends the request and returns the transfer time
//! plus [`RESPONSE_MARGIN_US`];
//! [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//! parses what has arrived and returns [`Error::NotReady`] while the reply
//! is incomplete (a module that never answers keeps returning `NotReady`;
//! the caller decides when to give up).
//!
//! The modules are mains powered and have no sleep mode, so `LowPower` is
//! not implemented.
//!
//! Reference: Peacefair "PZEM-004T V3.0 User Manual" and "PZEM-003/017 DC
//! communication module" manual (section 2, communication protocol).
//!
//! ```text
//! let mut pzem = Pzem::new(uart, Model::Ac).with_address(0x01);
//! let wait_us = pzem.start_measurement()?;
//! // ... wait_us later:
//! let m = pzem.read_measurement()?; // NotReady until the reply arrived
//! ```

#![no_std]

use zigbee_sensor::crc::crc16_modbus;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// General address: accepted by any module (single slave on the bus only).
pub const GENERAL_ADDRESS: u8 = 0xF8;
/// Function: read holding registers (parameters).
pub const FN_READ_HOLDING: u8 = 0x03;
/// Function: read input registers (measurements).
pub const FN_READ_INPUT: u8 = 0x04;
/// Function: write single holding register.
pub const FN_WRITE_SINGLE: u8 = 0x06;
/// Function: reset the energy counter.
pub const FN_RESET_ENERGY: u8 = 0x42;
/// Assumed module processing time added to the transfer time (the
/// manuals give none).
pub const RESPONSE_MARGIN_US: u32 = 50_000;
/// UART bytes discarded at most per request.
const MAX_DRAIN_BYTES: usize = 64;
/// UART bytes parsed at most per poll.
const MAX_READ_BYTES: usize = 64;
/// Longest reply: 5 + 2 · 10 registers.
const MAX_FRAME: usize = 25;

/// Holding registers (parameters).
pub mod param {
    /// AC: over-power alarm threshold, 1 W.
    pub const POWER_ALARM: u16 = 0x0001;
    /// DC: high-voltage alarm threshold, 0.01 V (5-350 V, default 300 V).
    pub const HIGH_VOLTAGE_ALARM: u16 = 0x0000;
    /// DC: low-voltage alarm threshold, 0.01 V (1-350 V, default 7 V).
    pub const LOW_VOLTAGE_ALARM: u16 = 0x0001;
    /// Modbus address 0x01-0xF7 (both models).
    pub const ADDRESS: u16 = 0x0002;
    /// PZEM-017: shunt range, see [`super::ShuntRange`].
    pub const SHUNT_RANGE: u16 = 0x0003;
}

/// Module family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Model {
    /// PZEM-004T v3.0 (TTL) / PZEM-016 (RS-485): 10 input registers.
    Ac,
    /// PZEM-003 / PZEM-017 (RS-485, 8N2): 8 input registers.
    Dc,
}

impl Model {
    /// Input registers holding one measurement.
    pub const fn input_registers(self) -> u16 {
        match self {
            Self::Ac => 10,
            Self::Dc => 8,
        }
    }

    const fn bits_per_byte(self) -> u32 {
        match self {
            Self::Ac => 10,
            Self::Dc => 11,
        }
    }
}

/// PZEM-017 external shunt ([`param::SHUNT_RANGE`] values).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ShuntRange {
    /// 100 A (default).
    A100 = 0,
    /// 50 A.
    A50 = 1,
    /// 200 A.
    A200 = 2,
    /// 300 A.
    A300 = 3,
}

/// One reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Voltage, mV (AC: RMS).
    pub voltage: i32,
    /// Current, µA (AC: RMS).
    pub current: i32,
    /// Active power, mW.
    pub power: i32,
    /// Energy since the last reset, mWh (1 Wh resolution).
    pub energy: u64,
    /// AC line frequency, mHz.
    pub frequency: Option<u32>,
    /// AC power factor, 0.001 (0.01 resolution).
    pub power_factor: Option<i16>,
    /// AC: power above [`param::POWER_ALARM`].
    pub power_alarm: bool,
    /// DC: voltage above [`param::HIGH_VOLTAGE_ALARM`].
    pub high_voltage_alarm: bool,
    /// DC: voltage below [`param::LOW_VOLTAGE_ALARM`].
    pub low_voltage_alarm: bool,
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

const fn sat_i32(v: u64) -> i32 {
    if v > i32::MAX as u64 {
        i32::MAX
    } else {
        v as i32
    }
}

/// 32-bit value stored low word first.
const fn u32_lo_hi(lo: u16, hi: u16) -> u32 {
    (hi as u32) << 16 | lo as u32
}

/// Decode the input registers of `model` (`regs` must hold
/// [`Model::input_registers`] values).
pub fn decode(model: Model, regs: &[u16]) -> Measurement {
    match model {
        Model::Ac => Measurement {
            voltage: regs[0] as i32 * 100,
            current: sat_i32(u32_lo_hi(regs[1], regs[2]) as u64 * 1_000),
            power: sat_i32(u32_lo_hi(regs[3], regs[4]) as u64 * 100),
            energy: u32_lo_hi(regs[5], regs[6]) as u64 * 1_000,
            frequency: Some(regs[7] as u32 * 100),
            power_factor: Some((regs[8].min(100) * 10) as i16),
            power_alarm: regs[9] != 0,
            high_voltage_alarm: false,
            low_voltage_alarm: false,
        },
        Model::Dc => Measurement {
            voltage: regs[0] as i32 * 10,
            current: regs[1] as i32 * 10_000,
            power: sat_i32(u32_lo_hi(regs[2], regs[3]) as u64 * 100),
            energy: u32_lo_hi(regs[4], regs[5]) as u64 * 1_000,
            frequency: None,
            power_factor: None,
            power_alarm: false,
            high_voltage_alarm: regs[6] != 0,
            low_voltage_alarm: regs[7] != 0,
        },
    }
}

/// Outstanding request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    None,
    Measure,
    Read(u16),
    Write(u16, u16),
    ResetEnergy,
}

impl Pending {
    const fn function(self) -> u8 {
        match self {
            Self::None | Self::Measure => FN_READ_INPUT,
            Self::Read(_) => FN_READ_HOLDING,
            Self::Write(..) => FN_WRITE_SINGLE,
            Self::ResetEnergy => FN_RESET_ENERGY,
        }
    }
}

/// Completed reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Reply {
    Registers([u16; 10]),
    Value(u16),
    Done,
}

fn check_write<E>(reg: u16, value: u16) -> Result<(), Error<E>> {
    if reg == param::ADDRESS && !(0x01..=0xF7).contains(&value) {
        Err(Error::InvalidArgument)
    } else {
        Ok(())
    }
}

fn with_crc<const N: usize>(mut f: [u8; N]) -> [u8; N] {
    let crc = crc16_modbus(&f[..N - 2]).to_le_bytes();
    f[N - 2..].copy_from_slice(&crc);
    f
}

/// Modbus RTU request builder and reply assembler.
#[derive(Debug)]
struct Link {
    model: Model,
    address: u8,
    pending: Pending,
    buf: [u8; MAX_FRAME],
    len: usize,
}

impl Link {
    const fn new(model: Model) -> Self {
        Self {
            model,
            address: GENERAL_ADDRESS,
            pending: Pending::None,
            buf: [0; MAX_FRAME],
            len: 0,
        }
    }

    /// Request bytes for `p` (8 bytes, or 4 for a reset) and the time
    /// until the reply is complete.
    fn request(&mut self, p: Pending) -> ([u8; 8], usize, WaitUs) {
        self.pending = p;
        self.len = 0;
        let a = self.address;
        let (frame, len, reply) = match p {
            Pending::None | Pending::Measure => {
                let n = self.model.input_registers();
                let [h, l] = n.to_be_bytes();
                let f = with_crc([a, FN_READ_INPUT, 0, 0, h, l, 0, 0]);
                (f, 8, 5 + 2 * n as u32)
            }
            Pending::Read(reg) => {
                let [rh, rl] = reg.to_be_bytes();
                (with_crc([a, FN_READ_HOLDING, rh, rl, 0, 1, 0, 0]), 8, 7)
            }
            Pending::Write(reg, v) => {
                let [rh, rl] = reg.to_be_bytes();
                let [vh, vl] = v.to_be_bytes();
                (with_crc([a, FN_WRITE_SINGLE, rh, rl, vh, vl, 0, 0]), 8, 8)
            }
            Pending::ResetEnergy => {
                let [b0, b1, c0, c1] = with_crc([a, FN_RESET_ENERGY, 0, 0]);
                ([b0, b1, c0, c1, 0, 0, 0, 0], 4, 4)
            }
        };
        let bits = (len as u32 + reply) * self.model.bits_per_byte();
        let wait = (bits * 1_000_000).div_ceil(9600) + RESPONSE_MARGIN_US;
        (frame, len, wait)
    }

    fn accepts_address(&self, b: u8) -> bool {
        // The general address is answered with the module's own address.
        self.address == GENERAL_ADDRESS || b == self.address
    }

    fn expected_len(&self) -> Option<usize> {
        if self.len < 2 {
            return None;
        }
        if self.buf[1] & 0x80 != 0 {
            return Some(5);
        }
        Some(match self.pending {
            Pending::None | Pending::Measure | Pending::Read(_) if self.len < 3 => return None,
            Pending::None | Pending::Measure | Pending::Read(_) => {
                (5 + self.buf[2] as usize).min(MAX_FRAME)
            }
            Pending::Write(..) => 8,
            Pending::ResetEnergy => 4,
        })
    }

    fn feed<E>(&mut self, b: u8) -> Option<Result<Reply, Error<E>>> {
        let function = self.pending.function();
        match self.len {
            0 if !self.accepts_address(b) => return None,
            1 if b != function && b != function | 0x80 => {
                self.len = usize::from(self.accepts_address(b));
                self.buf[0] = b;
                return None;
            }
            _ => {}
        }
        self.buf[self.len] = b;
        self.len += 1;
        match self.expected_len() {
            Some(n) if self.len >= n => {}
            _ => return None,
        }
        let pending = self.pending;
        self.pending = Pending::None;
        let f = &self.buf[..self.len];
        self.len = 0;
        Some(Self::parse(self.model, pending, f))
    }

    fn parse<E>(model: Model, pending: Pending, f: &[u8]) -> Result<Reply, Error<E>> {
        let n = f.len();
        if crc16_modbus(&f[..n - 2]).to_le_bytes() != [f[n - 2], f[n - 1]] {
            return Err(Error::Crc);
        }
        if f[1] & 0x80 != 0 {
            // Exception 0x01 = illegal function.
            return Err(if f[2] == 0x01 {
                Error::Unsupported
            } else {
                Error::InvalidData
            });
        }
        let word = |i: usize| u16::from_be_bytes([f[i], f[i + 1]]);
        match pending {
            Pending::Measure | Pending::None => {
                let count = model.input_registers() as usize;
                if f[2] as usize != 2 * count {
                    return Err(Error::InvalidData);
                }
                let mut regs = [0u16; 10];
                for (i, r) in regs.iter_mut().take(count).enumerate() {
                    *r = word(3 + 2 * i);
                }
                Ok(Reply::Registers(regs))
            }
            Pending::Read(_) if f[2] == 2 => Ok(Reply::Value(word(3))),
            Pending::Write(reg, v) if word(2) == reg && word(4) == v => Ok(Reply::Value(v)),
            Pending::ResetEnergy => Ok(Reply::Done),
            _ => Err(Error::InvalidData),
        }
    }

    fn finish_measurement<E>(&self, r: Option<Reply>) -> Result<Measurement, Error<E>> {
        match r {
            Some(Reply::Registers(regs)) => Ok(decode(self.model, &regs)),
            Some(_) => Err(Error::InvalidData),
            None => Err(Error::NotReady),
        }
    }

    fn finish_command<E>(&mut self, sent: Pending, r: Option<Reply>) -> Result<u16, Error<E>> {
        match (sent, r) {
            (_, None) => Err(Error::NotReady),
            (Pending::Write(param::ADDRESS, a), Some(Reply::Value(_))) => {
                self.address = a as u8;
                Ok(a)
            }
            (_, Some(Reply::Value(v))) => Ok(v),
            (_, Some(Reply::Done)) => Ok(0),
            (_, Some(Reply::Registers(_))) => Err(Error::InvalidData),
        }
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<U> $name<U> {
            /// Driver for `model` at the [`GENERAL_ADDRESS`](crate::GENERAL_ADDRESS); no bus
            /// traffic.
            pub const fn new(uart: U, model: Model) -> Self {
                Self {
                    uart,
                    link: Link::new(model),
                    sent: Pending::None,
                }
            }

            /// Modbus address 0x01-0xF7 (or [`GENERAL_ADDRESS`](crate::GENERAL_ADDRESS) when the
            /// module is alone on the bus).
            pub const fn with_address(mut self, address: u8) -> Self {
                self.link.address = address;
                self
            }

            /// Current Modbus address.
            pub const fn address(&self) -> u8 {
                self.link.address
            }

            /// Module family.
            pub const fn model(&self) -> Model {
                self.link.model
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
pub struct Pzem<U> {
    uart: U,
    link: Link,
    sent: Pending,
}

constructors!(Pzem);

impl<U> Pzem<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    fn send(&mut self, p: Pending) -> Result<WaitUs, Error<U::Error>> {
        let mut buf = [0u8; 16];
        let mut left = MAX_DRAIN_BYTES;
        while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
            let n = self.uart.read(&mut buf).map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            left = left.saturating_sub(n);
        }
        let (frame, len, wait) = self.link.request(p);
        self.sent = p;
        self.uart.write_all(&frame[..len]).map_err(Error::bus)?;
        self.uart.flush().map_err(Error::bus)?;
        Ok(wait)
    }

    fn poll(&mut self) -> Result<Option<Reply>, Error<U::Error>> {
        let mut buf = [0u8; MAX_FRAME];
        let mut left = MAX_READ_BYTES;
        while left > 0
            && self.link.pending != Pending::None
            && self.uart.read_ready().map_err(Error::bus)?
        {
            let want = left.min(MAX_FRAME - self.link.len).max(1);
            let n = self.uart.read(&mut buf[..want]).map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            left = left.saturating_sub(n);
            for b in &buf[..n] {
                if let Some(r) = self.link.feed(*b) {
                    return r.map(Some);
                }
            }
        }
        Ok(None)
    }

    /// Request a holding register (see [`param`]); fetch it with
    /// [`Self::read_reply`] after the returned time.
    pub fn request_parameter(&mut self, reg: u16) -> Result<WaitUs, Error<U::Error>> {
        self.send(Pending::Read(reg))
    }

    /// Write a holding register (see [`param`]); confirm with
    /// [`Self::read_reply`]. Writing [`param::ADDRESS`] switches the
    /// driver to the new address once confirmed; an address outside
    /// 0x01-0xF7 is [`Error::InvalidArgument`].
    pub fn request_write(&mut self, reg: u16, value: u16) -> Result<WaitUs, Error<U::Error>> {
        check_write(reg, value)?;
        self.send(Pending::Write(reg, value))
    }

    /// PZEM-017: select the external shunt.
    pub fn request_shunt_range(&mut self, range: ShuntRange) -> Result<WaitUs, Error<U::Error>> {
        self.request_write(param::SHUNT_RANGE, range as u16)
    }

    /// Reset the energy counter; confirm with [`Self::read_reply`].
    pub fn request_reset_energy(&mut self) -> Result<WaitUs, Error<U::Error>> {
        self.send(Pending::ResetEnergy)
    }

    /// Reply to the last parameter request: the register value for a
    /// read, the written value for a write, 0 for an energy reset.
    /// [`Error::NotReady`] while incomplete, [`Error::Unsupported`] /
    /// [`Error::InvalidData`] for a Modbus exception or a wrong echo.
    pub fn read_reply(&mut self) -> Result<u16, Error<U::Error>> {
        let r = self.poll()?;
        self.link.finish_command(self.sent, r)
    }
}

impl<U> zigbee_sensor::blocking::Sensor for Pzem<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type BusError = U::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        self.send(Pending::Measure)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
        let r = self.poll()?;
        self.link.finish_measurement(r)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        Link, MAX_DRAIN_BYTES, MAX_FRAME, MAX_READ_BYTES, Measurement, Model, Pending, Reply,
        ShuntRange, check_write, param,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Pzem`].
    #[derive(Debug)]
    pub struct Pzem<U> {
        uart: U,
        link: Link,
        sent: Pending,
    }

    constructors!(Pzem);

    impl<U> Pzem<U>
    where
        U: embedded_io_async::Read + embedded_io_async::ReadReady + embedded_io_async::Write,
    {
        async fn send(&mut self, p: Pending) -> Result<WaitUs, Error<U::Error>> {
            let mut buf = [0u8; 16];
            let mut left = MAX_DRAIN_BYTES;
            while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
                let n = self.uart.read(&mut buf).await.map_err(Error::bus)?;
                if n == 0 {
                    break;
                }
                left = left.saturating_sub(n);
            }
            let (frame, len, wait) = self.link.request(p);
            self.sent = p;
            self.uart
                .write_all(&frame[..len])
                .await
                .map_err(Error::bus)?;
            self.uart.flush().await.map_err(Error::bus)?;
            Ok(wait)
        }

        async fn poll(&mut self) -> Result<Option<Reply>, Error<U::Error>> {
            let mut buf = [0u8; MAX_FRAME];
            let mut left = MAX_READ_BYTES;
            while left > 0
                && self.link.pending != Pending::None
                && self.uart.read_ready().map_err(Error::bus)?
            {
                let want = left.min(MAX_FRAME - self.link.len).max(1);
                let n = self.uart.read(&mut buf[..want]).await.map_err(Error::bus)?;
                if n == 0 {
                    break;
                }
                left = left.saturating_sub(n);
                for b in &buf[..n] {
                    if let Some(r) = self.link.feed(*b) {
                        return r.map(Some);
                    }
                }
            }
            Ok(None)
        }

        /// See [`super::Pzem::request_parameter`].
        pub async fn request_parameter(&mut self, reg: u16) -> Result<WaitUs, Error<U::Error>> {
            self.send(Pending::Read(reg)).await
        }

        /// See [`super::Pzem::request_write`].
        pub async fn request_write(
            &mut self,
            reg: u16,
            value: u16,
        ) -> Result<WaitUs, Error<U::Error>> {
            check_write(reg, value)?;
            self.send(Pending::Write(reg, value)).await
        }

        /// See [`super::Pzem::request_shunt_range`].
        pub async fn request_shunt_range(
            &mut self,
            range: ShuntRange,
        ) -> Result<WaitUs, Error<U::Error>> {
            self.request_write(param::SHUNT_RANGE, range as u16).await
        }

        /// See [`super::Pzem::request_reset_energy`].
        pub async fn request_reset_energy(&mut self) -> Result<WaitUs, Error<U::Error>> {
            self.send(Pending::ResetEnergy).await
        }

        /// See [`super::Pzem::read_reply`].
        pub async fn read_reply(&mut self) -> Result<u16, Error<U::Error>> {
            let r = self.poll().await?;
            self.link.finish_command(self.sent, r)
        }
    }

    impl<U> zigbee_sensor::asynch::Sensor for Pzem<U>
    where
        U: embedded_io_async::Read + embedded_io_async::ReadReady + embedded_io_async::Write,
    {
        type BusError = U::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
            self.send(Pending::Measure).await
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
            let r = self.poll().await?;
            self.link.finish_measurement(r)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::collections::VecDeque;
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    /// In-memory UART: `rx` is what the module sends, `tx` what we wrote;
    /// `on_write` is queued into `rx` on the next write.
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

    fn crc(mut f: Vec<u8>) -> Vec<u8> {
        let c = crc16_modbus(&f).to_le_bytes();
        f.extend(c);
        f
    }

    /// PZEM-004T manual 2.3 example: 220.0 V, 1.000 A, 220.0 W, 0 Wh,
    /// 50.0 Hz, PF 1.00, no alarm.
    fn ac_reply(addr: u8) -> Vec<u8> {
        crc(vec![
            addr, 0x04, 0x14, 0x08, 0x98, 0x03, 0xE8, 0x00, 0x00, 0x08, 0x98, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01, 0xF4, 0x00, 0x64, 0x00, 0x00,
        ])
    }

    const AC: Measurement = Measurement {
        voltage: 220_000,
        current: 1_000_000,
        power: 220_000,
        energy: 0,
        frequency: Some(50_000),
        power_factor: Some(1000),
        power_alarm: false,
        high_voltage_alarm: false,
        low_voltage_alarm: false,
    };

    #[test]
    fn ac_measurement() {
        let mut uart = FakeUart::default();
        uart.rx.extend([0x01, 0x04]); // stale, drained by the request
        let mut pzem = Pzem::new(uart, Model::Ac);
        // (8 + 25) bytes · 10 bits at 9600 bit/s + 50 ms.
        assert_eq!(pzem.start_measurement(), Ok(84_375));
        assert_eq!(
            pzem.uart.tx,
            [0xF8, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x64, 0x64]
        );
        assert_eq!(pzem.read_measurement(), Err(Error::NotReady));
        // General address: the module answers with its own address.
        let reply = ac_reply(0x01);
        pzem.uart.rx.extend(&reply[..9]);
        assert_eq!(pzem.read_measurement(), Err(Error::NotReady));
        pzem.uart.rx.extend(&reply[9..]);
        assert_eq!(pzem.read_measurement(), Ok(AC));

        // Fixed address: replies from other modules are skipped; 32-bit
        // values are low word first; alarm set.
        let mut pzem = Pzem::new(pzem.release(), Model::Ac).with_address(0x01);
        pzem.uart.tx.clear();
        pzem.start_measurement().unwrap();
        assert_eq!(
            pzem.uart.tx,
            [0x01, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x70, 0x0D]
        );
        let mut regs = vec![0x01, 0x04, 0x14];
        for w in [
            2301u16, 0x86A0, 0x0001, 0x5A0C, 0x0000, 0x2345, 0x0001, 599, 95, 0xFFFF,
        ] {
            regs.extend(w.to_be_bytes());
        }
        pzem.uart.rx.extend([0x07]);
        pzem.uart.rx.extend(crc(regs));
        let m = pzem.read_measurement().unwrap();
        assert_eq!(
            (m.voltage, m.current, m.power, m.energy),
            (230_100, 100_000_000, 2_305_200, 74_565_000)
        );
        assert_eq!(
            (m.frequency, m.power_factor, m.power_alarm),
            (Some(59_900), Some(950), true)
        );

        // Corrupt CRC, then a Modbus exception.
        pzem.start_measurement().unwrap();
        let mut bad = ac_reply(0x01);
        bad[5] ^= 1;
        pzem.uart.rx.extend(bad);
        assert_eq!(pzem.read_measurement(), Err(Error::Crc));
        pzem.start_measurement().unwrap();
        pzem.uart.rx.extend(crc(vec![0x01, 0x84, 0x02]));
        assert_eq!(pzem.read_measurement(), Err(Error::InvalidData));
    }

    #[test]
    fn dc_measurement() {
        let mut pzem = Pzem::new(FakeUart::default(), Model::Dc).with_address(0x01);
        // (8 + 21) bytes · 11 bits at 9600 bit/s + 50 ms.
        assert_eq!(pzem.start_measurement(), Ok(83_230));
        assert_eq!(pzem.uart.tx[..6], [0x01, 0x04, 0x00, 0x00, 0x00, 0x08]);
        // PZEM-017 manual 2.3 example: 100.00 V, 1.00 A, 100.0 W; LV alarm.
        let mut r = vec![0x01, 0x04, 0x10, 0x27, 0x10, 0x00, 0x64, 0x03, 0xE8];
        r.extend([0; 6]);
        r.extend([0x00, 0x00, 0xFF, 0xFF]);
        pzem.uart.rx.extend(crc(r));
        assert_eq!(
            pzem.read_measurement(),
            Ok(Measurement {
                voltage: 100_000,
                current: 1_000_000,
                power: 100_000,
                energy: 0,
                frequency: None,
                power_factor: None,
                power_alarm: false,
                high_voltage_alarm: false,
                low_voltage_alarm: true,
            })
        );
    }

    #[test]
    fn parameters() {
        let mut pzem = Pzem::new(FakeUart::default(), Model::Ac).with_address(0x01);
        // Manual 2.4: alarm threshold 2300 W; the module echoes the request.
        pzem.uart.on_write = crc(vec![0x01, 0x06, 0x00, 0x01, 0x08, 0xFC]);
        pzem.request_write(param::POWER_ALARM, 2300).unwrap();
        assert_eq!(pzem.uart.tx, crc(vec![0x01, 0x06, 0x00, 0x01, 0x08, 0xFC]));
        assert_eq!(pzem.read_reply(), Ok(2300));

        pzem.uart.tx.clear();
        pzem.uart.on_write = crc(vec![0x01, 0x03, 0x02, 0x08, 0xFC]);
        pzem.request_parameter(param::POWER_ALARM).unwrap();
        assert_eq!(pzem.uart.tx, crc(vec![0x01, 0x03, 0x00, 0x01, 0x00, 0x01]));
        assert_eq!(pzem.read_reply(), Ok(2300));

        assert_eq!(
            pzem.request_write(param::ADDRESS, 0xF8),
            Err(Error::InvalidArgument)
        );
        // Address change takes effect once confirmed.
        pzem.uart.on_write = crc(vec![0x01, 0x06, 0x00, 0x02, 0x00, 0x05]);
        pzem.request_write(param::ADDRESS, 5).unwrap();
        assert_eq!(pzem.address(), 1);
        assert_eq!(pzem.read_reply(), Ok(5));
        assert_eq!(pzem.address(), 5);

        // Energy reset (4 bytes), wrong echo, illegal function.
        pzem.uart.tx.clear();
        pzem.uart.on_write = crc(vec![0x05, 0x42]);
        assert_eq!(pzem.request_reset_energy(), Ok(58_334));
        assert_eq!(pzem.uart.tx, crc(vec![0x05, 0x42]));
        assert_eq!(pzem.read_reply(), Ok(0));
        pzem.uart.on_write = crc(vec![0x05, 0x06, 0x00, 0x03, 0x00, 0x09]);
        pzem.request_shunt_range(ShuntRange::A300).unwrap();
        assert_eq!(pzem.read_reply(), Err(Error::InvalidData));
        pzem.uart.on_write = crc(vec![0x05, 0xC2, 0x01]);
        pzem.request_reset_energy().unwrap();
        assert_eq!(pzem.read_reply(), Err(Error::Unsupported));
        assert_eq!(pzem.read_reply(), Err(Error::NotReady));
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
        uart.rx.extend([0x55; 5]);
        uart.on_write = ac_reply(0x01);
        let mut pzem = asynch::Pzem::new(uart, Model::Ac);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(pzem.measure(&mut delay)), Ok(AC));
        assert_eq!(delay.0, 84_375);
        let mut uart = pzem.release();
        uart.on_write = crc(vec![0x01, 0x42]);
        let mut pzem = asynch::Pzem::new(uart, Model::Ac);
        block_on(pzem.request_reset_energy()).unwrap();
        assert_eq!(block_on(pzem.read_reply()), Ok(0));
        assert_eq!(pzem.release().tx[8..], [0xF8, 0x42, 0xC2, 0x41]);
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
