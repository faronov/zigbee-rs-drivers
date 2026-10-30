//! GSS / SST Sensing ExplorIR, SprintIR and CozIR (also MISIR, MinIR) NDIR
//! CO2 sensors over their ASCII UART protocol.
//!
//! UART 9600 8N1, 3.3 V levels. Commands are ASCII, case sensitive and end
//! in `\r\n`; parameters follow the command letter after a space. Every
//! response starts with a space and ends in `\r\n`; `?` means the command
//! was not recognised. There is no checksum, so the [`Parser`] validates the
//! syntax instead (`<id> <5 digits>` fields).
//!
//! Operating modes ([`Mode`], `K` command):
//!
//! - **Streaming** (`K 1`, factory default): readings such as
//!   ` Z 00842 z 00765\r\n` are sent twice per second (20 Hz on SprintIR).
//!   [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//!   returns 0 and
//!   [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//!   returns the newest line or [`Error::NotReady`].
//! - **Polling** (`K 2`): `start_measurement` sends `Q` (all fields enabled
//!   by the `M` mask) and returns [`RESPONSE_US`].
//! - **Command** (`K 0`): measurement stopped (used by
//!   [`LowPower::sleep`](zigbee_sensor::blocking::LowPower::sleep)); not
//!   retained over a power cycle — the sensor powers up in the last of
//!   streaming/polling. Leaving it runs a warm-up ([`warm_up_us`]).
//!
//! `Z`/`z` values are in sensor units; multiply by the `.` multiplier
//! (1 for ambient sensors, 10 or 100 for wide-range ones) to get ppm. The
//! driver applies the multiplier set with `with_multiplier` or read with
//! `request_multiplier` / `read_multiplier`.
//!
//! References: GSS "Sensor User's Manual — COZIR, SprintIR, MISIR and MinIR
//! Sensors" Rev. I (August 2015); SST Sensing "Quick Start Guide CO2
//! Sensors — CozIR, ExplorIR and SprintIR ranges" QS-003 Rev 1 (2018).
//!
//! ```ignore
//! let mut co2 = ExplorIr::new(uart).with_mode(Mode::Polling);
//! let wait_us = co2.start_measurement()?;
//! // ... wait_us later:
//! let m = co2.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Longest encoded command including `\r\n`.
pub const MAX_COMMAND_LEN: usize = 32;
/// Longest accepted response line (without `\r\n`); longer lines are
/// dropped.
pub const MAX_LINE_LEN: usize = 64;

/// Command reply allowance: the manual allows up to 100 ms command latency
/// during measurement activity plus ≈ 42 ms for a 40-character line at
/// 9600 Bd.
pub const RESPONSE_US: WaitUs = 150_000;
/// Streaming interval of CozIR / ExplorIR / MISIR / MinIR (2 Hz).
pub const STREAM_INTERVAL_US: WaitUs = 500_000;
/// Factory-default digital filter setting.
pub const DEFAULT_FILTER: u16 = 32;

const MAX_READ_BYTES: usize = 256;
const MAX_DRAIN_BYTES: usize = 256;

/// `M` command output-field mask bits (up to five fields are sent, highest
/// mask values first).
pub mod field {
    /// Humidity `H` (factory option).
    pub const HUMIDITY: u16 = 4096;
    /// Temperature `T` (factory option).
    pub const TEMPERATURE: u16 = 64;
    /// Digitally filtered CO2 `Z`.
    pub const CO2_FILTERED: u16 = 4;
    /// Unfiltered CO2 `z`.
    pub const CO2_RAW: u16 = 2;
    /// Factory default (`Z` and `z`).
    pub const DEFAULT: u16 = CO2_FILTERED | CO2_RAW;
}

/// Warm-up time after power-up or after leaving command mode for a digital
/// filter setting (manual table: 1 → 1.2 s, 2 → 3 s, 4 → 5 s, 8 → 9 s,
/// 16 → 16 s, 32 → 32 s; roughly the filter value in seconds). The smart
/// filter (0) is treated like the default 32.
pub const fn warm_up_us(filter: u16) -> WaitUs {
    let s = match filter {
        0 => DEFAULT_FILTER as u32,
        1 => return 1_200_000,
        2..=8 => filter as u32 + 1,
        _ => filter as u32,
    };
    s * 1_000_000
}

/// An encoded ASCII command, terminated by `\r\n`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Command {
    buf: [u8; MAX_COMMAND_LEN],
    len: u8,
}

impl Command {
    /// Command with no parameters, e.g. `Command::new(b'G')` → `G\r\n`.
    pub const fn new(id: u8) -> Self {
        let mut buf = [0u8; MAX_COMMAND_LEN];
        buf[0] = id;
        buf[1] = b'\r';
        buf[2] = b'\n';
        Self { buf, len: 3 }
    }

    fn push(&mut self, b: u8) {
        let body = usize::from(self.len) - 2;
        if body + 3 <= MAX_COMMAND_LEN {
            self.buf[body] = b;
            self.buf[body + 1] = b'\r';
            self.buf[body + 2] = b'\n';
            self.len += 1;
        }
    }

    fn push_decimal(&mut self, v: u32) {
        let mut digits = [0u8; 10];
        let mut n = 0;
        let mut v = v;
        loop {
            digits[n] = b'0' + (v % 10) as u8;
            n += 1;
            v /= 10;
            if v == 0 {
                break;
            }
        }
        for d in digits[..n].iter().rev() {
            self.push(*d);
        }
    }

    /// Append ` <decimal>`.
    pub fn arg(mut self, v: u32) -> Self {
        self.push(b' ');
        self.push_decimal(v);
        self
    }

    /// Append ` <i>.<f>` from a value in tenths (used by `@`).
    pub fn arg_tenths(mut self, tenths: u16) -> Self {
        self.push(b' ');
        self.push_decimal(u32::from(tenths / 10));
        self.push(b'.');
        self.push_decimal(u32::from(tenths % 10));
        self
    }

    /// Bytes to send, including `\r\n`.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..usize::from(self.len)]
    }

    /// Command letter.
    pub const fn id(&self) -> u8 {
        self.buf[0]
    }
}

/// Operating mode (`K` command).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// `K 0`: measurement stopped, commands only.
    Command,
    /// `K 1`: readings streamed continuously (factory default).
    #[default]
    Streaming,
    /// `K 2`: readings only on request.
    Polling,
}

impl Mode {
    const fn number(self) -> u32 {
        match self {
            Self::Command => 0,
            Self::Streaming => 1,
            Self::Polling => 2,
        }
    }
}

/// `K n`: select the operating mode; echoed as ` K 0000n`.
pub fn mode_command(mode: Mode) -> Command {
    Command::new(b'K').arg(mode.number())
}

/// `Q`: report all fields enabled by the `M` mask (polling).
pub fn poll_command() -> Command {
    Command::new(b'Q')
}

/// `A n`: set the digital filter (1–65535, 0 = smart filter, default 32).
pub fn set_filter_command(filter: u16) -> Command {
    Command::new(b'A').arg(u32::from(filter))
}

/// `a`: read the digital filter (` a 00032`).
pub fn read_filter_command() -> Command {
    Command::new(b'a')
}

/// `M n`: set the output fields (see [`field`]).
pub fn set_fields_command(mask: u16) -> Command {
    Command::new(b'M').arg(u32::from(mask))
}

/// `.`: read the ppm multiplier (` . 00100`).
pub fn multiplier_command() -> Command {
    Command::new(b'.')
}

/// `G`: zero-point calibration in fresh air (400 ppm by default; EEPROM
/// locations 10/11). Replies with the new zero point.
pub fn zero_fresh_air_command() -> Command {
    Command::new(b'G')
}

/// `U`: zero-point calibration in nitrogen (0 ppm).
pub fn zero_nitrogen_command() -> Command {
    Command::new(b'U')
}

/// `X n`: zero-point calibration in a known concentration given in sensor
/// units (ppm ÷ multiplier).
pub fn zero_known_gas_command(concentration: u32) -> Command {
    Command::new(b'X').arg(concentration)
}

/// `F r a`: fine-tune the zero point so that reading `reported` becomes
/// `actual` (sensor units).
pub fn fine_tune_command(reported: u32, actual: u32) -> Command {
    Command::new(b'F').arg(reported).arg(actual)
}

/// `@ i.i r.r` (intervals in tenths of days) or `@ 0` (disable).
/// Put the sensor in command mode first.
pub fn set_autocalibration_command(intervals: Option<(u16, u16)>) -> Command {
    match intervals {
        Some((initial, regular)) => Command::new(b'@').arg_tenths(initial).arg_tenths(regular),
        None => Command::new(b'@').arg(0),
    }
}

/// `@`: read the autocalibration settings.
pub fn read_autocalibration_command() -> Command {
    Command::new(b'@')
}

/// `Y`: firmware version and serial number (two lines; command mode only).
pub fn version_command() -> Command {
    Command::new(b'Y')
}

fn parse_u32(token: &[u8]) -> Option<u32> {
    if token.is_empty() || token.len() > 9 {
        return None;
    }
    token.iter().try_fold(0u32, |acc, b| {
        b.is_ascii_digit().then(|| acc * 10 + u32::from(b - b'0'))
    })
}

fn parse_tenths(token: &[u8]) -> Option<u16> {
    let (int, frac) = match token.iter().position(|b| *b == b'.') {
        Some(p) => (&token[..p], &token[p + 1..]),
        None => (token, &b"0"[..]),
    };
    if frac.len() != 1 {
        return None;
    }
    let v = parse_u32(int)? * 10 + parse_u32(frac)?;
    u16::try_from(v).ok()
}

/// One CO2 / temperature / humidity report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Measurement {
    /// Digitally filtered CO2 (`Z`) in ppm.
    pub co2_ppm: Option<u32>,
    /// Unfiltered CO2 (`z`) in ppm.
    pub co2_raw_ppm: Option<u32>,
    /// Temperature (`T`) in 0.01 °C; 0 °C (`T 01000`) if not fitted.
    pub temperature: Option<i32>,
    /// Humidity (`H`) in 0.01 %RH; 0 (`H 00000`) if not fitted.
    pub humidity: Option<u16>,
}

impl Readings for Measurement {
    /// Reports the filtered CO2 (or the unfiltered one if `Z` is not in the
    /// field mask), temperature and humidity when present.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if let Some(c) = self.co2_ppm.or(self.co2_raw_ppm) {
            f(Quantity::Co2(c));
        }
        if let Some(t) = self.temperature {
            f(Quantity::Temperature(t));
        }
        if let Some(h) = self.humidity {
            f(Quantity::Humidity(h));
        }
    }
}

/// One response line without `\r\n`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Line {
    buf: [u8; MAX_LINE_LEN],
    len: u8,
}

impl Line {
    /// Raw line (including the leading space).
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..usize::from(self.len)]
    }

    /// Space-separated tokens.
    pub fn tokens(&self) -> impl Iterator<Item = &[u8]> {
        self.as_bytes()
            .split(|b| *b == b' ')
            .filter(|t| !t.is_empty())
    }

    /// First non-space character (the echoed command letter / field id).
    pub fn id(&self) -> Option<u8> {
        self.as_bytes().iter().copied().find(|b| *b != b' ')
    }

    /// `i`-th decimal parameter after the id (e.g. the zero point in
    /// ` G 32950`).
    pub fn value(&self, i: usize) -> Option<u32> {
        parse_u32(self.tokens().nth(i + 1)?)
    }

    /// ` ?`: the previous command was not recognised.
    pub fn is_unknown_command(&self) -> bool {
        self.tokens().eq([&b"?"[..]])
    }

    /// Autocalibration reply: `Some(None)` for ` @ 0`, `Some(Some((initial,
    /// regular)))` in tenths of days for ` @ 1.0 8.0`.
    pub fn autocalibration(&self) -> Option<Option<(u16, u16)>> {
        let mut t = self.tokens();
        if t.next()? != b"@" {
            return None;
        }
        let first = t.next()?;
        match t.next() {
            None if first == b"0" => Some(None),
            Some(second) if t.next().is_none() => {
                Some(Some((parse_tenths(first)?, parse_tenths(second)?)))
            }
            _ => None,
        }
    }

    /// Decode a reading line (`Z`, `z`, `T`, `H` fields, other letter
    /// fields ignored) applying the ppm `multiplier`. `None` for replies
    /// without any of these fields or with malformed tokens.
    pub fn measurement(&self, multiplier: u32) -> Option<Measurement> {
        let mut m = Measurement::default();
        let mut any = false;
        let mut t = self.tokens();
        while let Some(id) = t.next() {
            let [id] = *id else { return None };
            let v = parse_u32(t.next()?)?;
            match id {
                b'Z' => m.co2_ppm = Some(v.saturating_mul(multiplier)),
                b'z' => m.co2_raw_ppm = Some(v.saturating_mul(multiplier)),
                b'T' => m.temperature = Some((v as i32 - 1000) * 10),
                b'H' => m.humidity = Some(v.saturating_mul(10).min(u32::from(u16::MAX)) as u16),
                c if c.is_ascii_alphabetic() => continue,
                _ => return None,
            }
            any = true;
        }
        any.then_some(m)
    }
}

/// Line assembler: collects bytes up to `\n`, drops `\r`, empty lines and
/// lines longer than [`MAX_LINE_LEN`].
#[derive(Debug, Clone)]
pub struct Parser {
    buf: [u8; MAX_LINE_LEN],
    len: usize,
    overflow: bool,
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    /// Empty parser.
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_LINE_LEN],
            len: 0,
            overflow: false,
        }
    }

    /// Drop any partial line.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Feed one received byte.
    pub fn feed(&mut self, byte: u8) -> Option<Line> {
        match byte {
            b'\r' => None,
            b'\n' => {
                let len = core::mem::take(&mut self.len);
                if core::mem::take(&mut self.overflow) || len == 0 {
                    return None;
                }
                Some(Line {
                    buf: self.buf,
                    len: len as u8,
                })
            }
            _ if self.len < MAX_LINE_LEN => {
                self.buf[self.len] = byte;
                self.len += 1;
                None
            }
            _ => {
                self.overflow = true;
                None
            }
        }
    }
}

fn reply<E>(found: Option<Line>) -> Result<Line, Error<E>> {
    match found {
        Some(l) if l.is_unknown_command() => Err(Error::Unsupported),
        Some(l) => Ok(l),
        None => Err(Error::NotReady),
    }
}

fn wants(l: &Line, id: u8) -> bool {
    l.id() == Some(id) || l.is_unknown_command()
}

macro_rules! constructors {
    () => {
        /// Create a driver assuming the factory-default streaming mode,
        /// multiplier 1 and filter 32; no bus traffic.
        pub const fn new(uart: U) -> Self {
            Self {
                uart,
                parser: Parser::new(),
                mode: Mode::Streaming,
                awake_mode: Mode::Streaming,
                multiplier: 1,
                filter: DEFAULT_FILTER,
            }
        }

        /// Assume the sensor is in `mode` (it powers up in the last used of
        /// streaming / polling); no bus traffic.
        pub const fn with_mode(mut self, mode: Mode) -> Self {
            self.mode = mode;
            if !matches!(mode, Mode::Command) {
                self.awake_mode = mode;
            }
            self
        }

        /// Use a known ppm multiplier (1 ambient, 10 or 100 wide-range).
        pub const fn with_multiplier(mut self, multiplier: u32) -> Self {
            self.multiplier = multiplier;
            self
        }

        /// Assume a digital filter setting (for the wake-up warm-up time).
        pub const fn with_filter(mut self, filter: u16) -> Self {
            self.filter = filter;
            self
        }

        /// Mode the driver assumes.
        pub const fn mode(&self) -> Mode {
            self.mode
        }

        /// ppm multiplier in use.
        pub const fn multiplier(&self) -> u32 {
            self.multiplier
        }

        /// Give the UART back.
        pub fn release(self) -> U {
            self.uart
        }
    };
}

/// Blocking driver.
pub struct ExplorIr<U> {
    uart: U,
    parser: Parser,
    mode: Mode,
    awake_mode: Mode,
    multiplier: u32,
    filter: u16,
}

impl<U> ExplorIr<U> {
    constructors!();
}

impl<U> ExplorIr<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    fn drain(&mut self) -> Result<(), Error<U::Error>> {
        let mut buf = [0u8; 16];
        let mut left = MAX_DRAIN_BYTES;
        while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
            let n = self.uart.read(&mut buf).map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            left = left.saturating_sub(n);
        }
        self.parser.reset();
        Ok(())
    }

    /// Feed all available bytes (bounded) to the parser and return the
    /// newest line accepted by `want`.
    pub fn poll_line(
        &mut self,
        want: impl Fn(&Line) -> bool,
    ) -> Result<Option<Line>, Error<U::Error>> {
        let mut latest = None;
        let mut buf = [0u8; 16];
        let mut left = MAX_READ_BYTES;
        while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
            let n_want = left.min(buf.len());
            let n = self.uart.read(&mut buf[..n_want]).map_err(Error::bus)?;
            if n == 0 {
                break;
            }
            left -= n;
            for b in &buf[..n] {
                if let Some(l) = self.parser.feed(*b) {
                    if want(&l) {
                        latest = Some(l);
                    }
                }
            }
        }
        Ok(latest)
    }

    /// Discard pending input and send `command`. Returns the time to wait
    /// before [`read_reply`](Self::read_reply).
    pub fn send_command(&mut self, command: &Command) -> Result<WaitUs, Error<U::Error>> {
        self.drain()?;
        self.uart
            .write_all(command.as_bytes())
            .map_err(Error::bus)?;
        self.uart.flush().map_err(Error::bus)?;
        Ok(RESPONSE_US)
    }

    /// Newest reply line starting with `id`; `Unsupported` if the sensor
    /// answered `?`, `NotReady` if nothing has arrived.
    pub fn read_reply(&mut self, id: u8) -> Result<Line, Error<U::Error>> {
        let found = self.poll_line(|l| wants(l, id))?;
        reply(found)
    }

    /// Switch the operating mode. Returns the warm-up time when leaving
    /// command mode, otherwise [`RESPONSE_US`].
    pub fn set_mode(&mut self, mode: Mode) -> Result<WaitUs, Error<U::Error>> {
        let wait = self.send_command(&mode_command(mode))?;
        let from_command = self.mode == Mode::Command;
        self.mode = mode;
        if mode == Mode::Command {
            return Ok(wait);
        }
        self.awake_mode = mode;
        Ok(if from_command {
            warm_up_us(self.filter)
        } else {
            wait
        })
    }

    /// Set the digital filter (stored in EEPROM).
    pub fn set_filter(&mut self, filter: u16) -> Result<WaitUs, Error<U::Error>> {
        let wait = self.send_command(&set_filter_command(filter))?;
        self.filter = filter;
        Ok(wait)
    }

    /// Request the ppm multiplier; then call
    /// [`read_multiplier`](Self::read_multiplier).
    pub fn request_multiplier(&mut self) -> Result<WaitUs, Error<U::Error>> {
        self.send_command(&multiplier_command())
    }

    /// Read the multiplier reply and use it for later readings.
    pub fn read_multiplier(&mut self) -> Result<u32, Error<U::Error>> {
        let m = self.read_reply(b'.')?.value(0).ok_or(Error::InvalidData)?;
        if m == 0 {
            return Err(Error::InvalidData);
        }
        self.multiplier = m;
        Ok(m)
    }
}

impl<U> zigbee_sensor::blocking::Sensor for ExplorIr<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type BusError = U::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
        match self.mode {
            Mode::Command => Err(Error::NotReady),
            Mode::Streaming => Ok(0),
            Mode::Polling => self.send_command(&poll_command()),
        }
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
        if self.mode == Mode::Command {
            return Err(Error::NotReady);
        }
        let mult = self.multiplier;
        self.poll_line(|l| l.measurement(mult).is_some())?
            .and_then(|l| l.measurement(mult))
            .ok_or(Error::NotReady)
    }
}

impl<U> zigbee_sensor::blocking::LowPower for ExplorIr<U>
where
    U: embedded_io::Read + embedded_io::ReadReady + embedded_io::Write,
{
    type BusError = U::Error;

    /// `K 0`: stop measuring (< 3.5 mW, configuration kept).
    fn sleep(&mut self) -> Result<(), Error<U::Error>> {
        self.set_mode(Mode::Command).map(|_| ())
    }

    /// Return to the previous streaming/polling mode; waits for the
    /// filter-dependent warm-up.
    fn wake(&mut self) -> Result<WaitUs, Error<U::Error>> {
        self.mode = Mode::Command;
        self.set_mode(self.awake_mode)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        Command, DEFAULT_FILTER, Line, MAX_DRAIN_BYTES, MAX_READ_BYTES, Measurement, Mode, Parser,
        RESPONSE_US, mode_command, multiplier_command, poll_command, reply, set_filter_command,
        wants, warm_up_us,
    };
    use embedded_io_async::{Read, ReadReady, Write};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::ExplorIr`].
    pub struct ExplorIr<U> {
        uart: U,
        parser: Parser,
        mode: Mode,
        awake_mode: Mode,
        multiplier: u32,
        filter: u16,
    }

    impl<U> ExplorIr<U> {
        constructors!();
    }

    impl<U: Read + ReadReady + Write> ExplorIr<U> {
        async fn drain(&mut self) -> Result<(), Error<U::Error>> {
            let mut buf = [0u8; 16];
            let mut left = MAX_DRAIN_BYTES;
            while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
                let n = self.uart.read(&mut buf).await.map_err(Error::bus)?;
                if n == 0 {
                    break;
                }
                left = left.saturating_sub(n);
            }
            self.parser.reset();
            Ok(())
        }

        /// See [`super::ExplorIr::poll_line`].
        pub async fn poll_line(
            &mut self,
            want: impl Fn(&Line) -> bool,
        ) -> Result<Option<Line>, Error<U::Error>> {
            let mut latest = None;
            let mut buf = [0u8; 16];
            let mut left = MAX_READ_BYTES;
            while left > 0 && self.uart.read_ready().map_err(Error::bus)? {
                let n_want = left.min(buf.len());
                let n = self
                    .uart
                    .read(&mut buf[..n_want])
                    .await
                    .map_err(Error::bus)?;
                if n == 0 {
                    break;
                }
                left -= n;
                for b in &buf[..n] {
                    if let Some(l) = self.parser.feed(*b) {
                        if want(&l) {
                            latest = Some(l);
                        }
                    }
                }
            }
            Ok(latest)
        }

        /// See [`super::ExplorIr::send_command`].
        pub async fn send_command(&mut self, command: &Command) -> Result<WaitUs, Error<U::Error>> {
            self.drain().await?;
            self.uart
                .write_all(command.as_bytes())
                .await
                .map_err(Error::bus)?;
            self.uart.flush().await.map_err(Error::bus)?;
            Ok(RESPONSE_US)
        }

        /// See [`super::ExplorIr::read_reply`].
        pub async fn read_reply(&mut self, id: u8) -> Result<Line, Error<U::Error>> {
            let found = self.poll_line(|l| wants(l, id)).await?;
            reply(found)
        }

        /// See [`super::ExplorIr::set_mode`].
        pub async fn set_mode(&mut self, mode: Mode) -> Result<WaitUs, Error<U::Error>> {
            let wait = self.send_command(&mode_command(mode)).await?;
            let from_command = self.mode == Mode::Command;
            self.mode = mode;
            if mode == Mode::Command {
                return Ok(wait);
            }
            self.awake_mode = mode;
            Ok(if from_command {
                warm_up_us(self.filter)
            } else {
                wait
            })
        }

        /// See [`super::ExplorIr::set_filter`].
        pub async fn set_filter(&mut self, filter: u16) -> Result<WaitUs, Error<U::Error>> {
            let wait = self.send_command(&set_filter_command(filter)).await?;
            self.filter = filter;
            Ok(wait)
        }

        /// See [`super::ExplorIr::request_multiplier`].
        pub async fn request_multiplier(&mut self) -> Result<WaitUs, Error<U::Error>> {
            self.send_command(&multiplier_command()).await
        }

        /// See [`super::ExplorIr::read_multiplier`].
        pub async fn read_multiplier(&mut self) -> Result<u32, Error<U::Error>> {
            let m = self
                .read_reply(b'.')
                .await?
                .value(0)
                .ok_or(Error::InvalidData)?;
            if m == 0 {
                return Err(Error::InvalidData);
            }
            self.multiplier = m;
            Ok(m)
        }
    }

    impl<U: Read + ReadReady + Write> zigbee_sensor::asynch::Sensor for ExplorIr<U> {
        type BusError = U::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<U::Error>> {
            match self.mode {
                Mode::Command => Err(Error::NotReady),
                Mode::Streaming => Ok(0),
                Mode::Polling => self.send_command(&poll_command()).await,
            }
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<U::Error>> {
            if self.mode == Mode::Command {
                return Err(Error::NotReady);
            }
            let mult = self.multiplier;
            self.poll_line(|l| l.measurement(mult).is_some())
                .await?
                .and_then(|l| l.measurement(mult))
                .ok_or(Error::NotReady)
        }
    }

    impl<U: Read + ReadReady + Write> zigbee_sensor::asynch::LowPower for ExplorIr<U> {
        type BusError = U::Error;

        async fn sleep(&mut self) -> Result<(), Error<U::Error>> {
            self.set_mode(Mode::Command).await.map(|_| ())
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<U::Error>> {
            self.mode = Mode::Command;
            self.set_mode(self.awake_mode).await
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
    use zigbee_sensor::blocking::{LowPower, Sensor};

    /// In-memory UART: `rx` is what the sensor sends, `tx` what we wrote.
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

    fn lines(bytes: &[u8]) -> Vec<Line> {
        let mut p = Parser::new();
        bytes.iter().filter_map(|b| p.feed(*b)).collect()
    }

    #[test]
    fn commands_match_manual() {
        assert_eq!(mode_command(Mode::Polling).as_bytes(), b"K 2\r\n");
        assert_eq!(set_filter_command(32).as_bytes(), b"A 32\r\n");
        assert_eq!(set_fields_command(4164).as_bytes(), b"M 4164\r\n");
        assert_eq!(zero_known_gas_command(2000).as_bytes(), b"X 2000\r\n");
        assert_eq!(fine_tune_command(410, 390).as_bytes(), b"F 410 390\r\n");
        assert_eq!(
            set_autocalibration_command(Some((10, 80))).as_bytes(),
            [
                0x40, 0x20, 0x31, 0x2E, 0x30, 0x20, 0x38, 0x2E, 0x30, 0x0D, 0x0A
            ]
        );
        assert_eq!(set_autocalibration_command(None).as_bytes(), b"@ 0\r\n");
        assert_eq!(multiplier_command().as_bytes(), b".\r\n");
        assert_eq!(
            fine_tune_command(u32::MAX, u32::MAX).as_bytes(),
            b"F 4294967295 4294967295\r\n"
        );
        assert_eq!(
            field::HUMIDITY | field::TEMPERATURE | field::CO2_FILTERED,
            4164
        );
        assert_eq!(warm_up_us(1), 1_200_000);
        assert_eq!(warm_up_us(4), 5_000_000);
        assert_eq!(warm_up_us(32), 32_000_000);
    }

    #[test]
    fn lines_decode() {
        let l = lines(b" H 00345 T 01195 Z 00651\r\n Z 00842 z 00765\r\n . 00100\r\n");
        assert_eq!(
            l[0].measurement(1),
            Some(Measurement {
                co2_ppm: Some(651),
                co2_raw_ppm: None,
                temperature: Some(1950),
                humidity: Some(3450),
            })
        );
        let m = l[1].measurement(10).unwrap();
        assert_eq!((m.co2_ppm, m.co2_raw_ppm), (Some(8420), Some(7650)));
        assert_eq!(l[2].measurement(1), None);
        assert_eq!((l[2].id(), l[2].value(0)), (Some(b'.'), Some(100)));

        for reply in [
            &b" K 00001"[..],
            b" G 32950",
            b" Y,Jan 30 2013,10:45:03,AL17",
            b" B 00233 00000",
            b" ?",
        ] {
            let mut bytes = reply.to_vec();
            bytes.extend(b"\r\n");
            assert_eq!(lines(&bytes)[0].measurement(1), None, "{reply:?}");
        }
        assert!(lines(b" ?\r\n")[0].is_unknown_command());
        assert_eq!(
            lines(b" @ 1.0 8.0\r\n")[0].autocalibration(),
            Some(Some((10, 80)))
        );
        assert_eq!(lines(b" @ 0\r\n")[0].autocalibration(), Some(None));

        let mut long = vec![b'Z'; MAX_LINE_LEN + 1];
        long.extend(b"\r\n\r\n Z 00500\r\n");
        let l = lines(&long);
        assert_eq!(l.len(), 1);
        assert_eq!(l[0].measurement(1).unwrap().co2_ppm, Some(500));
    }

    #[test]
    fn streaming_returns_newest_reading() {
        let mut uart = FakeUart::default();
        let mut co2 = ExplorIr::new(&mut uart);
        assert_eq!(co2.start_measurement().unwrap(), 0);
        assert_eq!(co2.read_measurement(), Err(Error::NotReady));
        co2.uart
            .rx
            .extend(b" Z 00800 z 00810\r\n Z 00801 z 00790\r\n Z 008");
        let m = co2.read_measurement().unwrap();
        assert_eq!(m.co2_ppm, Some(801));
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(seen, vec![Quantity::Co2(801)]);
        co2.uart.rx.extend(b"02 z 00799\r\n");
        assert_eq!(co2.read_measurement().unwrap().co2_ppm, Some(802));
        assert!(uart.tx.is_empty());
    }

    #[test]
    fn polling_with_multiplier() {
        let mut uart = FakeUart {
            on_write: b" . 00010\r\n".to_vec(),
            ..FakeUart::default()
        };
        let mut co2 = ExplorIr::new(&mut uart).with_mode(Mode::Polling);
        co2.request_multiplier().unwrap();
        assert_eq!(co2.read_multiplier().unwrap(), 10);
        co2.uart.on_write = b" Z 01200 z 01190\r\n".to_vec();
        assert_eq!(co2.start_measurement().unwrap(), RESPONSE_US);
        assert_eq!(co2.read_measurement().unwrap().co2_ppm, Some(12000));
        co2.uart.on_write = b" ?\r\n".to_vec();
        co2.send_command(&Command::new(b'x')).unwrap();
        assert_eq!(co2.read_reply(b'x'), Err(Error::Unsupported));
        assert_eq!(uart.tx, b".\r\nQ\r\nx\r\n");
    }

    #[test]
    fn sleep_and_wake() {
        let mut uart = FakeUart::default();
        let mut co2 = ExplorIr::new(&mut uart)
            .with_mode(Mode::Polling)
            .with_filter(8);
        co2.uart.on_write = b" K 00000\r\n".to_vec();
        co2.sleep().unwrap();
        assert_eq!(co2.mode(), Mode::Command);
        assert_eq!(co2.start_measurement(), Err(Error::NotReady));
        assert_eq!(co2.read_reply(b'K').unwrap().value(0), Some(0));
        assert_eq!(co2.wake().unwrap(), 9_000_000);
        assert_eq!(co2.mode(), Mode::Polling);
        assert_eq!(uart.tx, b"K 0\r\nK 2\r\n");
    }

    #[test]
    fn async_polling_and_sleep() {
        use zigbee_sensor::asynch::{LowPower as _, Sensor as _};

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let mut uart = FakeUart {
            on_write: b" H 00551 T 01235 Z 00631\r\n".to_vec(),
            ..FakeUart::default()
        };
        let mut co2 = asynch::ExplorIr::new(&mut uart).with_mode(Mode::Polling);
        let mut delay = RecordingDelay(0);
        let m = block_on(co2.measure(&mut delay)).unwrap();
        assert_eq!(
            (m.co2_ppm, m.temperature, m.humidity),
            (Some(631), Some(2350), Some(5510))
        );
        assert_eq!(delay.0, RESPONSE_US);
        block_on(co2.sleep()).unwrap();
        assert_eq!(block_on(co2.wake()).unwrap(), warm_up_us(DEFAULT_FILTER));
        assert_eq!(uart.tx, b"Q\r\nK 0\r\nK 2\r\n");
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
