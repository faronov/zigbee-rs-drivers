//! Analog Devices ADE7953 single-phase, two-current-channel energy
//! metering IC over SPI or I2C.
//!
//! The ADE7953 measures continuously; there is no triggered mode and no
//! power-down mode, so `start_measurement` returns 0 and there is no
//! `LowPower` implementation. One current channel ([`Channel::A`], shunt,
//! PGA up to 22, or [`Channel::B`], CT / Rogowski front end) is reported;
//! it can be switched at run time and keeps its own energy total.
//!
//! Required set-up (datasheet "Required Register Setting"): after every
//! power-up or reset write 0xAD to 0x0FE and then 0x0030 to the protected
//! register 0x120. [`Ade7953::init`] does that, locks the communication
//! interface (CONFIG.COMM_LOCK = 0), writes the PGA gains and optional gain
//! trims and clears the reset interrupt. Wait at least
//! [`RESET_WAIT_US`] after power-up or [`Ade7953::reset`] before `init`
//! (or wait for the IRQ pin). [`Ade7953::reset_occurred`] detects an
//! unexpected chip reset so `init` can be repeated.
//!
//! Register width follows the address: 0x0xx (and 0x702, 0x800) 8 bit,
//! 0x1xx 16 bit, 0x2xx 24 bit, 0x3xx 32 bit (sign-extended 24-bit data).
//! The driver uses the 24-bit aliases.
//!
//! Scaling ([`Calibration`]) is configuration: per-LSB weights of VRMS,
//! IRMS, active power and active energy, derived by
//! [`Calibration::from_front_end`] from the voltage divider, the shunt (or
//! CT burden / turns) and the PGA gains using the datasheet full-scale
//! codes, or set directly from a calibration against a reference meter.
//!
//! Reference: Analog Devices ADE7953 data sheet Rev. C.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address (0111000x, fixed).
pub const I2C_ADDRESS: u8 = 0x38;

/// Register addresses (24-bit aliases for 24/32-bit registers).
pub mod reg {
    /// SAGCYC (8 bit).
    pub const SAGCYC: u16 = 0x000;
    /// DISNOLOAD (8 bit).
    pub const DISNOLOAD: u16 = 0x001;
    /// LCYCMODE (8 bit, default 0x40 = RSTREAD).
    pub const LCYCMODE: u16 = 0x004;
    /// PGA_V (8 bit).
    pub const PGA_V: u16 = 0x007;
    /// PGA_IA (8 bit).
    pub const PGA_IA: u16 = 0x008;
    /// PGA_IB (8 bit).
    pub const PGA_IB: u16 = 0x009;
    /// WRITE_PROTECT (8 bit).
    pub const WRITE_PROTECT: u16 = 0x040;
    /// Unlock key register for 0x120 (8 bit).
    pub const UNLOCK: u16 = 0x0FE;
    /// Silicon version (8 bit).
    pub const VERSION: u16 = 0x702;
    /// EX_REF (8 bit).
    pub const EX_REF: u16 = 0x800;
    /// LINECYC (16 bit).
    pub const LINECYC: u16 = 0x101;
    /// CONFIG (16 bit, default 0x8004).
    pub const CONFIG: u16 = 0x102;
    /// CFMODE (16 bit).
    pub const CFMODE: u16 = 0x107;
    /// PFA power factor (16 bit, signed, 2^-15).
    pub const PFA: u16 = 0x10A;
    /// PFB power factor (16 bit, signed, 2^-15).
    pub const PFB: u16 = 0x10B;
    /// PERIOD (16 bit, 4.47 µs / LSB).
    pub const PERIOD: u16 = 0x10E;
    /// Protected timing register (16 bit), must be 0x0030.
    pub const OPTIMUM: u16 = 0x120;
    /// ACCMODE (24 bit).
    pub const ACCMODE: u16 = 0x201;
    /// AP_NOLOAD (24 bit).
    pub const AP_NOLOAD: u16 = 0x203;
    /// AWATT (24 bit, signed).
    pub const AWATT: u16 = 0x212;
    /// BWATT (24 bit, signed).
    pub const BWATT: u16 = 0x213;
    /// IRMSA (24 bit).
    pub const IRMSA: u16 = 0x21A;
    /// IRMSB (24 bit).
    pub const IRMSB: u16 = 0x21B;
    /// VRMS (24 bit).
    pub const VRMS: u16 = 0x21C;
    /// AENERGYA (24 bit, signed, read-with-reset).
    pub const AENERGYA: u16 = 0x21E;
    /// AENERGYB (24 bit, signed, read-with-reset).
    pub const AENERGYB: u16 = 0x21F;
    /// IRQENA (24 bit).
    pub const IRQENA: u16 = 0x22C;
    /// IRQSTATA (24 bit).
    pub const IRQSTATA: u16 = 0x22D;
    /// RSTIRQSTATA (24 bit, read clears IRQSTATA).
    pub const RSTIRQSTATA: u16 = 0x22E;
    /// AIGAIN (24 bit, 0x400000 = 1).
    pub const AIGAIN: u16 = 0x280;
    /// AVGAIN (24 bit, 0x400000 = 1).
    pub const AVGAIN: u16 = 0x281;
    /// AWGAIN (24 bit, 0x400000 = 1).
    pub const AWGAIN: u16 = 0x282;
    /// AIRMSOS (24 bit, signed).
    pub const AIRMSOS: u16 = 0x286;
    /// VRMSOS (24 bit, signed).
    pub const VRMSOS: u16 = 0x288;
    /// AWATTOS (24 bit, signed).
    pub const AWATTOS: u16 = 0x289;
    /// BIGAIN (24 bit, 0x400000 = 1).
    pub const BIGAIN: u16 = 0x28C;
    /// BWGAIN (24 bit, 0x400000 = 1).
    pub const BWGAIN: u16 = 0x28E;
    /// BIRMSOS (24 bit, signed).
    pub const BIRMSOS: u16 = 0x292;
    /// BWATTOS (24 bit, signed).
    pub const BWATTOS: u16 = 0x295;
}

/// Register width in bytes for an address (see the crate docs).
pub const fn register_width(addr: u16) -> usize {
    match addr >> 8 {
        1 => 2,
        2 => 3,
        3 => 4,
        _ => 1,
    }
}

/// CONFIG written by `init`: HPFEN = 1, COMM_LOCK = 0 (lock interface),
/// integrators off, PF from instantaneous powers.
pub const CONFIG_INIT: u16 = 1 << 2;
/// CONFIG reset value.
pub const CONFIG_DEFAULT: u16 = 0x8004;
const CONFIG_SWRST: u16 = 1 << 7;
/// Key written to [`reg::UNLOCK`].
pub const UNLOCK_KEY: u8 = 0xAD;
/// Value of [`reg::OPTIMUM`].
pub const OPTIMUM_VALUE: u16 = 0x0030;
/// LCYCMODE written by `init`: RSTREAD (energy registers clear on read),
/// no line-cycle accumulation.
pub const LCYCMODE_INIT: u8 = 1 << 6;
/// IRQSTATA reset flag (end of power-up, hardware or software reset).
pub const IRQ_RESET: u32 = 1 << 20;
/// Unity xIGAIN / xVGAIN / xWGAIN.
pub const UNITY_GAIN: u32 = 0x40_0000;

/// Start-up time after power-up or reset when the IRQ pin is not used
/// (datasheet: at least 100 ms).
pub const RESET_WAIT_US: WaitUs = 100_000;

/// IRMS / VRMS code for a full-scale input.
pub const FULL_SCALE_RMS: u32 = 9_032_007;
/// AWATT code for full-scale voltage and current inputs.
pub const FULL_SCALE_WATT: u32 = 4_862_401;
/// AENERGY accumulation rate with full-scale inputs and unity WGAIN (Hz).
pub const FULL_SCALE_ENERGY_HZ: u32 = 206_900;
/// Full-scale differential pin input RMS (500 mV peak / √2) in nV.
pub const PIN_FULL_SCALE_RMS_NV: u64 = 353_553_391;
/// PERIOD register clock (Hz).
pub const PERIOD_CLOCK_HZ: u32 = 223_750;

/// Register access over SPI or I2C.
pub trait Interface {
    /// Transport error.
    type Error;
    /// Read `buf.len()` bytes (MSB first) from a register.
    fn read_reg(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write `data` (MSB first, 1-4 bytes) to a register.
    fn write_reg(&mut self, addr: u16, data: &[u8]) -> Result<(), Self::Error>;
}

/// I2C transport (SCLK pin high, CS high), up to 400 kHz.
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    /// The bus.
    pub i2c: I2C,
}

/// SPI transport, up to 5 MHz, data sampled on the rising SCLK edge. Frame:
/// address high, address low, `0x80` (read) / `0x00` (write), data.
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    /// The device (bus + chip select).
    pub spi: SPI,
}

const SPI_READ: u8 = 0x80;
const SPI_WRITE: u8 = 0x00;

/// Build `prefix ++ data` into a buffer; returns the used length.
fn frame(prefix: &[u8], data: &[u8]) -> ([u8; 7], usize) {
    let mut buf = [0; 7];
    let n = prefix.len() + data.len().min(4);
    buf[..prefix.len()].copy_from_slice(prefix);
    buf[prefix.len()..n].copy_from_slice(&data[..n - prefix.len()]);
    (buf, n)
}

impl<I2C: embedded_hal::i2c::I2c> Interface for I2cInterface<I2C> {
    type Error = I2C::Error;

    fn read_reg(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(I2C_ADDRESS, &addr.to_be_bytes(), buf)
    }

    fn write_reg(&mut self, addr: u16, data: &[u8]) -> Result<(), Self::Error> {
        let (buf, n) = frame(&addr.to_be_bytes(), data);
        self.i2c.write(I2C_ADDRESS, &buf[..n])
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn read_reg(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        let [hi, lo] = addr.to_be_bytes();
        self.spi
            .transaction(&mut [Operation::Write(&[hi, lo, SPI_READ]), Operation::Read(buf)])
    }

    fn write_reg(&mut self, addr: u16, data: &[u8]) -> Result<(), Self::Error> {
        let [hi, lo] = addr.to_be_bytes();
        let (buf, n) = frame(&[hi, lo, SPI_WRITE], data);
        self.spi.write(&buf[..n])
    }
}

/// PGA gain (PGA_V / PGA_IA / PGA_IB).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gain {
    /// ×1, ±500 mV (Channel A pins limited to ±250 mV).
    #[default]
    X1,
    /// ×2, ±250 mV.
    X2,
    /// ×4, ±125 mV.
    X4,
    /// ×8, ±62.5 mV.
    X8,
    /// ×16, ±31.25 mV.
    X16,
    /// ×22, ±22.7 mV (Current Channel A only).
    X22,
}

impl Gain {
    /// Register code.
    pub const fn bits(self) -> u8 {
        self as u8
    }

    /// Gain factor.
    pub const fn factor(self) -> u32 {
        match self {
            Gain::X1 => 1,
            Gain::X2 => 2,
            Gain::X4 => 4,
            Gain::X8 => 8,
            Gain::X16 => 16,
            Gain::X22 => 22,
        }
    }
}

/// Reported current channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Channel {
    /// Current Channel A (IAP/IAN).
    #[default]
    A,
    /// Current Channel B (IBP/IBN).
    B,
}

impl Channel {
    const fn index(self) -> usize {
        self as usize
    }

    /// IRMSx, xWATT, AENERGYx, PFx.
    const fn registers(self) -> [u16; 4] {
        match self {
            Channel::A => [reg::IRMSA, reg::AWATT, reg::AENERGYA, reg::PFA],
            Channel::B => [reg::IRMSB, reg::BWATT, reg::AENERGYB, reg::PFB],
        }
    }
}

/// PGA configuration written by `init`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Voltage channel gain (×1..×16).
    pub pga_v: Gain,
    /// Current Channel A gain (×1..×22).
    pub pga_ia: Gain,
    /// Current Channel B gain (×1..×16).
    pub pga_ib: Gain,
}

impl Config {
    /// `false` when a gain is not available on its channel.
    pub const fn is_valid(&self) -> bool {
        !matches!(self.pga_v, Gain::X22) && !matches!(self.pga_ib, Gain::X22)
    }
}

/// Gain calibration registers (unsigned, [`UNITY_GAIN`] = 1.0, range
/// 0..2.0). BVGAIN must not be modified, so it is not here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GainTrim {
    /// AIGAIN.
    pub aigain: u32,
    /// AVGAIN (affects both channels).
    pub avgain: u32,
    /// AWGAIN.
    pub awgain: u32,
    /// BIGAIN.
    pub bigain: u32,
    /// BWGAIN.
    pub bwgain: u32,
}

impl Default for GainTrim {
    fn default() -> Self {
        Self {
            aigain: UNITY_GAIN,
            avgain: UNITY_GAIN,
            awgain: UNITY_GAIN,
            bigain: UNITY_GAIN,
            bwgain: UNITY_GAIN,
        }
    }
}

impl GainTrim {
    const fn writes(&self) -> [(u16, u32); 5] {
        [
            (reg::AIGAIN, self.aigain),
            (reg::AVGAIN, self.avgain),
            (reg::AWGAIN, self.awgain),
            (reg::BIGAIN, self.bigain),
            (reg::BWGAIN, self.bwgain),
        ]
    }
}

/// Per-LSB weights of the selected current channel.
///
/// Ranges: VRMS / IRMS codes ≤ 2^24 and the weights are `u32`, so the
/// products fit `u64`; results saturate to `i32`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// Line voltage per VRMS LSB (nV).
    pub voltage_nv: u32,
    /// Line current per IRMS LSB (pA).
    pub current_pa: u32,
    /// Active power per xWATT LSB (nW).
    pub power_nw: u32,
    /// Active energy per AENERGYx LSB (nWh).
    pub energy_nwh: u32,
}

impl Calibration {
    /// Derive the weights from the analog front end:
    ///
    /// ```text
    /// Vfs_pin  = 353.553 mV / gain              (500 mV peak sine)
    /// V/LSB    = Vfs_pin · (r_top + r_bottom) / r_bottom / 9 032 007
    /// I/LSB    = Vfs_pin_i / shunt / 9 032 007
    /// P/LSB    = Vfs_line · Ifs_line / 4 862 401
    /// E/LSB    = Vfs_line · Ifs_line / 206.9 kHz       (WGAIN = 1.0)
    /// ```
    ///
    /// `shunt_micro_ohm` is the shunt, or for a current transformer the
    /// burden resistance divided by the turns ratio. The full-scale codes
    /// and the energy rate are typical datasheet values; trim with
    /// [`GainTrim`] or adjust the weights after calibration. `None` on a
    /// zero argument or when a weight does not fit `u32`.
    pub const fn from_front_end(
        r_top_ohm: u32,
        r_bottom_ohm: u32,
        shunt_micro_ohm: u32,
        voltage_gain: Gain,
        current_gain: Gain,
    ) -> Option<Self> {
        if r_bottom_ohm == 0 || shunt_micro_ohm == 0 {
            return None;
        }
        let vfs = PIN_FULL_SCALE_RMS_NV as u128;
        let ratio = r_top_ohm as u128 + r_bottom_ohm as u128;
        let rb = r_bottom_ohm as u128;
        let gv = voltage_gain.factor() as u128;
        let gi = current_gain.factor() as u128;
        let sh = shunt_micro_ohm as u128;
        let rms = FULL_SCALE_RMS as u128;
        // nV / µΩ = mA, hence the 10^9 for pA.
        let voltage = div_round(vfs * ratio, rb * gv * rms);
        let current = div_round(vfs * 1_000_000_000, sh * gi * rms);
        let p_num = vfs * ratio * vfs;
        let p_den = rb * gv * gi * sh * 1_000;
        let power = div_round(p_num, p_den * FULL_SCALE_WATT as u128);
        let energy = div_round(p_num, p_den * FULL_SCALE_ENERGY_HZ as u128 * 3_600);
        if voltage > u32::MAX as u128
            || current > u32::MAX as u128
            || power > u32::MAX as u128
            || energy > u32::MAX as u128
        {
            return None;
        }
        Some(Self {
            voltage_nv: voltage as u32,
            current_pa: current as u32,
            power_nw: power as u32,
            energy_nwh: energy as u32,
        })
    }

    /// VRMS code → mV.
    pub const fn voltage_mv(&self, raw: u32) -> i32 {
        sat_i32(div_round_u64(
            raw as u64 * self.voltage_nv as u64,
            1_000_000,
        ))
    }

    /// IRMS code → µA.
    pub const fn current_ua(&self, raw: u32) -> i32 {
        sat_i32(div_round_u64(
            raw as u64 * self.current_pa as u64,
            1_000_000,
        ))
    }

    /// xWATT code (signed) → mW.
    pub const fn power_mw(&self, raw: i32) -> i32 {
        let mw = div_round_u64(raw.unsigned_abs() as u64 * self.power_nw as u64, 1_000_000);
        let mw = sat_i32(mw);
        if raw < 0 { -mw } else { mw }
    }

    /// AENERGYx code (signed) → nWh.
    pub const fn energy_nwh(&self, raw: i32) -> i64 {
        raw as i64 * self.energy_nwh as i64
    }
}

const fn div_round(n: u128, d: u128) -> u128 {
    (n + d / 2) / d
}

const fn div_round_u64(n: u64, d: u64) -> u64 {
    (n + d / 2) / d
}

const fn sat_i32(v: u64) -> i32 {
    if v > i32::MAX as u64 {
        i32::MAX
    } else {
        v as i32
    }
}

/// Sign-extend a 24-bit register value.
pub const fn sign24(raw: u32) -> i32 {
    ((raw << 8) as i32) >> 8
}

/// PFx (2^-15 / LSB) → 0.001 units.
pub const fn power_factor_milli(raw: i16) -> i16 {
    ((raw as i32 * 1_000 + 16_384) >> 15) as i16
}

/// PERIOD → line frequency in mHz: `223.75 kHz / (PERIOD + 1)`. `None`
/// for 0 / 0xFFFF (no line or not settled).
pub const fn frequency_mhz(period: u16) -> Option<u32> {
    match period {
        0 | 0xFFFF => None,
        p => {
            let d = p as u32 + 1;
            Some((PERIOD_CLOCK_HZ * 1_000 + d / 2) / d)
        }
    }
}

/// One measurement of the selected channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// RMS voltage in mV.
    pub voltage: i32,
    /// RMS current in µA.
    pub current: i32,
    /// Active power in mW (negative = export).
    pub power: i32,
    /// Accumulated net active energy in mWh (clamped at 0).
    pub energy: u64,
    /// Power factor in 0.001 (sign from reactive power: negative =
    /// capacitive).
    pub power_factor: i16,
    /// Line frequency in mHz.
    pub frequency: Option<u32>,
}

impl Readings for Measurement {
    /// Voltage, Current, Power, Energy, PowerFactor, then Frequency when
    /// available.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Voltage(self.voltage));
        f(Quantity::Current(self.current));
        f(Quantity::Power(self.power));
        f(Quantity::Energy(self.energy));
        f(Quantity::PowerFactor(self.power_factor));
        if let Some(hz) = self.frequency {
            f(Quantity::Frequency(hz));
        }
    }
}

/// Raw register values of one channel.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RawResults {
    /// VRMS.
    pub vrms: u32,
    /// IRMSx.
    pub irms: u32,
    /// xWATT (24-bit raw).
    pub watt: u32,
    /// PFx.
    pub pf: u16,
    /// PERIOD.
    pub period: u16,
}

/// Decode raw registers; `energy_total_nwh` is the channel's accumulated
/// energy.
pub const fn decode(cal: &Calibration, raw: &RawResults, energy_total_nwh: i64) -> Measurement {
    let energy = if energy_total_nwh > 0 {
        energy_total_nwh as u64 / 1_000_000
    } else {
        0
    };
    Measurement {
        voltage: cal.voltage_mv(raw.vrms),
        current: cal.current_ua(raw.irms),
        power: cal.power_mw(sign24(raw.watt)),
        energy,
        power_factor: power_factor_milli(raw.pf as i16),
        frequency: frequency_mhz(raw.period),
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Create a driver over any transport; no bus traffic.
            pub const fn new(iface: IFACE, calibration: Calibration) -> Self {
                Self {
                    iface,
                    calibration,
                    config: Config {
                        pga_v: Gain::X1,
                        pga_ia: Gain::X1,
                        pga_ib: Gain::X1,
                    },
                    trim: None,
                    channel: Channel::A,
                    energy_nwh: [0; 2],
                }
            }

            /// PGA gains written by `init` (must match the calibration).
            pub const fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self
            }

            /// Gain calibration registers written by `init` (default: not
            /// written, chip reset values = unity).
            pub const fn with_gain_trim(mut self, trim: GainTrim) -> Self {
                self.trim = Some(trim);
                self
            }

            /// Reported current channel (default A).
            pub const fn with_channel(mut self, channel: Channel) -> Self {
                self.channel = channel;
                self
            }

            /// Switch the reported channel; calibration must be updated
            /// with [`Self::set_calibration`] when the front ends differ.
            pub fn set_channel(&mut self, channel: Channel) {
                self.channel = channel;
            }

            /// Reported channel.
            pub const fn channel(&self) -> Channel {
                self.channel
            }

            /// Replace the calibration.
            pub fn set_calibration(&mut self, calibration: Calibration) {
                self.calibration = calibration;
            }

            /// Active calibration.
            pub const fn calibration(&self) -> &Calibration {
                &self.calibration
            }

            /// Net accumulated active energy of the reported channel (nWh,
            /// may be negative).
            pub const fn energy_nwh(&self) -> i64 {
                self.energy_nwh[self.channel.index()]
            }

            /// Set the reported channel's energy total (e.g. restored from
            /// NV storage) in mWh.
            pub fn set_energy_mwh(&mut self, mwh: u64) {
                let nwh = mwh.saturating_mul(1_000_000).min(i64::MAX as u64);
                self.energy_nwh[self.channel.index()] = nwh as i64;
            }

            /// Give the transport back.
            pub fn release(self) -> IFACE {
                self.iface
            }

            fn add_energy(&mut self, raw: u32) {
                let delta = self.calibration.energy_nwh(sign24(raw));
                let total = &mut self.energy_nwh[self.channel.index()];
                *total = total.saturating_add(delta);
            }
        }

        impl<I2C> $name<I2cInterface<I2C>> {
            /// ADE7953 on an I2C bus (address 0x38).
            pub const fn new_i2c(i2c: I2C, calibration: Calibration) -> Self {
                Self::new(I2cInterface { i2c }, calibration)
            }
        }

        impl<SPI> $name<SpiInterface<SPI>> {
            /// ADE7953 on an SPI device.
            pub const fn new_spi(spi: SPI, calibration: Calibration) -> Self {
                Self::new(SpiInterface { spi }, calibration)
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Ade7953<IFACE> {
    iface: IFACE,
    calibration: Calibration,
    config: Config,
    trim: Option<GainTrim>,
    channel: Channel,
    energy_nwh: [i64; 2],
}

constructors!(Ade7953);

impl<IFACE: Interface> Ade7953<IFACE> {
    /// Read a register (width from the address, zero-extended).
    pub fn read_register(&mut self, addr: u16) -> Result<u32, Error<IFACE::Error>> {
        let mut buf = [0; 4];
        let w = register_width(addr);
        self.iface
            .read_reg(addr, &mut buf[4 - w..])
            .map_err(Error::bus)?;
        Ok(u32::from_be_bytes(buf))
    }

    /// Write a register (low `width` bytes of `value`).
    pub fn write_register(&mut self, addr: u16, value: u32) -> Result<(), Error<IFACE::Error>> {
        let w = register_width(addr);
        self.iface
            .write_reg(addr, &value.to_be_bytes()[4 - w..])
            .map_err(Error::bus)
    }

    /// Silicon version (0x702).
    pub fn version(&mut self) -> Result<u8, Error<IFACE::Error>> {
        Ok(self.read_register(reg::VERSION)? as u8)
    }

    /// Lock the interface, apply the 0x120 setting, write LCYCMODE, gains
    /// and trims and clear the reset flag. [`Error::InvalidArgument`] for a ×22
    /// gain on the voltage channel or Channel B.
    pub fn init(&mut self) -> Result<(), Error<IFACE::Error>> {
        if !self.config.is_valid() {
            return Err(Error::InvalidArgument);
        }
        self.write_register(reg::CONFIG, CONFIG_INIT as u32)?;
        self.write_register(reg::UNLOCK, UNLOCK_KEY as u32)?;
        self.write_register(reg::OPTIMUM, OPTIMUM_VALUE as u32)?;
        self.write_register(reg::LCYCMODE, LCYCMODE_INIT as u32)?;
        self.write_register(reg::PGA_V, self.config.pga_v.bits() as u32)?;
        self.write_register(reg::PGA_IA, self.config.pga_ia.bits() as u32)?;
        self.write_register(reg::PGA_IB, self.config.pga_ib.bits() as u32)?;
        if let Some(trim) = self.trim {
            for (addr, value) in trim.writes() {
                self.write_register(addr, value)?;
            }
        }
        self.read_register(reg::RSTIRQSTATA)?;
        Ok(())
    }

    /// Software reset (CONFIG.SWRST); wait [`RESET_WAIT_US`] and call
    /// `init`. Energy totals kept by the driver are not touched.
    pub fn reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.write_register(reg::CONFIG, (CONFIG_DEFAULT | CONFIG_SWRST) as u32)?;
        Ok(RESET_WAIT_US)
    }

    /// Read (and clear) the interrupt status; `true` when the chip has been
    /// reset since `init`, which then has to be repeated.
    pub fn reset_occurred(&mut self) -> Result<bool, Error<IFACE::Error>> {
        Ok(self.read_register(reg::RSTIRQSTATA)? & IRQ_RESET != 0)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Ade7953<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    /// Continuous device: returns 0 (RMS values settle ~200 ms after
    /// power-up, PERIOD after 30-40 ms).
    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        Ok(0)
    }

    /// Reads VRMS, IRMSx, xWATT, PFx, PERIOD and finally AENERGYx
    /// (read-with-reset, added to the channel total).
    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        let [irms, watt, energy, pf] = self.channel.registers();
        let raw = RawResults {
            vrms: self.read_register(reg::VRMS)?,
            irms: self.read_register(irms)?,
            watt: self.read_register(watt)?,
            pf: self.read_register(pf)? as u16,
            period: self.read_register(reg::PERIOD)? as u16,
        };
        let e = self.read_register(energy)?;
        self.add_energy(e);
        Ok(decode(&self.calibration, &raw, self.energy_nwh()))
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CONFIG_DEFAULT, CONFIG_INIT, CONFIG_SWRST, Calibration, Channel, Config, Gain, GainTrim,
        I2C_ADDRESS, I2cInterface, IRQ_RESET, LCYCMODE_INIT, Measurement, OPTIMUM_VALUE,
        RESET_WAIT_US, RawResults, SPI_READ, SPI_WRITE, SpiInterface, UNLOCK_KEY, decode, frame,
        reg, register_width, sign24,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async register access.
    #[allow(async_fn_in_trait)]
    pub trait Interface {
        /// Transport error.
        type Error;
        /// Read `buf.len()` bytes from a register.
        async fn read_reg(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), Self::Error>;
        /// Write 1-4 bytes to a register.
        async fn write_reg(&mut self, addr: u16, data: &[u8]) -> Result<(), Self::Error>;
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Interface for I2cInterface<I2C> {
        type Error = I2C::Error;

        async fn read_reg(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), Self::Error> {
            self.i2c
                .write_read(I2C_ADDRESS, &addr.to_be_bytes(), buf)
                .await
        }

        async fn write_reg(&mut self, addr: u16, data: &[u8]) -> Result<(), Self::Error> {
            let (buf, n) = frame(&addr.to_be_bytes(), data);
            self.i2c.write(I2C_ADDRESS, &buf[..n]).await
        }
    }

    impl<SPI: embedded_hal_async::spi::SpiDevice> Interface for SpiInterface<SPI> {
        type Error = SPI::Error;

        async fn read_reg(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), Self::Error> {
            use embedded_hal_async::spi::Operation;
            let [hi, lo] = addr.to_be_bytes();
            self.spi
                .transaction(&mut [Operation::Write(&[hi, lo, SPI_READ]), Operation::Read(buf)])
                .await
        }

        async fn write_reg(&mut self, addr: u16, data: &[u8]) -> Result<(), Self::Error> {
            let [hi, lo] = addr.to_be_bytes();
            let (buf, n) = frame(&[hi, lo, SPI_WRITE], data);
            self.spi.write(&buf[..n]).await
        }
    }

    /// Async twin of [`super::Ade7953`].
    #[derive(Debug)]
    pub struct Ade7953<IFACE> {
        iface: IFACE,
        calibration: Calibration,
        config: Config,
        trim: Option<GainTrim>,
        channel: Channel,
        energy_nwh: [i64; 2],
    }

    constructors!(Ade7953);

    impl<IFACE: Interface> Ade7953<IFACE> {
        /// See [`super::Ade7953::read_register`].
        pub async fn read_register(&mut self, addr: u16) -> Result<u32, Error<IFACE::Error>> {
            let mut buf = [0; 4];
            let w = register_width(addr);
            self.iface
                .read_reg(addr, &mut buf[4 - w..])
                .await
                .map_err(Error::bus)?;
            Ok(u32::from_be_bytes(buf))
        }

        /// See [`super::Ade7953::write_register`].
        pub async fn write_register(
            &mut self,
            addr: u16,
            value: u32,
        ) -> Result<(), Error<IFACE::Error>> {
            let w = register_width(addr);
            self.iface
                .write_reg(addr, &value.to_be_bytes()[4 - w..])
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Ade7953::version`].
        pub async fn version(&mut self) -> Result<u8, Error<IFACE::Error>> {
            Ok(self.read_register(reg::VERSION).await? as u8)
        }

        /// See [`super::Ade7953::init`].
        pub async fn init(&mut self) -> Result<(), Error<IFACE::Error>> {
            if !self.config.is_valid() {
                return Err(Error::InvalidArgument);
            }
            self.write_register(reg::CONFIG, CONFIG_INIT as u32).await?;
            self.write_register(reg::UNLOCK, UNLOCK_KEY as u32).await?;
            self.write_register(reg::OPTIMUM, OPTIMUM_VALUE as u32)
                .await?;
            self.write_register(reg::LCYCMODE, LCYCMODE_INIT as u32)
                .await?;
            self.write_register(reg::PGA_V, self.config.pga_v.bits() as u32)
                .await?;
            self.write_register(reg::PGA_IA, self.config.pga_ia.bits() as u32)
                .await?;
            self.write_register(reg::PGA_IB, self.config.pga_ib.bits() as u32)
                .await?;
            if let Some(trim) = self.trim {
                for (addr, value) in trim.writes() {
                    self.write_register(addr, value).await?;
                }
            }
            self.read_register(reg::RSTIRQSTATA).await?;
            Ok(())
        }

        /// See [`super::Ade7953::reset`].
        pub async fn reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.write_register(reg::CONFIG, (CONFIG_DEFAULT | CONFIG_SWRST) as u32)
                .await?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Ade7953::reset_occurred`].
        pub async fn reset_occurred(&mut self) -> Result<bool, Error<IFACE::Error>> {
            Ok(self.read_register(reg::RSTIRQSTATA).await? & IRQ_RESET != 0)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Ade7953<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            let [irms, watt, energy, pf] = self.channel.registers();
            let raw = RawResults {
                vrms: self.read_register(reg::VRMS).await?,
                irms: self.read_register(irms).await?,
                watt: self.read_register(watt).await?,
                pf: self.read_register(pf).await? as u16,
                period: self.read_register(reg::PERIOD).await? as u16,
            };
            let e = self.read_register(energy).await?;
            self.add_energy(e);
            Ok(decode(&self.calibration, &raw, self.energy_nwh()))
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cT};
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    /// 1 MΩ / 1 kΩ divider, 1 mΩ shunt on Channel A with PGA ×16.
    fn plug_calibration() -> Calibration {
        Calibration::from_front_end(1_000_000, 1_000, 1_000, Gain::X1, Gain::X16).unwrap()
    }

    fn rd(addr: u16, value: u32) -> I2cT {
        let w = register_width(addr);
        I2cT::write_read(
            I2C_ADDRESS,
            addr.to_be_bytes().to_vec(),
            value.to_be_bytes()[4 - w..].to_vec(),
        )
    }

    fn wr(addr: u16, value: u32) -> I2cT {
        let w = register_width(addr);
        let mut data = addr.to_be_bytes().to_vec();
        data.extend_from_slice(&value.to_be_bytes()[4 - w..]);
        I2cT::write(I2C_ADDRESS, data)
    }

    /// 230 V, 1 A, 230 W, PF 1, 50 Hz; `energy` AENERGYA counts.
    fn reads_a(energy: u32) -> Vec<I2cT> {
        vec![
            rd(reg::VRMS, 5_869_742),
            rd(reg::IRMSA, 408_742),
            rd(reg::AWATT, 143_006),
            rd(reg::PFA, 0x7FFF),
            rd(reg::PERIOD, 4_474),
            rd(reg::AENERGYA, energy),
        ]
    }

    #[test]
    fn calibration_and_conversions() {
        let unity = Calibration::from_front_end(999_000, 1_000, 1_000, Gain::X1, Gain::X1).unwrap();
        assert_eq!(
            unity,
            Calibration {
                voltage_nv: 39_144,
                current_pa: 39_144_499,
                power_nw: 25_707_464,
                energy_nwh: 167_821,
            }
        );
        // Full-scale codes at the pins: 353.553 V and 353.553 A.
        assert_eq!(unity.voltage_mv(FULL_SCALE_RMS), 353_549);
        assert_eq!(unity.current_ua(FULL_SCALE_RMS), 353_553_389);
        assert_eq!(unity.power_mw(-(FULL_SCALE_WATT as i32)), -124_999_999);
        let cal = plug_calibration();
        assert_eq!(cal.current_pa, 2_446_531);
        assert_eq!(cal.energy_nwh, 10_499);
        assert_eq!(
            Calibration::from_front_end(1, 0, 1, Gain::X1, Gain::X1),
            None
        );
        assert_eq!(
            Calibration::from_front_end(1, 1, 1, Gain::X1, Gain::X1),
            None
        );
        assert_eq!(sign24(0xFF_FFFF), -1);
        assert_eq!(sign24(0x7F_FFFF), 0x7F_FFFF);
        assert_eq!(power_factor_milli(0x7FFF), 1_000);
        assert_eq!(power_factor_milli(i16::MIN), -1_000);
        assert_eq!(power_factor_milli(0x4000), 500);
        assert_eq!(frequency_mhz(4_474), Some(50_000));
        assert_eq!(frequency_mhz(3_728), Some(60_003));
        assert_eq!(frequency_mhz(0), None);
        assert_eq!(register_width(reg::VERSION), 1);
        assert_eq!(register_width(reg::OPTIMUM), 2);
        assert_eq!(register_width(0x31C), 4);
        assert!(
            !Config {
                pga_ib: Gain::X22,
                ..Config::default()
            }
            .is_valid()
        );
    }

    #[test]
    fn init_and_measure_i2c() {
        let mut expectations = vec![
            wr(reg::CONFIG, 0x0004),
            wr(reg::UNLOCK, 0xAD),
            wr(reg::OPTIMUM, 0x0030),
            wr(reg::LCYCMODE, 0x40),
            wr(reg::PGA_V, 0),
            wr(reg::PGA_IA, 4),
            wr(reg::PGA_IB, 0),
            wr(reg::AIGAIN, 0x41_0000),
            wr(reg::AVGAIN, UNITY_GAIN),
            wr(reg::AWGAIN, UNITY_GAIN),
            wr(reg::BIGAIN, UNITY_GAIN),
            wr(reg::BWGAIN, UNITY_GAIN),
            rd(reg::RSTIRQSTATA, IRQ_RESET),
        ];
        expectations.extend(reads_a(100_000));
        expectations.extend(reads_a(0xFF_FFFF & (-5_000i32) as u32));
        expectations.push(rd(reg::RSTIRQSTATA, 0));
        let mut i2c = I2cMock::new(&expectations);
        let mut ade = Ade7953::new_i2c(&mut i2c, plug_calibration())
            .with_config(Config {
                pga_ia: Gain::X16,
                ..Config::default()
            })
            .with_gain_trim(GainTrim {
                aigain: 0x41_0000,
                ..GainTrim::default()
            });
        ade.init().unwrap();
        assert_eq!(ade.start_measurement().unwrap(), 0);
        let m = ade.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                voltage: 230_000,
                current: 1_000_000,
                power: 230_000,
                energy: 1_049,
                power_factor: 1_000,
                frequency: Some(50_000),
            }
        );
        let mut quantities = Vec::new();
        m.for_each(&mut |q| quantities.push(q));
        assert_eq!(quantities.len(), 6);
        assert_eq!(quantities[3], Quantity::Energy(1_049));
        assert_eq!(ade.energy_nwh(), 1_049_900_000);
        // Negative energy is netted.
        assert_eq!(ade.read_measurement().unwrap().energy, 997);
        assert!(!ade.reset_occurred().unwrap());
        i2c.done();
    }

    #[test]
    fn channel_b_and_invalid_gain() {
        let expectations = [
            rd(reg::VRMS, 0),
            rd(reg::IRMSB, 0),
            rd(reg::BWATT, 0xFF_FFFF),
            rd(reg::PFB, 0),
            rd(reg::PERIOD, 0xFFFF),
            rd(reg::AENERGYB, 0),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ade = Ade7953::new_i2c(&mut i2c, plug_calibration())
            .with_channel(Channel::B)
            .with_config(Config {
                pga_v: Gain::X22,
                ..Config::default()
            });
        assert_eq!(ade.init(), Err(Error::InvalidArgument));
        ade.set_energy_mwh(5_000);
        let m = ade.read_measurement().unwrap();
        // One LSB of export = -1.608 mW.
        assert_eq!(m.power, -2);
        assert_eq!(m.energy, 5_000);
        assert_eq!(m.frequency, None);
        ade.set_channel(Channel::A);
        assert_eq!(ade.energy_nwh(), 0);
        i2c.done();
    }

    #[test]
    fn spi_frames_and_reset() {
        let expectations = [
            SpiT::transaction_start(),
            SpiT::write_vec(vec![0x01, 0x02, 0x00, 0x80, 0x84]),
            SpiT::transaction_end(),
            SpiT::transaction_start(),
            SpiT::write_vec(vec![0x07, 0x02, 0x80]),
            SpiT::read_vec(vec![0x80]),
            SpiT::transaction_end(),
            SpiT::transaction_start(),
            SpiT::write_vec(vec![0x02, 0x2E, 0x80]),
            SpiT::read_vec(vec![0x10, 0x00, 0x00]),
            SpiT::transaction_end(),
            SpiT::transaction_start(),
            SpiT::write_vec(vec![0x02, 0x80, 0x00, 0x40, 0x00, 0x00]),
            SpiT::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut ade = Ade7953::new_spi(&mut spi, plug_calibration());
        assert_eq!(ade.reset().unwrap(), RESET_WAIT_US);
        assert_eq!(ade.version().unwrap(), 0x80);
        assert!(ade.reset_occurred().unwrap());
        ade.write_register(reg::AIGAIN, UNITY_GAIN).unwrap();
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

        let mut i2c = I2cMock::new(&reads_a(100_000));
        let mut ade = asynch::Ade7953::new_i2c(&mut i2c, plug_calibration());
        let mut delay = RecordingDelay(0);
        let m = block_on(ade.measure(&mut delay)).unwrap();
        assert_eq!(m.voltage, 230_000);
        assert_eq!(m.energy, 1_049);
        assert_eq!(delay.0, 0);
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
