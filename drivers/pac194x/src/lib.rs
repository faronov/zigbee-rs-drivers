//! Microchip PAC1941/PAC1942/PAC1943/PAC1944 one- to four-channel 9 V
//! power/energy monitors with 56-bit accumulators (I2C/SMBus).
//!
//! The device samples continuously at a fixed rate (1024/256/64/8 SPS) and
//! keeps the results in an internal shadow set; a REFRESH command copies
//! them to the readable registers (stable 1 ms later). The driver therefore
//! uses `start_measurement` = REFRESH_V (snapshot without clearing the
//! accumulators) → 1 ms, and `read_measurement` reads the snapshot. At
//! 8 SPS with four channels the part draws 12 µA typ, close to its 5 µA
//! sleep current, so continuous sampling is the low-power mode;
//! [`zigbee_sensor::blocking::LowPower`] selects the Sleep sample mode.
//!
//! Single-shot mode is not used: its results only become readable with a
//! second REFRESH ≥ 1 ms after the conversion, which does not fit the
//! start → wait → read model.
//!
//! Per channel the driver reports bus voltage (mV), current (µA, from the
//! configured shunt), power (mW, from the on-chip VPOWER product) and
//! energy (mWh, from the power accumulator, Equation 5-9).
//!
//! Reference: Microchip DS20006543C (PAC194X, 2022).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Address with ADDRSEL tied to GND. Resistors select 0x10..=0x1F.
pub const DEFAULT_ADDRESS: u8 = 0x10;

/// Maximum number of channels.
pub const CHANNELS: usize = 4;

/// Register addresses.
pub mod reg {
    /// REFRESH (send byte): snapshot results and clear accumulators.
    pub const REFRESH: u8 = 0x00;
    /// CTRL (sample mode, ALERT pins, channel disables).
    pub const CTRL: u8 = 0x01;
    /// ACC_COUNT (32 bit).
    pub const ACC_COUNT: u8 = 0x02;
    /// Accumulator of channel `n` (0-based, 56 bit).
    pub const fn vacc(n: usize) -> u8 {
        0x03 + n as u8
    }
    /// VBUS of channel `n`.
    pub const fn vbus(n: usize) -> u8 {
        0x07 + n as u8
    }
    /// VSENSE of channel `n`.
    pub const fn vsense(n: usize) -> u8 {
        0x0B + n as u8
    }
    /// 8-sample rolling average of VBUS of channel `n`.
    pub const fn vbus_avg(n: usize) -> u8 {
        0x0F + n as u8
    }
    /// 8-sample rolling average of VSENSE of channel `n`.
    pub const fn vsense_avg(n: usize) -> u8 {
        0x13 + n as u8
    }
    /// VPOWER of channel `n` (30 bit in bits 31..2).
    pub const fn vpower(n: usize) -> u8 {
        0x17 + n as u8
    }
    /// SMBUS_SETTINGS.
    pub const SMBUS_SETTINGS: u8 = 0x1C;
    /// NEG_PWR_FSR (unipolar/bipolar ranges).
    pub const NEG_PWR_FSR: u8 = 0x1D;
    /// REFRESH_G (send byte, general call).
    pub const REFRESH_G: u8 = 0x1E;
    /// REFRESH_V (send byte): snapshot results, keep accumulators.
    pub const REFRESH_V: u8 = 0x1F;
    /// ACCUM_CONFIG (accumulate VPOWER/VSENSE/VBUS).
    pub const ACCUM_CONFIG: u8 = 0x25;
    /// ALERT_STATUS.
    pub const ALERT_STATUS: u8 = 0x26;
    /// PRODUCT_ID.
    pub const PRODUCT_ID: u8 = 0xFD;
    /// MANUFACTURER_ID.
    pub const MANUFACTURER_ID: u8 = 0xFE;
    /// REVISION_ID.
    pub const REVISION_ID: u8 = 0xFF;
}

/// MANUFACTURER_ID register value.
pub const MANUFACTURER_ID: u8 = 0x54;

/// Results are stable, and new commands are accepted, 1 ms after any
/// REFRESH command (section 5.2).
pub const REFRESH_WAIT_US: WaitUs = 1_000;

/// Band-gap start-up when leaving Sleep (section 5.15.4).
pub const SLEEP_EXIT_US: WaitUs = 1_000;

/// Smallest supported shunt (keeps the 100 mV full scale within `i32` µA).
pub const MIN_SHUNT_MICRO_OHM: u32 = 50;

const SAMPLE_MODE_SLEEP: u16 = 0b1111;
/// CTRL bits 11..8 at POR: GPIO/ALERT2 = GPIO input, SLOW/ALERT1 = SLOW.
const CTRL_PIN_DEFAULTS: u16 = 0x0700;

/// Part number from PRODUCT_ID.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    /// PAC1941-1 (one channel).
    Pac1941_1,
    /// PAC1942-1 (two channels).
    Pac1942_1,
    /// PAC1943-1 (three channels).
    Pac1943_1,
    /// PAC1944-1 (four channels).
    Pac1944_1,
    /// PAC1941-2 (one channel, separate VBUS pin).
    Pac1941_2,
    /// PAC1942-2 (two channels, separate VBUS pins).
    Pac1942_2,
}

impl Chip {
    /// Number of channels.
    pub const fn channels(self) -> usize {
        match self {
            Self::Pac1941_1 | Self::Pac1941_2 => 1,
            Self::Pac1942_1 | Self::Pac1942_2 => 2,
            Self::Pac1943_1 => 3,
            Self::Pac1944_1 => 4,
        }
    }
}

/// Fixed sample rate (CTRL.SAMPLE_MODE 0100..0111). The accumulators
/// integrate at this rate; the SLOW pin must stay low (it forces 8 SPS).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SampleRate {
    /// 1024 samples/s (395 µA typ with four channels).
    Sps1024,
    /// 256 samples/s.
    Sps256,
    /// 64 samples/s.
    Sps64,
    /// 8 samples/s (12 µA typ with four channels).
    #[default]
    Sps8,
}

impl SampleRate {
    const fn mode(self) -> u16 {
        match self {
            Self::Sps1024 => 0b0100,
            Self::Sps256 => 0b0101,
            Self::Sps64 => 0b0110,
            Self::Sps8 => 0b0111,
        }
    }

    /// log2 of the rate in samples/s.
    pub const fn log2(self) -> u32 {
        match self {
            Self::Sps1024 => 10,
            Self::Sps256 => 8,
            Self::Sps64 => 6,
            Self::Sps8 => 3,
        }
    }
}

/// VBUS or VSENSE range (NEG_PWR_FSR field).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Range {
    /// 0..FSR, unsigned (default).
    #[default]
    Unipolar,
    /// ±FSR, two's complement, half resolution.
    Bipolar,
    /// ±FSR/2, two's complement, same LSB as unipolar.
    BipolarHalf,
}

impl Range {
    const fn bits(self) -> u16 {
        match self {
            Self::Unipolar => 0b00,
            Self::Bipolar => 0b01,
            Self::BipolarHalf => 0b10,
        }
    }
}

/// One channel: shunt resistance, ranges and precomputed scale factors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelConfig {
    /// Shunt resistance in µΩ.
    pub shunt_micro_ohm: u32,
    /// VBUS range.
    pub vbus: Range,
    /// VSENSE range.
    pub vsense: Range,
    current_q32: i64,
    power_q32: i64,
    energy_q62: u64,
}

impl ChannelConfig {
    /// Unipolar channel. `None` below [`MIN_SHUNT_MICRO_OHM`].
    pub const fn new(shunt_micro_ohm: u32) -> Option<Self> {
        Self::with_ranges(shunt_micro_ohm, Range::Unipolar, Range::Unipolar)
    }

    /// Channel with explicit VBUS/VSENSE ranges.
    pub const fn with_ranges(shunt_micro_ohm: u32, vbus: Range, vsense: Range) -> Option<Self> {
        if shunt_micro_ohm < MIN_SHUNT_MICRO_OHM {
            return None;
        }
        let r = shunt_micro_ohm as u64;
        // Equation 5-4: I = 0.1 V / R × VSENSE / 2^16 (2^15 bipolar) →
        // µA = VSENSE × 1e11 × 2^16 / R >> 32.
        let current_den = match vsense {
            Range::Bipolar => r,
            _ => 2 * r,
        };
        // Equations 5-5..5-7: P = 0.9 V² / R × VPOWER / 2^30 (2^29 when
        // VBUS or VSENSE is bipolar ±FSR) → mW = VPOWER × 3.6e9 / R >> 32.
        let power_bipolar = matches!(vbus, Range::Bipolar) || matches!(vsense, Range::Bipolar);
        let power_den = if power_bipolar { r } else { 2 * r };
        Some(Self {
            shunt_micro_ohm,
            vbus,
            vsense,
            current_q32: ((13_107_200_000_000_000 + current_den / 2) / current_den) as i64,
            power_q32: ((7_200_000_000 + power_den / 2) / power_den) as i64,
            // Equation 5-9: E = VACC × P_LSB / fs → mWh = VACC × 2.5e5 / R
            // / 2^30 / fs = VACC × (2.5e5 × 2^32 / R) >> (62 + log2 fs).
            energy_q62: (2_147_483_648_000_000 + power_den / 2) / power_den,
        })
    }

    const fn signed_power(&self) -> bool {
        !matches!(self.vbus, Range::Unipolar) || !matches!(self.vsense, Range::Unipolar)
    }

    /// Bus voltage in mV from a VBUS register value (137.3 µV LSB, 274.7 µV
    /// for bipolar ±9 V).
    pub const fn bus_millivolts(&self, raw: u16) -> i32 {
        match self.vbus {
            Range::Unipolar => ((raw as i32 * 9_000) + (1 << 15)) >> 16,
            Range::Bipolar => ((raw as i16 as i32 * 9_000) + (1 << 14)) >> 15,
            Range::BipolarHalf => ((raw as i16 as i32 * 9_000) + (1 << 15)) >> 16,
        }
    }

    /// Current in µA from a VSENSE register value.
    pub const fn current_ua(&self, raw: u16) -> i32 {
        let v = match self.vsense {
            Range::Unipolar => raw as i64,
            _ => raw as i16 as i64,
        };
        ((v * self.current_q32 + (1 << 31)) >> 32) as i32
    }

    /// Power in mW from a VPOWER register value (bits 31..2).
    pub const fn power_mw(&self, raw: u32) -> i32 {
        let v = if self.signed_power() {
            (raw as i32 >> 2) as i64
        } else {
            (raw >> 2) as i64
        };
        ((v * self.power_q32 + (1 << 31)) >> 32) as i32
    }

    /// Energy in mWh from a 56-bit VACC value accumulated at `rate`.
    pub const fn energy_mwh(&self, raw: u64, rate: SampleRate) -> i64 {
        let v = if self.signed_power() {
            ((raw << 8) as i64 >> 8) as i128
        } else {
            (raw & 0xFF_FFFF_FFFF_FFFF) as i128
        };
        let shift = 62 + rate.log2();
        ((v * self.energy_q62 as i128 + (1 << (shift - 1))) >> shift) as i64
    }
}

/// One channel reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Channel {
    /// Bus voltage in mV.
    pub bus_voltage: i32,
    /// Current in µA.
    pub current: i32,
    /// Power in mW.
    pub power: i32,
    /// Energy since `init` / `reset_accumulators` in mWh (negative for net
    /// reverse flow in bipolar modes).
    pub energy: i64,
}

/// One measurement (`None` for disabled channels).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Channels 1..=4.
    pub channels: [Option<Channel>; CHANNELS],
}

impl Readings for Measurement {
    /// Voltage, Current, Power and Energy per enabled channel; Energy is
    /// skipped while the bipolar net energy is negative.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        for ch in self.channels.iter().flatten() {
            f(Quantity::Voltage(ch.bus_voltage));
            f(Quantity::Current(ch.current));
            f(Quantity::Power(ch.power));
            if ch.energy >= 0 {
                f(Quantity::Energy(ch.energy as u64));
            }
        }
    }
}

/// Raw register values of one channel.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RawChannel {
    /// VBUS or VBUS_AVG.
    pub vbus: u16,
    /// VSENSE or VSENSE_AVG.
    pub vsense: u16,
    /// VPOWER.
    pub vpower: u32,
    /// VACC (56 bit).
    pub vacc: u64,
}

/// Decode one channel.
pub const fn decode_channel(cfg: &ChannelConfig, rate: SampleRate, raw: &RawChannel) -> Channel {
    Channel {
        bus_voltage: cfg.bus_millivolts(raw.vbus),
        current: cfg.current_ua(raw.vsense),
        power: cfg.power_mw(raw.vpower),
        energy: cfg.energy_mwh(raw.vacc, rate),
    }
}

/// Identify the part from PRODUCT_ID / MANUFACTURER_ID.
pub fn check_id<E>(product: u8, manufacturer: u8) -> Result<Chip, Error<E>> {
    let chip = match product {
        0x68 => Chip::Pac1941_1,
        0x69 => Chip::Pac1942_1,
        0x6A => Chip::Pac1943_1,
        0x6B => Chip::Pac1944_1,
        0x6C => Chip::Pac1941_2,
        0x6D => Chip::Pac1942_2,
        _ => {
            return Err(Error::UnexpectedId(
                (u32::from(manufacturer) << 8) | u32::from(product),
            ));
        }
    };
    if manufacturer != MANUFACTURER_ID {
        return Err(Error::UnexpectedId(
            (u32::from(manufacturer) << 8) | u32::from(product),
        ));
    }
    Ok(chip)
}

macro_rules! constructors {
    ($name:ident) => {
        impl<I2C> $name<I2C> {
            /// Create a driver with no channels enabled; add them with
            /// [`Self::with_channel`]. No bus traffic.
            pub const fn new(i2c: I2C, address: u8) -> Self {
                Self {
                    i2c,
                    address,
                    channels: [None; CHANNELS],
                    rate: SampleRate::Sps8,
                    averaged: true,
                }
            }

            /// Enable channel `index` (0-based); out-of-range indices are
            /// ignored.
            pub const fn with_channel(mut self, index: usize, config: ChannelConfig) -> Self {
                if index < CHANNELS {
                    self.channels[index] = Some(config);
                }
                self
            }

            /// Sample rate (default 8 SPS).
            pub const fn with_sample_rate(mut self, rate: SampleRate) -> Self {
                self.rate = rate;
                self
            }

            /// Report the 8-sample rolling averages of VBUS/VSENSE (default)
            /// or the latest sample.
            pub const fn with_averaging(mut self, averaged: bool) -> Self {
                self.averaged = averaged;
                self
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }

            const fn ctrl(&self, mode: u16) -> u16 {
                let mut off = 0u16;
                let mut n = 0;
                while n < CHANNELS {
                    if self.channels[n].is_none() {
                        off |= 1 << (7 - n);
                    }
                    n += 1;
                }
                (mode << 12) | CTRL_PIN_DEFAULTS | off
            }

            const fn neg_pwr_fsr(&self) -> u16 {
                let mut v = 0u16;
                let mut n = 0;
                while n < CHANNELS {
                    if let Some(c) = &self.channels[n] {
                        v |= c.vsense.bits() << (14 - 2 * n);
                        v |= c.vbus.bits() << (6 - 2 * n);
                    }
                    n += 1;
                }
                v
            }

            const fn bus_reg(&self, n: usize) -> u8 {
                if self.averaged {
                    reg::vbus_avg(n)
                } else {
                    reg::vbus(n)
                }
            }

            const fn sense_reg(&self, n: usize) -> u8 {
                if self.averaged {
                    reg::vsense_avg(n)
                } else {
                    reg::vsense(n)
                }
            }

            /// Time after `wake` until the snapshot holds post-sleep data:
            /// band-gap start + one sample period + REFRESH settling.
            pub const fn wake_us(&self) -> WaitUs {
                SLEEP_EXIT_US + (1_000_000 >> self.rate.log2()) + REFRESH_WAIT_US
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Pac194x<I2C> {
    i2c: I2C,
    address: u8,
    channels: [Option<ChannelConfig>; CHANNELS],
    rate: SampleRate,
    averaged: bool,
}

constructors!(Pac194x);

impl<I2C: embedded_hal::i2c::I2c> Pac194x<I2C> {
    fn read<const N: usize>(&mut self, reg: u8) -> Result<[u8; N], Error<I2C::Error>> {
        let mut buf = [0; N];
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .map_err(Error::bus)?;
        Ok(buf)
    }

    fn write16(&mut self, reg: u8, value: u16) -> Result<(), Error<I2C::Error>> {
        let [m, l] = value.to_be_bytes();
        self.i2c
            .write(self.address, &[reg, m, l])
            .map_err(Error::bus)
    }

    fn send(&mut self, command: u8) -> Result<WaitUs, Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[command])
            .map_err(Error::bus)?;
        Ok(REFRESH_WAIT_US)
    }

    /// Read PRODUCT_ID and MANUFACTURER_ID.
    pub fn probe(&mut self) -> Result<Chip, Error<I2C::Error>> {
        let [product] = self.read(reg::PRODUCT_ID)?;
        let [manufacturer] = self.read(reg::MANUFACTURER_ID)?;
        check_id(product, manufacturer)
    }

    /// Write NEG_PWR_FSR, ACCUM_CONFIG (VPOWER) and CTRL, then REFRESH to
    /// activate them and clear the accumulators. Wait the returned time
    /// before the next command (the device NACKs during it).
    pub fn init(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write16(reg::NEG_PWR_FSR, self.neg_pwr_fsr())?;
        self.i2c
            .write(self.address, &[reg::ACCUM_CONFIG, 0])
            .map_err(Error::bus)?;
        self.write16(reg::CTRL, self.ctrl(self.rate.mode()))?;
        self.send(reg::REFRESH)
    }

    /// REFRESH: snapshot and clear the energy accumulators.
    pub fn reset_accumulators(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.send(reg::REFRESH)
    }

    /// Accumulator count of the last snapshot.
    pub fn accumulator_count(&mut self) -> Result<u32, Error<I2C::Error>> {
        Ok(u32::from_be_bytes(self.read(reg::ACC_COUNT)?))
    }

    fn read_channel(&mut self, n: usize) -> Result<RawChannel, Error<I2C::Error>> {
        let vbus = u16::from_be_bytes(self.read(self.bus_reg(n))?);
        let vsense = u16::from_be_bytes(self.read(self.sense_reg(n))?);
        let vpower = u32::from_be_bytes(self.read(reg::vpower(n))?);
        let [a, b, c, d, e, f, g] = self.read(reg::vacc(n))?;
        Ok(RawChannel {
            vbus,
            vsense,
            vpower,
            vacc: u64::from_be_bytes([0, a, b, c, d, e, f, g]),
        })
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Pac194x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// REFRESH_V: snapshot the latest results without clearing the
    /// accumulators.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.send(reg::REFRESH_V)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut m = Measurement {
            channels: [None; CHANNELS],
        };
        for n in 0..CHANNELS {
            if let Some(cfg) = self.channels[n] {
                let raw = self.read_channel(n)?;
                m.channels[n] = Some(decode_channel(&cfg, self.rate, &raw));
            }
        }
        Ok(m)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Pac194x<I2C> {
    type BusError = I2C::Error;

    /// Sleep sample mode (5 µA typ), activated with REFRESH_V so the
    /// accumulators are kept. Wait [`REFRESH_WAIT_US`] before the next
    /// command.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write16(reg::CTRL, self.ctrl(SAMPLE_MODE_SLEEP))?;
        self.send(reg::REFRESH_V).map(|_| ())
    }

    /// Restore the sample rate; returns [`Pac194x::wake_us`].
    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write16(reg::CTRL, self.ctrl(self.rate.mode()))?;
        self.send(reg::REFRESH_V)?;
        Ok(self.wake_us())
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CHANNELS, CTRL_PIN_DEFAULTS, ChannelConfig, Chip, Measurement, REFRESH_WAIT_US, RawChannel,
        SAMPLE_MODE_SLEEP, SLEEP_EXIT_US, SampleRate, check_id, decode_channel, reg,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Pac194x`].
    #[derive(Debug)]
    pub struct Pac194x<I2C> {
        i2c: I2C,
        address: u8,
        channels: [Option<ChannelConfig>; CHANNELS],
        rate: SampleRate,
        averaged: bool,
    }

    constructors!(Pac194x);

    impl<I2C: I2c> Pac194x<I2C> {
        async fn read<const N: usize>(&mut self, reg: u8) -> Result<[u8; N], Error<I2C::Error>> {
            let mut buf = [0; N];
            self.i2c
                .write_read(self.address, &[reg], &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(buf)
        }

        async fn write16(&mut self, reg: u8, value: u16) -> Result<(), Error<I2C::Error>> {
            let [m, l] = value.to_be_bytes();
            self.i2c
                .write(self.address, &[reg, m, l])
                .await
                .map_err(Error::bus)
        }

        async fn send(&mut self, command: u8) -> Result<WaitUs, Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[command])
                .await
                .map_err(Error::bus)?;
            Ok(REFRESH_WAIT_US)
        }

        /// See [`super::Pac194x::probe`].
        pub async fn probe(&mut self) -> Result<Chip, Error<I2C::Error>> {
            let [product] = self.read(reg::PRODUCT_ID).await?;
            let [manufacturer] = self.read(reg::MANUFACTURER_ID).await?;
            check_id(product, manufacturer)
        }

        /// See [`super::Pac194x::init`].
        pub async fn init(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write16(reg::NEG_PWR_FSR, self.neg_pwr_fsr()).await?;
            self.i2c
                .write(self.address, &[reg::ACCUM_CONFIG, 0])
                .await
                .map_err(Error::bus)?;
            self.write16(reg::CTRL, self.ctrl(self.rate.mode())).await?;
            self.send(reg::REFRESH).await
        }

        /// See [`super::Pac194x::reset_accumulators`].
        pub async fn reset_accumulators(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(reg::REFRESH).await
        }

        /// See [`super::Pac194x::accumulator_count`].
        pub async fn accumulator_count(&mut self) -> Result<u32, Error<I2C::Error>> {
            Ok(u32::from_be_bytes(self.read(reg::ACC_COUNT).await?))
        }

        async fn read_channel(&mut self, n: usize) -> Result<RawChannel, Error<I2C::Error>> {
            let vbus = u16::from_be_bytes(self.read(self.bus_reg(n)).await?);
            let vsense = u16::from_be_bytes(self.read(self.sense_reg(n)).await?);
            let vpower = u32::from_be_bytes(self.read(reg::vpower(n)).await?);
            let [a, b, c, d, e, f, g] = self.read(reg::vacc(n)).await?;
            Ok(RawChannel {
                vbus,
                vsense,
                vpower,
                vacc: u64::from_be_bytes([0, a, b, c, d, e, f, g]),
            })
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Pac194x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(reg::REFRESH_V).await
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut m = Measurement {
                channels: [None; CHANNELS],
            };
            for n in 0..CHANNELS {
                if let Some(cfg) = self.channels[n] {
                    let raw = self.read_channel(n).await?;
                    m.channels[n] = Some(decode_channel(&cfg, self.rate, &raw));
                }
            }
            Ok(m)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Pac194x<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write16(reg::CTRL, self.ctrl(SAMPLE_MODE_SLEEP))
                .await?;
            self.send(reg::REFRESH_V).await.map(|_| ())
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write16(reg::CTRL, self.ctrl(self.rate.mode())).await?;
            self.send(reg::REFRESH_V).await?;
            Ok(self.wake_us())
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as T};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const ADDR: u8 = DEFAULT_ADDRESS;

    fn rd(reg: u8, data: Vec<u8>) -> T {
        T::write_read(ADDR, vec![reg], data)
    }

    fn channel_reads(n: usize, avg: bool, raw: RawChannel) -> Vec<T> {
        let (b, s) = if avg {
            (reg::vbus_avg(n), reg::vsense_avg(n))
        } else {
            (reg::vbus(n), reg::vsense(n))
        };
        vec![
            rd(b, raw.vbus.to_be_bytes().to_vec()),
            rd(s, raw.vsense.to_be_bytes().to_vec()),
            rd(reg::vpower(n), raw.vpower.to_be_bytes().to_vec()),
            rd(reg::vacc(n), raw.vacc.to_be_bytes()[1..].to_vec()),
        ]
    }

    /// 10 mΩ, half-scale bus (4.5 V) and sense (50 mV → 5 A); 22.5 W held
    /// for one hour at 8 SPS.
    const HALF: RawChannel = RawChannel {
        vbus: 0x8000,
        vsense: 0x8000,
        vpower: (1 << 28) << 2,
        vacc: (1 << 28) * 8 * 3_600,
    };

    const HALF_DECODED: Channel = Channel {
        bus_voltage: 4_500,
        current: 5_000_000,
        power: 22_500,
        energy: 22_500,
    };

    #[test]
    fn unipolar_scaling() {
        let c = ChannelConfig::new(10_000).unwrap();
        assert_eq!(decode_channel(&c, SampleRate::Sps8, &HALF), HALF_DECODED);
        assert_eq!(c.bus_millivolts(0xFFFF), 9_000);
        // Full scale with the minimum shunt still fits i32.
        let min = ChannelConfig::new(MIN_SHUNT_MICRO_OHM).unwrap();
        assert_eq!(min.current_ua(0xFFFF), 1_999_969_482);
        assert_eq!(ChannelConfig::new(49), None);
        // Same energy at 1024 SPS needs 128× the accumulator.
        assert_eq!(c.energy_mwh(HALF.vacc * 128, SampleRate::Sps1024), 22_500);
    }

    #[test]
    fn bipolar_scaling() {
        let c = ChannelConfig::with_ranges(10_000, Range::Bipolar, Range::Bipolar).unwrap();
        assert_eq!(c.bus_millivolts(0x8000), -9_000);
        assert_eq!(c.bus_millivolts(0x4000), 4_500);
        assert_eq!(c.current_ua(0x8000), -10_000_000);
        assert_eq!(c.current_ua(0x2000), 2_500_000);
        // −2^27 of 2^29 full scale (90 W) = −22.5 W.
        assert_eq!(c.power_mw(((-(1i32 << 27)) << 2) as u32), -22_500);
        let vacc = (-(1i64 << 27) * 8 * 3_600) as u64 & 0xFF_FFFF_FFFF_FFFF;
        assert_eq!(c.energy_mwh(vacc, SampleRate::Sps8), -22_500);
        let half =
            ChannelConfig::with_ranges(10_000, Range::BipolarHalf, Range::BipolarHalf).unwrap();
        assert_eq!(half.bus_millivolts(0xC000), -2_250);
        assert_eq!(half.current_ua(0xC000), -2_500_000);
        assert_eq!(half.power_mw(((-(1i32 << 28)) << 2) as u32), -22_500);
    }

    #[test]
    fn probe_identifies_parts() {
        let expectations = [
            rd(reg::PRODUCT_ID, vec![0x6B]),
            rd(reg::MANUFACTURER_ID, vec![0x54]),
            rd(reg::PRODUCT_ID, vec![0x6D]),
            rd(reg::MANUFACTURER_ID, vec![0x55]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut pac = Pac194x::new(&mut i2c, ADDR);
        let chip = pac.probe().unwrap();
        assert_eq!((chip, chip.channels()), (Chip::Pac1944_1, 4));
        assert_eq!(pac.probe(), Err(Error::UnexpectedId(0x556D)));
        i2c.done();
    }

    #[test]
    fn init_measure_sleep() {
        let mut expectations = vec![
            // Channel 1 bipolar VSENSE, channel 3 unipolar; 2 and 4 off.
            T::write(ADDR, vec![reg::NEG_PWR_FSR, 0x40, 0x00]),
            T::write(ADDR, vec![reg::ACCUM_CONFIG, 0x00]),
            T::write(ADDR, vec![reg::CTRL, 0x77, 0x50]),
            T::write(ADDR, vec![reg::REFRESH]),
            T::write(ADDR, vec![reg::REFRESH_V]),
        ];
        let bipolar = RawChannel {
            vbus: 0x2000,
            vsense: 0xF000,
            vpower: ((-(1i32 << 23)) << 2) as u32,
            vacc: 0,
        };
        expectations.extend(channel_reads(0, true, bipolar));
        expectations.extend(channel_reads(2, true, HALF));
        expectations.extend([
            T::write(ADDR, vec![reg::CTRL, 0xF7, 0x50]),
            T::write(ADDR, vec![reg::REFRESH_V]),
            T::write(ADDR, vec![reg::CTRL, 0x77, 0x50]),
            T::write(ADDR, vec![reg::REFRESH_V]),
        ]);
        let mut i2c = I2cMock::new(&expectations);
        let ch1 = ChannelConfig::with_ranges(10_000, Range::Unipolar, Range::Bipolar).unwrap();
        let mut pac = Pac194x::new(&mut i2c, ADDR)
            .with_channel(0, ch1)
            .with_channel(2, ChannelConfig::new(10_000).unwrap());
        assert_eq!(pac.init().unwrap(), REFRESH_WAIT_US);
        assert_eq!(pac.start_measurement().unwrap(), REFRESH_WAIT_US);
        let m = pac.read_measurement().unwrap();
        assert_eq!(
            m.channels[0],
            Some(Channel {
                bus_voltage: 1_125,
                current: -1_250_000,
                power: -1_406,
                energy: 0,
            })
        );
        assert_eq!(m.channels[1], None);
        assert_eq!(m.channels[2], Some(HALF_DECODED));
        let mut q = Vec::new();
        m.for_each(&mut |x| q.push(x));
        assert_eq!(q.len(), 8);
        pac.sleep().unwrap();
        assert_eq!(pac.wake().unwrap(), 127_000);
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

        let mut expectations = vec![T::write(ADDR, vec![reg::REFRESH_V])];
        expectations.extend(channel_reads(0, false, HALF));
        let mut i2c = I2cMock::new(&expectations);
        let mut pac = asynch::Pac194x::new(&mut i2c, ADDR)
            .with_channel(0, ChannelConfig::new(10_000).unwrap())
            .with_averaging(false);
        let mut delay = RecordingDelay(0);
        let m = block_on(pac.measure(&mut delay)).unwrap();
        assert_eq!(m.channels[0], Some(HALF_DECODED));
        assert_eq!(delay.0, REFRESH_WAIT_US);
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
