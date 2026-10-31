//! Sensirion SEN50 / SEN54 / SEN55 environmental sensor nodes (I2C, 0x69):
//! PM1.0 / PM2.5 / PM4.0 / PM10, compensated RH and temperature, VOC and NOx
//! index. Start / stop, RHT/gas-only mode, data-ready flag, temperature
//! compensation, warm start, RH/T acceleration, VOC / NOx tuning, VOC
//! algorithm state, fan cleaning, product name (`probe`), serial number,
//! firmware version, device status and reset.
//!
//! # Measurement through the [`Sensor`](zigbee_sensor::blocking::Sensor) trait
//!
//! The first `start_measurement` starts the configured [`Mode`] and returns
//! the 1 s sampling interval; the next read is [`Error::NotReady`]. Later
//! starts send `Read Measured Values` and return 20 ms; the read then
//! returns the latest sample (the sensor is free-running at 1 Hz).
//!
//! With `with_number_concentration(true)` (and [`Mode::Full`]), every
//! second cycle sends `Read Measured PM Values` (0x0413) instead: that
//! sample has PM mass, number concentrations and typical particle size but
//! no RH/T/VOC/NOx. The SEN50, which has PM only, always uses it.
//!
//! Outputs the variant does not have, and values the sensor flags as
//! unknown (0xFFFF for PM, 0x7FFF for the signed values — e.g. PM in
//! RHT/gas-only mode, or the NOx index during its start-up), are `None`.
//!
//! The SEN5x has no sleep mode (idle 0.7–3.8 mA), so there is no
//! `LowPower` implementation; switch the 5 V supply to save power.
//!
//! Reference: Sensirion "Datasheet SEN5x", version 2 (March 2022, D1).
//! `Read Measured PM Values` (0x0413) is not in that datasheet; its layout
//! follows Sensirion's `embedded-i2c-sen5x` driver.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address.
pub const ADDRESS: u8 = 0x69;

mod cmd {
    pub const START_MEASUREMENT: u16 = 0x0021;
    pub const START_RHT_GAS_ONLY: u16 = 0x0037;
    pub const STOP_MEASUREMENT: u16 = 0x0104;
    pub const READ_DATA_READY: u16 = 0x0202;
    pub const READ_MEASURED_VALUES: u16 = 0x03C4;
    pub const READ_MEASURED_PM_VALUES: u16 = 0x0413;
    pub const TEMPERATURE_COMPENSATION: u16 = 0x60B2;
    pub const WARM_START: u16 = 0x60C6;
    pub const VOC_TUNING: u16 = 0x60D0;
    pub const NOX_TUNING: u16 = 0x60E1;
    pub const RHT_ACCELERATION: u16 = 0x60F7;
    pub const VOC_STATE: u16 = 0x6181;
    pub const START_FAN_CLEANING: u16 = 0x5607;
    pub const AUTO_CLEANING_INTERVAL: u16 = 0x8004;
    pub const PRODUCT_NAME: u16 = 0xD014;
    pub const SERIAL_NUMBER: u16 = 0xD033;
    pub const FIRMWARE_VERSION: u16 = 0xD100;
    pub const READ_DEVICE_STATUS: u16 = 0xD206;
    pub const CLEAR_DEVICE_STATUS: u16 = 0xD210;
    pub const RESET: u16 = 0xD304;
}

/// Start measurement execution time.
pub const START_WAIT_US: WaitUs = 50_000;
/// Stop measurement execution time.
pub const STOP_WAIT_US: WaitUs = 200_000;
/// Execution time of the other commands (wait before reading a response).
pub const COMMAND_WAIT_US: WaitUs = 20_000;
/// Device reset execution time.
pub const RESET_WAIT_US: WaitUs = 100_000;
/// Sampling interval in either measurement mode.
pub const SAMPLE_INTERVAL_US: WaitUs = 1_000_000;

/// Device status register: fan speed out of range.
pub const STATUS_FAN_SPEED: u32 = 1 << 21;
/// Device status register: fan cleaning active.
pub const STATUS_FAN_CLEANING: u32 = 1 << 19;
/// Device status register: gas sensor (VOC / NOx) error.
pub const STATUS_GAS_SENSOR: u32 = 1 << 7;
/// Device status register: internal RHT sensor communication error.
pub const STATUS_RHT: u32 = 1 << 6;
/// Device status register: laser current out of range.
pub const STATUS_LASER: u32 = 1 << 5;
/// Device status register: fan blocked or broken.
pub const STATUS_FAN: u32 = 1 << 4;

/// Product variant; selects which outputs exist.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// PM only.
    Sen50,
    /// PM, RH/T, VOC index.
    Sen54,
    /// PM, RH/T, VOC and NOx index.
    Sen55,
}

impl Variant {
    const fn has_rht_voc(self) -> bool {
        !matches!(self, Self::Sen50)
    }

    const fn has_nox(self) -> bool {
        matches!(self, Self::Sen55)
    }
}

/// Measurement mode started by the [`Sensor`](zigbee_sensor::blocking::Sensor)
/// trait.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// All outputs (fan and laser on, 63–70 mA).
    #[default]
    Full,
    /// RH/T and gas only, fan and laser off (6.5–6.8 mA; SEN54/SEN55).
    RhtGasOnly,
}

/// One `Read Measured Values` result; `None` = not available.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Measurement {
    /// PM1.0 mass concentration, 0.1 µg/m³.
    pub pm1_0: Option<u32>,
    /// PM2.5 mass concentration, 0.1 µg/m³.
    pub pm2_5: Option<u32>,
    /// PM4.0 mass concentration, 0.1 µg/m³.
    pub pm4_0: Option<u32>,
    /// PM10 mass concentration, 0.1 µg/m³.
    pub pm10: Option<u32>,
    /// Compensated relative humidity, 0.01 %RH.
    pub humidity: Option<u16>,
    /// Compensated temperature, 0.01 °C.
    pub temperature: Option<i32>,
    /// VOC index (1–500).
    pub voc_index: Option<u16>,
    /// NOx index (1–500).
    pub nox_index: Option<u16>,
    /// Number concentrations (`Read Measured PM Values` samples only).
    pub number_concentration: Option<NumberConcentration>,
    /// Typical particle size, nm (`Read Measured PM Values` samples only).
    pub typical_particle_size: Option<u32>,
}

/// Upper size bounds of the number-concentration bins, nm.
pub const NC_SIZES_NM: [u16; 5] = [500, 1000, 2500, 4000, 10_000];

/// Number concentrations in 0.01 particles/cm³ (sensor resolution
/// 0.1 #/cm³; `None` = unknown).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NumberConcentration {
    /// Particles 0.3–0.5 µm.
    pub pm0_5: Option<u32>,
    /// Particles 0.3–1.0 µm.
    pub pm1_0: Option<u32>,
    /// Particles 0.3–2.5 µm.
    pub pm2_5: Option<u32>,
    /// Particles 0.3–4.0 µm.
    pub pm4_0: Option<u32>,
    /// Particles 0.3–10 µm.
    pub pm10: Option<u32>,
}

impl Readings for NumberConcentration {
    /// One [`Quantity::ParticlesBelow`] per known bin.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        let counts = [self.pm0_5, self.pm1_0, self.pm2_5, self.pm4_0, self.pm10];
        for (&size_nm, count) in NC_SIZES_NM.iter().zip(counts) {
            if let Some(count) = count {
                f(Quantity::ParticlesBelow { size_nm, count });
            }
        }
    }
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        let values = [
            self.temperature.map(Quantity::Temperature),
            self.humidity.map(Quantity::Humidity),
            self.pm1_0.map(Quantity::Pm1),
            self.pm2_5.map(Quantity::Pm25),
            self.pm4_0.map(Quantity::Pm4),
            self.pm10.map(Quantity::Pm10),
            self.voc_index.map(Quantity::VocIndex),
            self.nox_index.map(Quantity::NoxIndex),
        ];
        values.into_iter().flatten().for_each(&mut *f);
        if let Some(n) = &self.number_concentration {
            n.for_each(f);
        }
        if let Some(size) = self.typical_particle_size {
            f(Quantity::TypicalParticleSize(size));
        }
    }
}

/// Temperature compensation: `T = T_ambient + slope·T_ambient + offset`,
/// applied with the given time constant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TemperatureCompensation {
    /// Offset in 0.01 °C (-163.84–163.83 °C).
    pub offset: i32,
    /// Normalised slope in 1/10 000.
    pub slope: i16,
    /// Time constant in seconds (0 = immediately).
    pub time_constant_s: u16,
}

/// VOC / NOx algorithm tuning parameters (section 6.1.8 / 6.1.9).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AlgorithmTuning {
    /// Index representing typical conditions (1–250).
    pub index_offset: i16,
    /// Offset learning time, hours (1–1000).
    pub learning_time_offset_hours: i16,
    /// Gain learning time, hours (1–1000; must be 12 for NOx).
    pub learning_time_gain_hours: i16,
    /// Maximum gating duration, minutes (0–3000).
    pub gating_max_duration_minutes: i16,
    /// Initial standard deviation (10–5000; must be 50 for NOx).
    pub std_initial: i16,
    /// Gain factor (1–1000).
    pub gain_factor: i16,
}

impl AlgorithmTuning {
    /// VOC defaults.
    pub const VOC_DEFAULT: Self = Self {
        index_offset: 100,
        learning_time_offset_hours: 12,
        learning_time_gain_hours: 12,
        gating_max_duration_minutes: 180,
        std_initial: 50,
        gain_factor: 230,
    };
    /// NOx defaults.
    pub const NOX_DEFAULT: Self = Self {
        index_offset: 1,
        learning_time_offset_hours: 12,
        learning_time_gain_hours: 12,
        gating_max_duration_minutes: 720,
        std_initial: 50,
        gain_factor: 230,
    };

    fn to_words(self) -> [u16; 6] {
        [
            self.index_offset as u16,
            self.learning_time_offset_hours as u16,
            self.learning_time_gain_hours as u16,
            self.gating_max_duration_minutes as u16,
            self.std_initial as u16,
            self.gain_factor as u16,
        ]
    }

    fn from_words(w: [u16; 6]) -> Self {
        Self {
            index_offset: w[0] as i16,
            learning_time_offset_hours: w[1] as i16,
            learning_time_gain_hours: w[2] as i16,
            gating_max_duration_minutes: w[3] as i16,
            std_initial: w[4] as i16,
            gain_factor: w[5] as i16,
        }
    }
}

/// RH/T acceleration mode (applied at the next start).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RhtAcceleration {
    /// Stationary devices (default).
    Low = 0,
    /// Devices with large temperature changes.
    High = 1,
    /// Between low and high.
    Medium = 2,
}

fn mass(raw: u16) -> Option<u32> {
    (raw != 0xFFFF).then_some(u32::from(raw))
}

fn signed(raw: u16) -> Option<i16> {
    (raw != 0x7FFF).then_some(raw as i16)
}

fn index(raw: u16) -> Option<u16> {
    signed(raw).map(|v| ((i32::from(v) + 5) / 10).clamp(0, 500) as u16)
}

fn convert(w: &[u16; 8], variant: Variant) -> Measurement {
    let rht_voc = variant.has_rht_voc();
    Measurement {
        pm1_0: mass(w[0]),
        pm2_5: mass(w[1]),
        pm4_0: mass(w[2]),
        pm10: mass(w[3]),
        humidity: signed(w[4])
            .filter(|_| rht_voc)
            .map(|v| v.clamp(0, 10_000) as u16),
        temperature: signed(w[5]).filter(|_| rht_voc).map(|v| i32::from(v) / 2),
        voc_index: index(w[6]).filter(|_| rht_voc),
        nox_index: index(w[7]).filter(|_| variant.has_nox()),
        ..Measurement::default()
    }
}

/// `Read Measured PM Values`: mass, number concentration (0.1 #/cm³) and
/// typical particle size (0.001 µm = nm).
fn convert_pm(w: &[u16; 10]) -> Measurement {
    let number = |raw: u16| mass(raw).map(|v| v * 10);
    Measurement {
        pm1_0: mass(w[0]),
        pm2_5: mass(w[1]),
        pm4_0: mass(w[2]),
        pm10: mass(w[3]),
        number_concentration: Some(NumberConcentration {
            pm0_5: number(w[4]),
            pm1_0: number(w[5]),
            pm2_5: number(w[6]),
            pm4_0: number(w[7]),
            pm10: number(w[8]),
        }),
        typical_particle_size: mass(w[9]),
        ..Measurement::default()
    }
}

fn u32_from_words(w: [u16; 2]) -> u32 {
    (u32::from(w[0]) << 16) | u32::from(w[1])
}

/// Decode a NUL-terminated product name ("SEN50", "SEN54", "SEN55").
fn variant_from_name(name: &[u8]) -> Option<Variant> {
    match name {
        [b'S', b'E', b'N', b'5', b'0', 0, ..] => Some(Variant::Sen50),
        [b'S', b'E', b'N', b'5', b'4', 0, ..] => Some(Variant::Sen54),
        [b'S', b'E', b'N', b'5', b'5', 0, ..] => Some(Variant::Sen55),
        _ => None,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    Nothing,
    Read,
    ReadPm,
}

macro_rules! constructors {
    () => {
        /// Create a driver for `variant`; no bus traffic.
        pub const fn new(i2c: I2C, variant: Variant) -> Self {
            Self {
                bus: Interface::new(i2c, ADDRESS),
                variant,
                mode: Mode::Full,
                running: false,
                pending: Pending::Nothing,
                number_concentration: false,
                next_pm: false,
            }
        }

        /// Let the [`Sensor`](zigbee_sensor::blocking::Sensor) cycle read
        /// number concentrations and typical particle size in
        /// [`Mode::Full`] (default off): every second sample on the
        /// SEN54/SEN55 (without RH/T/VOC/NOx), every sample on the SEN50.
        pub const fn with_number_concentration(mut self, enable: bool) -> Self {
            self.number_concentration = enable;
            self
        }

        /// Mode started by `start_measurement` (default [`Mode::Full`]).
        pub const fn with_mode(mut self, mode: Mode) -> Self {
            self.mode = mode;
            self
        }

        /// Configured variant (updated by `probe`).
        pub const fn variant(&self) -> Variant {
            self.variant
        }

        /// `true` while the driver has started a measurement mode.
        pub const fn is_running(&self) -> bool {
            self.running
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.bus.release()
        }
    };
}

// Bus methods shared by the blocking and async drivers: `$async` is empty or
// `async`, `$aw` is empty or `.await`.
macro_rules! methods {
    ([$($async:tt)*] [$($aw:tt)*] $Delay:path) => {
        $($async)* fn send(&mut self, cmd: u16, args: &[u16], wait: WaitUs) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(cmd, args) $($aw)* ?;
            Ok(wait)
        }

        $($async)* fn get<const N: usize, D: $Delay>(&mut self, cmd: u16, delay: &mut D) -> Result<[u16; N], Error<I2C::Error>> {
            let mut w = [0u16; N];
            self.bus.fetch(cmd, &[], COMMAND_WAIT_US, delay, &mut w) $($aw)* ?;
            Ok(w)
        }

        /// Read the product name and select the [`Variant`]; an unknown
        /// name gives [`Error::UnexpectedId`] with its first four bytes.
        pub $($async)* fn probe<D: $Delay>(&mut self, delay: &mut D) -> Result<Variant, Error<I2C::Error>> {
            let name = self.product_name(delay) $($aw)* ?;
            let variant = variant_from_name(&name).ok_or_else(|| {
                let [a, b, c, d, ..] = name;
                Error::UnexpectedId(u32::from_be_bytes([a, b, c, d]))
            })?;
            self.variant = variant;
            Ok(variant)
        }

        /// Product name, NUL-terminated ASCII (e.g. `"SEN55"`).
        pub $($async)* fn product_name<D: $Delay>(&mut self, delay: &mut D) -> Result<[u8; 32], Error<I2C::Error>> {
            let w: [u16; 16] = self.get(cmd::PRODUCT_NAME, delay) $($aw)* ?;
            let mut out = [0u8; 32];
            words_to_bytes(&w, &mut out);
            Ok(out)
        }

        /// Serial number, NUL-terminated ASCII.
        pub $($async)* fn serial_number<D: $Delay>(&mut self, delay: &mut D) -> Result<[u8; 32], Error<I2C::Error>> {
            let w: [u16; 16] = self.get(cmd::SERIAL_NUMBER, delay) $($aw)* ?;
            let mut out = [0u8; 32];
            words_to_bytes(&w, &mut out);
            Ok(out)
        }

        /// Firmware version (direct mode switching needs ≥ 2).
        pub $($async)* fn firmware_version<D: $Delay>(&mut self, delay: &mut D) -> Result<u8, Error<I2C::Error>> {
            let [w]: [u16; 1] = self.get(cmd::FIRMWARE_VERSION, delay) $($aw)* ?;
            Ok(w.to_be_bytes()[0])
        }

        /// Start Measurement-Mode (all outputs).
        pub $($async)* fn start_continuous_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::START_MEASUREMENT, &[], START_WAIT_US) $($aw)* ?;
            self.running = true;
            self.pending = Pending::Nothing;
            Ok(START_WAIT_US)
        }

        /// Start RHT/gas-only mode (fan and laser off, PM = `None`). The
        /// SEN50 gives [`Error::Unsupported`] without bus traffic.
        pub $($async)* fn start_rht_gas_only_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if !self.variant.has_rht_voc() {
                return Err(Error::Unsupported);
            }
            self.send(cmd::START_RHT_GAS_ONLY, &[], START_WAIT_US) $($aw)* ?;
            self.running = true;
            self.pending = Pending::Nothing;
            Ok(START_WAIT_US)
        }

        /// Return to Idle-Mode (200 ms).
        pub $($async)* fn stop_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::STOP_MEASUREMENT, &[], STOP_WAIT_US) $($aw)* ?;
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(STOP_WAIT_US)
        }

        /// `true` if a new sample is available.
        pub $($async)* fn read_data_ready<D: $Delay>(&mut self, delay: &mut D) -> Result<bool, Error<I2C::Error>> {
            let [w]: [u16; 1] = self.get(cmd::READ_DATA_READY, delay) $($aw)* ?;
            Ok(w & 0x00FF == 1)
        }

        /// Request the latest values (clears the data-ready flag); read
        /// them after 20 ms with [`read_measured_values`](Self::read_measured_values).
        pub $($async)* fn request_measured_values(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::READ_MEASURED_VALUES, &[], COMMAND_WAIT_US) $($aw)* ?;
            self.pending = Pending::Read;
            Ok(COMMAND_WAIT_US)
        }

        /// Request PM mass, number concentrations and typical particle
        /// size; read them after 20 ms with
        /// [`read_measured_values`](Self::read_measured_values).
        pub $($async)* fn request_measured_pm_values(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::READ_MEASURED_PM_VALUES, &[], COMMAND_WAIT_US) $($aw)* ?;
            self.pending = Pending::ReadPm;
            Ok(COMMAND_WAIT_US)
        }

        /// Values requested by `request_measured_values` or
        /// `request_measured_pm_values`; [`Error::NotReady`] if nothing was
        /// requested or the sensor is still busy.
        pub $($async)* fn read_measured_values(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            match self.pending {
                Pending::Nothing => Err(Error::NotReady),
                Pending::Read => {
                    let mut w = [0u16; 8];
                    self.bus.read(&mut w) $($aw)* .map_err(nack_as_not_ready)?;
                    self.pending = Pending::Nothing;
                    self.next_pm = self.number_concentration && self.mode == Mode::Full;
                    Ok(convert(&w, self.variant))
                }
                Pending::ReadPm => {
                    let mut w = [0u16; 10];
                    self.bus.read(&mut w) $($aw)* .map_err(nack_as_not_ready)?;
                    self.pending = Pending::Nothing;
                    Ok(convert_pm(&w))
                }
            }
        }

        /// Set the temperature compensation (volatile). An offset outside
        /// ±163.83 °C gives [`Error::InvalidArgument`] without bus traffic.
        pub $($async)* fn set_temperature_compensation(&mut self, p: TemperatureCompensation) -> Result<WaitUs, Error<I2C::Error>> {
            let offset = i16::try_from(p.offset * 2).map_err(|_| Error::InvalidArgument)?;
            let args = [offset as u16, p.slope as u16, p.time_constant_s];
            self.send(cmd::TEMPERATURE_COMPENSATION, &args, COMMAND_WAIT_US) $($aw)*
        }

        /// Read the temperature compensation.
        pub $($async)* fn temperature_compensation<D: $Delay>(&mut self, delay: &mut D) -> Result<TemperatureCompensation, Error<I2C::Error>> {
            let w: [u16; 3] = self.get(cmd::TEMPERATURE_COMPENSATION, delay) $($aw)* ?;
            Ok(TemperatureCompensation {
                offset: i32::from(w[0] as i16) / 2,
                slope: w[1] as i16,
                time_constant_s: w[2],
            })
        }

        /// Warm-start parameter 0 (cold, default) – 65535 (warm); applied
        /// at the next start.
        pub $($async)* fn set_warm_start(&mut self, value: u16) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::WARM_START, &[value], COMMAND_WAIT_US) $($aw)*
        }

        /// RH/T acceleration mode; applied at the next start.
        pub $($async)* fn set_rht_acceleration(&mut self, mode: RhtAcceleration) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::RHT_ACCELERATION, &[mode as u16], COMMAND_WAIT_US) $($aw)*
        }

        /// Set VOC algorithm tuning (Idle-Mode only; ignored by the sensor
        /// if any value is out of range).
        pub $($async)* fn set_voc_tuning(&mut self, t: AlgorithmTuning) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::VOC_TUNING, &t.to_words(), COMMAND_WAIT_US) $($aw)*
        }

        /// Read VOC algorithm tuning.
        pub $($async)* fn voc_tuning<D: $Delay>(&mut self, delay: &mut D) -> Result<AlgorithmTuning, Error<I2C::Error>> {
            Ok(AlgorithmTuning::from_words(self.get(cmd::VOC_TUNING, delay) $($aw)* ?))
        }

        /// Set NOx algorithm tuning (Idle-Mode only).
        pub $($async)* fn set_nox_tuning(&mut self, t: AlgorithmTuning) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::NOX_TUNING, &t.to_words(), COMMAND_WAIT_US) $($aw)*
        }

        /// Read NOx algorithm tuning.
        pub $($async)* fn nox_tuning<D: $Delay>(&mut self, delay: &mut D) -> Result<AlgorithmTuning, Error<I2C::Error>> {
            Ok(AlgorithmTuning::from_words(self.get(cmd::NOX_TUNING, delay) $($aw)* ?))
        }

        /// Back up the 8-byte VOC algorithm state.
        pub $($async)* fn voc_state<D: $Delay>(&mut self, delay: &mut D) -> Result<[u8; 8], Error<I2C::Error>> {
            let w: [u16; 4] = self.get(cmd::VOC_STATE, delay) $($aw)* ?;
            let mut out = [0u8; 8];
            words_to_bytes(&w, &mut out);
            Ok(out)
        }

        /// Restore the VOC algorithm state before starting a measurement.
        pub $($async)* fn set_voc_state(&mut self, state: [u8; 8]) -> Result<WaitUs, Error<I2C::Error>> {
            let w = |i: usize| u16::from_be_bytes([state[i], state[i + 1]]);
            self.send(cmd::VOC_STATE, &[w(0), w(2), w(4), w(6)], COMMAND_WAIT_US) $($aw)*
        }

        /// Start a manual 10 s fan cleaning (Measurement-Mode only).
        pub $($async)* fn start_fan_cleaning(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::START_FAN_CLEANING, &[], COMMAND_WAIT_US) $($aw)*
        }

        /// Automatic fan-cleaning interval in seconds (default 604 800).
        pub $($async)* fn auto_cleaning_interval<D: $Delay>(&mut self, delay: &mut D) -> Result<u32, Error<I2C::Error>> {
            Ok(u32_from_words(self.get(cmd::AUTO_CLEANING_INTERVAL, delay) $($aw)* ?))
        }

        /// Set the fan-cleaning interval in seconds (0 disables; volatile).
        pub $($async)* fn set_auto_cleaning_interval(&mut self, seconds: u32) -> Result<WaitUs, Error<I2C::Error>> {
            let args = [(seconds >> 16) as u16, seconds as u16];
            self.send(cmd::AUTO_CLEANING_INTERVAL, &args, COMMAND_WAIT_US) $($aw)*
        }

        /// Device status register (see the `STATUS_*` masks).
        pub $($async)* fn device_status<D: $Delay>(&mut self, delay: &mut D) -> Result<u32, Error<I2C::Error>> {
            Ok(u32_from_words(self.get(cmd::READ_DEVICE_STATUS, delay) $($aw)* ?))
        }

        /// Clear the device status register.
        pub $($async)* fn clear_device_status(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::CLEAR_DEVICE_STATUS, &[], COMMAND_WAIT_US) $($aw)*
        }

        /// Software reset (100 ms; all settings revert to defaults).
        pub $($async)* fn reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::RESET, &[], RESET_WAIT_US) $($aw)* ?;
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(RESET_WAIT_US)
        }

        $($async)* fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.running {
                let pm_only = self.number_concentration
                    && self.mode == Mode::Full
                    && !self.variant.has_rht_voc();
                if core::mem::take(&mut self.next_pm) || pm_only {
                    return self.request_measured_pm_values() $($aw)*;
                }
                return self.request_measured_values() $($aw)*;
            }
            self.next_pm = false;
            match self.mode {
                Mode::Full => self.start_continuous_measurement() $($aw)* ?,
                Mode::RhtGasOnly => self.start_rht_gas_only_measurement() $($aw)* ?,
            };
            Ok(SAMPLE_INTERVAL_US)
        }
    };
}

macro_rules! trait_impls {
    ($Sensor:path, $I2c:path, [$($async:tt)*] [$($aw:tt)*]) => {
        impl<I2C: $I2c> $Sensor for Sen5x<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Starts the [`Mode`](crate::Mode) on the first call (returns
            /// 1 s), then requests the latest values (20 ms).
            $($async)* fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.start() $($aw)*
            }

            /// [`Error::NotReady`] right after starting the mode.
            $($async)* fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                self.read_measured_values() $($aw)*
            }
        }
    };
}

use zigbee_sensirion_common::blocking::Interface;
use zigbee_sensirion_common::{nack_as_not_ready, words_to_bytes};

/// Blocking driver.
#[derive(Debug)]
pub struct Sen5x<I2C> {
    bus: Interface<I2C>,
    variant: Variant,
    mode: Mode,
    running: bool,
    pending: Pending,
    number_concentration: bool,
    next_pm: bool,
}

impl<I2C> Sen5x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Sen5x<I2C> {
    methods!([] [] embedded_hal::delay::DelayNs);
}

trait_impls!(
    zigbee_sensor::blocking::Sensor,
    embedded_hal::i2c::I2c,
    [] []
);

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, AlgorithmTuning, COMMAND_WAIT_US, Measurement, Mode, Pending, RESET_WAIT_US,
        RhtAcceleration, SAMPLE_INTERVAL_US, START_WAIT_US, STOP_WAIT_US, TemperatureCompensation,
        Variant, cmd, convert, convert_pm, u32_from_words, variant_from_name,
    };
    use zigbee_sensirion_common::asynch::Interface;
    use zigbee_sensirion_common::{nack_as_not_ready, words_to_bytes};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Sen5x`].
    #[derive(Debug)]
    pub struct Sen5x<I2C> {
        bus: Interface<I2C>,
        variant: Variant,
        mode: Mode,
        running: bool,
        pending: Pending,
        number_concentration: bool,
        next_pm: bool,
    }

    impl<I2C> Sen5x<I2C> {
        constructors!();
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Sen5x<I2C> {
        methods!([async] [.await] embedded_hal_async::delay::DelayNs);
    }

    trait_impls!(
        zigbee_sensor::asynch::Sensor,
        embedded_hal_async::i2c::I2c,
        [async] [.await]
    );
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
    use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as D};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensirion_common::encode_word;
    use zigbee_sensor::blocking::Sensor;

    // PM 1.2/2.5/3.1/4.0 µg/m³, 45.20 %RH, 22.55 °C (4510/200), VOC 100.0,
    // NOx 1.0.
    const SAMPLE: [u16; 8] = [12, 25, 31, 40, 4520, 4510, 1000, 10];
    // Same PM; NC 10/20/30/unknown/50 #/cm³; typical size 0.45 µm.
    const PM_SAMPLE: [u16; 10] = [12, 25, 31, 40, 100, 200, 300, 0xFFFF, 500, 450];

    fn words(w: &[u16]) -> Vec<u8> {
        w.iter().flat_map(|&x| encode_word(x)).collect()
    }

    fn name(s: &[u8]) -> Vec<u8> {
        let mut bytes = [0u8; 32];
        bytes[..s.len()].copy_from_slice(s);
        let w: Vec<u16> = bytes
            .chunks(2)
            .map(|c| u16::from_be_bytes([c[0], c[1]]))
            .collect();
        words(&w)
    }

    fn nack(t: Transaction) -> Transaction {
        t.with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
    }

    #[test]
    fn probe_and_full_cycle() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0xD0, 0x14]),
            Transaction::read(ADDRESS, name(b"SEN55")),
            Transaction::write(ADDRESS, vec![0x00, 0x21]),
            Transaction::write(ADDRESS, vec![0x03, 0xC4]),
            nack(Transaction::read(ADDRESS, vec![0; 24])),
            Transaction::write(ADDRESS, vec![0x03, 0xC4]),
            Transaction::read(ADDRESS, words(&SAMPLE)),
            Transaction::write(ADDRESS, vec![0x04, 0x13]),
            Transaction::read(ADDRESS, words(&PM_SAMPLE)),
            Transaction::write(ADDRESS, vec![0x03, 0xC4]),
            Transaction::write(ADDRESS, vec![0x01, 0x04]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::blocking_delay_us(20_000)]);
        let mut sen = Sen5x::new(&mut i2c, Variant::Sen50).with_number_concentration(true);
        assert_eq!(sen.probe(&mut delay).unwrap(), Variant::Sen55);
        assert_eq!(sen.start_measurement().unwrap(), 1_000_000);
        assert_eq!(sen.read_measurement(), Err(Error::NotReady));
        assert_eq!(sen.start_measurement().unwrap(), 20_000);
        assert_eq!(sen.read_measurement(), Err(Error::NotReady));
        sen.start_measurement().unwrap();
        let m = sen.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                pm1_0: Some(12),
                pm2_5: Some(25),
                pm4_0: Some(31),
                pm10: Some(40),
                humidity: Some(4520),
                temperature: Some(2255),
                voc_index: Some(100),
                nox_index: Some(1),
                ..Measurement::default()
            }
        );
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(seen.len(), 8);
        assert_eq!(seen[0], Quantity::Temperature(2255));
        assert_eq!(seen[7], Quantity::NoxIndex(1));
        // Every second sample: PM values with number concentrations.
        assert_eq!(sen.start_measurement().unwrap(), 20_000);
        let m = sen.read_measurement().unwrap();
        assert_eq!((m.pm2_5, m.temperature), (Some(25), None));
        seen.clear();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(
            seen[4..],
            [
                Quantity::ParticlesBelow {
                    size_nm: 500,
                    count: 1000
                },
                Quantity::ParticlesBelow {
                    size_nm: 1000,
                    count: 2000
                },
                Quantity::ParticlesBelow {
                    size_nm: 2500,
                    count: 3000
                },
                Quantity::ParticlesBelow {
                    size_nm: 10_000,
                    count: 5000
                },
                Quantity::TypicalParticleSize(450),
            ]
        );
        sen.start_measurement().unwrap();
        assert_eq!(sen.stop_measurement().unwrap(), 200_000);
        delay.done();
        i2c.done();
    }

    #[test]
    fn variants_and_unknown_values() {
        // RHT/gas-only on a SEN54: PM 0xFFFF, NOx 0x7FFF; negative T.
        let rht = [
            0xFFFF,
            0xFFFF,
            0xFFFF,
            0xFFFF,
            3000,
            (-1001i16) as u16,
            1000,
            0x7FFF,
        ];
        let expectations = [
            Transaction::write(ADDRESS, vec![0xD0, 0x14]),
            Transaction::read(ADDRESS, name(b"SEN60")),
            Transaction::write(ADDRESS, vec![0x00, 0x37]),
            Transaction::write(ADDRESS, vec![0x03, 0xC4]),
            Transaction::read(ADDRESS, words(&rht)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::blocking_delay_us(20_000)]);
        let mut sen = Sen5x::new(&mut i2c, Variant::Sen54).with_mode(Mode::RhtGasOnly);
        assert_eq!(
            sen.probe(&mut delay),
            Err(Error::UnexpectedId(u32::from_be_bytes(*b"SEN6")))
        );
        sen.start_measurement().unwrap();
        sen.start_measurement().unwrap();
        let m = sen.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                humidity: Some(3000),
                temperature: Some(-500),
                voc_index: Some(100),
                ..Measurement::default()
            }
        );
        delay.done();
        i2c.done();

        // SEN50 ignores RH/T/VOC/NOx words and has no RHT-only mode.
        assert_eq!(convert(&SAMPLE, Variant::Sen50).humidity, None);
        assert_eq!(convert(&SAMPLE, Variant::Sen50).pm2_5, Some(25));
        assert_eq!(convert(&SAMPLE, Variant::Sen54).nox_index, None);
        let mut i2c = Mock::new(&[]);
        let mut sen = Sen5x::new(&mut i2c, Variant::Sen50).with_mode(Mode::RhtGasOnly);
        assert_eq!(sen.start_measurement(), Err(Error::Unsupported));
        i2c.done();

        // SEN50 with number concentrations: always `Read Measured PM Values`.
        let expectations = [
            Transaction::write(ADDRESS, vec![0x00, 0x21]),
            Transaction::write(ADDRESS, vec![0x04, 0x13]),
            Transaction::read(ADDRESS, words(&PM_SAMPLE)),
            Transaction::write(ADDRESS, vec![0x04, 0x13]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sen = Sen5x::new(&mut i2c, Variant::Sen50).with_number_concentration(true);
        sen.start_measurement().unwrap();
        sen.start_measurement().unwrap();
        let m = sen.read_measurement().unwrap();
        assert_eq!(m.typical_particle_size, Some(450));
        sen.start_measurement().unwrap();
        i2c.done();
    }

    #[test]
    fn configuration_commands() {
        let tuning = AlgorithmTuning::VOC_DEFAULT;
        let mut voc_write = vec![0x60, 0xD0];
        voc_write.extend(words(&[100, 12, 12, 180, 50, 230]));
        let mut temp_write = vec![0x60, 0xB2];
        // -1.50 °C → -300 (scale 200); slope 0.01 → 100; 600 s.
        temp_write.extend(words(&[(-300i16) as u16, 100, 600]));
        let expectations = [
            Transaction::write(ADDRESS, temp_write),
            Transaction::write(ADDRESS, vec![0x60, 0xB2]),
            Transaction::read(ADDRESS, words(&[(-300i16) as u16, 100, 600])),
            Transaction::write(ADDRESS, voc_write),
            Transaction::write(ADDRESS, vec![0x60, 0xE1]),
            Transaction::read(ADDRESS, words(&[1, 12, 12, 720, 50, 230])),
            Transaction::write(ADDRESS, vec![0x60, 0xF7, 0x00, 0x02, 0xE3]),
            Transaction::write(ADDRESS, vec![0x60, 0xC6, 0xFF, 0xFF, 0xAC]),
            Transaction::write(ADDRESS, vec![0x56, 0x07]),
            Transaction::write(ADDRESS, vec![0xD2, 0x06]),
            Transaction::read(ADDRESS, words(&[0x0020, 0x00C0])),
            Transaction::write(ADDRESS, vec![0xD1, 0x00]),
            Transaction::read(ADDRESS, words(&[0x0200])),
            Transaction::write(ADDRESS, vec![0xD3, 0x04]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[
            D::blocking_delay_us(20_000),
            D::blocking_delay_us(20_000),
            D::blocking_delay_us(20_000),
            D::blocking_delay_us(20_000),
        ]);
        let mut sen = Sen5x::new(&mut i2c, Variant::Sen55);
        let comp = TemperatureCompensation {
            offset: -150,
            slope: 100,
            time_constant_s: 600,
        };
        assert_eq!(
            sen.set_temperature_compensation(TemperatureCompensation {
                offset: 20_000,
                ..comp
            }),
            Err(Error::InvalidArgument)
        );
        sen.set_temperature_compensation(comp).unwrap();
        assert_eq!(sen.temperature_compensation(&mut delay).unwrap(), comp);
        sen.set_voc_tuning(tuning).unwrap();
        assert_eq!(
            sen.nox_tuning(&mut delay).unwrap(),
            AlgorithmTuning::NOX_DEFAULT
        );
        sen.set_rht_acceleration(RhtAcceleration::Medium).unwrap();
        sen.set_warm_start(u16::MAX).unwrap();
        sen.start_fan_cleaning().unwrap();
        let status = sen.device_status(&mut delay).unwrap();
        assert_eq!(status, STATUS_FAN_SPEED | STATUS_GAS_SENSOR | STATUS_RHT);
        assert_eq!(sen.firmware_version(&mut delay).unwrap(), 2);
        assert_eq!(sen.reset().unwrap(), 100_000);
        delay.done();
        i2c.done();
    }

    #[test]
    fn async_measure_and_crc() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut bad = words(&SAMPLE);
        bad[5] ^= 1;
        let mut state_write = vec![0x61, 0x81];
        state_write.extend(words(&[0x0102, 0x0304, 0x0506, 0x0708]));
        let expectations = [
            Transaction::write(ADDRESS, vec![0x61, 0x81]),
            Transaction::read(ADDRESS, words(&[0x0102, 0x0304, 0x0506, 0x0708])),
            Transaction::write(ADDRESS, state_write),
            Transaction::write(ADDRESS, vec![0x00, 0x21]),
            Transaction::write(ADDRESS, vec![0x03, 0xC4]),
            Transaction::read(ADDRESS, words(&SAMPLE)),
            Transaction::write(ADDRESS, vec![0x03, 0xC4]),
            Transaction::read(ADDRESS, bad),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[
            D::async_delay_us(20_000),
            D::async_delay_us(1_000_000),
            D::async_delay_us(20_000),
        ]);
        let mut sen = asynch::Sen5x::new(&mut i2c, Variant::Sen55);
        block_on(async {
            let state = sen.voc_state(&mut delay).await.unwrap();
            assert_eq!(state, [1, 2, 3, 4, 5, 6, 7, 8]);
            sen.set_voc_state(state).await.unwrap();
            assert_eq!(sen.measure(&mut delay).await, Err(Error::NotReady));
            let m = sen.measure(&mut delay).await.unwrap();
            assert_eq!(m.voc_index, Some(100));
            sen.start_measurement().await.unwrap();
            assert_eq!(sen.read_measurement().await, Err(Error::Crc));
        });
        delay.done();
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
