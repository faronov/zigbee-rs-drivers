//! ScioSense ENS160 / ENS161 digital metal-oxide multi-gas sensor (I2C or SPI).
//!
//! The chip runs its own algorithms and reports TVOC (ppb), equivalent CO2
//! (ppm) and the UBA air-quality index (1..=5). The ENS161 adds two
//! low-power operating modes and the extended AQI-S index (0..=500).
//!
//! Gas sensing is free-running: once an active operating mode is selected
//! the chip produces a sample every second (STANDARD) and flags it with
//! `NEWDAT`. [`Ens160::start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! enters the configured active mode if needed and returns the time to the
//! first sample (or `0` when already running);
//! [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//! returns [`Error::NotReady`] until a new sample exists.
//!
//! The bus is abstracted by the minimal [`Interface`] trait, implemented for
//! I2C ([`I2cInterface`]) and SPI ([`SpiInterface`]).
//!
//! References: ScioSense "ENS160 Datasheet" v1.3, SC-001224-DS-9
//! (2023-03-29); ScioSense application note "ENS161 operating modes"
//! SC-002489-AN-1 v1.0 (2024-12-12).
//!
//! ```ignore
//! let mut ens = Ens160::new_i2c(i2c, PRIMARY_ADDRESS);
//! let variant = ens.probe()?;
//! ens.set_compensation(2_350, 4_500)?; // 23.50 °C, 45.00 %RH from a T/RH sensor
//! let wait_us = ens.start_measurement()?; // enters STANDARD mode
//! // ... let the Zigbee stack run for wait_us ...
//! let m = ens.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with ADDR pin low.
pub const PRIMARY_ADDRESS: u8 = 0x52;
/// I2C address with ADDR pin high.
pub const SECONDARY_ADDRESS: u8 = 0x53;

/// PART_ID of the ENS160.
pub const PART_ID_ENS160: u16 = 0x0160;
/// PART_ID of the ENS161.
pub const PART_ID_ENS161: u16 = 0x0161;

/// Register addresses (datasheet table 16).
pub mod reg {
    /// Device identity, 2 bytes little-endian.
    pub const PART_ID: u8 = 0x00;
    /// Operating mode.
    pub const OPMODE: u8 = 0x10;
    /// Interrupt pin configuration.
    pub const CONFIG: u8 = 0x11;
    /// Additional system commands (IDLE mode only).
    pub const COMMAND: u8 = 0x12;
    /// Host ambient temperature for compensation (Kelvin × 64).
    pub const TEMP_IN: u8 = 0x13;
    /// Host relative humidity for compensation (%RH × 512).
    pub const RH_IN: u8 = 0x15;
    /// Operating mode status flags.
    pub const DEVICE_STATUS: u8 = 0x20;
    /// UBA air-quality index (bits 2:0).
    pub const DATA_AQI: u8 = 0x21;
    /// TVOC in ppb, 2 bytes.
    pub const DATA_TVOC: u8 = 0x22;
    /// Equivalent CO2 in ppm, 2 bytes.
    pub const DATA_ECO2: u8 = 0x24;
    /// ENS161 only: AQI-S (ScioSense relative index 0..=500), 2 bytes.
    pub const DATA_AQI_S: u8 = 0x26;
    /// Temperature used in calculations, 2 bytes.
    pub const DATA_T: u8 = 0x30;
    /// Relative humidity used in calculations, 2 bytes.
    pub const DATA_RH: u8 = 0x32;
    /// Data integrity checksum (MISR).
    pub const DATA_MISR: u8 = 0x38;
    /// General purpose write registers, 8 bytes.
    pub const GPR_WRITE: u8 = 0x40;
    /// General purpose read registers, 8 bytes.
    pub const GPR_READ: u8 = 0x48;
}

const CMD_NOP: u8 = 0x00;
const CMD_GET_APPVER: u8 = 0x0E;
const CMD_CLRGPR: u8 = 0xCC;

const STATUS_STATER: u8 = 1 << 6;
const STATUS_NEWDAT: u8 = 1 << 1;
const STATUS_NEWGPR: u8 = 1 << 0;

/// Time from entering STANDARD mode to the first sample (1 s update rate).
pub const STANDARD_SAMPLE_US: WaitUs = 1_000_000;
/// ENS161 LOW POWER mode sample interval (1 per minute).
pub const LOW_POWER_SAMPLE_US: WaitUs = 60_000_000;
/// ENS161 ULTRA LOW POWER mode sample interval (1 per 5 minutes).
pub const ULTRA_LOW_POWER_SAMPLE_US: WaitUs = 300_000_000;
/// Wait after `OPMODE = RESET` or leaving DEEP SLEEP. The datasheet gives no
/// figure; 10 ms is a conservative value that covers the internal boot.
pub const BOOT_WAIT_US: WaitUs = 10_000;

/// Part detected by [`Ens160::probe`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// ENS160.
    #[default]
    Ens160,
    /// ENS161 (adds LP/ULP modes and AQI-S).
    Ens161,
}

/// Operating mode (OPMODE register).
///
/// The chip must pass through [`OperatingMode::Idle`] when switching between
/// two active modes. Before leaving an active mode keep it running at least
/// 10 ms (STANDARD), 2 s (LOW POWER) or 25 s (ULTRA LOW POWER) (ENS161
/// application note, section 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperatingMode {
    /// 0x00: deep sleep, 10 µA. Only an OPMODE write is accepted.
    DeepSleep,
    /// 0x01: idle (power-up default), 2 mA; configuration and commands.
    Idle,
    /// 0x02: standard gas sensing, one sample per second.
    Standard,
    /// 0x03: ENS161 only, one sample per minute.
    LowPower,
    /// 0x04: ENS161 only, one sample every five minutes.
    UltraLowPower,
    /// 0xF0: reset; the chip returns to IDLE.
    Reset,
}

impl OperatingMode {
    /// OPMODE register value.
    pub const fn bits(self) -> u8 {
        match self {
            Self::DeepSleep => 0x00,
            Self::Idle => 0x01,
            Self::Standard => 0x02,
            Self::LowPower => 0x03,
            Self::UltraLowPower => 0x04,
            Self::Reset => 0xF0,
        }
    }

    /// `true` for the gas-sensing modes.
    pub const fn is_active(self) -> bool {
        matches!(self, Self::Standard | Self::LowPower | Self::UltraLowPower)
    }

    /// Sample interval of an active mode, `0` otherwise.
    pub const fn sample_interval_us(self) -> WaitUs {
        match self {
            Self::Standard => STANDARD_SAMPLE_US,
            Self::LowPower => LOW_POWER_SAMPLE_US,
            Self::UltraLowPower => ULTRA_LOW_POWER_SAMPLE_US,
            _ => 0,
        }
    }

    fn wait_after_entering(self) -> WaitUs {
        match self {
            Self::Reset => BOOT_WAIT_US,
            mode => mode.sample_interval_us(),
        }
    }
}

/// Validity flag of DEVICE_STATUS bits 3:2 (datasheet table 10).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Validity {
    /// Normal operation.
    Normal,
    /// Warm-up: first 3 minutes after power-on.
    WarmUp,
    /// Initial start-up: first hour of operation in the sensor's life.
    InitialStartUp,
    /// No valid output.
    Invalid,
}

/// DEVICE_STATUS register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceStatus(pub u8);

impl DeviceStatus {
    /// An operating mode is running (STATAS).
    pub const fn running(self) -> bool {
        self.0 & 0x80 != 0
    }

    /// An error was detected, e.g. invalid operating mode (STATER).
    pub const fn error(self) -> bool {
        self.0 & STATUS_STATER != 0
    }

    /// Signal validity.
    pub const fn validity(self) -> Validity {
        match (self.0 >> 2) & 0b11 {
            0 => Validity::Normal,
            1 => Validity::WarmUp,
            2 => Validity::InitialStartUp,
            _ => Validity::Invalid,
        }
    }

    /// New data in the DATA_x registers (cleared when they are read).
    pub const fn new_data(self) -> bool {
        self.0 & STATUS_NEWDAT != 0
    }

    /// New data in the GPR_READ registers.
    pub const fn new_gpr(self) -> bool {
        self.0 & STATUS_NEWGPR != 0
    }
}

/// One gas-sensing sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Signal validity; values during warm-up/start-up are less accurate.
    pub validity: Validity,
    /// UBA air-quality index, 1 (excellent) ..= 5 (unhealthy).
    pub aqi_uba: u8,
    /// TVOC in ppb.
    pub tvoc_ppb: u16,
    /// Equivalent CO2 in ppm (400..=65000).
    pub eco2_ppm: u16,
    /// ENS161 only: AQI-S relative index 0..=500 (100 = typical average).
    /// Not reported through [`Readings`] to avoid a second
    /// `AirQualityIndex` quantity.
    pub aqi_s: Option<u16>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Tvoc(u32::from(self.tvoc_ppb)));
        f(Quantity::Eco2(u32::from(self.eco2_ppm)));
        f(Quantity::AirQualityIndex(u16::from(self.aqi_uba)));
    }
}

/// Application firmware version (COMMAND GET_APPVER).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareVersion {
    /// Major.
    pub major: u8,
    /// Minor.
    pub minor: u8,
    /// Release.
    pub release: u8,
}

/// Raw resistance words from GPR_READ (STANDARD mode). The resistance is
/// `R = 2^(raw / 2048)` Ω (datasheet section 16.2.16).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawResistance {
    /// Hot plate 1 (GPR_READ 0..1).
    pub r1: u16,
    /// Hot plate 4 (GPR_READ 6..7).
    pub r4: u16,
}

/// Encode a temperature in 0.01 °C for TEMP_IN (Kelvin × 64, rounded).
pub fn encode_temperature(centi_celsius: i32) -> u16 {
    let kelvin_centi = (centi_celsius + 27_315).clamp(0, 102_396);
    ((kelvin_centi * 64 + 50) / 100) as u16
}

/// Encode a relative humidity in 0.01 %RH for RH_IN (%RH × 512, rounded).
pub fn encode_humidity(centi_percent: u16) -> u16 {
    let rh = u32::from(centi_percent.min(10_000));
    ((rh * 512 + 50) / 100) as u16
}

/// Decode DATA_T (Kelvin × 64) to 0.01 °C.
pub fn decode_temperature(raw: u16) -> i32 {
    (i32::from(raw) * 100 + 32) / 64 - 27_315
}

/// Decode DATA_RH (%RH × 512) to 0.01 %RH.
pub fn decode_humidity(raw: u16) -> u16 {
    ((u32::from(raw) * 100 + 256) / 512) as u16
}

/// Decode the DEVICE_STATUS..DATA_ECO2 block (0x20..=0x25), optionally
/// followed by DATA_AQI_S (0x26..=0x27, ENS161).
///
/// Returns [`Error::NotReady`] when `NEWDAT` is clear and
/// [`Error::InvalidData`] on STATER or validity "no valid output".
pub fn decode_measurement<E>(block: &[u8]) -> Result<Measurement, Error<E>> {
    if block.len() < 6 {
        return Err(Error::InvalidData);
    }
    let status = DeviceStatus(block[0]);
    if status.error() || status.validity() == Validity::Invalid {
        return Err(Error::InvalidData);
    }
    if !status.new_data() {
        return Err(Error::NotReady);
    }
    Ok(Measurement {
        validity: status.validity(),
        aqi_uba: block[1] & 0x07,
        tvoc_ppb: u16::from_le_bytes([block[2], block[3]]),
        eco2_ppm: u16::from_le_bytes([block[4], block[5]]),
        aqi_s: (block.len() >= 8).then(|| u16::from_le_bytes([block[6], block[7]])),
    })
}

fn variant_from_id<E>(id: u16) -> Result<Variant, Error<E>> {
    match id {
        PART_ID_ENS160 => Ok(Variant::Ens160),
        PART_ID_ENS161 => Ok(Variant::Ens161),
        other => Err(Error::UnexpectedId(u32::from(other))),
    }
}

fn check_mode<E>(variant: Variant, mode: OperatingMode) -> Result<(), Error<E>> {
    if variant == Variant::Ens160
        && matches!(mode, OperatingMode::LowPower | OperatingMode::UltraLowPower)
    {
        return Err(Error::Unsupported);
    }
    Ok(())
}

fn compensation_bytes(centi_celsius: i32, centi_percent: u16) -> [u8; 4] {
    let [t0, t1] = encode_temperature(centi_celsius).to_le_bytes();
    let [h0, h1] = encode_humidity(centi_percent).to_le_bytes();
    [t0, t1, h0, h1]
}

/// Minimal register interface of the ENS160 (blocking).
pub trait Interface {
    /// Transport error.
    type Error;
    /// Read consecutive registers starting at `reg`.
    fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write consecutive registers starting at `reg` (at most 8 bytes).
    fn write_registers(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error>;
}

/// I2C transport.
pub struct I2cInterface<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Wrap an I2C bus; `address` is [`PRIMARY_ADDRESS`] or [`SECONDARY_ADDRESS`].
    pub const fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Give the bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }
}

const MAX_WRITE: usize = 8;

fn i2c_frame(reg: u8, data: &[u8]) -> ([u8; MAX_WRITE + 1], usize) {
    let n = data.len().min(MAX_WRITE);
    let mut frame = [0u8; MAX_WRITE + 1];
    frame[0] = reg;
    frame[1..=n].copy_from_slice(&data[..n]);
    (frame, n + 1)
}

impl<I2C: embedded_hal::i2c::I2c> Interface for I2cInterface<I2C> {
    type Error = I2C::Error;

    fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[reg], buf)
    }

    fn write_registers(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        let (frame, len) = i2c_frame(reg, data);
        self.i2c.write(self.address, &frame[..len])
    }
}

/// SPI transport (mode 0 or 3, ≤ 10 MHz). CSn must be low at power-up for
/// the chip to select SPI.
///
/// The address byte is `reg << 1 | R/W` (1 = read). Reads auto-increment;
/// writes do not, so every data byte is preceded by its own address byte
/// (datasheet section 15.2).
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI> SpiInterface<SPI> {
    /// Wrap an SPI device (bus + chip select).
    pub const fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Give the device back.
    pub fn release(self) -> SPI {
        self.spi
    }
}

/// SPI read address byte.
pub const fn spi_read_address(reg: u8) -> u8 {
    (reg << 1) | 1
}

/// SPI write burst: `[(reg << 1), d0, ((reg + 1) << 1), d1, ...]`.
pub fn spi_write_frame(reg: u8, data: &[u8]) -> ([u8; 2 * MAX_WRITE], usize) {
    let n = data.len().min(MAX_WRITE);
    let mut frame = [0u8; 2 * MAX_WRITE];
    for (i, byte) in data[..n].iter().enumerate() {
        frame[2 * i] = reg.wrapping_add(i as u8) << 1;
        frame[2 * i + 1] = *byte;
    }
    (frame, 2 * n)
}

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.spi.transaction(&mut [
            Operation::Write(&[spi_read_address(reg)]),
            Operation::Read(buf),
        ])
    }

    fn write_registers(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        let (frame, len) = spi_write_frame(reg, data);
        self.spi.write(&frame[..len])
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver over any [`Interface`]; no bus traffic.
        pub const fn new(iface: IFACE) -> Self {
            Self {
                iface,
                variant: Variant::Ens160,
                mode: OperatingMode::Idle,
                active_mode: OperatingMode::Standard,
            }
        }

        /// Declare the part without probing (default ENS160).
        pub const fn with_variant(mut self, variant: Variant) -> Self {
            self.variant = variant;
            self
        }

        /// Active mode entered by `start_measurement` (default STANDARD).
        /// LOW POWER and ULTRA LOW POWER require an ENS161.
        pub const fn with_active_mode(mut self, mode: OperatingMode) -> Self {
            self.active_mode = mode;
            self
        }

        /// Part variant (set by `probe` or `with_variant`).
        pub const fn variant(&self) -> Variant {
            self.variant
        }

        /// Operating mode last written by this driver.
        pub const fn mode(&self) -> OperatingMode {
            self.mode
        }

        /// Give the interface back.
        pub fn release(self) -> IFACE {
            self.iface
        }
    };
}

/// Blocking driver.
pub struct Ens160<IFACE> {
    iface: IFACE,
    variant: Variant,
    mode: OperatingMode,
    active_mode: OperatingMode,
}

impl<IFACE> Ens160<IFACE> {
    constructors!();
}

impl<I2C> Ens160<I2cInterface<I2C>> {
    /// Driver on an I2C bus.
    pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
        Self::new(I2cInterface::new(i2c, address))
    }
}

impl<SPI> Ens160<SpiInterface<SPI>> {
    /// Driver on an SPI device.
    pub const fn new_spi(spi: SPI) -> Self {
        Self::new(SpiInterface::new(spi))
    }
}

impl<IFACE: Interface> Ens160<IFACE> {
    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
        self.iface.read_registers(reg, buf).map_err(Error::bus)
    }

    fn write(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<IFACE::Error>> {
        self.iface.write_registers(reg, data).map_err(Error::bus)
    }

    /// Read PART_ID, record and return the variant.
    pub fn probe(&mut self) -> Result<Variant, Error<IFACE::Error>> {
        let mut id = [0u8; 2];
        self.read(reg::PART_ID, &mut id)?;
        self.variant = variant_from_id(u16::from_le_bytes(id))?;
        Ok(self.variant)
    }

    /// Write OPMODE. Returns the time until the chip is usable in that mode
    /// (first sample for active modes, boot time for reset).
    pub fn set_mode(&mut self, mode: OperatingMode) -> Result<WaitUs, Error<IFACE::Error>> {
        check_mode(self.variant, mode)?;
        self.write(reg::OPMODE, &[mode.bits()])?;
        self.mode = match mode {
            OperatingMode::Reset => OperatingMode::Idle,
            mode => mode,
        };
        Ok(mode.wait_after_entering())
    }

    /// Write CONFIG (interrupt pin): bit 6 INTPOL (1 = active high),
    /// bit 5 INT_CFG (1 = push-pull), bit 3 INTGPR, bit 1 INTDAT, bit 0 INTEN.
    pub fn set_interrupt_config(&mut self, config: u8) -> Result<(), Error<IFACE::Error>> {
        self.write(reg::CONFIG, &[config])
    }

    /// Feed ambient temperature (0.01 °C) and humidity (0.01 %RH) for the
    /// on-chip compensation (TEMP_IN + RH_IN in one burst).
    pub fn set_compensation(
        &mut self,
        centi_celsius: i32,
        centi_percent: u16,
    ) -> Result<(), Error<IFACE::Error>> {
        self.write(
            reg::TEMP_IN,
            &compensation_bytes(centi_celsius, centi_percent),
        )
    }

    /// Read back the temperature (0.01 °C) and humidity (0.01 %RH) used by
    /// the algorithms (DATA_T, DATA_RH).
    pub fn read_compensation(&mut self) -> Result<(i32, u16), Error<IFACE::Error>> {
        let mut buf = [0u8; 4];
        self.read(reg::DATA_T, &mut buf)?;
        Ok((
            decode_temperature(u16::from_le_bytes([buf[0], buf[1]])),
            decode_humidity(u16::from_le_bytes([buf[2], buf[3]])),
        ))
    }

    /// Read DEVICE_STATUS.
    pub fn read_status(&mut self) -> Result<DeviceStatus, Error<IFACE::Error>> {
        let mut buf = [0u8; 1];
        self.read(reg::DEVICE_STATUS, &mut buf)?;
        Ok(DeviceStatus(buf[0]))
    }

    /// Issue GET_APPVER (IDLE mode only). Read the result with
    /// [`Self::read_firmware_version`].
    pub fn request_firmware_version(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write(reg::COMMAND, &[CMD_NOP])?;
        self.write(reg::COMMAND, &[CMD_CLRGPR])?;
        self.write(reg::COMMAND, &[CMD_GET_APPVER])
    }

    /// Read the GET_APPVER result; [`Error::NotReady`] until NEWGPR is set.
    pub fn read_firmware_version(&mut self) -> Result<FirmwareVersion, Error<IFACE::Error>> {
        if !self.read_status()?.new_gpr() {
            return Err(Error::NotReady);
        }
        let mut buf = [0u8; 3];
        self.read(reg::GPR_READ + 4, &mut buf)?;
        Ok(FirmwareVersion {
            major: buf[0],
            minor: buf[1],
            release: buf[2],
        })
    }

    /// Read the raw hot-plate resistances (valid in STANDARD mode).
    pub fn read_raw_resistance(&mut self) -> Result<RawResistance, Error<IFACE::Error>> {
        let mut buf = [0u8; 8];
        self.read(reg::GPR_READ, &mut buf)?;
        Ok(RawResistance {
            r1: u16::from_le_bytes([buf[0], buf[1]]),
            r4: u16::from_le_bytes([buf[6], buf[7]]),
        })
    }

    fn block_len(&self) -> usize {
        match self.variant {
            Variant::Ens160 => 6,
            Variant::Ens161 => 8,
        }
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Ens160<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    /// Enters the configured active mode if the chip is not sensing yet and
    /// returns the time to the first sample; returns `0` while running.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        if self.mode.is_active() {
            return Ok(0);
        }
        if self.mode == OperatingMode::DeepSleep {
            self.set_mode(OperatingMode::Idle)?;
        }
        self.set_mode(self.active_mode)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        let mut block = [0u8; 8];
        let len = self.block_len();
        self.read(reg::DEVICE_STATUS, &mut block[..len])?;
        decode_measurement(&block[..len])
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Ens160<IFACE> {
    type BusError = IFACE::Error;

    /// DEEP SLEEP (10 µA). Configuration registers are kept.
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.set_mode(OperatingMode::DeepSleep).map(|_| ())
    }

    /// Back to IDLE; returns [`BOOT_WAIT_US`].
    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.set_mode(OperatingMode::Idle)?;
        Ok(BOOT_WAIT_US)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_CLRGPR, CMD_GET_APPVER, CMD_NOP, DeviceStatus, FirmwareVersion, I2cInterface,
        Measurement, OperatingMode, RawResistance, SpiInterface, Variant, check_mode,
        compensation_bytes, decode_humidity, decode_measurement, decode_temperature, i2c_frame,
        reg, spi_read_address, spi_write_frame, variant_from_id,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Interface`].
    #[allow(async_fn_in_trait)]
    pub trait Interface {
        /// Transport error.
        type Error;
        /// Read consecutive registers starting at `reg`.
        async fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
        /// Write consecutive registers starting at `reg` (at most 8 bytes).
        async fn write_registers(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error>;
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Interface for I2cInterface<I2C> {
        type Error = I2C::Error;

        async fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            self.i2c.write_read(self.address, &[reg], buf).await
        }

        async fn write_registers(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
            let (frame, len) = i2c_frame(reg, data);
            self.i2c.write(self.address, &frame[..len]).await
        }
    }

    impl<SPI: embedded_hal_async::spi::SpiDevice> Interface for SpiInterface<SPI> {
        type Error = SPI::Error;

        async fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            use embedded_hal_async::spi::Operation;
            self.spi
                .transaction(&mut [
                    Operation::Write(&[spi_read_address(reg)]),
                    Operation::Read(buf),
                ])
                .await
        }

        async fn write_registers(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
            let (frame, len) = spi_write_frame(reg, data);
            self.spi.write(&frame[..len]).await
        }
    }

    /// Async twin of [`super::Ens160`].
    pub struct Ens160<IFACE> {
        iface: IFACE,
        variant: Variant,
        mode: OperatingMode,
        active_mode: OperatingMode,
    }

    impl<IFACE> Ens160<IFACE> {
        constructors!();
    }

    impl<I2C> Ens160<I2cInterface<I2C>> {
        /// Driver on an I2C bus.
        pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
            Self::new(I2cInterface::new(i2c, address))
        }
    }

    impl<SPI> Ens160<SpiInterface<SPI>> {
        /// Driver on an SPI device.
        pub const fn new_spi(spi: SPI) -> Self {
            Self::new(SpiInterface::new(spi))
        }
    }

    impl<IFACE: Interface> Ens160<IFACE> {
        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
            self.iface
                .read_registers(reg, buf)
                .await
                .map_err(Error::bus)
        }

        async fn write(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<IFACE::Error>> {
            self.iface
                .write_registers(reg, data)
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Ens160::probe`].
        pub async fn probe(&mut self) -> Result<Variant, Error<IFACE::Error>> {
            let mut id = [0u8; 2];
            self.read(reg::PART_ID, &mut id).await?;
            self.variant = variant_from_id(u16::from_le_bytes(id))?;
            Ok(self.variant)
        }

        /// See [`super::Ens160::set_mode`].
        pub async fn set_mode(
            &mut self,
            mode: OperatingMode,
        ) -> Result<WaitUs, Error<IFACE::Error>> {
            check_mode(self.variant, mode)?;
            self.write(reg::OPMODE, &[mode.bits()]).await?;
            self.mode = match mode {
                OperatingMode::Reset => OperatingMode::Idle,
                mode => mode,
            };
            Ok(mode.wait_after_entering())
        }

        /// See [`super::Ens160::set_interrupt_config`].
        pub async fn set_interrupt_config(
            &mut self,
            config: u8,
        ) -> Result<(), Error<IFACE::Error>> {
            self.write(reg::CONFIG, &[config]).await
        }

        /// See [`super::Ens160::set_compensation`].
        pub async fn set_compensation(
            &mut self,
            centi_celsius: i32,
            centi_percent: u16,
        ) -> Result<(), Error<IFACE::Error>> {
            self.write(
                reg::TEMP_IN,
                &compensation_bytes(centi_celsius, centi_percent),
            )
            .await
        }

        /// See [`super::Ens160::read_compensation`].
        pub async fn read_compensation(&mut self) -> Result<(i32, u16), Error<IFACE::Error>> {
            let mut buf = [0u8; 4];
            self.read(reg::DATA_T, &mut buf).await?;
            Ok((
                decode_temperature(u16::from_le_bytes([buf[0], buf[1]])),
                decode_humidity(u16::from_le_bytes([buf[2], buf[3]])),
            ))
        }

        /// See [`super::Ens160::read_status`].
        pub async fn read_status(&mut self) -> Result<DeviceStatus, Error<IFACE::Error>> {
            let mut buf = [0u8; 1];
            self.read(reg::DEVICE_STATUS, &mut buf).await?;
            Ok(DeviceStatus(buf[0]))
        }

        /// See [`super::Ens160::request_firmware_version`].
        pub async fn request_firmware_version(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write(reg::COMMAND, &[CMD_NOP]).await?;
            self.write(reg::COMMAND, &[CMD_CLRGPR]).await?;
            self.write(reg::COMMAND, &[CMD_GET_APPVER]).await
        }

        /// See [`super::Ens160::read_firmware_version`].
        pub async fn read_firmware_version(
            &mut self,
        ) -> Result<FirmwareVersion, Error<IFACE::Error>> {
            if !self.read_status().await?.new_gpr() {
                return Err(Error::NotReady);
            }
            let mut buf = [0u8; 3];
            self.read(reg::GPR_READ + 4, &mut buf).await?;
            Ok(FirmwareVersion {
                major: buf[0],
                minor: buf[1],
                release: buf[2],
            })
        }

        /// See [`super::Ens160::read_raw_resistance`].
        pub async fn read_raw_resistance(&mut self) -> Result<RawResistance, Error<IFACE::Error>> {
            let mut buf = [0u8; 8];
            self.read(reg::GPR_READ, &mut buf).await?;
            Ok(RawResistance {
                r1: u16::from_le_bytes([buf[0], buf[1]]),
                r4: u16::from_le_bytes([buf[6], buf[7]]),
            })
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Ens160<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            if self.mode.is_active() {
                return Ok(0);
            }
            if self.mode == OperatingMode::DeepSleep {
                self.set_mode(OperatingMode::Idle).await?;
            }
            self.set_mode(self.active_mode).await
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            let mut block = [0u8; 8];
            let len = match self.variant {
                Variant::Ens160 => 6,
                Variant::Ens161 => 8,
            };
            self.read(reg::DEVICE_STATUS, &mut block[..len]).await?;
            decode_measurement(&block[..len])
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Ens160<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.set_mode(OperatingMode::DeepSleep).await.map(|_| ())
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.set_mode(OperatingMode::Idle).await?;
            Ok(super::BOOT_WAIT_US)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTx};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = PRIMARY_ADDRESS;

    #[test]
    fn compensation_encoding_matches_datasheet_examples() {
        // Section 16.2.5/16.2.6: 25 °C → 0x4A8A, 50 %RH → 0x6400.
        assert_eq!(encode_temperature(2_500), 0x4A8A);
        assert_eq!(encode_humidity(5_000), 0x6400);
        assert_eq!(decode_temperature(0x4A8A), 2_501);
        assert_eq!(decode_humidity(0x6400), 5_000);
        assert_eq!(compensation_bytes(2_500, 5_000), [0x8A, 0x4A, 0x00, 0x64]);
    }

    #[test]
    fn probe_detects_variants_and_rejects_others() {
        let expectations = [
            I2cTx::write_read(A, vec![0x00], vec![0x60, 0x01]),
            I2cTx::write_read(A, vec![0x00], vec![0x61, 0x01]),
            I2cTx::write_read(A, vec![0x00], vec![0x81, 0x00]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ens = Ens160::new_i2c(&mut i2c, A);
        assert_eq!(ens.probe().unwrap(), Variant::Ens160);
        assert_eq!(ens.probe().unwrap(), Variant::Ens161);
        assert_eq!(ens.probe(), Err(Error::UnexpectedId(0x0081)));
        i2c.done();
    }

    #[test]
    fn start_enters_standard_then_reads_sample() {
        let expectations = [
            I2cTx::write(A, vec![0x13, 0x8A, 0x4A, 0x00, 0x64]),
            I2cTx::write(A, vec![0x10, 0x02]),
            // NEWDAT clear: not ready.
            I2cTx::write_read(A, vec![0x20], vec![0x80, 0, 0, 0, 0, 0]),
            // STATAS | validity warm-up | NEWDAT, AQI 2, TVOC 0x0123, eCO2 0x0258 (600).
            I2cTx::write_read(A, vec![0x20], vec![0x86, 0x02, 0x23, 0x01, 0x58, 0x02]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ens = Ens160::new_i2c(&mut i2c, A);
        ens.set_compensation(2_500, 5_000).unwrap();
        assert_eq!(ens.start_measurement().unwrap(), STANDARD_SAMPLE_US);
        assert_eq!(ens.start_measurement().unwrap(), 0);
        assert_eq!(ens.read_measurement(), Err(Error::NotReady));
        let m = ens.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                validity: Validity::WarmUp,
                aqi_uba: 2,
                tvoc_ppb: 0x0123,
                eco2_ppm: 600,
                aqi_s: None,
            }
        );
        let mut seen = vec![];
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(
            seen,
            vec![
                Quantity::Tvoc(0x0123),
                Quantity::Eco2(600),
                Quantity::AirQualityIndex(2)
            ]
        );
        i2c.done();
    }

    #[test]
    fn error_and_invalid_status_are_reported() {
        let error = [0xC2, 1, 0, 0, 0x90, 0x01];
        let invalid = [0x8E, 1, 0, 0, 0x90, 0x01];
        assert_eq!(decode_measurement::<()>(&error), Err(Error::InvalidData));
        assert_eq!(decode_measurement::<()>(&invalid), Err(Error::InvalidData));
    }

    #[test]
    fn ens161_low_power_mode_and_aqi_s() {
        let expectations = [
            I2cTx::write_read(A, vec![0x00], vec![0x61, 0x01]),
            I2cTx::write(A, vec![0x10, 0x03]),
            I2cTx::write_read(
                A,
                vec![0x20],
                vec![0x82, 0x01, 0x10, 0x00, 0x90, 0x01, 0x64, 0x00],
            ),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ens = Ens160::new_i2c(&mut i2c, A).with_active_mode(OperatingMode::LowPower);
        ens.probe().unwrap();
        assert_eq!(ens.start_measurement().unwrap(), LOW_POWER_SAMPLE_US);
        let m = ens.read_measurement().unwrap();
        assert_eq!(m.aqi_s, Some(100));
        assert_eq!(m.eco2_ppm, 400);
        i2c.done();
    }

    #[test]
    fn ens160_rejects_ens161_modes() {
        let mut i2c = I2cMock::new(&[]);
        let mut ens = Ens160::new_i2c(&mut i2c, A);
        assert_eq!(
            ens.set_mode(OperatingMode::UltraLowPower),
            Err(Error::Unsupported)
        );
        i2c.done();
    }

    #[test]
    fn sleep_wake_and_restart() {
        let expectations = [
            I2cTx::write(A, vec![0x10, 0x02]),
            I2cTx::write(A, vec![0x10, 0x00]),
            I2cTx::write(A, vec![0x10, 0x01]),
            I2cTx::write(A, vec![0x10, 0x02]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ens = Ens160::new_i2c(&mut i2c, A);
        ens.start_measurement().unwrap();
        ens.sleep().unwrap();
        assert_eq!(ens.mode(), OperatingMode::DeepSleep);
        // Leaving deep sleep goes through IDLE before STANDARD.
        assert_eq!(ens.start_measurement().unwrap(), STANDARD_SAMPLE_US);
        i2c.done();
    }

    #[test]
    fn firmware_version_waits_for_newgpr() {
        let expectations = [
            I2cTx::write(A, vec![0x12, 0x00]),
            I2cTx::write(A, vec![0x12, 0xCC]),
            I2cTx::write(A, vec![0x12, 0x0E]),
            I2cTx::write_read(A, vec![0x20], vec![0x00]),
            I2cTx::write_read(A, vec![0x20], vec![0x01]),
            I2cTx::write_read(A, vec![0x4C], vec![5, 4, 6]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ens = Ens160::new_i2c(&mut i2c, A);
        ens.request_firmware_version().unwrap();
        assert_eq!(ens.read_firmware_version(), Err(Error::NotReady));
        assert_eq!(
            ens.read_firmware_version().unwrap(),
            FirmwareVersion {
                major: 5,
                minor: 4,
                release: 6
            }
        );
        i2c.done();
    }

    #[test]
    fn spi_framing() {
        let expectations = [
            // Read PART_ID: address byte 0x00 << 1 | 1.
            SpiTx::transaction_start(),
            SpiTx::write(0x01),
            SpiTx::read_vec(vec![0x60, 0x01]),
            SpiTx::transaction_end(),
            // Write TEMP_IN/RH_IN: address byte before every data byte.
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x26, 0x8A, 0x28, 0x4A, 0x2A, 0x00, 0x2C, 0x64]),
            SpiTx::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut ens = Ens160::new_spi(&mut spi);
        assert_eq!(ens.probe().unwrap(), Variant::Ens160);
        ens.set_compensation(2_500, 5_000).unwrap();
        spi.done();
    }

    #[test]
    fn async_measure_awaits_first_sample() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            I2cTx::write(A, vec![0x10, 0x02]),
            I2cTx::write_read(A, vec![0x20], vec![0x82, 0x01, 0x05, 0x00, 0x90, 0x01]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ens = asynch::Ens160::new_i2c(&mut i2c, A);
        let mut delay = RecordingDelay(0);
        let m = block_on(ens.measure(&mut delay)).unwrap();
        assert_eq!(m.tvoc_ppb, 5);
        assert_eq!(m.validity, Validity::Normal);
        assert_eq!(delay.0, STANDARD_SAMPLE_US);
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
