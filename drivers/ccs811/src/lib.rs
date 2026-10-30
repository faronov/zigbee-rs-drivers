//! ams (now ScioSense) CCS811 metal-oxide gas sensor: equivalent CO2 (ppm)
//! and TVOC (ppb) over I2C.
//!
//! After power-on the CCS811 sits in its boot loader; [`Ccs811::start_app`]
//! moves it to application mode. In application mode the chip measures on
//! its own at the rate of the selected [`DriveMode`] and flags each sample
//! with `DATA_READY`.
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! selects the configured drive mode if the chip is idle and returns the
//! sample interval, or `0` when it is already running;
//! [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//! returns [`Error::NotReady`] until a new sample exists.
//!
//! **nWAKE**: the pin is active low and must be held low for the whole I2C
//! transaction (assert ≥ 50 µs before, de-assert ≥ 20 µs between
//! transactions). Tie it to GND unless you need the 19 µA sleep current;
//! the pin is not managed by this driver.
//!
//! Reference: ams "CCS811 Ultra-Low Power Digital Gas Sensor for Monitoring
//! Indoor Air Quality" datasheet v1-00 (2016-Dec-23).
//!
//! ```ignore
//! let mut ccs = Ccs811::new(i2c, PRIMARY_ADDRESS);
//! ccs.probe()?;
//! let wait_us = ccs.start_app()?;
//! // ... wait_us later:
//! ccs.set_environment(2_350, 4_500)?; // from a T/RH sensor
//! let wait_us = ccs.start_measurement()?; // drive mode 1 (1 s)
//! // ... wait_us later:
//! let m = ccs.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with ADDR low.
pub const PRIMARY_ADDRESS: u8 = 0x5A;
/// I2C address with ADDR high.
pub const SECONDARY_ADDRESS: u8 = 0x5B;
/// HW_ID register value.
pub const HW_ID: u8 = 0x81;

/// Register (mailbox) addresses.
pub mod reg {
    /// Status (1 byte).
    pub const STATUS: u8 = 0x00;
    /// Measurement mode and interrupt configuration (1 byte).
    pub const MEAS_MODE: u8 = 0x01;
    /// eCO2, TVOC, STATUS, ERROR_ID, RAW_DATA (8 bytes).
    pub const ALG_RESULT_DATA: u8 = 0x02;
    /// Raw current and ADC reading (2 bytes).
    pub const RAW_DATA: u8 = 0x03;
    /// Humidity and temperature for compensation (4 bytes).
    pub const ENV_DATA: u8 = 0x05;
    /// NTC reference and thermistor voltages (4 bytes).
    pub const NTC: u8 = 0x06;
    /// eCO2 interrupt thresholds (5 bytes).
    pub const THRESHOLDS: u8 = 0x10;
    /// Encoded algorithm baseline (2 bytes).
    pub const BASELINE: u8 = 0x11;
    /// Hardware ID, 0x81.
    pub const HW_ID: u8 = 0x20;
    /// Hardware version, 0x1X.
    pub const HW_VERSION: u8 = 0x21;
    /// Boot loader firmware version (2 bytes).
    pub const FW_BOOT_VERSION: u8 = 0x23;
    /// Application firmware version (2 bytes).
    pub const FW_APP_VERSION: u8 = 0x24;
    /// Error source flags.
    pub const ERROR_ID: u8 = 0xE0;
    /// Boot loader: leave boot mode (write without data).
    pub const APP_START: u8 = 0xF4;
    /// Software reset (4-byte key).
    pub const SW_RESET: u8 = 0xFF;
}

const SW_RESET_KEY: [u8; 4] = [0x11, 0xE5, 0x72, 0x8A];

/// Time until the chip answers after power-on (tSTART max).
pub const POWER_ON_US: WaitUs = 20_000;
/// Time until the chip answers after SW_RESET / nRESET (tSTART max).
pub const RESET_US: WaitUs = 2_000;
/// Wait after APP_START. Not specified in the datasheet; 2 ms is a
/// conservative assumption (application start is a jump in firmware).
pub const APP_START_US: WaitUs = 2_000;

/// STATUS register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status(pub u8);

impl Status {
    /// Application mode (1) or boot mode (0).
    pub const fn app_mode(self) -> bool {
        self.0 & 0x80 != 0
    }

    /// A valid application firmware is present.
    pub const fn app_valid(self) -> bool {
        self.0 & 0x10 != 0
    }

    /// New sample in ALG_RESULT_DATA (cleared by reading it).
    pub const fn data_ready(self) -> bool {
        self.0 & 0x08 != 0
    }

    /// Error; details in ERROR_ID.
    pub const fn error(self) -> bool {
        self.0 & 0x01 != 0
    }
}

/// ERROR_ID register flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ErrorFlags(pub u8);

impl ErrorFlags {
    /// Write to an invalid register.
    pub const fn write_reg_invalid(self) -> bool {
        self.0 & 0x01 != 0
    }
    /// Read from an invalid register.
    pub const fn read_reg_invalid(self) -> bool {
        self.0 & 0x02 != 0
    }
    /// Unsupported MEAS_MODE value.
    pub const fn meas_mode_invalid(self) -> bool {
        self.0 & 0x04 != 0
    }
    /// Sensor resistance reached the maximum range.
    pub const fn max_resistance(self) -> bool {
        self.0 & 0x08 != 0
    }
    /// Heater current out of range.
    pub const fn heater_fault(self) -> bool {
        self.0 & 0x10 != 0
    }
    /// Heater voltage not applied correctly.
    pub const fn heater_supply(self) -> bool {
        self.0 & 0x20 != 0
    }
}

/// Drive mode (MEAS_MODE bits 6:4).
///
/// When moving to a mode with a lower sample rate (e.g. 1 → 3), stay in
/// [`DriveMode::Idle`] for at least 10 minutes first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveMode {
    /// Mode 0: idle, no measurements (0.034 mW).
    Idle,
    /// Mode 1: constant power, sample every second (46 mW).
    Every1s,
    /// Mode 2: pulse heating, sample every 10 s (7 mW).
    Every10s,
    /// Mode 3: low-power pulse heating, sample every 60 s (1.2 mW).
    Every60s,
    /// Mode 4: raw data only every 250 ms; ALG_RESULT_DATA is not updated.
    Raw250ms,
}

impl DriveMode {
    /// MEAS_MODE DRIVE_MODE field value.
    pub const fn bits(self) -> u8 {
        match self {
            Self::Idle => 0,
            Self::Every1s => 1,
            Self::Every10s => 2,
            Self::Every60s => 3,
            Self::Raw250ms => 4,
        }
    }

    /// Sample interval, `0` for idle.
    pub const fn interval_us(self) -> WaitUs {
        match self {
            Self::Idle => 0,
            Self::Every1s => 1_000_000,
            Self::Every10s => 10_000_000,
            Self::Every60s => 60_000_000,
            Self::Raw250ms => 250_000,
        }
    }
}

/// MEAS_MODE value: drive mode, INT_DATARDY (bit 3), INT_THRESH (bit 2).
pub const fn meas_mode(mode: DriveMode, int_data_ready: bool, int_threshold: bool) -> u8 {
    (mode.bits() << 4) | ((int_data_ready as u8) << 3) | ((int_threshold as u8) << 2)
}

/// Raw sensor data (RAW_DATA).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawData {
    /// Sensor current in µA (0..=63).
    pub current_ua: u8,
    /// Voltage across the sensor, 1023 = 1.65 V.
    pub adc: u16,
}

/// Decode the 2-byte RAW_DATA block.
pub const fn decode_raw(bytes: [u8; 2]) -> RawData {
    RawData {
        current_ua: bytes[0] >> 2,
        adc: (((bytes[0] & 0x03) as u16) << 8) | bytes[1] as u16,
    }
}

/// One algorithm result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Equivalent CO2, 400..=8192 ppm (clipped).
    pub eco2_ppm: u16,
    /// TVOC, 0..=1187 ppb (clipped).
    pub tvoc_ppb: u16,
    /// Raw data of the same sample.
    pub raw: RawData,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Eco2(u32::from(self.eco2_ppm)));
        f(Quantity::Tvoc(u32::from(self.tvoc_ppb)));
    }
}

/// Decode the 8-byte ALG_RESULT_DATA block.
///
/// [`Error::InvalidData`] when the embedded STATUS has ERROR set (read
/// [`Ccs811::error_flags`] for details), [`Error::NotReady`] when
/// DATA_READY is clear.
pub fn decode_result<E>(block: &[u8; 8]) -> Result<Measurement, Error<E>> {
    let status = Status(block[4]);
    if status.error() {
        return Err(Error::InvalidData);
    }
    if !status.data_ready() {
        return Err(Error::NotReady);
    }
    Ok(Measurement {
        eco2_ppm: u16::from_be_bytes([block[0], block[1]]),
        tvoc_ppb: u16::from_be_bytes([block[2], block[3]]),
        raw: decode_raw([block[6], block[7]]),
    })
}

/// Encode ENV_DATA: humidity (0.01 %RH) and temperature (0.01 °C), both in
/// 1/512 units, temperature offset by +25 °C, big-endian.
pub fn encode_environment(centi_celsius: i32, centi_percent: u16) -> [u8; 4] {
    let rh = u32::from(centi_percent.min(10_000));
    let rh = ((rh * 512 + 50) / 100) as u16;
    let t = (centi_celsius.clamp(-2_500, 10_000) + 2_500) as u32;
    let t = ((t * 512 + 50) / 100) as u16;
    let [h0, h1] = rh.to_be_bytes();
    let [t0, t1] = t.to_be_bytes();
    [h0, h1, t0, t1]
}

/// Encode THRESHOLDS: low→medium and medium→high eCO2 thresholds (ppm,
/// defaults 1500 / 2500) and hysteresis (ppm, default 50).
pub fn encode_thresholds(low_to_medium: u16, medium_to_high: u16, hysteresis: u8) -> [u8; 5] {
    let [a, b] = low_to_medium.to_be_bytes();
    let [c, d] = medium_to_high.to_be_bytes();
    [a, b, c, d, hysteresis]
}

/// Firmware version `major.minor.trivial`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareVersion {
    /// Major.
    pub major: u8,
    /// Minor.
    pub minor: u8,
    /// Trivial.
    pub trivial: u8,
}

/// Decode FW_BOOT_VERSION / FW_APP_VERSION.
pub const fn decode_version(bytes: [u8; 2]) -> FirmwareVersion {
    FirmwareVersion {
        major: bytes[0] >> 4,
        minor: bytes[0] & 0x0F,
        trivial: bytes[1],
    }
}

fn check_hw_id<E>(id: u8) -> Result<(), Error<E>> {
    if id == HW_ID {
        Ok(())
    } else {
        Err(Error::UnexpectedId(u32::from(id)))
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic. Default active mode: 1 s.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                mode: DriveMode::Idle,
                active_mode: DriveMode::Every1s,
                int_data_ready: false,
                int_threshold: false,
            }
        }

        /// Drive mode selected by `start_measurement` and `wake`.
        pub const fn with_active_mode(mut self, mode: DriveMode) -> Self {
            self.active_mode = mode;
            self
        }

        /// Interrupt bits written together with the drive mode.
        pub const fn with_interrupts(mut self, data_ready: bool, threshold: bool) -> Self {
            self.int_data_ready = data_ready;
            self.int_threshold = threshold;
            self
        }

        /// Drive mode last written by this driver.
        pub const fn mode(&self) -> DriveMode {
            self.mode
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct Ccs811<I2C> {
    i2c: I2C,
    address: u8,
    mode: DriveMode,
    active_mode: DriveMode,
    int_data_ready: bool,
    int_threshold: bool,
}

impl<I2C> Ccs811<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Ccs811<I2C> {
    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(self.address, &[reg], buf)
            .map_err(Error::bus)
    }

    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, bytes).map_err(Error::bus)
    }

    /// Check HW_ID (0x81) and return HW_VERSION (0x1X).
    pub fn probe(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut id = [0u8; 2];
        self.read(reg::HW_ID, &mut id)?;
        check_hw_id(id[0])?;
        Ok(id[1])
    }

    /// Read STATUS.
    pub fn status(&mut self) -> Result<Status, Error<I2C::Error>> {
        let mut b = [0u8; 1];
        self.read(reg::STATUS, &mut b)?;
        Ok(Status(b[0]))
    }

    /// Read ERROR_ID (cleared by reading).
    pub fn error_flags(&mut self) -> Result<ErrorFlags, Error<I2C::Error>> {
        let mut b = [0u8; 1];
        self.read(reg::ERROR_ID, &mut b)?;
        Ok(ErrorFlags(b[0]))
    }

    /// Leave the boot loader. [`Error::InvalidData`] if no valid
    /// application is present. Returns [`APP_START_US`] (0 if already in
    /// application mode).
    pub fn start_app(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        let status = self.status()?;
        if status.app_mode() {
            return Ok(0);
        }
        if !status.app_valid() {
            return Err(Error::InvalidData);
        }
        self.write(&[reg::APP_START])?;
        Ok(APP_START_US)
    }

    /// Software reset back to boot mode; returns [`RESET_US`].
    pub fn reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        let [a, b, c, d] = SW_RESET_KEY;
        self.write(&[reg::SW_RESET, a, b, c, d])?;
        self.mode = DriveMode::Idle;
        Ok(RESET_US)
    }

    /// Write MEAS_MODE with the configured interrupt bits.
    pub fn set_drive_mode(&mut self, mode: DriveMode) -> Result<(), Error<I2C::Error>> {
        let value = meas_mode(mode, self.int_data_ready, self.int_threshold);
        self.write(&[reg::MEAS_MODE, value])?;
        self.mode = mode;
        Ok(())
    }

    /// Feed ambient temperature (0.01 °C) and humidity (0.01 %RH).
    pub fn set_environment(
        &mut self,
        centi_celsius: i32,
        centi_percent: u16,
    ) -> Result<(), Error<I2C::Error>> {
        let [a, b, c, d] = encode_environment(centi_celsius, centi_percent);
        self.write(&[reg::ENV_DATA, a, b, c, d])
    }

    /// Set the eCO2 thresholds used with interrupt-on-threshold.
    pub fn set_thresholds(
        &mut self,
        low_to_medium: u16,
        medium_to_high: u16,
        hysteresis: u8,
    ) -> Result<(), Error<I2C::Error>> {
        let [a, b, c, d, e] = encode_thresholds(low_to_medium, medium_to_high, hysteresis);
        self.write(&[reg::THRESHOLDS, a, b, c, d, e])
    }

    /// Read the encoded baseline (save it after conditioning).
    pub fn baseline(&mut self) -> Result<[u8; 2], Error<I2C::Error>> {
        let mut b = [0u8; 2];
        self.read(reg::BASELINE, &mut b)?;
        Ok(b)
    }

    /// Restore a saved baseline (only after the 20 min conditioning period).
    pub fn set_baseline(&mut self, baseline: [u8; 2]) -> Result<(), Error<I2C::Error>> {
        self.write(&[reg::BASELINE, baseline[0], baseline[1]])
    }

    /// Read RAW_DATA (the only output in [`DriveMode::Raw250ms`]).
    pub fn read_raw(&mut self) -> Result<RawData, Error<I2C::Error>> {
        let mut b = [0u8; 2];
        self.read(reg::RAW_DATA, &mut b)?;
        Ok(decode_raw(b))
    }

    /// Read NTC: voltages across RREF and RNTC in mV.
    pub fn read_ntc(&mut self) -> Result<(u16, u16), Error<I2C::Error>> {
        let mut b = [0u8; 4];
        self.read(reg::NTC, &mut b)?;
        Ok((
            u16::from_be_bytes([b[0], b[1]]),
            u16::from_be_bytes([b[2], b[3]]),
        ))
    }

    /// Read the boot loader and application firmware versions.
    pub fn firmware_versions(
        &mut self,
    ) -> Result<(FirmwareVersion, FirmwareVersion), Error<I2C::Error>> {
        let mut boot = [0u8; 2];
        let mut app = [0u8; 2];
        self.read(reg::FW_BOOT_VERSION, &mut boot)?;
        self.read(reg::FW_APP_VERSION, &mut app)?;
        Ok((decode_version(boot), decode_version(app)))
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Ccs811<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Selects the active drive mode if idle and returns its interval;
    /// returns `0` while already measuring. Requires application mode.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        if self.mode != DriveMode::Idle {
            return Ok(0);
        }
        self.set_drive_mode(self.active_mode)?;
        Ok(self.active_mode.interval_us())
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        if self.mode == DriveMode::Raw250ms {
            return Err(Error::Unsupported);
        }
        let mut block = [0u8; 8];
        self.read(reg::ALG_RESULT_DATA, &mut block)?;
        decode_result(&block)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Ccs811<I2C> {
    type BusError = I2C::Error;

    /// Drive mode 0 (idle). For the 19 µA sleep also release nWAKE.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_drive_mode(DriveMode::Idle)
    }

    /// Re-enter the active drive mode; returns its sample interval.
    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.set_drive_mode(self.active_mode)?;
        Ok(self.active_mode.interval_us())
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        DriveMode, ErrorFlags, FirmwareVersion, Measurement, RawData, SW_RESET_KEY, Status,
        check_hw_id, decode_raw, decode_result, decode_version, encode_environment,
        encode_thresholds, meas_mode, reg,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Ccs811`].
    pub struct Ccs811<I2C> {
        i2c: I2C,
        address: u8,
        mode: DriveMode,
        active_mode: DriveMode,
        int_data_ready: bool,
        int_threshold: bool,
    }

    impl<I2C> Ccs811<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Ccs811<I2C> {
        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write_read(self.address, &[reg], buf)
                .await
                .map_err(Error::bus)
        }

        async fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, bytes)
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Ccs811::probe`].
        pub async fn probe(&mut self) -> Result<u8, Error<I2C::Error>> {
            let mut id = [0u8; 2];
            self.read(reg::HW_ID, &mut id).await?;
            check_hw_id(id[0])?;
            Ok(id[1])
        }

        /// See [`super::Ccs811::status`].
        pub async fn status(&mut self) -> Result<Status, Error<I2C::Error>> {
            let mut b = [0u8; 1];
            self.read(reg::STATUS, &mut b).await?;
            Ok(Status(b[0]))
        }

        /// See [`super::Ccs811::error_flags`].
        pub async fn error_flags(&mut self) -> Result<ErrorFlags, Error<I2C::Error>> {
            let mut b = [0u8; 1];
            self.read(reg::ERROR_ID, &mut b).await?;
            Ok(ErrorFlags(b[0]))
        }

        /// See [`super::Ccs811::start_app`].
        pub async fn start_app(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let status = self.status().await?;
            if status.app_mode() {
                return Ok(0);
            }
            if !status.app_valid() {
                return Err(Error::InvalidData);
            }
            self.write(&[reg::APP_START]).await?;
            Ok(super::APP_START_US)
        }

        /// See [`super::Ccs811::reset`].
        pub async fn reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let [a, b, c, d] = SW_RESET_KEY;
            self.write(&[reg::SW_RESET, a, b, c, d]).await?;
            self.mode = DriveMode::Idle;
            Ok(super::RESET_US)
        }

        /// See [`super::Ccs811::set_drive_mode`].
        pub async fn set_drive_mode(&mut self, mode: DriveMode) -> Result<(), Error<I2C::Error>> {
            let value = meas_mode(mode, self.int_data_ready, self.int_threshold);
            self.write(&[reg::MEAS_MODE, value]).await?;
            self.mode = mode;
            Ok(())
        }

        /// See [`super::Ccs811::set_environment`].
        pub async fn set_environment(
            &mut self,
            centi_celsius: i32,
            centi_percent: u16,
        ) -> Result<(), Error<I2C::Error>> {
            let [a, b, c, d] = encode_environment(centi_celsius, centi_percent);
            self.write(&[reg::ENV_DATA, a, b, c, d]).await
        }

        /// See [`super::Ccs811::set_thresholds`].
        pub async fn set_thresholds(
            &mut self,
            low_to_medium: u16,
            medium_to_high: u16,
            hysteresis: u8,
        ) -> Result<(), Error<I2C::Error>> {
            let [a, b, c, d, e] = encode_thresholds(low_to_medium, medium_to_high, hysteresis);
            self.write(&[reg::THRESHOLDS, a, b, c, d, e]).await
        }

        /// See [`super::Ccs811::baseline`].
        pub async fn baseline(&mut self) -> Result<[u8; 2], Error<I2C::Error>> {
            let mut b = [0u8; 2];
            self.read(reg::BASELINE, &mut b).await?;
            Ok(b)
        }

        /// See [`super::Ccs811::set_baseline`].
        pub async fn set_baseline(&mut self, baseline: [u8; 2]) -> Result<(), Error<I2C::Error>> {
            self.write(&[reg::BASELINE, baseline[0], baseline[1]]).await
        }

        /// See [`super::Ccs811::read_raw`].
        pub async fn read_raw(&mut self) -> Result<RawData, Error<I2C::Error>> {
            let mut b = [0u8; 2];
            self.read(reg::RAW_DATA, &mut b).await?;
            Ok(decode_raw(b))
        }

        /// See [`super::Ccs811::read_ntc`].
        pub async fn read_ntc(&mut self) -> Result<(u16, u16), Error<I2C::Error>> {
            let mut b = [0u8; 4];
            self.read(reg::NTC, &mut b).await?;
            Ok((
                u16::from_be_bytes([b[0], b[1]]),
                u16::from_be_bytes([b[2], b[3]]),
            ))
        }

        /// See [`super::Ccs811::firmware_versions`].
        pub async fn firmware_versions(
            &mut self,
        ) -> Result<(FirmwareVersion, FirmwareVersion), Error<I2C::Error>> {
            let mut boot = [0u8; 2];
            let mut app = [0u8; 2];
            self.read(reg::FW_BOOT_VERSION, &mut boot).await?;
            self.read(reg::FW_APP_VERSION, &mut app).await?;
            Ok((decode_version(boot), decode_version(app)))
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Ccs811<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.mode != DriveMode::Idle {
                return Ok(0);
            }
            self.set_drive_mode(self.active_mode).await?;
            Ok(self.active_mode.interval_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if self.mode == DriveMode::Raw250ms {
                return Err(Error::Unsupported);
            }
            let mut block = [0u8; 8];
            self.read(reg::ALG_RESULT_DATA, &mut block).await?;
            decode_result(&block)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Ccs811<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.set_drive_mode(DriveMode::Idle).await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.set_drive_mode(self.active_mode).await?;
            Ok(self.active_mode.interval_us())
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = PRIMARY_ADDRESS;

    #[test]
    fn environment_matches_datasheet_examples() {
        // 48.5 %RH → 0x61 0x00; 23.5 °C → 0x61 0x00; defaults 50 %, 25 °C → 0x64 0x00.
        assert_eq!(encode_environment(2_350, 4_850), [0x61, 0x00, 0x61, 0x00]);
        assert_eq!(encode_environment(2_500, 5_000), [0x64, 0x00, 0x64, 0x00]);
        assert_eq!(encode_environment(-4_000, 12_000), [0xC8, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn thresholds_and_versions_encode() {
        assert_eq!(
            encode_thresholds(1_500, 2_500, 50),
            [0x05, 0xDC, 0x09, 0xC4, 0x32]
        );
        assert_eq!(
            decode_version([0x21, 0x03]),
            FirmwareVersion {
                major: 2,
                minor: 1,
                trivial: 3
            }
        );
        assert_eq!(meas_mode(DriveMode::Every10s, true, false), 0x28);
    }

    #[test]
    fn probe_and_app_start() {
        let expectations = [
            Transaction::write_read(A, vec![0x20], vec![0x81, 0x12]),
            Transaction::write_read(A, vec![0x20], vec![0x55, 0x12]),
            // Boot mode, application valid → APP_START.
            Transaction::write_read(A, vec![0x00], vec![0x10]),
            Transaction::write(A, vec![0xF4]),
            // Boot mode, no application.
            Transaction::write_read(A, vec![0x00], vec![0x00]),
            // Already in application mode.
            Transaction::write_read(A, vec![0x00], vec![0x90]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ccs = Ccs811::new(&mut i2c, A);
        assert_eq!(ccs.probe().unwrap(), 0x12);
        assert_eq!(ccs.probe(), Err(Error::UnexpectedId(0x55)));
        assert_eq!(ccs.start_app().unwrap(), APP_START_US);
        assert_eq!(ccs.start_app(), Err(Error::InvalidData));
        assert_eq!(ccs.start_app().unwrap(), 0);
        i2c.done();
    }

    #[test]
    fn measurement_cycle() {
        let expectations = [
            Transaction::write(A, vec![0x01, 0x10]),
            // DATA_READY clear.
            Transaction::write_read(A, vec![0x02], vec![0, 0, 0, 0, 0x90, 0, 0, 0]),
            // eCO2 0x01F4 (500), TVOC 0x0010 (16), status 0x98, raw: 32 µA, ADC 0x1A5.
            Transaction::write_read(
                A,
                vec![0x02],
                vec![0x01, 0xF4, 0x00, 0x10, 0x98, 0x00, 0x81, 0xA5],
            ),
            // ERROR set.
            Transaction::write_read(A, vec![0x02], vec![0, 0, 0, 0, 0x99, 0x10, 0, 0]),
            Transaction::write_read(A, vec![0xE0], vec![0x10]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ccs = Ccs811::new(&mut i2c, A);
        assert_eq!(ccs.start_measurement().unwrap(), 1_000_000);
        assert_eq!(ccs.start_measurement().unwrap(), 0);
        assert_eq!(ccs.read_measurement(), Err(Error::NotReady));
        let m = ccs.read_measurement().unwrap();
        assert_eq!(m.eco2_ppm, 500);
        assert_eq!(m.tvoc_ppb, 16);
        assert_eq!(
            m.raw,
            RawData {
                current_ua: 32,
                adc: 0x1A5
            }
        );
        let mut seen = vec![];
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(seen, vec![Quantity::Eco2(500), Quantity::Tvoc(16)]);
        assert_eq!(ccs.read_measurement(), Err(Error::InvalidData));
        assert!(ccs.error_flags().unwrap().heater_fault());
        i2c.done();
    }

    #[test]
    fn sleep_wake_reset_and_baseline() {
        let expectations = [
            Transaction::write(A, vec![0x01, 0x38]),
            Transaction::write(A, vec![0x01, 0x08]),
            Transaction::write(A, vec![0x01, 0x38]),
            Transaction::write_read(A, vec![0x11], vec![0x84, 0x7A]),
            Transaction::write(A, vec![0x11, 0x84, 0x7A]),
            Transaction::write(A, vec![0xFF, 0x11, 0xE5, 0x72, 0x8A]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ccs = Ccs811::new(&mut i2c, A)
            .with_active_mode(DriveMode::Every60s)
            .with_interrupts(true, false);
        assert_eq!(ccs.start_measurement().unwrap(), 60_000_000);
        ccs.sleep().unwrap();
        assert_eq!(ccs.wake().unwrap(), 60_000_000);
        let baseline = ccs.baseline().unwrap();
        ccs.set_baseline(baseline).unwrap();
        assert_eq!(ccs.reset().unwrap(), RESET_US);
        assert_eq!(ccs.mode(), DriveMode::Idle);
        i2c.done();
    }

    #[test]
    fn raw_mode_has_no_algorithm_result() {
        let expectations = [
            Transaction::write(A, vec![0x01, 0x40]),
            Transaction::write_read(A, vec![0x03], vec![0x0B, 0xFF]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ccs = Ccs811::new(&mut i2c, A).with_active_mode(DriveMode::Raw250ms);
        assert_eq!(ccs.start_measurement().unwrap(), 250_000);
        assert_eq!(ccs.read_measurement(), Err(Error::Unsupported));
        assert_eq!(
            ccs.read_raw().unwrap(),
            RawData {
                current_ua: 2,
                adc: 0x3FF
            }
        );
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

        let expectations = [
            Transaction::write_read(A, vec![0x20], vec![0x81, 0x12]),
            Transaction::write(A, vec![0x01, 0x10]),
            Transaction::write_read(
                A,
                vec![0x02],
                vec![0x01, 0x90, 0x00, 0x00, 0x98, 0x00, 0x00, 0x00],
            ),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ccs = asynch::Ccs811::new(&mut i2c, A);
        block_on(ccs.probe()).unwrap();
        let mut delay = RecordingDelay(0);
        let m = block_on(ccs.measure(&mut delay)).unwrap();
        assert_eq!(m.eco2_ppm, 400);
        assert_eq!(delay.0, 1_000_000);
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
