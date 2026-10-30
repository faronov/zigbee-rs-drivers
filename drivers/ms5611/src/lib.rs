//! TE Connectivity MS5611-01BA and MS5607-02BA pressure and temperature
//! sensors over I2C or SPI.
//!
//! The parts have no status register: a conversion is started with a
//! command and the result is read with `ADC read` once the conversion time
//! has passed. Pressure compensation needs the temperature of the same
//! cycle, so the driver is a small two-phase state machine (same contract
//! as `zigbee-bmp180`):
//!
//! 1. `start_measurement` starts the temperature conversion (D2) and
//!    returns its wait.
//! 2. The first `read_measurement` reads D2, starts the pressure
//!    conversion (D1) and returns [`Error::NotReady`];
//!    [`Ms5611::pending_wait_us`] then gives the D1 conversion time.
//! 3. The second `read_measurement` returns the compensated
//!    [`Measurement`].
//!
//! Reading before the conversion has finished returns 0 and corrupts the
//! running conversion (datasheet "Conversion sequence"); the driver maps
//! a zero result to [`Error::InvalidData`] and goes back to idle, so the
//! cycle must be started again. Waits are the datasheet maximum
//! conversion times. The generic `measure` helpers (start, one wait, one
//! read) stop at step 2; use [`Ms5611::measure_full`] or drive the steps
//! from the application. The part idles (< 0.15 µA) between conversions.
//!
//! The parts have no ID register. [`Ms5611::init`] reads the PROM and
//! checks its CRC-4, which doubles as a presence check. The variant
//! ([`Variant`]) cannot be read back and is chosen by the application;
//! it selects the compensation constants. Send [`Ms5611::soft_reset`] once
//! after power-up (the datasheet requires it to load the PROM) and wait
//! [`RESET_WAIT_US`] before `init`.
//!
//! Compensation is the datasheet integer algorithm including the
//! second-order (low-temperature) correction, in 64-bit arithmetic.
//!
//! References:
//! * TE "MS5611-01BA03 Barometric Pressure Sensor" datasheet, 06/2017.
//! * Measurement Specialties "MS5607-02BA03" datasheet DA5607-02BA03_018,
//!   2012-09-26.
//!
//! ```ignore
//! let mut ms = Ms5611::new_i2c(i2c, ADDRESS_CSB_LOW, Variant::Ms5611);
//! let wait = ms.soft_reset()?; // once after power-up
//! // ... wait ...
//! let mut wait_us = ms.start_measurement()?;
//! let m = loop {
//!     // ... let the Zigbee stack run / sleep for wait_us ...
//!     match ms.read_measurement() {
//!         Err(Error::NotReady) => wait_us = ms.pending_wait_us(),
//!         other => break other?,
//!     }
//! };
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with CSB tied to GND (the address LSB is the complement of
/// CSB).
pub const ADDRESS_CSB_LOW: u8 = 0x77;
/// I2C address with CSB tied to VDD.
pub const ADDRESS_CSB_HIGH: u8 = 0x76;

const CMD_RESET: u8 = 0x1E;
const CMD_ADC_READ: u8 = 0x00;
const CMD_CONVERT_D1: u8 = 0x40;
const CMD_CONVERT_D2: u8 = 0x50;
const CMD_PROM_READ: u8 = 0xA0;

/// PROM reload time after a reset (datasheet "Reset sequence": 2.8 ms),
/// with margin.
pub const RESET_WAIT_US: WaitUs = 3_000;

/// Command access over I2C or SPI.
pub trait Interface {
    /// Transport error.
    type Error;
    /// Send a one-byte command.
    fn command(&mut self, cmd: u8) -> Result<(), Self::Error>;
    /// Send a command and read its result (PROM word or ADC value).
    fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
}

/// I2C transport.
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    /// The bus.
    pub i2c: I2C,
    /// 7-bit address ([`ADDRESS_CSB_LOW`] or [`ADDRESS_CSB_HIGH`]).
    pub address: u8,
}

/// SPI transport (mode 0 or 3, up to 20 MHz; PS pin low).
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    /// The device (bus + chip select).
    pub spi: SPI,
}

impl<I2C: embedded_hal::i2c::I2c> Interface for I2cInterface<I2C> {
    type Error = I2C::Error;

    fn command(&mut self, cmd: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[cmd])
    }

    fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[cmd], buf)
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn command(&mut self, cmd: u8) -> Result<(), Self::Error> {
        self.spi.write(&[cmd])
    }

    fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.spi
            .transaction(&mut [Operation::Write(&[cmd]), Operation::Read(buf)])
    }
}

/// Part, selecting the compensation constants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// MS5611-01BA (10-1200 mbar).
    Ms5611,
    /// MS5607-02BA (10-1200 mbar, different scaling).
    Ms5607,
}

impl Variant {
    /// `(C2 shift, C4·dT divisor shift, C1 shift, C3·dT divisor shift)`.
    const fn first_order(self) -> (u32, u32, u32, u32) {
        match self {
            Self::Ms5611 => (16, 7, 15, 8),
            Self::Ms5607 => (17, 6, 16, 7),
        }
    }
}

/// Oversampling ratio of one conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(missing_docs)]
pub enum Osr {
    X256 = 0,
    X512 = 1,
    X1024 = 2,
    X2048 = 3,
    X4096 = 4,
}

impl Osr {
    /// Maximum conversion time in µs (datasheet ADC table).
    pub const fn conversion_us(self) -> WaitUs {
        [600, 1_170, 2_280, 4_540, 9_040][self as usize]
    }

    const fn cmd_offset(self) -> u8 {
        2 * self as u8
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Pressure (D1) oversampling.
    pub pressure: Osr,
    /// Temperature (D2) oversampling.
    pub temperature: Osr,
}

impl Config {
    /// Highest resolution for both conversions (2 × 9.04 ms).
    pub const HIGH_RES: Self = Self {
        pressure: Osr::X4096,
        temperature: Osr::X4096,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self::HIGH_RES
    }
}

/// One compensated sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Pressure in Pa.
    pub pressure: u32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
        f(Quantity::Pressure(self.pressure));
    }
}

/// Factory calibration C1..C6 (PROM words 1..6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub struct Calibration {
    pub c1: u16,
    pub c2: u16,
    pub c3: u16,
    pub c4: u16,
    pub c5: u16,
    pub c6: u16,
}

/// CRC-4 of the 8 PROM words (datasheet / AN520): the CRC itself (low
/// nibble of word 7) is excluded.
pub fn crc4(prom: &[u16; 8]) -> u8 {
    let mut rem: u16 = 0;
    for i in 0..16 {
        let mut word = prom[i >> 1];
        if i >> 1 == 7 {
            word &= 0xFF00;
        }
        rem ^= if i % 2 == 1 { word & 0x00FF } else { word >> 8 };
        for _ in 0..8 {
            rem = if rem & 0x8000 != 0 {
                (rem << 1) ^ 0x3000
            } else {
                rem << 1
            };
        }
    }
    (rem >> 12) as u8
}

impl Calibration {
    /// Check the CRC of the 8 PROM words and extract C1..C6.
    pub fn from_prom<E>(prom: &[u16; 8]) -> Result<Self, Error<E>> {
        if crc4(prom) != (prom[7] & 0x0F) as u8 {
            return Err(Error::Crc);
        }
        Ok(Self {
            c1: prom[1],
            c2: prom[2],
            c3: prom[3],
            c4: prom[4],
            c5: prom[5],
            c6: prom[6],
        })
    }

    /// First- and second-order compensation of the raw pressure `d1` and
    /// temperature `d2` (datasheet "Pressure and temperature calculation"
    /// and "Second order temperature compensation").
    pub fn compensate(&self, variant: Variant, d1: u32, d2: u32) -> Measurement {
        let dt = i64::from(d2) - (i64::from(self.c5) << 8);
        let mut temp = 2000 + dt * i64::from(self.c6) / (1 << 23);
        let (c2_shift, c4_shift, c1_shift, c3_shift) = variant.first_order();
        let mut off = (i64::from(self.c2) << c2_shift) + i64::from(self.c4) * dt / (1 << c4_shift);
        let mut sens = (i64::from(self.c1) << c1_shift) + i64::from(self.c3) * dt / (1 << c3_shift);
        if temp < 2000 {
            let t2 = dt * dt / (1 << 31);
            let low = (temp - 2000) * (temp - 2000);
            let (mut off2, mut sens2) = match variant {
                Variant::Ms5611 => (5 * low / 2, 5 * low / 4),
                Variant::Ms5607 => (61 * low / 16, 2 * low),
            };
            if temp < -1500 {
                let very_low = (temp + 1500) * (temp + 1500);
                match variant {
                    Variant::Ms5611 => {
                        off2 += 7 * very_low;
                        sens2 += 11 * very_low / 2;
                    }
                    Variant::Ms5607 => {
                        off2 += 15 * very_low;
                        sens2 += 8 * very_low;
                    }
                }
            }
            temp -= t2;
            off -= off2;
            sens -= sens2;
        }
        let p = (i64::from(d1) * sens / (1 << 21) - off) / (1 << 15);
        Measurement {
            temperature: temp.clamp(i32::MIN.into(), i32::MAX.into()) as i32,
            pressure: p.clamp(0, u32::MAX.into()) as u32,
        }
    }
}

fn adc24(b: &[u8; 3]) -> u32 {
    u32::from(b[0]) << 16 | u32::from(b[1]) << 8 | u32::from(b[2])
}

/// Where the two-phase cycle stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// No conversion started; `read_measurement` returns `NotReady`.
    Idle,
    /// Temperature (D2) conversion running.
    Temperature,
    /// Pressure (D1) conversion running (D2 captured).
    Pressure,
}

#[derive(Debug)]
struct State {
    variant: Variant,
    config: Config,
    calibration: Option<Calibration>,
    phase: Phase,
    d2: u32,
}

impl State {
    const fn new(variant: Variant) -> Self {
        Self {
            variant,
            config: Config::HIGH_RES,
            calibration: None,
            phase: Phase::Idle,
            d2: 0,
        }
    }

    const fn pending_wait_us(&self) -> WaitUs {
        match self.phase {
            Phase::Idle => 0,
            Phase::Temperature => self.config.temperature.conversion_us(),
            Phase::Pressure => self.config.pressure.conversion_us(),
        }
    }

    const fn start_cmd(&self) -> u8 {
        CMD_CONVERT_D2 + self.config.temperature.cmd_offset()
    }

    const fn pressure_cmd(&self) -> u8 {
        CMD_CONVERT_D1 + self.config.pressure.cmd_offset()
    }

    /// Consume an ADC result: `None` means "start the pressure phase".
    fn on_result<E>(&mut self, raw: &[u8; 3]) -> Result<Option<Measurement>, Error<E>> {
        let value = adc24(raw);
        if value == 0 {
            // Read too early: the running conversion is spoiled.
            self.phase = Phase::Idle;
            return Err(Error::InvalidData);
        }
        if self.phase == Phase::Temperature {
            self.d2 = value;
            self.phase = Phase::Pressure;
            return Ok(None);
        }
        self.phase = Phase::Idle;
        let c = self.calibration.as_ref().ok_or(Error::NotReady)?;
        Ok(Some(c.compensate(self.variant, value, self.d2)))
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Create a driver over any transport; no bus traffic.
            pub const fn new(iface: IFACE, variant: Variant) -> Self {
                Self {
                    iface,
                    state: State::new(variant),
                }
            }

            /// Select the oversampling used by the next cycle.
            pub const fn with_config(mut self, config: Config) -> Self {
                self.state.config = config;
                self
            }

            /// Change the oversampling (aborts nothing on the device; a
            /// running cycle keeps its commands but uses the new waits).
            pub fn set_config(&mut self, config: Config) {
                self.state.config = config;
            }

            /// Current configuration.
            pub const fn config(&self) -> &Config {
                &self.state.config
            }

            /// Variant selected at construction.
            pub const fn variant(&self) -> Variant {
                self.state.variant
            }

            /// Calibration read by `init` (`None` before).
            pub const fn calibration(&self) -> Option<&Calibration> {
                self.state.calibration.as_ref()
            }

            /// Current step of the two-phase cycle.
            pub const fn phase(&self) -> Phase {
                self.state.phase
            }

            /// Time to wait before the next `read_measurement`: the
            /// conversion currently running (0 when idle).
            pub const fn pending_wait_us(&self) -> WaitUs {
                self.state.pending_wait_us()
            }

            /// Give the transport back.
            pub fn release(self) -> IFACE {
                self.iface
            }
        }

        impl<I2C> $name<I2cInterface<I2C>> {
            /// Driver on an I2C bus.
            pub const fn new_i2c(i2c: I2C, address: u8, variant: Variant) -> Self {
                Self::new(I2cInterface { i2c, address }, variant)
            }
        }

        impl<SPI> $name<SpiInterface<SPI>> {
            /// Driver on an SPI device.
            pub const fn new_spi(spi: SPI, variant: Variant) -> Self {
                Self::new(SpiInterface { spi }, variant)
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Ms5611<IFACE> {
    iface: IFACE,
    state: State,
}

constructors!(Ms5611);

impl<IFACE: Interface> Ms5611<IFACE> {
    /// Reset (reloads the PROM). Wait [`RESET_WAIT_US`] before `init`.
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.state.calibration = None;
        self.state.phase = Phase::Idle;
        self.iface.command(CMD_RESET).map_err(Error::bus)?;
        Ok(RESET_WAIT_US)
    }

    /// Read the 8 PROM words and check the CRC ([`Error::Crc`] on a
    /// mismatch, e.g. without the post-power-up reset). Called by
    /// `start_measurement`.
    pub fn init(&mut self) -> Result<Calibration, Error<IFACE::Error>> {
        let mut prom = [0u16; 8];
        for (i, word) in (0u8..).zip(prom.iter_mut()) {
            let mut b = [0; 2];
            self.iface
                .read(CMD_PROM_READ + 2 * i, &mut b)
                .map_err(Error::bus)?;
            *word = u16::from_be_bytes(b);
        }
        let calibration = Calibration::from_prom(&prom)?;
        self.state.calibration = Some(calibration);
        Ok(calibration)
    }

    /// Run a complete cycle (temperature then pressure), blocking on
    /// `delay`. For tests and bring-up, not for a Zigbee event loop.
    pub fn measure_full<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Measurement, Error<IFACE::Error>> {
        use zigbee_sensor::blocking::Sensor;
        let mut wait = self.start_measurement()?;
        for _ in 0..2 {
            delay.delay_us(wait);
            match self.read_measurement() {
                Err(Error::NotReady) => wait = self.pending_wait_us(),
                other => return other,
            }
        }
        Err(Error::Timeout)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Ms5611<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    /// Start the temperature conversion (first phase).
    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        if self.state.calibration.is_none() {
            self.init()?;
        }
        self.iface
            .command(self.state.start_cmd())
            .map_err(Error::bus)?;
        self.state.phase = Phase::Temperature;
        Ok(self.state.pending_wait_us())
    }

    /// After the temperature phase: start the pressure conversion and
    /// return [`Error::NotReady`] (wait [`Ms5611::pending_wait_us`]).
    /// After the pressure phase: the measurement.
    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        if self.state.phase == Phase::Idle {
            return Err(Error::NotReady);
        }
        let mut raw = [0; 3];
        self.iface
            .read(CMD_ADC_READ, &mut raw)
            .map_err(Error::bus)?;
        match self.state.on_result(&raw)? {
            Some(m) => Ok(m),
            None => {
                self.iface
                    .command(self.state.pressure_cmd())
                    .map_err(Error::bus)?;
                Err(Error::NotReady)
            }
        }
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Ms5611<IFACE> {
    type BusError = IFACE::Error;

    /// Forget a running cycle; the part idles by itself after each
    /// conversion (no command needed).
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.state.phase = Phase::Idle;
        Ok(())
    }

    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_ADC_READ, CMD_PROM_READ, CMD_RESET, Calibration, Config, I2cInterface, Measurement,
        Phase, RESET_WAIT_US, SpiInterface, State, Variant,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async command access.
    #[allow(async_fn_in_trait)]
    pub trait Interface {
        /// Transport error.
        type Error;
        /// Send a one-byte command.
        async fn command(&mut self, cmd: u8) -> Result<(), Self::Error>;
        /// Send a command and read its result.
        async fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Interface for I2cInterface<I2C> {
        type Error = I2C::Error;

        async fn command(&mut self, cmd: u8) -> Result<(), Self::Error> {
            self.i2c.write(self.address, &[cmd]).await
        }

        async fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            self.i2c.write_read(self.address, &[cmd], buf).await
        }
    }

    impl<SPI: embedded_hal_async::spi::SpiDevice> Interface for SpiInterface<SPI> {
        type Error = SPI::Error;

        async fn command(&mut self, cmd: u8) -> Result<(), Self::Error> {
            self.spi.write(&[cmd]).await
        }

        async fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            use embedded_hal_async::spi::Operation;
            self.spi
                .transaction(&mut [Operation::Write(&[cmd]), Operation::Read(buf)])
                .await
        }
    }

    /// Async twin of [`super::Ms5611`].
    #[derive(Debug)]
    pub struct Ms5611<IFACE> {
        iface: IFACE,
        state: State,
    }

    constructors!(Ms5611);

    impl<IFACE: Interface> Ms5611<IFACE> {
        /// See [`super::Ms5611::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.state.calibration = None;
            self.state.phase = Phase::Idle;
            self.iface.command(CMD_RESET).await.map_err(Error::bus)?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Ms5611::init`].
        pub async fn init(&mut self) -> Result<Calibration, Error<IFACE::Error>> {
            let mut prom = [0u16; 8];
            for (i, word) in (0u8..).zip(prom.iter_mut()) {
                let mut b = [0; 2];
                self.iface
                    .read(CMD_PROM_READ + 2 * i, &mut b)
                    .await
                    .map_err(Error::bus)?;
                *word = u16::from_be_bytes(b);
            }
            let calibration = Calibration::from_prom(&prom)?;
            self.state.calibration = Some(calibration);
            Ok(calibration)
        }

        /// See [`super::Ms5611::measure_full`].
        pub async fn measure_full<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<Measurement, Error<IFACE::Error>> {
            use zigbee_sensor::asynch::Sensor;
            let mut wait = self.start_measurement().await?;
            for _ in 0..2 {
                delay.delay_us(wait).await;
                match self.read_measurement().await {
                    Err(Error::NotReady) => wait = self.pending_wait_us(),
                    other => return other,
                }
            }
            Err(Error::Timeout)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Ms5611<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            if self.state.calibration.is_none() {
                self.init().await?;
            }
            self.iface
                .command(self.state.start_cmd())
                .await
                .map_err(Error::bus)?;
            self.state.phase = Phase::Temperature;
            Ok(self.state.pending_wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            if self.state.phase == Phase::Idle {
                return Err(Error::NotReady);
            }
            let mut raw = [0; 3];
            self.iface
                .read(CMD_ADC_READ, &mut raw)
                .await
                .map_err(Error::bus)?;
            match self.state.on_result(&raw)? {
                Some(m) => Ok(m),
                None => {
                    self.iface
                        .command(self.state.pressure_cmd())
                        .await
                        .map_err(Error::bus)?;
                    Err(Error::NotReady)
                }
            }
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Ms5611<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.state.phase = Phase::Idle;
            Ok(())
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            Ok(0)
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
    use std::vec::Vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    /// MS5611 datasheet example coefficients.
    const CAL_5611: Calibration = Calibration {
        c1: 40_127,
        c2: 36_924,
        c3: 23_317,
        c4: 23_282,
        c5: 33_464,
        c6: 28_312,
    };

    /// MS5607 datasheet example coefficients.
    const CAL_5607: Calibration = Calibration {
        c1: 46_372,
        c2: 43_981,
        c3: 29_059,
        c4: 27_842,
        c5: 31_553,
        c6: 28_165,
    };

    /// PROM with factory word 0x0050, serial 0x5A3 and its CRC (0xE,
    /// computed with the datasheet C routine).
    const PROM: [u16; 8] = [
        0x0050, 40_127, 36_924, 23_317, 23_282, 33_464, 28_312, 0x5A3E,
    ];

    #[test]
    fn datasheet_examples() {
        let m = CAL_5611.compensate(Variant::Ms5611, 9_085_466, 8_569_150);
        assert_eq!(
            m,
            Measurement {
                temperature: 2_007,
                pressure: 100_009,
            }
        );
        let m = CAL_5607.compensate(Variant::Ms5607, 6_465_444, 8_077_636);
        assert_eq!(
            m,
            Measurement {
                temperature: 2_000,
                pressure: 110_002,
            }
        );
    }

    #[test]
    fn second_order_low_and_very_low_temperature() {
        // Expected values from the datasheet algorithm evaluated with
        // C-style (truncating) integer division.
        let cases = [
            (Variant::Ms5611, CAL_5611, 9_085_466, 8_000_000, -61, 95_989),
            (
                Variant::Ms5611,
                CAL_5611,
                9_085_466,
                7_000_000,
                -4_430,
                85_696,
            ),
            (
                Variant::Ms5607,
                CAL_5607,
                6_465_444,
                8_000_000,
                1_738,
                109_370,
            ),
            (
                Variant::Ms5607,
                CAL_5607,
                6_465_444,
                7_000_000,
                -2_157,
                100_348,
            ),
        ];
        for (v, c, d1, d2, t, p) in cases {
            let m = c.compensate(v, d1, d2);
            assert_eq!((m.temperature, m.pressure), (t, p), "{v:?} D2 {d2}");
        }
    }

    #[test]
    fn crc_and_prom_decode() {
        assert_eq!(crc4(&PROM), 0xE);
        assert_eq!(Calibration::from_prom::<()>(&PROM), Ok(CAL_5611));
        let mut bad = PROM;
        bad[3] ^= 0x0100;
        assert_eq!(Calibration::from_prom::<()>(&bad), Err(Error::Crc));
    }

    #[test]
    fn compensation_is_panic_free() {
        let max = Calibration {
            c1: u16::MAX,
            c2: u16::MAX,
            c3: u16::MAX,
            c4: u16::MAX,
            c5: u16::MAX,
            c6: u16::MAX,
        };
        let min = Calibration {
            c5: u16::MAX,
            c6: u16::MAX,
            ..Calibration::default()
        };
        let raws = [0, 1, 1 << 23, (1 << 24) - 1];
        for v in [Variant::Ms5611, Variant::Ms5607] {
            for c in [CAL_5611, max, min, Calibration::default()] {
                for d1 in raws {
                    for d2 in raws {
                        let _ = c.compensate(v, d1, d2);
                    }
                }
            }
        }
    }

    fn i2c_prom(a: u8, prom: &[u16; 8]) -> Vec<I2cTx> {
        (0u8..)
            .zip(prom)
            .map(|(i, w)| I2cTx::write_read(a, vec![0xA0 + 2 * i], w.to_be_bytes().to_vec()))
            .collect()
    }

    fn adc(v: u32) -> Vec<u8> {
        v.to_be_bytes()[1..].to_vec()
    }

    #[test]
    fn i2c_two_phase_cycle() {
        let a = ADDRESS_CSB_LOW;
        let mut ex = i2c_prom(a, &PROM);
        ex.extend([
            I2cTx::write(a, vec![0x58]),
            I2cTx::write_read(a, vec![0x00], adc(8_569_150)),
            I2cTx::write(a, vec![0x48]),
            I2cTx::write_read(a, vec![0x00], adc(9_085_466)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut ms = Ms5611::new_i2c(&mut i2c, a, Variant::Ms5611);
        assert_eq!(ms.read_measurement(), Err(Error::NotReady));
        assert_eq!(ms.start_measurement().unwrap(), 9_040);
        assert_eq!(ms.read_measurement(), Err(Error::NotReady));
        assert_eq!(ms.phase(), Phase::Pressure);
        assert_eq!(ms.pending_wait_us(), 9_040);
        let m = ms.read_measurement().unwrap();
        assert_eq!((m.temperature, m.pressure), (2_007, 100_009));
        assert_eq!(ms.phase(), Phase::Idle);
        ms.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn early_read_and_bad_crc() {
        let a = ADDRESS_CSB_HIGH;
        let mut bad = PROM;
        bad[7] ^= 1;
        let mut ex = i2c_prom(a, &bad);
        ex.extend(i2c_prom(a, &PROM));
        ex.extend([
            I2cTx::write(a, vec![0x50]),
            I2cTx::write_read(a, vec![0x00], adc(0)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut ms = Ms5611::new_i2c(&mut i2c, a, Variant::Ms5611).with_config(Config {
            pressure: Osr::X256,
            temperature: Osr::X256,
        });
        assert_eq!(ms.start_measurement(), Err(Error::Crc));
        assert_eq!(ms.start_measurement().unwrap(), 600);
        assert_eq!(ms.read_measurement(), Err(Error::InvalidData));
        assert_eq!(ms.phase(), Phase::Idle);
        i2c.done();
    }

    #[test]
    fn spi_reset_init_and_cycle() {
        let rd = |cmd: u8, v: Vec<u8>| {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![cmd]),
                SpiTx::read_vec(v),
                SpiTx::transaction_end(),
            ]
        };
        let cmd = |c: u8| {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![c]),
                SpiTx::transaction_end(),
            ]
        };
        let prom = {
            let mut p = [
                0x00C0, 46_372, 43_981, 29_059, 27_842, 31_553, 28_165, 0x1230,
            ];
            p[7] |= u16::from(crc4(&p));
            p
        };
        let mut ex = Vec::new();
        ex.extend(cmd(0x1E));
        for (i, w) in (0u8..).zip(prom) {
            ex.extend(rd(0xA0 + 2 * i, w.to_be_bytes().to_vec()));
        }
        ex.extend(cmd(0x54));
        ex.extend(rd(0x00, adc(8_077_636)));
        ex.extend(cmd(0x46));
        ex.extend(rd(0x00, adc(6_465_444)));
        let mut spi = SpiMock::new(&ex);
        let mut ms = Ms5611::new_spi(&mut spi, Variant::Ms5607).with_config(Config {
            pressure: Osr::X2048,
            temperature: Osr::X1024,
        });
        assert_eq!(ms.soft_reset().unwrap(), RESET_WAIT_US);
        assert_eq!(ms.init().unwrap(), CAL_5607);
        assert_eq!(ms.start_measurement().unwrap(), 2_280);
        assert_eq!(ms.read_measurement(), Err(Error::NotReady));
        assert_eq!(ms.pending_wait_us(), 4_540);
        let m = ms.read_measurement().unwrap();
        assert_eq!((m.temperature, m.pressure), (2_000, 110_002));
        spi.done();
    }

    #[test]
    fn async_measure_full_waits_both_phases() {
        struct RecordingDelay(Vec<u32>);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0.push(ns / 1_000);
            }
        }

        let a = ADDRESS_CSB_LOW;
        let mut ex = i2c_prom(a, &PROM);
        ex.extend([
            I2cTx::write(a, vec![0x58]),
            I2cTx::write_read(a, vec![0x00], adc(8_569_150)),
            I2cTx::write(a, vec![0x48]),
            I2cTx::write_read(a, vec![0x00], adc(9_085_466)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut ms = asynch::Ms5611::new_i2c(&mut i2c, a, Variant::Ms5611);
        let mut delay = RecordingDelay(Vec::new());
        let m = block_on(ms.measure_full(&mut delay)).unwrap();
        assert_eq!(m.pressure, 100_009);
        assert_eq!(delay.0, [9_040, 9_040]);
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
