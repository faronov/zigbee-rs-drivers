//! MAX31865 RTD-to-digital converter for PT100 / PT1000 platinum RTDs.
//!
//! * 2-, 3- or 4-wire RTD, 50 or 60 Hz notch filter.
//! * Reference resistor and RTD nominal resistance (R0) are configurable,
//!   in mΩ (typical boards: 430 Ω for PT100, 4.3 kΩ for PT1000).
//! * Resistance → temperature with the IEC 60751 Callendar-Van Dusen
//!   equation in integer arithmetic ([`temperature_from_ratio`]).
//! * Fault status register, automatic fault-detection cycle and fault
//!   thresholds.
//!
//! Reference: Maxim MAX31865 datasheet 19-6478 Rev 0 (Tables 1–3,
//! Electrical Characteristics tCONV, Note 4 / "1-Shot (D5)" bias settling).
//!
//! # Measurement flow
//!
//! The bias voltage drives ≈ 2 V / (Rref + RTD) through the RTD (≈ 3.8 mA
//! for PT100 / 430 Ω), so the driver keeps it off between readings
//! ("normally off" mode):
//!
//! * Bias off (default): `start_measurement` switches bias on together with
//!   automatic conversion mode and returns the time until a conversion that
//!   started after the input RC filter settled has completed. With the
//!   defaults (11.5 ms settling, 60 Hz) that is 72.6 ms.
//!   `read_measurement` reads the result and switches bias and conversions
//!   off again.
//! * Bias on (after [`Max31865::enable_bias`] and its settling wait):
//!   `start_measurement` triggers a 1-shot conversion (55 ms at 60 Hz, 66 ms
//!   at 50 Hz) and bias stays on until [`Max31865::disable_bias`].
//!
//! The settling time is 10.5 time constants of the RC filter across the RTD
//! inputs plus 1 ms; the datasheet maximum (10 kΩ, 0.1 µF) gives 11.5 ms,
//! the default. Set it with [`Max31865::with_bias_settle_us`].
//!
//! The part has no shutdown mode: it draws 1.5 mA typ (3 mA max) with bias
//! and ADC off. Switch its supply for battery devices.
//!
//! Wiring: SPI mode 1 or 3, ≤ 5 MHz; use an `embedded_hal::spi::SpiDevice`
//! that drives CS (the 1-shot conversion starts on CS rising).

#![no_std]

mod rtd;

pub use rtd::temperature_from_ratio;

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Register addresses (read address; writes set bit 7).
pub mod reg {
    /// Configuration.
    pub const CONFIG: u8 = 0x00;
    /// RTD MSB (LSB at 0x02, D0 = fault).
    pub const RTD_MSB: u8 = 0x01;
    /// High fault threshold MSB (LSB at 0x04).
    pub const HIGH_FAULT_MSB: u8 = 0x03;
    /// Low fault threshold MSB (LSB at 0x06).
    pub const LOW_FAULT_MSB: u8 = 0x05;
    /// Fault status.
    pub const FAULT_STATUS: u8 = 0x07;
    /// Write flag.
    pub const WRITE: u8 = 0x80;
}

/// Configuration register bits.
pub mod cfg {
    /// D7: bias voltage on.
    pub const VBIAS: u8 = 0x80;
    /// D6: automatic (continuous) conversion.
    pub const AUTO: u8 = 0x40;
    /// D5: 1-shot (self-clearing).
    pub const ONE_SHOT: u8 = 0x20;
    /// D4: 3-wire RTD.
    pub const THREE_WIRE: u8 = 0x10;
    /// D3:2: fault-detection cycle control.
    pub const FAULT_CYCLE: u8 = 0x0C;
    /// D3:2 = 01: automatic fault detection.
    pub const FAULT_AUTO: u8 = 0x04;
    /// D1: fault status clear (self-clearing).
    pub const FAULT_CLEAR: u8 = 0x02;
    /// D0: 50 Hz notch (0 = 60 Hz).
    pub const FILTER_50HZ: u8 = 0x01;
}

/// Duration of the automatic fault-detection cycle.
pub const FAULT_DETECTION_US: WaitUs = 600;

/// RTD connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wiring {
    /// 2-wire RTD.
    TwoWire,
    /// 3-wire RTD.
    ThreeWire,
    /// 4-wire RTD.
    FourWire,
}

/// Notch filter, matching the mains frequency.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Filter {
    /// 50 Hz mains.
    Hz50,
    /// 60 Hz mains.
    Hz60,
}

impl Filter {
    /// 1-shot (and first automatic) conversion time, maximum.
    pub const fn single_us(self) -> WaitUs {
        match self {
            Self::Hz50 => 66_000,
            Self::Hz60 => 55_000,
        }
    }

    /// Automatic-mode conversion period, maximum.
    pub const fn continuous_us(self) -> WaitUs {
        match self {
            Self::Hz50 => 21_000,
            Self::Hz60 => 17_600,
        }
    }
}

/// Fault status register (07h).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultStatus(pub u8);

impl FaultStatus {
    /// D7: RTD code above the high threshold.
    pub const fn rtd_high_threshold(&self) -> bool {
        self.0 & 0x80 != 0
    }
    /// D6: RTD code below the low threshold.
    pub const fn rtd_low_threshold(&self) -> bool {
        self.0 & 0x40 != 0
    }
    /// D5: REFIN− > 0.85 × VBIAS.
    pub const fn refin_high(&self) -> bool {
        self.0 & 0x20 != 0
    }
    /// D4: REFIN− < 0.85 × VBIAS with FORCE− open.
    pub const fn refin_low_force_open(&self) -> bool {
        self.0 & 0x10 != 0
    }
    /// D3: RTDIN− < 0.85 × VBIAS with FORCE− open.
    pub const fn rtdin_low_force_open(&self) -> bool {
        self.0 & 0x08 != 0
    }
    /// D2: over- or under-voltage on an input.
    pub const fn over_under_voltage(&self) -> bool {
        self.0 & 0x04 != 0
    }
    /// Any fault.
    pub const fn any(&self) -> bool {
        self.0 & 0xFC != 0
    }
}

/// One reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// RTD resistance in mΩ.
    pub resistance_mohm: u32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
    }
}

/// RTD resistance in mΩ for a 15-bit ADC code: `code · Rref / 2^15`.
pub const fn resistance_mohm(code: u16, rref_mohm: u32) -> u32 {
    ((code as u64 * rref_mohm as u64 + (1 << 14)) >> 15) as u32
}

/// Decode the two RTD registers into a measurement. The fault bit (D0) or
/// a resistance outside the conversion range is [`Error::InvalidData`].
pub fn decode<E>(rtd: [u8; 2], rref_mohm: u32, r0_mohm: u32) -> Result<Measurement, Error<E>> {
    let word = u16::from_be_bytes(rtd);
    if word & 1 != 0 || r0_mohm == 0 {
        return Err(Error::InvalidData);
    }
    let code = word >> 1;
    let r0 = u64::from(r0_mohm);
    let ratio = (u64::from(code) * u64::from(rref_mohm) * 1_000_000 + r0 * (1 << 14)) / (r0 << 15);
    let ratio = u32::try_from(ratio).map_err(|_| Error::InvalidData)?;
    Ok(Measurement {
        temperature: temperature_from_ratio(ratio).ok_or(Error::InvalidData)?,
        resistance_mohm: resistance_mohm(code, rref_mohm),
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    None,
    OneShot,
    Auto,
}

macro_rules! constructors {
    () => {
        /// PT100 with a 430 Ω reference, 4-wire, 60 Hz filter. No bus
        /// traffic.
        pub const fn new(spi: SPI) -> Self {
            Self {
                spi,
                r0_mohm: 100_000,
                rref_mohm: 430_000,
                wiring: Wiring::FourWire,
                filter: Filter::Hz60,
                settle_us: 11_500,
                bias_on: false,
                pending: Pending::None,
            }
        }

        /// RTD nominal resistance at 0 °C and reference resistor, in mΩ
        /// (PT1000 with 4.3 kΩ: `with_rtd(1_000_000, 4_300_000)`).
        pub const fn with_rtd(mut self, r0_mohm: u32, rref_mohm: u32) -> Self {
            self.r0_mohm = r0_mohm;
            self.rref_mohm = rref_mohm;
            self
        }

        /// RTD connection.
        pub const fn with_wiring(mut self, wiring: Wiring) -> Self {
            self.wiring = wiring;
            self
        }

        /// Notch filter.
        pub const fn with_filter(mut self, filter: Filter) -> Self {
            self.filter = filter;
            self
        }

        /// Bias settling time: 10.5 × RC of the input filter + 1 ms.
        pub const fn with_bias_settle_us(mut self, settle_us: WaitUs) -> Self {
            self.settle_us = settle_us;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> SPI {
            self.spi
        }

        /// RTD code for a resistance in mΩ with this driver's reference
        /// resistor (for `set_fault_thresholds`).
        pub const fn code_for_resistance(&self, mohm: u32) -> u16 {
            let code = ((mohm as u64) << 15) / self.rref_mohm as u64;
            if code > 0x7FFF { 0x7FFF } else { code as u16 }
        }

        /// Wiring and filter bits of the configuration register.
        const fn base(&self) -> u8 {
            let wire = match self.wiring {
                Wiring::ThreeWire => cfg::THREE_WIRE,
                _ => 0,
            };
            let filter = match self.filter {
                Filter::Hz50 => cfg::FILTER_50HZ,
                Filter::Hz60 => 0,
            };
            wire | filter
        }

        const fn bias_bit(&self) -> u8 {
            if self.bias_on { cfg::VBIAS } else { 0 }
        }

        /// Wait for the automatic-mode start: the first conversion takes the
        /// single-conversion time, the next ones the continuous period; one
        /// full conversion must start after the bias settled.
        const fn auto_wait_us(&self) -> WaitUs {
            let single = self.filter.single_us();
            let period = self.filter.continuous_us();
            let mut start = single;
            while start < self.settle_us {
                start += period;
            }
            start + period
        }
    };
}

/// Blocking driver.
pub struct Max31865<SPI> {
    spi: SPI,
    r0_mohm: u32,
    rref_mohm: u32,
    wiring: Wiring,
    filter: Filter,
    settle_us: WaitUs,
    bias_on: bool,
    pending: Pending,
}

impl<SPI> Max31865<SPI> {
    constructors!();
}

impl<SPI: embedded_hal::spi::SpiDevice> Max31865<SPI> {
    fn write_reg(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        use embedded_hal::spi::Operation;
        self.spi
            .transaction(&mut [
                Operation::Write(&[reg::WRITE | reg]),
                Operation::Write(data),
            ])
            .map_err(Error::bus)
    }

    fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        use embedded_hal::spi::Operation;
        self.spi
            .transaction(&mut [Operation::Write(&[reg]), Operation::Read(buf)])
            .map_err(Error::bus)
    }

    fn write_config(&mut self, bits: u8) -> Result<(), Error<SPI::Error>> {
        let value = self.base() | bits;
        self.write_reg(reg::CONFIG, &[value])
    }

    /// Write the configuration (wiring, filter; bias and conversions off).
    /// Change the filter only with conversions stopped.
    pub fn configure(&mut self) -> Result<(), Error<SPI::Error>> {
        self.bias_on = false;
        self.pending = Pending::None;
        self.write_config(0)
    }

    /// Switch bias on and keep it on; wait the returned settling time before
    /// `start_measurement`, which then uses 1-shot conversions.
    pub fn enable_bias(&mut self) -> Result<WaitUs, Error<SPI::Error>> {
        self.write_config(cfg::VBIAS)?;
        self.bias_on = true;
        Ok(self.settle_us)
    }

    /// Switch bias and conversions off.
    pub fn disable_bias(&mut self) -> Result<(), Error<SPI::Error>> {
        self.configure()
    }

    /// Read the configuration register.
    pub fn read_config(&mut self) -> Result<u8, Error<SPI::Error>> {
        let mut buf = [0u8];
        self.read_reg(reg::CONFIG, &mut buf)?;
        Ok(buf[0])
    }

    /// Read the fault status register.
    pub fn read_fault_status(&mut self) -> Result<FaultStatus, Error<SPI::Error>> {
        let mut buf = [0u8];
        self.read_reg(reg::FAULT_STATUS, &mut buf)?;
        Ok(FaultStatus(buf[0]))
    }

    /// Clear the fault status register (keeps the bias state).
    pub fn clear_faults(&mut self) -> Result<(), Error<SPI::Error>> {
        let bias = self.bias_bit();
        self.write_config(bias | cfg::FAULT_CLEAR)
    }

    /// Start the automatic fault-detection cycle (bias on, `100X010X`).
    /// After the returned wait, check [`Self::fault_detection_done`] and
    /// read [`Self::read_fault_status`]. Bias stays on afterwards.
    pub fn start_fault_detection(&mut self) -> Result<WaitUs, Error<SPI::Error>> {
        self.pending = Pending::None;
        self.write_config(cfg::VBIAS | cfg::FAULT_AUTO)?;
        self.bias_on = true;
        Ok(FAULT_DETECTION_US)
    }

    /// The fault-detection cycle has finished (D3:2 back to 00).
    pub fn fault_detection_done(&mut self) -> Result<bool, Error<SPI::Error>> {
        Ok(self.read_config()? & cfg::FAULT_CYCLE == 0)
    }

    /// Set the low / high fault thresholds as 15-bit RTD codes.
    pub fn set_fault_thresholds(&mut self, low: u16, high: u16) -> Result<(), Error<SPI::Error>> {
        let [hh, hl] = (high << 1).to_be_bytes();
        let [lh, ll] = (low << 1).to_be_bytes();
        self.write_reg(reg::HIGH_FAULT_MSB, &[hh, hl, lh, ll])
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> zigbee_sensor::blocking::Sensor for Max31865<SPI> {
    type BusError = SPI::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<SPI::Error>> {
        if self.bias_on {
            self.write_config(cfg::VBIAS | cfg::ONE_SHOT)?;
            self.pending = Pending::OneShot;
            Ok(self.filter.single_us())
        } else {
            self.write_config(cfg::VBIAS | cfg::AUTO)?;
            self.pending = Pending::Auto;
            Ok(self.auto_wait_us())
        }
    }

    /// Reads the RTD registers; stops automatic mode and bias if
    /// `start_measurement` enabled them. A set fault bit is
    /// [`Error::InvalidData`]; see [`Max31865::read_fault_status`].
    fn read_measurement(&mut self) -> Result<Measurement, Error<SPI::Error>> {
        let mut buf = [0u8; 2];
        self.read_reg(reg::RTD_MSB, &mut buf)?;
        if self.pending == Pending::Auto {
            self.configure()?;
        }
        self.pending = Pending::None;
        decode(buf, self.rref_mohm, self.r0_mohm)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        FAULT_DETECTION_US, FaultStatus, Filter, Measurement, Pending, Wiring, cfg, decode, reg,
    };
    use embedded_hal_async::spi::{Operation, SpiDevice};
    use zigbee_sensor::{Error, WaitUs};

    /// Async driver; same API as [`super::Max31865`].
    pub struct Max31865<SPI> {
        spi: SPI,
        r0_mohm: u32,
        rref_mohm: u32,
        wiring: Wiring,
        filter: Filter,
        settle_us: WaitUs,
        bias_on: bool,
        pending: Pending,
    }

    impl<SPI> Max31865<SPI> {
        constructors!();
    }

    impl<SPI: SpiDevice> Max31865<SPI> {
        async fn write_reg(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<SPI::Error>> {
            self.spi
                .transaction(&mut [
                    Operation::Write(&[reg::WRITE | reg]),
                    Operation::Write(data),
                ])
                .await
                .map_err(Error::bus)
        }

        async fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
            self.spi
                .transaction(&mut [Operation::Write(&[reg]), Operation::Read(buf)])
                .await
                .map_err(Error::bus)
        }

        async fn write_config(&mut self, bits: u8) -> Result<(), Error<SPI::Error>> {
            let value = self.base() | bits;
            self.write_reg(reg::CONFIG, &[value]).await
        }

        /// See [`super::Max31865::configure`].
        pub async fn configure(&mut self) -> Result<(), Error<SPI::Error>> {
            self.bias_on = false;
            self.pending = Pending::None;
            self.write_config(0).await
        }

        /// See [`super::Max31865::enable_bias`].
        pub async fn enable_bias(&mut self) -> Result<WaitUs, Error<SPI::Error>> {
            self.write_config(cfg::VBIAS).await?;
            self.bias_on = true;
            Ok(self.settle_us)
        }

        /// See [`super::Max31865::disable_bias`].
        pub async fn disable_bias(&mut self) -> Result<(), Error<SPI::Error>> {
            self.configure().await
        }

        /// See [`super::Max31865::read_config`].
        pub async fn read_config(&mut self) -> Result<u8, Error<SPI::Error>> {
            let mut buf = [0u8];
            self.read_reg(reg::CONFIG, &mut buf).await?;
            Ok(buf[0])
        }

        /// See [`super::Max31865::read_fault_status`].
        pub async fn read_fault_status(&mut self) -> Result<FaultStatus, Error<SPI::Error>> {
            let mut buf = [0u8];
            self.read_reg(reg::FAULT_STATUS, &mut buf).await?;
            Ok(FaultStatus(buf[0]))
        }

        /// See [`super::Max31865::clear_faults`].
        pub async fn clear_faults(&mut self) -> Result<(), Error<SPI::Error>> {
            let bias = self.bias_bit();
            self.write_config(bias | cfg::FAULT_CLEAR).await
        }

        /// See [`super::Max31865::start_fault_detection`].
        pub async fn start_fault_detection(&mut self) -> Result<WaitUs, Error<SPI::Error>> {
            self.pending = Pending::None;
            self.write_config(cfg::VBIAS | cfg::FAULT_AUTO).await?;
            self.bias_on = true;
            Ok(FAULT_DETECTION_US)
        }

        /// See [`super::Max31865::fault_detection_done`].
        pub async fn fault_detection_done(&mut self) -> Result<bool, Error<SPI::Error>> {
            Ok(self.read_config().await? & cfg::FAULT_CYCLE == 0)
        }

        /// See [`super::Max31865::set_fault_thresholds`].
        pub async fn set_fault_thresholds(
            &mut self,
            low: u16,
            high: u16,
        ) -> Result<(), Error<SPI::Error>> {
            let [hh, hl] = (high << 1).to_be_bytes();
            let [lh, ll] = (low << 1).to_be_bytes();
            self.write_reg(reg::HIGH_FAULT_MSB, &[hh, hl, lh, ll]).await
        }
    }

    impl<SPI: SpiDevice> zigbee_sensor::asynch::Sensor for Max31865<SPI> {
        type BusError = SPI::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<SPI::Error>> {
            if self.bias_on {
                self.write_config(cfg::VBIAS | cfg::ONE_SHOT).await?;
                self.pending = Pending::OneShot;
                Ok(self.filter.single_us())
            } else {
                self.write_config(cfg::VBIAS | cfg::AUTO).await?;
                self.pending = Pending::Auto;
                Ok(self.auto_wait_us())
            }
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<SPI::Error>> {
            let mut buf = [0u8; 2];
            self.read_reg(reg::RTD_MSB, &mut buf).await?;
            if self.pending == Pending::Auto {
                self.configure().await?;
            }
            self.pending = Pending::None;
            decode(buf, self.rref_mohm, self.r0_mohm)
        }
    }
}

#[cfg(test)]
mod tests;
