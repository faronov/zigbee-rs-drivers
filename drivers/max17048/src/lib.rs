//! Maxim (Analog Devices) MAX17043/MAX17044 and MAX17048/MAX17049
//! ModelGauge fuel gauges for 1-cell (MAX17043/48) and 2-cell
//! (MAX17044/49) Li-ion packs, I2C address 0x36.
//!
//! The gauge measures VCELL continuously (every 250 ms on the MAX17048,
//! 500 ms on the MAX17043; every 45 s in MAX17048 hibernate) and updates SOC
//! by itself, so `start_measurement` returns 0 and `read_measurement` reads
//! VCELL, SOC and (MAX17048/49) CRATE.
//!
//! MAX17048/49 only: hibernate thresholds (HIBRT), voltage alerts (VALRT),
//! CRATE, VRESET/ID, STATUS and the SOC-change alert. The MAX17043/44 have
//! VCELL, SOC, MODE, VERSION, CONFIG and COMMAND only.
//!
//! References: MAX17048/MAX17049 data sheet 19-6171 Rev 5 (2018);
//! MAX17043/MAX17044 data sheet 19-4703 Rev 5.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address (0x6C/0x6D 8-bit).
pub const ADDRESS: u8 = 0x36;

/// Register addresses (16-bit, big-endian).
pub mod reg {
    /// VCELL.
    pub const VCELL: u8 = 0x02;
    /// SOC.
    pub const SOC: u8 = 0x04;
    /// MODE (write only).
    pub const MODE: u8 = 0x06;
    /// VERSION.
    pub const VERSION: u8 = 0x08;
    /// HIBRT (MAX17048/49).
    pub const HIBRT: u8 = 0x0A;
    /// CONFIG (RCOMP, SLEEP, ALSC, ALRT, ATHD).
    pub const CONFIG: u8 = 0x0C;
    /// VALRT (MAX17048/49).
    pub const VALRT: u8 = 0x14;
    /// CRATE (MAX17048/49).
    pub const CRATE: u8 = 0x16;
    /// VRESET/ID (MAX17048/49).
    pub const VRESET_ID: u8 = 0x18;
    /// STATUS (MAX17048/49).
    pub const STATUS: u8 = 0x1A;
    /// CMD / COMMAND.
    pub const CMD: u8 = 0xFE;
}

/// MODE: quick-start.
pub const MODE_QUICK_START: u16 = 0x4000;
/// MODE: enable sleep (MAX17048/49).
pub const MODE_EN_SLEEP: u16 = 0x2000;
/// MODE: hibernate status (MAX17048/49, read only).
pub const MODE_HIB_STAT: u16 = 0x1000;

/// CONFIG: force sleep.
pub const CONFIG_SLEEP: u16 = 1 << 7;
/// CONFIG: alert on 1 % SOC change (MAX17048/49).
pub const CONFIG_ALSC: u16 = 1 << 6;
/// CONFIG: alert flag (write 0 to release the ALRT pin).
pub const CONFIG_ALRT: u16 = 1 << 5;

/// STATUS: reset indicator (device not configured).
pub const STATUS_RI: u16 = 1 << 8;
/// STATUS: VCELL above VALRT.MAX.
pub const STATUS_VH: u16 = 1 << 9;
/// STATUS: VCELL below VALRT.MIN.
pub const STATUS_VL: u16 = 1 << 10;
/// STATUS: voltage reset.
pub const STATUS_VR: u16 = 1 << 11;
/// STATUS: SOC below CONFIG.ATHD.
pub const STATUS_HD: u16 = 1 << 12;
/// STATUS: SOC changed by 1 %.
pub const STATUS_SC: u16 = 1 << 13;
/// STATUS: enable voltage-reset alert.
pub const STATUS_ENVR: u16 = 1 << 14;

/// HIBRT power-on value (HibThr 0x80, ActThr 0x30).
pub const HIBRT_DEFAULT: u16 = 0x8030;
/// RCOMP power-on value.
pub const RCOMP_DEFAULT: u8 = 0x97;

/// Supported parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// MAX17043, 1 cell, 12-bit VCELL (1.25 mV).
    Max17043,
    /// MAX17044, 2 cells, 12-bit VCELL (2.5 mV).
    Max17044,
    /// MAX17048, 1 cell (78.125 µV).
    #[default]
    Max17048,
    /// MAX17049, 2 cells (78.125 µV per cell).
    Max17049,
}

impl Variant {
    /// Cells in series.
    pub const fn cells(self) -> i32 {
        match self {
            Self::Max17043 | Self::Max17048 => 1,
            Self::Max17044 | Self::Max17049 => 2,
        }
    }

    /// MAX17048/49 register set (HIBRT, VALRT, CRATE, STATUS, …).
    pub const fn is_max1704x8(self) -> bool {
        matches!(self, Self::Max17048 | Self::Max17049)
    }

    const fn por_command(self) -> u16 {
        if self.is_max1704x8() { 0x5400 } else { 0x0054 }
    }

    /// Time until the first SOC update after POR: 1 s (MAX17048) or
    /// 250 ms (MAX17043).
    pub const fn por_wait_us(self) -> WaitUs {
        if self.is_max1704x8() {
            1_000_000
        } else {
            250_000
        }
    }

    /// Time until VCELL is updated after leaving sleep: one ADC period
    /// (250 ms) on the MAX17048, 500 ms on the MAX17043.
    pub const fn wake_wait_us(self) -> WaitUs {
        if self.is_max1704x8() {
            250_000
        } else {
            500_000
        }
    }
}

/// Pack voltage in mV from VCELL (78.125 µV per cell LSB; the 12-bit
/// MAX17043/44 value in bits 15..4 has the same scale).
pub const fn vcell_millivolts(raw: u16, variant: Variant) -> i32 {
    ((raw as i32 * 625 * variant.cells()) + 4_000) / 8_000
}

/// SOC (1/256 % LSB) in 0.5 % units, clamped to 200.
pub const fn soc_half_percent(raw: u16) -> u8 {
    let half = (raw as u32 + 64) >> 7;
    if half > 200 { 200 } else { half as u8 }
}

/// CRATE (0.208 %/h LSB, signed) in 0.001 %/h.
pub const fn crate_milli_percent_per_hour(raw: u16) -> i32 {
    raw as i16 as i32 * 208
}

/// Temperature-compensated RCOMP (data sheet "Temperature
/// Compensation"): `RCOMP0 + (T − 20) × TempCoUp` above 20 °C, else
/// `TempCoDown`. Temperature in 0.01 °C, coefficients in 0.001/°C
/// (defaults −500 and −5000). Clamped to 0..=255.
pub const fn compensated_rcomp(
    rcomp0: u8,
    tempco_up_milli: i32,
    tempco_down_milli: i32,
    temperature_centi: i32,
) -> u8 {
    let dt = temperature_centi - 2_000;
    let co = if dt > 0 {
        tempco_up_milli
    } else {
        tempco_down_milli
    };
    // dt [0.01 °C] × co [0.001/°C] → 1e-5 RCOMP units.
    let delta = dt as i64 * co as i64;
    let delta = if delta >= 0 {
        (delta + 50_000) / 100_000
    } else {
        (delta - 50_000) / 100_000
    };
    let r = rcomp0 as i64 + delta;
    if r < 0 {
        0
    } else if r > 255 {
        255
    } else {
        r as u8
    }
}

/// One reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Pack voltage in mV.
    pub voltage: i32,
    /// State of charge in 0.5 % units (0..=200).
    pub percent: u8,
    /// SOC rate of change in 0.001 %/h (MAX17048/49 only).
    pub charge_rate: Option<i32>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Voltage(self.voltage));
        f(Quantity::BatteryPercent(self.percent));
    }
}

fn valrt<E>(min_mv: u16, max_mv: u16) -> Result<u16, Error<E>> {
    if min_mv > 5_100 || max_mv > 5_100 {
        return Err(Error::InvalidArgument);
    }
    Ok(u16::from_be_bytes([
        (min_mv / 20) as u8,
        (max_mv / 20) as u8,
    ]))
}

/// CONFIG register value from RCOMP, empty-alert threshold (clamped to
/// 1…32 %) and ALSC.
pub const fn config_value(rcomp: u8, empty_alert_percent: u8, soc_change_alert: bool) -> u16 {
    let pct = if empty_alert_percent < 1 {
        1
    } else if empty_alert_percent > 32 {
        32
    } else {
        empty_alert_percent
    };
    ((rcomp as u16) << 8) | if soc_change_alert { CONFIG_ALSC } else { 0 } | (32 - pct) as u16
}

macro_rules! constructors {
    ($name:ident) => {
        impl<I2C> $name<I2C> {
            /// Create a driver; no bus traffic.
            pub const fn new(i2c: I2C, variant: Variant) -> Self {
                Self {
                    i2c,
                    variant,
                    config: config_value(RCOMP_DEFAULT, 4, false),
                }
            }

            /// RCOMP written by `init` (from the custom battery model).
            pub const fn with_rcomp(mut self, rcomp: u8) -> Self {
                self.config = (self.config & 0x00FF) | ((rcomp as u16) << 8);
                self
            }

            /// Empty alert threshold 1..=32 % (default 4 %) written by
            /// `init`.
            pub const fn with_empty_alert(mut self, percent: u8) -> Self {
                self.config = config_value(
                    (self.config >> 8) as u8,
                    percent,
                    self.config & CONFIG_ALSC != 0,
                );
                self
            }

            /// Enable the 1 % SOC-change alert (MAX17048/49).
            pub const fn with_soc_change_alert(mut self, enable: bool) -> Self {
                self.config = if enable {
                    self.config | CONFIG_ALSC
                } else {
                    self.config & !CONFIG_ALSC
                };
                self
            }

            /// Configured variant.
            pub const fn variant(&self) -> Variant {
                self.variant
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Max17048<I2C> {
    i2c: I2C,
    variant: Variant,
    config: u16,
}

constructors!(Max17048);

impl<I2C: embedded_hal::i2c::I2c> Max17048<I2C> {
    /// Read a 16-bit register.
    pub fn read_register(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0; 2];
        self.i2c
            .write_read(ADDRESS, &[reg], &mut buf)
            .map_err(Error::bus)?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Write a 16-bit register.
    pub fn write_register(&mut self, reg: u8, value: u16) -> Result<(), Error<I2C::Error>> {
        let [m, l] = value.to_be_bytes();
        self.i2c.write(ADDRESS, &[reg, m, l]).map_err(Error::bus)
    }

    /// Write CONFIG (RCOMP, alert settings; clears ALRT) and, on the
    /// MAX17048/49, clear STATUS.RI. Returns whether RI was set (the
    /// gauge was reset and any custom model must be reloaded).
    pub fn init(&mut self) -> Result<bool, Error<I2C::Error>> {
        self.write_register(reg::CONFIG, self.config)?;
        if !self.variant.is_max1704x8() {
            return Ok(false);
        }
        let status = self.read_register(reg::STATUS)?;
        if status & STATUS_RI != 0 {
            self.write_register(reg::STATUS, status & !STATUS_RI)?;
        }
        Ok(status & STATUS_RI != 0)
    }

    /// VERSION register.
    pub fn version(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_register(reg::VERSION)
    }

    /// Restart the SOC estimate from the present cell voltage.
    pub fn quick_start(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::MODE, MODE_QUICK_START)
    }

    /// Power-on reset command. The IC does not ACK it, so bus errors are
    /// ignored. Wait the returned time, then call `init`.
    pub fn reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        let [m, l] = self.variant.por_command().to_be_bytes();
        let _ = self.i2c.write(ADDRESS, &[reg::CMD, m, l]);
        Ok(self.variant.por_wait_us())
    }

    /// Update RCOMP, keeping the other CONFIG bits.
    pub fn set_rcomp(&mut self, rcomp: u8) -> Result<(), Error<I2C::Error>> {
        let config = self.read_register(reg::CONFIG)?;
        self.config = (self.config & 0x00FF) | ((rcomp as u16) << 8);
        self.write_register(reg::CONFIG, (config & 0x00FF) | ((rcomp as u16) << 8))
    }

    /// Clear CONFIG.ALRT (and the given STATUS bits on MAX17048/49).
    pub fn clear_alert(&mut self, status_bits: u16) -> Result<(), Error<I2C::Error>> {
        if self.variant.is_max1704x8() && status_bits != 0 {
            let status = self.read_register(reg::STATUS)?;
            self.write_register(reg::STATUS, status & !status_bits)?;
        }
        let config = self.read_register(reg::CONFIG)?;
        self.write_register(reg::CONFIG, config & !CONFIG_ALRT)
    }

    /// STATUS register (MAX17048/49).
    pub fn status(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.require_max17048()?;
        self.read_register(reg::STATUS)
    }

    /// Hibernate thresholds (MAX17048/49): HibThr in 0.208 %/h, ActThr in
    /// 1.25 mV. `(0, 0)` disables hibernate, `(0xFF, 0xFF)` forces it.
    pub fn set_hibernate(&mut self, hib_thr: u8, act_thr: u8) -> Result<(), Error<I2C::Error>> {
        self.require_max17048()?;
        self.write_register(reg::HIBRT, u16::from_be_bytes([hib_thr, act_thr]))
    }

    /// VCELL alert window in mV per cell (20 mV steps, MAX17048/49);
    /// [`Error::InvalidArgument`] above 5100 mV.
    pub fn set_voltage_alert(&mut self, min_mv: u16, max_mv: u16) -> Result<(), Error<I2C::Error>> {
        self.require_max17048()?;
        let valrt = valrt(min_mv, max_mv)?;
        self.write_register(reg::VALRT, valrt)
    }

    fn require_max17048(&self) -> Result<(), Error<I2C::Error>> {
        if self.variant.is_max1704x8() {
            Ok(())
        } else {
            Err(Error::Unsupported)
        }
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Max17048<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// The gauge runs autonomously; nothing to start.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        Ok(0)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let vcell = self.read_register(reg::VCELL)?;
        let soc = self.read_register(reg::SOC)?;
        let charge_rate = if self.variant.is_max1704x8() {
            Some(crate_milli_percent_per_hour(
                self.read_register(reg::CRATE)?,
            ))
        } else {
            None
        };
        Ok(Measurement {
            voltage: vcell_millivolts(vcell, self.variant),
            percent: soc_half_percent(soc),
            charge_rate,
        })
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Max17048<I2C> {
    type BusError = I2C::Error;

    /// Sleep (< 1 µA, MAX17048: 0.5 µA typ): MODE.EnSleep (MAX17048/49)
    /// then CONFIG.SLEEP. The gauge does not track SOC while asleep.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        if self.variant.is_max1704x8() {
            self.write_register(reg::MODE, MODE_EN_SLEEP)?;
        }
        let config = self.read_register(reg::CONFIG)?;
        self.write_register(reg::CONFIG, config | CONFIG_SLEEP)
    }

    /// Clear CONFIG.SLEEP (and EnSleep); returns one VCELL update period.
    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        let config = self.read_register(reg::CONFIG)?;
        self.write_register(reg::CONFIG, config & !CONFIG_SLEEP)?;
        if self.variant.is_max1704x8() {
            self.write_register(reg::MODE, 0)?;
        }
        Ok(self.variant.wake_wait_us())
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, CONFIG_ALRT, CONFIG_ALSC, CONFIG_SLEEP, MODE_EN_SLEEP, MODE_QUICK_START,
        Measurement, RCOMP_DEFAULT, STATUS_RI, Variant, config_value, crate_milli_percent_per_hour,
        reg, soc_half_percent, valrt, vcell_millivolts,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Max17048`].
    #[derive(Debug)]
    pub struct Max17048<I2C> {
        i2c: I2C,
        variant: Variant,
        config: u16,
    }

    constructors!(Max17048);

    impl<I2C: I2c> Max17048<I2C> {
        /// See [`super::Max17048::read_register`].
        pub async fn read_register(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
            let mut buf = [0; 2];
            self.i2c
                .write_read(ADDRESS, &[reg], &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(u16::from_be_bytes(buf))
        }

        /// See [`super::Max17048::write_register`].
        pub async fn write_register(
            &mut self,
            reg: u8,
            value: u16,
        ) -> Result<(), Error<I2C::Error>> {
            let [m, l] = value.to_be_bytes();
            self.i2c
                .write(ADDRESS, &[reg, m, l])
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Max17048::init`].
        pub async fn init(&mut self) -> Result<bool, Error<I2C::Error>> {
            self.write_register(reg::CONFIG, self.config).await?;
            if !self.variant.is_max1704x8() {
                return Ok(false);
            }
            let status = self.read_register(reg::STATUS).await?;
            if status & STATUS_RI != 0 {
                self.write_register(reg::STATUS, status & !STATUS_RI)
                    .await?;
            }
            Ok(status & STATUS_RI != 0)
        }

        /// See [`super::Max17048::version`].
        pub async fn version(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.read_register(reg::VERSION).await
        }

        /// See [`super::Max17048::quick_start`].
        pub async fn quick_start(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::MODE, MODE_QUICK_START).await
        }

        /// See [`super::Max17048::reset`].
        pub async fn reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let [m, l] = self.variant.por_command().to_be_bytes();
            let _ = self.i2c.write(ADDRESS, &[reg::CMD, m, l]).await;
            Ok(self.variant.por_wait_us())
        }

        /// See [`super::Max17048::set_rcomp`].
        pub async fn set_rcomp(&mut self, rcomp: u8) -> Result<(), Error<I2C::Error>> {
            let config = self.read_register(reg::CONFIG).await?;
            self.config = (self.config & 0x00FF) | ((rcomp as u16) << 8);
            self.write_register(reg::CONFIG, (config & 0x00FF) | ((rcomp as u16) << 8))
                .await
        }

        /// See [`super::Max17048::clear_alert`].
        pub async fn clear_alert(&mut self, status_bits: u16) -> Result<(), Error<I2C::Error>> {
            if self.variant.is_max1704x8() && status_bits != 0 {
                let status = self.read_register(reg::STATUS).await?;
                self.write_register(reg::STATUS, status & !status_bits)
                    .await?;
            }
            let config = self.read_register(reg::CONFIG).await?;
            self.write_register(reg::CONFIG, config & !CONFIG_ALRT)
                .await
        }

        /// See [`super::Max17048::status`].
        pub async fn status(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.require_max17048()?;
            self.read_register(reg::STATUS).await
        }

        /// See [`super::Max17048::set_hibernate`].
        pub async fn set_hibernate(
            &mut self,
            hib_thr: u8,
            act_thr: u8,
        ) -> Result<(), Error<I2C::Error>> {
            self.require_max17048()?;
            self.write_register(reg::HIBRT, u16::from_be_bytes([hib_thr, act_thr]))
                .await
        }

        /// See [`super::Max17048::set_voltage_alert`].
        pub async fn set_voltage_alert(
            &mut self,
            min_mv: u16,
            max_mv: u16,
        ) -> Result<(), Error<I2C::Error>> {
            self.require_max17048()?;
            let valrt = valrt(min_mv, max_mv)?;
            self.write_register(reg::VALRT, valrt).await
        }

        fn require_max17048(&self) -> Result<(), Error<I2C::Error>> {
            if self.variant.is_max1704x8() {
                Ok(())
            } else {
                Err(Error::Unsupported)
            }
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Max17048<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let vcell = self.read_register(reg::VCELL).await?;
            let soc = self.read_register(reg::SOC).await?;
            let charge_rate = if self.variant.is_max1704x8() {
                Some(crate_milli_percent_per_hour(
                    self.read_register(reg::CRATE).await?,
                ))
            } else {
                None
            };
            Ok(Measurement {
                voltage: vcell_millivolts(vcell, self.variant),
                percent: soc_half_percent(soc),
                charge_rate,
            })
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Max17048<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            if self.variant.is_max1704x8() {
                self.write_register(reg::MODE, MODE_EN_SLEEP).await?;
            }
            let config = self.read_register(reg::CONFIG).await?;
            self.write_register(reg::CONFIG, config | CONFIG_SLEEP)
                .await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let config = self.read_register(reg::CONFIG).await?;
            self.write_register(reg::CONFIG, config & !CONFIG_SLEEP)
                .await?;
            if self.variant.is_max1704x8() {
                self.write_register(reg::MODE, 0).await?;
            }
            Ok(self.variant.wake_wait_us())
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as T};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    fn rd(reg: u8, value: u16) -> T {
        T::write_read(ADDRESS, vec![reg], value.to_be_bytes().to_vec())
    }

    fn wr(reg: u8, value: u16) -> T {
        let [m, l] = value.to_be_bytes();
        T::write(ADDRESS, vec![reg, m, l])
    }

    #[test]
    fn conversions() {
        // 3.7 V = 47360 × 78.125 µV.
        assert_eq!(vcell_millivolts(47_360, Variant::Max17048), 3_700);
        assert_eq!(vcell_millivolts(47_360, Variant::Max17049), 7_400);
        // MAX17043: 0xB900 → 2960 × 1.25 mV; MAX17044: × 2.5 mV.
        assert_eq!(vcell_millivolts(0xB900, Variant::Max17043), 3_700);
        assert_eq!(vcell_millivolts(0xB900, Variant::Max17044), 7_400);
        assert_eq!(soc_half_percent(0x4B80), 151);
        assert_eq!(soc_half_percent(0x6600), 200);
        assert_eq!(crate_milli_percent_per_hour(0xFFF6), -2_080);
        assert_eq!(config_value(0x97, 4, false), 0x971C);
        assert_eq!(config_value(0x97, 32, true), 0x9740);
        assert_eq!(compensated_rcomp(0x97, -500, -5_000, 3_000), 0x97 - 5);
        assert_eq!(compensated_rcomp(0x97, -500, -5_000, 0), 0x97 + 100);
        assert_eq!(compensated_rcomp(0x97, -500, -5_000, -4_000), 255);
    }

    #[test]
    fn init_measure_sleep_max17048() {
        let expectations = [
            wr(reg::CONFIG, 0x8041),
            rd(reg::STATUS, 0x0100),
            wr(reg::STATUS, 0x0000),
            rd(reg::VCELL, 47_360),
            rd(reg::SOC, 0x3200),
            rd(reg::CRATE, 0x0005),
            wr(reg::MODE, MODE_EN_SLEEP),
            rd(reg::CONFIG, 0x8041),
            wr(reg::CONFIG, 0x80C1),
            rd(reg::CONFIG, 0x80C1),
            wr(reg::CONFIG, 0x8041),
            wr(reg::MODE, 0),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = Max17048::new(&mut i2c, Variant::Max17048)
            .with_rcomp(0x80)
            .with_soc_change_alert(true)
            .with_empty_alert(31);
        assert_eq!(gauge.init(), Ok(true));
        assert_eq!(gauge.start_measurement(), Ok(0));
        assert_eq!(
            gauge.read_measurement().unwrap(),
            Measurement {
                voltage: 3_700,
                percent: 100,
                charge_rate: Some(1_040),
            }
        );
        gauge.sleep().unwrap();
        assert_eq!(gauge.wake(), Ok(250_000));
        i2c.done();
    }

    #[test]
    fn max17043_subset() {
        let expectations = [
            wr(reg::CONFIG, 0x971C),
            rd(reg::VCELL, 0xB900),
            rd(reg::SOC, 0x6400),
            rd(reg::CONFIG, 0x971C),
            wr(reg::CONFIG, 0x979C),
            T::write(ADDRESS, vec![reg::CMD, 0x00, 0x54]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = Max17048::new(&mut i2c, Variant::Max17043);
        assert_eq!(gauge.init(), Ok(false));
        let m = gauge.read_measurement().unwrap();
        assert_eq!((m.voltage, m.percent, m.charge_rate), (3_700, 200, None));
        assert_eq!(gauge.status(), Err(Error::Unsupported));
        gauge.sleep().unwrap();
        assert_eq!(gauge.reset(), Ok(250_000));
        i2c.done();
    }

    #[test]
    fn alerts_and_thresholds() {
        let expectations = [
            wr(reg::HIBRT, 0x0000),
            wr(reg::VALRT, 0xAFD2),
            rd(reg::STATUS, 0x1100),
            wr(reg::STATUS, 0x0100),
            rd(reg::CONFIG, 0x973C),
            wr(reg::CONFIG, 0x971C),
            rd(reg::CONFIG, 0x971C),
            wr(reg::CONFIG, 0x701C),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = Max17048::new(&mut i2c, Variant::Max17049);
        gauge.set_hibernate(0, 0).unwrap();
        gauge.set_voltage_alert(3_500, 4_200).unwrap();
        assert_eq!(
            gauge.set_voltage_alert(3_500, 5_120),
            Err(Error::InvalidArgument)
        );
        gauge.clear_alert(STATUS_HD).unwrap();
        gauge.set_rcomp(0x70).unwrap();
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
            rd(reg::VCELL, 47_360),
            rd(reg::SOC, 0x1980),
            rd(reg::CRATE, 0xFFFF),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = asynch::Max17048::new(&mut i2c, Variant::Max17049);
        let mut delay = RecordingDelay(0);
        let m = block_on(gauge.measure(&mut delay)).unwrap();
        assert_eq!(
            m,
            Measurement {
                voltage: 7_400,
                percent: 51,
                charge_rate: Some(-208),
            }
        );
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
