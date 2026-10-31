//! Phosense XBR818 10.525 GHz radar presence-detector SoC (I2C
//! configuration, presence on the IO_VAL pin), e.g. Ai-Thinker Rd-04.
//!
//! References:
//! * Phosense "XBR818 Datasheet" Rev 2.3 (2023-03-15) — hardware only
//! * Ai-Thinker "Rd-04 Module Manual" V1.0.0 (2023-06-30) §6 — register map
//!
//! * I2C address 0x71 (fixed); no ID register, so there is no `probe()`.
//! * The I2C block is only powered while the I2C_EN pin is high; drive it
//!   high ≥ 10 µs ([`I2C_ENABLE_WAIT_US`]) before configuring and low
//!   afterwards to save power. The detector keeps running either way.
//! * Registers are volatile: call [`Xbr818::configure`] after every
//!   power-up. Bit 0 of 0x13 (detection enable) is cleared first and set
//!   last so the new parameters are latched.
//! * Presence is the level of the IO_VAL pin (active high with
//!   [`Output::ActiveHigh`]); the read-only detection registers are not
//!   fully documented and are not used.
//!
//! Delay/lock timers count a ~32 kHz RC clock ([`ms_to_ticks`]); its
//! frequency is trimmed by register 0x06, so times are approximate.

#![no_std]

use embedded_hal::digital::InputPin;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// 7-bit I2C address.
pub const ADDRESS: u8 = 0x71;
/// Wait after driving I2C_EN high before the first transaction.
pub const I2C_ENABLE_WAIT_US: WaitUs = 10;

const REG_RF_POWER: u8 = 0x03;
const REG_SAMPLE_DIV: u8 = 0x10;
const REG_DETECT_CTL: u8 = 0x13;
const REG_THRESHOLD: u8 = 0x18;
const REG_DELAY_TIME: u8 = 0x1D;
const REG_PIN_CTL: u8 = 0x23;

/// Timer clock, Hz.
pub const TIMER_HZ: u32 = 32_000;
/// Largest 24-bit timer value in ms (≈ 524 s).
pub const MAX_TIMER_MS: u32 = 0x00FF_FFFF / (TIMER_HZ / 1_000);

/// Convert ms to 24-bit timer ticks (1/32000 s), saturating.
pub const fn ms_to_ticks(ms: u32) -> u32 {
    if ms > MAX_TIMER_MS {
        0x00FF_FFFF
    } else {
        ms * (TIMER_HZ / 1_000)
    }
}

/// IO_VAL pin function (register 0x23 bits 3:0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Output {
    /// High while presence is detected (0xC).
    #[default]
    ActiveHigh,
    /// Low while presence is detected (0xD).
    ActiveLow,
}

impl Output {
    const fn nibble(self) -> u8 {
        match self {
            Self::ActiveHigh => 0x0C,
            Self::ActiveLow => 0x0D,
        }
    }
}

/// Detector configuration. `Default` = manual defaults with IO_VAL as an
/// active-high presence output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// TX power step 0–7 (0x03 bits 2:0, default 5).
    pub rf_power: u8,
    /// Sample-rate divider of the 32 kHz clock, ≥ 2 (0x10, default 32 =
    /// 1 kHz).
    pub sample_divider: u8,
    /// Detection (trigger) threshold (0x18/0x19, default 0x0100); higher =
    /// less sensitive.
    pub trigger_threshold: u16,
    /// Noise-update threshold (0x1A/0x1B, default 0x0800).
    pub noise_threshold: u16,
    /// Hold time after the last detection, ms (0x1D–0x1F, default 30 s).
    pub delay_ms: u32,
    /// Lock (blanking) time after the output drops, ms (0x20–0x22,
    /// default 2 s).
    pub lock_ms: u32,
    /// IO_VAL polarity.
    pub output: Output,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            rf_power: 5,
            sample_divider: 0x20,
            trigger_threshold: 0x0100,
            noise_threshold: 0x0800,
            delay_ms: 30_000,
            lock_ms: 2_000,
            output: Output::ActiveHigh,
        }
    }
}

impl Config {
    fn valid(&self) -> bool {
        self.rf_power <= 7 && self.sample_divider >= 2
    }

    fn thresholds(&self) -> [u8; 5] {
        let [t0, t1] = self.trigger_threshold.to_le_bytes();
        let [n0, n1] = self.noise_threshold.to_le_bytes();
        [REG_THRESHOLD, t0, t1, n0, n1]
    }

    fn timers(&self) -> [u8; 7] {
        let d = ms_to_ticks(self.delay_ms).to_le_bytes();
        let l = ms_to_ticks(self.lock_ms).to_le_bytes();
        [REG_DELAY_TIME, d[0], d[1], d[2], l[0], l[1], l[2]]
    }
}

/// Presence sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Presence detected (IO_VAL asserted).
    pub present: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Occupancy(self.present));
    }
}

/// Bus error: I2C or the IO_VAL pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusError<I, P> {
    /// I2C error.
    I2c(I),
    /// IO_VAL pin error.
    Pin(P),
}

macro_rules! common {
    ($($asyncness:ident)?; $($aw:tt)*) => {
        /// Create a driver; no bus traffic. `io_val` is the presence pin.
        pub const fn new(i2c: I, io_val: P) -> Self {
            Self {
                i2c,
                io_val,
                output: Output::ActiveHigh,
            }
        }

        /// Give the bus and pin back.
        pub fn release(self) -> (I, P) {
            (self.i2c, self.io_val)
        }

        /// Write registers starting at `buf[0]`.
        pub $($asyncness)? fn write(&mut self, buf: &[u8]) -> Result<(), Error<BusError<I::Error, P::Error>>> {
            self.i2c
                .write(ADDRESS, buf)
                $($aw)*
                .map_err(|e| Error::Bus(BusError::I2c(e)))
        }

        /// Read one register.
        pub $($asyncness)? fn read_register(&mut self, reg: u8) -> Result<u8, Error<BusError<I::Error, P::Error>>> {
            let mut b = [0u8];
            self.i2c
                .write_read(ADDRESS, &[reg], &mut b)
                $($aw)*
                .map_err(|e| Error::Bus(BusError::I2c(e)))?;
            Ok(b[0])
        }

        /// Write `config` (I2C_EN must be high). Returns `InvalidArgument` for
        /// an out-of-range field without touching the chip.
        pub $($asyncness)? fn configure(&mut self, config: &Config) -> Result<(), Error<BusError<I::Error, P::Error>>> {
            if !config.valid() {
                return Err(Error::InvalidArgument);
            }
            self.write(&[REG_DETECT_CTL, 0x00]) $($aw)* ?;
            let rf = self.read_register(REG_RF_POWER) $($aw)* ?;
            self.write(&[REG_RF_POWER, (rf & !0x07) | config.rf_power]) $($aw)* ?;
            self.write(&[REG_SAMPLE_DIV, config.sample_divider]) $($aw)* ?;
            self.write(&config.thresholds()) $($aw)* ?;
            self.write(&config.timers()) $($aw)* ?;
            let pin = self.read_register(REG_PIN_CTL) $($aw)* ?;
            self.write(&[REG_PIN_CTL, (pin & 0xF0) | config.output.nibble()]) $($aw)* ?;
            self.write(&[REG_DETECT_CTL, 0x01]) $($aw)* ?;
            self.output = config.output;
            Ok(())
        }
    };
}

fn present<P: InputPin>(pin: &mut P, output: Output) -> Result<bool, P::Error> {
    let high = pin.is_high()?;
    Ok(high == (output == Output::ActiveHigh))
}

/// Blocking driver.
pub struct Xbr818<I, P> {
    i2c: I,
    io_val: P,
    output: Output,
}

impl<I: embedded_hal::i2c::I2c, P: InputPin> Xbr818<I, P> {
    common!(;);
}

impl<I: embedded_hal::i2c::I2c, P: InputPin> zigbee_sensor::blocking::Sensor for Xbr818<I, P> {
    type BusError = BusError<I::Error, P::Error>;
    type Measurement = Measurement;

    /// The detector runs autonomously: returns `0`.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<Self::BusError>> {
        Ok(0)
    }

    /// Sample the IO_VAL pin.
    fn read_measurement(&mut self) -> Result<Measurement, Error<Self::BusError>> {
        let present =
            present(&mut self.io_val, self.output).map_err(|e| Error::Bus(BusError::Pin(e)))?;
        Ok(Measurement { present })
    }
}

/// Async driver (`embedded-hal-async` I2C; `Wait` pin for edge waits).
pub mod asynch {
    use super::{
        ADDRESS, BusError, Config, Measurement, Output, REG_DETECT_CTL, REG_PIN_CTL, REG_RF_POWER,
        REG_SAMPLE_DIV, present,
    };
    use embedded_hal::digital::InputPin;
    use embedded_hal_async::digital::Wait;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Xbr818`].
    pub struct Xbr818<I, P> {
        i2c: I,
        io_val: P,
        output: Output,
    }

    impl<I: embedded_hal_async::i2c::I2c, P: InputPin> Xbr818<I, P> {
        common!(async; .await);
    }

    impl<I: embedded_hal_async::i2c::I2c, P: InputPin + Wait> Xbr818<I, P> {
        /// Await the next IO_VAL edge, then return the new state.
        pub async fn wait_for_change(
            &mut self,
        ) -> Result<Measurement, Error<BusError<I::Error, P::Error>>> {
            self.io_val
                .wait_for_any_edge()
                .await
                .map_err(|e| Error::Bus(BusError::Pin(e)))?;
            let present =
                present(&mut self.io_val, self.output).map_err(|e| Error::Bus(BusError::Pin(e)))?;
            Ok(Measurement { present })
        }
    }

    impl<I: embedded_hal_async::i2c::I2c, P: InputPin> zigbee_sensor::asynch::Sensor for Xbr818<I, P> {
        type BusError = BusError<I::Error, P::Error>;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<Self::BusError>> {
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<Self::BusError>> {
            let present =
                present(&mut self.io_val, self.output).map_err(|e| Error::Bus(BusError::Pin(e)))?;
            Ok(Measurement { present })
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::digital::{Mock as PinMock, State, Transaction as PinTx};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::Sensor;

    fn configure_transactions(
        c: &Config,
        rf_old: u8,
        pin_old: u8,
        pin_new: u8,
    ) -> vec::Vec<Transaction> {
        let mut timers = vec![0x1D];
        timers.extend_from_slice(&ms_to_ticks(c.delay_ms).to_le_bytes()[..3]);
        timers.extend_from_slice(&ms_to_ticks(c.lock_ms).to_le_bytes()[..3]);
        vec![
            Transaction::write(ADDRESS, vec![0x13, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x03], vec![rf_old]),
            Transaction::write(ADDRESS, vec![0x03, (rf_old & 0xF8) | c.rf_power]),
            Transaction::write(ADDRESS, vec![0x10, c.sample_divider]),
            Transaction::write(ADDRESS, c.thresholds().to_vec()),
            Transaction::write(ADDRESS, timers),
            Transaction::write_read(ADDRESS, vec![0x23], vec![pin_old]),
            Transaction::write(ADDRESS, vec![0x23, pin_new]),
            Transaction::write(ADDRESS, vec![0x13, 0x01]),
        ]
    }

    #[test]
    fn timer_ticks_match_manual_defaults() {
        // Rd-04 manual §6: delay default 0x0EA600 = 30 s, lock 0x00FA00 = 2 s.
        assert_eq!(ms_to_ticks(30_000), 0x0E_A600);
        assert_eq!(ms_to_ticks(2_000), 0x00_FA00);
        assert_eq!(ms_to_ticks(u32::MAX), 0xFF_FFFF);
        let c = Config::default();
        assert_eq!(c.thresholds(), [0x18, 0x00, 0x01, 0x00, 0x08]);
        assert_eq!(c.timers(), [0x1D, 0x00, 0xA6, 0x0E, 0x00, 0xFA, 0x00]);
    }

    #[test]
    fn configure_sequence_and_active_low_output() {
        let c = Config {
            rf_power: 2,
            output: Output::ActiveLow,
            ..Config::default()
        };
        let mut i2c = Mock::new(&configure_transactions(&c, 0x45, 0x30, 0x3D));
        let mut pin = PinMock::new(&[PinTx::get(State::Low), PinTx::get(State::High)]);
        let mut s = Xbr818::new(i2c.clone(), pin.clone());
        s.configure(&c).unwrap();
        assert_eq!(s.start_measurement(), Ok(0));
        assert_eq!(s.read_measurement(), Ok(Measurement { present: true }));
        assert_eq!(s.read_measurement(), Ok(Measurement { present: false }));
        i2c.done();
        pin.done();
    }

    #[test]
    fn rejects_invalid_config() {
        let mut i2c = Mock::new(&[]);
        let mut pin = PinMock::new(&[]);
        let mut s = Xbr818::new(i2c.clone(), pin.clone());
        let bad = Config {
            rf_power: 8,
            ..Config::default()
        };
        assert_eq!(s.configure(&bad), Err(Error::InvalidArgument));
        i2c.done();
        pin.done();
    }

    #[test]
    fn async_configure_and_wait() {
        use zigbee_sensor::asynch::Sensor as _;
        let c = Config::default();
        let mut i2c = Mock::new(&configure_transactions(&c, 0x45, 0x00, 0x0C));
        let mut pin = PinMock::new(&[
            PinTx::wait_for_edge(embedded_hal_mock::eh1::digital::Edge::Any),
            PinTx::get(State::High),
            PinTx::get(State::Low),
        ]);
        let mut s = asynch::Xbr818::new(i2c.clone(), pin.clone());
        block_on(s.configure(&c)).unwrap();
        assert!(block_on(s.wait_for_change()).unwrap().present);
        assert!(!block_on(s.read_measurement()).unwrap().present);
        i2c.done();
        pin.done();
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
