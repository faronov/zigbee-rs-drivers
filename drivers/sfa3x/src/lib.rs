//! Sensirion SFA30 formaldehyde sensor module (I2C, 0x5D): HCHO in ppb plus
//! on-board SHT relative humidity and temperature. Start / stop continuous
//! measurement, read measured values, device marking (`probe`) and reset.
//!
//! # Measurement through the [`Sensor`](zigbee_sensor::blocking::Sensor) trait
//!
//! The first `start_measurement` starts continuous measurement and returns
//! 500 ms (the first 0.5 s measurement); the next read is
//! [`Error::NotReady`]. Later starts send `Read Measured Values` and return
//! 5 ms; the read then returns the average of all 0.5 s measurements since
//! the previous read (exponential smoothing over ~60 s for slower reads).
//! Read every 0.5–60 s; 1 s is recommended.
//!
//! The formaldehyde output is 0 for the first 10 s after power-up; RH/T
//! are valid immediately.
//!
//! The SFA30 has no sleep mode (idle 0.5–1.0 mA), so there is no
//! `LowPower` implementation.
//!
//! References: Sensirion "Description of the SFA30 I2C Interface", V1.0
//! (July 2022, D1) and "Data Sheet SFA30", V1.3 (January 2025, D1). The
//! I2C note gives only the 5 ms read wait; the start (1 ms), stop (50 ms)
//! and reset (1 s) times follow Sensirion's `embedded-i2c-sfa3x` driver.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address.
pub const ADDRESS: u8 = 0x5D;

mod cmd {
    pub const START_MEASUREMENT: u16 = 0x0006;
    pub const STOP_MEASUREMENT: u16 = 0x0104;
    pub const READ_MEASURED_VALUES: u16 = 0x0327;
    pub const DEVICE_MARKING: u16 = 0xD060;
    pub const RESET: u16 = 0xD304;
}

/// Start continuous measurement execution time.
pub const START_WAIT_US: WaitUs = 1_000;
/// Stop measurement execution time.
pub const STOP_WAIT_US: WaitUs = 50_000;
/// Wait between a read command and reading its response.
pub const READ_WAIT_US: WaitUs = 5_000;
/// Soft reset time.
pub const RESET_WAIT_US: WaitUs = 1_000_000;
/// Duration of one measurement; no data before it completes.
pub const FIRST_SAMPLE_US: WaitUs = 500_000;

/// One `Read Measured Values` result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Formaldehyde, ppb (rounded from 0.2 ppb steps; negative clamped
    /// to 0).
    pub formaldehyde: u32,
    /// Relative humidity, 0.01 %RH.
    pub humidity: u16,
    /// Temperature, 0.01 °C.
    pub temperature: i32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Formaldehyde(self.formaldehyde));
        f(Quantity::Humidity(self.humidity));
        f(Quantity::Temperature(self.temperature));
    }
}

fn convert(w: &[u16; 3]) -> Measurement {
    let hcho = i32::from(w[0] as i16);
    Measurement {
        formaldehyde: ((hcho + 2) / 5).max(0) as u32,
        humidity: (w[1] as i16).clamp(0, 10_000) as u16,
        temperature: i32::from(w[2] as i16) / 2,
    }
}

/// Non-empty printable ASCII up to the first NUL.
fn valid_marking(m: &[u8; 32]) -> bool {
    let len = m.iter().position(|&b| b == 0).unwrap_or(m.len());
    len > 0 && m[..len].iter().all(|b| b.is_ascii_graphic())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    Nothing,
    Read,
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                bus: Interface::new(i2c, ADDRESS),
                running: false,
                pending: Pending::Nothing,
            }
        }

        /// `true` while the driver has started a measurement.
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
        $($async)* fn send(&mut self, cmd: u16, wait: WaitUs) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(cmd, &[]) $($aw)* ?;
            Ok(wait)
        }

        /// Read the device marking and check that it is a printable ASCII
        /// string; otherwise [`Error::UnexpectedId`] with its first four
        /// bytes.
        pub $($async)* fn probe<D: $Delay>(&mut self, delay: &mut D) -> Result<[u8; 32], Error<I2C::Error>> {
            let marking = self.device_marking(delay) $($aw)* ?;
            if !valid_marking(&marking) {
                let [a, b, c, d, ..] = marking;
                return Err(Error::UnexpectedId(u32::from_be_bytes([a, b, c, d])));
            }
            Ok(marking)
        }

        /// Device marking (the 2D bar code text), NUL-terminated ASCII.
        pub $($async)* fn device_marking<D: $Delay>(&mut self, delay: &mut D) -> Result<[u8; 32], Error<I2C::Error>> {
            let mut w = [0u16; 16];
            self.bus.fetch(cmd::DEVICE_MARKING, &[], READ_WAIT_US, delay, &mut w) $($aw)* ?;
            let mut out = [0u8; 32];
            words_to_bytes(&w, &mut out);
            Ok(out)
        }

        /// Start continuous measurement (idle only).
        pub $($async)* fn start_continuous_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::START_MEASUREMENT, START_WAIT_US) $($aw)* ?;
            self.running = true;
            self.pending = Pending::Nothing;
            Ok(START_WAIT_US)
        }

        /// Return to idle.
        pub $($async)* fn stop_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::STOP_MEASUREMENT, STOP_WAIT_US) $($aw)* ?;
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(STOP_WAIT_US)
        }

        /// Request the averaged values; read them after 5 ms with
        /// [`read_measured_values`](Self::read_measured_values).
        pub $($async)* fn request_measured_values(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::READ_MEASURED_VALUES, READ_WAIT_US) $($aw)* ?;
            self.pending = Pending::Read;
            Ok(READ_WAIT_US)
        }

        /// Values requested by `request_measured_values`; [`Error::NotReady`]
        /// if nothing was requested or the sensor NACKs (busy, or no
        /// measurement yet).
        pub $($async)* fn read_measured_values(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if self.pending != Pending::Read {
                return Err(Error::NotReady);
            }
            let mut w = [0u16; 3];
            let result = self.bus.read(&mut w) $($aw)*;
            self.pending = Pending::Nothing;
            result.map_err(nack_as_not_ready)?;
            Ok(convert(&w))
        }

        /// Soft reset, same state as after power-up.
        pub $($async)* fn reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::RESET, RESET_WAIT_US) $($aw)* ?;
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(RESET_WAIT_US)
        }

        $($async)* fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.running {
                return self.request_measured_values() $($aw)*;
            }
            self.start_continuous_measurement() $($aw)* ?;
            Ok(FIRST_SAMPLE_US)
        }
    };
}

macro_rules! trait_impls {
    ($Sensor:path, $I2c:path, [$($async:tt)*] [$($aw:tt)*]) => {
        impl<I2C: $I2c> $Sensor for Sfa3x<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Starts continuous measurement on the first call (returns
            /// 500 ms), then requests the averaged values (5 ms).
            $($async)* fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.start() $($aw)*
            }

            /// [`Error::NotReady`] right after starting.
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
pub struct Sfa3x<I2C> {
    bus: Interface<I2C>,
    running: bool,
    pending: Pending,
}

impl<I2C> Sfa3x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Sfa3x<I2C> {
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
        ADDRESS, FIRST_SAMPLE_US, Measurement, Pending, READ_WAIT_US, RESET_WAIT_US, START_WAIT_US,
        STOP_WAIT_US, cmd, convert, valid_marking,
    };
    use zigbee_sensirion_common::asynch::Interface;
    use zigbee_sensirion_common::{nack_as_not_ready, words_to_bytes};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Sfa3x`].
    #[derive(Debug)]
    pub struct Sfa3x<I2C> {
        bus: Interface<I2C>,
        running: bool,
        pending: Pending,
    }

    impl<I2C> Sfa3x<I2C> {
        constructors!();
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Sfa3x<I2C> {
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

    // Datasheet example (section 4.3.3): 19.8 ppb, 42.50 %RH, 22.71 °C.
    const SAMPLE: [u16; 3] = [0x0063, 0x109A, 0x11BE];

    fn words(w: &[u16]) -> Vec<u8> {
        w.iter().flat_map(|&x| encode_word(x)).collect()
    }

    fn marking(s: &[u8]) -> Vec<u8> {
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
    fn probe_and_measurement_cycle() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0xD0, 0x60]),
            Transaction::read(ADDRESS, marking(b"20365C629F073BE1")),
            Transaction::write(ADDRESS, vec![0x00, 0x06]),
            Transaction::write(ADDRESS, vec![0x03, 0x27]),
            nack(Transaction::read(ADDRESS, vec![0; 9])),
            Transaction::write(ADDRESS, vec![0x03, 0x27]),
            Transaction::read(ADDRESS, words(&SAMPLE)),
            Transaction::write(ADDRESS, vec![0x01, 0x04]),
            Transaction::write(ADDRESS, vec![0xD3, 0x04]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::blocking_delay_us(5_000)]);
        let mut sfa = Sfa3x::new(&mut i2c);
        let m = sfa.probe(&mut delay).unwrap();
        assert_eq!(&m[..17], b"20365C629F073BE1\0");
        assert_eq!(sfa.start_measurement().unwrap(), 500_000);
        assert_eq!(sfa.read_measurement(), Err(Error::NotReady));
        assert_eq!(sfa.start_measurement().unwrap(), 5_000);
        assert_eq!(sfa.read_measurement(), Err(Error::NotReady));
        sfa.start_measurement().unwrap();
        let m = sfa.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                formaldehyde: 20,
                humidity: 4250,
                temperature: 2271,
            }
        );
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(
            seen,
            [
                Quantity::Formaldehyde(20),
                Quantity::Humidity(4250),
                Quantity::Temperature(2271),
            ]
        );
        assert_eq!(sfa.stop_measurement().unwrap(), 50_000);
        assert!(!sfa.is_running());
        assert_eq!(sfa.reset().unwrap(), 1_000_000);
        delay.done();
        i2c.done();
    }

    #[test]
    fn conversions_and_bad_marking() {
        // Negative HCHO clamps to 0; -10.5 °C; RH above 100 % clamps.
        let m = convert(&[(-3i16) as u16, 10_050, (-2100i16) as u16]);
        assert_eq!(
            (m.formaldehyde, m.humidity, m.temperature),
            (0, 10_000, -1050)
        );
        // 0.4 ppb rounds down, 0.6 ppb up.
        assert_eq!(convert(&[2, 0, 0]).formaldehyde, 0);
        assert_eq!(convert(&[3, 0, 0]).formaldehyde, 1);

        let expectations = [
            Transaction::write(ADDRESS, vec![0xD0, 0x60]),
            Transaction::read(ADDRESS, marking(b"")),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::blocking_delay_us(5_000)]);
        let mut sfa = Sfa3x::new(&mut i2c);
        assert_eq!(sfa.probe(&mut delay), Err(Error::UnexpectedId(0)));
        delay.done();
        i2c.done();
    }

    #[test]
    fn async_measure_and_crc() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut bad = words(&SAMPLE);
        bad[2] ^= 1;
        let expectations = [
            Transaction::write(ADDRESS, vec![0x00, 0x06]),
            Transaction::write(ADDRESS, vec![0x03, 0x27]),
            Transaction::read(ADDRESS, words(&SAMPLE)),
            Transaction::write(ADDRESS, vec![0x03, 0x27]),
            Transaction::read(ADDRESS, bad),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::async_delay_us(500_000), D::async_delay_us(5_000)]);
        let mut sfa = asynch::Sfa3x::new(&mut i2c);
        block_on(async {
            assert_eq!(sfa.measure(&mut delay).await, Err(Error::NotReady));
            let m = sfa.measure(&mut delay).await.unwrap();
            assert_eq!(m.temperature, 2271);
            sfa.start_measurement().await.unwrap();
            assert_eq!(sfa.read_measurement().await, Err(Error::Crc));
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
