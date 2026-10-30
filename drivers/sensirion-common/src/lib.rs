//! Sensirion I2C framing shared by the zigbee-rs Sensirion drivers.
//!
//! Sensirion's I2C parts (SHT4x, SCD4x, SGP4x, SPS30, SEN5x, SEN6x, ...)
//! exchange 16-bit big-endian words, each followed by a CRC-8 (polynomial
//! 0x31, init 0xFF). Commands are one 16-bit word (one byte on SHT4x/STS4x),
//! optionally followed by argument words with their CRC.
//!
//! A command that returns data has an execution time: the sensor NACKs its
//! read header until the command has finished (Sensirion datasheets,
//! "send command and fetch result" sequence). [`blocking::Interface`] and
//! [`asynch::Interface`] wrap the bus with these primitives; the drivers
//! build on them.
//!
//! Reference: CRC and word format as specified in, e.g., Sensirion
//! "Datasheet SCD4x" v1.7 section 3.12 and "Datasheet SHT4x" v7.3 section 4.4.

#![no_std]

use embedded_hal::i2c::ErrorKind;
use zigbee_sensor::{Error, crc::crc8_sensirion};

pub use zigbee_sensor::crc::crc8_sensirion as crc8;

/// Largest response handled by [`blocking::Interface::read`] in words.
pub const MAX_WORDS: usize = 16;
/// Largest number of argument words of a write command.
pub const MAX_ARGS: usize = 6;

const READ_BUF: usize = 3 * MAX_WORDS;
/// Size of the buffer taken by [`encode`].
pub const WRITE_BUF: usize = 2 + 3 * MAX_ARGS;

/// One word with its CRC, as sent on the bus.
pub fn encode_word(word: u16) -> [u8; 3] {
    let [msb, lsb] = word.to_be_bytes();
    [msb, lsb, crc8_sensirion(&[msb, lsb])]
}

/// Check the CRC of one 3-byte chunk and return its word.
pub fn decode_word<E>(chunk: &[u8]) -> Result<u16, Error<E>> {
    if crc8_sensirion(&chunk[..2]) != chunk[2] {
        return Err(Error::Crc);
    }
    Ok(u16::from_be_bytes([chunk[0], chunk[1]]))
}

/// Decode `out.len()` words from `bytes` (3 bytes per word).
pub fn decode<E>(bytes: &[u8], out: &mut [u16]) -> Result<(), Error<E>> {
    for (chunk, word) in bytes.chunks_exact(3).zip(out.iter_mut()) {
        *word = decode_word(chunk)?;
    }
    Ok(())
}

/// Serialise a 16-bit command plus argument words into `buf`; returns the
/// number of bytes used.
///
/// # Panics
/// If `args` has more than [`MAX_ARGS`] words.
pub fn encode(cmd: u16, args: &[u16], buf: &mut [u8; WRITE_BUF]) -> usize {
    assert!(args.len() <= MAX_ARGS);
    buf[..2].copy_from_slice(&cmd.to_be_bytes());
    for (chunk, &arg) in buf[2..].chunks_exact_mut(3).zip(args) {
        chunk.copy_from_slice(&encode_word(arg));
    }
    2 + 3 * args.len()
}

/// Unpack words into big-endian bytes (ASCII product names, serials).
pub fn words_to_bytes(words: &[u16], out: &mut [u8]) {
    for (pair, word) in out.chunks_mut(2).zip(words) {
        let bytes = word.to_be_bytes();
        pair.copy_from_slice(&bytes[..pair.len()]);
    }
}

/// `true` if the transport error is a NACK (Sensirion parts NACK their read
/// header while busy or when no data is available).
pub fn is_nack<E: embedded_hal::i2c::Error>(error: &E) -> bool {
    matches!(error.kind(), ErrorKind::NoAcknowledge(_))
}

/// Map a NACK on the read header to [`Error::NotReady`], keep any other
/// error. Use only where the datasheet defines a NACK as "busy" or "no new
/// data".
pub fn nack_as_not_ready<E: embedded_hal::i2c::Error>(error: Error<E>) -> Error<E> {
    match error {
        Error::Bus(e) if is_nack(&e) => Error::NotReady,
        other => other,
    }
}

/// Treat a NACK as success, for commands the sensor does not acknowledge
/// (wake-up, general-call reset).
pub fn ignore_nack<E: embedded_hal::i2c::Error>(
    result: Result<(), Error<E>>,
) -> Result<(), Error<E>> {
    match result {
        Err(Error::Bus(e)) if is_nack(&e) => Ok(()),
        other => other,
    }
}

/// Sensirion's common temperature transfer function `-45 + 175 × S / 65535`
/// in 0.01 °C, rounded (SHT4x, STS4x, SCD4x, SGP4x compensation, ...).
pub fn temperature_from_ticks(ticks: u16) -> i32 {
    ((17_500 * u32::from(ticks) + 32_767) / 65_535) as i32 - 4_500
}

/// Sensirion's common humidity transfer function `100 × S / 65535` in
/// 0.01 %RH, rounded.
pub fn humidity_from_ticks(ticks: u16) -> u16 {
    ((10_000 * u32::from(ticks) + 32_767) / 65_535) as u16
}

/// Inverse of [`temperature_from_ticks`], clamped to -45..=130 °C.
pub fn ticks_from_temperature(centi_celsius: i32) -> u16 {
    let t = (centi_celsius.clamp(-4_500, 13_000) + 4_500) as u32;
    ((t * 65_535 + 8_750) / 17_500) as u16
}

/// Inverse of [`humidity_from_ticks`], clamped to 0..=100 %RH.
pub fn ticks_from_humidity(centi_percent: u16) -> u16 {
    let rh = u32::from(centi_percent.min(10_000));
    ((rh * 65_535 + 5_000) / 10_000) as u16
}

/// Blocking bus primitives.
pub mod blocking {
    use super::{READ_BUF, WRITE_BUF, decode, encode, is_nack};
    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::I2c;
    use zigbee_sensor::Error;

    /// An I2C bus plus the device address.
    #[derive(Debug)]
    pub struct Interface<I2C> {
        i2c: I2C,
        address: u8,
    }

    impl<I2C> Interface<I2C> {
        /// Wrap the bus; no traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self { i2c, address }
        }

        /// Device address.
        pub const fn address(&self) -> u8 {
            self.address
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    }

    impl<I2C: I2c> Interface<I2C> {
        /// Send a 16-bit command followed by argument words (with CRC).
        pub fn write(&mut self, cmd: u16, args: &[u16]) -> Result<(), Error<I2C::Error>> {
            let mut buf = [0u8; WRITE_BUF];
            let len = encode(cmd, args, &mut buf);
            self.write_bytes(&buf[..len])
        }

        /// Send raw bytes (8-bit commands, general call).
        pub fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c.write(self.address, bytes).map_err(Error::bus)
        }

        /// I2C general-call reset (byte 0x06 to address 0x00). Resets every
        /// device on the bus that supports it; a NACK is not an error.
        pub fn general_call_reset(&mut self) -> Result<(), Error<I2C::Error>> {
            super::ignore_nack(self.i2c.write(0x00, &[0x06]).map_err(Error::bus))
        }

        /// Read `out.len()` words and check their CRCs.
        pub fn read(&mut self, out: &mut [u16]) -> Result<(), Error<I2C::Error>> {
            let mut buf = [0u8; READ_BUF];
            let buf = &mut buf[..3 * out.len()];
            self.i2c.read(self.address, buf).map_err(Error::bus)?;
            decode(buf, out)
        }

        /// Read words, retrying while the sensor NACKs its read header
        /// (I2C acknowledge polling) at most `attempts` times; then
        /// [`Error::Timeout`]. Used where a command's execution time is
        /// shorter than one bus retry loop and a delay is not allowed.
        pub fn read_polling(
            &mut self,
            out: &mut [u16],
            attempts: u16,
        ) -> Result<(), Error<I2C::Error>> {
            for _ in 0..attempts {
                match self.read(out) {
                    Err(Error::Bus(e)) if is_nack(&e) => {}
                    other => return other,
                }
            }
            Err(Error::Timeout)
        }

        /// "Send command and fetch result": write, wait the execution time
        /// on `delay`, read `out.len()` words.
        pub fn fetch<D: DelayNs>(
            &mut self,
            cmd: u16,
            args: &[u16],
            exec_us: u32,
            delay: &mut D,
            out: &mut [u16],
        ) -> Result<(), Error<I2C::Error>> {
            self.write(cmd, args)?;
            delay.delay_us(exec_us);
            self.read(out)
        }
    }
}

/// Async bus primitives; see [`blocking`].
pub mod asynch {
    use super::{READ_BUF, WRITE_BUF, decode, encode, is_nack};
    use embedded_hal_async::delay::DelayNs;
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::Error;

    /// Async twin of [`super::blocking::Interface`].
    #[derive(Debug)]
    pub struct Interface<I2C> {
        i2c: I2C,
        address: u8,
    }

    impl<I2C> Interface<I2C> {
        /// Wrap the bus; no traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self { i2c, address }
        }

        /// Device address.
        pub const fn address(&self) -> u8 {
            self.address
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    }

    impl<I2C: I2c> Interface<I2C> {
        /// See [`super::blocking::Interface::write`].
        pub async fn write(&mut self, cmd: u16, args: &[u16]) -> Result<(), Error<I2C::Error>> {
            let mut buf = [0u8; WRITE_BUF];
            let len = encode(cmd, args, &mut buf);
            self.write_bytes(&buf[..len]).await
        }

        /// See [`super::blocking::Interface::write_bytes`].
        pub async fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, bytes)
                .await
                .map_err(Error::bus)
        }

        /// See [`super::blocking::Interface::general_call_reset`].
        pub async fn general_call_reset(&mut self) -> Result<(), Error<I2C::Error>> {
            super::ignore_nack(self.i2c.write(0x00, &[0x06]).await.map_err(Error::bus))
        }

        /// See [`super::blocking::Interface::read`].
        pub async fn read(&mut self, out: &mut [u16]) -> Result<(), Error<I2C::Error>> {
            let mut buf = [0u8; READ_BUF];
            let buf = &mut buf[..3 * out.len()];
            self.i2c.read(self.address, buf).await.map_err(Error::bus)?;
            decode(buf, out)
        }

        /// See [`super::blocking::Interface::read_polling`].
        pub async fn read_polling(
            &mut self,
            out: &mut [u16],
            attempts: u16,
        ) -> Result<(), Error<I2C::Error>> {
            for _ in 0..attempts {
                match self.read(out).await {
                    Err(Error::Bus(e)) if is_nack(&e) => {}
                    other => return other,
                }
            }
            Err(Error::Timeout)
        }

        /// See [`super::blocking::Interface::fetch`].
        pub async fn fetch<D: DelayNs>(
            &mut self,
            cmd: u16,
            args: &[u16],
            exec_us: u32,
            delay: &mut D,
            out: &mut [u16],
        ) -> Result<(), Error<I2C::Error>> {
            self.write(cmd, args).await?;
            delay.delay_us(exec_us).await;
            self.read(out).await
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal::i2c::NoAcknowledgeSource;
    use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as Delay};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;

    #[test]
    fn word_codec_matches_datasheet_example() {
        // CRC(0xBEEF) = 0x92 (all Sensirion datasheets).
        assert_eq!(encode_word(0xBEEF), [0xBE, 0xEF, 0x92]);
        assert_eq!(decode_word::<()>(&[0xBE, 0xEF, 0x92]), Ok(0xBEEF));
        assert_eq!(decode_word::<()>(&[0xBE, 0xEF, 0x93]), Err(Error::Crc));
    }

    #[test]
    fn encode_command_with_arguments() {
        // SCD4x datasheet v1.7 table 13: set_temperature_offset 5.4 °C.
        let mut buf = [0u8; WRITE_BUF];
        let len = encode(0x241D, &[0x07E6], &mut buf);
        assert_eq!(&buf[..len], &[0x24, 0x1D, 0x07, 0xE6, 0x48]);
    }

    #[test]
    fn transfer_functions() {
        // SCD4x datasheet v1.7 table 11: 0x6667 = 25 °C, 0x5EB9 = 37 %RH.
        assert_eq!(temperature_from_ticks(0x6667), 2_500);
        assert_eq!(humidity_from_ticks(0x5EB9), 3_700);
        assert_eq!(temperature_from_ticks(0), -4_500);
        assert_eq!(temperature_from_ticks(u16::MAX), 13_000);
        // SGP4x datasheet: default compensation 25 °C = 0x6666, 50 %RH = 0x8000.
        assert_eq!(ticks_from_temperature(2_500), 0x6666);
        assert_eq!(ticks_from_humidity(5_000), 0x8000);
        assert_eq!(ticks_from_humidity(20_000), 0xFFFF);
    }

    #[test]
    fn strings_unpack() {
        let mut out = [0u8; 5];
        words_to_bytes(&[0x5345, 0x4E35, 0x3500], &mut out);
        assert_eq!(&out, b"SEN55");
    }

    #[test]
    fn fetch_writes_waits_reads() {
        let expectations = [
            Transaction::write(0x62, vec![0x23, 0x22]),
            Transaction::read(0x62, vec![0x04, 0x4C, 0x42]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[Delay::blocking_delay_us(1_000)]);
        let mut bus = blocking::Interface::new(&mut i2c, 0x62);
        let mut out = [0u16; 1];
        bus.fetch(0x2322, &[], 1_000, &mut delay, &mut out).unwrap();
        assert_eq!(out, [1_100]);
        i2c.done();
        delay.done();
    }

    #[test]
    fn polling_retries_on_nack_only() {
        let nack = ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address);
        let expectations = [
            Transaction::read(0x62, vec![0; 3]).with_error(nack),
            Transaction::read(0x62, vec![0xBE, 0xEF, 0x92]),
            Transaction::read(0x62, vec![0; 3]).with_error(nack),
            Transaction::read(0x62, vec![0; 3]).with_error(nack),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut bus = blocking::Interface::new(&mut i2c, 0x62);
        let mut out = [0u16; 1];
        bus.read_polling(&mut out, 5).unwrap();
        assert_eq!(out, [0xBEEF]);
        assert_eq!(bus.read_polling(&mut out, 2), Err(Error::Timeout));
        assert_eq!(nack_as_not_ready(Error::Bus(nack)), Error::NotReady);
        assert_eq!(
            nack_as_not_ready(Error::Bus(ErrorKind::Bus)),
            Error::Bus(ErrorKind::Bus)
        );
        i2c.done();
    }

    #[test]
    fn async_fetch() {
        let expectations = [
            Transaction::write(0x62, vec![0x24, 0x27, 0x07, 0x9E, 0x09]),
            Transaction::write(0x62, vec![0x23, 0x22]),
            Transaction::read(0x62, vec![0x04, 0x4C, 0x42]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[Delay::async_delay_us(1_000)]);
        let mut bus = asynch::Interface::new(&mut i2c, 0x62);
        let mut out = [0u16; 1];
        block_on(async {
            bus.write(0x2427, &[1_950]).await.unwrap();
            bus.fetch(0x2322, &[], 1_000, &mut delay, &mut out)
                .await
                .unwrap();
        });
        assert_eq!(out, [1_100]);
        i2c.done();
        delay.done();
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
