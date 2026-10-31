//! TDK InvenSense ICP-10100, ICP-10101, ICP-10110 and ICP-10111 barometric
//! pressure and temperature sensors (I2C, address 0x63).
//!
//! All four parts share the die and command set; they differ only in
//! package and lid (ICP-10100/10110: 3-hole, IPx8 water resistant;
//! ICP-10101/10111: 1-hole).
//!
//! The sensor is command based. Each measurement command starts one
//! temperature + pressure conversion; the part idles (1 µA) afterwards, so
//! there is no sleep command. [`start_measurement`] sends the command for
//! the configured [`Mode`] and returns its maximum conversion time
//! (datasheet table 2). The sensor NACKs all I2C traffic while converting,
//! so reading before the wait has elapsed fails with a bus error.
//!
//! # Conversion
//!
//! `T = -45 °C + 175 °C · t_dout / 2^16`. Pressure uses the datasheet
//! formula `P = A + B / (C + p_dout)` where `A`, `B`, `C` follow from three
//! temperature-dependent look-up points `s1`, `s2`, `s3` (built from the
//! OTP constants) mapped to 45 000, 80 000 and 105 000 Pa (datasheet
//! sections 5.9 and 5.11). The datasheet evaluates it in `float`; this
//! driver uses an equivalent closed form in integers. With `a = s2 - s1`,
//! `b = s3 - s1` and `y = p_dout - s1`, the three-point rational fit
//! reduces to
//!
//! ```text
//! P = 45000 + 420000 · (b - a) · y / (5·a·b + (7·b - 12·a) · y)
//! ```
//!
//! evaluated in `i64` with one rounding division (no float, no `i128`).
//! The host tests compare it with the datasheet `float` algorithm (in
//! `f64`) over random OTP sets; the difference stays below 0.52 Pa,
//! i.e. the integer rounding.
//!
//! Reference: TDK InvenSense "ICP-101xx" datasheet DS-000186 revision 1.2.
//! The formula is re-derived from the published equations; no vendor code
//! is used.
//!
//! [`start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x63;

/// Product code in bits 5:0 of the ID register.
pub const PRODUCT_ID: u16 = 0x08;

/// Soft-reset time (`tSR`, 170 µs typ.; rounded up).
pub const RESET_WAIT_US: WaitUs = 200;

const CMD_SOFT_RESET: [u8; 2] = [0x80, 0x5D];
const CMD_READ_ID: [u8; 2] = [0xEF, 0xC8];
const CMD_OTP_SETUP: [u8; 5] = [0xC5, 0x95, 0x00, 0x66, 0x9C];
const CMD_OTP_READ: [u8; 2] = [0xC7, 0xF7];

/// Measurement mode (datasheet table 2 / 13).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// 1.8 ms max, 1.3 µA at 1 Hz, 3.2 Pa RMS.
    LowPower,
    /// 6.3 ms max, 2.6 µA at 1 Hz, 1.6 Pa RMS (default).
    #[default]
    Normal,
    /// 23.8 ms max, 5.2 µA at 1 Hz, 0.8 Pa RMS.
    LowNoise,
    /// 94.5 ms max, 10.4 µA at 1 Hz, 0.4 Pa RMS.
    UltraLowNoise,
}

impl Mode {
    /// "Transmit T first" command.
    const fn command(self) -> [u8; 2] {
        match self {
            Self::LowPower => [0x60, 0x9C],
            Self::Normal => [0x68, 0x25],
            Self::LowNoise => [0x70, 0xDF],
            Self::UltraLowNoise => [0x78, 0x66],
        }
    }

    /// Maximum conversion time.
    pub const fn conversion_us(self) -> WaitUs {
        match self {
            Self::LowPower => 1_800,
            Self::Normal => 6_300,
            Self::LowNoise => 23_800,
            Self::UltraLowNoise => 94_500,
        }
    }
}

/// One sample.
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

/// CRC-8, polynomial 0x31, init 0xFF (datasheet table 17).
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0xFFu8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x31
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Check the CRC of a `[msb, lsb, crc]` word and return it.
fn word<E>(chunk: &[u8]) -> Result<u16, Error<E>> {
    if crc8(&chunk[..2]) != chunk[2] {
        return Err(Error::Crc);
    }
    Ok(u16::from_be_bytes([chunk[0], chunk[1]]))
}

/// The four OTP calibration constants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration(pub [i16; 4]);

impl Calibration {
    /// Convert raw outputs (`t_dout`, 24-bit `p_dout`). Returns
    /// [`Error::InvalidData`] if the OTP constants are so far out of range
    /// that the fit degenerates or overflows.
    pub fn compensate<E>(&self, t_dout: u16, p_dout: u32) -> Result<Measurement, Error<E>> {
        let temperature = ((17_500 * i32::from(t_dout) + 32_768) >> 16) - 4_500;

        let c = self.0.map(i64::from);
        let t = i64::from(t_dout) - 32_768;
        let t2 = t * t;
        // c·t² / 2^24, rounded; |c·t²| < 2^45.
        let quad = |k: i64| (k * t2 + (1 << 23)) >> 24;
        let s1 = 3_670_016 + quad(c[0]); // 3.5 · 2^20
        let s2 = 2_048 * c[3] + quad(c[1]);
        let s3 = 12_058_624 + quad(c[2]); // 11.5 · 2^20
        let a = s2 - s1;
        let b = s3 - s1;
        let y = i64::from(p_dout) - s1;

        let n = (b - a).checked_mul(y);
        let d = (5 * a)
            .checked_mul(b)
            .zip((7 * b - 12 * a).checked_mul(y))
            .and_then(|(x, z)| x.checked_add(z));
        let (Some(n), Some(d)) = (n, d) else {
            return Err(Error::InvalidData);
        };
        if d == 0 {
            return Err(Error::InvalidData);
        }
        // 420000 · n / d, rounded, with 420000 = 13125 · 2^5 to stay in u64.
        let num = n
            .unsigned_abs()
            .checked_mul(13_125)
            .ok_or(Error::InvalidData)?;
        let den = d.unsigned_abs();
        let (q, r) = (num / den, num % den);
        let v = (q << 5) + ((r << 5) + den / 2) / den;
        let v = i64::try_from(v).map_err(|_| Error::InvalidData)?;
        let p = if (n < 0) != (d < 0) {
            45_000 - v
        } else {
            45_000 + v
        };
        Ok(Measurement {
            temperature,
            pressure: u32::try_from(p.max(0)).map_err(|_| Error::InvalidData)?,
        })
    }
}

/// Decode a 9-byte "T first" result: `T, crc, P_MMSB, P_MLSB, crc,
/// P_LMSB, P_LLSB, crc`.
fn decode<E>(raw: &[u8; 9]) -> Result<(u16, u32), Error<E>> {
    let t = word(&raw[0..3])?;
    let p_hi = word(&raw[3..6])?;
    let p_lo = word(&raw[6..9])?;
    Ok((t, u32::from(p_hi) << 8 | u32::from(p_lo >> 8)))
}

macro_rules! common {
    ($name:ident) => {
        impl<I2C> $name<I2C> {
            /// Create a driver; no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    mode: Mode::Normal,
                    calibration: None,
                    pending: false,
                }
            }

            /// Select the measurement mode.
            pub const fn with_mode(mut self, mode: Mode) -> Self {
                self.mode = mode;
                self
            }

            /// Change the measurement mode (applies to the next start).
            pub fn set_mode(&mut self, mode: Mode) {
                self.mode = mode;
            }

            /// Current measurement mode.
            pub const fn mode(&self) -> Mode {
                self.mode
            }

            /// OTP constants read by `init` (`None` before).
            pub const fn calibration(&self) -> Option<&Calibration> {
                self.calibration.as_ref()
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
pub struct Icp101xx<I2C> {
    i2c: I2C,
    mode: Mode,
    calibration: Option<Calibration>,
    pending: bool,
}

common!(Icp101xx);

impl<I2C: embedded_hal::i2c::I2c> Icp101xx<I2C> {
    /// Soft reset. Wait [`RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.pending = false;
        self.i2c
            .write(ADDRESS, &CMD_SOFT_RESET)
            .map_err(Error::bus)?;
        Ok(RESET_WAIT_US)
    }

    /// Check the product code and read the OTP constants. Called by
    /// `start_measurement` if needed.
    pub fn init(&mut self) -> Result<Calibration, Error<I2C::Error>> {
        let mut buf = [0; 3];
        self.i2c
            .write_read(ADDRESS, &CMD_READ_ID, &mut buf)
            .map_err(Error::bus)?;
        let id = word(&buf)?;
        if id & 0x3F != PRODUCT_ID {
            return Err(Error::UnexpectedId(id.into()));
        }
        self.i2c
            .write(ADDRESS, &CMD_OTP_SETUP)
            .map_err(Error::bus)?;
        let mut otp = [0i16; 4];
        for c in &mut otp {
            self.i2c.write(ADDRESS, &CMD_OTP_READ).map_err(Error::bus)?;
            self.i2c.read(ADDRESS, &mut buf).map_err(Error::bus)?;
            *c = word(&buf)? as i16;
        }
        let cal = Calibration(otp);
        self.calibration = Some(cal);
        Ok(cal)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Icp101xx<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        if self.calibration.is_none() {
            self.init()?;
        }
        self.i2c
            .write(ADDRESS, &self.mode.command())
            .map_err(Error::bus)?;
        self.pending = true;
        Ok(self.mode.conversion_us())
    }

    /// [`Error::NotReady`] if no conversion was started; a bus error
    /// (NACK) if the conversion is still running.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let Some(cal) = self.calibration.filter(|_| self.pending) else {
            return Err(Error::NotReady);
        };
        let mut raw = [0; 9];
        self.i2c.read(ADDRESS, &mut raw).map_err(Error::bus)?;
        self.pending = false;
        let (t, p) = decode(&raw)?;
        cal.compensate(t, p)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Icp101xx<I2C> {
    type BusError = I2C::Error;

    /// The part idles by itself after each conversion; nothing to send.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        Ok(())
    }

    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, CMD_OTP_READ, CMD_OTP_SETUP, CMD_READ_ID, CMD_SOFT_RESET, Calibration,
        Measurement, Mode, PRODUCT_ID, decode, word,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Icp101xx`].
    #[derive(Debug)]
    pub struct Icp101xx<I2C> {
        i2c: I2C,
        mode: Mode,
        calibration: Option<Calibration>,
        pending: bool,
    }

    common!(Icp101xx);

    impl<I2C: I2c> Icp101xx<I2C> {
        /// See [`super::Icp101xx::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.pending = false;
            self.i2c
                .write(ADDRESS, &CMD_SOFT_RESET)
                .await
                .map_err(Error::bus)?;
            Ok(super::RESET_WAIT_US)
        }

        /// See [`super::Icp101xx::init`].
        pub async fn init(&mut self) -> Result<Calibration, Error<I2C::Error>> {
            let mut buf = [0; 3];
            self.i2c
                .write_read(ADDRESS, &CMD_READ_ID, &mut buf)
                .await
                .map_err(Error::bus)?;
            let id = word(&buf)?;
            if id & 0x3F != PRODUCT_ID {
                return Err(Error::UnexpectedId(id.into()));
            }
            self.i2c
                .write(ADDRESS, &CMD_OTP_SETUP)
                .await
                .map_err(Error::bus)?;
            let mut otp = [0i16; 4];
            for c in &mut otp {
                self.i2c
                    .write(ADDRESS, &CMD_OTP_READ)
                    .await
                    .map_err(Error::bus)?;
                self.i2c.read(ADDRESS, &mut buf).await.map_err(Error::bus)?;
                *c = word(&buf)? as i16;
            }
            let cal = Calibration(otp);
            self.calibration = Some(cal);
            Ok(cal)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Icp101xx<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.calibration.is_none() {
                self.init().await?;
            }
            self.i2c
                .write(ADDRESS, &self.mode.command())
                .await
                .map_err(Error::bus)?;
            self.pending = true;
            Ok(self.mode.conversion_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let Some(cal) = self.calibration.filter(|_| self.pending) else {
                return Err(Error::NotReady);
            };
            let mut raw = [0; 9];
            self.i2c.read(ADDRESS, &mut raw).await.map_err(Error::bus)?;
            self.pending = false;
            let (t, p) = decode(&raw)?;
            cal.compensate(t, p)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Icp101xx<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            Ok(())
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            Ok(0)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    type E = core::convert::Infallible;

    fn w(v: u16) -> [u8; 3] {
        let b = v.to_be_bytes();
        [b[0], b[1], crc8(&b)]
    }

    fn result(t: u16, p: u32) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend(w(t));
        v.extend(w((p >> 8) as u16));
        v.extend(w(((p & 0xFF) << 8) as u16 | 0x5A));
        v
    }

    /// Datasheet float algorithm (section 5.11), evaluated in f64.
    #[allow(clippy::float_arithmetic)]
    fn reference(c: [i16; 4], p: u32, t: u16) -> f64 {
        let c = c.map(f64::from);
        let t = f64::from(t) - 32_768.0;
        let s = [
            3.5 * 1_048_576.0 + c[0] * t * t / 16_777_216.0,
            2_048.0 * c[3] + c[1] * t * t / 16_777_216.0,
            11.5 * 1_048_576.0 + c[2] * t * t / 16_777_216.0,
        ];
        let pa = [45_000.0, 80_000.0, 105_000.0];
        let cc = (s[0] * s[1] * (pa[0] - pa[1])
            + s[1] * s[2] * (pa[1] - pa[2])
            + s[2] * s[0] * (pa[2] - pa[0]))
            / (s[2] * (pa[0] - pa[1]) + s[0] * (pa[1] - pa[2]) + s[1] * (pa[2] - pa[0]));
        let a = (pa[0] * s[0] - pa[1] * s[1] - (pa[1] - pa[0]) * cc) / (s[0] - s[1]);
        let b = (pa[0] - a) * (s[0] + cc);
        a + b / (cc + f64::from(p))
    }

    #[test]
    fn crc_examples() {
        assert_eq!(crc8(&[0x00]), 0xAC);
        assert_eq!(crc8(&[0xBE, 0xEF]), 0x92);
        assert_eq!(crc8(&[0x00, 0x66]), 0x9C);
    }

    #[test]
    #[allow(clippy::float_arithmetic)]
    fn compensation_matches_float_reference() {
        // Datasheet sample values (section 5.13).
        let cal = Calibration([1_000, 2_000, 3_000, 4_000]);
        let m = cal.compensate::<E>(32_000, 8_000_000).unwrap();
        assert_eq!(m.pressure, 78_651); // float: 78 650.67
        assert_eq!(m.temperature, ((17_500 * 32_000 + 32_768) >> 16) - 4_500);

        // Pseudo-random OTP sets, temperatures and pressures (xorshift).
        let mut x = 0x2545_F491_u32;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x
        };
        let mut worst = 0.0f64;
        let mut checked = 0;
        while checked < 20_000 {
            let c = [
                (next() % 6_001) as i16 - 3_000,
                (next() % 6_001) as i16 - 3_000,
                (next() % 6_001) as i16 - 3_000,
                (next() % 2_001) as i16 + 3_000,
            ];
            let t = next() as u16;
            let p = next() & 0xFF_FFFF;
            let r = reference(c, p, t);
            if !(30_000.0..=120_000.0).contains(&r) {
                continue;
            }
            let got = Calibration(c).compensate::<E>(t, p).unwrap().pressure;
            worst = worst.max((f64::from(got) - r).abs());
            checked += 1;
        }
        assert!(worst < 0.52, "worst {worst}");
    }

    #[test]
    fn temperature_range() {
        let cal = Calibration([0, 0, 0, 4_000]);
        assert_eq!(
            cal.compensate::<E>(0, 8_000_000).unwrap().temperature,
            -4_500
        );
        assert_eq!(
            cal.compensate::<E>(u16::MAX, 8_000_000)
                .unwrap()
                .temperature,
            13_000
        );
        // 25 °C: t_dout = (25 + 45) / 175 · 65536 = 26214.4.
        assert_eq!(
            cal.compensate::<E>(26_214, 8_000_000).unwrap().temperature,
            2_500
        );
    }

    #[test]
    fn degenerate_otp_is_invalid() {
        // s2 == s1 at t = 0 → a = 0, and y = 0 makes the denominator 0.
        let cal = Calibration([0, 0, 0, 1_792]);
        assert_eq!(
            cal.compensate::<E>(32_768, 3_670_016),
            Err(Error::InvalidData)
        );
    }

    fn init_ex(otp: [i16; 4]) -> Vec<I2cTx> {
        let mut ex = vec![
            I2cTx::write_read(ADDRESS, vec![0xEF, 0xC8], w(0x5A48).to_vec()),
            I2cTx::write(ADDRESS, CMD_OTP_SETUP.to_vec()),
        ];
        for c in otp {
            ex.push(I2cTx::write(ADDRESS, vec![0xC7, 0xF7]));
            ex.push(I2cTx::read(ADDRESS, w(c as u16).to_vec()));
        }
        ex
    }

    #[test]
    fn i2c_cycle() {
        let otp = [-1_234, 2_000, 567, 4_100];
        let (t, p) = (26_214, 7_654_321);
        let mut ex = init_ex(otp);
        ex.extend([
            I2cTx::write(ADDRESS, vec![0x70, 0xDF]),
            I2cTx::read(ADDRESS, result(t, p)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut icp = Icp101xx::new(&mut i2c).with_mode(Mode::LowNoise);
        assert_eq!(icp.read_measurement(), Err(Error::NotReady));
        assert_eq!(icp.start_measurement().unwrap(), 23_800);
        assert_eq!(icp.calibration(), Some(&Calibration(otp)));
        let m = icp.read_measurement().unwrap();
        let expected = Calibration(otp).compensate::<E>(t, p).unwrap();
        assert_eq!(m, expected);
        #[allow(clippy::float_arithmetic)]
        let r = reference(otp, p, t);
        #[allow(clippy::float_arithmetic)]
        let diff = (f64::from(m.pressure) - r).abs();
        assert!(diff < 0.52);
        assert_eq!(icp.read_measurement(), Err(Error::NotReady));
        icp.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn wrong_id_bad_crc_and_reset() {
        let ex = [
            I2cTx::write_read(ADDRESS, vec![0xEF, 0xC8], w(0x0047).to_vec()),
            I2cTx::write_read(ADDRESS, vec![0xEF, 0xC8], vec![0x00, 0x08, 0x00]),
            I2cTx::write(ADDRESS, vec![0x80, 0x5D]),
        ];
        let mut i2c = I2cMock::new(&ex);
        let mut icp = Icp101xx::new(&mut i2c);
        assert_eq!(icp.start_measurement(), Err(Error::UnexpectedId(0x47)));
        assert_eq!(icp.start_measurement(), Err(Error::Crc));
        assert_eq!(icp.soft_reset().unwrap(), RESET_WAIT_US);
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

        let otp = [1_000, 2_000, 3_000, 4_000];
        let mut ex = init_ex(otp);
        ex.extend([
            I2cTx::write(ADDRESS, vec![0x68, 0x25]),
            I2cTx::read(ADDRESS, result(32_000, 8_000_000)),
        ]);
        let mut i2c = I2cMock::new(&ex);
        let mut icp = asynch::Icp101xx::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        let m = block_on(icp.measure(&mut delay)).unwrap();
        assert_eq!(m.pressure, 78_651);
        assert_eq!(delay.0, 6_300);
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
