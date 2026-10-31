//! HM3301 / HM3302 (HM-3300 series) laser particulate matter sensors over
//! I2C.
//!
//! I2C address 0x40 (datasheet: 0x80 write / 0x81 read), 100–400 kHz. The
//! module starts in UART active-upload mode; the select command `0x88` must
//! be written once after power-on to switch to I2C, otherwise "the data is
//! wrong". A read returns 29 bytes: 2 reserved, sensor number, PM1.0 /
//! PM2.5 / PM10 for CF=1 and for atmospheric environment (µg/m³), six
//! particle counts per litre (> 0.3, 0.5, 1.0, 2.5, 5.0, 10 µm), and a
//! checksum (low byte of the sum of bytes 0..=27). All words big-endian.
//!
//! The sensor refreshes once per second on its own:
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! sends the select command the first time and returns 0. Sleep is only
//! available through the SET pin (low = sleep, < 150 µA); allow ≥ 30 s after
//! waking for the fan to stabilise.
//!
//! Reference: "HM-3300/3600 Dust Sensor Data Sheet" V2.1 (2018-07-20),
//! Appendix 2.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// 7-bit I2C address.
pub const ADDRESS: u8 = 0x40;
/// Command that switches the module from UART to I2C.
pub const SELECT_I2C: u8 = 0x88;
/// Length of a data read.
pub const FRAME_LEN: usize = 29;
/// Data refresh interval.
pub const REFRESH_US: WaitUs = 1_000_000;
/// Stabilisation time after power-on or SET-pin wake-up.
pub const STABILIZE_US: WaitUs = 30_000_000;

/// Mass concentrations in µg/m³.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Pm {
    /// PM1.0.
    pub pm1_0: u16,
    /// PM2.5.
    pub pm2_5: u16,
    /// PM10.
    pub pm10: u16,
}

/// Lower size bounds in nm of [`Measurement::counts_per_litre`].
pub const COUNT_SIZES_NM: [u16; 6] = [300, 500, 1000, 2500, 5000, 10000];

/// One decoded frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Sensor number.
    pub sensor_number: u16,
    /// CF=1, standard particulate.
    pub standard: Pm,
    /// Atmospheric environment (reported as [`Quantity`]).
    pub atmospheric: Pm,
    /// Particle counts per litre with diameter ≥ 0.3, 0.5, 1.0, 2.5, 5.0
    /// and 10 µm.
    pub counts_per_litre: [u16; 6],
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        let a = self.atmospheric;
        f(Quantity::Pm1(u32::from(a.pm1_0) * 10));
        f(Quantity::Pm25(u32::from(a.pm2_5) * 10));
        f(Quantity::Pm10(u32::from(a.pm10) * 10));
        // Per litre → per 0.1 L (= 0.01 #/cm³), truncated.
        for (size_nm, count) in COUNT_SIZES_NM.into_iter().zip(self.counts_per_litre) {
            f(Quantity::ParticlesAbove {
                size_nm,
                count: u32::from(count / 10),
            });
        }
    }
}

/// Checksum: low byte of the sum of bytes 0..=27.
pub fn checksum(frame: &[u8; FRAME_LEN]) -> u8 {
    frame[..28].iter().fold(0u8, |acc, b| acc.wrapping_add(*b))
}

/// Verify and decode a 29-byte frame.
pub fn decode_frame<E>(frame: &[u8; FRAME_LEN]) -> Result<Measurement, Error<E>> {
    if checksum(frame) != frame[28] {
        return Err(Error::Crc);
    }
    let w = |i: usize| u16::from_be_bytes([frame[2 * i], frame[2 * i + 1]]);
    let mut counts_per_litre = [0u16; 6];
    for (k, c) in counts_per_litre.iter_mut().enumerate() {
        *c = w(8 + k);
    }
    Ok(Measurement {
        sensor_number: w(1),
        standard: Pm {
            pm1_0: w(2),
            pm2_5: w(3),
            pm10: w(4),
        },
        atmospheric: Pm {
            pm1_0: w(5),
            pm2_5: w(6),
            pm10: w(7),
        },
        counts_per_litre,
    })
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                selected: false,
            }
        }

        /// Forget that I2C mode was selected (call after the module was
        /// power-cycled) so the next measurement re-sends it.
        pub fn reset_selection(&mut self) {
            self.selected = false;
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct Hm330x<I2C> {
    i2c: I2C,
    selected: bool,
}

impl<I2C> Hm330x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Hm330x<I2C> {
    /// Send the I2C select command (`0x88`).
    pub fn select_i2c(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(ADDRESS, &[SELECT_I2C]).map_err(Error::bus)?;
        self.selected = true;
        Ok(())
    }

    /// Read the raw 29-byte frame.
    pub fn read_frame(&mut self) -> Result<[u8; FRAME_LEN], Error<I2C::Error>> {
        let mut f = [0u8; FRAME_LEN];
        self.i2c.read(ADDRESS, &mut f).map_err(Error::bus)?;
        Ok(f)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Hm330x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        if !self.selected {
            self.select_i2c()?;
        }
        Ok(0)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        decode_frame(&self.read_frame()?)
    }
}

/// Async driver.
pub mod asynch {
    use super::{ADDRESS, FRAME_LEN, Measurement, SELECT_I2C, decode_frame};
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Hm330x`].
    pub struct Hm330x<I2C> {
        i2c: I2C,
        selected: bool,
    }

    impl<I2C> Hm330x<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Hm330x<I2C> {
        /// See [`super::Hm330x::select_i2c`].
        pub async fn select_i2c(&mut self) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(ADDRESS, &[SELECT_I2C])
                .await
                .map_err(Error::bus)?;
            self.selected = true;
            Ok(())
        }

        /// See [`super::Hm330x::read_frame`].
        pub async fn read_frame(&mut self) -> Result<[u8; FRAME_LEN], Error<I2C::Error>> {
            let mut f = [0u8; FRAME_LEN];
            self.i2c.read(ADDRESS, &mut f).await.map_err(Error::bus)?;
            Ok(f)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Hm330x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if !self.selected {
                self.select_i2c().await?;
            }
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            decode_frame(&self.read_frame().await?)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    fn frame(words: [u16; 14]) -> Vec<u8> {
        let mut f = [0u8; FRAME_LEN];
        for (i, w) in words.iter().enumerate() {
            f[2 * i..2 * i + 2].copy_from_slice(&w.to_be_bytes());
        }
        f[28] = checksum(&f);
        f.to_vec()
    }

    fn sample(pm25: u16) -> Vec<u8> {
        frame([0, 2, 5, pm25 + 1, 14, 4, pm25, 13, 3000, 900, 200, 30, 6, 1])
    }

    #[test]
    fn decode_layout_and_checksum() {
        let f: [u8; FRAME_LEN] = sample(11).try_into().unwrap();
        let m = decode_frame::<()>(&f).unwrap();
        assert_eq!(m.sensor_number, 2);
        assert_eq!(
            m.standard,
            Pm {
                pm1_0: 5,
                pm2_5: 12,
                pm10: 14
            }
        );
        assert_eq!(m.atmospheric.pm2_5, 11);
        assert_eq!(m.counts_per_litre, [3000, 900, 200, 30, 6, 1]);
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(
            seen[..3],
            [Quantity::Pm1(40), Quantity::Pm25(110), Quantity::Pm10(130)]
        );
        assert_eq!(
            seen[3..],
            [
                Quantity::ParticlesAbove {
                    size_nm: 300,
                    count: 300
                },
                Quantity::ParticlesAbove {
                    size_nm: 500,
                    count: 90
                },
                Quantity::ParticlesAbove {
                    size_nm: 1000,
                    count: 20
                },
                Quantity::ParticlesAbove {
                    size_nm: 2500,
                    count: 3
                },
                Quantity::ParticlesAbove {
                    size_nm: 5000,
                    count: 0
                },
                Quantity::ParticlesAbove {
                    size_nm: 10000,
                    count: 0
                },
            ]
        );
        let mut bad = f;
        bad[12] ^= 1;
        assert_eq!(decode_frame::<()>(&bad), Err(Error::Crc));
    }

    #[test]
    fn selects_i2c_once_then_reads() {
        let mut bad = sample(1);
        bad[28] ^= 0xFF;
        let expectations = [
            Transaction::write(ADDRESS, vec![SELECT_I2C]),
            Transaction::read(ADDRESS, sample(8)),
            Transaction::read(ADDRESS, bad),
            Transaction::write(ADDRESS, vec![SELECT_I2C]),
            Transaction::read(ADDRESS, sample(9)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hm = Hm330x::new(&mut i2c);
        assert_eq!(hm.start_measurement().unwrap(), 0);
        assert_eq!(hm.read_measurement().unwrap().atmospheric.pm2_5, 8);
        hm.start_measurement().unwrap();
        assert_eq!(hm.read_measurement(), Err(Error::Crc));
        hm.reset_selection();
        hm.start_measurement().unwrap();
        assert_eq!(hm.read_measurement().unwrap().atmospheric.pm2_5, 9);
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
            Transaction::write(ADDRESS, vec![SELECT_I2C]),
            Transaction::read(ADDRESS, sample(42)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut hm = asynch::Hm330x::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        let m = block_on(hm.measure(&mut delay)).unwrap();
        assert_eq!(m.atmospheric.pm2_5, 42);
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
