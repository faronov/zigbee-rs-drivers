//! Allegro ALS31300 3D linear hall-effect sensor (±500/±1000/±2000 G and
//! joystick variants) with I2C.
//!
//! Reference: Allegro MicroSystems "ALS31300 3D Linear Hall-Effect Sensor
//! with I2C Output and Advanced Low Power Management", Rev. 14.
//!
//! * 32-bit registers, MSB first. Field and temperature come from a single
//!   8-byte read of 0x28/0x29 (the device auto-increments while the master
//!   keeps acknowledging), so all values belong to one sample.
//! * The device converts continuously in active mode, so
//!   `start_measurement` returns 0. `read_measurement` returns `NotReady`
//!   when the New Data flag is clear, i.e. no update happened since the last
//!   read.
//! * Writing anything except the sleep bits needs the customer access code
//!   (sent automatically before the first write; call [`Als31300::unlock`]
//!   again after a power cycle).
//! * EEPROM (0x02, 0x03, 0x0D–0x0F) has 1000 write cycles and a 50 ms
//!   write time: the update helpers read first, skip identical values and
//!   otherwise return the 50 ms the caller must wait.
//! * CRC (0x02 bit 18) is not supported; leave it disabled.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

const REG_EEPROM_02: u8 = 0x02;
const REG_EEPROM_03: u8 = 0x03;
const REG_ACCESS: u8 = 0x35;
const REG_POWER: u8 = 0x27;
const REG_DATA: u8 = 0x28;
/// Customer access code for register 0x35.
pub const ACCESS_CODE: u32 = 0x2C41_3534;
/// EEPROM write time, µs.
pub const EEPROM_WRITE_US: WaitUs = 50_000;
/// Wake from sleep: power-on delay (600 µs at BW 0) plus one 3-channel
/// update (495 µs), µs.
pub const WAKE_US: WaitUs = 1_100;

const NEW_DATA: u8 = 0x80;
const INTERRUPT: u8 = 0x40;
const MASK_MEASUREMENT: u32 = 0x00E0_01E0;

/// Factory sensitivity option (part number suffix).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// ALS31300EEJASR-500: 4 LSB/G.
    #[default]
    Gauss500,
    /// ALS31300EEJASR-1000: 2 LSB/G.
    Gauss1000,
    /// ALS31300EEJASR-2000: 1 LSB/G.
    Gauss2000,
    /// ALS31300EEJASR-JOY: X/Y 1 LSB/G, Z 0.25 LSB/G.
    Joystick,
}

impl Variant {
    /// nT per LSB for (X/Y, Z); 1 G = 100 µT.
    pub const fn nt_per_lsb(self) -> (i32, i32) {
        match self {
            Self::Gauss500 => (25_000, 25_000),
            Self::Gauss1000 => (50_000, 50_000),
            Self::Gauss2000 => (100_000, 100_000),
            Self::Joystick => (100_000, 400_000),
        }
    }
}

/// Filter / update-rate setting (BW Select). Typical update times are for
/// 1/2/3 channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Bandwidth {
    /// FIR, 3.5 kHz: 160/330/495 µs (lowest noise, default).
    #[default]
    Fir3k5 = 0,
    /// FIR, 7 kHz: 80/170/255 µs.
    Fir7k = 1,
    /// FIR, 14 kHz: 40/90/135 µs.
    Fir14k = 2,
    /// No FIR, 10 kHz: 64/138/207 µs.
    Raw10k = 4,
    /// No FIR, 20 kHz: 32/74/111 µs.
    Raw20k = 5,
    /// No FIR, 40 kHz: 16/42/63 µs.
    Raw40k = 6,
}

/// Inactive time in low-power duty-cycle mode (Low-Power Mode Count Max).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum InactiveTime {
    /// 0.5 ms.
    #[default]
    Us500 = 0,
    /// 1 ms.
    Ms1,
    /// 5 ms.
    Ms5,
    /// 10 ms.
    Ms10,
    /// 50 ms.
    Ms50,
    /// 100 ms.
    Ms100,
    /// 500 ms.
    Ms500,
    /// 1 s.
    Ms1000,
}

/// Operating mode (0x27 Sleep).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PowerMode {
    /// Continuous conversion, 3.4 mA.
    #[default]
    Active,
    /// Ultra-low-power sleep, 14 nA; I2C still answers, data frozen.
    Sleep,
    /// Low-power duty-cycle mode (12 µA–2 mA average); temperature updates
    /// every 10 cycles. Pair with [`Interrupt`] to wake the host.
    DutyCycle(InactiveTime),
}

/// Interrupt configuration (EEPROM 0x03). Threshold codes are 6-bit: in
/// absolute mode the limit is `(code + 1) × 32 − 1` LSB, in signed mode
/// the code is two's complement and the limit is `(code + 1) × 64 − 1`
/// (positive) or `(code + 1) × 64` (negative) LSB.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Interrupt {
    /// X, Y, Z threshold codes; `None` disables the axis.
    pub thresholds: [Option<u8>; 3],
    /// Signed thresholds (trigger in one direction only).
    pub signed: bool,
    /// Delta mode: compare against the sample taken on entering LPDCM.
    pub delta: bool,
    /// Record events in the INT EEPROM Status bit (costs an EEPROM write).
    pub eeprom_flag: bool,
}

impl Interrupt {
    /// EEPROM 0x03 value.
    pub const fn register(&self) -> u32 {
        let mut v = ((self.signed as u32) << 24)
            | ((self.delta as u32) << 23)
            | ((self.eeprom_flag as u32) << 21);
        let mut axis = 0;
        while axis < 3 {
            if let Some(code) = self.thresholds[axis] {
                v |= 1 << (18 + axis);
                v |= ((code & 0x3F) as u32) << (6 * axis);
            }
            axis += 1;
        }
        v
    }
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Flux density, nT (disabled channels read as their last value).
    pub field: Vector3,
    /// Junction temperature, 0.01 °C.
    pub temperature: i32,
    /// Interrupt status bit.
    pub interrupt: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::MagneticField(self.field));
        f(Quantity::Temperature(self.temperature));
    }
}

/// Temperature in 0.01 °C from the 12-bit code: 302 × (T − 1708) / 4096.
pub const fn temperature_centi(raw: u16) -> i32 {
    let num = 30_200 * (raw as i32 - 1_708);
    let half = if num < 0 { -2_048 } else { 2_048 };
    (num + half) / 4_096
}

/// Decode the 8-byte read of 0x28/0x29. `None` if New Data is clear.
pub fn decode(buf: &[u8; 8], variant: Variant) -> Option<Measurement> {
    if buf[3] & NEW_DATA == 0 {
        return None;
    }
    let axis = |msb: u8, lsb: u8| (((msb as i8 as i16) << 4) | (lsb & 0x0F) as i16) as i32;
    let (xy, z) = variant.nt_per_lsb();
    let raw_t = (((buf[3] & 0x3F) as u16) << 6) | (buf[7] & 0x3F) as u16;
    Some(Measurement {
        field: Vector3 {
            x: axis(buf[0], buf[5]) * xy,
            y: axis(buf[1], buf[6] >> 4) * xy,
            z: axis(buf[2], buf[6]) * z,
        },
        temperature: temperature_centi(raw_t),
        interrupt: buf[3] & INTERRUPT != 0,
    })
}

macro_rules! api {
    ($($asyncness:ident)?; $($aw:tt)*) => {
        /// Create a driver for the part at `address` (96–110 via ADR0/ADR1);
        /// no bus traffic. The part needs 600 µs after power-up.
        pub const fn new(i2c: I2C, address: u8, variant: Variant) -> Self {
            Self { i2c, address, variant, unlocked: false, inactive: InactiveTime::Us500 }
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        /// Read a 32-bit register.
        pub $($asyncness)? fn read_register(&mut self, reg: u8) -> Result<u32, Error<I2C::Error>> {
            let mut b = [0u8; 4];
            self.i2c.write_read(self.address, &[reg], &mut b) $($aw)* .map_err(Error::Bus)?;
            Ok(u32::from_be_bytes(b))
        }

        /// Write a 32-bit register (enters customer access first if needed).
        pub $($asyncness)? fn write_register(&mut self, reg: u8, value: u32) -> Result<(), Error<I2C::Error>> {
            if !self.unlocked {
                self.unlock() $($aw)* ?;
            }
            self.write_raw(reg, value) $($aw)*
        }

        $($asyncness)? fn write_raw(&mut self, reg: u8, value: u32) -> Result<(), Error<I2C::Error>> {
            let v = value.to_be_bytes();
            self.i2c.write(self.address, &[reg, v[0], v[1], v[2], v[3]]) $($aw)* .map_err(Error::Bus)
        }

        /// Send the customer access code (valid until the next power cycle).
        pub $($asyncness)? fn unlock(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_raw(REG_ACCESS, ACCESS_CODE) $($aw)* ?;
            self.unlocked = true;
            Ok(())
        }

        /// Read-modify-write an EEPROM register (0x02, 0x03, 0x0D–0x0F).
        /// Returns 0 if nothing changed, otherwise the 50 ms write time to
        /// wait before the next access.
        pub $($asyncness)? fn update_eeprom(&mut self, reg: u8, mask: u32, bits: u32) -> Result<WaitUs, Error<I2C::Error>> {
            let old = self.read_register(reg) $($aw)* ?;
            let new = (old & !mask) | (bits & mask);
            if new == old {
                return Ok(0);
            }
            self.write_register(reg, new) $($aw)* ?;
            Ok(EEPROM_WRITE_US)
        }

        /// Bandwidth, enabled channels (x, y, z) and INT latching (EEPROM
        /// 0x02; other fields are preserved).
        pub $($asyncness)? fn set_measurement(&mut self, bandwidth: Bandwidth, channels: [bool; 3], int_latch: bool) -> Result<WaitUs, Error<I2C::Error>> {
            let [x, y, z] = channels;
            let bits = ((bandwidth as u32) << 21)
                | ((z as u32) << 8)
                | ((y as u32) << 7)
                | ((x as u32) << 6)
                | ((int_latch as u32) << 5);
            self.update_eeprom(REG_EEPROM_02, MASK_MEASUREMENT, bits) $($aw)*
        }

        /// Interrupt configuration (EEPROM 0x03).
        pub $($asyncness)? fn set_interrupt(&mut self, int: &Interrupt) -> Result<WaitUs, Error<I2C::Error>> {
            self.update_eeprom(REG_EEPROM_03, 0x01FF_FFFF, int.register()) $($aw)*
        }

        /// Select the operating mode (volatile 0x27; single read mode).
        /// Leaving [`PowerMode::Sleep`] takes [`WAKE_US`].
        pub $($asyncness)? fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), Error<I2C::Error>> {
            let sleep = match mode {
                PowerMode::Active => 0,
                PowerMode::Sleep => 1,
                PowerMode::DutyCycle(t) => {
                    self.inactive = t;
                    2
                }
            };
            let value = ((self.inactive as u32) << 4) | sleep;
            if sleep == 1 {
                // The sleep field is writable without customer access.
                self.write_raw(REG_POWER, value) $($aw)*
            } else {
                self.write_register(REG_POWER, value) $($aw)*
            }
        }

        $($asyncness)? fn read_data(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut b = [0u8; 8];
            self.i2c.write_read(self.address, &[REG_DATA], &mut b) $($aw)* .map_err(Error::Bus)?;
            decode(&b, self.variant).ok_or(Error::NotReady)
        }
    };
}

/// Blocking driver.
pub struct Als31300<I2C> {
    i2c: I2C,
    address: u8,
    variant: Variant,
    unlocked: bool,
    inactive: InactiveTime,
}

impl<I2C: embedded_hal::i2c::I2c> Als31300<I2C> {
    api!(;);
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Als31300<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Free-running: nothing to start.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        Ok(0)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        self.read_data()
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Als31300<I2C> {
    type BusError = I2C::Error;

    /// Sleep mode (14 nA); needs no access code.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_power_mode(PowerMode::Sleep)
    }

    /// Back to active mode.
    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.set_power_mode(PowerMode::Active)?;
        Ok(WAKE_US)
    }
}

/// Async driver (`embedded-hal-async` I2C).
pub mod asynch {
    use super::*;

    /// Async twin of [`super::Als31300`].
    pub struct Als31300<I2C> {
        i2c: I2C,
        address: u8,
        variant: Variant,
        unlocked: bool,
        inactive: InactiveTime,
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Als31300<I2C> {
        api!(async; .await);
    }

    impl<I2C: embedded_hal_async::i2c::I2c> zigbee_sensor::asynch::Sensor for Als31300<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            self.read_data().await
        }
    }

    impl<I2C: embedded_hal_async::i2c::I2c> zigbee_sensor::asynch::LowPower for Als31300<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.set_power_mode(PowerMode::Sleep).await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.set_power_mode(PowerMode::Active).await?;
            Ok(WAKE_US)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const ADDR: u8 = 96;
    const UNLOCK: [u8; 5] = [0x35, 0x2C, 0x41, 0x35, 0x34];

    #[test]
    fn decodes_datasheet_example() {
        // X MSB 0xC0 + LSB 0x6 = −1018 LSB = −254.5 G on the 500 G part.
        let buf = [0xC0, 0x00, 0x7F, 0x80 | 0x40 | 0x1F, 0x00, 0x06, 0x0F, 0x3F];
        let m = decode(&buf, Variant::Gauss500).unwrap();
        assert_eq!(
            m.field,
            Vector3 {
                x: -25_450_000,
                y: 0,
                z: 2_047 * 25_000
            }
        );
        // Temperature code 0x7FF = 2047 → 302 × 339 / 4096 = 24.99 °C.
        assert_eq!(m.temperature, 2_499);
        assert!(m.interrupt);
        let j = decode(&[0, 0, 0xFF, 0x80, 0, 0, 0x0F, 0], Variant::Joystick).unwrap();
        assert_eq!(j.field.z, -400_000);
        assert_eq!(temperature_centi(1_708), 0);
        assert_eq!(decode(&[0; 8], Variant::Gauss500), None);
    }

    #[test]
    fn interrupt_register_layout() {
        let int = Interrupt {
            thresholds: [Some(0x3F), None, Some(0x01)],
            signed: true,
            delta: false,
            eeprom_flag: true,
        };
        assert_eq!(
            int.register(),
            (1 << 24) | (1 << 21) | (1 << 20) | (1 << 18) | (0x01 << 12) | 0x3F
        );
    }

    #[test]
    fn eeprom_update_unlocks_once_and_skips_unchanged() {
        let default: u32 = 0x0000_03C0; // 1.8 V I2C, X/Y/Z enabled.
        let wanted: u32 = (6 << 21) | 0x0300; // BW 6, Z only, threshold bit kept.
        let mut i2c = Mock::new(&[
            T::write_read(ADDR, vec![0x02], default.to_be_bytes().to_vec()),
            T::write(ADDR, UNLOCK.to_vec()),
            T::write(ADDR, vec![0x02, 0x00, 0xC0, 0x03, 0x00]),
            T::write_read(ADDR, vec![0x02], wanted.to_be_bytes().to_vec()),
            T::write(ADDR, vec![0x27, 0, 0, 0, 0x72]),
        ]);
        let mut s = Als31300::new(i2c.clone(), ADDR, Variant::Gauss500);
        let ch = [false, false, true];
        assert_eq!(s.set_measurement(Bandwidth::Raw40k, ch, false), Ok(50_000));
        assert_eq!(s.set_measurement(Bandwidth::Raw40k, ch, false), Ok(0));
        s.set_power_mode(PowerMode::DutyCycle(InactiveTime::Ms1000))
            .unwrap();
        i2c.done();
    }

    #[test]
    fn read_sleep_wake() {
        let mut i2c = Mock::new(&[
            T::write_read(ADDR, vec![0x28], vec![0; 8]),
            T::write(ADDR, vec![0x27, 0, 0, 0, 0x01]),
            T::write(ADDR, UNLOCK.to_vec()),
            T::write(ADDR, vec![0x27, 0, 0, 0, 0x00]),
        ]);
        let mut s = Als31300::new(i2c.clone(), ADDR, Variant::Gauss2000);
        assert_eq!(s.start_measurement(), Ok(0));
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        s.sleep().unwrap();
        assert_eq!(s.wake(), Ok(WAKE_US));
        i2c.done();
    }

    #[test]
    fn async_read() {
        use zigbee_sensor::asynch::Sensor as _;
        let mut i2c = Mock::new(&[T::write_read(
            ADDR,
            vec![0x28],
            vec![0x01, 0xFF, 0x00, 0x80, 0, 0x02, 0xF1, 0],
        )]);
        let mut s = asynch::Als31300::new(i2c.clone(), ADDR, Variant::Gauss1000);
        let m = block_on(s.read_measurement()).unwrap();
        // X = 0x012 = 18, Y = 0xFFF = −1, Z = 0x001 = 1 (×50 µT).
        assert_eq!(
            m.field,
            Vector3 {
                x: 900_000,
                y: -50_000,
                z: 50_000
            }
        );
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
