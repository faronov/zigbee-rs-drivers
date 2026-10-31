//! Texas Instruments TMAG5273 low-power linear 3D hall-effect sensor
//! (A1/B1/C1/D1 ±40/80 mT, A2/B2/C2/D2 ±133/266 mT).
//!
//! Reference: TI "TMAG5273 Low-Power Linear 3D Hall-Effect Sensor With I2C
//! Interface", SLYS045C (June 2021, revised April 2026).
//!
//! * Factory address by orderable letter ([`Address`]); changeable at run
//!   time ([`Tmag5273::set_address`], not retained over power cycles).
//! * Default operation is standby (trigger) mode: `start_measurement`
//!   sends the register pointer with the conversion-trigger bit (MSB of
//!   the register-address byte) and returns the conversion time; the
//!   result is read in one 12-byte burst (T, X, Y, Z, CONV_STATUS, angle,
//!   magnitude). `NotReady` until `RESULT_STATUS` is set.
//! * [`LowPower::sleep`](zigbee_sensor::blocking::LowPower::sleep) selects
//!   sleep mode (5 nA). In sleep the device does not ACK its address;
//!   [`wake`](zigbee_sensor::blocking::LowPower::wake) addresses it anyway
//!   (ignoring the NACK) and returns the 50 µs `tstart_sleep`.
//! * CRC and the 1-byte read modes are left disabled.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// Factory 7-bit address by orderable variant letter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Address {
    /// TMAG5273A1/A2.
    A = 0x35,
    /// TMAG5273B1/B2.
    B = 0x22,
    /// TMAG5273C1/C2.
    C = 0x78,
    /// TMAG5273D1/D2.
    D = 0x44,
}

const REG_DEVICE_CONFIG_1: u8 = 0x00;
const REG_DEVICE_CONFIG_2: u8 = 0x01;
const REG_X_THR_CONFIG: u8 = 0x04;
const REG_T_CONFIG: u8 = 0x07;
const REG_INT_CONFIG_1: u8 = 0x08;
const REG_MAG_GAIN_CONFIG: u8 = 0x09;
const REG_I2C_ADDRESS: u8 = 0x0C;
const REG_DEVICE_ID: u8 = 0x0D;
const REG_T_MSB_RESULT: u8 = 0x10;
const REG_CONV_STATUS: u8 = 0x18;
/// Conversion-trigger bit in the register-address byte.
const TRIGGER: u8 = 0x80;
const RESULT_STATUS: u8 = 0x01;
const POR: u8 = 0x10;

/// MANUFACTURER_ID (0x0F:0x0E) = "TI".
pub const MANUFACTURER_ID: u16 = 0x5449;
/// Power-up to standby, µs.
pub const POWER_UP_US: WaitUs = 270;
/// Sleep to standby, µs.
pub const WAKE_US: WaitUs = 50;
const START_MEASURE_US: WaitUs = 70;

/// Device range version (DEVICE_ID.VER).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Version {
    /// x1 parts: ±40 mT / ±80 mT.
    #[default]
    Mt40,
    /// x2 parts: ±133 mT / ±266 mT.
    Mt133,
}

impl Version {
    /// Full-scale B_R in mT for the normal or doubled range.
    pub const fn range_mt(self, wide: bool) -> i64 {
        let base = match self {
            Self::Mt40 => 40,
            Self::Mt133 => 133,
        };
        if wide { base * 2 } else { base }
    }
}

/// Enabled magnetic channels (MAG_CH_EN). The pseudo-simultaneous
/// sequences sample the first axis twice.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Channels {
    /// Magnetic channels off (temperature only).
    None = 0,
    /// X.
    X = 1,
    /// Y.
    Y = 2,
    /// X, Y.
    Xy = 3,
    /// Z.
    Z = 4,
    /// Z, X.
    Zx = 5,
    /// Y, Z.
    Yz = 6,
    /// X, Y, Z.
    #[default]
    Xyz = 7,
    /// X, Y, X.
    Xyx = 8,
    /// Y, X, Y.
    Yxy = 9,
    /// Y, Z, Y.
    Yzy = 10,
    /// X, Z, X.
    Xzx = 11,
}

impl Channels {
    /// (x, y, z) enabled.
    pub const fn axes(self) -> (bool, bool, bool) {
        match self {
            Self::None => (false, false, false),
            Self::X => (true, false, false),
            Self::Y => (false, true, false),
            Self::Xy | Self::Xyx | Self::Yxy => (true, true, false),
            Self::Z => (false, false, true),
            Self::Zx | Self::Xzx => (true, false, true),
            Self::Yz | Self::Yzy => (false, true, true),
            Self::Xyz => (true, true, true),
        }
    }

    /// Magnetic conversions per measurement.
    pub const fn conversions(self) -> u32 {
        match self {
            Self::None => 0,
            Self::X | Self::Y | Self::Z => 1,
            Self::Xy | Self::Zx | Self::Yz => 2,
            _ => 3,
        }
    }
}

/// Angle calculation axes (ANGLE_EN).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AngleAxes {
    /// No angle, gain or offset correction.
    #[default]
    Off,
    /// X first, Y second.
    Xy,
    /// Y first, Z second.
    Yz,
    /// X first, Z second.
    Xz,
}

/// Magnet temperature coefficient compensation (MAG_TEMPCO).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MagnetTempco {
    /// 0 %/°C.
    #[default]
    None,
    /// 0.12 %/°C (NdFeB).
    NdFeB,
    /// 0.2 %/°C (ceramic).
    Ceramic,
}

/// Duty-cycle interval in wake-up-and-sleep mode (SLEEPTIME).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SleepTime {
    /// 1 ms.
    #[default]
    Ms1 = 0,
    /// 5 ms.
    Ms5,
    /// 10 ms.
    Ms10,
    /// 15 ms.
    Ms15,
    /// 20 ms.
    Ms20,
    /// 30 ms.
    Ms30,
    /// 50 ms.
    Ms50,
    /// 100 ms.
    Ms100,
    /// 500 ms.
    Ms500,
    /// 1 s.
    Ms1000,
    /// 2 s.
    Ms2000,
    /// 5 s.
    Ms5000,
    /// 20 s.
    Ms20000,
}

/// Operating mode (OPERATING_MODE).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// Standby; one conversion per I2C trigger (default).
    #[default]
    Triggered,
    /// Free-running conversions; `start_measurement` returns 0.
    Continuous,
    /// Wake-up-and-sleep duty cycling at the configured [`SleepTime`]
    /// (use with the INT threshold interrupt; registers are not
    /// accessible until the device is woken).
    WakeUpAndSleep,
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Magnetic channels.
    pub channels: Channels,
    /// Also convert temperature.
    pub temperature: bool,
    /// log2 of the averaging count, 0–5 (1×…32×).
    pub averaging: u8,
    /// Use the doubled X/Y range.
    pub xy_wide: bool,
    /// Use the doubled Z range.
    pub z_wide: bool,
    /// Low-noise mode (3.0 mA instead of 2.3 mA while converting).
    pub low_noise: bool,
    /// Angle (CORDIC) axes.
    pub angle: AngleAxes,
    /// Apply MAG_GAIN_CONFIG to the second (instead of first) angle axis.
    pub gain_second_axis: bool,
    /// Magnet temperature compensation.
    pub tempco: MagnetTempco,
    /// Operating mode.
    pub mode: Mode,
    /// Interval for [`Mode::WakeUpAndSleep`].
    pub sleep_time: SleepTime,
}

impl Config {
    fn registers(&self) -> [u8; 5] {
        let tempco = match self.tempco {
            MagnetTempco::None => 0,
            MagnetTempco::NdFeB => 1,
            MagnetTempco::Ceramic => 3,
        };
        let angle = match self.angle {
            AngleAxes::Off => 0,
            AngleAxes::Xy => 1,
            AngleAxes::Yz => 2,
            AngleAxes::Xz => 3,
        };
        [
            REG_DEVICE_CONFIG_1,
            (tempco << 5) | (self.averaging << 2),
            ((self.low_noise as u8) << 4) | self.mode_bits(),
            ((self.channels as u8) << 4) | self.sleep_time as u8,
            ((self.gain_second_axis as u8) << 4)
                | (angle << 2)
                | ((self.xy_wide as u8) << 1)
                | self.z_wide as u8,
        ]
    }

    const fn mode_bits(&self) -> u8 {
        match self.mode {
            Mode::Triggered => 0,
            Mode::Continuous => 2,
            Mode::WakeUpAndSleep => 3,
        }
    }

    /// Conversion time after a trigger, µs (datasheet §5.11 plus the 70 µs
    /// start-up): 25 µs + 25 µs × averages per conversion.
    pub const fn conversion_us(&self) -> WaitUs {
        let n = 1u32 << self.averaging;
        let mut conv = self.channels.conversions();
        if self.temperature && (self.averaging > 0 || conv == 0) {
            conv += 1;
        }
        START_MEASURE_US + 25 + 25 * n * conv
    }
}

/// Angle result in 1/16 degree (0…5760).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Angle(pub u16);

impl Angle {
    /// Angle in 0.01 degree.
    pub const fn centidegrees(self) -> u32 {
        (self.0 as u32 * 100 + 8) / 16
    }
}

/// One conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Flux density, nT (disabled axes are 0).
    pub field: Vector3,
    /// Temperature, 0.01 °C.
    pub temperature: Option<i32>,
    /// Angle between the selected axes.
    pub angle: Option<Angle>,
    /// Vector magnitude code from the angle engine.
    pub magnitude: Option<u8>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::MagneticField(self.field));
        if let Some(t) = self.temperature {
            f(Quantity::Temperature(t));
        }
        if let Some(a) = self.angle {
            f(Quantity::Angle(a.centidegrees() as i32));
        }
    }
}

/// Field in nT from a 16-bit result for full-scale `range_mt`.
pub const fn field_nt(raw: i16, range_mt: i64) -> i32 {
    // B = raw / 2^16 × 2·B_R mT.
    let num = raw as i64 * range_mt * 1_000_000;
    let half = if num < 0 { -16_384 } else { 16_384 };
    ((num + half) / 32_768) as i32
}

/// Temperature in 0.01 °C from the 16-bit result (T0 = 25 °C at 17508,
/// 58 LSB/°C).
pub const fn temperature_centi(raw: i16) -> i32 {
    let num = (raw as i32 - 17_508) * 100;
    let half = if num < 0 { -29 } else { 29 };
    2_500 + (num + half) / 58
}

/// Decode the 12-byte result burst starting at T_MSB_RESULT.
pub fn decode(buf: &[u8; 12], config: &Config, version: Version) -> Measurement {
    let word = |i: usize| i16::from_be_bytes([buf[i], buf[i + 1]]);
    let (x, y, z) = config.channels.axes();
    let xy = version.range_mt(config.xy_wide);
    let zr = version.range_mt(config.z_wide);
    let axis = |on: bool, i: usize, r: i64| if on { field_nt(word(i), r) } else { 0 };
    let angle = config.angle != AngleAxes::Off;
    Measurement {
        field: Vector3 {
            x: axis(x, 2, xy),
            y: axis(y, 4, xy),
            z: axis(z, 6, zr),
        },
        temperature: config.temperature.then(|| temperature_centi(word(0))),
        angle: angle.then(|| Angle(u16::from_be_bytes([buf[9], buf[10]]) & 0x1FFF)),
        magnitude: angle.then_some(buf[11]),
    }
}

macro_rules! api {
    ($($asyncness:ident)?; $($aw:tt)*) => {
        /// Create a driver; no bus traffic. Call [`Self::probe`] (detects
        /// the range version) and [`Self::configure`].
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                version: Version::Mt40,
                config: Config {
                    channels: Channels::Xyz,
                    temperature: false,
                    averaging: 0,
                    xy_wide: false,
                    z_wide: false,
                    low_noise: false,
                    angle: AngleAxes::Off,
                    gain_second_axis: false,
                    tempco: MagnetTempco::None,
                    mode: Mode::Triggered,
                    sleep_time: SleepTime::Ms1,
                },
            }
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        /// Range version detected by [`Self::probe`].
        pub fn version(&self) -> Version {
            self.version
        }

        /// Write registers starting at `buf[0]`.
        pub $($asyncness)? fn write(&mut self, buf: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c.write(self.address, buf) $($aw)* .map_err(Error::Bus)
        }

        /// Read registers starting at `reg`.
        pub $($asyncness)? fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c.write_read(self.address, &[reg], buf) $($aw)* .map_err(Error::Bus)
        }

        /// Check the manufacturer ID and read the range version.
        pub $($asyncness)? fn probe(&mut self) -> Result<Version, Error<I2C::Error>> {
            let mut b = [0u8; 3];
            self.read(REG_DEVICE_ID, &mut b) $($aw)* ?;
            let id = u16::from_le_bytes([b[1], b[2]]);
            if id != MANUFACTURER_ID {
                return Err(Error::UnexpectedId(id as u32));
            }
            self.version = match b[0] & 0x03 {
                1 => Version::Mt40,
                2 => Version::Mt133,
                _ => return Err(Error::UnexpectedId(b[0] as u32)),
            };
            Ok(self.version)
        }

        /// Apply `config` and clear the POR flag. `InvalidArgument` if
        /// `averaging` > 5.
        pub $($asyncness)? fn configure(&mut self, config: &Config) -> Result<(), Error<I2C::Error>> {
            if config.averaging > 5 {
                return Err(Error::InvalidArgument);
            }
            self.write(&config.registers()) $($aw)* ?;
            self.write(&[REG_T_CONFIG, config.temperature as u8]) $($aw)* ?;
            self.write(&[REG_CONV_STATUS, POR]) $($aw)* ?;
            self.config = *config;
            Ok(())
        }

        /// Per-axis threshold codes (±128 = ±B_R; 0 disables) and
        /// INT_CONFIG_1.
        pub $($asyncness)? fn set_thresholds(&mut self, xyz: [i8; 3], int_config: u8) -> Result<(), Error<I2C::Error>> {
            let [x, y, z] = xyz;
            self.write(&[REG_X_THR_CONFIG, x as u8, y as u8, z as u8]) $($aw)* ?;
            self.write(&[REG_INT_CONFIG_1, int_config]) $($aw)*
        }

        /// Angle calibration: gain (n/256, 0 = 1.0) for the axis chosen by
        /// [`Config::gain_second_axis`] and offsets of both axes.
        pub $($asyncness)? fn set_angle_correction(&mut self, gain: u8, offsets: [i8; 2]) -> Result<(), Error<I2C::Error>> {
            self.write(&[REG_MAG_GAIN_CONFIG, gain, offsets[0] as u8, offsets[1] as u8]) $($aw)*
        }

        /// Move the device to `address` (lost at power-off).
        pub $($asyncness)? fn set_address(&mut self, address: u8) -> Result<(), Error<I2C::Error>> {
            self.write(&[REG_I2C_ADDRESS, (address << 1) | 1]) $($aw)* ?;
            self.address = address;
            Ok(())
        }

        $($asyncness)? fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.config.mode != Mode::Triggered {
                return Ok(0);
            }
            self.write(&[TRIGGER | REG_CONV_STATUS]) $($aw)* ?;
            Ok(self.config.conversion_us())
        }

        $($asyncness)? fn read_result(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut b = [0u8; 12];
            self.read(REG_T_MSB_RESULT, &mut b) $($aw)* ?;
            if b[8] & RESULT_STATUS == 0 {
                return Err(Error::NotReady);
            }
            Ok(decode(&b, &self.config, self.version))
        }

        $($asyncness)? fn enter_sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            let low_noise = (self.config.low_noise as u8) << 4;
            self.write(&[REG_DEVICE_CONFIG_2, low_noise | 0x01]) $($aw)*
        }

        $($asyncness)? fn wake_up(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            // A sleeping device wakes on its address but does not ACK.
            let _ = self.i2c.write(self.address, &[]) $($aw)*;
            let cfg2 = self.config.registers()[2];
            self.write(&[REG_DEVICE_CONFIG_2, cfg2]) $($aw)* ?;
            Ok(WAKE_US)
        }
    };
}

/// Blocking driver.
pub struct Tmag5273<I2C> {
    i2c: I2C,
    address: u8,
    version: Version,
    config: Config,
}

impl<I2C: embedded_hal::i2c::I2c> Tmag5273<I2C> {
    api!(;);
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Tmag5273<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.start()
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        self.read_result()
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Tmag5273<I2C> {
    type BusError = I2C::Error;

    /// Sleep mode (5 nA); results are lost, configuration kept.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.enter_sleep()
    }

    /// Wake (address NACK ignored) and restore the configured mode.
    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.wake_up()
    }
}

/// Async driver (`embedded-hal-async` I2C).
pub mod asynch {
    use super::*;

    /// Async twin of [`super::Tmag5273`].
    pub struct Tmag5273<I2C> {
        i2c: I2C,
        address: u8,
        version: Version,
        config: Config,
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Tmag5273<I2C> {
        api!(async; .await);
    }

    impl<I2C: embedded_hal_async::i2c::I2c> zigbee_sensor::asynch::Sensor for Tmag5273<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.start().await
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            self.read_result().await
        }
    }

    impl<I2C: embedded_hal_async::i2c::I2c> zigbee_sensor::asynch::LowPower for Tmag5273<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.enter_sleep().await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.wake_up().await
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
    use std::{vec, vec::Vec};
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = Address::A as u8;

    #[test]
    fn conversions_match_datasheet() {
        // ±40 mT: full-scale code → 40 mT; −32768 → −40 mT.
        assert_eq!(field_nt(i16::MAX, 40), 39_998_779);
        assert_eq!(field_nt(i16::MIN, 40), -40_000_000);
        assert_eq!(field_nt(1, 133), 4_059);
        assert_eq!(temperature_centi(17_508), 2_500);
        assert_eq!(temperature_centi(17_508 + 58 * 10), 3_500);
        assert_eq!(temperature_centi(17_508 - 58 * 65), -4_000);
        // §6.5.2.3 examples: 354.50° and 17.25°.
        assert_eq!(Angle(0b0001_0110_0010_1000).centidegrees(), 35_450);
        assert_eq!(Angle(0b0000_0001_0001_0100).centidegrees(), 1_725);
        let mut c = Config {
            channels: Channels::X,
            ..Config::default()
        };
        assert_eq!(c.conversion_us(), 70 + 50);
        c.averaging = 5;
        c.channels = Channels::Xy;
        assert_eq!(c.conversion_us(), 70 + 825 + 800);
    }

    #[test]
    fn probe_configure_trigger_read() {
        let cfg = Config {
            channels: Channels::Xyz,
            temperature: true,
            averaging: 2,
            z_wide: true,
            angle: AngleAxes::Xy,
            tempco: MagnetTempco::NdFeB,
            ..Config::default()
        };
        let mut i2c = Mock::new(&[
            T::write_read(A, vec![0x0D], vec![0x02, 0x49, 0x54]),
            T::write(A, vec![0x00, 0x28, 0x00, 0x70, 0x05]),
            T::write(A, vec![0x07, 0x01]),
            T::write(A, vec![0x18, 0x10]),
            T::write(A, vec![0x98]),
            T::write_read(A, vec![0x10], vec![0; 12]),
            T::write_read(
                A,
                vec![0x10],
                vec![
                    0x44, 0x64, 0x40, 0x00, 0xC0, 0x00, 0x40, 0x00, 0x01, 0x16, 0x28, 0x7F,
                ],
            ),
        ]);
        let mut s = Tmag5273::new(i2c.clone(), A);
        assert_eq!(s.probe(), Ok(Version::Mt133));
        s.configure(&cfg).unwrap();
        assert_eq!(s.start_measurement(), Ok(70 + 25 + 25 * 4 * 4));
        assert_eq!(s.read_measurement(), Err(Error::NotReady));
        let m = s.read_measurement().unwrap();
        // 0x4000 = ¼ of the 2^15 span: ±133 → 66.5 mT, Z wide → 133 mT.
        assert_eq!(
            m.field,
            Vector3 {
                x: 66_500_000,
                y: -66_500_000,
                z: 133_000_000
            }
        );
        assert_eq!(m.temperature, Some(2_500));
        assert_eq!(m.angle, Some(Angle(0x1628)));
        assert_eq!(m.magnitude, Some(0x7F));
        let mut out = Vec::new();
        m.for_each(&mut |q| out.push(q));
        assert_eq!(
            out[1..],
            [Quantity::Temperature(2_500), Quantity::Angle(35_450)]
        );
        i2c.done();
    }

    #[test]
    fn probe_rejects_foreign_id() {
        let mut i2c = Mock::new(&[T::write_read(A, vec![0x0D], vec![0x01, 0x00, 0x00])]);
        let mut s = Tmag5273::new(i2c.clone(), A);
        assert_eq!(s.probe(), Err(Error::UnexpectedId(0)));
        let bad = Config {
            averaging: 6,
            ..Config::default()
        };
        assert_eq!(s.configure(&bad), Err(Error::InvalidArgument));
        i2c.done();
    }

    #[test]
    fn sleep_and_wake_ignore_nack() {
        let mut i2c = Mock::new(&[
            T::write(A, vec![0x01, 0x01]),
            T::write(A, vec![]).with_error(ErrorKind::Other),
            T::write(A, vec![0x01, 0x00]),
        ]);
        let mut s = Tmag5273::new(i2c.clone(), A);
        s.sleep().unwrap();
        assert_eq!(s.wake(), Ok(WAKE_US));
        i2c.done();
    }

    #[test]
    fn async_continuous_mode_reads_without_trigger() {
        use zigbee_sensor::asynch::Sensor as _;
        let cfg = Config {
            channels: Channels::Z,
            mode: Mode::Continuous,
            ..Config::default()
        };
        let mut i2c = Mock::new(&[
            T::write(A, vec![0x00, 0x00, 0x02, 0x40, 0x00]),
            T::write(A, vec![0x07, 0x00]),
            T::write(A, vec![0x18, 0x10]),
            T::write_read(
                A,
                vec![0x10],
                vec![0, 0, 0, 0, 0, 0, 0xFF, 0xFF, 0x01, 0, 0, 0],
            ),
        ]);
        let mut s = asynch::Tmag5273::new(i2c.clone(), A);
        block_on(s.configure(&cfg)).unwrap();
        assert_eq!(block_on(s.start_measurement()), Ok(0));
        let m = block_on(s.read_measurement()).unwrap();
        assert_eq!(
            m.field,
            Vector3 {
                x: 0,
                y: 0,
                z: -1_221
            }
        );
        assert_eq!((m.temperature, m.angle), (None, None));
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
