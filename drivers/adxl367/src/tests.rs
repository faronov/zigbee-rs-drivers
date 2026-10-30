extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cT};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
use std::vec;
use std::vec::Vec;
use zigbee_sensor::blocking::{LowPower, Sensor};

const ADDR: u8 = ADDRESS_ASEL_HIGH;

fn w(reg: u8, value: u8) -> [SpiT<u8>; 3] {
    [
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0x0A, reg, value]),
        SpiT::transaction_end(),
    ]
}

fn r(reg: u8, data: &[u8]) -> [SpiT<u8>; 4] {
    [
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0x0B, reg]),
        SpiT::read_vec(data.to_vec()),
        SpiT::transaction_end(),
    ]
}

fn script(parts: &[&[SpiT<u8>]]) -> Vec<SpiT<u8>> {
    parts.iter().flat_map(|p| p.iter().cloned()).collect()
}

fn iw(reg: u8, value: u8) -> I2cT {
    I2cT::write(ADDR, vec![reg, value])
}

fn ir(reg: u8, data: &[u8]) -> I2cT {
    I2cT::write_read(ADDR, vec![reg], data.to_vec())
}

#[test]
fn scale_factors_match_datasheet_table_1() {
    // 4000 / 2000 / 1000 LSB/g, 14-bit left-justified big-endian.
    assert_eq!(raw_to_mg(4000, Range::G2), 1000);
    assert_eq!(raw_to_mg(-1000, Range::G8), -1000);
    assert_eq!(raw_to_mg(-3, Range::G2), 0);
    let v = decode(&[0x3E, 0x80, 0x07, 0xD0, 0xC1, 0x80], Range::G4);
    assert_eq!(
        v,
        Vector3 {
            x: 2000,
            y: 250,
            z: -2000
        }
    );
    assert_eq!(threshold_reg(5000, Range::G2), 0x1FFF);
    assert_eq!(threshold_reg(175, Range::G8), 175);
    // Measurement settling and slowest wake-up rate.
    assert_eq!(turn_on_us(PowerMode::Normal, WakeUpRate::Ms640), 100_000);
    assert_eq!(turn_on_us(PowerMode::WakeUp, WakeUpRate::Ms80), 100_000);
    assert_eq!(turn_on_us(PowerMode::WakeUp, WakeUpRate::Ms640), 650_000);
}

#[test]
fn threshold_bytes_match_motion_switch_routine() {
    // "Start-Up Routine", steps 7 and 8 (±2 g): 0x0F/0xA0 and 0x0A/0xF0.
    // The datasheet labels the first as 62.5 mg, but 1000 codes at
    // 0.25 mg/LSB is 250 mg; the second (700 codes) is the stated 175 mg.
    assert_eq!(threshold_bytes(threshold_reg(250, Range::G2)), [0x0F, 0xA0]);
    assert_eq!(threshold_bytes(threshold_reg(175, Range::G2)), [0x0A, 0xF0]);
}

#[test]
fn probe_checks_all_three_ids_over_i2c() {
    let exp = [ir(0x00, &[0xAD, 0x1D, 0xF7]), ir(0x00, &[0xAD, 0x1D, 0xF2])];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Adxl367::new_i2c(&mut i2c, ADDR);
    assert_eq!(acc.probe(), Ok(()));
    assert_eq!(acc.probe(), Err(Error::UnexpectedId(0x00AD_1DF2)));
    i2c.done();
}

#[test]
fn autonomous_motion_switch_over_spi() {
    // Referenced activity 250 mg, referenced inactivity 175 mg, loop mode,
    // AWAKE on INT2 (datasheet "Autonomous Motion Switch").
    let exp = script(&[
        &w(0x2C, 0x23),
        &w(0x2A, 0x00),
        &w(0x2B, 0x00),
        &w(0x39, 0x00),
        &w(0x2D, 0x02),
        &w(0x2D, 0x00), // standby while reconfiguring
        &w(0x20, 0x0F),
        &w(0x21, 0xA0),
        &w(0x22, 0x00),
        &w(0x23, 0x0A),
        &w(0x24, 0xF0),
        &w(0x25, 0x00),
        &w(0x26, 0x1E),
        &w(0x27, 0x3F),
        &w(0x2A, 0x00),
        &w(0x2B, 0x40),
        &w(0x2D, 0x02),
    ]);
    let mut spi = SpiMock::new(&exp);
    let mut acc = Adxl367::new_spi(&mut spi);
    assert_eq!(acc.configure(Config::default()), Ok(100_000));
    acc.configure_wake_up(&WakeUpConfig {
        pin: IntPin::Int2,
        inactivity: Some(InactivityConfig {
            threshold_mg: 175,
            ..InactivityConfig::default()
        }),
        linking: Linking::Loop,
        awake_output: true,
        ..WakeUpConfig::default()
    })
    .unwrap();
    spi.done();
}

#[test]
fn invalid_linking_is_unsupported() {
    let mut i2c = I2cMock::new(&[]);
    let mut acc = Adxl367::new_i2c(&mut i2c, ADDR);
    let linked = WakeUpConfig {
        linking: Linking::Linked,
        ..WakeUpConfig::default()
    };
    assert_eq!(acc.configure_wake_up(&linked), Err(Error::Unsupported));
    let sleep = WakeUpConfig {
        inactivity: Some(InactivityConfig::default()),
        auto_sleep: true,
        ..WakeUpConfig::default()
    };
    assert_eq!(acc.configure_wake_up(&sleep), Err(Error::Unsupported));
    i2c.done();
}

#[test]
fn measurement_keeps_activity_seen_in_status() {
    let exp = [
        ir(0x0B, &[0x10, 0, 0, 0, 0, 0, 0, 0, 0]),
        ir(0x0B, &[0x41, 0, 0, 0x3E, 0x80, 0, 0, 0xC1, 0x80]),
        ir(0x0B, &[0x40]),
        ir(0x45, &[0x00]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Adxl367::new_i2c(&mut i2c, ADDR);
    assert_eq!(acc.read_measurement(), Err(Error::NotReady));
    let m = acc.read_measurement().unwrap();
    assert_eq!(
        m.acceleration,
        Vector3 {
            x: 1000,
            y: 0,
            z: -1000
        }
    );
    let src = acc.read_interrupt_source().unwrap();
    assert!(src.activity() && src.awake() && !src.inactivity() && !src.single_tap());
    i2c.done();
}

#[test]
fn double_tap_on_int2() {
    let exp = [
        iw(0x2F, 0x30), // 3 g / 62.5 mg
        iw(0x30, 0x20), // 20 ms / 625 µs
        iw(0x31, 0x50), // 100 ms / 1.25 ms
        iw(0x32, 0xC8), // 250 ms / 1.25 ms
        iw(0x43, 0x20), // Z axis
        iw(0x3A, 0x00),
        iw(0x3B, 0x03),
        ir(0x0B, &[0x40]),
        ir(0x45, &[0x02]),
        iw(0x2F, 0x00),
        iw(0x3A, 0x00),
        iw(0x3B, 0x00),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Adxl367::new_i2c(&mut i2c, ADDR);
    acc.configure_tap(&TapConfig {
        double: true,
        pin: IntPin::Int2,
        ..TapConfig::default()
    })
    .unwrap();
    let src = acc.read_interrupt_source().unwrap();
    assert!(src.double_tap() && !src.single_tap());
    acc.disable_tap().unwrap();
    i2c.done();
}

#[test]
fn wake_up_mode_standby_and_pin_polarity() {
    let exp = [
        iw(0x2C, 0xA0),
        iw(0x2A, 0x80),
        iw(0x2B, 0x80),
        iw(0x39, 0xC0),
        iw(0x2D, 0x0A), // wake-up, measure
        iw(0x2D, 0x08),
        iw(0x2B, 0x80),
        iw(0x3B, 0x00),
        iw(0x2D, 0x0A),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Adxl367::new_i2c(&mut i2c, ADDR);
    let cfg = Config {
        range: Range::G8,
        rate: DataRate::Hz12_5,
        mode: PowerMode::WakeUp,
        wake_up_rate: WakeUpRate::Ms640,
        int_active_low: true,
    };
    assert_eq!(acc.configure(cfg), Ok(650_000));
    acc.sleep().unwrap();
    acc.disable_interrupt(IntPin::Int2).unwrap();
    assert_eq!(acc.wake(), Ok(650_000));
    i2c.done();
}

#[test]
fn async_measure_from_standby_over_spi() {
    use zigbee_sensor::asynch::Sensor as _;

    struct NoDelay;
    impl embedded_hal_async::delay::DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    let exp = script(&[
        &w(0x2D, 0x02),
        &r(0x0B, &[0x01, 0, 0, 0, 0, 0, 0, 0x07, 0xD0]),
    ]);
    let mut spi = SpiMock::new(&exp);
    let mut acc = asynch::Adxl367::new_spi(&mut spi);
    let m = block_on(acc.measure(&mut NoDelay)).unwrap();
    assert_eq!(m.acceleration.z, 125);
    spi.done();
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
