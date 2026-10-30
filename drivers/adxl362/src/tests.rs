extern crate std;

use super::*;
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
use std::vec;
use std::vec::Vec;
use zigbee_sensor::blocking::{LowPower, Sensor};

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

#[test]
fn scale_factors_match_datasheet_table_1() {
    // 1000 / 500 / 250 LSB/g.
    assert_eq!(raw_to_mg(1000, Range::G2), 1000);
    assert_eq!(raw_to_mg(-250, Range::G8), -1000);
    assert_eq!(threshold_reg(250, Range::G2), 250);
    assert_eq!(threshold_reg(250, Range::G8), 63);
    assert_eq!(threshold_reg(5000, Range::G2), 0x7FF);
    // Measurement command to valid data: 4/ODR.
    assert_eq!(turn_on_us(PowerMode::Normal, DataRate::Hz100), 40_000);
}

#[test]
fn probe_checks_all_three_ids() {
    let exp = script(&[&r(0x00, &[0xAD, 0x1D, 0xF2]), &r(0x00, &[0xAD, 0x1D, 0xF3])]);
    let mut spi = SpiMock::new(&exp);
    let mut acc = Adxl362::new(&mut spi);
    assert_eq!(acc.probe(), Ok(()));
    assert_eq!(acc.probe(), Err(Error::UnexpectedId(0x00AD_1DF3)));
    spi.done();
}

#[test]
fn autonomous_motion_switch_example() {
    // Datasheet "Autonomous Motion Switch": 250 mg activity, 150 mg / 30
    // samples inactivity, loop mode, referenced, AWAKE on INT2, wake-up mode.
    let exp = script(&[
        &w(0x2C, 0x10),
        &w(0x2A, 0x00),
        &w(0x2B, 0x00),
        &w(0x2D, 0x0A),
        &w(0x2D, 0x08), // standby while reconfiguring
        &w(0x20, 0xFA),
        &w(0x21, 0x00),
        &w(0x22, 0x00),
        &w(0x23, 0x96),
        &w(0x24, 0x00),
        &w(0x25, 0x1E),
        &w(0x26, 0x00),
        &w(0x27, 0x3F),
        &w(0x2A, 0x00),
        &w(0x2B, 0x40),
        &w(0x2D, 0x0A),
    ]);
    let mut spi = SpiMock::new(&exp);
    let mut acc = Adxl362::new(&mut spi);
    let cfg = Config {
        mode: PowerMode::WakeUp,
        ..Config::default()
    };
    assert_eq!(acc.configure(cfg), Ok(4 * WAKE_UP_PERIOD_US));
    acc.configure_wake_up(&WakeUpConfig {
        pin: IntPin::Int2,
        inactivity: Some(InactivityConfig::default()),
        linking: Linking::Loop,
        awake_output: true,
        ..WakeUpConfig::default()
    })
    .unwrap();
    spi.done();
}

#[test]
fn invalid_linking_is_unsupported() {
    let mut spi = SpiMock::new(&[]);
    let mut acc = Adxl362::new(&mut spi);
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
    spi.done();
}

#[test]
fn measurement_keeps_activity_seen_in_status() {
    let exp = script(&[
        &r(0x0B, &[0x10, 0, 0, 0, 0, 0, 0, 0, 0]),
        &r(0x0B, &[0x41, 0, 0, 0xE8, 0x03, 0, 0, 0x18, 0xFC]),
        &r(0x0B, &[0x40]),
    ]);
    let mut spi = SpiMock::new(&exp);
    let mut acc = Adxl362::new(&mut spi);
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
    assert!(src.activity() && src.awake() && !src.inactivity());
    spi.done();
}

#[test]
fn standby_and_pin_polarity() {
    let exp = script(&[
        &w(0x2C, 0x13),
        &w(0x2A, 0x80),
        &w(0x2B, 0x80),
        &w(0x2D, 0x12), // low noise, measure
        &w(0x2D, 0x10),
        &w(0x2B, 0x80),
        &w(0x2D, 0x12),
    ]);
    let mut spi = SpiMock::new(&exp);
    let mut acc = Adxl362::new(&mut spi);
    let cfg = Config {
        rate: DataRate::Hz100,
        mode: PowerMode::LowNoise,
        int_active_low: true,
        ..Config::default()
    };
    assert_eq!(acc.configure(cfg), Ok(40_000));
    acc.sleep().unwrap();
    acc.disable_interrupt(IntPin::Int2).unwrap();
    assert_eq!(acc.wake(), Ok(40_000));
    spi.done();
}

#[test]
fn async_measure_from_standby() {
    use zigbee_sensor::asynch::Sensor as _;

    struct NoDelay;
    impl embedded_hal_async::delay::DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    let exp = script(&[
        &w(0x2D, 0x02),
        &r(0x0B, &[0x01, 0, 0, 0, 0, 0, 0, 0xF4, 0x01]),
    ]);
    let mut spi = SpiMock::new(&exp);
    let mut acc = asynch::Adxl362::new(&mut spi);
    let m = block_on(acc.measure(&mut NoDelay)).unwrap();
    assert_eq!(m.acceleration.z, 500);
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
