extern crate std;

use super::*;
use embedded_hal_mock::eh1::spi::{Mock, Transaction};
use std::vec;
use std::vec::Vec;
use zigbee_sensor::blocking::Sensor;

fn write(reg: u8, data: &[u8]) -> Vec<Transaction<u8>> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x80 | reg]),
        Transaction::write_vec(data.to_vec()),
        Transaction::transaction_end(),
    ]
}

fn read(reg: u8, data: &[u8]) -> Vec<Transaction<u8>> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![reg]),
        Transaction::read_vec(data.to_vec()),
        Transaction::transaction_end(),
    ]
}

fn script(parts: &[Vec<Transaction<u8>>]) -> Mock<u8> {
    Mock::new(&parts.concat())
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

#[test]
fn decode_pt100_and_pt1000() {
    // 138.5055 Ω (100 °C) with 430 Ω reference → code 10555.
    let m = decode::<()>((10555u16 << 1).to_be_bytes(), 430_000, 100_000).unwrap();
    assert_eq!(
        m,
        Measurement {
            temperature: 10_001,
            resistance_mohm: 138_509
        }
    );
    // 80.306 Ω (−50 °C) → code 6120.
    let m = decode::<()>((6120u16 << 1).to_be_bytes(), 430_000, 100_000).unwrap();
    assert_eq!(m.temperature, -4_999);
    // PT1000 with 4.3 kΩ gives the same codes.
    let m = decode::<()>((10555u16 << 1).to_be_bytes(), 4_300_000, 1_000_000).unwrap();
    assert_eq!((m.temperature, m.resistance_mohm), (10_001, 1_385_086));
    // 390.48 Ω (850 °C) → code 29756.
    let m = decode::<()>((29756u16 << 1).to_be_bytes(), 430_000, 100_000).unwrap();
    assert_eq!(m.temperature, 84_998);
    assert_eq!(
        decode::<()>([0x52, 0x77], 430_000, 100_000),
        Err(Error::InvalidData)
    );
    assert_eq!(
        decode::<()>([0, 0], 430_000, 100_000),
        Err(Error::InvalidData)
    );
}

#[test]
fn normally_off_flow_uses_auto_mode() {
    let mut spi = script(&[
        write(0x00, &[0xC0]),
        read(0x01, &[0x52, 0x76]),
        write(0x00, &[0x00]),
    ]);
    let mut dev = Max31865::new(spi.clone());
    // First conversion 55 ms (60 Hz), then one 17.6 ms period after settling.
    assert_eq!(dev.start_measurement(), Ok(72_600));
    assert_eq!(dev.read_measurement().unwrap().temperature, 10_001);
    spi.done();
}

#[test]
fn slow_rc_filter_extends_auto_wait() {
    let mut spi = script(&[write(0x00, &[0xC1])]);
    let mut dev = Max31865::new(spi.clone())
        .with_filter(Filter::Hz50)
        .with_bias_settle_us(80_000);
    // 66 → 87 (≥ 80 ms settled) → 108 ms.
    assert_eq!(dev.start_measurement(), Ok(108_000));
    spi.done();
}

#[test]
fn bias_on_uses_one_shot_three_wire_50hz() {
    let mut spi = script(&[
        write(0x00, &[0x91]),
        write(0x00, &[0xB1]),
        read(0x01, &[0x2F, 0xD0]),
        write(0x00, &[0x11]),
    ]);
    let mut dev = Max31865::new(spi.clone())
        .with_wiring(Wiring::ThreeWire)
        .with_filter(Filter::Hz50);
    assert_eq!(dev.enable_bias(), Ok(11_500));
    assert_eq!(dev.start_measurement(), Ok(66_000));
    assert_eq!(dev.read_measurement().unwrap().temperature, -4_999);
    dev.disable_bias().unwrap();
    spi.done();
}

#[test]
fn faults() {
    let mut spi = script(&[
        write(0x00, &[0xC0]),
        read(0x01, &[0xFF, 0xFF]),
        write(0x00, &[0x00]),
        read(0x07, &[0x84]),
        write(0x00, &[0x02]),
        write(0x00, &[0x84]),
        read(0x00, &[0x84]),
        read(0x00, &[0x80]),
        write(0x03, &[0xEE, 0x22, 0x0A, 0xB6]),
    ]);
    let mut dev = Max31865::new(spi.clone());
    dev.start_measurement().unwrap();
    assert_eq!(dev.read_measurement(), Err(Error::InvalidData));
    let status = dev.read_fault_status().unwrap();
    assert!(status.rtd_high_threshold() && status.over_under_voltage());
    assert!(!status.refin_high() && status.any());
    dev.clear_faults().unwrap();
    assert_eq!(dev.start_fault_detection(), Ok(FAULT_DETECTION_US));
    assert_eq!(dev.fault_detection_done(), Ok(false));
    assert_eq!(dev.fault_detection_done(), Ok(true));
    let low = dev.code_for_resistance(18_000);
    let high = dev.code_for_resistance(400_000);
    dev.set_fault_thresholds(low, high).unwrap();
    spi.done();
}

#[test]
fn async_normally_off_flow() {
    use zigbee_sensor::asynch::Sensor as _;
    let mut spi = script(&[
        write(0x00, &[0xC0]),
        read(0x01, &[0x0B, 0x06]),
        write(0x00, &[0x00]),
    ]);
    let mut dev = asynch::Max31865::new(spi.clone());
    assert_eq!(block_on(dev.start_measurement()), Ok(72_600));
    let m = block_on(dev.read_measurement()).unwrap();
    // 18.52 Ω, code 1411: −200 °C.
    assert_eq!(m.temperature, -20_001);
    spi.done();
}
