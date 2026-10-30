extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
use std::vec;
use zigbee_sensor::blocking::{LowPower, Sensor};

const ADDR: u8 = 0x67;

fn block_on<F: core::future::Future>(fut: F) -> F::Output {
    let mut cx = core::task::Context::from_waker(core::task::Waker::noop());
    let mut fut = core::pin::pin!(fut);
    loop {
        if let core::task::Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return v;
        }
    }
}

fn reg_read(reg: u8, data: &[u8]) -> [Transaction; 2] {
    [
        Transaction::write(ADDR, vec![reg]),
        Transaction::read(ADDR, data.to_vec()),
    ]
}

#[test]
fn register_formats() {
    // Register 5-2 / 5-4 format: 0.0625 °C two's complement.
    assert_eq!(centi([0x01, 0x90]), 2_500);
    assert_eq!(centi([0x19, 0x00]), 40_000);
    assert_eq!(centi([0xFF, 0x90]), -700);
    assert_eq!(centi([0x00, 0x01]), 6);
    assert_eq!(centi([0xFF, 0xFF]), -6);
    assert_eq!(centi([0xFC, 0x90]), -5_500);
    assert_eq!(raw_adc([0x01, 0xFF, 0xFF]), 131_071);
    assert_eq!(raw_adc([0xFE, 0x00, 0x00]), -131_072);
}

#[test]
fn device_id_and_errata() {
    let id = DeviceId::decode::<()>([0x40, 0x12]).unwrap();
    assert_eq!(id.variant, Variant::Mcp9600);
    assert!(id.has_i2c_errata());
    assert!(
        !DeviceId::decode::<()>([0x40, 0x13])
            .unwrap()
            .has_i2c_errata()
    );
    let id = DeviceId::decode::<()>([0x41, 0x10]).unwrap();
    assert_eq!(id.variant, Variant::Mcp9601);
    assert!(!id.has_i2c_errata());
    assert_eq!(
        DeviceId::decode::<()>([0x00, 0x00]),
        Err(Error::UnexpectedId(0))
    );
}

#[test]
fn status_checks() {
    assert_eq!(check_status::<()>(0xC0, Mode::Burst(0)), Ok(()));
    assert_eq!(
        check_status::<()>(0x40, Mode::Burst(0)),
        Err(Error::NotReady)
    );
    assert_eq!(check_status::<()>(0x40, Mode::Continuous), Ok(()));
    assert_eq!(
        check_status::<()>(0x80, Mode::Continuous),
        Err(Error::NotReady)
    );
    assert_eq!(
        check_status::<()>(0xD0, Mode::Burst(0)),
        Err(Error::InvalidData)
    );
    assert_eq!(
        check_status::<()>(0xE0, Mode::Burst(0)),
        Err(Error::InvalidData)
    );
}

#[test]
fn burst_measurement() {
    let mut script = vec![];
    script.extend(reg_read(0x20, &[0x40, 0x13]));
    script.extend([
        Transaction::write(ADDR, vec![0x05, 0x00]),
        Transaction::write(ADDR, vec![0x04, 0x00]),
        Transaction::write(ADDR, vec![0x06, 0x02]),
    ]);
    script.extend(reg_read(0x04, &[0x40]));
    script.extend(reg_read(0x04, &[0xC0]));
    script.extend(reg_read(0x00, &[0x01, 0x90]));
    script.extend(reg_read(0x02, &[0x01, 0x80]));
    script.extend([
        Transaction::write(ADDR, vec![0x04, 0x00]),
        Transaction::write(ADDR, vec![0x06, 0x02]),
    ]);
    let mut i2c = Mock::new(&script);
    let mut dev = Mcp9600::new(i2c.clone(), ADDR);
    assert_eq!(dev.probe().unwrap().variant, Variant::Mcp9600);
    // (320 ms ADC + 12 ms tCALC) × 1.25.
    assert_eq!(dev.start_measurement(), Ok(415_000));
    assert_eq!(dev.read_measurement(), Err(Error::NotReady));
    assert_eq!(
        dev.read_measurement(),
        Ok(Measurement {
            temperature: 2_500,
            cold_junction: 2_400
        })
    );
    // Sensor configuration is written only once.
    assert_eq!(dev.start_measurement(), Ok(415_000));
    i2c.done();
}

#[test]
fn continuous_mode_low_power_and_config() {
    let mut i2c = Mock::new(&[
        Transaction::write(ADDR, vec![0x05, 0x14]),
        Transaction::write(ADDR, vec![0x06, 0xE1]),
        Transaction::write(ADDR, vec![0x04, 0x00]),
        Transaction::write(ADDR, vec![0x06, 0xE0]),
        Transaction::write(ADDR, vec![0x06, 0xE1]),
        Transaction::write(ADDR, vec![0x06, 0xE0]),
    ]);
    let mut dev = Mcp9600::new(i2c.clone(), ADDR)
        .with_thermocouple(ThermocoupleType::J)
        .with_filter(4)
        .with_resolution(AdcResolution::Bits12, ColdJunctionResolution::Coarse)
        .with_mode(Mode::Continuous);
    dev.configure().unwrap();
    // (63 ms cold junction + 12 ms) × 1.25.
    assert_eq!(dev.start_measurement(), Ok(93_750));
    dev.sleep().unwrap();
    assert_eq!(dev.wake(), Ok(93_750));
    i2c.done();

    let burst = Mcp9600::new((), ADDR)
        .with_resolution(AdcResolution::Bits14, ColdJunctionResolution::Coarse)
        .with_mode(Mode::Burst(9));
    assert_eq!(burst.conversion_us(), 128 * 75_000 / 4 * 5);
}

#[test]
fn async_open_thermocouple_and_raw_adc() {
    use zigbee_sensor::asynch::Sensor as _;
    let mut script = vec![
        Transaction::write(ADDR, vec![0x05, 0x70]),
        Transaction::write(ADDR, vec![0x04, 0x00]),
        Transaction::write(ADDR, vec![0x06, 0x0E]),
    ];
    script.extend(reg_read(0x04, &[0x90]));
    script.extend(reg_read(0x03, &[0x00, 0x10, 0x00]));
    let mut i2c = Mock::new(&script);
    let mut dev = asynch::Mcp9600::new(i2c.clone(), ADDR)
        .with_thermocouple(ThermocoupleType::R)
        .with_mode(Mode::Burst(3));
    assert_eq!(block_on(dev.start_measurement()), Ok(8 * 332_000 / 4 * 5));
    assert_eq!(block_on(dev.read_measurement()), Err(Error::InvalidData));
    assert_eq!(block_on(dev.read_raw_adc()), Ok(0x1000));
    i2c.done();
}
