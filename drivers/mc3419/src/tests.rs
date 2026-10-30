extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cT};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
use std::vec;
use std::vec::Vec;
use zigbee_sensor::blocking::{LowPower, Sensor};

const ADDR: u8 = ADDRESS_A6_LOW;

fn w(reg: u8, value: u8) -> I2cT {
    I2cT::write(ADDR, vec![reg, value])
}

fn r(reg: u8, data: &[u8]) -> I2cT {
    I2cT::write_read(ADDR, vec![reg], data.to_vec())
}

fn spi_r(reg: u8, data: &[u8]) -> Vec<SpiT<u8>> {
    vec![
        SpiT::transaction_start(),
        SpiT::write_vec(vec![reg | 0x80, 0x00]),
        SpiT::read_vec(data.to_vec()),
        SpiT::transaction_end(),
    ]
}

fn spi_w(reg: u8, value: u8) -> Vec<SpiT<u8>> {
    vec![
        SpiT::transaction_start(),
        SpiT::write_vec(vec![reg, value]),
        SpiT::transaction_end(),
    ]
}

#[test]
fn scaling_and_register_codes() {
    // Table 5: 16384/8192/4096/2730/2048 LSB/g.
    assert_eq!(raw_to_mg(16384, Range::G2), 1000);
    assert_eq!(raw_to_mg(-2730, Range::G12), -1000);
    assert_eq!(raw_to_mg(i16::MIN, Range::G16), -16000);
    let v = decode(&[0x00, 0x40, 0x00, 0x00, 0x00, 0xC0], Range::G2);
    assert_eq!(
        v,
        Vector3 {
            x: 1000,
            y: 0,
            z: -1000
        }
    );
    assert_eq!(any_motion_threshold_reg(250, Range::G2), 4096);
    assert_eq!(any_motion_threshold_reg(250, Range::G12), 683);
    assert_eq!(any_motion_threshold_reg(5000, Range::G2), 0x7FFF);
    // Tables 20/28 (MC3419 selection A) and MC3479 Table 20.
    assert_eq!(sample_rate_reg(Part::Mc3419, DataRate::Hz25), Some(0x10));
    assert_eq!(sample_rate_reg(Part::Mc3419, DataRate::Hz1000), Some(0x17));
    assert_eq!(sample_rate_reg(Part::Mc3419, DataRate::Hz2000), None);
    assert_eq!(sample_rate_reg(Part::Mc3479, DataRate::Hz2000), Some(0x0F));
    assert_eq!(sample_rate_reg(Part::Mc3479, DataRate::Hz25), None);
    // Start-up time 1/ODR + 1 ms.
    assert_eq!(turn_on_us(DataRate::Hz100), 11_000);
}

#[test]
fn probe_checks_chip_id() {
    let exp = [r(0x18, &[0xA4]), r(0x18, &[0x88])];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Mc3419::new_i2c(&mut i2c, ADDR);
    assert_eq!(acc.probe(), Ok(()));
    assert_eq!(acc.probe(), Err(Error::UnexpectedId(0x88)));
    i2c.done();
}

#[test]
fn configure_writes_in_standby_then_wakes() {
    let exp = [
        w(0x07, 0x00),
        w(0x08, 0x09), // MC3479 100 Hz
        w(0x20, 0x2D), // ±8 g, LPF ODR/16
        w(0x33, 0xCC), // both pins push-pull active high
        w(0x07, 0x01),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Mc3419::new_i2c(&mut i2c, ADDR);
    let bad = Config {
        part: Part::Mc3479,
        rate: DataRate::Hz25,
        ..Config::default()
    };
    assert_eq!(acc.configure(bad), Err(Error::Unsupported));
    let cfg = Config {
        part: Part::Mc3479,
        range: Range::G8,
        rate: DataRate::Hz100,
        low_pass: Some(LowPass::Div16),
        int_push_pull: true,
        int_active_high: true,
    };
    assert_eq!(acc.configure(cfg), Ok(11_000));
    i2c.done();
}

#[test]
fn any_motion_on_int2_and_clear() {
    let exp = [
        w(0x07, 0x00),
        w(0x08, 0x10),
        w(0x20, 0x00),
        w(0x33, 0x00),
        w(0x07, 0x01),
        // configure_motion: back to STANDBY, program, WAKE again.
        w(0x07, 0x00),
        w(0x43, 0x00),
        w(0x44, 0x10), // 250 mg = 4096 LSB at ±2 g
        w(0x45, 0x05),
        w(0x09, 0x04),
        w(0x31, 0x10),
        w(0x06, 0x44), // ANYM + AUTO_CLR
        w(0x07, 0x01),
        r(0x14, &[0x04]),
        w(0x14, 0x00),
        r(0x14, &[0x00]),
        w(0x07, 0x00),
        w(0x06, 0x00),
        w(0x09, 0x00),
        w(0x07, 0x01),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Mc3419::new_i2c(&mut i2c, ADDR);
    let cfg = Config {
        rate: DataRate::Hz25,
        ..Config::default()
    };
    assert_eq!(acc.configure(cfg), Ok(41_000));
    acc.configure_motion(&MotionConfig {
        debounce: 5,
        pin: IntPin::Int2,
        latch: false,
        ..MotionConfig::default()
    })
    .unwrap();
    let src = acc.read_interrupt_source().unwrap();
    assert!(src.any_motion() && !src.shake() && !src.data_ready());
    assert_eq!(acc.read_interrupt_source(), Ok(InterruptSource(0)));
    acc.disable_motion().unwrap();
    i2c.done();
}

#[test]
fn not_ready_then_sample_over_spi() {
    let exp: Vec<_> = [
        spi_w(0x07, 0x01),
        spi_r(0x0D, &[0, 0, 0, 0, 0, 0, 0x00]),
        spi_r(0x0D, &[0x00, 0x40, 0x00, 0x00, 0x00, 0xC0, 0x80]),
        spi_r(0x05, &[0x01]),
        spi_w(0x07, 0x00),
        spi_r(0x05, &[0x00]),
    ]
    .concat();
    let mut spi = SpiMock::new(&exp);
    let mut acc = Mc3419::new_spi(&mut spi);
    assert_eq!(acc.start_measurement(), Ok(21_000));
    assert_eq!(acc.start_measurement(), Ok(0));
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
    assert_eq!(acc.state(), Ok(State::Wake));
    acc.sleep().unwrap();
    assert_eq!(acc.state(), Ok(State::Standby));
    spi.done();
}

#[test]
fn motion_config_in_standby_does_not_wake() {
    let exp = [
        w(0x43, 0x9A),
        w(0x44, 0x01), // 25 mg at ±2 g = 410 LSB
        w(0x45, 0x00),
        w(0x09, 0x04),
        w(0x31, 0x00),
        w(0x06, 0x04),
        w(0x07, 0x01),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Mc3419::new_i2c(&mut i2c, ADDR);
    acc.configure_motion(&MotionConfig {
        threshold_mg: 25,
        ..MotionConfig::default()
    })
    .unwrap();
    assert_eq!(acc.wake(), Ok(21_000));
    i2c.done();
}

#[test]
fn async_measure_over_i2c() {
    use zigbee_sensor::asynch::Sensor as _;

    struct NoDelay;
    impl embedded_hal_async::delay::DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    let exp = [
        w(0x07, 0x01),
        r(0x0D, &[0x00, 0x00, 0x00, 0xE0, 0x00, 0x20, 0x80]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = asynch::Mc3419::new_i2c(&mut i2c, ADDR);
    let m = block_on(acc.measure(&mut NoDelay)).unwrap();
    assert_eq!(
        m.acceleration,
        Vector3 {
            x: 0,
            y: -500,
            z: 500
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
