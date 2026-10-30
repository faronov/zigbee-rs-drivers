extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cT};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
use std::vec;
use std::vec::Vec;
use zigbee_sensor::blocking::{LowPower, Sensor};

const ADDR: u8 = ADDRESS_SDO_LOW;

fn w(reg: u8, value: u8) -> I2cT {
    I2cT::write(ADDR, vec![reg, value])
}

fn r(reg: u8, data: &[u8]) -> I2cT {
    I2cT::write_read(ADDR, vec![reg], data.to_vec())
}

fn spi_r(reg: u8, data: &[u8]) -> Vec<SpiT<u8>> {
    vec![
        SpiT::transaction_start(),
        SpiT::write_vec(vec![reg | 0x80]),
        SpiT::read_vec(vec![0xFF]),
        SpiT::read_vec(data.to_vec()),
        SpiT::transaction_end(),
    ]
}

#[test]
fn scale_factors_match_datasheet() {
    // Sensitivity 1024/512/256/128 LSB/g, 12-bit two's complement.
    assert_eq!(raw_to_mg(1024, Range::G2), 1000);
    assert_eq!(raw_to_mg(-128, Range::G16), -1000);
    let v = decode(&[0x00, 0x04, 0xFF, 0x0F, 0x00, 0x0C], Range::G2);
    assert_eq!(
        v,
        Vector3 {
            x: 1000,
            y: 0,
            z: -1000
        }
    );
    // Wake-up threshold LSB = 2^(2+range)/256 g; generic 8 mg/LSB.
    assert_eq!(wake_up_threshold_reg(125, Range::G2), 8);
    assert_eq!(wake_up_threshold_reg(250, Range::G4), 8);
    assert_eq!(wake_up_threshold_reg(5000, Range::G2), 255);
    assert_eq!(generic_threshold_reg(128), 0x10);
    // Timeouts 2.5 ms/LSB, 12-bit.
    assert_eq!(timeout_reg(1000), 400);
    assert_eq!(timeout_reg(20_000), 0xFFF);
    // t_w_up = 2/ODR.
    assert_eq!(turn_on_us(PowerMode::LowPower, DataRate::Hz800), 80_000);
    assert_eq!(turn_on_us(PowerMode::Normal, DataRate::Hz100), 20_000);
}

#[test]
fn probe_checks_chip_id_over_i2c() {
    let exp = [r(0x00, &[0x90]), r(0x00, &[0x00])];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Bma400::new_i2c(&mut i2c, ADDR);
    assert_eq!(acc.probe(), Ok(()));
    assert_eq!(acc.probe(), Err(Error::UnexpectedId(0)));
    i2c.done();
}

#[test]
fn probe_over_spi_does_dummy_read_first() {
    // Quick start example 1: the first SPI access switches the interface
    // and is ignored; every read returns a dummy byte first.
    let exp: Vec<_> = [spi_r(0x00, &[0x00]), spi_r(0x00, &[0x90])].concat();
    let mut spi = SpiMock::new(&exp);
    let mut acc = Bma400::new_spi(&mut spi);
    assert_eq!(acc.probe(), Ok(()));
    spi.done();
}

#[test]
fn quick_start_normal_mode_100hz() {
    // Example 1: ACC_CONFIG0 = 0x02, ACC_CONFIG1 = 0x38 (2 g, OSR 3, 100 Hz).
    let exp = [w(0x1A, 0x38), w(0x20, 0x00), w(0x24, 0x22), w(0x19, 0x02)];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Bma400::new_i2c(&mut i2c, ADDR);
    let cfg = Config {
        mode: PowerMode::Normal,
        rate: DataRate::Hz100,
        osr: Oversampling::Osr3,
        latch: false,
        ..Config::default()
    };
    assert_eq!(acc.configure(cfg), Ok(20_000));
    i2c.done();
}

#[test]
fn quick_start_gen1_inactivity() {
    // Example 3: inactivity on gen1, 128 mg, 15 samples, 48 mg hysteresis,
    // filt2, reference every time, AND of all axes, INT1.
    let exp = [
        w(0x3F, 0xFA),
        w(0x40, 0x01),
        w(0x41, 0x10),
        w(0x42, 0x00),
        w(0x43, 0x0F),
        w(0x21, 0x04),
        w(0x22, 0x00),
        w(0x1F, 0x04),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Bma400::new_i2c(&mut i2c, ADDR);
    acc.configure_generic(
        Generic::Gen1,
        &GenericConfig {
            criterion: Criterion::Inactivity,
            threshold_mg: 128,
            duration: 15,
            hysteresis: Hysteresis::Mg48,
            reference: GenericReference::EveryTime,
            all_axes: true,
            ..GenericConfig::default()
        },
    )
    .unwrap();
    i2c.done();
}

#[test]
fn low_power_wake_up_with_auto_power() {
    let exp = [
        w(0x1A, 0x06),
        w(0x20, 0x80),
        w(0x24, 0x22),
        w(0x19, 0x01),
        w(0x2F, 0xE9), // xyz, 3 samples, one-time reference
        w(0x30, 0x08), // 125 mg at 2 g
        w(0x21, 0x00),
        w(0x22, 0x01),
        w(0x2C, 0x19), // 1000 ms
        w(0x2D, 0x06),
        w(0x2A, 0x7D), // 5000 ms
        w(0x2B, 0x08),
        r(0x0E, &[0x01, 0x00, 0x00]),
        r(0x03, &[0x04]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Bma400::new_i2c(&mut i2c, ADDR);
    assert_eq!(acc.configure(Config::default()), Ok(80_000));
    let bad = WakeUpConfig {
        samples: 0,
        ..WakeUpConfig::default()
    };
    assert_eq!(acc.configure_wake_up(&bad), Err(Error::Unsupported));
    acc.configure_wake_up(&WakeUpConfig {
        samples: 3,
        pin: IntPin::Int2,
        ..WakeUpConfig::default()
    })
    .unwrap();
    acc.configure_auto_wake_up(&AutoWakeUp {
        on_motion: true,
        timeout_ms: Some(1000),
    })
    .unwrap();
    acc.configure_auto_low_power(&AutoLowPower {
        timeout_ms: Some(5000),
        timeout_reset_by_gen2: true,
        ..AutoLowPower::default()
    })
    .unwrap();
    let src = acc.read_interrupt_source().unwrap();
    assert!(src.wake_up() && !src.gen1());
    assert_eq!(acc.state(), Ok(State::Normal));
    i2c.done();
}

#[test]
fn not_ready_then_data_then_sleep() {
    let exp = [
        w(0x1A, 0x06),
        w(0x20, 0x80),
        w(0x24, 0x22),
        w(0x19, 0x61), // low-power, osr_lp 3
        r(0x03, &[0x02, 0, 0, 0, 0, 0, 0]),
        r(0x03, &[0x82, 0x00, 0x04, 0, 0, 0x00, 0x0C]),
        w(0x19, 0x60), // sleep keeps osr_lp
        w(0x19, 0x61),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Bma400::new_i2c(&mut i2c, ADDR);
    let cfg = Config {
        osr_lp: Oversampling::Osr3,
        ..Config::default()
    };
    assert_eq!(acc.configure(cfg), Ok(80_000));
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
    acc.sleep().unwrap();
    assert_eq!(acc.wake(), Ok(80_000));
    i2c.done();
}

#[test]
fn async_measure_from_sleep_over_spi() {
    use zigbee_sensor::asynch::Sensor as _;

    struct NoDelay;
    impl embedded_hal_async::delay::DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    let exp: Vec<_> = [
        vec![
            SpiT::transaction_start(),
            SpiT::write_vec(vec![0x19, 0x01]),
            SpiT::transaction_end(),
        ],
        spi_r(0x03, &[0x80, 0, 0, 0, 0, 0x00, 0x02]),
    ]
    .concat();
    let mut spi = SpiMock::new(&exp);
    let mut acc = asynch::Bma400::new_spi(&mut spi);
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
