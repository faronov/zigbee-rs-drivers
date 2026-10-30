extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cT};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
use std::vec;
use zigbee_sensor::blocking::{LowPower, Sensor};

const A: u8 = ADDRESS_SA0_LOW;

#[test]
fn sensitivity_matches_datasheet_table_3() {
    // 0.244 mg/digit (14-bit, ±2 g): 4096 digits = 1 g, left-justified by 2.
    assert_eq!(raw_to_mg(4096 << 2, Range::G2, PowerMode::LowPower2), 1000);
    assert_eq!(
        raw_to_mg(4096 << 2, Range::G2, PowerMode::HighPerformance),
        1000
    );
    // 1.952 mg/digit at ±16 g.
    assert_eq!(raw_to_mg(512 << 2, Range::G16, PowerMode::LowPower4), 1000);
    // Low-power mode 1: 12-bit, 0.976 mg/digit at ±2 g.
    assert_eq!(
        raw_to_mg(-1024 << 4, Range::G2, PowerMode::LowPower1),
        -1000
    );
    // 7.808 mg/digit at ±16 g in LP1.
    assert_eq!(raw_to_mg(128 << 4, Range::G16, PowerMode::LowPower1), 1000);
}

#[test]
fn wake_up_threshold_is_full_scale_over_64() {
    assert_eq!(threshold_reg(63, Range::G2), 2);
    assert_eq!(threshold_reg(500, Range::G2), 16);
    assert_eq!(threshold_reg(500, Range::G8), 4);
    assert_eq!(threshold_reg(u16::MAX, Range::G2), 63);
}

#[test]
fn probe_checks_who_am_i() {
    let exp = [
        I2cT::write_read(A, vec![0x0F], vec![0x44]),
        I2cT::write_read(A, vec![0x0F], vec![0x33]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2dw12::new_i2c(&mut i2c, A);
    assert_eq!(acc.probe(), Ok(()));
    assert_eq!(acc.probe(), Err(Error::UnexpectedId(0x33)));
    i2c.done();
}

#[test]
fn single_conversion_on_demand() {
    let exp = [
        I2cT::write(A, vec![0x21, 0x0C]), // BDU | IF_ADD_INC
        I2cT::write(A, vec![0x25, 0x10]), // ±4 g
        I2cT::write(A, vec![0x22, 0x02]), // SLP_MODE_SEL
        I2cT::write(A, vec![0x20, 0x2A]), // 12.5 Hz, MODE=10, LP3
        I2cT::write(A, vec![0x22, 0x03]), // SLP_MODE_1: trigger
        I2cT::write_read(A, vec![0x27], vec![0, 0, 0, 0, 0, 0, 0]),
        // ±4 g 14-bit: 0.488 mg/digit; x = 2048 digits, z = -2048.
        I2cT::write_read(A, vec![0x27], vec![0x01, 0x00, 0x20, 0, 0, 0x00, 0xE0]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2dw12::new_i2c(&mut i2c, A);
    let cfg = Config {
        range: Range::G4,
        mode: PowerMode::LowPower3,
        ..Config::default()
    };
    assert_eq!(acc.configure(cfg), Ok(0));
    assert_eq!(acc.start_measurement(), Ok(2_300));
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
    i2c.done();
}

#[test]
fn on_demand_rejects_high_performance() {
    let mut i2c = I2cMock::new(&[]);
    let mut acc = Lis2dw12::new_i2c(&mut i2c, A);
    let cfg = Config {
        mode: PowerMode::HighPerformance,
        ..Config::default()
    };
    assert_eq!(acc.configure(cfg), Err(Error::Unsupported));
    i2c.done();
}

#[test]
fn continuous_wake_up_on_int1() {
    let exp = [
        I2cT::write(A, vec![0x21, 0x0C]),
        I2cT::write(A, vec![0x25, 0x00]),
        I2cT::write(A, vec![0x22, 0x00]),
        I2cT::write(A, vec![0x20, 0x10]), // 1.6 Hz, LP1, continuous
        I2cT::write(A, vec![0x35, 0x20]), // WAKE_DUR = 1
        I2cT::write(A, vec![0x34, 0x03]), // 94 mg / 31.25 mg
        I2cT::write(A, vec![0x22, 0x10]), // LIR
        I2cT::write(A, vec![0x23, 0x20]), // INT1_WU
        I2cT::write(A, vec![0x24, 0x00]),
        I2cT::write(A, vec![0x3F, 0x20]), // INTERRUPTS_ENABLE
        I2cT::write_read(A, vec![0x38], vec![0x0A, 0x00, 0x00, 0x02]),
        I2cT::write(A, vec![0x23, 0x00]),
        I2cT::write(A, vec![0x24, 0x00]),
        I2cT::write(A, vec![0x3F, 0x00]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2dw12::new_i2c(&mut i2c, A);
    let cfg = Config {
        rate: DataRate::Hz1_6,
        conversion: Conversion::Continuous,
        ..Config::default()
    };
    assert_eq!(acc.configure(cfg), Ok(626_200));
    acc.configure_wake_up(&WakeUpConfig {
        threshold_mg: 94,
        duration: 1,
        ..WakeUpConfig::default()
    })
    .unwrap();
    let src = acc.read_interrupt_source().unwrap();
    assert!(src.wake_up() && src.y_wake_up() && !src.x_wake_up());
    acc.disable_interrupt(IntPin::Int1).unwrap();
    i2c.done();
}

#[test]
fn activity_inactivity_on_int2() {
    let exp = [
        I2cT::write(A, vec![0x35, 0x02]), // SLEEP_DUR = 2
        I2cT::write(A, vec![0x34, 0x42]), // SLEEP_ON | 2
        I2cT::write(A, vec![0x22, 0x02]), // pulsed, on-demand trigger kept
        I2cT::write(A, vec![0x23, 0x00]),
        I2cT::write(A, vec![0x24, 0x40]), // INT2_SLEEP_CHG
        I2cT::write(A, vec![0x3F, 0x20]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2dw12::new_i2c(&mut i2c, A);
    let mut wake = WakeUpConfig {
        pin: IntPin::Int2,
        latch: false,
        ..WakeUpConfig::default()
    };
    assert_eq!(acc.configure_wake_up(&wake), Err(Error::Unsupported));
    wake.auto_sleep = Some(2);
    acc.configure_wake_up(&wake).unwrap();
    i2c.done();
}

#[test]
fn orientation_6d_on_int1() {
    let exp = [
        I2cT::write(A, vec![0x30, 0xE0]), // 4D_EN | 50°
        I2cT::write(A, vec![0x23, 0x80]), // INT1_6D
        I2cT::write(A, vec![0x24, 0x00]),
        I2cT::write(A, vec![0x3F, 0x20]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2dw12::new_i2c(&mut i2c, A);
    acc.configure_orientation(&OrientationConfig {
        threshold: OrientationThreshold::Deg50,
        four_d: true,
    })
    .unwrap();
    i2c.done();
}

#[test]
fn sleep_then_start_powers_up_and_triggers() {
    let exp = [
        I2cT::write(A, vec![0x20, 0x00]),
        I2cT::write(A, vec![0x20, 0x28]), // restore on-demand CTRL1
        I2cT::write(A, vec![0x22, 0x03]), // SLP_MODE_SEL | SLP_MODE_1
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2dw12::new_i2c(&mut i2c, A);
    acc.sleep().unwrap();
    assert_eq!(acc.start_measurement(), Ok(1_200));
    i2c.done();
}

#[test]
fn spi_framing() {
    let exp = [
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0x8F]),
        SpiT::read_vec(vec![0x44]),
        SpiT::transaction_end(),
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0x20, 0x00]),
        SpiT::transaction_end(),
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0xA7]),
        SpiT::read_vec(vec![0x01, 0, 0x10, 0, 0, 0, 0]),
        SpiT::transaction_end(),
    ];
    let mut spi = SpiMock::new(&exp);
    let mut acc = Lis2dw12::new_spi(&mut spi);
    acc.probe().unwrap();
    acc.sleep().unwrap();
    // LP1 ±2 g: 256 digits × 0.976 mg.
    let m = acc.read_measurement().unwrap();
    assert_eq!(m.acceleration.x, 250);
    spi.done();
}

#[test]
fn async_measure_on_demand() {
    use zigbee_sensor::asynch::Sensor as _;

    struct NoDelay;
    impl embedded_hal_async::delay::DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    let exp = [
        I2cT::write(A, vec![0x20, 0x28]),
        I2cT::write(A, vec![0x22, 0x03]),
        I2cT::write_read(A, vec![0x27], vec![0x01, 0, 0, 0, 0, 0, 0x40]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = asynch::Lis2dw12::new_i2c(&mut i2c, A);
    let m = block_on(acc.measure(&mut NoDelay)).unwrap();
    assert_eq!(m.acceleration.z, 1000);
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
