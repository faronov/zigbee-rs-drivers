extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cT};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
use std::vec;
use zigbee_sensor::blocking::{LowPower, Sensor};

const A: u8 = ADDRESS_SA0_LOW;

#[test]
fn sensitivity_matches_datasheet_table_2() {
    // 0.976 mg/digit at ±2 g: 1024 digits = 1 g, 12-bit left-justified.
    assert_eq!(raw_to_mg(1024 << 4, Range::G2), 1000);
    // 1.952 mg/digit at ±4 g.
    assert_eq!(raw_to_mg(-512 << 4, Range::G4), -1000);
    // 7.808 mg/digit at ±16 g.
    assert_eq!(raw_to_mg(128 << 4, Range::G16), 1000);
}

#[test]
fn wake_up_threshold_picks_finest_weight() {
    // FS/256 = 7.8 mg at ±2 g.
    assert_eq!(threshold_reg(50, Range::G2), (6, true));
    // 500 mg does not fit in 63 × 7.8 mg: FS/64 = 31.25 mg.
    assert_eq!(threshold_reg(500, Range::G2), (16, false));
    assert_eq!(threshold_reg(u16::MAX, Range::G2), (63, false));
}

#[test]
fn probe_checks_who_am_i_and_enables_auto_increment() {
    let exp = [
        I2cT::write_read(A, vec![0x43], vec![0x45]),
        I2cT::write(A, vec![0x10, 0x17]),
        I2cT::write_read(A, vec![0x43], vec![0x44]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2du12::new_i2c(&mut i2c, A);
    assert_eq!(acc.probe(), Ok(()));
    assert_eq!(acc.probe(), Err(Error::UnexpectedId(0x44)));
    i2c.done();
}

#[test]
fn one_shot_over_interface() {
    let exp = [
        I2cT::write(A, vec![0x10, 0x17]), // IF_ADD_INC | WU_X/Y/Z_EN
        I2cT::write(A, vec![0x13, 0x20]), // BDU
        I2cT::write(A, vec![0x17, 0x00]),
        I2cT::write(A, vec![0x14, 0xF0]), // one-shot via interface, ±2 g
        I2cT::write(A, vec![0x13, 0x22]), // SOC
        I2cT::write_read(A, vec![0x25], vec![0; 9]),
        I2cT::write_read(
            A,
            vec![0x25],
            vec![0x01, 0, 0, 0x00, 0x40, 0x00, 0xE0, 0x00, 0x00],
        ),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2du12::new_i2c(&mut i2c, A);
    assert_eq!(acc.configure(Config::default()), Ok(0));
    assert_eq!(acc.start_measurement(), Ok(ONE_SHOT_WAIT_US));
    assert_eq!(acc.read_measurement(), Err(Error::NotReady));
    let m = acc.read_measurement().unwrap();
    assert_eq!(
        m.acceleration,
        Vector3 {
            x: 1000,
            y: -500,
            z: 0
        }
    );
    i2c.done();
}

#[test]
fn wake_up_with_auto_sleep_on_int2() {
    let exp = [
        I2cT::write(A, vec![0x10, 0x17]),
        I2cT::write(A, vec![0x13, 0x20]),
        I2cT::write(A, vec![0x17, 0x00]),
        I2cT::write(A, vec![0x14, 0x51]), // 12.5 Hz, ±4 g
        I2cT::write(A, vec![0x10, 0x17]),
        I2cT::write(A, vec![0x13, 0xE0]), // INACT_ODR = 1.6 Hz | BDU
        I2cT::write(A, vec![0x1D, 0x43]), // WAKE_DUR = 2, SLEEP_DUR = 3
        I2cT::write(A, vec![0x1C, 0x46]), // SLEEP_ON | 100 mg / 15.6 mg
        I2cT::write(A, vec![0x1F, 0x00]),
        I2cT::write(A, vec![0x20, 0x20]), // INT2_WU
        I2cT::write(A, vec![0x17, 0x63]), // SHORT_EN | THS_W | LIR | IE
        I2cT::write_read(A, vec![0x21], vec![0x0C, 0, 0, 0x42]),
        I2cT::write(A, vec![0x1F, 0x00]),
        I2cT::write(A, vec![0x20, 0x00]),
        I2cT::write(A, vec![0x17, 0x62]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2du12::new_i2c(&mut i2c, A);
    let cfg = Config {
        range: Range::G4,
        rate: DataRate::Hz12_5,
        conversion: Conversion::Continuous,
        ..Config::default()
    };
    assert_eq!(acc.configure(cfg), Ok(160_000));
    acc.configure_wake_up(&WakeUpConfig {
        threshold_mg: 100,
        duration: 2,
        pin: IntPin::Int2,
        latch: true,
        auto_sleep: Some(AutoSleep {
            duration: 3,
            rate: InactiveRate::Hz1_6,
        }),
    })
    .unwrap();
    let src = acc.read_interrupt_source().unwrap();
    assert!(src.wake_up() && src.x_wake_up() && src.any() && !src.sleep_state());
    acc.disable_interrupt(IntPin::Int2).unwrap();
    i2c.done();
}

#[test]
fn orientation_on_int1() {
    let exp = [
        I2cT::write(A, vec![0x18, 0x40]), // 60°
        I2cT::write(A, vec![0x1F, 0x04]), // INT1_6D
        I2cT::write(A, vec![0x20, 0x00]),
        I2cT::write(A, vec![0x17, 0x01]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2du12::new_i2c(&mut i2c, A);
    acc.configure_orientation(&OrientationConfig::default())
        .unwrap();
    i2c.done();
}

#[test]
fn sleep_and_wake() {
    let exp = [
        I2cT::write(A, vec![0x14, 0x00]),
        I2cT::write(A, vec![0x14, 0xF0]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2du12::new_i2c(&mut i2c, A);
    acc.sleep().unwrap();
    assert_eq!(acc.wake(), Ok(0));
    i2c.done();
}

#[test]
fn spi_framing() {
    let exp = [
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0xC3]),
        SpiT::read_vec(vec![0x45]),
        SpiT::transaction_end(),
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0x10, 0x17]),
        SpiT::transaction_end(),
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0xA5]),
        SpiT::read_vec(vec![0x01, 0, 0, 0, 0, 0, 0, 0x00, 0xC0]),
        SpiT::transaction_end(),
    ];
    let mut spi = SpiMock::new(&exp);
    let mut acc = Lis2du12::new_spi(&mut spi);
    acc.probe().unwrap();
    let m = acc.read_measurement().unwrap();
    assert_eq!(m.acceleration.z, -1000);
    spi.done();
}

#[test]
fn async_one_shot() {
    use zigbee_sensor::asynch::Sensor as _;

    struct NoDelay;
    impl embedded_hal_async::delay::DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    let exp = [
        I2cT::write(A, vec![0x14, 0xF0]),
        I2cT::write(A, vec![0x13, 0x22]),
        I2cT::write_read(A, vec![0x25], vec![0x01, 0, 0, 0, 0, 0, 0, 0x00, 0x40]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = asynch::Lis2du12::new_i2c(&mut i2c, A);
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
