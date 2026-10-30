extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cT};
use std::vec;
use zigbee_sensor::blocking::{LowPower, Sensor};

const ADDR: u8 = ADDRESS_ADDR_LOW;

fn w(reg: u8, value: u8) -> I2cT {
    I2cT::write(ADDR, vec![reg, value])
}

fn r(reg: u8, data: &[u8]) -> I2cT {
    I2cT::write_read(ADDR, vec![reg], data.to_vec())
}

#[test]
fn scaling_and_register_codes() {
    // Table 1 sensitivities: 8-bit 64, 12-bit 1024, 14-bit (±8 g) 1024 counts/g.
    assert_eq!(raw_to_mg(0x40FF, 8, Range::G2), 1000);
    assert_eq!(raw_to_mg(0x40F0, 12, Range::G2), 1014);
    assert_eq!(raw_to_mg(-16384, 12, Range::G2), -1000);
    assert_eq!(raw_to_mg(0x1003, 14, Range::G8), 1000);
    assert_eq!(raw_to_mg(i16::MIN, 14, Range::G16), -16000);
    let v = decode(&[0x00, 0x40, 0x00, 0x00, 0x00, 0xC0], 12, Range::G2);
    assert_eq!(
        v,
        Vector3 {
            x: 1000,
            y: 0,
            z: -1000
        }
    );
    // Factory default 0x080 = 0.5 g; 3.9 mg/count.
    assert_eq!(wake_up_threshold_reg(500), 0x080);
    assert_eq!(wake_up_threshold_reg(100), 26);
    assert_eq!(wake_up_threshold_reg(20_000), 0xFFF);
    let hr8 = Config {
        mode: PowerMode::HighResolution,
        range: Range::G8,
        ..Config::DEFAULT
    };
    assert_eq!(hr8.resolution_bits(), 14);
    assert_eq!(Config::DEFAULT.resolution_bits(), 8);
    let fast = Config {
        rate: DataRate::Hz800,
        ..Config::DEFAULT
    };
    assert_eq!(fast.resolution_bits(), 12);
}

#[test]
fn probe_checks_who_am_i() {
    let exp = [r(0x0F, &[0x35]), r(0x0F, &[0x33])];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Kxtj3::new(&mut i2c, ADDR);
    acc.probe().unwrap();
    assert!(matches!(acc.probe(), Err(Error::UnexpectedId(0x33))));
    i2c.done();
}

#[test]
fn configure_writes_in_standby_then_operates() {
    let exp = [
        w(0x1B, 0x58),
        w(0x21, 0x03),
        w(0x1B, 0xD8),
        // ODR ≥ 400 Hz selects RES; ±16 g 14-bit code.
        w(0x1B, 0x5C),
        w(0x21, 0x06),
        w(0x1B, 0xDC),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Kxtj3::new(&mut i2c, ADDR);
    let wait = acc
        .configure(Config {
            mode: PowerMode::HighResolution,
            range: Range::G8,
            rate: DataRate::Hz100,
            data_ready: false,
        })
        .unwrap();
    assert_eq!(wait, 11_000);
    acc.configure(Config {
        range: Range::G16,
        rate: DataRate::Hz800,
        ..Config::DEFAULT
    })
    .unwrap();
    i2c.done();
}

#[test]
fn wake_up_while_active_then_read_source_and_sleep() {
    let exp = [
        w(0x1B, 0x00),
        w(0x21, 0x00),
        w(0x1B, 0x80),
        // Stand-by, wake-up registers, back to operating with WUFE.
        w(0x1B, 0x00),
        w(0x1D, 0x04),
        w(0x29, 3),
        w(0x2A, 2),
        w(0x6A, 0x01),
        w(0x6B, 0xA0),
        w(0x1F, 0x3F),
        w(0x1E, 0x30),
        w(0x1B, 0x82),
        r(0x16, &[0x02, 0x21]),
        r(0x1A, &[0x00]),
        w(0x1B, 0x02),
        w(0x1B, 0x82),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Kxtj3::new(&mut i2c, ADDR);
    acc.configure(Config::DEFAULT).unwrap();
    acc.configure_wake_up(&WakeUpConfig {
        threshold_mg: 100,
        rate: WakeUpRate::Hz12_5,
        duration: 3,
        non_activity: 2,
        ..Default::default()
    })
    .unwrap();
    let src = acc.read_interrupt_source().unwrap();
    assert!(src.wake_up());
    assert!(!src.data_ready());
    assert!(src.directions().contains(Directions::X_NEG));
    assert!(src.directions().contains(Directions::Z_POS));
    assert!(!src.directions().contains(Directions::Y_POS));
    acc.sleep().unwrap();
    assert_eq!(acc.wake().unwrap(), 81_000);
    i2c.done();
}

#[test]
fn wake_up_modes_and_invalid_counters() {
    let exp = [
        w(0x1D, 0x00),
        w(0x29, 1),
        w(0x2A, 1),
        w(0x6A, 0x08),
        w(0x6B, 0x00),
        w(0x1F, 0x80 | 0x03),
        w(0x1E, 0x20),
        w(0x1B, 0x02),
        w(0x1E, 0x10),
        w(0x1B, 0x00),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Kxtj3::new(&mut i2c, ADDR);
    let bad = WakeUpConfig {
        duration: 0,
        ..Default::default()
    };
    assert!(matches!(
        acc.configure_wake_up(&bad),
        Err(Error::Unsupported)
    ));
    acc.configure_wake_up(&WakeUpConfig {
        rate: WakeUpRate::Hz0_781,
        directions: Directions::Z_NEG.or(Directions::Z_POS),
        mode: IntMode::Unlatched,
        active_high: false,
        ..Default::default()
    })
    .unwrap();
    acc.disable_wake_up().unwrap();
    i2c.done();
}

#[test]
fn data_ready_gates_reads() {
    let exp = [
        w(0x1B, 0x20),
        w(0x21, 0x00),
        w(0x1B, 0xA0),
        r(0x16, &[0x00]),
        r(0x16, &[0x10]),
        r(0x06, &[0x00, 0x40, 0x00, 0x00, 0x00, 0xC0]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Kxtj3::new(&mut i2c, ADDR);
    acc.configure(Config {
        data_ready: true,
        ..Config::DEFAULT
    })
    .unwrap();
    assert_eq!(acc.start_measurement().unwrap(), 0);
    assert!(matches!(acc.read_measurement(), Err(Error::NotReady)));
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
fn async_measure() {
    use zigbee_sensor::asynch::Sensor as _;

    struct NoDelay;
    impl embedded_hal_async::delay::DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    let exp = [
        w(0x1B, 0x80),
        r(0x06, &[0x00, 0x00, 0x00, 0xC0, 0x00, 0x40]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = asynch::Kxtj3::new(&mut i2c, ADDR);
    let m = block_on(acc.measure(&mut NoDelay)).unwrap();
    assert_eq!(
        m.acceleration,
        Vector3 {
            x: 0,
            y: -1000,
            z: 1000
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
