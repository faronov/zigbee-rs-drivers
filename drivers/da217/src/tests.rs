extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cT};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
use std::vec;
use std::vec::Vec;
use zigbee_sensor::blocking::{LowPower, Sensor};

const ADDR: u8 = ADDRESS_SA0_LOW;

fn w(reg: u8, value: u8) -> I2cT {
    I2cT::write(ADDR, vec![reg, value])
}

fn r(reg: u8, data: &[u8]) -> I2cT {
    I2cT::write_read(ADDR, vec![reg], data.to_vec())
}

// One single-byte SPI read per register.
fn spi_r(reg: u8, data: &[u8]) -> Vec<SpiT<u8>> {
    let mut v = Vec::new();
    for (i, b) in data.iter().enumerate() {
        v.extend([
            SpiT::transaction_start(),
            SpiT::write_vec(vec![0x80, reg + i as u8]),
            SpiT::read_vec(vec![*b]),
            SpiT::transaction_end(),
        ]);
    }
    v
}

fn spi_w(reg: u8, value: u8) -> Vec<SpiT<u8>> {
    vec![
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0x00, reg, value]),
        SpiT::transaction_end(),
    ]
}

#[test]
fn scaling_and_register_codes() {
    // Table 2: 4096 LSB/g at ±2 g, 14-bit left-justified.
    assert_eq!(raw_to_mg(0x4000, 14, Range::G2), 1000);
    assert_eq!(raw_to_mg(0x40FC, 14, Range::G2), 1015);
    assert_eq!(raw_to_mg(0x40FC, 8, Range::G2), 1000);
    assert_eq!(raw_to_mg(i16::MIN, 14, Range::G16), -16000);
    let v = decode(&[0x00, 0x40, 0x00, 0x00, 0x00, 0xC0], 14, Range::G2);
    assert_eq!(
        v,
        Vector3 {
            x: 1000,
            y: 0,
            z: -1000
        }
    );
    // Table 77: K = 3.91 mg at ±2 g, 31.25 mg at ±16 g.
    assert_eq!(active_threshold_reg(100, Range::G2), 26);
    assert_eq!(active_threshold_reg(100, Range::G16), 3);
    assert_eq!(active_threshold_reg(2000, Range::G2), 255);
    assert_eq!(DataRate::Hz1.period_us(), 1_000_000);
    assert_eq!(DataRate::Hz1_95.period_us(), 512_000);
    assert_eq!(DataRate::Hz31_25.period_us(), 32_000);
    assert_eq!(DataRate::Hz500.period_us(), 2_000);
    assert_eq!(Resolution::Bits10.bits(), 10);
}

#[test]
fn probe_i2c_checks_chip_id() {
    let exp = [r(0x01, &[0x13]), r(0x01, &[0x12])];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Da217::new_i2c(&mut i2c, ADDR);
    acc.probe().unwrap();
    assert!(matches!(acc.probe(), Err(Error::UnexpectedId(0x12))));
    i2c.done();
}

#[test]
fn spi_probe_enables_4_wire_and_reads_per_byte() {
    let mut exp = spi_w(0x00, 0x81);
    exp.extend(spi_r(0x01, &[0x13]));
    exp.extend(spi_r(0x02, &[0x00, 0x40, 0x00, 0x00, 0x00, 0xC0]));
    let mut spi = SpiMock::new(&exp);
    let mut acc = Da217::new_spi(&mut spi);
    acc.probe().unwrap();
    let m = acc.read_measurement().unwrap();
    assert_eq!(
        m.acceleration,
        Vector3 {
            x: 1000,
            y: 0,
            z: -1000
        }
    );
    spi.done();
}

#[test]
fn configure_writes_registers() {
    let exp = [w(0x0F, 0x45), w(0x10, 0x04), w(0x17, 0x00), w(0x11, 0x1D)];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Da217::new_i2c(&mut i2c, ADDR);
    let wait = acc
        .configure(Config {
            range: Range::G4,
            resolution: Resolution::Bits12,
            rate: DataRate::Hz15_63,
            bandwidth: Bandwidth::Hz100,
            auto_sleep: true,
            data_ready: false,
        })
        .unwrap();
    assert_eq!(wait, 65_000);
    assert_eq!(acc.start_measurement().unwrap(), 0);
    i2c.done();
}

#[test]
fn motion_interrupt_source_and_sleep() {
    let exp = [
        w(0x28, 26),
        w(0x27, 0x42),
        w(0x21, 0x77),
        w(0x20, 0x0F),
        w(0x19, 0x00),
        w(0x1B, 0x04),
        w(0x16, 0x07),
        r(0x09, &[0x04, 0x00, 0x0C]),
        w(0x20, 0x8F),
        w(0x11, 0x98),
        w(0x11, 0x18),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Da217::new_i2c(&mut i2c, ADDR);
    acc.configure_motion(&MotionConfig {
        threshold_mg: 100,
        duration: 3,
        inactive_duration: 5,
        pin: IntPin::Int2,
        active_low: true,
        open_drain: true,
        ..Default::default()
    })
    .unwrap();
    let src = acc.read_interrupt_source().unwrap();
    assert!(src.active());
    assert!(!src.data_ready());
    assert_eq!(src.active_axes(), Axes::X);
    assert!(src.active_negative());
    acc.sleep().unwrap();
    assert_eq!(acc.wake().unwrap(), 33_000);
    i2c.done();
}

#[test]
fn motion_validation_and_disable() {
    let exp = [w(0x16, 0x00), w(0x19, 0x00), w(0x1B, 0x00)];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Da217::new_i2c(&mut i2c, ADDR);
    for bad in [
        MotionConfig {
            duration: 0,
            ..Default::default()
        },
        MotionConfig {
            inactive_duration: 17,
            ..Default::default()
        },
        MotionConfig {
            axes: Axes(0),
            ..Default::default()
        },
    ] {
        assert!(matches!(
            acc.configure_motion(&bad),
            Err(Error::Unsupported)
        ));
    }
    acc.disable_motion().unwrap();
    i2c.done();
}

#[test]
fn data_ready_gates_reads() {
    let exp = [
        w(0x0F, 0x40),
        w(0x10, 0x05),
        w(0x17, 0x10),
        w(0x11, 0x18),
        r(0x0A, &[0x00]),
        r(0x0A, &[0x01]),
        r(0x02, &[0x00, 0x00, 0x00, 0xC0, 0x00, 0x40]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Da217::new_i2c(&mut i2c, ADDR);
    acc.configure(Config {
        data_ready: true,
        ..Config::DEFAULT
    })
    .unwrap();
    assert!(matches!(acc.read_measurement(), Err(Error::NotReady)));
    let m = acc.read_measurement().unwrap();
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

#[test]
fn async_measure() {
    use zigbee_sensor::asynch::Sensor as _;

    struct NoDelay;
    impl embedded_hal_async::delay::DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    let exp = [
        w(0x11, 0x18),
        r(0x02, &[0x00, 0x40, 0x00, 0x00, 0x00, 0x00]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = asynch::Da217::new_i2c(&mut i2c, ADDR);
    let m = block_on(acc.measure(&mut NoDelay)).unwrap();
    assert_eq!(
        m.acceleration,
        Vector3 {
            x: 1000,
            y: 0,
            z: 0
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
