extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cT};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
use std::vec;
use zigbee_sensor::blocking::{LowPower, Sensor};

const A: u8 = ADDRESS_SA0_LOW;

#[test]
fn sensitivity_matches_datasheet_tables() {
    // LIS2DH12 Table 4: 1 mg/digit HR ±2 g; 1 g = 1000 digits left-justified.
    assert_eq!(
        raw_to_mg(1000 << 4, Variant::St, Range::G2, Mode::HighResolution),
        1000
    );
    // Normal ±4 g: 8 mg/digit (10-bit).
    assert_eq!(
        raw_to_mg(125 << 6, Variant::St, Range::G4, Mode::Normal),
        1000
    );
    // Low-power ±16 g: 192 mg/digit (8-bit), negative value.
    assert_eq!(
        raw_to_mg(-5 << 8, Variant::St, Range::G16, Mode::LowPower),
        -960
    );
    // ST ±16 g HR: 12 mg/digit; SC7A20: 8 mg/digit.
    assert_eq!(
        raw_to_mg(100 << 4, Variant::St, Range::G16, Mode::HighResolution),
        1200
    );
    assert_eq!(
        raw_to_mg(125 << 4, Variant::Sc7a20, Range::G16, Mode::HighResolution),
        1000
    );
}

#[test]
fn threshold_steps_per_variant() {
    assert_eq!(threshold_reg(64, Variant::St, Range::G2), 4);
    assert_eq!(threshold_reg(1000, Variant::St, Range::G16), 5);
    assert_eq!(threshold_reg(1000, Variant::Sc7a20, Range::G16), 8);
    assert_eq!(threshold_reg(124, Variant::St, Range::G8), 2);
    assert_eq!(threshold_reg(u16::MAX, Variant::St, Range::G2), 127);
}

#[test]
fn probe_detects_variants_and_rejects_unknown_id() {
    let exp = [
        I2cT::write_read(A, vec![0x0F], vec![0x33]),
        I2cT::write_read(A, vec![0x0F], vec![0x11]),
        I2cT::write_read(A, vec![0x0F], vec![0x44]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2dh::new_i2c(&mut i2c, A);
    assert_eq!(acc.probe(), Ok(Variant::St));
    assert_eq!(acc.probe(), Ok(Variant::Sc7a20));
    assert_eq!(acc.probe(), Err(Error::UnexpectedId(0x44)));
    i2c.done();
}

#[test]
fn configure_then_read_with_data_ready() {
    let exp = [
        // CTRL4: BDU | FS=±4 g | HR, CTRL6, REFERENCE read, CTRL1: 100 Hz XYZ.
        I2cT::write(A, vec![0x23, 0x98]),
        I2cT::write(A, vec![0x25, 0x00]),
        I2cT::write_read(A, vec![0x26], vec![0x00]),
        I2cT::write(A, vec![0x20, 0x57]),
        // Not ready yet.
        I2cT::write_read(A, vec![0xA7], vec![0, 0, 0, 0, 0, 0, 0]),
        // ZYXDA: x = +500 digits, y = -250, z = 0.
        I2cT::write_read(
            A,
            vec![0xA7],
            vec![0x0F, 0x40, 0x1F, 0x60, 0xF0, 0x00, 0x00],
        ),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2dh::new_i2c(&mut i2c, A);
    let cfg = Config {
        range: Range::G4,
        mode: Mode::HighResolution,
        rate: DataRate::Hz100,
        int_active_low: false,
    };
    assert_eq!(acc.configure(cfg), Ok(70_000));
    assert_eq!(acc.start_measurement(), Ok(0));
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
fn rejects_1620_hz_outside_low_power() {
    let mut i2c = I2cMock::new(&[]);
    let mut acc = Lis2dh::new_i2c(&mut i2c, A);
    let cfg = Config {
        mode: Mode::Normal,
        rate: DataRate::Hz1620,
        ..Config::default()
    };
    assert_eq!(acc.configure(cfg), Err(Error::Unsupported));
    i2c.done();
}

#[test]
fn wake_up_on_int1_latched_with_high_pass() {
    let exp = [
        I2cT::write(A, vec![0x21, 0x01]), // HP_IA1
        I2cT::write(A, vec![0x32, 0x05]), // 80 mg / 16 mg
        I2cT::write(A, vec![0x33, 0x02]),
        I2cT::write(A, vec![0x24, 0x08]),            // LIR_INT1
        I2cT::write_read(A, vec![0x26], vec![0x00]), // reset HP filter
        I2cT::write(A, vec![0x30, 0x2A]),            // XH | YH | ZH, OR
        I2cT::write(A, vec![0x22, 0x40]),            // I1_IA1
        I2cT::write_read(A, vec![0x31], vec![0x62]),
        // Disable.
        I2cT::write(A, vec![0x30, 0x00]),
        I2cT::write(A, vec![0x22, 0x00]),
        I2cT::write(A, vec![0x21, 0x00]),
        I2cT::write(A, vec![0x24, 0x00]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2dh::new_i2c(&mut i2c, A);
    acc.configure_wake_up(&WakeUpConfig {
        threshold_mg: 80,
        duration: 2,
        ..WakeUpConfig::default()
    })
    .unwrap();
    let src = acc.read_interrupt_source(IntPin::Int1).unwrap();
    assert!(src.active() && src.z_high() && src.x_high() && !src.y_high());
    acc.disable_interrupt(IntPin::Int1).unwrap();
    i2c.done();
}

#[test]
fn orientation_on_int2_uses_variant_routing_bit() {
    let exp = [
        I2cT::write(A, vec![0x21, 0x00]),
        I2cT::write(A, vec![0x36, 0x25]), // 590 mg / 16 mg -> 37
        I2cT::write(A, vec![0x37, 0x00]),
        I2cT::write(A, vec![0x24, 0x02]), // LIR_INT2
        I2cT::write(A, vec![0x34, 0x7F]), // 6D movement, all axes
        I2cT::write(A, vec![0x25, 0x40]), // SC7A20: AOI2 on INT2 is bit 6
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2dh::new_i2c(&mut i2c, A).with_variant(Variant::Sc7a20);
    acc.configure_orientation(&OrientationConfig {
        threshold_mg: 590,
        pin: IntPin::Int2,
        ..OrientationConfig::default()
    })
    .unwrap();
    i2c.done();
}

#[test]
fn sleep_and_start_measurement_powers_up_again() {
    let exp = [
        I2cT::write(A, vec![0x20, 0x07]),
        I2cT::write(A, vec![0x20, 0x2F]), // 10 Hz, LPen, XYZ
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Lis2dh::new_i2c(&mut i2c, A);
    acc.sleep().unwrap();
    assert_eq!(acc.start_measurement(), Ok(101_000));
    assert_eq!(acc.start_measurement(), Ok(0));
    i2c.done();
}

#[test]
fn spi_framing() {
    let exp = [
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0x8F]),
        SpiT::read_vec(vec![0x33]),
        SpiT::transaction_end(),
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0x20, 0x07]),
        SpiT::transaction_end(),
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0xE7]),
        SpiT::read_vec(vec![0x08, 0, 0x40, 0, 0, 0, 0xC0]),
        SpiT::transaction_end(),
    ];
    let mut spi = SpiMock::new(&exp);
    let mut acc = Lis2dh::new_spi(&mut spi);
    assert_eq!(acc.probe(), Ok(Variant::St));
    acc.sleep().unwrap();
    // Low-power ±2 g: 16 mg/digit.
    let m = acc.read_measurement().unwrap();
    assert_eq!(
        m.acceleration,
        Vector3 {
            x: 1024,
            y: 0,
            z: -1024
        }
    );
    spi.done();
}

#[test]
fn async_measure_over_i2c() {
    use zigbee_sensor::asynch::Sensor as _;

    struct NoDelay;
    impl embedded_hal_async::delay::DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    let exp = [
        I2cT::write(A, vec![0x20, 0x2F]),
        I2cT::write_read(A, vec![0xA7], vec![0x08, 0, 0, 0, 0, 0, 0x40]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = asynch::Lis2dh::new_i2c(&mut i2c, A);
    let m = block_on(acc.measure(&mut NoDelay)).unwrap();
    assert_eq!(m.acceleration.z, 1024);
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
