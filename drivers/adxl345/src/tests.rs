extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cT};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
use std::vec;
use zigbee_sensor::blocking::{LowPower, Sensor};

const A: u8 = ADDRESS_ALT_LOW;

#[test]
fn scale_factors_match_datasheet() {
    // Full resolution: 256 LSB/g in every range.
    assert_eq!(raw_to_mg(256), 1000);
    assert_eq!(raw_to_mg(-4096), -16000);
    // Thresholds: 62.5 mg/LSB, 0 is not allowed.
    assert_eq!(threshold_reg(250), 4);
    assert_eq!(threshold_reg(0), 1);
    assert_eq!(threshold_reg(16_000), 255);
    assert_eq!(threshold_reg(u16::MAX), 255);
    // Turn-on ≈ 11.1 ms at 100 Hz (Table 1 note 7).
    assert_eq!(turn_on_us(DataRate::Hz100), 11_100);
    assert_eq!(DataRate::Hz3200.period_us(), 313);
    assert_eq!(DataRate::Hz0_10.period_us(), 10_240_000);
}

#[test]
fn probe_checks_devid() {
    let exp = [
        I2cT::write_read(A, vec![0x00], vec![0xE5]),
        I2cT::write_read(A, vec![0x00], vec![0xE6]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Adxl345::new_i2c(&mut i2c, A);
    assert_eq!(acc.probe(), Ok(()));
    assert_eq!(acc.probe(), Err(Error::UnexpectedId(0xE6)));
    i2c.done();
}

#[test]
fn measure_keeps_events_seen_in_int_source() {
    let exp = [
        I2cT::write(A, vec![0x31, 0x08]), // FULL_RES, ±2 g
        I2cT::write(A, vec![0x2C, 0x17]), // LOW_POWER, 12.5 Hz
        I2cT::write(A, vec![0x2D, 0x08]), // Measure
        // Activity latched, no new data yet.
        I2cT::write_read(A, vec![0x30], vec![0x10, 0x08, 0, 0, 0, 0, 0, 0]),
        I2cT::write_read(
            A,
            vec![0x30],
            vec![0x80, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0xFF],
        ),
        I2cT::write_read(A, vec![0x2B], vec![0x40]),
        I2cT::write_read(A, vec![0x30], vec![0x00]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Adxl345::new_i2c(&mut i2c, A);
    assert_eq!(acc.configure(Config::default()), Ok(81_100));
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
    let src = acc.read_interrupt_source().unwrap();
    assert!(src.activity() && src.activity_x() && !src.inactivity());
    i2c.done();
}

#[test]
fn activity_inactivity_auto_sleep_on_int2() {
    let exp = [
        I2cT::write(A, vec![0x2E, 0x00]), // disable before reprogramming
        I2cT::write(A, vec![0x24, 0x04]), // 250 mg
        I2cT::write(A, vec![0x25, 0x02]), // 125 mg
        I2cT::write(A, vec![0x26, 0x05]), // 5 s
        I2cT::write(A, vec![0x27, 0xFF]), // AC, all axes, both functions
        I2cT::write(A, vec![0x2F, 0x18]), // activity + inactivity → INT2
        I2cT::write(A, vec![0x2E, 0x18]),
        I2cT::write(A, vec![0x2D, 0x3A]), // Link | AUTO_SLEEP | Measure | 2 Hz
        // Back to plain activity on INT1 while measuring.
        I2cT::write(A, vec![0x2E, 0x00]),
        I2cT::write(A, vec![0x24, 0x04]),
        I2cT::write(A, vec![0x27, 0xF0]),
        I2cT::write(A, vec![0x2F, 0x00]),
        I2cT::write(A, vec![0x2D, 0x00]), // standby when clearing Link
        I2cT::write(A, vec![0x2D, 0x08]),
        I2cT::write(A, vec![0x2E, 0x10]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Adxl345::new_i2c(&mut i2c, A);
    let mut wake = WakeUpConfig {
        pin: IntPin::Int2,
        link: true,
        ..WakeUpConfig::default()
    };
    assert_eq!(acc.configure_wake_up(&wake), Err(Error::Unsupported));
    wake.inactivity = Some(InactivityConfig::default());
    wake.auto_sleep = Some(SleepRate::Hz2);
    acc.configure_wake_up(&wake).unwrap();
    assert_eq!(acc.start_measurement(), Ok(81_100));
    acc.configure_wake_up(&WakeUpConfig::default()).unwrap();
    i2c.done();
}

#[test]
fn double_tap_then_disable() {
    let exp = [
        I2cT::write(A, vec![0x2E, 0x00]),
        I2cT::write(A, vec![0x1D, 0x30]), // 3 g / 62.5 mg
        I2cT::write(A, vec![0x21, 0x10]), // 10 ms / 625 µs
        I2cT::write(A, vec![0x22, 0x28]), // 50 ms / 1.25 ms
        I2cT::write(A, vec![0x23, 0xA0]), // 200 ms / 1.25 ms
        I2cT::write(A, vec![0x2A, 0x07]),
        I2cT::write(A, vec![0x2F, 0x00]),
        I2cT::write(A, vec![0x2E, 0x60]),
        I2cT::write(A, vec![0x2E, 0x00]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = Adxl345::new_i2c(&mut i2c, A);
    acc.configure_tap(&TapConfig {
        double: Some((50, 200)),
        ..TapConfig::default()
    })
    .unwrap();
    acc.disable_interrupt(IntPin::Int1).unwrap();
    i2c.done();
}

#[test]
fn spi_framing_and_standby() {
    let exp = [
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0x80]),
        SpiT::read_vec(vec![0xE5]),
        SpiT::transaction_end(),
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0xF0]), // read | MB | 0x30
        SpiT::read_vec(vec![0x80, 0x0B, 0x80, 0x00, 0, 0, 0, 0]),
        SpiT::transaction_end(),
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0x2D, 0x00]),
        SpiT::transaction_end(),
        SpiT::transaction_start(),
        SpiT::write_vec(vec![0x2D, 0x08]),
        SpiT::transaction_end(),
    ];
    let mut spi = SpiMock::new(&exp);
    let mut acc = Adxl345::new_spi(&mut spi);
    acc.probe().unwrap();
    assert_eq!(acc.read_measurement().unwrap().acceleration.x, 500);
    acc.sleep().unwrap();
    assert_eq!(acc.wake(), Ok(81_100));
    spi.done();
}

#[test]
fn async_measure_from_standby() {
    use zigbee_sensor::asynch::Sensor as _;

    struct NoDelay;
    impl embedded_hal_async::delay::DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    let exp = [
        I2cT::write(A, vec![0x2D, 0x08]),
        I2cT::write_read(A, vec![0x30], vec![0x82, 0x08, 0, 0, 0, 0, 0x00, 0x01]),
    ];
    let mut i2c = I2cMock::new(&exp);
    let mut acc = asynch::Adxl345::new_i2c(&mut i2c, A);
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
