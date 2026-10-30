extern crate std;

use super::*;
use zigbee_sensor::blocking::Sensor;

/// Scripted ADC: returns the listed voltages in order, `None` = error.
struct FakeAdc(&'static [Option<u32>]);

impl FakeAdc {
    fn next(&mut self) -> Result<u32, ()> {
        let (first, rest) = self.0.split_first().expect("ADC script exhausted");
        self.0 = rest;
        first.ok_or(())
    }
}

impl Millivolts for FakeAdc {
    type Error = ();
    fn read_mv(&mut self) -> Result<u32, ()> {
        self.next()
    }
}

impl AsyncMillivolts for FakeAdc {
    type Error = ();
    async fn read_mv(&mut self) -> Result<u32, ()> {
        self.next()
    }
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

fn assert_near(got: Option<i32>, want: i32, tol: i32) {
    let got = got.expect("model returned None");
    assert!((got - want).abs() <= tol, "got {got}, want {want}±{tol}");
}

const DIVIDER: Divider = Divider {
    topology: Topology::NtcToGround,
    series_ohm: 10_000,
    supply_mv: 3_300,
};

#[test]
fn ln_fixed_point() {
    assert_eq!(ln_q16(0), None);
    assert_eq!(ln_q16(1), Some(0));
    // ln(x) · 65536 reference values.
    for (x, want) in [
        (2u64, 45_426),
        (1_000, 452_707),
        (10_000, 603_609),
        (33_620_604, 1_135_781),
        (u64::MAX, 2_907_270),
    ] {
        let got = ln_q16(x).unwrap();
        assert!((got - want).abs() <= 2, "ln({x}) = {got}, want {want}");
    }
    // ln(a·b) = ln a + ln b across the range.
    let mut a = 3u64;
    while a < 1 << 31 {
        let sum = ln_q16(a).unwrap() + ln_q16(a + 7).unwrap();
        assert!((ln_q16(a * (a + 7)).unwrap() - sum).abs() <= 3);
        a = a * 5 + 1;
    }
}

#[test]
fn beta_10k_b3950() {
    // R = 10 kΩ · exp(3950 · (1/T − 1/298.15 K)), in mΩ.
    for (r_mohm, t) in [
        (401_859_725u64, -4_000),
        (105_384_690, -2_000),
        (33_620_604, 0),
        (10_000_000, 2_500),
        (3_588_183, 5_000),
        (1_086_671, 8_500),
        (697_520, 10_000),
        (358_834, 12_500),
    ] {
        assert_near(Model::B3950_10K.temperature(r_mohm), t, 1);
    }
    assert_eq!(Model::B3950_10K.temperature(0), None);
}

#[test]
fn steinhart_hart() {
    // A = 1.009249522e-3, B = 2.378405444e-4, C = 2.019202697e-7.
    let model = Model::SteinhartHart {
        a: 1_009_249_522_000,
        b: 237_840_544_400,
        c: 201_920_270,
    };
    for (ohm, t) in [
        (500u64, 12_120),
        (1_000, 9_467),
        (10_000, 2_468),
        (33_000, -370),
        (100_000, -2_658),
        (1_000_000, -6_601),
    ] {
        assert_near(model.temperature(ohm * 1000), t, 1);
    }
}

#[test]
fn lookup_table() {
    static DESC: [(u32, i32); 3] = [(33_621, 0), (10_000, 2_500), (3_588, 5_000)];
    static ASC: [(u32, i32); 3] = [(3_588, 5_000), (10_000, 2_500), (33_621, 0)];
    for table in [&DESC[..], &ASC[..]] {
        let m = Model::Table(table);
        assert_eq!(m.temperature(10_000_000), Some(2_500));
        assert_eq!(m.temperature(33_621_000), Some(0));
        // Halfway between 33 621 Ω and 10 000 Ω.
        assert_eq!(m.temperature(21_810_500), Some(1_250));
        assert_eq!(m.temperature(6_794_000), Some(3_750));
        assert_eq!(m.temperature(3_587_999), None);
        assert_eq!(m.temperature(33_621_001), None);
    }
    assert_eq!(Model::Table(&DESC[..1]).temperature(33_621_000), None);
}

#[test]
fn divider_topologies() {
    let high = Divider {
        topology: Topology::NtcToSupply,
        ..DIVIDER
    };
    assert_eq!(DIVIDER.resistance_mohm(1_650, 3_300), Some(10_000_000));
    assert_eq!(DIVIDER.resistance_mohm(2_543, 3_300), Some(33_593_131));
    assert_eq!(high.resistance_mohm(757, 3_300), Some(33_593_131));
    assert_eq!(DIVIDER.resistance_mohm(0, 3_300), None);
    assert_eq!(DIVIDER.resistance_mohm(3_300, 3_300), None);
    assert_eq!(high.resistance_mohm(0, 3_300), None);
}

#[test]
fn blocking_reads() {
    // 2542/2544 average to 2543 mV → 33 593 Ω → 0.015 °C.
    static SCRIPT: [Option<u32>; 6] = [
        Some(2_542),
        Some(2_544),
        Some(871),
        Some(871),
        Some(3_300),
        None,
    ];
    let mut dev = Ntc::new(FakeAdc(&SCRIPT), DIVIDER, Model::B3950_10K).with_samples(2);
    assert_eq!(dev.start_measurement(), Ok(0));
    assert_eq!(
        dev.read_measurement(),
        Ok(Measurement {
            temperature: 2,
            resistance_mohm: 33_593_131,
        })
    );
    // 871 mV → 3 585.8 Ω → 50.02 °C.
    assert_near(dev.read_measurement().ok().map(|m| m.temperature), 5_002, 1);
    // Open thermistor (node at supply) and ADC failure.
    let mut dev = Ntc::new(dev.release(), DIVIDER, Model::B3950_10K);
    assert_eq!(dev.read_measurement(), Err(Error::InvalidData));
    assert_eq!(dev.read_measurement(), Err(Error::Bus(())));
}

#[test]
fn async_supply_side_with_measured_rail() {
    use zigbee_sensor::asynch::Sensor as _;
    static SCRIPT: [Option<u32>; 2] = [Some(3_085), Some(1_500)];
    let high = Divider {
        topology: Topology::NtcToSupply,
        ..DIVIDER
    };
    let mut dev = asynch::Ntc::new(FakeAdc(&SCRIPT), high, Model::B3950_10K);
    assert_eq!(block_on(dev.start_measurement()), Ok(0));
    // 3085 mV of 3300 → 696.9 Ω → 100.03 °C.
    assert_near(
        block_on(dev.read_measurement()).ok().map(|m| m.temperature),
        10_003,
        1,
    );
    // Rail measured at 3000 mV: 1500 mV is mid-scale → 10 kΩ → 25 °C.
    assert_eq!(
        block_on(dev.measure_with_supply(3_000)),
        Ok(Measurement {
            temperature: 2_500,
            resistance_mohm: 10_000_000,
        })
    );
}
