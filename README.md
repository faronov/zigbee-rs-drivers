# zigbee-rs-drivers

Sensor drivers for [zigbee-rs](https://github.com/faronov/zigbee-rs)
firmware. Every driver is `no_std`, has a blocking (embedded-hal 1.0) and an
async (embedded-hal-async 1.0) API, and runs on every zigbee-rs platform
(nRF52, EFR32, ESP32-C6/H2, TLSR, BL702, PHY62xx, CC2340, ...).

Drivers never block the Zigbee stack while a sensor converts:

```rust,ignore
use zigbee_sensor::blocking::Sensor;

let mut sht = zigbee_sht3x::Sht3x::new(i2c, zigbee_sht3x::PRIMARY_ADDRESS);
let wait_us = sht.start_measurement()?;   // trigger, returns 15_500
// ... the stack keeps running / the device sleeps for wait_us ...
let m = sht.read_measurement()?;          // 0.01 °C, 0.01 %RH
let zcl_temp = zigbee_sensor::zcl::temperature(m.temperature);
```

or, in an async executor, `sht.measure(&mut delay).await`.

* [`core`](core): `zigbee-sensor` — traits, units (`Quantity`), ZCL
  conversions, CRC helpers, ADC trait.
* [`drivers`](drivers): one crate per part family.
* [Sensor catalogue](docs/catalog.md) — supported and planned parts.
* [Driver guide](docs/driver-guide.md) — rules for writing a driver.

## License

Licensed under either of [Apache License 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option. Third-party material is listed
in [NOTICE](NOTICE).
