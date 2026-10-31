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

## Displays

Display drivers follow the same rule: nothing blocks the stack while an
e-paper panel refreshes for seconds.

```rust,ignore
let mut buf = [0u8; zigbee_display::mono_len(122, 250)];   // 4 000 B, yours
let mut fb = MonoFrame::new(&mut buf[..], 122, 250).unwrap();
Text::new("21.5 °C", Point::new(4, 20), style).draw(&mut fb)?; // embedded-graphics
let wait_us = epd.update(&fb)?;           // write RAM + start refresh
// ... schedule a timer for wait_us; later `epd.is_busy()?`, then `epd.sleep()?`
```

Frame buffers are caller-provided slices (or arrays), so RAM use is
explicit. `drivers/display` (`zigbee-display`) holds the shared traits,
the SPI+DC / I2C interface and the embedded-graphics frame buffers. Only
display crates depend on it: sensor-only firmware does not pull in
embedded-graphics. Supported: Sharp Memory LCD (incl. the SiLabs WSTK
LS013B7DH03), SSD1306/SH1106 OLED, SSD1680/SSD1681/SSD1675, UC8151/IL0373
and IL3820/IL3895 e-paper (full and partial refresh), and ST7789/ST7735
TFT (mains-powered devices only). All are **mock-tested, not run on
hardware** yet.

* [`core`](core): `zigbee-sensor` — traits, units (`Quantity`), ZCL
  conversions, CRC helpers, ADC trait.
* [`drivers`](drivers): one crate per part family.
* [Catalogue](docs/catalog.md) — supported and planned sensors and displays.
* [Driver guide](docs/driver-guide.md) — rules for writing a driver.

## License

Licensed under either of [Apache License 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option. Third-party material is listed
in [NOTICE](NOTICE).
