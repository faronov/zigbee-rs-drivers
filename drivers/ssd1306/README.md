# zigbee-ssd1306

Monochrome OLED controllers Solomon **SSD1306** (128x64, 128x32) and Sino
Wealth **SH1106** (1.3" 128x64), over I2C or 4-wire SPI.

| | |
|---|---|
| Bus | I2C at 0x3C / 0x3D (control byte 0x00 / 0x40, data in 32-byte writes), or SPI mode 0 ≤ 10 MHz + D/C (low = command) |
| Addressing | page mode (`0xB0 \| page`, column nibbles); SH1106 column offset 2 |
| Refresh | immediate; `start_refresh()` returns 0 |
| Current | ~10–20 mA lit (content dependent); ~10 µA in `sleep()` (display + charge pump off) |
| Source | SSD1306 datasheet rev 1.1; SH1106 datasheet v2.3 |
| Status | **mock-tested, not run on hardware** |

OLEDs draw milliamps while on, so on battery devices show the screen
briefly and then call `sleep()`.

## API

```ignore
let mut oled = Ssd1306::new(I2cInterface::new(i2c, ADDRESS), Panel::SSD1306_128X64)
    .with_reset(rst)          // optional
    .with_contrast(0x40)
    .with_rotate180(false);
oled.init(&mut delay)?;       // reset, init, clear RAM, display on
oled.flush(&fb)?;             // whole PageFrame
oled.flush_dirty(&mut fb)?;   // only touched pages/columns
oled.flush_area(&fb, area)?;
oled.set_contrast(0x80)?; oled.set_invert(true)?;
oled.sleep()?; oled.wake()?;
```

SPI: `Ssd1306::new(SpiInterface::new(spi_device, dc), panel)`.
`asynch::Ssd1306` has the same API over `AsyncInterface`. Presets are
`Panel::SSD1306_128X64`, `SSD1306_128X32` and `SH1106_128X64`; use
`Panel::new` for other sizes and offsets (e.g. 72x40 at offset 28).

## RAM

`PageFrame` (bit 1 = lit):

| Panel | Bytes |
|---|---|
| 128x64 | 1 024 |
| 128x32 | 512 |

## Wiring

| Module pin | I2C | SPI |
|---|---|---|
| SCL / D0 | SCL | SCK |
| SDA / D1 | SDA | MOSI |
| RES | GPIO (optional; many I2C modules have an RC reset) | GPIO |
| DC | SA0: address select | D/C GPIO |
| CS | n/a | SpiDevice CS |
