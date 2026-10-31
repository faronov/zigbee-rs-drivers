# zigbee-sharp-memlcd

Sharp Memory LCDs (memory-in-pixel, reflective, ~µW static): LS013B7DH03
128x128 (Silicon Labs WSTK BRD4001A main board), LS013B7DH05 144x168,
LS011B7DH03 160x68, LS027B7DH01(A) 400x240, or any `Panel::new(w, h)`
with 8-bit gate addressing.

| | |
|---|---|
| Bus | write-only SPI, mode 0, MSB first, ≤ 1.1 MHz (LS013B7DH03) / 2 MHz (LS027B7DH01); **CS active high** (driver owns `SpiBus` + CS pin) |
| Commands | write lines `0x80`, clear all `0x20`, VCOM bit `0x40`; gate address 1-based, bit-reversed |
| VCOM | `Vcom::Software` (M1 bit, EXTMODE low) or `Vcom::ExtComIn` (pin or hardware timer, EXTMODE high); invert at ≥ 1 Hz |
| Refresh | immediate; `start_refresh()` returns 0 |
| Current | ~5 µW static (LS013B7DH03, datasheet typ.) |
| Source | Sharp LS013B7DH03 and LS027B7DH01 datasheets |
| Status | **mock-tested, not run on hardware** |

## API

```ignore
let mut lcd = SharpMemLcd::new(spi_bus, cs, delay, Panel::LS013B7DH03)
    .with_extcomin(extcomin_pin)  // or .with_hardware_extcomin()
    .with_disp(enable_pin);       // optional
lcd.init()?;                      // DISP high, clear-all
lcd.flush(&fb)?;                  // all lines
lcd.flush_dirty(&mut fb)?;        // only lines in the dirty rectangle
lcd.flush_rows(&fb, first, n)?;
lcd.toggle_vcom()?;               // from a 1 Hz timer (software VCOM)
lcd.clear_all()?;
```

`asynch::SharpMemLcd` has the same API over embedded-hal-async `SpiBus`
and `DelayNs`. Both implement the `zigbee_display` `Display` traits.

## RAM

Frame buffer (`MonoFrame`, 1 = white) plus 66 bytes of stack while flushing:

| Panel | Bytes |
|---|---|
| LS013B7DH03 128x128 | 2 048 |
| LS013B7DH05 144x168 | 3 024 |
| LS011B7DH03 160x68 | 1 360 |
| LS027B7DH01 400x240 | 12 000 |

## Wiring

| Panel pin | MCU |
|---|---|
| SCLK | SPI SCK |
| SI | SPI MOSI |
| SCS | GPIO (driven high during a transfer) |
| DISP | GPIO or tie high |
| EXTCOMIN | GPIO / timer output, or tie low with software VCOM |
| EXTMODE | high = EXTCOMIN, low = software VCOM (fixed on the board) |

### Silicon Labs WSTK (BRD4001A main board)

The LCD signals come from the radio board. Values below are from GSDK 4.5
`hardware/kit/*/config/hal-config-board.h` and `sl_memlcd_usart_config.h`.
The WSTK ties EXTMODE high, so use `Vcom::ExtComIn`. GSDK drives EXTCOMIN
at 60 Hz from hardware (PRS/RTCC). With a plain GPIO, call `toggle_vcom()`
from a timer instead.

| Radio board | SPI | SCLK | SI | SCS | DISP_ENABLE | EXTCOMIN |
|---|---|---|---|---|---|---|
| BRD4182A (MG22, QFN40) | USART0 | PC2 | PC0 | PC6 | PC7 | PA0 |
| BRD4181A / BRD4180A (MG21) | USART2 | PC2 | PC0 | PC3 | PD4 | PA0 |
| BRD4183A (MG22, QFN32) | none: GSDK has no SPIDISPLAY mapping | | | | | |

DISP_ENABLE powers the display circuit on the WSTK; drive it high (pass
it to `with_disp` or set it yourself) before `init`. GSDK runs the bus at
1.1 MHz with 6 µs CS setup and 2 µs hold, which are this driver's default
`Timing`.
