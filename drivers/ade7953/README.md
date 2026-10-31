# zigbee-ade7953

Analog Devices ADE7953 single-phase energy metering IC with two current
channels (Shelly 2.5 / Plus style smart relays), SPI or I2C.

| | |
|---|---|
| Interface | SPI ≤ 5 MHz (`[addr_hi, addr_lo, 0x80 read / 0x00 write, data]`, MSB first) or I2C ≤ 400 kHz at 0x38; interface chosen by CS / SCLK pins, locked by `init` |
| Conversion | continuous; RMS/power update 6.99 kHz, RMS settling ≈ 200 ms, PERIOD 30-40 ms; `start_measurement` returns 0 |
| Power-down current | none (no power-down mode) |
| Output | Voltage mV, Current µA, Power mW (signed), Energy mWh (net, driver-accumulated), PowerFactor 0.001, Frequency mHz |
| Calibration | `Calibration::from_front_end(R_top Ω, R_bottom Ω, shunt µΩ, PGA_V, PGA_I)` (nV / pA / nW / nWh per LSB) or set directly; `GainTrim` for AIGAIN/AVGAIN/AWGAIN/BIGAIN/BWGAIN |
| Source | Analog Devices ADE7953 data sheet Rev. C |
| Status | mock-tested |

Notes:
- Errata / required setting: after every power-up or reset, `init`
  writes 0xAD to 0x0FE and immediately 0x0030 to the protected register
  0x120. It also writes CONFIG = 0x0004 (COMM_LOCK cleared: interface
  locked, HPF on), LCYCMODE = 0x40 (energy read-with-reset), PGA_V /
  PGA_IA / PGA_IB, the optional gain trims, and reads RSTIRQSTATA to clear
  the reset flag. Wait ≥ 100 ms (`RESET_WAIT_US`) after power-up /
  `reset()` or use the IRQ pin. Call `reset_occurred()` periodically:
  `true` means the chip reset (brown-out) and `init` must be repeated.
- SPI lock caveat: when CONFIG.COMM_LOCK is written over SPI, CS must stay
  low ≥ 1.2 µs after the last SCLK edge — configure the `SpiDevice`
  accordingly. I2C needs ≥ 100 ns between SCL and SDA edges. The UART
  interface is not supported.
- Register width follows the address (0x0xx 8 bit, 0x1xx 16 bit, 0x2xx
  24 bit, 0x3xx 32 bit sign-extended; 0x702 VERSION and 0x800 EX_REF are
  8 bit); `read_register` / `write_register` handle it. The driver uses the
  24-bit aliases.
- `read_measurement` reads VRMS, IRMSx, xWATT, PFx, PERIOD and then
  AENERGYx (read-with-reset), adding the signed delta to a per-channel
  `i64` nWh total (`energy_nwh()`, `set_energy_mwh()` for NV restore).
  Read at least every ≈ 40 s at full scale (AENERGY 24-bit roll-over) —
  longer at lower power. Energy reported = max(total, 0).
- Channel A (shunt, PGA ×1-×22) or B (CT, ×1-×16) via `with_channel` /
  `set_channel`; give each its own `Calibration` if the front ends differ.
  ×22 on PGA_V or PGA_IB → `init` returns `Unsupported`.
- Scaling: datasheet full-scale codes (IRMS/VRMS 9 032 007 for 500 mV
  peak, AWATT 4 862 401, AENERGY 206.9 kHz at unity WGAIN) are typical;
  calibrate against a reference meter. Frequency = 223.75 kHz / (PERIOD +
  1); PF = PFx · 2^-15 (negative = capacitive). Per read: three `u64`
  divisions by 10^6 and one `i64`→`u64` division; `u128` only in the
  const calibration helper.
