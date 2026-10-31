# zigbee-ens160

ScioSense ENS160 / ENS161 digital metal-oxide multi-gas sensor: TVOC,
equivalent CO2 and the UBA air-quality index, computed on-chip. The ENS161
adds LOW POWER (1 sample/min) and ULTRA LOW POWER (1 sample/5 min) modes
and the AQI-S index (0..=500, `Measurement::aqi_s`, reported as
`RelativeAirQualityIndex`).

| | |
|---|---|
| Bus | I2C 0x52 (ADDR low), 0x53 (ADDR high), ≤ 1 MHz; or SPI mode 0/3 ≤ 10 MHz (CSn low at power-up selects SPI) |
| ID | PART_ID 0x0160 (ENS160), 0x0161 (ENS161) |
| Sampling | STANDARD 1 s; ENS161 LP 60 s, ULP 300 s (free-running, `NEWDAT` flag) |
| Warm-up | 3 min after power-on; first hour of life is "initial start-up" (`Validity`) |
| Current | deep sleep 10 µA, idle 2 mA, STANDARD 29 mA typ. (1.8 V VDD, 1.8/3.3 V VDDIO) |
| Output | TVOC ppb, eCO2 ppm, AQI-UBA 1..=5; ENS161 AQI-S 0..=500 |
| Source | ScioSense ENS160 Datasheet v1.3 (SC-001224-DS-9, 2023-03-29); ENS161 operating-modes app note SC-002489-AN-1 v1.0 |
| Status | mock-tested |

## Usage

`start_measurement` enters the configured active mode (`with_active_mode`,
default STANDARD) and returns the time to the first sample, or `0` while the
chip is already sensing. `read_measurement` returns `NotReady` until a new
sample is flagged and `InvalidData` on STATER or "no valid output".
`LowPower::sleep` selects DEEP SLEEP, `wake` returns to IDLE.

Feed ambient conditions from a T/RH sensor with `set_compensation`
(0.01 °C, 0.01 %RH) for best accuracy.

## Notes

- The datasheet gives no reset/boot time; `BOOT_WAIT_US` (10 ms) is a
  conservative assumption used after `Reset` and DEEP SLEEP wake.
- When switching between active modes go through IDLE and keep the previous
  mode running ≥ 10 ms (STANDARD), 2 s (LP) or 25 s (ULP).
- The ENS160 core runs at 1.71–1.98 V; most breakouts carry an LDO.
- `read_raw_resistance` exposes the raw GPR words; R = 2^(raw/2048) Ω.

## Wiring

| ENS160 | MCU |
|---|---|
| VDD / VDDIO | 1.8 V / 1.8–3.3 V |
| SDA/MOSI, SCL/SCLK | I2C or SPI |
| ADDR/MISO | GND (0x52) / VDDIO (0x53), or SPI MISO |
| CS | VDDIO for I2C, chip select for SPI |
| INTn | optional data-ready GPIO |
