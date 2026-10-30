# zigbee-senseair

Senseair S8 and Sunrise / Sunlight NDIR CO2 sensors over Modbus RTU (UART):
CO2 and status readings, ABC period, background / zero / target
calibration, firmware and serial number, and the Sunrise / Sunlight
single-measurement mode with state save / restore (EN-pin power cycling).

| | |
|---|---|
| Bus | UART 9600 8N1, Modbus RTU (CRC-16/MODBUS, low byte first); S8 address `0xFE` ("any sensor"), Sunrise / Sunlight `0x68` |
| ID | IR29 firmware revision + IR30–31 serial number (both families); S8 type ID IR26–27; Sunrise firmware type IR24 |
| Timing | 180 ms response time-out; S8 2 s lamp cycle (wait ≥ 2 s after calibration); Sunrise 35 ms start-up after EN, 160–300 ms per sample, 2.4 s single measurement with the default 8 samples, default period 16 s |
| Current | not given in the Modbus documents; see the product specifications (Sunrise average-current curves in the integration manual) |
| Output | `Co2` in ppm (omitted when the status flags mark the value invalid) |
| Source | Senseair TDE2067 "Modbus on Senseair S8"; TDE7318 "Modbus on Senseair Sunrise & Sunlight"; Sunrise handling and integration manual |
| Status | mock-tested |

## Usage

```rust,ignore
use zigbee_senseair::{Calibration, Senseair};
use zigbee_sensor::blocking::Sensor;

let mut co2 = Senseair::s8(uart); // or Senseair::sunrise(uart)
let wait_us = co2.start_measurement()?;   // reads IR1–IR4
// ... wait_us later:
let m = co2.read_measurement()?;          // m.co2_ppm, m.status, m.is_valid()

// Any other request: encode with co2.device(), then read the reply.
let wait_us = co2.send(co2.device().set_abc_period(180))?;
// ... wait_us later:
co2.read_ack()?;
```

`Device` encodes every request (`read_input`, `read_holding`, `write`,
`set_abc_period`, `calibrate`, `set_measurement_mode`, `set_pressure`,
`start_single`, `read_state`, …). Requests the model does not implement
(e.g. target calibration or `write_many` on the S8) or out-of-range
arguments give `None`, which `send` reports as `Unsupported`.

Calibration: `send(device.clear_calibration_status())`, then
`send(device.calibrate(Calibration::Background))`; on the S8 wait
`S8_CALIBRATION_US` and read HR1 (`read_calibration_status`), on the
Sunrise the calibration is applied on the next measurement.

Sunrise / Sunlight single-measurement mode (after
`set_measurement_mode(true)` and `reset()`), per TDE7318 §3.3:

1. EN high, wait `SUNRISE_STARTUP_US` (35 ms).
2. `send(device.start_single(saved_state.as_ref(), None))`, `read_ack()`.
   With no saved state only the start command is written — never write
   zeros to the state registers.
3. Wait for nRDY low or `SUNRISE_SINGLE_US` (2.4 s).
4. `start_measurement()` / `read_measurement()`.
5. `send(device.read_state())`, `saved_state = Some(read_state()?)`; with
   ABC enabled add the powered-down hours with `add_abc_hours`.
6. EN low.

## Notes

- No sleep commands: no `LowPower` implementation. The S8 runs
  continuously; the Sunrise / Sunlight is powered down with EN.
- S8 writes use function 06, Sunrise / Sunlight writes function 0x10 (06 is
  not implemented there); `Device::write` picks the right one.
- S8 packets are limited to 39 bytes, so reads are capped at 17 registers
  (32 on the Sunrise).
- Exception replies: illegal function / address → `Unsupported`, others →
  `InvalidData`.
- Sunrise "(EE)" registers (measurement mode/period/samples, ABC period and
  target, meter control, address, …) are EEPROM-backed: keep the total
  below 10 000 writes, configure once, and do not remove power before the
  write reply arrives. Mode, period, samples, ABC target and address need
  a reset.
- Sunrise ABC requires a non-zero HR14 *and* bit 1 of HR19 clear; use
  `read_meter_control` / `set_meter_control` for read-modify-write.
- With pressure compensation enabled, pass the pressure to `start_single`
  (or `set_pressure` before a start without state); with ABC as well, also
  save and restore HR48 (`sunrise::ABC_PRESSURE`).
- S8 HR32 = 0 suspends ABC without clearing ABC history.
- IR1 bit 7 on the Sunrise ("no measurement completed") is set until the
  first measurement after power-up finishes and is treated as invalid.

## Wiring

| Sensor signal | MCU |
|---|---|
| GND | GND |
| Supply (S8 G+; Sunrise VBB) | supply per product specification |
| UART TxD | UART RX |
| UART RxD | UART TX |
| Sunrise EN | GPIO out (single-measurement mode), or tied high for continuous mode |
| Sunrise nRDY | GPIO in (optional; otherwise use worst-case timing) |
| Sunrise VDDIO | from VBB (continuous) or host-switched I/O supply (low power) |

Pin numbers differ between models; see the product specifications. The
Sunrise integration manual notes that long TxD wires may need an external
pull-up in addition to the internal 100 kΩ.
