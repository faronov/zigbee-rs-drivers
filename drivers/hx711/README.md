# zigbee-hx711

Avia Semiconductor HX711 24-bit ADC for load cells (bathroom/kitchen
scales, bee hives, tank weight).

| | |
|---|---|
| Interface | PD_SCK output + DOUT input (bit-banged, 1 µs per half bit) |
| Channels | A gain 128 / A gain 64 / B gain 32 (selected by 25/26/27 pulses, applies to the next conversion) |
| Rate | 10 SPS (RATE low) or 80 SPS (RATE high); settling 400 ms / 50 ms after power-up, reset or gain change |
| Output | `Quantity::Mass` in mg via integer tare + Q16 scale; raw 24-bit value |
| Current | ≈ 1.5 mA running, < 1 µA powered down (PD_SCK high > 60 µs) |
| Source | Avia Semiconductor HX711 datasheet |
| Status | mock-tested |

Timing exception: the 24–27 clock pulses use a blocking `DelayNs` (≈ 55 µs
per read). PD_SCK high for more than 60 µs powers the chip down, so mask
interrupts around `read_measurement` if a long ISR could preempt it.

Wiring notes:

* AVDD/DVDD 2.6–5.5 V. Common breakout boards run the load-cell excitation
  from the on-chip regulator (E+ ≈ 4.3 V when VSUP = 5 V); at 3.3 V supply
  tie the board's VCC and VDD to 3.3 V.
* Power down (`sleep()`) also removes the load-cell excitation, which is
  the dominant current on battery scales.
* Calibrate once: `tare(raw_empty)`, then
  `Calibration::from_reference(offset, raw_with_known_mass, mass_mg)` and
  store the result.
