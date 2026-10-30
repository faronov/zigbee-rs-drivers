# Driver guide

Rules every driver in this repository follows. `drivers/sht3x` is the
reference implementation; copy its structure.

## Why the API looks like this

A Zigbee node runs a radio stack with hard timing (MAC ACKs, parent polls,
retries). A driver that busy-waits 15 ms for a humidity conversion, or 5 s
for a CO2 sample, stalls the stack and keeps the CPU awake on a sleepy end
device. So:

* **No delay during a measurement.** `start_measurement()` triggers the
  conversion and returns the wait in microseconds (`WaitUs`);
  `read_measurement()` fetches the result. The application schedules the
  wait (timer, `await`, or sleep).
* The same rule applies to reset, wake-up and configuration: return a
  `WaitUs` instead of calling a delay.
* The only exception is bit-banged protocols whose bit timing is in
  microseconds (1-Wire, DHT11/22, HX711). They may take a `DelayNs` for the
  bit timing of a single transaction; document it in the crate docs.

## Crate layout

* One crate per part family: `drivers/<name>/`, package `zigbee-<name>`.
  Inherit `version`, `edition`, `rust-version`, `license` and `repository`
  from the workspace, and set `[lints] workspace = true`.
* Dependencies: `zigbee-sensor`, `embedded-hal`, `embedded-hal-async`,
  `embedded-io`, `embedded-io-async` (workspace versions). Nothing else.
  Dev-dependency: `embedded-hal-mock` (workspace). Ask before adding
  anything else.
* `#![no_std]`, no `alloc`, no `unsafe` (forbidden by the workspace lints).
* Integer arithmetic. Floating point only where the datasheet algorithm
  cannot reasonably be done in fixed point; put `#[allow(clippy::float_arithmetic)]`
  on the smallest item and explain why.
* Avoid 64-bit division in hot paths and large lookup tables: targets
  include Cortex-M0 parts with 128–256 KB of flash.

## API shape

* Blocking driver type at the crate root (`Sht3x<I2C>`), async twin with the
  same name in `pub mod asynch`. Share every constant, encoder, decoder and
  compensation routine between them; the two types only differ in `.await`.
* Constructors: `new(bus, address)` (or `new(bus)` for fixed-address /
  UART / SPI parts), `release(self) -> Bus`. Builder-style `with_*` for
  configuration that needs no bus traffic.
* Implement `zigbee_sensor::blocking::Sensor` and
  `zigbee_sensor::asynch::Sensor`. Implement `LowPower` when the part has a
  sleep/power-down command.
* The measurement type implements `zigbee_sensor::Readings` and uses the
  units of `zigbee_sensor::Quantity` (0.01 °C, 0.01 %RH, Pa, milli-lux, ppm,
  0.1 µg/m³, mV, µA, mW, mg, nT, milli-g ...). If a quantity is missing in
  `Quantity`, report it in your summary instead of editing `core`.
* Errors: `zigbee_sensor::Error<BusError>`. Map transport errors with
  `.map_err(Error::bus)`. Use `Error::NotReady` when the device reports data
  not ready, `Error::UnexpectedId` for chip-ID checks, `Error::Crc` for
  checksums, `Error::InvalidArgument` for caller arguments outside the
  part's range. `NotReady` carries no wait hint: the caller retries after
  its own back-off (the expected wait is what `start_measurement` returned).
* A reset or wake that uses the I2C general call (address 0x00) reaches
  every device on the bus. Name it `general_call_reset()` or similar and
  say so in its doc comment and the crate README.
* Expose a `probe()`/`check_id()` when the part has an ID register.
* Free-running/continuous parts: `start_measurement()` may return `0` and
  `read_measurement()` returns `NotReady` until a fresh sample exists.
* UART streaming parts: a pure `Parser` with `feed(byte) -> Option<Frame>`
  plus a driver over `embedded_io::Read + Write` (and the async twin).
* Analog parts take `zigbee_sensor::adc::Millivolts` /
  `AsyncMillivolts`.

## Tests

Host tests (`cargo test`) with `embedded-hal-mock`:

* every bus transaction the driver issues, exact bytes;
* conversion/compensation against datasheet example values or worked
  examples (cite the section);
* checksum failure, ID mismatch, not-ready paths;
* at least one async test (copy `block_on` from `drivers/sht3x`).

Run before committing:

```sh
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo build --workspace --target thumbv6m-none-eabi
cargo build --workspace --target riscv32imc-unknown-none-elf
```

## Sources and licensing

The repository is MIT OR Apache-2.0 and the drivers are our own code:

* Write from the manufacturer datasheet / application notes. Cite the
  document name and revision in the crate docs.
* Do not copy code from other drivers (Zephyr, Linux, Arduino, crates.io,
  vendor SDKs). Reading them to cross-check register values or errata is
  fine.
* If a part needs a vendor algorithm or binary blob (Sensirion gas index,
  BMI270 config file, VL53L1X ULD), use it only if its license is
  permissive (MIT/BSD/Apache/Zlib); keep the original copyright notice in
  the file and add an entry to `NOTICE`. Proprietary blobs (e.g. Bosch
  BSEC) are not allowed: expose the raw values instead.

## Documentation

Each crate has a `README.md`: supported parts, bus and addresses,
conversion times, sleep current, wiring notes, and the validation status
(`mock-tested` until it has been run on hardware; then the board/platform).
