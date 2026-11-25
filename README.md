ina260
======

[![crates.io](https://img.shields.io/crates/v/ina260.svg)](https://crates.io/crates/ina260)
[![docs.rs](https://docs.rs/ina260/badge.svg)](https://docs.rs/ina260)

A `no_std` driver for the [TI INA260][], a power monitor with an integrated 2 mΩ shunt that
measures current, bus voltage and power, over I2C. It works with any I2C bus implementing
[embedded-hal][] 1.0 (blocking) or [embedded-hal-async][] 1.0.

Features
--------

| Feature | Default | Effect |
|---------|---------|--------|
| `async` | yes | `AsyncINA260`, over `embedded_hal_async::i2c::I2c` |
| `sync`  | no  | `INA260`, over `embedded_hal::i2c::I2c` |
| `defmt` | no  | `defmt::Format` for `Register`, `Averaging`, `BVConvTime`, `SCConvTime` and `OperMode`, and register accesses logged with [defmt][] |
| `log`   | no  | register accesses logged with [log][] |

At least one of `async` and `sync` is required, and both can be enabled together. `defmt` and
`log` exclude each other.

Usage
-----

```toml
[dependencies]
ina260 = "0.4"
```

```rust
use embedded_hal_async::i2c::I2c;
use ina260::{AsyncINA260, Averaging};

async fn measure<I: I2c>(i2c: I) -> Result<(), I::Error> {
    // Resets the chip at the default address, 0x40 (A0 and A1 to GND)
    let mut ina260 = AsyncINA260::new(i2c).await?;
    ina260.set_averaging_mode(Averaging::AVG16).await?;

    let voltage = ina260.voltage().await?; // µV
    let current = ina260.current().await?; // µA, signed
    let power = ina260.power().await?; // mW
    Ok(())
}
```

The blocking `INA260` has the same methods, without `.await`:

```toml
[dependencies]
ina260 = { version = "0.4", default-features = false, features = ["sync"] }
```

A chip at another address (0x40 to 0x4F, set by the A0 and A1 pins) is opened with
`new_with_address(i2c, address)`.

API
---

- Measurements: `current()` (µA, signed), `voltage()` (µV) and `power()` (mW); the register
  values with `current_raw()`, `voltage_raw()` and `power_raw()` (1.25 mA, 1.25 mV and 10 mW per
  bit); whole amperes, volts or watts and the rest in 10 µA, 10 µV or 10 µW with
  `current_split()`, `voltage_split()` and `power_split()`.
- Configuration: `set_operating_mode()` (current and/or voltage, continuous or triggered, or
  shutdown), `set_averaging_mode()` (1 to 1024 samples), `set_bvconvtime_mode()` and
  `set_scconvtime_mode()` (conversion times from 140 µs to 8.244 ms), and the ALERT pin with
  `set_mask_enable()` and `set_alert_limit()`.
- `did()` and `rid()`: the die ID and revision. `release()` shuts the chip down and gives the
  I2C bus back.

License
-------

[0-clause BSD license](LICENSE-0BSD.txt).

[TI INA260]: https://www.ti.com/lit/ds/symlink/ina260.pdf
[embedded-hal]: https://crates.io/crates/embedded-hal
[embedded-hal-async]: https://crates.io/crates/embedded-hal-async
[defmt]: https://crates.io/crates/defmt
[log]: https://crates.io/crates/log
