# APDS-9253 Digital RGB and Ambient Light Sensor Driver

[![Crates.io](https://img.shields.io/crates/v/apds9253.svg)](https://crates.io/crates/apds9253)
[![Documentation](https://docs.rs/apds9253/badge.svg)](https://docs.rs/apds9253)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](https://github.com/yourusername/apds9253-rs)

A platform-agnostic Rust driver for the APDS-9253 digital RGB and ambient light sensor, built using the [`device-driver`](https://crates.io/crates/device-driver) toolkit for type-safe register operations.

## Features

- 🚀 **Type-safe register operations** using device-driver generated code
- 🎨 **High-level API** for RGB and ALS measurements
- ⚡ **Interrupt support** with threshold and variance modes
- 🔧 **Configurable gain** (1x to 18x) and integration time (3.125ms to 400ms)
- 💡 **Lux calculation** for ambient light sensing
- 🌈 **Color temperature estimation** with CIE 1931 chromaticity
- 💾 **Power management** with sleep modes
- 📐 **No-std compatible** for embedded systems
- 🔌 **embedded-hal** compatible I2C interface

## The Device

The APDS-9253 is a digital RGB sensor that provides:

- **Individual channels**: Red, Green, Blue, and IR
- **High resolution**: 13 to 20-bit ADC with programmable integration time
- **Flexible gain**: 1x, 3x, 6x, 9x, 18x analog gain settings
- **I2C interface**: Standard 7-bit addressing (0x52)
- **Interrupt support**: Configurable thresholds with persistence
- **Low power**: Standby and active modes

## Quick Start

Add this to your `Cargo.toml`:

```toml
[dependencies]
apds9253 = "0.1"
embedded-hal = "1.0"
```

### Basic Usage

```rust
use apds9253::{Apds9253, GainRange, Resolution};
use embedded_hal::i2c::I2c;

// Initialize your I2C interface
let i2c = /* your I2C implementation */;
let mut sensor = Apds9253::new(i2c);

// Initialize the sensor
sensor.init()?;

// Configure measurement settings
sensor.set_gain(GainRange::Gain3x)?;
sensor.set_resolution(Resolution::Bits18_100ms)?;

// Enable RGB mode and start measurements
sensor.enable_rgb_mode(true)?;
sensor.enable(true)?;

// Wait for measurement (integration time dependent)
// ... wait 200ms for 100ms integration time + processing

// Read RGB data
let rgb = sensor.read_rgb_data()?;
println!("R: {}, G: {}, B: {}, IR: {}", rgb.red, rgb.green, rgb.blue, rgb.ir);

// Calculate ambient light in lux
let lux = sensor.calculate_lux(&rgb)?;
println!("Ambient light: {:.2} lux", lux);

// Calculate color temperature
let color = sensor.calculate_color_temperature(&rgb)?;
println!("Color temperature: {}K", color.cct);
```

### Interrupt-Based Operation

```rust
use apds9253::{InterruptSource, Persistence};

// Configure interrupt for green channel (ALS)
sensor.configure_interrupt(
    InterruptSource::Green,
    1000,    // Low threshold
    50000,   // High threshold
    Persistence::Consecutive2, // Require 2 consecutive readings
)?;

// In your main loop or interrupt handler
if sensor.check_interrupt()? {
    let rgb = sensor.read_rgb_data()?;
    println!("Interrupt triggered! Green: {}", rgb.green);
    sensor.clear_interrupt()?;
}
```

## Configuration Options

### Gain Settings

| Gain | Use Case |
| --------- | ----------------------------------- |
| `Gain1x` | Bright light conditions |
| `Gain3x` | Normal indoor lighting (default) |
| `Gain6x` | Dim lighting |
| `Gain9x` | Low light conditions |
| `Gain18x` | Very low light, maximum sensitivity |

### Resolution and Integration Time

| Resolution | Integration Time | Use Case |
| ---------------- | ---------------- | -------------------------------- |
| `Bits13_3_125ms` | 3.125ms | Fast measurements, low precision |
| `Bits16_25ms` | 25ms | Quick measurements |
| `Bits17_50ms` | 50ms | Balanced speed/precision |
| `Bits18_100ms` | 100ms | Good precision (default) |
| `Bits19_200ms` | 200ms | High precision |
| `Bits20_400ms` | 400ms | Maximum precision, slow |

### Measurement Rates

Control how often the sensor takes measurements:

- `Ms25` - Every 25ms (40 Hz)
- `Ms50` - Every 50ms (20 Hz)
- `Ms100` - Every 100ms (10 Hz, default)
- `Ms200` - Every 200ms (5 Hz)
- `Ms500` - Every 500ms (2 Hz)
- `Ms1000` - Every 1000ms (1 Hz)
- `Ms2000` - Every 2000ms (0.5 Hz)

## Examples

The `examples/` directory contains several demonstration programs:

- [`basic_rgb.rs`](examples/basic_rgb.rs) - Basic RGB and lux measurements
- [`interrupt_demo.rs`](examples/interrupt_demo.rs) - Interrupt-based operation
- [`auto_gain.rs`](examples/auto_gain.rs) - Automatic gain control implementation

Run an example with:

```bash
cargo run --example basic_rgb
```

## Advanced Features

### Automatic Gain Control

Implement automatic gain control to optimize readings across different light conditions:

```rust
// Check for saturation (>90% of max ADC value)
let max_channel = rgb.red.max(rgb.green).max(rgb.blue);
if max_channel > (MAX_ADC_VALUE * 90 / 100) {
    // Reduce gain
    sensor.set_gain(lower_gain)?;
} else if max_channel < (MAX_ADC_VALUE / 100) {
    // Increase gain for better sensitivity
    sensor.set_gain(higher_gain)?;
}
```

### Color Analysis

Calculate color temperature and chromaticity coordinates:

```rust
let color = sensor.calculate_color_temperature(&rgb)?;
println!("CCT: {}K, x: {:.3}, y: {:.3}", color.cct, color.x, color.y);

// Classify light sources
match color.cct {
    2700..=3000 => println!("Warm white (incandescent)"),
    4000..=5000 => println!("Cool white (fluorescent)"),
    5500..=6500 => println!("Daylight"),
    _ => println!("Other light source"),
}
```

### Power Management

```rust
// Put sensor in standby to save power
sensor.standby()?;

// Wake up and resume measurements
sensor.enable(true)?;
```

## Platform Support

This driver works with any platform that provides:

- **I2C interface** implementing `embedded-hal::i2c::I2c`
- **Delay provider** implementing `embedded-hal::delay::DelayNs`

Tested platforms:

- **Linux** (Raspberry Pi, etc.)
- **STM32** microcontrollers
- **ESP32** with esp-hal
- **RP2040** (Raspberry Pi Pico)
- **nRF52/nRF53** Nordic chips

## Device-Driver Benefits

This driver leverages the [`device-driver`](https://crates.io/crates/device-driver) toolkit, providing:

- **Generated register definitions** from YAML specification
- **Type-safe field access** with compile-time validation
- **Automatic documentation** from register descriptions
- **Reduced boilerplate** compared to manual implementations
- **Consistent API patterns** across all registers

## Error Handling

The driver provides comprehensive error handling:

```rust
use apds9253::Error;

match sensor.read_rgb_data() {
    Ok(rgb) => println!("RGB: {:?}", rgb),
    Err(Error::I2c(e)) => println!("I2C error: {:?}", e),
    Err(Error::Device(e)) => println!("Device error: {:?}", e),
    Err(Error::NotReady) => println!("Data not ready"),
    Err(Error::InvalidConfig) => println!("Invalid configuration"),
}
```

### Development Setup

```bash
git clone https://github.com/yourusername/apds9253-rs
cd apds9253-rs
cargo test
cargo doc --open

# Run examples (requires I2C hardware)
cargo run --example basic_rgb
```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

## Acknowledgments

- Built with the [`device-driver`](https://crates.io/crates/device-driver) toolkit
- Inspired by the [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) ecosystem
- APDS-9253 datasheet and specifications from Broadcom

## Related Projects

- [`apds9960`](https://crates.io/crates/apds9960) - Driver for APDS-9960 with gesture detection
- [`veml6030`](https://crates.io/crates/veml6030) - Alternative ambient light sensor
- [`tcs3472`](https://crates.io/crates/tcs3472) - Another RGB color sensor driver
