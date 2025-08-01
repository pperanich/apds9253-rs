# nRF HUB75 Examples

This directory contains examples for using the `hub75-embassy` driver with nRF52 series microcontrollers.

## Setup

### Prerequisites

These examples are set up assuming you are using [probe-rs](https://probe.rs) for flashing and debugging your chips. Make sure you have it installed:

```bash
cargo install cargo-binstall # binary installer tool (optional but recommended)
cargo binstall probe-rs-tools # or cargo install if you don't use binstall
```

You may also need to install the chip's toolchain:

```bash
rustup target add thumbv7em-none-eabihf  # For nRF52833/nRF52840
rustup target add thumbv7em-none-eabi    # For nRF52832
```

### Supported Chips

We use features to turn on the appropriate configurations for the chip you are flashing to. Currently supported chips are:

- `nrf52832` - 512KB Flash, 64KB RAM
- `nrf52833` - 512KB Flash, 128KB RAM
- `nrf52840` - 1MB Flash, 256KB RAM (default)

## Hardware Connections

The examples assume the following pin connections for a 64x32 HUB75 panel:

| HUB75 Pin | nRF52 Pin | Description |
|-----------|-----------|-------------|
| R1 | P0.02 | Upper half red data |
| G1 | P0.03 | Upper half green data |
| B1 | P0.04 | Upper half blue data |
| R2 | P0.05 | Lower half red data |
| G2 | P0.06 | Lower half green data |
| B2 | P0.07 | Lower half blue data |
| A | P0.08 | Row address bit 0 |
| B | P0.09 | Row address bit 1 |
| C | P0.10 | Row address bit 2 |
| D | P0.11 | Row address bit 3 |
| CLK | P0.12 | Clock signal |
| LAT | P0.13 | Latch signal |
| OE | P0.14 | Output enable (active low) |

**Note:** You can modify the pin assignments in each example's source code to match your hardware setup.

## Power Considerations

HUB75 panels can draw significant current (several amps for larger panels). Make sure your power supply can handle the load:

- Use a dedicated 5V power supply for the panel
- Connect the nRF52 and panel grounds together
- The nRF52 GPIO pins operate at 3.3V, which is compatible with most HUB75 panels

## Examples

### Basic Display (`basic_display.rs`)

Demonstrates basic HUB75 setup with simple shapes and text display.

```bash
cd examples/nrf
cargo run --release --features nrf52840 --bin basic_display
```

### Animated Patterns (`animated_patterns.rs`)

Shows various animated effects including rainbow bars, bouncing balls, plasma effects, and sparkles.

```bash
cargo run --release --features nrf52840 --bin animated_patterns
```

### Text Display (`text_display.rs`)

Demonstrates scrolling text, multiple fonts, and real-time information display.

```bash
cargo run --release --features nrf52840 --bin text_display
```

### Graphics Demo (`graphics_demo.rs`)

Advanced embedded-graphics features with complex shapes and patterns.

```bash
cargo run --release --features nrf52840 --bin graphics_demo
```

## Running on Different Chips

For nRF52833 (e.g., BBC micro:bit v2):

```bash
cargo run --release --features nrf52833 --target thumbv7em-none-eabihf --bin basic_display
```

For nRF52832:

```bash
cargo run --release --features nrf52832 --target thumbv7em-none-eabi --bin basic_display
```

## Memory Configuration

The examples include memory layout files for different nRF52 variants:

- `memory.x` - Default (nRF52840)
- `memory-nrf52832.x` - For nRF52832
- `memory-nrf52833.x` - For nRF52833

The build system automatically selects the correct memory layout based on the feature flags.

## Troubleshooting

### Compilation Issues

1. **Missing target**: Install the correct target with `rustup target add thumbv7em-none-eabihf`
1. **Probe-rs not found**: Install with `cargo install probe-rs-tools`
1. **Memory errors**: Ensure you're using the correct memory layout for your chip

### Runtime Issues

1. **Display not working**: Check pin connections and power supply
1. **Flickering**: Ensure adequate power supply and proper grounding
1. **Colors wrong**: Verify RGB pin connections and color bit depth settings

### Performance Tips

1. Use `--release` builds for better performance
1. Enable double buffering for smooth animations
1. Consider reducing color depth for faster refresh rates
1. Use const generics to optimize for your specific panel size

## Additional Resources

- [Embassy Documentation](https://embassy.dev/)
- [nRF52 Reference Manual](https://infocenter.nordicsemi.com/topic/struct_nrf52/struct/nrf52840.html)
- [HUB75 Protocol Documentation](https://github.com/hzeller/rpi-rgb-led-matrix)
- [embedded-graphics Documentation](https://docs.rs/embedded-graphics/)

## Contributing

Feel free to submit additional examples or improvements to existing ones!
