# RP2040/RP2350 HUB75 Examples

This directory contains examples for using the `hub75-embassy` driver with RP2040 and RP2350 microcontrollers (Raspberry Pi Pico, Pico W, and compatible boards).

## Setup

### Prerequisites

These examples are set up assuming you are using [probe-rs](https://probe.rs) for flashing and debugging. Make sure you have it installed:

```bash
cargo install cargo-binstall # binary installer tool (optional but recommended)
cargo binstall probe-rs-tools # or cargo install if you don't use binstall
```

You'll also need to install the ARM Cortex-M0+ target:

```bash
rustup target add thumbv6m-none-eabi
```

### Supported Boards

- **Raspberry Pi Pico** - RP2040 with 2MB Flash, 264KB RAM
- **Raspberry Pi Pico W** - RP2040 with WiFi capability
- **RP2350-based boards** - Newer generation with enhanced features
- **Compatible third-party RP2040 boards**

## Hardware Connections

The examples assume the following pin connections for a 64x32 HUB75 panel:

| HUB75 Pin | Pico Pin | GPIO | Description |
|-----------|----------|------|-------------|
| R1 | Pin 4 | GP2 | Upper half red data |
| G1 | Pin 5 | GP3 | Upper half green data |
| B1 | Pin 6 | GP4 | Upper half blue data |
| R2 | Pin 7 | GP5 | Lower half red data |
| G2 | Pin 9 | GP6 | Lower half green data |
| B2 | Pin 10 | GP7 | Lower half blue data |
| A | Pin 11 | GP8 | Row address bit 0 |
| B | Pin 12 | GP9 | Row address bit 1 |
| C | Pin 14 | GP10 | Row address bit 2 |
| D | Pin 15 | GP11 | Row address bit 3 |
| CLK | Pin 16 | GP12 | Clock signal |
| LAT | Pin 17 | GP13 | Latch signal |
| OE | Pin 19 | GP14 | Output enable (active low) |

**Power Connections:**

- HUB75 5V → External 5V power supply (NOT Pico's VBUS)
- HUB75 GND → Pico GND + Power supply GND
- Pico can be powered via USB or external 3.3V

**Important:** HUB75 panels can draw several amps. Use a dedicated 5V power supply rated for your panel's requirements.

## Examples

### Basic Display (`basic_display.rs`)

Simple demonstration of HUB75 setup with basic shapes and text.

```bash
cd examples/rp
cargo run --release --bin basic_display
```

### Comprehensive Demo (`comprehensive_demo.rs`)

The original full-featured example with advanced task management and multiple animation modes.

```bash
cargo run --release --bin comprehensive_demo
```

### Animated Effects (`animated_effects.rs`)

Showcase of various visual effects including rainbow waves, matrix rain, fire simulation, plasma tunnels, and starfield.

```bash
cargo run --release --bin animated_effects
```

### Performance Test (`performance_test.rs`)

Performance measurement and optimization testing with different rendering patterns and frame rate analysis.

```bash
cargo run --release --bin performance_test
```

### Pico W Demo (`pico_w_demo.rs`)

Demonstrates WiFi status display on Pico W boards (requires `pico-w` feature).

```bash
cargo run --release --features pico-w --bin pico_w_demo
```

## Features

### Panel Size Support

Add panel size features to optimize for your specific display:

```bash
# For 32x16 panels
cargo run --release --features size-32x16 --bin basic_display

# For 64x64 panels  
cargo run --release --features size-64x64 --bin basic_display
```

### Color Depth Options

Choose color depth based on your performance requirements:

```bash
# 4-bit color (faster refresh, lower quality)
cargo run --release --features color-4bit --bin basic_display

# 8-bit color (slower refresh, higher quality)
cargo run --release --features color-8bit --bin basic_display
```

### Board-Specific Features

```bash
# Standard Pico
cargo run --release --features pico --bin basic_display

# Pico W with WiFi
cargo run --release --features pico-w --bin pico_w_demo
```

## Performance Optimization

### RP2040-Specific Optimizations

1. **Dual Core Usage**: The RP2040's second core can be used for display refresh while the main core handles graphics
1. **PIO Integration**: Future versions may use PIO for hardware-accelerated HUB75 timing
1. **DMA Support**: Potential for DMA-based data transfer to reduce CPU load
1. **Clock Configuration**: Optimize system clocks for display timing

### Memory Considerations

- **Flash**: 2MB available, examples use ~50-100KB
- **RAM**: 264KB available, display buffers use ~8KB for 64x32 panels
- **Stack**: Monitor stack usage in complex animations

### Recommended Settings

For best performance:

- Use `--release` builds
- Enable appropriate color depth for your needs
- Consider panel size optimizations
- Monitor frame rates with performance test

## Troubleshooting

### Compilation Issues

1. **Target not found**: `rustup target add thumbv6m-none-eabi`
1. **Probe-rs errors**: Ensure probe-rs is installed and device is connected
1. **Memory errors**: Check memory.x configuration for your specific board

### Runtime Issues

1. **Display not working**:

   - Verify pin connections
   - Check power supply (5V, adequate current)
   - Ensure proper grounding

1. **Flickering or artifacts**:

   - Increase power supply capacity
   - Check for loose connections
   - Verify clock timing

1. **Performance issues**:

   - Use release builds (`--release`)
   - Reduce color depth if needed
   - Monitor with performance_test example

1. **WiFi issues (Pico W)**:

   - Ensure `pico-w` feature is enabled
   - Check WiFi credentials and network
   - Monitor with defmt logs

### Debug Output

All examples use `defmt` for logging. View debug output with:

```bash
# Run with probe-rs to see defmt logs
cargo run --release --bin basic_display
```

## Hardware Variants

### RP2040 vs RP2350

- **RP2040**: Original chip, well-tested, 133MHz max
- **RP2350**: Newer chip with enhanced features, higher clock speeds
- Examples work on both, but RP2350 may offer better performance

### Third-Party Boards

Most RP2040-based boards should work with pin adjustments:

- **Adafruit Feather RP2040**
- **SparkFun Pro Micro RP2040**
- **Arduino Nano RP2040 Connect**
- **Pimoroni boards**

Adjust pin assignments in the example code as needed.

## Advanced Usage

### Custom Pin Mapping

To use different pins, modify the `Hub75Pins` structure:

```rust
let pins = Hub75Pins {
    r1: Output::new(p.PIN_0, Level::Low),  // Changed from PIN_2
    g1: Output::new(p.PIN_1, Level::Low),  // Changed from PIN_3
    // ... adjust other pins as needed
};
```

### Multiple Panels

For larger displays, chain multiple panels and adjust the width constant:

```rust
type Display = Hub75Display<Output<'static>, 128, 32, 6>; // Two 64x32 panels
```

### Integration with Other Peripherals

Examples can be extended to work with:

- **Sensors**: I2C/SPI sensors for data display
- **Audio**: PWM audio output alongside display
- **Storage**: SD card or flash storage for images
- **Networking**: HTTP servers, MQTT, etc. (Pico W)

## Contributing

Feel free to submit additional examples or improvements:

- New visual effects
- Hardware integrations
- Performance optimizations
- Board-specific examples

## Additional Resources

- [RP2040 Datasheet](https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf)
- [Pico Pinout](https://datasheets.raspberrypi.org/pico/Pico-R3-A4-Pinout.pdf)
- [Embassy RP Documentation](https://docs.embassy.dev/embassy-rp/)
- [HUB75 Protocol Guide](https://github.com/hzeller/rpi-rgb-led-matrix)
