//! Interrupt-based sensor reading example
//!
//! This example demonstrates how to:
//! - Configure interrupt thresholds
//! - Use interrupts to trigger readings
//! - Handle different interrupt sources

use apds9253::{Apds9253, LsGainRange, LsIntSel, LsPersist, LsResolution};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

// This example uses linux-embedded-hal for demonstration
use linux_embedded_hal::{Delay, I2cdev};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize I2C interface
    let i2c = I2cdev::new("/dev/i2c-1")?;
    let mut delay = Delay;

    // Create sensor instance
    let mut sensor = Apds9253::new(i2c);

    println!("Initializing APDS-9253 sensor with interrupt support...");

    // Initialize the sensor
    sensor.init()?;

    // Configure sensor settings
    sensor.set_gain(LsGainRange::Gain3x)?;
    sensor.set_resolution(LsResolution::Bits18_100ms)?;

    // Enable RGB mode
    sensor.enable_rgb_mode(true)?;

    // Configure threshold interrupt for green channel (ALS)
    // Trigger interrupt when green value is below 1000 or above 50000
    sensor.configure_threshold_interrupt(
        LsIntSel::Green,
        1000,                      // Low threshold
        50000,                     // High threshold
        LsPersist::Consecutive2,   // Require 2 consecutive readings
    )?;

    // Enable the sensor
    sensor.enable(true)?;

    println!("Sensor configured with interrupt thresholds:");
    println!("- Source: Green channel (ALS)");
    println!("- Low threshold: 1000");
    println!("- High threshold: 50000");
    println!("- Persistence: 2 consecutive readings");
    println!("\nMonitoring for interrupt conditions...");
    println!("Press Ctrl+C to exit\n");

    let mut reading_count = 0;

    loop {
        // Wait for measurement
        delay.delay_ms(200);

        // Check if data is ready
        if sensor.is_data_ready()? {
            reading_count += 1;

            // Read current data
            let rgb = sensor.read_rgb_data()?;
            let lux = sensor.calculate_lux(&rgb)?;

            // Check interrupt status
            let interrupt_active = sensor.check_interrupt()?;

            if interrupt_active {
                println!(
                    "🚨 INTERRUPT #{}: G: {:6} (Lux: {:6.2}) - Threshold exceeded!",
                    reading_count, rgb.green, lux
                );

                // Clear the interrupt
                sensor.clear_interrupt()?;

                // Show full RGB data when interrupt occurs
                println!(
                    "   Full reading: R: {:6} G: {:6} B: {:6} IR: {:6}",
                    rgb.red, rgb.green, rgb.blue, rgb.ir
                );
            } else {
                // Normal reading - just show green channel and lux
                println!(
                    "Reading #{:3}: G: {:6} (Lux: {:6.2}) - Normal",
                    reading_count, rgb.green, lux
                );
            }
        }

        // Wait before next check
        delay.delay_ms(1000);
    }
}

#[cfg(not(target_os = "linux"))]
fn main() {
    println!("This example requires Linux with I2C support.");
    println!("Please adapt the I2C initialization for your platform.");
}
