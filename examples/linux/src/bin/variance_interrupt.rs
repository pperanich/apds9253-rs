//! Variance interrupt example
//!
//! This example demonstrates how to:
//! - Configure variance-based interrupts
//! - Detect sudden changes in light conditions
//! - Use variance thresholds for motion detection

use apds9253::{Apds9253, LsGainRange, LsIntSel, LsPersist, LsResolution, LsThresVar};
use embedded_hal::delay::DelayNs;
use linux_embedded_hal::{Delay, I2cdev};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize I2C interface
    let i2c = I2cdev::new("/dev/i2c-1")?;
    let mut delay = Delay;

    // Create sensor instance
    let mut sensor = Apds9253::new(i2c);

    println!("Initializing APDS-9253 sensor with variance interrupt...");

    // Initialize the sensor
    sensor.init()?;

    // Configure sensor settings for responsive detection
    sensor.set_gain(LsGainRange::Gain6x)?;
    sensor.set_resolution(LsResolution::Bits17_50ms)?; // Faster sampling

    // Enable RGB mode
    sensor.enable_rgb_mode(true)?;

    // Configure variance interrupt for green channel
    // Trigger when light changes by more than 32 counts between readings
    sensor.configure_variance_interrupt(
        LsIntSel::Green,
        LsThresVar::Counts32,     // Variance threshold
        LsPersist::Every,         // Trigger on every variance detection
    )?;

    // Enable the sensor
    sensor.enable(true)?;

    println!("Sensor configured with variance interrupt:");
    println!("- Source: Green channel (ALS)");
    println!("- Variance threshold: 32 counts");
    println!("- Persistence: Every detection");
    println!("- Integration time: 50ms (fast response)");
    println!("\nMonitoring for light changes...");
    println!("Try moving your hand over the sensor or changing lighting");
    println!("Press Ctrl+C to exit\n");

    let mut reading_count = 0;
    let mut last_green = 0u32;

    loop {
        // Wait for measurement
        delay.delay_ms(100); // Fast polling for responsive detection

        // Check if data is ready
        if sensor.is_data_ready()? {
            reading_count += 1;

            // Read current data
            let rgb = sensor.read_rgb_data()?;
            let lux = sensor.calculate_lux(&rgb)?;

            // Calculate actual variance from previous reading
            let variance = if last_green > 0 {
                rgb.green.abs_diff(last_green)
            } else {
                0
            };

            // Check interrupt status
            let interrupt_active = sensor.check_interrupt()?;

            if interrupt_active {
                println!(
                    "🔄 VARIANCE DETECTED #{}: G: {:6} -> {:6} (Δ{:+6}) | Lux: {:6.2}",
                    reading_count, last_green, rgb.green, 
                    rgb.green as i32 - last_green as i32, lux
                );

                // Clear the interrupt
                sensor.clear_interrupt()?;

                // Show full RGB data when variance detected
                println!(
                    "   Full reading: R: {:6} G: {:6} B: {:6} IR: {:6}",
                    rgb.red, rgb.green, rgb.blue, rgb.ir
                );
            } else {
                // Normal reading - show minimal info
                if reading_count % 10 == 0 {
                    println!(
                        "Reading #{:3}: G: {:6} (Lux: {:6.2}) - Stable",
                        reading_count, rgb.green, lux
                    );
                }
            }

            last_green = rgb.green;
        }

        // Short delay for responsive detection
        delay.delay_ms(50);
    }
}

#[cfg(not(target_os = "linux"))]
fn main() {
    println!("This example requires Linux with I2C support.");
    println!("Please adapt the I2C initialization for your platform.");
}