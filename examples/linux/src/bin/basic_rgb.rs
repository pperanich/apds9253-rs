//! Basic RGB sensor reading example
//!
//! This example demonstrates how to:
//! - Initialize the APDS-9253 sensor
//! - Configure gain and integration time
//! - Read RGB and IR values
//! - Calculate lux and color temperature

use apds9253::{Apds9253, LsGainRange, LsMeasurementRate, LsResolution};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

// This example uses linux-embedded-hal for demonstration
// Replace with your platform's I2C implementation
use linux_embedded_hal::{Delay, I2cdev};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize I2C interface
    let i2c = I2cdev::new("/dev/i2c-1")?;
    let mut delay = Delay;

    // Create sensor instance
    let mut sensor = Apds9253::new(i2c);

    println!("Initializing APDS-9253 sensor...");

    // Initialize the sensor
    sensor.init()?;

    // Verify device ID
    let (part_id, revision_id) = sensor.get_device_id()?;
    println!("Device ID: 0x{:X}, Revision: 0x{:X}", part_id, revision_id);

    // Configure sensor settings
    sensor.set_gain(LsGainRange::Gain3x)?;
    sensor.set_resolution(LsResolution::Bits18_100ms)?;
    sensor.set_measurement_rate(LsMeasurementRate::Ms100)?;

    // Enable RGB mode (all channels)
    sensor.enable_rgb_mode(true)?;

    // Enable the sensor
    sensor.enable(true)?;

    println!("Sensor configured and enabled. Starting measurements...");
    println!("Press Ctrl+C to exit\n");

    loop {
        // Wait for measurement to complete
        delay.delay_ms(200);

        // Check if data is ready
        if sensor.is_data_ready()? {
            // Read RGB data
            let rgb = sensor.read_rgb_data()?;

            // Calculate lux
            let lux = sensor.calculate_lux(&rgb)?;

            // Calculate color temperature
            match sensor.calculate_color_temperature(&rgb) {
                Ok(color) => {
                    println!(
                        "R: {:6} G: {:6} B: {:6} IR: {:6} | Lux: {:6.2} | CCT: {:4}K | x: {:.3} y: {:.3}",
                        rgb.red, rgb.green, rgb.blue, rgb.ir,
                        lux, color.cct, color.x, color.y
                    );
                }
                Err(_) => {
                    println!(
                        "R: {:6} G: {:6} B: {:6} IR: {:6} | Lux: {:6.2} | CCT: ----K",
                        rgb.red, rgb.green, rgb.blue, rgb.ir, lux
                    );
                }
            }
        } else {
            println!("Data not ready, waiting...");
        }

        // Wait before next reading
        delay.delay_ms(1000);
    }
}

#[cfg(not(target_os = "linux"))]
fn main() {
    println!("This example requires Linux with I2C support.");
    println!("Please adapt the I2C initialization for your platform.");
}
