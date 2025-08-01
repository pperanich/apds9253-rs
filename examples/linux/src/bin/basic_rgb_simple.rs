//! Basic RGB sensor reading example
//!
//! This example demonstrates how to:
//! - Initialize the APDS-9253 sensor
//! - Configure gain and integration time
//! - Read RGB and IR values
//! - Calculate lux and color temperature
//! - Handle errors gracefully

use apds9253::{Apds9253, LsGainRange, LsMeasurementRate, LsResolution};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

// This example uses linux-embedded-hal for demonstration
// Replace with your platform's I2C implementation
#[cfg(target_os = "linux")]
use linux_embedded_hal::{Delay, I2cdev};

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize I2C interface
    let i2c = I2cdev::new("/dev/i2c-1")?;
    let mut delay = Delay;

    // Create sensor instance with delay support for blocking operations
    let mut sensor = Apds9253::new_with_delay(i2c, delay);

    println!("🌈 APDS-9253 Basic RGB Reading Example");
    println!("=====================================");

    // Initialize the sensor
    println!("Initializing sensor...");
    sensor.init()?;

    // Verify device ID
    let (part_id, revision_id) = sensor.get_device_id()?;
    println!(
        "✅ Device ID: 0x{:X}, Revision: 0x{:X}",
        part_id, revision_id
    );

    // Configure sensor settings for optimal RGB reading
    println!("Configuring sensor settings...");
    sensor.set_gain(LsGainRange::Gain3x)?; // 3x gain for good sensitivity
    sensor.set_resolution(LsResolution::Bits18_100ms)?; // 18-bit resolution, 100ms integration
    sensor.set_measurement_rate(LsMeasurementRate::Ms100)?; // 100ms measurement rate

    // Enable RGB mode (all channels: R, G, B, IR)
    sensor.enable_rgb_mode(true)?;

    // Enable the sensor
    sensor.enable(true)?;

    println!("✅ Sensor configured:");
    println!("   - Gain: 3x");
    println!("   - Resolution: 18-bit (100ms integration)");
    println!("   - Measurement rate: 100ms");
    println!("   - RGB mode: Enabled (all channels active)");
    println!();

    println!("Starting measurements... (Press Ctrl+C to exit)");
    println!("┌─────────┬─────────┬─────────┬─────────┬──────────┬─────────┬─────────┐");
    println!("│   Red   │  Green  │  Blue   │   IR    │   Lux    │  CCT(K) │ Quality │");
    println!("├─────────┼─────────┼─────────┼─────────┼──────────┼─────────┼─────────┤");

    let mut reading_count = 0;
    let mut error_count = 0;

    loop {
        // Wait for measurement to complete
        std::thread::sleep(std::time::Duration::from_millis(150));

        match take_measurement(&mut sensor) {
            Ok((rgb, lux, color_info)) => {
                reading_count += 1;

                // Display the measurement
                println!(
                    "│ {:7} │ {:7} │ {:7} │ {:7} │ {:8.2} │ {:7} │ {:7} │",
                    rgb.red, rgb.green, rgb.blue, rgb.ir, lux, color_info.0, color_info.1
                );

                // Show detailed analysis every 10 readings
                if reading_count % 10 == 0 {
                    show_detailed_analysis(&rgb, lux, &color_info);
                }
            }
            Err(e) => {
                error_count += 1;
                println!("│ Error reading sensor: {:50} │", format!("{:?}", e));

                // If too many errors, try to reinitialize
                if error_count > 5 {
                    println!("Too many errors, attempting to reinitialize sensor...");
                    if let Err(init_err) = sensor.init() {
                        println!("Failed to reinitialize: {:?}", init_err);
                        break;
                    }
                    error_count = 0;
                }
            }
        }
    }

    Ok(())
}

#[cfg(target_os = "linux")]
fn take_measurement(
    sensor: &mut Apds9253<I2cdev, Delay>,
) -> Result<(apds9253::RgbData, f32, (String, String)), Box<dyn std::error::Error>> {
    // Check if data is ready
    if !sensor.is_data_ready()? {
        return Err("Data not ready".into());
    }

    // Read RGB data using block read for data coherency
    let rgb = sensor.read_rgb_data()?;

    // Calculate lux from the measurement
    let lux = sensor.calculate_lux(&rgb)?;

    // Calculate color temperature and quality assessment
    let (cct_str, quality) = match sensor.calculate_color_temperature(&rgb) {
        Ok(color) => {
            let quality = assess_color_quality(&rgb, color.cct);
            (format!("{:4}", color.cct), quality)
        }
        Err(_) => ("----".to_string(), "Poor".to_string()),
    };

    Ok((rgb, lux, (cct_str, quality)))
}

#[cfg(target_os = "linux")]
fn assess_color_quality(rgb: &apds9253::RgbData, cct: u16) -> String {
    // Simple quality assessment based on signal strength and color temperature range
    let max_channel = rgb.red.max(rgb.green).max(rgb.blue);
    let min_channel = rgb.red.min(rgb.green).min(rgb.blue);

    // Check for saturation (assuming 18-bit resolution)
    let max_18bit = (1 << 18) - 1;
    if max_channel > (max_18bit * 95) / 100 {
        return "Sat".to_string(); // Saturated
    }

    // Check for very low light
    if max_channel < 100 {
        return "Low".to_string();
    }

    // Check color temperature range (typical indoor/outdoor range)
    if cct < 2000 || cct > 10000 {
        return "Edge".to_string(); // Edge case
    }

    // Check channel balance (good color measurement should have reasonable ratios)
    if min_channel > 0 && (max_channel / min_channel) > 50 {
        return "Unbal".to_string(); // Unbalanced
    }

    "Good".to_string()
}

#[cfg(target_os = "linux")]
fn show_detailed_analysis(rgb: &apds9253::RgbData, lux: f32, color_info: &(String, String)) {
    println!("├─────────┼─────────┼─────────┼─────────┼──────────┼─────────┼─────────┤");
    println!(
        "│ 📊 Detailed Analysis (Reading #{})                                    │",
        ""
    );

    // Channel ratios
    let total = rgb.red + rgb.green + rgb.blue;
    if total > 0 {
        let r_pct = (rgb.red * 100) / total;
        let g_pct = (rgb.green * 100) / total;
        let b_pct = (rgb.blue * 100) / total;
        println!(
            "│ RGB Ratios: R:{:2}% G:{:2}% B:{:2}%                                      │",
            r_pct, g_pct, b_pct
        );
    }

    // Light level assessment
    let light_level = if lux < 1.0 {
        "Very Dark"
    } else if lux < 10.0 {
        "Dark"
    } else if lux < 100.0 {
        "Dim"
    } else if lux < 1000.0 {
        "Normal"
    } else if lux < 10000.0 {
        "Bright"
    } else {
        "Very Bright"
    };

    println!(
        "│ Light Level: {:12} | IR/Visible Ratio: {:6.2}                    │",
        light_level,
        if total > 0 {
            rgb.ir as f32 / total as f32
        } else {
            0.0
        }
    );

    println!("├─────────┼─────────┼─────────┼─────────┼──────────┼─────────┼─────────┤");
}

#[cfg(not(target_os = "linux"))]
fn main() {
    println!("🌈 APDS-9253 Basic RGB Reading Example");
    println!("=====================================");
    println!();
    println!("This example requires Linux with I2C support.");
    println!("To adapt for your platform:");
    println!("1. Replace linux-embedded-hal with your platform's HAL");
    println!("2. Update the I2C initialization code");
    println!("3. Ensure your platform supports the embedded-hal traits");
    println!();
    println!("Example adaptations:");
    println!("- STM32: use stm32f4xx-hal or similar");
    println!("- ESP32: use esp-idf-hal");
    println!("- Raspberry Pi: use rppal or linux-embedded-hal");
    println!("- Arduino: use arduino-hal (with avr-hal)");
}
