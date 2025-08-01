//! Automatic gain control example
//!
//! This example demonstrates how to:
//! - Implement automatic gain control
//! - Optimize sensor readings for different light conditions
//! - Handle sensor saturation and low-light conditions

use apds9253::{Apds9253, LsGainRange, LsResolution, RgbData};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

// This example uses linux-embedded-hal for demonstration
use linux_embedded_hal::{Delay, I2cdev};

const MAX_COUNT_18BIT: u32 = (1 << 18) - 1; // Maximum value for 18-bit ADC
const SATURATION_THRESHOLD: u32 = (MAX_COUNT_18BIT * 90) / 100; // 90% of max
const LOW_LIGHT_THRESHOLD: u32 = MAX_COUNT_18BIT / 100; // 1% of max

struct AutoGainController {
    current_gain: LsGainRange,
    stable_readings: u8,
    min_stable_readings: u8,
}

impl AutoGainController {
    fn new() -> Self {
        Self {
            current_gain: LsGainRange::Gain3x,
            stable_readings: 0,
            min_stable_readings: 3,
        }
    }

    fn update_gain(
        &mut self,
        sensor: &mut Apds9253<impl I2c>,
        rgb: &RgbData,
    ) -> Result<bool, Box<dyn std::error::Error>> {
        // Find the maximum channel value
        let max_channel = rgb.red.max(rgb.green).max(rgb.blue);

        let mut gain_changed = false;

        // Check for saturation - reduce gain
        if max_channel > SATURATION_THRESHOLD {
            let new_gain = match self.current_gain {
                LsGainRange::Gain18x => LsGainRange::Gain9x,
                LsGainRange::Gain9x => LsGainRange::Gain6x,
                LsGainRange::Gain6x => LsGainRange::Gain3x,
                LsGainRange::Gain3x => LsGainRange::Gain1x,
                LsGainRange::Gain1x => LsGainRange::Gain1x, // Already at minimum
            };

            if new_gain as u8 != self.current_gain as u8 {
                println!(
                    "📉 Reducing gain: {:?} -> {:?} (saturation detected: {})",
                    self.current_gain, new_gain, max_channel
                );
                sensor.set_gain(new_gain)?;
                self.current_gain = new_gain;
                self.stable_readings = 0;
                gain_changed = true;
            }
        }
        // Check for low light - increase gain (only if readings have been stable)
        else if max_channel < LOW_LIGHT_THRESHOLD
            && self.stable_readings >= self.min_stable_readings
        {
            let new_gain = match self.current_gain {
                LsGainRange::Gain1x => LsGainRange::Gain3x,
                LsGainRange::Gain3x => LsGainRange::Gain6x,
                LsGainRange::Gain6x => LsGainRange::Gain9x,
                LsGainRange::Gain9x => LsGainRange::Gain18x,
                LsGainRange::Gain18x => LsGainRange::Gain18x, // Already at maximum
            };

            if new_gain as u8 != self.current_gain as u8 {
                println!(
                    "📈 Increasing gain: {:?} -> {:?} (low light detected: {})",
                    self.current_gain, new_gain, max_channel
                );
                sensor.set_gain(new_gain)?;
                self.current_gain = new_gain;
                self.stable_readings = 0;
                gain_changed = true;
            }
        }

        // Track stable readings
        if !gain_changed {
            self.stable_readings = self.stable_readings.saturating_add(1);
        }

        Ok(gain_changed)
    }

    fn get_current_gain(&self) -> LsGainRange {
        self.current_gain
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize I2C interface
    let i2c = I2cdev::new("/dev/i2c-1")?;
    let mut delay = Delay;

    // Create sensor instance
    let mut sensor = Apds9253::new(i2c);
    let mut agc = AutoGainController::new();

    println!("Initializing APDS-9253 sensor with automatic gain control...");

    // Initialize the sensor
    sensor.init()?;

    // Configure sensor settings
    sensor.set_resolution(LsResolution::Bits18_100ms)?;
    sensor.set_gain(agc.get_current_gain())?;

    // Enable RGB mode
    sensor.enable_rgb_mode(true)?;
    sensor.enable(true)?;

    println!("Sensor initialized with automatic gain control");
    println!("- Resolution: 18-bit (100ms integration)");
    println!("- Initial gain: {:?}", agc.get_current_gain());
    println!("- Saturation threshold: {} counts", SATURATION_THRESHOLD);
    println!("- Low light threshold: {} counts", LOW_LIGHT_THRESHOLD);
    println!("\nStarting automatic gain control...");
    println!("Press Ctrl+C to exit\n");

    let mut reading_count = 0;

    loop {
        // Wait for measurement
        delay.delay_ms(200);

        // Check if data is ready
        if sensor.is_data_ready()? {
            reading_count += 1;

            // Read RGB data
            let rgb = sensor.read_rgb_data()?;

            // Update automatic gain control
            let gain_changed = agc.update_gain(&mut sensor, &rgb)?;

            // Calculate lux with current settings
            let lux = sensor.calculate_lux(&rgb)?;

            // Find max channel for display
            let max_channel = rgb.red.max(rgb.green).max(rgb.blue);
            let saturation_percent = (max_channel * 100) / MAX_COUNT_18BIT;

            // Display reading with gain info
            if gain_changed {
                println!(
                    "Reading #{:3}: Max: {:6} ({:2}%) | Lux: {:8.2} | Gain: {:?} ⚡",
                    reading_count,
                    max_channel,
                    saturation_percent,
                    lux,
                    agc.get_current_gain()
                );

                // Wait extra time after gain change for sensor to settle
                delay.delay_ms(300);
            } else {
                println!(
                    "Reading #{:3}: Max: {:6} ({:2}%) | Lux: {:8.2} | Gain: {:?}",
                    reading_count,
                    max_channel,
                    saturation_percent,
                    lux,
                    agc.get_current_gain()
                );
            }

            // Show detailed RGB values every 10 readings
            if reading_count % 10 == 0 {
                println!(
                    "   Detailed: R: {:6} G: {:6} B: {:6} IR: {:6}",
                    rgb.red, rgb.green, rgb.blue, rgb.ir
                );
            }
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
