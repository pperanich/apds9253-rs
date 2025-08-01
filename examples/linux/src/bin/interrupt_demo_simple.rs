//! Interrupt-based sensor reading example
//!
//! This example demonstrates how to:
//! - Configure threshold-based interrupts
//! - Configure variance-based interrupts  
//! - Handle interrupt events efficiently
//! - Use Sleep After Interrupt (SAI) mode
//! - Monitor different interrupt sources

use apds9253::{
    Apds9253, LsGainRange, LsIntSel, LsMeasurementRate, LsPersist, LsResolution, LsThresVar,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use std::io::{self, Write};

// This example uses linux-embedded-hal for demonstration
#[cfg(target_os = "linux")]
use linux_embedded_hal::{Delay, I2cdev};

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize I2C interface
    let i2c = I2cdev::new("/dev/i2c-1")?;
    let delay = Delay;

    // Create sensor instance
    let mut sensor = Apds9253::new_with_delay(i2c, delay);

    println!("🚨 APDS-9253 Interrupt Demo");
    println!("===========================");

    // Initialize the sensor
    println!("Initializing sensor...");
    sensor.init()?;

    // Configure sensor settings
    sensor.set_gain(LsGainRange::Gain3x)?;
    sensor.set_resolution(LsResolution::Bits18_100ms)?;
    sensor.set_measurement_rate(LsMeasurementRate::Ms100)?;

    // Enable RGB mode
    sensor.enable_rgb_mode(true)?;
    sensor.enable(true)?;

    println!("✅ Sensor initialized and enabled");
    println!();

    // Show interrupt configuration menu
    show_interrupt_menu();

    let interrupt_type = get_user_choice()?;

    match interrupt_type {
        1 => run_threshold_interrupt_demo(&mut sensor)?,
        2 => run_variance_interrupt_demo(&mut sensor)?,
        3 => run_multi_channel_demo(&mut sensor)?,
        4 => run_sai_demo(&mut sensor)?,
        _ => {
            println!("Invalid choice, running threshold demo...");
            run_threshold_interrupt_demo(&mut sensor)?;
        }
    }

    Ok(())
}

#[cfg(target_os = "linux")]
fn show_interrupt_menu() {
    println!("Select interrupt demo type:");
    println!("1. Threshold-based interrupts (light level changes)");
    println!("2. Variance-based interrupts (detect rapid changes)");
    println!("3. Multi-channel interrupt comparison");
    println!("4. Sleep After Interrupt (SAI) mode demo");
    print!("Enter choice (1-4): ");
    io::stdout().flush().unwrap();
}

#[cfg(target_os = "linux")]
fn get_user_choice() -> Result<u32, Box<dyn std::error::Error>> {
    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    Ok(input.trim().parse().unwrap_or(1))
}

#[cfg(target_os = "linux")]
fn run_threshold_interrupt_demo(
    sensor: &mut Apds9253<I2cdev, Delay>,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("🎯 Threshold Interrupt Demo");
    println!("============================");

    // Take a baseline reading to set intelligent thresholds
    std::thread::sleep(std::time::Duration::from_millis(200));
    let baseline = sensor.read_rgb_data()?;
    let baseline_lux = sensor.calculate_lux(&baseline)?;

    // Set thresholds at ±30% of baseline green channel
    let low_threshold = (baseline.green * 70) / 100;
    let high_threshold = (baseline.green * 130) / 100;

    println!("📊 Baseline reading:");
    println!(
        "   Green: {} counts ({:.2} lux)",
        baseline.green, baseline_lux
    );
    println!(
        "   Setting thresholds: {} - {} counts",
        low_threshold, high_threshold
    );

    // Configure threshold interrupt on green channel (ALS)
    sensor.configure_threshold_interrupt(
        LsIntSel::Green,
        low_threshold,
        high_threshold,
        LsPersist::Consecutive2, // Require 2 consecutive readings to avoid noise
    )?;

    println!("✅ Threshold interrupt configured:");
    println!("   - Source: Green channel (ALS)");
    println!("   - Low threshold: {} counts", low_threshold);
    println!("   - High threshold: {} counts", high_threshold);
    println!("   - Persistence: 2 consecutive readings");
    println!();
    println!("💡 Try covering/uncovering the sensor or changing lighting...");
    println!("Press Ctrl+C to exit");
    println!();

    let mut reading_count = 0;
    let mut interrupt_count = 0;

    loop {
        std::thread::sleep(std::time::Duration::from_millis(150));

        if sensor.is_data_ready()? {
            reading_count += 1;
            let rgb = sensor.read_rgb_data()?;
            let lux = sensor.calculate_lux(&rgb)?;

            // Check interrupt status
            let interrupt_active = sensor.check_interrupt()?;

            if interrupt_active {
                interrupt_count += 1;

                // Determine which threshold was crossed
                let threshold_type = if rgb.green < low_threshold {
                    "LOW"
                } else if rgb.green > high_threshold {
                    "HIGH"
                } else {
                    "EDGE" // Edge case - might be clearing
                };

                println!(
                    "🚨 INTERRUPT #{:2} [{}]: Green: {:6} ({:8.2} lux) | Change: {:+6.1}%",
                    interrupt_count,
                    threshold_type,
                    rgb.green,
                    lux,
                    ((rgb.green as f32 - baseline.green as f32) / baseline.green as f32) * 100.0
                );

                // Clear the interrupt
                sensor.clear_interrupt()?;
            } else {
                // Show periodic status
                if reading_count % 10 == 0 {
                    println!(
                        "📈 Reading #{:3}: Green: {:6} ({:8.2} lux) | Normal operation",
                        reading_count, rgb.green, lux
                    );
                }
            }
        }
    }
}

#[cfg(target_os = "linux")]
fn run_variance_interrupt_demo(
    sensor: &mut Apds9253<I2cdev, Delay>,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("📊 Variance Interrupt Demo");
    println!("===========================");

    // Configure variance interrupt on green channel
    // This will trigger when the reading changes by more than the specified amount
    sensor.configure_variance_interrupt(
        LsIntSel::Green,
        LsThresVar::Counts128, // Trigger on changes > 128 counts
        LsPersist::Every,      // Trigger immediately (no persistence)
    )?;

    println!("✅ Variance interrupt configured:");
    println!("   - Source: Green channel (ALS)");
    println!("   - Variance threshold: 128 counts");
    println!("   - Persistence: Every measurement");
    println!();
    println!("💡 Try making quick movements near the sensor...");
    println!("Press Ctrl+C to exit");
    println!();

    let mut reading_count = 0;
    let mut interrupt_count = 0;
    let mut last_green = 0u32;

    loop {
        std::thread::sleep(std::time::Duration::from_millis(100)); // Faster sampling for variance

        if sensor.is_data_ready()? {
            reading_count += 1;
            let rgb = sensor.read_rgb_data()?;
            let lux = sensor.calculate_lux(&rgb)?;

            // Check interrupt status
            let interrupt_active = sensor.check_interrupt()?;

            if interrupt_active {
                interrupt_count += 1;

                let change = if last_green > 0 {
                    rgb.green as i32 - last_green as i32
                } else {
                    0
                };

                println!(
                    "⚡ VARIANCE INT #{:2}: Green: {:6} ({:8.2} lux) | Change: {:+6} counts",
                    interrupt_count, rgb.green, lux, change
                );

                // Clear the interrupt
                sensor.clear_interrupt()?;
            } else if reading_count % 20 == 0 {
                println!(
                    "📊 Reading #{:3}: Green: {:6} ({:8.2} lux) | Stable",
                    reading_count, rgb.green, lux
                );
            }

            last_green = rgb.green;
        }
    }
}

#[cfg(target_os = "linux")]
fn run_multi_channel_demo(
    sensor: &mut Apds9253<I2cdev, Delay>,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("🌈 Multi-Channel Interrupt Demo");
    println!("================================");

    println!("This demo will cycle through different interrupt sources:");
    println!("IR → Green → Red → Blue → (repeat)");
    println!();

    let channels = [
        (LsIntSel::IR, "IR", "infrared"),
        (LsIntSel::Green, "Green", "ambient light"),
        (LsIntSel::Red, "Red", "red light"),
        (LsIntSel::Blue, "Blue", "blue light"),
    ];

    for (i, &(channel, name, description)) in channels.iter().cycle().enumerate() {
        if i >= 20 {
            break;
        } // Limit demo duration

        println!("🔄 Switching to {} channel ({})...", name, description);

        // Take baseline reading
        std::thread::sleep(std::time::Duration::from_millis(200));
        let baseline = sensor.read_rgb_data()?;

        let baseline_value = match channel {
            LsIntSel::IR => baseline.ir,
            LsIntSel::Green => baseline.green,
            LsIntSel::Red => baseline.red,
            LsIntSel::Blue => baseline.blue,
        };

        // Set thresholds at ±25% of baseline
        let low_threshold = (baseline_value * 75) / 100;
        let high_threshold = (baseline_value * 125) / 100;

        sensor.configure_threshold_interrupt(
            channel,
            low_threshold,
            high_threshold,
            LsPersist::Consecutive2,
        )?;

        println!(
            "   Baseline: {} counts, Thresholds: {} - {}",
            baseline_value, low_threshold, high_threshold
        );

        // Monitor this channel for 5 seconds
        let start_time = std::time::Instant::now();
        let mut channel_interrupts = 0;

        while start_time.elapsed().as_secs() < 5 {
            std::thread::sleep(std::time::Duration::from_millis(100));

            if sensor.is_data_ready()? {
                let rgb = sensor.read_rgb_data()?;

                if sensor.check_interrupt()? {
                    channel_interrupts += 1;

                    let current_value = match channel {
                        LsIntSel::IR => rgb.ir,
                        LsIntSel::Green => rgb.green,
                        LsIntSel::Red => rgb.red,
                        LsIntSel::Blue => rgb.blue,
                    };

                    println!("   🚨 {} interrupt: {} counts", name, current_value);
                    sensor.clear_interrupt()?;
                }
            }
        }

        println!(
            "   ✅ {} channel: {} interrupts in 5 seconds",
            name, channel_interrupts
        );
        println!();
    }

    println!("🏁 Multi-channel demo completed!");
    Ok(())
}

#[cfg(target_os = "linux")]
fn run_sai_demo(sensor: &mut Apds9253<I2cdev, Delay>) -> Result<(), Box<dyn std::error::Error>> {
    println!("😴 Sleep After Interrupt (SAI) Demo");
    println!("====================================");

    // Take baseline reading
    std::thread::sleep(std::time::Duration::from_millis(200));
    let baseline = sensor.read_rgb_data()?;

    // Configure interrupt with wide thresholds to demonstrate SAI
    let low_threshold = baseline.green / 2;
    let high_threshold = baseline.green * 2;

    sensor.configure_threshold_interrupt(
        LsIntSel::Green,
        low_threshold,
        high_threshold,
        LsPersist::Every,
    )?;

    // Enable Sleep After Interrupt
    sensor.enable_sleep_after_interrupt(true)?;

    println!("✅ SAI mode configured:");
    println!("   - Sensor will sleep after each interrupt");
    println!("   - Must manually re-enable after interrupt");
    println!("   - Useful for low-power applications");
    println!();
    println!("💡 Try changing lighting conditions...");
    println!("Press Ctrl+C to exit");
    println!();

    let mut interrupt_count = 0;

    loop {
        std::thread::sleep(std::time::Duration::from_millis(200));

        // Check if sensor is still enabled (will be disabled after interrupt in SAI mode)
        let status = sensor.get_status()?;

        if status.interrupt_active {
            interrupt_count += 1;

            // Read data before clearing interrupt
            let rgb = sensor.read_rgb_data()?;
            let lux = sensor.calculate_lux(&rgb)?;

            println!(
                "😴 SAI INTERRUPT #{}: Green: {:6} ({:8.2} lux) - Sensor going to sleep...",
                interrupt_count, rgb.green, lux
            );

            // Clear interrupt
            sensor.clear_interrupt()?;

            // In SAI mode, sensor automatically goes to standby
            // We need to re-enable it for the next measurement
            println!("   ⏰ Waiting 3 seconds before re-enabling sensor...");
            std::thread::sleep(std::time::Duration::from_millis(3000));

            sensor.enable(true)?;
            println!("   ✅ Sensor re-enabled, ready for next interrupt");
            println!();
        } else if status.data_ready {
            // Normal reading (no interrupt)
            let rgb = sensor.read_rgb_data()?;
            let lux = sensor.calculate_lux(&rgb)?;

            println!(
                "📊 Normal reading: Green: {:6} ({:8.2} lux)",
                rgb.green, lux
            );
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn main() {
    println!("🚨 APDS-9253 Interrupt Demo");
    println!("===========================");
    println!();
    println!("This example requires Linux with I2C support.");
    println!("To adapt for your platform:");
    println!("1. Replace linux-embedded-hal with your platform's HAL");
    println!("2. Update the I2C initialization code");
    println!("3. Consider using GPIO interrupts for hardware interrupt handling");
    println!();
    println!("Hardware interrupt setup (optional):");
    println!("- Connect APDS-9253 INT pin to a GPIO input");
    println!("- Configure GPIO interrupt on falling edge");
    println!("- Use interrupt handler to wake from sleep or trigger readings");
    println!();
    println!("Example interrupt handler pattern:");
    println!("```rust");
    println!("// In interrupt handler");
    println!("static INTERRUPT_FLAG: AtomicBool = AtomicBool::new(false);");
    println!("fn interrupt_handler() {{");
    println!("    INTERRUPT_FLAG.store(true, Ordering::Relaxed);");
    println!("}}");
    println!();
    println!("// In main loop");
    println!("if INTERRUPT_FLAG.load(Ordering::Relaxed) {{");
    println!("    INTERRUPT_FLAG.store(false, Ordering::Relaxed);");
    println!("    handle_sensor_interrupt(&mut sensor);");
    println!("}}");
    println!("```");
}
