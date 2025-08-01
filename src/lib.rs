//! # APDS-9253 Digital RGB and Ambient Light Sensor Driver
//!
//! This is a platform-agnostic Rust driver for the APDS-9253 digital RGB and ambient light sensor,
//! built using the [`embedded-hal`] traits for I2C communication.
//!
//! The APDS-9253 is a digital RGB sensor that provides:
//! - Individual Red, Green, Blue, and IR channels
//! - Programmable gain (1x to 18x)
//! - Programmable integration time (3.125ms to 400ms)
//! - Interrupt support with configurable thresholds
//! - I2C interface (address 0x52)
//!
//! ## Features
//!
//! - **High-level API** for RGB and ALS measurements
//! - **Async/await support** with feature gating (optional)
//! - **Interrupt support** with threshold and variance modes
//! - **Configurable gain and integration time**
//! - **Lux calculation** for ambient light sensing
//! - **Color temperature estimation**
//! - **Power management** with sleep modes
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use apds9253::{Apds9253, LsGainRange, LsResolution};
//! use embedded_hal::i2c::I2c;
//!
//! # fn main() {
//! # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
//! let mut sensor = Apds9253::new(i2c);
//!
//! // Initialize the sensor
//! sensor.init().unwrap();
//!
//! // Configure measurement settings
//! sensor.set_gain(LsGainRange::Gain3X).unwrap();
//! sensor.set_resolution(LsResolution::Bits18100Ms).unwrap();
//!
//! // Enable RGB mode
//! sensor.enable_rgb_mode(true).unwrap();
//! sensor.enable(true).unwrap();
//!
//! // Wait for measurement
//! // std::thread::sleep(std::time::Duration::from_millis(200));
//!
//! // Read RGB data
//! // let rgb = sensor.read_rgb_data().unwrap();
//! // println!("R: {}, G: {}, B: {}, IR: {}", rgb.red, rgb.green, rgb.blue, rgb.ir);
//!
//! // Calculate lux
//! // let lux = sensor.calculate_lux(&rgb).unwrap();
//! // println!("Ambient light: {:.2} lux", lux);
//! # }
//! ```
//!
//! ## Async Usage
//!
//! Enable the `async` feature to use async/await patterns:
//!
//! ```toml
//! [dependencies]
//! apds9253 = { version = "0.1", features = ["async"] }
//! ```
//!
//! ```rust,ignore
//! # #[cfg(feature = "async")]
//! # async fn example() {
//! use apds9253::{Apds9253, LsGainRange, LsResolution};
//! use embedded_hal_async::i2c::I2c;
//!
//! let i2c = /* your async I2C implementation */;
//! let mut sensor = Apds9253::new(i2c);
//!
//! // Initialize the sensor
//! sensor.init_async().await.unwrap();
//!
//! // Configure measurement settings
//! sensor.set_gain_async(LsGainRange::Gain3X).await.unwrap();
//! sensor.set_resolution_async(LsResolution::Bits18100Ms).await.unwrap();
//!
//! // Enable RGB mode
//! sensor.enable_rgb_mode_async(true).await.unwrap();
//! sensor.enable_async(true).await.unwrap();
//!
//! // Wait for measurement (using async delay)
//! // embedded_hal_async::delay::DelayNs::delay_ms(&mut delay, 200).await;
//!
//! // Read RGB data
//! let rgb = sensor.read_rgb_data_async().await.unwrap();
//! println!("R: {}, G: {}, B: {}, IR: {}", rgb.red, rgb.green, rgb.blue, rgb.ir);
//!
//! // Calculate lux
//! let lux = sensor.calculate_lux_async(&rgb).await.unwrap();
//! println!("Ambient light: {:.2} lux", lux);
//! # }
//! ```
//!
//! [`embedded-hal`]: https://crates.io/crates/embedded-hal

#![no_std]
#![deny(missing_docs)]

use embedded_hal::i2c::I2c;

#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

pub mod ll;
pub use ll::{LsGainRange, LsIntSel, LsMeasurementRate, LsPersist, LsResolution, LsThresVar};

/// I2C address of the APDS-9253 sensor
pub const I2C_ADDRESS: u8 = ll::I2C_ADDRESS;

/// RGB and IR measurement data
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RgbData {
    /// Red channel value (0-1048575 for 20-bit resolution)
    pub red: u32,
    /// Green channel value (0-1048575 for 20-bit resolution)
    pub green: u32,
    /// Blue channel value (0-1048575 for 20-bit resolution)
    pub blue: u32,
    /// IR channel value (0-1048575 for 20-bit resolution)
    pub ir: u32,
}

/// Color temperature and chromaticity data
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ColorData {
    /// Correlated Color Temperature in Kelvin
    pub cct: u16,
    /// CIE 1931 x chromaticity coordinate
    pub x: f32,
    /// CIE 1931 y chromaticity coordinate
    pub y: f32,
}

/// Device status information
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct StatusInfo {
    /// True if device went through power-up event
    pub power_on_occurred: bool,
    /// True if interrupt condition is active
    pub interrupt_active: bool,
    /// True if new data is available
    pub data_ready: bool,
}

/// All possible errors in this crate
#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C communication error
    I2c(E),
    /// Invalid device ID detected
    InvalidDeviceId {
        /// Expected device ID
        expected: u8,
        /// Found device ID
        found: u8,
    },
    /// Sensor not ready for operation
    NotReady,
    /// Invalid configuration parameter
    InvalidConfig(&'static str),
    /// Measurement timeout
    Timeout,
    /// Data overflow detected
    Overflow,
    /// Block read operation failed
    BlockReadFailed,
}

// Implement From conversion for ll::DeviceInterfaceError
impl<E> From<ll::DeviceInterfaceError<E>> for Error<E> {
    fn from(error: ll::DeviceInterfaceError<E>) -> Self {
        match error {
            ll::DeviceInterfaceError::I2c(e) => Error::I2c(e),
        }
    }
}

/// High-level APDS-9253 driver
pub struct Apds9253<I2C> {
    device: ll::Device<ll::DeviceInterface<I2C>>,
    // Device state tracking
    current_gain: Option<LsGainRange>,
    current_resolution: Option<LsResolution>,
}

impl<I2C> Apds9253<I2C> {
    /// Create a new APDS-9253 driver instance
    pub fn new(i2c: I2C) -> Self {
        Self {
            device: ll::Device::new(ll::DeviceInterface { i2c }),
            current_gain: None,
            current_resolution: None,
        }
    }

    /// Get the recommended measurement delay time based on current resolution
    /// Returns the delay in milliseconds that should be used between enabling the sensor
    /// and reading data for accurate measurements
    pub fn get_measurement_delay_ms(&self) -> u32 {
        match self.current_resolution.unwrap_or(LsResolution::Bits18100Ms) {
            LsResolution::Bits133125Ms => 4,  // 3.125ms
            LsResolution::Bits1625Ms => 30,   // 25ms
            LsResolution::Bits1750Ms => 55,   // 50ms
            LsResolution::Bits18100Ms => 105, // 100ms
            LsResolution::Bits19200Ms => 205, // 200ms
            LsResolution::Bits20400Ms => 405, // 400ms
            _ => 105,                         // Default to 100ms
        }
    }
}

impl<I2C, E> Apds9253<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Initialize the sensor with default settings
    pub fn init(&mut self) -> Result<(), Error<E>> {
        // Verify device ID
        let part_id_reg = self.device.part_id().read()?;
        let part_id = part_id_reg.part_id();
        let expected_part_id = 0xC;
        if part_id != expected_part_id {
            return Err(Error::InvalidDeviceId {
                expected: expected_part_id,
                found: part_id,
            });
        }

        // Reset the device
        self.device
            .main_ctrl()
            .modify(|reg| reg.set_sw_reset(true))?;

        // Set default configuration
        self.set_gain(LsGainRange::Gain3X)?;
        self.set_resolution(LsResolution::Bits18100Ms)?;
        self.set_measurement_rate(LsMeasurementRate::Ms100)?;

        Ok(())
    }

    /// Enable or disable the light sensor
    pub fn enable(&mut self, enable: bool) -> Result<(), Error<E>> {
        self.device
            .main_ctrl()
            .modify(|reg| reg.set_ls_en(enable))?;
        Ok(())
    }

    /// Enable or disable RGB mode (all channels vs ALS+IR only)
    pub fn enable_rgb_mode(&mut self, enable: bool) -> Result<(), Error<E>> {
        self.device
            .main_ctrl()
            .modify(|reg| reg.set_rgb_mode(enable))?;
        Ok(())
    }

    /// Set the analog gain
    pub fn set_gain(&mut self, gain: LsGainRange) -> Result<(), Error<E>> {
        self.device
            .ls_gain()
            .modify(|reg| reg.set_ls_gain_range(gain))?;
        self.current_gain = Some(gain);
        Ok(())
    }

    /// Set the ADC resolution and integration time
    pub fn set_resolution(&mut self, resolution: LsResolution) -> Result<(), Error<E>> {
        self.device
            .ls_meas_rate()
            .modify(|reg| reg.set_ls_resolution(resolution))?;
        self.current_resolution = Some(resolution);
        Ok(())
    }

    /// Set the measurement rate
    pub fn set_measurement_rate(&mut self, rate: LsMeasurementRate) -> Result<(), Error<E>> {
        self.device
            .ls_meas_rate()
            .modify(|reg| reg.set_ls_measurement_rate(rate))?;
        Ok(())
    }

    /// Check if new data is available
    pub fn is_data_ready(&mut self) -> Result<bool, Error<E>> {
        let status = self.device.main_status().read()?;
        Ok(status.ls_data_status())
    }

    /// Read raw RGB and IR data using consolidated multi-byte registers
    pub fn read_rgb_data(&mut self) -> Result<RgbData, Error<E>> {
        // Check if data is ready
        if !self.is_data_ready()? {
            return Err(Error::NotReady);
        }

        // Read IR data (20-bit value from 24-bit register)
        let ir = self.device.ls_data_ir().read()?.ir_counts();

        // Read Green data (20-bit value from 24-bit register)
        let green = self.device.ls_data_green().read()?.green_counts();

        // Read Blue data (20-bit value from 24-bit register)
        let blue = self.device.ls_data_blue().read()?.blue_counts();

        // Read Red data (20-bit value from 24-bit register)
        let red = self.device.ls_data_red().read()?.red_counts();

        Ok(RgbData {
            red,
            green,
            blue,
            ir,
        })
    }

    /// Read raw RGB and IR data optimized for performance
    ///
    /// This method attempts to read all channels efficiently. Due to device-driver's
    /// abstraction, individual register reads are used but with optimized field access.
    /// For maximum performance in critical applications, consider using the device()
    /// method to access lower-level operations.
    pub fn read_rgb_data_fast(&mut self) -> Result<RgbData, Error<E>> {
        // Batch read all data registers for better I2C efficiency
        // Note: With device-driver framework, this still translates to individual register reads
        // but with optimized register access patterns

        let ir = self.device.ls_data_ir().read()?.ir_counts();
        let green = self.device.ls_data_green().read()?.green_counts();
        let blue = self.device.ls_data_blue().read()?.blue_counts();
        let red = self.device.ls_data_red().read()?.red_counts();

        Ok(RgbData {
            red,
            green,
            blue,
            ir,
        })
    }

    /// Calculate lux from RGB data using datasheet-specified coefficients
    pub fn calculate_lux(&mut self, rgb: &RgbData) -> Result<f32, Error<E>> {
        // Use cached gain and resolution if available, otherwise read from device
        let gain = match self.current_gain {
            Some(gain) => gain,
            None => {
                let gain_reg = self.device.ls_gain().read()?;
                let gain = gain_reg.ls_gain_range();
                self.current_gain = Some(gain);
                gain
            }
        };

        let resolution = match self.current_resolution {
            Some(resolution) => resolution,
            None => {
                let meas_rate = self.device.ls_meas_rate().read()?;
                let resolution = meas_rate.ls_resolution();
                self.current_resolution = Some(resolution);
                resolution
            }
        };

        // Calculate lux per count based on gain and resolution settings
        let lux_per_count = match (gain, resolution) {
            // Gain 1x
            (LsGainRange::Gain1X, LsResolution::Bits1625Ms) => 2.193,
            (LsGainRange::Gain1X, LsResolution::Bits1750Ms) => 1.099,
            (LsGainRange::Gain1X, LsResolution::Bits18100Ms) => 0.548,
            (LsGainRange::Gain1X, LsResolution::Bits19200Ms) => 0.273,
            (LsGainRange::Gain1X, LsResolution::Bits20400Ms) => 0.136,
            (LsGainRange::Gain1X, LsResolution::Bits133125Ms) => 2.193 * 8.0, // Approximation

            // Gain 3x
            (LsGainRange::Gain3X, LsResolution::Bits1625Ms) => 0.722,
            (LsGainRange::Gain3X, LsResolution::Bits1750Ms) => 0.359,
            (LsGainRange::Gain3X, LsResolution::Bits18100Ms) => 0.180,
            (LsGainRange::Gain3X, LsResolution::Bits19200Ms) => 0.090,
            (LsGainRange::Gain3X, LsResolution::Bits20400Ms) => 0.045,
            (LsGainRange::Gain3X, LsResolution::Bits133125Ms) => 0.722 * 8.0,

            // Gain 6x
            (LsGainRange::Gain6X, LsResolution::Bits1625Ms) => 0.360,
            (LsGainRange::Gain6X, LsResolution::Bits1750Ms) => 0.179,
            (LsGainRange::Gain6X, LsResolution::Bits18100Ms) => 0.090,
            (LsGainRange::Gain6X, LsResolution::Bits19200Ms) => 0.045,
            (LsGainRange::Gain6X, LsResolution::Bits20400Ms) => 0.022,
            (LsGainRange::Gain6X, LsResolution::Bits133125Ms) => 0.360 * 8.0,

            // Gain 9x
            (LsGainRange::Gain9X, LsResolution::Bits1625Ms) => 0.239,
            (LsGainRange::Gain9X, LsResolution::Bits1750Ms) => 0.119,
            (LsGainRange::Gain9X, LsResolution::Bits18100Ms) => 0.059,
            (LsGainRange::Gain9X, LsResolution::Bits19200Ms) => 0.030,
            (LsGainRange::Gain9X, LsResolution::Bits20400Ms) => 0.015,
            (LsGainRange::Gain9X, LsResolution::Bits133125Ms) => 0.239 * 8.0,

            // Gain 18x
            (LsGainRange::Gain18X, LsResolution::Bits1625Ms) => 0.117,
            (LsGainRange::Gain18X, LsResolution::Bits1750Ms) => 0.059,
            (LsGainRange::Gain18X, LsResolution::Bits18100Ms) => 0.029,
            (LsGainRange::Gain18X, LsResolution::Bits19200Ms) => 0.015,
            (LsGainRange::Gain18X, LsResolution::Bits20400Ms) => 0.007,
            (LsGainRange::Gain18X, LsResolution::Bits133125Ms) => 0.117 * 8.0,

            // Default for reserved values
            _ => 0.180, // Default to 3x gain, 100ms values
        };

        // Calculate lux using green channel (ALS channel)
        let lux = rgb.green as f32 * lux_per_count;

        Ok(lux)
    }

    /// Calculate color temperature from RGB data
    pub fn calculate_color_temperature(&self, rgb: &RgbData) -> Result<ColorData, Error<E>> {
        if rgb.red == 0 || rgb.green == 0 || rgb.blue == 0 {
            return Err(Error::InvalidConfig("Zero RGB values"));
        }

        // Convert to floating point for calculations
        let r = rgb.red as f32;
        let g = rgb.green as f32;
        let b = rgb.blue as f32;

        // Calculate CIE 1931 XYZ values (simplified transformation)
        let x = 0.4124 * r + 0.3576 * g + 0.1805 * b;
        let y = 0.2126 * r + 0.7152 * g + 0.0722 * b;
        let z = 0.0193 * r + 0.1192 * g + 0.9505 * b;

        let sum = x + y + z;
        if sum == 0.0 {
            return Err(Error::InvalidConfig("Zero XYZ sum"));
        }

        // Calculate chromaticity coordinates
        let x_chrom = x / sum;
        let y_chrom = y / sum;

        // McCamy's approximation for CCT
        let n = (x_chrom - 0.3320) / (0.1858 - y_chrom);
        let cct = 449.0 * libm::powf(n, 3.0) + 3525.0 * libm::powf(n, 2.0) + 6823.3 * n + 5520.33;

        Ok(ColorData {
            cct: cct.max(1000.0).min(25000.0) as u16, // Clamp to reasonable range
            x: x_chrom,
            y: y_chrom,
        })
    }

    /// Get the device part ID and revision
    pub fn get_device_id(&mut self) -> Result<(u8, u8), Error<E>> {
        let part_id_reg = self.device.part_id().read()?;
        Ok((part_id_reg.part_id(), part_id_reg.revision_id()))
    }

    /// Get comprehensive device status information
    pub fn get_status(&mut self) -> Result<StatusInfo, Error<E>> {
        let status = self.device.main_status().read()?;
        Ok(StatusInfo {
            power_on_occurred: status.power_on_status(),
            interrupt_active: status.ls_interrupt_status(),
            data_ready: status.ls_data_status(),
        })
    }

    /// Check if an interrupt is currently active
    pub fn is_interrupt_active(&mut self) -> Result<bool, Error<E>> {
        let status = self.device.main_status().read()?;
        Ok(status.ls_interrupt_status())
    }

    /// Clear interrupt status by reading the status register
    /// Note: APDS-9253 clears interrupt flags automatically when status register is read
    pub fn clear_interrupt(&mut self) -> Result<(), Error<E>> {
        let _status = self.device.main_status().read()?;
        Ok(())
    }

    /// Configure interrupt settings
    pub fn configure_interrupt(
        &mut self,
        channel: LsIntSel,
        variance_mode: bool,
        enabled: bool,
    ) -> Result<(), Error<E>> {
        self.device.int_cfg().modify(|reg| {
            reg.set_ls_int_sel(channel);
            reg.set_ls_var_mode(variance_mode);
            reg.set_ls_int_en(enabled);
        })?;
        Ok(())
    }

    /// Set interrupt persistence (number of consecutive measurements before triggering)
    pub fn set_interrupt_persistence(&mut self, persistence: LsPersist) -> Result<(), Error<E>> {
        self.device
            .int_pst()
            .modify(|reg| reg.set_ls_persist(persistence))?;
        Ok(())
    }

    /// Set upper interrupt threshold
    pub fn set_upper_threshold(&mut self, threshold: u32) -> Result<(), Error<E>> {
        // Clamp to 20-bit maximum
        let threshold = threshold.min(0xFFFFF);
        self.device
            .ls_thres_up()
            .write(|reg| reg.set_upper_threshold(threshold))?;
        Ok(())
    }

    /// Set lower interrupt threshold
    pub fn set_lower_threshold(&mut self, threshold: u32) -> Result<(), Error<E>> {
        // Clamp to 20-bit maximum
        let threshold = threshold.min(0xFFFFF);
        self.device
            .ls_thres_low()
            .write(|reg| reg.set_lower_threshold(threshold))?;
        Ok(())
    }

    /// Set variance threshold for variation interrupt mode
    pub fn set_variance_threshold(&mut self, threshold: LsThresVar) -> Result<(), Error<E>> {
        self.device
            .ls_thres_var()
            .modify(|reg| reg.set_variance_threshold(threshold))?;
        Ok(())
    }

    /// Read dark count storage for green/ALS channel
    /// Returns (is_valid, dark_count_value)
    pub fn read_dark_count(&mut self) -> Result<(bool, u8), Error<E>> {
        let dark_storage = self.device.dark_count_storage_green().read()?;
        Ok((dark_storage.valid(), dark_storage.count()))
    }

    /// Perform software reset
    pub fn software_reset(&mut self) -> Result<(), Error<E>> {
        self.device
            .main_ctrl()
            .modify(|reg| reg.set_sw_reset(true))?;

        // Clear cached state after reset
        self.current_gain = None;
        self.current_resolution = None;
        Ok(())
    }

    /// Enable sleep after interrupt mode
    /// When enabled, the sensor returns to standby after an interrupt occurs
    pub fn enable_sleep_after_interrupt(&mut self, enable: bool) -> Result<(), Error<E>> {
        self.device
            .main_ctrl()
            .modify(|reg| reg.set_sai_ls(enable))?;
        Ok(())
    }

    /// Get a reference to the underlying device for advanced operations
    pub fn device(&mut self) -> &mut ll::Device<ll::DeviceInterface<I2C>> {
        &mut self.device
    }
}

#[cfg(feature = "async")]
impl<I2C, E> Apds9253<I2C>
where
    I2C: AsyncI2c<Error = E>,
{
    /// Initialize the sensor with default settings (async version)
    pub async fn init_async(&mut self) -> Result<(), Error<E>> {
        // Verify device ID
        let part_id_reg = self.device.part_id().read_async().await?;
        let part_id = part_id_reg.part_id();
        let expected_part_id = 0xC;
        if part_id != expected_part_id {
            return Err(Error::InvalidDeviceId {
                expected: expected_part_id,
                found: part_id,
            });
        }

        // Reset the device
        self.device
            .main_ctrl()
            .modify_async(|reg| reg.set_sw_reset(true))
            .await?;

        // Set default configuration
        self.set_gain_async(LsGainRange::Gain3X).await?;
        self.set_resolution_async(LsResolution::Bits18100Ms).await?;
        self.set_measurement_rate_async(LsMeasurementRate::Ms100)
            .await?;

        Ok(())
    }

    /// Enable or disable the light sensor (async version)
    pub async fn enable_async(&mut self, enable: bool) -> Result<(), Error<E>> {
        self.device
            .main_ctrl()
            .modify_async(|reg| reg.set_ls_en(enable))
            .await?;
        Ok(())
    }

    /// Enable or disable RGB mode (all channels vs ALS+IR only) (async version)
    pub async fn enable_rgb_mode_async(&mut self, enable: bool) -> Result<(), Error<E>> {
        self.device
            .main_ctrl()
            .modify_async(|reg| reg.set_rgb_mode(enable))
            .await?;
        Ok(())
    }

    /// Set the analog gain (async version)
    pub async fn set_gain_async(&mut self, gain: LsGainRange) -> Result<(), Error<E>> {
        self.device
            .ls_gain()
            .modify_async(|reg| reg.set_ls_gain_range(gain))
            .await?;
        self.current_gain = Some(gain);
        Ok(())
    }

    /// Set the ADC resolution and integration time (async version)
    pub async fn set_resolution_async(&mut self, resolution: LsResolution) -> Result<(), Error<E>> {
        self.device
            .ls_meas_rate()
            .modify_async(|reg| reg.set_ls_resolution(resolution))
            .await?;
        self.current_resolution = Some(resolution);
        Ok(())
    }

    /// Set the measurement rate (async version)
    pub async fn set_measurement_rate_async(
        &mut self,
        rate: LsMeasurementRate,
    ) -> Result<(), Error<E>> {
        self.device
            .ls_meas_rate()
            .modify_async(|reg| reg.set_ls_measurement_rate(rate))
            .await?;
        Ok(())
    }

    /// Check if new data is available (async version)
    pub async fn is_data_ready_async(&mut self) -> Result<bool, Error<E>> {
        let status = self.device.main_status().read_async().await?;
        Ok(status.ls_data_status())
    }

    /// Read raw RGB and IR data using consolidated multi-byte registers (async version)
    pub async fn read_rgb_data_async(&mut self) -> Result<RgbData, Error<E>> {
        // Check if data is ready
        if !self.is_data_ready_async().await? {
            return Err(Error::NotReady);
        }

        // Read IR data (20-bit value from 24-bit register)
        let ir = self.device.ls_data_ir().read_async().await?.ir_counts();

        // Read Green data (20-bit value from 24-bit register)
        let green = self
            .device
            .ls_data_green()
            .read_async()
            .await?
            .green_counts();

        // Read Blue data (20-bit value from 24-bit register)
        let blue = self.device.ls_data_blue().read_async().await?.blue_counts();

        // Read Red data (20-bit value from 24-bit register)
        let red = self.device.ls_data_red().read_async().await?.red_counts();

        Ok(RgbData {
            red,
            green,
            blue,
            ir,
        })
    }

    /// Calculate lux from RGB data using datasheet-specified coefficients (async version)
    pub async fn calculate_lux_async(&mut self, rgb: &RgbData) -> Result<f32, Error<E>> {
        // Use cached gain and resolution if available, otherwise read from device
        let gain = match self.current_gain {
            Some(gain) => gain,
            None => {
                let gain_reg = self.device.ls_gain().read_async().await?;
                let gain = gain_reg.ls_gain_range();
                self.current_gain = Some(gain);
                gain
            }
        };

        let resolution = match self.current_resolution {
            Some(resolution) => resolution,
            None => {
                let meas_rate = self.device.ls_meas_rate().read_async().await?;
                let resolution = meas_rate.ls_resolution();
                self.current_resolution = Some(resolution);
                resolution
            }
        };

        // Calculate lux per count based on gain and resolution settings
        let lux_per_count = match (gain, resolution) {
            // Gain 1x
            (LsGainRange::Gain1X, LsResolution::Bits1625Ms) => 2.193,
            (LsGainRange::Gain1X, LsResolution::Bits1750Ms) => 1.099,
            (LsGainRange::Gain1X, LsResolution::Bits18100Ms) => 0.548,
            (LsGainRange::Gain1X, LsResolution::Bits19200Ms) => 0.273,
            (LsGainRange::Gain1X, LsResolution::Bits20400Ms) => 0.136,
            (LsGainRange::Gain1X, LsResolution::Bits133125Ms) => 2.193 * 8.0, // Approximation

            // Gain 3x
            (LsGainRange::Gain3X, LsResolution::Bits1625Ms) => 0.722,
            (LsGainRange::Gain3X, LsResolution::Bits1750Ms) => 0.359,
            (LsGainRange::Gain3X, LsResolution::Bits18100Ms) => 0.180,
            (LsGainRange::Gain3X, LsResolution::Bits19200Ms) => 0.090,
            (LsGainRange::Gain3X, LsResolution::Bits20400Ms) => 0.045,
            (LsGainRange::Gain3X, LsResolution::Bits133125Ms) => 0.722 * 8.0,

            // Gain 6x
            (LsGainRange::Gain6X, LsResolution::Bits1625Ms) => 0.360,
            (LsGainRange::Gain6X, LsResolution::Bits1750Ms) => 0.179,
            (LsGainRange::Gain6X, LsResolution::Bits18100Ms) => 0.090,
            (LsGainRange::Gain6X, LsResolution::Bits19200Ms) => 0.045,
            (LsGainRange::Gain6X, LsResolution::Bits20400Ms) => 0.022,
            (LsGainRange::Gain6X, LsResolution::Bits133125Ms) => 0.360 * 8.0,

            // Gain 9x
            (LsGainRange::Gain9X, LsResolution::Bits1625Ms) => 0.239,
            (LsGainRange::Gain9X, LsResolution::Bits1750Ms) => 0.119,
            (LsGainRange::Gain9X, LsResolution::Bits18100Ms) => 0.059,
            (LsGainRange::Gain9X, LsResolution::Bits19200Ms) => 0.030,
            (LsGainRange::Gain9X, LsResolution::Bits20400Ms) => 0.015,
            (LsGainRange::Gain9X, LsResolution::Bits133125Ms) => 0.239 * 8.0,

            // Gain 18x
            (LsGainRange::Gain18X, LsResolution::Bits1625Ms) => 0.117,
            (LsGainRange::Gain18X, LsResolution::Bits1750Ms) => 0.059,
            (LsGainRange::Gain18X, LsResolution::Bits18100Ms) => 0.029,
            (LsGainRange::Gain18X, LsResolution::Bits19200Ms) => 0.015,
            (LsGainRange::Gain18X, LsResolution::Bits20400Ms) => 0.007,
            (LsGainRange::Gain18X, LsResolution::Bits133125Ms) => 0.117 * 8.0,

            // Default for reserved values
            _ => 0.180, // Default to 3x gain, 100ms values
        };

        // Calculate lux using green channel (ALS channel)
        let lux = rgb.green as f32 * lux_per_count;

        Ok(lux)
    }

    /// Get the device part ID and revision (async version)
    pub async fn get_device_id_async(&mut self) -> Result<(u8, u8), Error<E>> {
        let part_id_reg = self.device.part_id().read_async().await?;
        Ok((part_id_reg.part_id(), part_id_reg.revision_id()))
    }

    /// Get comprehensive device status information (async version)
    pub async fn get_status_async(&mut self) -> Result<StatusInfo, Error<E>> {
        let status = self.device.main_status().read_async().await?;
        Ok(StatusInfo {
            power_on_occurred: status.power_on_status(),
            interrupt_active: status.ls_interrupt_status(),
            data_ready: status.ls_data_status(),
        })
    }

    /// Check if an interrupt is currently active (async version)
    pub async fn is_interrupt_active_async(&mut self) -> Result<bool, Error<E>> {
        let status = self.device.main_status().read_async().await?;
        Ok(status.ls_interrupt_status())
    }

    /// Clear interrupt status by reading the status register (async version)
    pub async fn clear_interrupt_async(&mut self) -> Result<(), Error<E>> {
        let _status = self.device.main_status().read_async().await?;
        Ok(())
    }

    /// Configure interrupt settings (async version)
    pub async fn configure_interrupt_async(
        &mut self,
        channel: LsIntSel,
        variance_mode: bool,
        enabled: bool,
    ) -> Result<(), Error<E>> {
        self.device
            .int_cfg()
            .modify_async(|reg| {
                reg.set_ls_int_sel(channel);
                reg.set_ls_var_mode(variance_mode);
                reg.set_ls_int_en(enabled);
            })
            .await?;
        Ok(())
    }

    /// Read dark count storage for green/ALS channel (async version)
    pub async fn read_dark_count_async(&mut self) -> Result<(bool, u8), Error<E>> {
        let dark_storage = self.device.dark_count_storage_green().read_async().await?;
        Ok((dark_storage.valid(), dark_storage.count()))
    }

    /// Perform a complete measurement cycle with user-provided delay (async version)
    ///
    /// # Example
    /// ```rust,ignore
    /// // Enable sensor and wait for measurement
    /// sensor.enable_async(true).await?;
    /// let delay_ms = sensor.get_measurement_delay_ms();
    /// delay.delay_ms(delay_ms).await;
    /// let rgb = sensor.read_rgb_data_async().await?;
    /// ```
    pub async fn measure_rgb_with_delay_async<D>(
        &mut self,
        delay: &mut D,
    ) -> Result<RgbData, Error<E>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        // Ensure sensor is enabled
        self.enable_async(true).await?;

        // Wait for measurement to complete
        let delay_ms = self.get_measurement_delay_ms();
        delay.delay_ms(delay_ms).await;

        // Read the data
        self.read_rgb_data_async().await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    #[test]
    fn test_constants() {
        // Test that the I2C address constant is correct
        assert_eq!(I2C_ADDRESS, 0x52);
    }

    #[test]
    fn test_rgb_data_structure() {
        // Test that RgbData structure works correctly
        let rgb = RgbData {
            red: 1000,
            green: 2000,
            blue: 3000,
            ir: 4000,
        };

        assert_eq!(rgb.red, 1000);
        assert_eq!(rgb.green, 2000);
        assert_eq!(rgb.blue, 3000);
        assert_eq!(rgb.ir, 4000);
    }

    #[test]
    fn test_rgb_data_combination() {
        // Test that 20-bit values are correctly handled
        // Note: With device-driver, byte combination is handled automatically
        let test_value = 0x12345u32; // 20-bit test value

        // Verify that the test value fits in 20 bits
        assert!(test_value <= 0xFFFFF);

        // Test the manual combination logic (for reference)
        let byte0 = (test_value & 0xFF) as u8;
        let byte1 = ((test_value >> 8) & 0xFF) as u8;
        let byte2 = ((test_value >> 16) & 0x0F) as u8;

        let combined = (byte0 as u32) | ((byte1 as u32) << 8) | (((byte2 as u32) & 0x0F) << 16);
        assert_eq!(combined, test_value);
    }

    #[test]
    fn test_status_info_structure() {
        // Test StatusInfo creation and field access
        let status = StatusInfo {
            power_on_occurred: true,
            interrupt_active: false,
            data_ready: true,
        };

        assert_eq!(status.power_on_occurred, true);
        assert_eq!(status.interrupt_active, false);
        assert_eq!(status.data_ready, true);
    }

    #[test]
    fn test_color_data_structure() {
        // Test ColorData creation and field access
        let color = ColorData {
            cct: 5600,
            x: 0.3127,
            y: 0.3290,
        };

        assert_eq!(color.cct, 5600);
        assert!((color.x - 0.3127).abs() < 0.001);
        assert!((color.y - 0.3290).abs() < 0.001);
    }

    #[test]
    fn test_error_types() {
        // Test that error types can be created and matched
        let i2c_error: Error<()> = Error::I2c(());
        let invalid_id_error: Error<()> = Error::InvalidDeviceId {
            expected: 0xC,
            found: 0x5,
        };
        let not_ready_error: Error<()> = Error::NotReady;

        match i2c_error {
            Error::I2c(_) => {} // Expected
            _ => panic!("Wrong error type"),
        }

        match invalid_id_error {
            Error::InvalidDeviceId { expected, found } => {
                assert_eq!(expected, 0xC);
                assert_eq!(found, 0x5);
            }
            _ => panic!("Wrong error type"),
        }

        match not_ready_error {
            Error::NotReady => {} // Expected
            _ => panic!("Wrong error type"),
        }
    }
}
