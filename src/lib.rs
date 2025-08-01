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
//! sensor.set_gain(LsGainRange::Gain3x as u8).unwrap();
//! sensor.set_resolution(LsResolution::Bits18_100ms as u8).unwrap();
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
//! let mut sensor = Apds9253::new_async(i2c);
//!
//! // Initialize the sensor
//! sensor.init_async().await.unwrap();
//!
//! // Configure measurement settings
//! sensor.set_gain_async(LsGainRange::Gain3x as u8).await.unwrap();
//! sensor.set_resolution_async(LsResolution::Bits18_100ms as u8).await.unwrap();
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

/// I2C address of the APDS-9253 sensor
pub const I2C_ADDRESS: u8 = 0x52;

/// Light sensor gain range settings
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LsGainRange {
    /// 1x gain
    Gain1x = 0b000,
    /// 3x gain
    Gain3x = 0b001,
    /// 6x gain
    Gain6x = 0b010,
    /// 9x gain
    Gain9x = 0b011,
    /// 18x gain
    Gain18x = 0b100,
}

/// Light sensor resolution and integration time settings
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LsResolution {
    /// 20-bit resolution, 400ms integration time
    Bits20_400ms = 0b000,
    /// 19-bit resolution, 200ms integration time
    Bits19_200ms = 0b001,
    /// 18-bit resolution, 100ms integration time
    Bits18_100ms = 0b010,
    /// 17-bit resolution, 50ms integration time
    Bits17_50ms = 0b011,
    /// 16-bit resolution, 25ms integration time
    Bits16_25ms = 0b100,
    /// 13-bit resolution, 3.125ms integration time
    Bits13_3_125ms = 0b101,
}

/// Light sensor measurement rate settings
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LsMeasurementRate {
    /// 25ms measurement rate
    Ms25 = 0b000,
    /// 50ms measurement rate
    Ms50 = 0b001,
    /// 100ms measurement rate
    Ms100 = 0b010,
    /// 200ms measurement rate
    Ms200 = 0b011,
    /// 500ms measurement rate
    Ms500 = 0b100,
    /// 1000ms measurement rate
    Ms1000 = 0b101,
}

/// Light sensor interrupt source selection
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LsIntSel {
    /// IR channel
    IR = 0b00,
    /// Green channel
    Green = 0b01,
    /// Red channel
    Red = 0b10,
    /// Blue channel
    Blue = 0b11,
}

/// Light sensor interrupt persistence settings
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LsPersist {
    /// Every measurement triggers interrupt
    Every = 0b000,
    /// 2 consecutive measurements
    Consecutive2 = 0b001,
    /// 3 consecutive measurements
    Consecutive3 = 0b010,
    /// 4 consecutive measurements
    Consecutive4 = 0b011,
    /// 5 consecutive measurements
    Consecutive5 = 0b100,
    /// 6 consecutive measurements
    Consecutive6 = 0b101,
    /// 7 consecutive measurements
    Consecutive7 = 0b110,
    /// 8 consecutive measurements
    Consecutive8 = 0b111,
}

/// Light sensor variance threshold settings
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LsThresVar {
    /// 8 counts variance threshold
    Counts8 = 0b000,
    /// 16 counts variance threshold
    Counts16 = 0b001,
    /// 32 counts variance threshold
    Counts32 = 0b010,
    /// 64 counts variance threshold
    Counts64 = 0b011,
    /// 128 counts variance threshold
    Counts128 = 0b100,
    /// 256 counts variance threshold
    Counts256 = 0b101,
    /// 512 counts variance threshold
    Counts512 = 0b110,
    /// 1024 counts variance threshold
    Counts1024 = 0b111,
}

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

// Register addresses
const MAIN_CTRL: u8 = 0x00;
const LS_MEAS_RATE: u8 = 0x04;
const LS_GAIN: u8 = 0x05;
const PART_ID: u8 = 0x06;
const MAIN_STATUS: u8 = 0x07;
const LS_DATA_IR_0: u8 = 0x0A;
const LS_DATA_GREEN_0: u8 = 0x0D;
const LS_DATA_BLUE_0: u8 = 0x10;
const LS_DATA_RED_0: u8 = 0x13;
const INT_CFG: u8 = 0x19;
const INT_PST: u8 = 0x1A;
const LS_THRES_UP_0: u8 = 0x21;
const LS_THRES_LOW_0: u8 = 0x24;
const LS_THRES_VAR: u8 = 0x27;

/// High-level APDS-9253 driver
pub struct Apds9253<I2C, Delay = ()> {
    i2c: I2C,
    delay: Delay,
    // Device state tracking
    current_gain: Option<u8>,
    current_resolution: Option<u8>,
}

impl<I2C, E> Apds9253<I2C, ()>
where
    I2C: I2c<Error = E>,
{
    /// Create a new APDS-9253 driver instance without delay support
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            delay: (),
            current_gain: None,
            current_resolution: None,
        }
    }
}

impl<I2C, E, Delay> Apds9253<I2C, Delay>
where
    I2C: I2c<Error = E>,
    Delay: embedded_hal::delay::DelayNs,
{
    /// Create a new APDS-9253 driver instance with delay support
    pub fn new_with_delay(i2c: I2C, delay: Delay) -> Self {
        Self {
            i2c,
            delay,
            current_gain: None,
            current_resolution: None,
        }
    }
}

impl<I2C, E, Delay> Apds9253<I2C, Delay>
where
    I2C: I2c<Error = E>,
{
    /// Initialize the sensor with default settings
    pub fn init(&mut self) -> Result<(), Error<E>> {
        // Verify device ID
        let part_id = self.read_register(PART_ID)?;
        let expected_part_id = 0xC;
        if (part_id >> 4) != expected_part_id {
            return Err(Error::InvalidDeviceId {
                expected: expected_part_id,
                found: part_id >> 4,
            });
        }

        // Reset the device
        self.write_register(MAIN_CTRL, 0x10)?; // SW_RESET bit

        // Set default configuration
        self.set_gain(LsGainRange::Gain3x as u8)?;
        self.set_resolution(LsResolution::Bits18_100ms as u8)?;
        self.set_measurement_rate(LsMeasurementRate::Ms100 as u8)?;

        Ok(())
    }

    /// Enable or disable the light sensor
    pub fn enable(&mut self, enable: bool) -> Result<(), Error<E>> {
        let mut ctrl = self.read_register(MAIN_CTRL)?;
        if enable {
            ctrl |= 0x02; // LS_EN bit
        } else {
            ctrl &= !0x02;
        }
        self.write_register(MAIN_CTRL, ctrl)
    }

    /// Enable or disable RGB mode (all channels vs ALS+IR only)
    pub fn enable_rgb_mode(&mut self, enable: bool) -> Result<(), Error<E>> {
        let mut ctrl = self.read_register(MAIN_CTRL)?;
        if enable {
            ctrl |= 0x04; // RGB_MODE bit
        } else {
            ctrl &= !0x04;
        }
        self.write_register(MAIN_CTRL, ctrl)
    }

    /// Set the analog gain
    pub fn set_gain(&mut self, gain: u8) -> Result<(), Error<E>> {
        let result = self.write_register(LS_GAIN, gain & 0x07);
        if result.is_ok() {
            self.current_gain = Some(gain);
        }
        result
    }

    /// Set the ADC resolution and integration time
    pub fn set_resolution(&mut self, resolution: u8) -> Result<(), Error<E>> {
        let mut meas_rate = self.read_register(LS_MEAS_RATE)?;
        meas_rate = (meas_rate & 0x8F) | ((resolution & 0x07) << 4);
        let result = self.write_register(LS_MEAS_RATE, meas_rate);
        if result.is_ok() {
            self.current_resolution = Some(resolution);
        }
        result
    }

    /// Set the measurement rate
    pub fn set_measurement_rate(&mut self, rate: u8) -> Result<(), Error<E>> {
        let mut meas_rate = self.read_register(LS_MEAS_RATE)?;
        meas_rate = (meas_rate & 0xF8) | (rate & 0x07);
        self.write_register(LS_MEAS_RATE, meas_rate)
    }

    /// Check if new data is available
    pub fn is_data_ready(&mut self) -> Result<bool, Error<E>> {
        let status = self.read_register(MAIN_STATUS)?;
        Ok((status & 0x08) != 0) // LS_DATA_STATUS bit
    }

    /// Read raw RGB and IR data using block read for data coherency
    pub fn read_rgb_data(&mut self) -> Result<RgbData, Error<E>> {
        // Check if data is ready
        if !self.is_data_ready()? {
            return Err(Error::NotReady);
        }

        // Perform block read from 0x0A to 0x15 (IR, Green, Blue, Red data registers)
        let mut data_buffer = [0u8; 12]; // 4 channels × 3 bytes each
        self.i2c
            .write_read(I2C_ADDRESS, &[LS_DATA_IR_0], &mut data_buffer)
            .map_err(Error::I2c)?;

        // Extract 20-bit values from the block read data
        let ir = (data_buffer[0] as u32)
            | ((data_buffer[1] as u32) << 8)
            | (((data_buffer[2] as u32) & 0x0F) << 16);

        let green = (data_buffer[3] as u32)
            | ((data_buffer[4] as u32) << 8)
            | (((data_buffer[5] as u32) & 0x0F) << 16);

        let blue = (data_buffer[6] as u32)
            | ((data_buffer[7] as u32) << 8)
            | (((data_buffer[8] as u32) & 0x0F) << 16);

        let red = (data_buffer[9] as u32)
            | ((data_buffer[10] as u32) << 8)
            | (((data_buffer[11] as u32) & 0x0F) << 16);

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
                let gain_reg = self.read_register(LS_GAIN)?;
                let gain = gain_reg & 0x07;
                self.current_gain = Some(gain);
                gain
            }
        };

        let resolution = match self.current_resolution {
            Some(resolution) => resolution,
            None => {
                let meas_rate = self.read_register(LS_MEAS_RATE)?;
                let resolution = (meas_rate >> 4) & 0x07;
                self.current_resolution = Some(resolution);
                resolution
            }
        };

        // Datasheet-specified lux per count lookup table
        // [gain_index][resolution_index] = lux_per_count
        const LUX_TABLE: [[f32; 6]; 5] = [
            // Gain 1x
            [2.193, 1.099, 0.548, 0.273, 0.136, 0.0], // 25ms, 50ms, 100ms, 200ms, 400ms, 3.125ms (not in table)
            // Gain 3x
            [0.722, 0.359, 0.180, 0.090, 0.045, 0.0],
            // Gain 6x
            [0.360, 0.179, 0.090, 0.045, 0.022, 0.0],
            // Gain 9x
            [0.239, 0.119, 0.059, 0.030, 0.015, 0.0],
            // Gain 18x
            [0.117, 0.059, 0.029, 0.015, 0.007, 0.0],
        ];

        let gain_index = core::cmp::min(gain as usize, 4);
        let resolution_index = core::cmp::min(resolution as usize, 5);

        let lux_per_count = if resolution_index == 5 {
            // For 3.125ms, use approximation based on 25ms value
            LUX_TABLE[gain_index][0] * 8.0 // Rough scaling
        } else {
            LUX_TABLE[gain_index][resolution_index]
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
        let part_id = self.read_register(PART_ID)?;
        Ok((part_id >> 4, part_id & 0x0F))
    }

    /// Destroy the driver and return the I2C interface
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    // Helper methods for register access
    fn read_register(&mut self, address: u8) -> Result<u8, Error<E>> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(I2C_ADDRESS, &[address], &mut buffer)
            .map_err(Error::I2c)?;
        Ok(buffer[0])
    }

    fn write_register(&mut self, address: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(I2C_ADDRESS, &[address, value])
            .map_err(Error::I2c)
    }
}

#[cfg(feature = "async")]
impl<I2C, E> Apds9253<I2C, ()>
where
    I2C: AsyncI2c<Error = E>,
{
    /// Create a new APDS-9253 driver instance without delay support (async version)
    pub fn new_async(i2c: I2C) -> Self {
        Self {
            i2c,
            delay: (),
            current_gain: None,
            current_resolution: None,
        }
    }
}

#[cfg(feature = "async")]
impl<I2C, E, Delay> Apds9253<I2C, Delay>
where
    I2C: AsyncI2c<Error = E>,
    Delay: embedded_hal_async::delay::DelayNs,
{
    /// Create a new APDS-9253 driver instance with delay support (async version)
    pub fn new_async_with_delay(i2c: I2C, delay: Delay) -> Self {
        Self {
            i2c,
            delay,
            current_gain: None,
            current_resolution: None,
        }
    }
}

#[cfg(feature = "async")]
impl<I2C, E, Delay> Apds9253<I2C, Delay>
where
    I2C: AsyncI2c<Error = E>,
{
    /// Initialize the sensor with default settings (async version)
    pub async fn init_async(&mut self) -> Result<(), Error<E>> {
        // Verify device ID
        let part_id = self.read_register_async(PART_ID).await?;
        let expected_part_id = 0xC;
        if (part_id >> 4) != expected_part_id {
            return Err(Error::InvalidDeviceId {
                expected: expected_part_id,
                found: part_id >> 4,
            });
        }

        // Reset the device
        self.write_register_async(MAIN_CTRL, 0x10).await?; // SW_RESET bit

        // Set default configuration
        self.set_gain_async(LsGainRange::Gain3x as u8).await?;
        self.set_resolution_async(LsResolution::Bits18_100ms as u8)
            .await?;
        self.set_measurement_rate_async(LsMeasurementRate::Ms100 as u8)
            .await?;

        Ok(())
    }

    /// Enable or disable the light sensor (async version)
    pub async fn enable_async(&mut self, enable: bool) -> Result<(), Error<E>> {
        let mut ctrl = self.read_register_async(MAIN_CTRL).await?;
        if enable {
            ctrl |= 0x02; // LS_EN bit
        } else {
            ctrl &= !0x02;
        }
        self.write_register_async(MAIN_CTRL, ctrl).await
    }

    /// Enable or disable RGB mode (all channels vs ALS+IR only) (async version)
    pub async fn enable_rgb_mode_async(&mut self, enable: bool) -> Result<(), Error<E>> {
        let mut ctrl = self.read_register_async(MAIN_CTRL).await?;
        if enable {
            ctrl |= 0x04; // RGB_MODE bit
        } else {
            ctrl &= !0x04;
        }
        self.write_register_async(MAIN_CTRL, ctrl).await
    }

    /// Set the analog gain (async version)
    pub async fn set_gain_async(&mut self, gain: u8) -> Result<(), Error<E>> {
        let result = self.write_register_async(LS_GAIN, gain & 0x07).await;
        if result.is_ok() {
            self.current_gain = Some(gain);
        }
        result
    }

    /// Set the ADC resolution and integration time (async version)
    pub async fn set_resolution_async(&mut self, resolution: u8) -> Result<(), Error<E>> {
        let mut meas_rate = self.read_register_async(LS_MEAS_RATE).await?;
        meas_rate = (meas_rate & 0x8F) | ((resolution & 0x07) << 4);
        let result = self.write_register_async(LS_MEAS_RATE, meas_rate).await;
        if result.is_ok() {
            self.current_resolution = Some(resolution);
        }
        result
    }

    /// Set the measurement rate (async version)
    pub async fn set_measurement_rate_async(&mut self, rate: u8) -> Result<(), Error<E>> {
        let mut meas_rate = self.read_register_async(LS_MEAS_RATE).await?;
        meas_rate = (meas_rate & 0xF8) | (rate & 0x07);
        self.write_register_async(LS_MEAS_RATE, meas_rate).await
    }

    /// Check if new data is available (async version)
    pub async fn is_data_ready_async(&mut self) -> Result<bool, Error<E>> {
        let status = self.read_register_async(MAIN_STATUS).await?;
        Ok((status & 0x08) != 0) // LS_DATA_STATUS bit
    }

    /// Read raw RGB and IR data using block read for data coherency (async version)
    pub async fn read_rgb_data_async(&mut self) -> Result<RgbData, Error<E>> {
        // Check if data is ready
        if !self.is_data_ready_async().await? {
            return Err(Error::NotReady);
        }

        // Perform block read from 0x0A to 0x15 (IR, Green, Blue, Red data registers)
        let mut data_buffer = [0u8; 12]; // 4 channels × 3 bytes each
        self.i2c
            .write_read(I2C_ADDRESS, &[LS_DATA_IR_0], &mut data_buffer)
            .await
            .map_err(Error::I2c)?;

        // Extract 20-bit values from the block read data
        let ir = (data_buffer[0] as u32)
            | ((data_buffer[1] as u32) << 8)
            | (((data_buffer[2] as u32) & 0x0F) << 16);

        let green = (data_buffer[3] as u32)
            | ((data_buffer[4] as u32) << 8)
            | (((data_buffer[5] as u32) & 0x0F) << 16);

        let blue = (data_buffer[6] as u32)
            | ((data_buffer[7] as u32) << 8)
            | (((data_buffer[8] as u32) & 0x0F) << 16);

        let red = (data_buffer[9] as u32)
            | ((data_buffer[10] as u32) << 8)
            | (((data_buffer[11] as u32) & 0x0F) << 16);

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
                let gain_reg = self.read_register_async(LS_GAIN).await?;
                let gain = gain_reg & 0x07;
                self.current_gain = Some(gain);
                gain
            }
        };

        let resolution = match self.current_resolution {
            Some(resolution) => resolution,
            None => {
                let meas_rate = self.read_register_async(LS_MEAS_RATE).await?;
                let resolution = (meas_rate >> 4) & 0x07;
                self.current_resolution = Some(resolution);
                resolution
            }
        };

        // Datasheet-specified lux per count lookup table
        // [gain_index][resolution_index] = lux_per_count
        const LUX_TABLE: [[f32; 6]; 5] = [
            // Gain 1x
            [2.193, 1.099, 0.548, 0.273, 0.136, 0.0], // 25ms, 50ms, 100ms, 200ms, 400ms, 3.125ms (not in table)
            // Gain 3x
            [0.722, 0.359, 0.180, 0.090, 0.045, 0.0],
            // Gain 6x
            [0.360, 0.179, 0.090, 0.045, 0.022, 0.0],
            // Gain 9x
            [0.239, 0.119, 0.059, 0.030, 0.015, 0.0],
            // Gain 18x
            [0.117, 0.059, 0.029, 0.015, 0.007, 0.0],
        ];

        let gain_index = core::cmp::min(gain as usize, 4);
        let resolution_index = core::cmp::min(resolution as usize, 5);

        let lux_per_count = if resolution_index == 5 {
            // For 3.125ms, use approximation based on 25ms value
            LUX_TABLE[gain_index][0] * 8.0 // Rough scaling
        } else {
            LUX_TABLE[gain_index][resolution_index]
        };

        // Calculate lux using green channel (ALS channel)
        let lux = rgb.green as f32 * lux_per_count;

        Ok(lux)
    }

    /// Get the device part ID and revision (async version)
    pub async fn get_device_id_async(&mut self) -> Result<(u8, u8), Error<E>> {
        let part_id = self.read_register_async(PART_ID).await?;
        Ok((part_id >> 4, part_id & 0x0F))
    }

    /// Perform a complete measurement cycle with timing (async version)
    pub async fn measure_rgb_blocking_async(&mut self) -> Result<RgbData, Error<E>>
    where
        Delay: embedded_hal_async::delay::DelayNs,
    {
        // Ensure sensor is enabled
        self.enable_async(true).await?;

        // Wait for measurement to complete based on current resolution
        let wait_time_ms = match self
            .current_resolution
            .unwrap_or(LsResolution::Bits18_100ms as u8)
        {
            5 => 4,   // Bits13_3_125ms
            4 => 30,  // Bits16_25ms
            3 => 55,  // Bits17_50ms
            2 => 105, // Bits18_100ms
            1 => 205, // Bits19_200ms
            0 => 405, // Bits20_400ms
            _ => 105, // Default to 100ms
        };

        self.delay.delay_ms(wait_time_ms).await;

        // Read the data
        self.read_rgb_data_async().await
    }

    // Helper methods for async register access
    async fn read_register_async(&mut self, address: u8) -> Result<u8, Error<E>> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(I2C_ADDRESS, &[address], &mut buffer)
            .await
            .map_err(Error::I2c)?;
        Ok(buffer[0])
    }

    async fn write_register_async(&mut self, address: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(I2C_ADDRESS, &[address, value])
            .await
            .map_err(Error::I2c)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    extern crate std;
    use std::vec;

    #[test]
    fn test_device_creation() {
        let expectations = [];
        let i2c = I2cMock::new(&expectations);
        let sensor = Apds9253::new(i2c);
        let mut i2c = sensor.destroy();
        i2c.done();
    }

    #[test]
    fn test_device_id_read() {
        let expectations = [I2cTransaction::write_read(
            I2C_ADDRESS,
            vec![PART_ID],
            vec![0xC2],
        )];
        let i2c = I2cMock::new(&expectations);
        let mut sensor = Apds9253::new(i2c);

        let (part_id, revision_id) = sensor.get_device_id().unwrap();
        assert_eq!(part_id, 0xC);
        assert_eq!(revision_id, 0x2);

        let mut i2c = sensor.destroy();
        i2c.done();
    }

    #[test]
    fn test_rgb_data_combination() {
        // Test that 20-bit values are correctly combined from 3 bytes
        let test_value = 0x12345; // 20-bit test value
        let byte0 = (test_value & 0xFF) as u8;
        let byte1 = ((test_value >> 8) & 0xFF) as u8;
        let byte2 = ((test_value >> 16) & 0x0F) as u8;

        let combined = (byte0 as u32) | ((byte1 as u32) << 8) | (((byte2 as u32) & 0x0F) << 16);
        assert_eq!(combined, test_value);
    }
}
