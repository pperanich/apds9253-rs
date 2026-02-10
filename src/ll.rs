//! Low-level register and interface definitions for APDS-9253

use embedded_hal::i2c::I2c;

/// I2C address of the APDS-9253
pub const I2C_ADDRESS: u8 = 0x52;

/// Device interface error types
#[derive(Debug)]
pub enum DeviceInterfaceError<I2cError> {
    /// I2C communication error
    I2c(I2cError),
}

// Allow missing docs for generated device code
#[allow(missing_docs)]
mod device_generated {
    device_driver::create_device!(
        device_name: Device,
        manifest: "device.yaml"
    );
}
pub use device_generated::*;

/// Device interface implementation
#[derive(Debug)]
pub struct DeviceInterface<I2c> {
    /// The I2C interface
    pub i2c: I2c,
}

impl<I2cTrait: I2c> device_driver::RegisterInterface for DeviceInterface<I2cTrait> {
    type AddressType = u8;
    type Error = DeviceInterfaceError<I2cTrait::Error>;

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c
            .write_read(I2C_ADDRESS, &[address], data)
            .map_err(DeviceInterfaceError::I2c)
    }

    fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        // Max register size is 24 bits (3 bytes) + 1 address byte = 4 bytes
        let mut buf = [0u8; 4];
        buf[0] = address;
        let len = data.len();
        buf[1..1 + len].copy_from_slice(data);
        self.i2c
            .write(I2C_ADDRESS, &buf[..1 + len])
            .map_err(DeviceInterfaceError::I2c)
    }
}

#[cfg(feature = "async")]
impl<I2cTrait: embedded_hal_async::i2c::I2c> device_driver::AsyncRegisterInterface
    for DeviceInterface<I2cTrait>
{
    type AddressType = u8;
    type Error = DeviceInterfaceError<I2cTrait::Error>;

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c
            .write_read(I2C_ADDRESS, &[address], data)
            .await
            .map_err(DeviceInterfaceError::I2c)
    }

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        // Max register size is 24 bits (3 bytes) + 1 address byte = 4 bytes
        let mut buf = [0u8; 4];
        buf[0] = address;
        let len = data.len();
        buf[1..1 + len].copy_from_slice(data);
        self.i2c
            .write(I2C_ADDRESS, &buf[..1 + len])
            .await
            .map_err(DeviceInterfaceError::I2c)
    }
}
