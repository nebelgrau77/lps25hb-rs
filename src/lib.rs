//! TO DO: CHECK REGISTER NAMES, VALUES ETC. TO ADAPT FROM LPS22HB TO LPS25HB
//! 
//! 
//! A platform agnostic driver to interface with LPS25HB pressure sensor module.
//! 
//! 
//!

#![no_std]
//#![deny(warnings, missing_docs)]

pub mod sensor;
use sensor::SensorSettings;

pub mod register;
use register::{Registers, Bitmasks};

pub mod interface;
use interface::{Interface};

/// Sensor's ID 
const WHOAMI: u8 = 0b10111101; // decimal value 189

/// The output of the temperature sensor must be divided by 100, see p. 10 of the datasheet.
const TEMP_SCALE: f32 = 100.0;
/// The output of the pressure sensor must be divided by 4096, see p. 10 of the datasheet.
const PRESS_SCALE: f32 = 4096.0;

/// LPS22HB init struct.
/// Use this struct to configure sensors and init LPS22HB with an interface of your choice.
pub struct LPS25HBInit {
    pub sensor: SensorSettings,    
}

impl Default for LPS25HBInit {
    fn default() -> Self {
        Self {
            sensor: SensorSettings::default(),            
        }
    }
}

impl LPS25HBInit {
    /// Constructs a new LPS25HB driver instance with a I2C or SPI peripheral.
    ///
    /// # Arguments
    /// * `interface` - `SpiInterface` or `I2cInterface`
    pub fn with_interface<T>(self, interface: T) -> LPS22HB<T>
    where
        T: Interface,
    {
        LPS25HB {
            interface,
            sensor: self.sensor,            
        }
    }
}

/// LPS25HB sensor
pub struct LPS25HB<T>
where
    T: Interface,
{
    interface: T,
    sensor: SensorSettings,    
}

impl<T> LPS25HB<T>
where
    T: Interface,
{   
    /// Verifies communication with WHO_AM_I register    
    pub fn sensor_is_reachable(&mut self) -> Result<bool, T::Error> {
        let mut bytes = [0u8; 1];
        let (who_am_i, register) = (WHOAMI, Registers::WHO_AM_I.addr());        
        self.interface.read(register, &mut bytes)?;
        Ok(bytes[0] == who_am_i)
    }

    /// Initializes the sensor with selected settings
    pub fn begin_sensor(&mut self) -> Result <(), T::Error> {        
        self.interface.write(
            Registers::CTRL_REG1.addr(),
            self.sensor.ctrl_reg1(),
        )?;         
        self.interface.write(
            Registers::CTRL_REG2.addr(),
            self.sensor.ctrl_reg2(),
        )?;
        Ok(())
    }    

    /// Raw sensor reading (3 bytes of pressure data and 2 bytes of temperature data)
    fn read_sensor_raw(&mut self) -> Result<(i32, i32), T::Error> {
        let mut data = [0u8;5];
        self.interface.read(Registers::PRESS_OUT_XL.addr(), &mut data)?;
        let p: i32 = (data[2] as i32) << 16 | (data[1] as i32) << 8 | (data[0] as i32);
        let t: i32 = (data[4] as i32) << 8 | (data[3] as i32);
        Ok((p, t))
    }

    /// Calculated pressure reading in hPa
    pub fn read_pressure(&mut self) -> Result<f32, T::Error> {
        let (p,_t) = self.read_sensor_raw()?;
        let pressure: f32 = (p as f32) / PRESS_SCALE;
        Ok(pressure)
    }

    /// Calculated temperaure reading in degrees Celsius 
    pub fn read_temperature(&mut self) -> Result<f32, T::Error> {
        let (_p,t) = self.read_sensor_raw()?;
        let temperature: f32 = (t as f32) / TEMP_SCALE;
        Ok(temperature)
    }
    
    /// Clear selected bits using a bitmask
    fn clear_register_bit_flag(&mut self, address: Registers, bitmask: Bitmasks) -> Result<(), T::Error> {
        let mut reg_data = [0u8;1];
        self.interface.read(address.addr(), &mut reg_data)?;
        let bitmask = bitmask.bitmask();
        let payload: u8 = reg_data[0] & !bitmask;
        self.interface.write(            
            address.addr(),
            payload,
        )?;
        Ok(())
    }    

    /// Set selected bits using a bitmask
    fn set_register_bit_flag(&mut self, address: Registers, bitmask: Bitmasks) -> Result<(), T::Error> {
        let mut reg_data = [0u8;1];
        self.interface.read(address.addr(), &mut reg_data)?;
        let payload: u8 = reg_data[0] | bitmask.bitmask();
        self.interface.write(            
            address.addr(),
            payload,
        )?;
        Ok(())
    }

    /// Enable single shot data acquisition (self cleared by hardware)
    pub fn enable_one_shot(&mut self) -> Result<(), T::Error> {
        self.set_register_bit_flag(Registers::CTRL_REG2, Bitmasks::ONE_SHOT)?;
        Ok(())
    }
    

}
