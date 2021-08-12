//! TO DO: CHECK REGISTER NAMES, VALUES ETC. TO ADAPT FROM LPS22HB TO LPS25HB
//! 
//! IT'S WORKING BUT THE TEMPERATURE READINGS ARE OFF, AROUND 122
//! 
//!  
//! NOTE: I2C MULTIBYTE READ
//! In order to read multiple bytes incrementing the register address, it is necessary to assert
//! the most significant bit of the sub-address field. In other words, SUB(7) must be equal to 1
//! while SUB(6-0) represents the address of the first register to be read.
//! 
//! The I2C embedded in the LPS25HB behaves like a slave device and the following protocol
//! must be adhered to. After the start condition (ST) a slave address is sent, once a slave
//! acknowledge (SAK) has been returned, an 8-bit sub-address (SUB) will be transmitted: the 7
//! LSB represents the actual register address while the MSB enables address auto increment.
//! If the MSb of the SUB field is ‘1’, the SUB (register address) will be automatically increased
//! to allow multiple data read/write.
//! 
//! 
//! A platform agnostic driver to interface with LPS25HB pressure sensor module.
//! 
//! 
//!

#![no_std]
//#![deny(warnings, missing_docs)]

pub mod sensor;
use sensor::*;

pub mod register;
use register::*;

pub mod fifo;
use fifo::*;

pub mod config;
use config::*;

pub mod interface;
use interface::Interface;

/// Sensor's ID 
const WHOAMI: u8 = 0b10111101; // decimal value 189

// https://www.st.com/resource/en/technical_note/dm00242307-how-to-interpret-pressure-and-temperature-readings-in-the-lps25hb-pressure-sensor-stmicroelectronics.pdf

/// The output of the temperature sensor must be divided by 480, see Table 3 of the datasheet.
const TEMP_SCALE: f32 = 480.0;
/// An offset value must be added to the result. This is NOT mentioned in the LPS25HB datasheet, but is described in the LPS25H datasheet.
const TEMP_OFFSET: f32 = 42.5;
/// The output of the pressure sensor must be divided by 4096, see Table 3 of the datasheet.
const PRESS_SCALE: f32 = 4096.0;

/*
/// LPS22HB init struct.
/// Use this struct to configure sensors and init LPS25HB with an interface of your choice.
pub struct LPS25HBInit {
    //pub sensor: SensorSettings,    
}

/*
impl Default for LPS25HBInit {
    fn default() -> Self {
        Self {
            sensor: SensorSettings::default(),            
        }
    }
}
 */
impl LPS25HBInit {
    /// Constructs a new LPS25HB driver instance with a I2C or SPI peripheral.
    ///
    /// # Arguments
    /// * `interface` - `SpiInterface` or `I2cInterface`
    pub fn with_interface<T>(self, interface: T) -> LPS25HB<T>
    where
        T: Interface,
    {
        LPS25HB {
            interface,
            //sensor: self.sensor,            
        }
    }
}

/// LPS25HB sensor
pub struct LPS25HB<T>
where
    T: Interface,
{
    interface: T,
    //sensor: SensorSettings,    
}
 */

pub struct LPS25HB<T> {
    interface: T,
}

impl<T, E> LPS25HB<T> 
where
    T: Interface<Error = E>,
{   
    
    /// Create a new instance of the LPS25HB driver.
    pub fn new(interface: T) -> Self {
        LPS25HB {interface}
    }

    /// Destroy driver instance, return interface instance.
    pub fn destroy(self) -> T {
        self.interface
    }

    
    /// Verifies communication with WHO_AM_I register    
    /// 
    /// USE GET_DEVICE_ID FOR THIS
    pub fn sensor_is_reachable(&mut self) -> Result<bool, T::Error> {
        let mut bytes = [0u8; 1];
        let (who_am_i, register) = (WHOAMI, Registers::WHO_AM_I.addr());        
        self.interface.read(register, &mut bytes)?;
        Ok(bytes[0] == who_am_i)
    }


    // CHANGE BACK TO PRIVATE
    /// Read a byte from the given register.
    pub fn read_register(&mut self, address: Registers) -> Result<u8, T::Error> {
        let mut reg_data = [0u8];
        self.interface.read(address.addr(), &mut reg_data)?;
        Ok(reg_data[0])
    }
            
    /// Clear selected bits using a bitmask
    fn clear_register_bit_flag(&mut self, address: Registers, bitmask: u8) -> Result<(), T::Error> {
        let mut reg_data = [0u8];
        self.interface.read(address.addr(), &mut reg_data)?;        
        let payload: u8 = reg_data[0] & !bitmask;
        self.interface.write(            
            address.addr(),
            payload,
        )?;
        Ok(())
    }    

    /// Set selected bits using a bitmask
    fn set_register_bit_flag(&mut self, address: Registers, bitmask: u8) -> Result<(), T::Error> {
        let mut reg_data = [0u8];
        self.interface.read(address.addr(), &mut reg_data)?;
        let payload: u8 = reg_data[0] | bitmask;
        self.interface.write(            
            address.addr(),
            payload,
        )?;
        Ok(())
    }

    /// Check if specific bits are set.
    fn is_register_bit_flag_high(&mut self, address: Registers, bitmask: u8) -> Result<bool, T::Error> {
        let data = self.read_register(address)?;
        Ok((data & bitmask) != 0)
    }

    /*

    /// FOR DEBUGGING PURPOSES ONLY
    pub fn get_mask(&mut self, mask: ODR) -> Result<u8, T::Error> {
        Ok(mask.value())
    }

    */
     
}
