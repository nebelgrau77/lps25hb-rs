//! TO DO: 
//! - reference pressure reading
//! - pressure offset reading (what is the pressure offset for?)
//! 
//! 


use super::*;

/*
/// Pressure sensor settings. Use this struct to configure the sensor.
#[derive(Debug)]
pub struct SensorSettings {    
    /// Output data rate & power mode selection
    pub sample_rate: ODR,    
    //pub auto_addr_inc: bool,

}
*/

impl<T, E> LPS25HB<T> 
where
    T: Interface<Error = E>,
{   

    /// Read the device ID ("who am I")
    pub fn get_device_id(&mut self) -> Result<u8, T::Error> {
    //pub fn get_device_id(&mut self) -> Result<u8, Error<E>> {
        let mut data = [0u8;1];
        self.interface.read(Registers::WHO_AM_I.addr(), &mut data)?;
        let whoami = data[0];
        Ok(whoami)
    }

    /// Raw sensor reading (3 bytes of pressure data and 2 bytes of temperature data)
    fn read_sensor_raw(&mut self) -> Result<(i32, i16), T::Error> {
        let mut data = [0u8;5];
        self.interface.read(Registers::PRESS_OUT_XL.addr() | Bitmasks::MULTIBYTE, &mut data)?;
        let p: i32 = (data[2] as i32) << 16 | (data[1] as i32) << 8 | (data[0] as i32);
        let t: i16 = (data[4] as i16) << 8 | (data[3] as i16);
        Ok((p, t))
    }

    /// Calculated pressure reading in hPa
    pub fn read_pressure(&mut self) -> Result<f32, T::Error> {
        let (p,_t) = self.read_sensor_raw()?;        
        let pressure = (p as f32) / PRESS_SCALE; // no need to take care of negative values
        Ok(pressure) 
    }

    /// Calculated temperaure reading in degrees Celsius 
    pub fn read_temperature(&mut self) -> Result<f32, T::Error> {
        let (_p,t) = self.read_sensor_raw()?;
        // negative values taken care of, as the raw value is a signed 16-bit
        let temperature = (t as f32) / TEMP_SCALE + TEMP_OFFSET;    
        Ok(temperature)
    }

    /* 
    /// Turn the sensor on (sensor is in power down by default)
    pub fn sensor_power(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::CTRL_REG1, Bitmasks::PD)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::CTRL_REG1, Bitmasks::PD)
            }
        }
    }

    */


    /// Turn the sensor on (sensor is in power down by default)
    pub fn sensor_on(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => {
                self.set_register_bit_flag(Registers::CTRL_REG1, Bitmasks::PD)
            }
            false => {
                self.clear_register_bit_flag(Registers::CTRL_REG1, Bitmasks::PD)
            }
        }
    }

    /// Reboot. Refreshes the content of the internal registers stored in the Flash memory block.
    /// At device power-up the content of the Flash memory block is transferred to the internal registers 
    /// related to the trimming functions to allow correct behavior of the device itself.
    /// If for any reason the content of the trimming registers is modified, 
    /// it is sufficient to use this bit to restore the correct values.
    pub fn reboot(&mut self) -> Result<(), T::Error> {
        self.set_register_bit_flag(Registers::CTRL_REG2, Bitmasks::BOOT)
    }

    /// Run software reset (resets the device to the power-on configuration, takes 4 usec)
    pub fn software_reset(&mut self) -> Result<(), T::Error> {
        self.set_register_bit_flag(Registers::CTRL_REG2, Bitmasks::SWRESET)
    }

    
    /// Has any interrupt event been generated? (self clearing)
    pub fn interrupt_active(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::INT_SOURCE, Bitmasks::IA)
    }

    /// Has low differential pressure event been generated? (self clearing)
    pub fn low_pressure_event_occurred(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::INT_SOURCE, Bitmasks::PL)
    }

    /// Has high differential pressure event been generated? (self clearing)
    pub fn high_pressure_event_occurred(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::INT_SOURCE, Bitmasks::PL)
    }

    /// Has new pressure data overwritten the previous one?
    pub fn pressure_data_overrun(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::P_OR)
    }

    /// Has new temperature data overwritten the previous one?
    pub fn temperature_data_overrun(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::T_OR)
    }

    /// Is new pressure data available?
    pub fn pressure_data_available(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::P_DA)
    }

    /// Is new temperature data available?
    pub fn temperature_data_available(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::T_DA)
    }

    



}





