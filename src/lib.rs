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

   /// `WHO_AM_I` register.
    pub fn get_device_id(&mut self) -> Result<u8, T::Error> {
        let mut data = [0u8;1];
        self.interface.read(Registers::WHO_AM_I.addr(), &mut data)?;
        let whoami = data[0];
        Ok(whoami)
    }

    /// Enable single shot data acquisition (self cleared by hardware)
    pub fn enable_one_shot(&mut self) -> Result<(), T::Error> {
        self.set_register_bit_flag(Registers::CTRL_REG2, ONE_SHOT)?;
        Ok(())
    }

    /// Enable single shot data acquisition (self cleared by hardware)
    pub fn sensor_power(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::CTRL_REG1, PD)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::CTRL_REG1, PD)
            }
        }
    }

    /// Reboot. Refreshes the content of the internal registers stored in the Flash memory block.
    /// At device power-up the content of the Flash memory block is transferred to the internal registers 
    /// related to the trimming functions to allow correct behavior of the device itself.
    /// If for any reason the content of the trimming registers is modified, 
    /// it is sufficient to use this bit to restore the correct values.
    pub fn reboot(&mut self) -> Result<(), T::Error> {
        self.set_register_bit_flag(Registers::CTRL_REG2, BOOT)
    }

    /// Run software reset (resets the device to the power-on configuration, takes 4 usec)
    pub fn software_reset(&mut self) -> Result<(), T::Error> {
        self.set_register_bit_flag(Registers::CTRL_REG2, SWRESET)
    }

    /// Enable or disable block data update
    pub fn bdu_config(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::CTRL_REG1, BDU)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::CTRL_REG1, BDU)
            }
        }
    }

    /// Configuration of the interrupt generation (enabled/disable)
    pub fn int_gen_config(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::CTRL_REG1, DIFF_EN)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::CTRL_REG1, DIFF_EN)
            }
        }
    }

    /// Resets the Autozero function. Self-cleared.
    pub fn autozero_reset(&mut self) -> Result<(), T::Error> {
        self.set_register_bit_flag(Registers::CTRL_REG1, RESET_AZ)
    }

    /// AUTOZERO: when set to ‘1’, the actual pressure output value is copied in
    /// REF_P_H (0Ah), REF_P_L (09h) and REF_P_XL (08h).
    /// When this bit is enabled, the register content of REF_P is subtracted from the pressure output value.
    pub fn autozero_config(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::CTRL_REG2, AUTOZERO)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::CTRL_REG2, AUTOZERO)
            }
        }
    }

    /// Disables I2C interface (default 0, I2C enabled)
    pub fn i2c_disable(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::CTRL_REG2, I2C_DIS)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::CTRL_REG2, I2C_DIS)
            }
        }
    }

    /// Sets SPI Mode (default 4-wire)
    pub fn spi_config(&mut self, mode: SPI_Mode) -> Result<(), T::Error> {
        match mode {
            SPI_Mode::_3wire => {
                self.set_register_bit_flag(Registers::CTRL_REG1, SIM)
            }
            SPI_Mode::_4wire => {
                self.clear_register_bit_flag(Registers::CTRL_REG1, SIM)
            }
        } 
    }

    /// Temperature internal average configuration (default 64).
    pub fn temperature_resolution(&mut self, resolution: TEMP_RES) -> Result<(), T::Error> {
        let mut reg_data = [0u8];
        self.interface.read(Registers::RES_CONF.addr(), &mut reg_data)?;
        let mut payload = reg_data[0];
        payload &= !AVGT_MASK;
        payload |= resolution.value();
        self.interface.write(
            Registers::RES_CONF.addr(),
            payload,
        )?;
        Ok(())
    }

    /// Pressure internal average configuration (default 512).
    pub fn pressure_resolution(&mut self, resolution: PRESS_RES) -> Result<(), T::Error> {
        let mut reg_data = [0u8];
        self.interface.read(Registers::RES_CONF.addr(), &mut reg_data)?;
        let mut payload = reg_data[0];
        payload &= !AVGP_MASK;
        payload |= resolution.value();
        self.interface.write(
            Registers::RES_CONF.addr(),
            payload,
        )?;
        Ok(())
    }

    /// Set output data rate        
    pub fn set_datarate(&mut self, odr: ODR) -> Result<(), T::Error> {
        let mut reg_data = [0u8];
        self.interface.read(Registers::CTRL_REG1.addr(), &mut reg_data)?;
        let mut payload = reg_data[0];
        payload &= !ODR_MASK;
        payload |= odr.value();
        self.interface.write(
            Registers::CTRL_REG1.addr(),
            payload,
        )?;
        Ok(())
    }
    
    /// FIFO enable/disable
    pub fn fifo_config(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::CTRL_REG2, FIFO_EN)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::CTRL_REG2, FIFO_EN)
            }
        }
    }


    /// Select FIFO operation mode (see Table 22 for details)        
    pub fn fifo_mode_config(&mut self, mode: FIFO_MODE) -> Result<(), T::Error> {
        let mut reg_data = [0u8];
        self.interface.read(Registers::FIFO_CTRL.addr(), &mut reg_data)?;
        let mut payload = reg_data[0];
        payload &= !F_MODE_MASK;
        payload |= mode.value();
        self.interface.write(
            Registers::FIFO_CTRL.addr(),
            payload,
        )?;
        Ok(())
    }

    /// Select sample size for FIFO Mean mode running average (see Table 23 for details)        
    pub fn fifo_mean_config(&mut self, sample: FIFO_MEAN) -> Result<(), T::Error> {
        let mut reg_data = [0u8];
        self.interface.read(Registers::FIFO_CTRL.addr(), &mut reg_data)?;
        let mut payload = reg_data[0];
        payload &= !WTM_POINT_MASK;
        payload |= sample.value();
        self.interface.write(
            Registers::FIFO_CTRL.addr(),
            payload,
        )?;
        Ok(())
    }


    /// Interrupt request latching to INT_SOURCE
     pub fn int_latch_config(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::INTERRUPT_CFG, LIR)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::INTERRUPT_CFG, LIR)
            }
        }
    }

    /// Enable interrupt on differential pressure low event
    pub fn diff_press_low_config(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::INTERRUPT_CFG, PL_E)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::INTERRUPT_CFG, PL_E)
            }
        }
    }

    /// Enable interrupt on differential pressure high event
    pub fn diff_press_high_config(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::INTERRUPT_CFG, PH_E)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::INTERRUPT_CFG, PH_E)
            }
        }
    }

    /// FIFO empty flag on INT_DRDY pin
    pub fn fifo_empty_drdy_config(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::CTRL_REG4, F_EMPTY)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::CTRL_REG4, F_EMPTY)
            }
        }
    }

    /// FIFO filled up to threshold (watermark) level on INT_DRDY pin 
    pub fn fifo_filled_drdy_config(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::CTRL_REG4, F_FTH)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::CTRL_REG4, F_FTH)
            }
        }
    }

    /// FIFO overrun interrupt on INT_DRDY pin 
    pub fn fifo_overrun_drdy_config(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::CTRL_REG4, F_OVR)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::CTRL_REG4, F_OVR)
            }
        }
    }

    /// Data-ready signal on INT_DRDY pin 
    pub fn data_signal_drdy_config(&mut self, flag: Control) -> Result<(), T::Error> {
        match flag {
            Control::On => {
                self.set_register_bit_flag(Registers::CTRL_REG4, DRDY)
            }
            Control::Off => {
                self.clear_register_bit_flag(Registers::CTRL_REG4, DRDY)
            }
        }
    }


    // a block of get_status functions, following this example:
    // 
    // pub fn get_voltage_low_flag(&mut self) -> Result<bool, Error<E>> {
    //    self.is_register_bit_flag_high(Register::VL_SECONDS, BitFlags::VL)} 

    /// Has any interrupt event been generated? (self clearing)
    pub fn interrupt_active(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::INT_SOURCE, IA)
    }

    /// Has low differential pressure event been generated? (self clearing)
    pub fn low_pressure_event_occurred(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::INT_SOURCE, PL)
    }

    /// Has high differential pressure event been generated? (self clearing)
    pub fn high_pressure_event_occurred(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::INT_SOURCE, PL)
    }

    /// Has new pressure data overwritten the previous one?
    pub fn pressure_data_overrun(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, P_OR)
    }

    /// Has new temperature data overwritten the previous one?
    pub fn temperature_data_overrun(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, T_OR)
    }

    /// Is new pressure data available?
    pub fn pressure_data_available(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, P_DA)
    }

    /// Is new temperature data available?
    pub fn temperature_data_available(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, T_DA)
    }

    /// Is FIFO filling equal or higher than the threshold?
    pub fn fifo_threshold_status(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::FIFO_STATUS, FTH_FIFO)
    }

    /// Is FIFO full and at least one sample has been overwritten?
    pub fn fifo_overrun_status(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::FIFO_STATUS, OVR)
    }

    /// Is FIFO empty?
    pub fn fifo_empty_status(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::FIFO_STATUS, EMPTY_FIFO)
    }

    /// Raw sensor reading (3 bytes of pressure data and 2 bytes of temperature data)
    fn read_sensor_raw(&mut self) -> Result<(i32, i16), T::Error> {
        let mut data = [0u8;5];
        self.interface.read(Registers::PRESS_OUT_XL.addr() | MULTIBYTE, &mut data)?;
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


    // CHANGE BACK TO PRIVATE
    /// Read a byte from the given register.
    pub fn read_register(&mut self, address: Registers) -> Result<u8, T::Error> {
        let mut reg_data = [0u8];
        self.interface.read(address.addr(), &mut reg_data)?;
        Ok(reg_data[0])
    }

    /// Check if specific bits are set.
    fn is_register_bit_flag_high(&mut self, address: Registers, bitmask: u8) -> Result<bool, T::Error> {
        let data = self.read_register(address)?;
        Ok((data & bitmask) != 0)
    }



    /// FOR DEBUGGING PURPOSES ONLY
    pub fn get_mask(&mut self, mask: ODR) -> Result<u8, T::Error> {
        Ok(mask.value())
    }


     

        /*
        /// Write to a register.
        fn write_register(&mut self, address: Registers, data: u8) -> Result<(), T::Error> {
            let payload: [u8; 2] = [address, data]; 
            self.interface.write(
                address.addr(),
                payload,
            )?;
            Ok(())
        }
        */


    }
