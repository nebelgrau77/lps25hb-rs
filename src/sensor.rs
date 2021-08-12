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

    /// `WHO_AM_I` register.
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

    



}





/// Output data rate and power mode selection (ODR). (Refer to Table 20)
#[derive(Debug, Clone, Copy)]
pub enum ODR {
    /// One-shot mode enabled
    OneShot = 0b000,
    /// 1 Hz
    _1Hz = 0b001,
    /// 7 Hz
    _7Hz = 0b010,
    /// 12.5 Hz
    _12_5Hz = 0b011,
    /// 25 Hz
    _25Hz = 0b100,    
}

impl ODR {
    pub fn value(self) -> u8 {
        (self as u8) << 4 // shifted into the right position, can be used directly
    }
}

/// SPI interface mode
#[derive(Debug, Clone, Copy)]
pub enum SPI_Mode {
    /// 4-wire mode (default)
    _4wire,
    /// 3-wire mode
    _3wire,    
}


/// INT_DRDY pin configuration. (Refer to Table 21)
#[derive(Debug, Clone, Copy)]
pub enum INT_DRDY {
    /// Data signal (see CTRL_REG4)
    DataSignal = 0b00,
    /// Pressure high
    P_high = 0b01,
    /// Pressure low
    P_low = 0b10,
    /// Pressure low or high
    P_low_or_high = 0b011,
    
}

impl INT_DRDY {
    pub fn value(self) -> u8 {
        self as u8 // no need to shift, bits 0:1
    }
}


/// FIFO mode selection. (Refer to Table 22)
#[derive(Debug, Clone, Copy)]
pub enum FIFO_MODE {
    /// Bypass mode
    Bypass = 0b000,
    /// FIFO mode
    FIFO = 0b001,
    /// Stream mode
    Stream = 0b010,
    /// Stream-to-FIFO mode
    Stream_to_FIFO = 0b011,
    /// Bypass-to-stream mode
    Bypass_to_stream = 0b100,
    /// FIFO Mean mode
    FIFO_Mean = 0b110,
    /// Bypass-to-FIFO mode
    Bypass_to_FIFO = 0b111,
    
}

impl FIFO_MODE {
    pub fn value(self) -> u8 {
        (self as u8) << 5 // shifted into the right position, can be used directly
    }
}

/// FIFO Mean mode running average sample size. (Refer to Table 23)
#[derive(Debug, Clone, Copy)]
pub enum FIFO_MEAN {
    /// 2-sample moving average
    _2sample = 0b00001,
    /// 4-sample moving average
    _4sample = 0b00011,
    /// 8-sample moving average
    _8sample = 0b00111,
    /// 16-sample moving average
    _16sample = 0b01111,
    /// 32-sample moving average
    _32sample = 0b11111,
    
}

impl FIFO_MEAN {
    pub fn value(self) -> u8 {
        self as u8 // no need to shift, bits 0:4
    }
}


/// Temperature resolution configuration, number of internal average(Refer to Table 18)
#[derive(Debug, Clone, Copy)]
pub enum TEMP_RES {
    /// Nr. internal average 8
    _8 = 0b00,
    /// Nr. internal average 16
    _16 = 0b01,
    /// Nr. internal average 32
    _32 = 0b10,
    /// Nr. internal average 64
    _64 = 0b11,
    
}

impl TEMP_RES {
    pub fn value(self) -> u8 {
        (self as u8) << 2 // shifted into the right position, can be used directly
    }
}

/// Pressure resolution configuration, number of internal average(Refer to Table 19)
#[derive(Debug, Clone, Copy)]
pub enum PRESS_RES {
    /// Nr. internal average 8
    _8 = 0b00,
    /// Nr. internal average 32
    _32 = 0b01,
    /// Nr. internal average 128
    _128 = 0b10,
    /// Nr. internal average 512
    _512 = 0b11,
    
}

impl PRESS_RES {
    pub fn value(self) -> u8 {
        self as u8 // no need to shift
    }
}


/// Two possible choices, used for various enable/disable bit flags
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]

pub enum Control {    
    /// Enable some feature, eg. timer 
    On, 
    /// Disable some feature, eg. timer
    Off,     
}

impl Control {
    pub fn value(self) -> u8 {
        self as u8
    }
}




/*
impl Default for SensorSettings {
    fn default() -> Self {
        SensorSettings {            
            sample_rate: ODR::OneShot,            
            //auto_addr_inc: true,
        }
    }
}

impl SensorSettings {
    
    /// Returns `u8` to write to CTRL_REG1 (0x20)
    /// # CTRL_REG1: [PD][ODR2][ODR1][ODR0][DIFF_EN][BDU][RESET_AZ][SIM]
    /// - PD - power-down control, default value 0, 1 - active mode
    /// - ODR[2:0] - Output data rate & power mode selection    
    /// - DIFF_EN - Interrupt generation enable, default value 0, 1 - interrupt generation enabled
    /// - BDU - Block data update
    /// - RESET_AZ - Reset auto-zero function, default value 0, 1 - reset Autozero.
    /// - SIM - SPI Serial Interface Mode selection, 0 - 4-wire interface (default), 1 - 3-wire interface
    
    pub fn ctrl_reg1(&self) -> u8 {
        self.sample_rate.value()
    }
    
    /// Returns `u8` to write to CTRL_REG2 (0x21)
    /// # CTRL_REG2: [BOOT][FIFO_EN][STOP_ON_FTH][FIFO_MEAN_DEC][I2C_DIS][SWRESET][AUTO_ZERO][ONE_SHOT]    
    /// - BOOT - Reboot memory content
    /// - FIFO_EN - FIFO enable
    /// - STOP_ON_FTH - Stop on FIFO watermark. Enable FIFO watermark level use    
    /// - FIFO_MEAN_DEC - enable to decimate output pressure to 1Hz with FIFO Mean mode
    /// - I2C_DIS - Disable I2C interface
    /// - SWRESET - Software reset
    /// - AUTOZERO - Autozero enable
    /// - ONE_SHOT - One-shot enable    
    pub fn ctrl_reg2(&self) -> u8 {
        let mut result = 0_u8;
        /*
        if self.auto_addr_inc {
            result |= 1 << 4;
        }
        */
        result        
    }

    /// Returns `u8` to write to CTRL_REG3 (0x22)
    /// # CTRL_REG3: [INT_H_L][PP_OD][Reserved][Reserved][Reserved][Reserved][INT_S2][INT_S1]
    /// - INT_H_L - Interrupt active high-low, 0 - active high (default), 1 - active low
    /// - PP_OD - Push-pull/open drain selection on interrupt pads, 0 - push-pull (default), 1 - open drain
    /// - INT_S[2:1] - Data signal on INT_DRDY pin control bits. Default value 00, 01 - pressure high (P_high), 10 - pressure low (P_low), 11 - pressure low OR high
    pub fn ctrl_reg3(&self) -> u8 {
        let mut result = 0_u8;
        
        result        
    }

    /// Returns `u8` to write to CTRL_REG4 (0x23)
    /// # CTRL_REG4: [0][0][0][0][F_EMPTY][F_FTH][F_OVR][DRDY]
    /// - F_EMPTY - FIFO empty flag on INT_DRDY pin, default 0, 1 - enabled
    /// - F_FTH - FIFO threshold (watermark) status on INT_DRDY pin to indicate that FIFO is filled up to the threshold level, default 0, 1 - enabled
    /// - F_OVR - FIFO overrun interrupt on INT_DRDY pin to indicate that FIFO is full in FIFO mode or that overrun occured in Stream mode, default 0, 1 - enabled
    /// - DRDY - data-ready signal on INT_DRDY pin, default value 0, 1 - enabled
    pub fn ctrl_reg4(&self) -> u8 {
        let mut result = 0_u8;
        
        result        
    }


    /// Returns `u8` to write to RES_CONF(0x10)
    /// # RES_CONF: [Reserved][Reserved][Reserved][Reserved][AVGT1][AVGT0][AVGP1][AVGP0]
    /// - AVGT[1:0] - temperature internal average configuration, 00 - nr. internal average 8, 01 - 16, 10 - 32, 11 - 64 [Table 18]
    /// - AVGP[1:0] - pressure internal average configuration, 00 - nr. internal average 8, 01 - 16, 10 - 32, 11 - 64 [Table 19]
    pub fn res_conf(&self) -> u8 {
        let mut result = 0_u8;
        
        result        
    }

    /// Returns `u8` to write to INTERRUPT_CFG(0x24)
    /// # INTERRUPT_CFG: [Reserved][Reserved][Reserved][Reserved][Reserved][LIR][PL_E][PH_E]    
    /// - LIR - latch interrupt request to INT_SOURCE register, default value 0, 1 - interrupt request latched (see Fig. 20 and 21 for detailed explanation)
    /// - PL_E - enable interrupt generation on differential pressure low event, default 0, 1 - enable interrupt request on measured differential pressure value lower than preset threshold
    /// - PH_E - enable interrupt generation on differential pressure high event, default 0, 1 - enable interrupt request on measured differential pressure value lower than preset threshold
    pub fn interrupt_cfg(&self) -> u8 {
        let mut result = 0_u8;
        
        result        
    }


    /// Returns `u8` to write to FIFO_CTRL(0x2E)
    /// # FIFO_CTRL: [F_MODE2][F_MODE1][F_MODE0][WTM_POINT4][WTM_POINT3][WTM_POINT2][WTM_POINT1][WTM_POINT0]
    /// - F_MODE[2:0] - FIFO mode selection, default 000, see table 22 and section 4 for details
    /// - WTM_POINT[4:0] - FIFO threshold (watermark) level selection, see table 23 for details
    pub fn fifo_ctrl(&self) -> u8 {
        let mut result = 0_u8;
        
        result        
    }

    

}

*/
 




#[test]
fn sensor_init_values() {
    let settings = SensorSettings::default();
    //assert_eq!(settings.ctrl_reg5_xl(), 0b0011_1000); // [DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
    //assert_eq!(settings.ctrl_reg6_xl(), 0b0110_0000); // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
    //assert_eq!(settings.ctrl_reg7_xl(), 0b0000_0000); // [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
}
