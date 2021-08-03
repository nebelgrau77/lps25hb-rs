//! TO DO: CHECK AND MODIFY ACCORDINGLY FOR THE LPS25HB SENSOR
//! 
//! DEVICE MUST BE TURNED ON WITH POWER DOWN BIT OF CTRL_REG1
//! BUT THERE IS ALSO A ONESHOT BIT IN CTRL_REG2 TO ACQUIRE A SINGLE VALUE
//! 
//! AUTO_ADD_INC DOES NOT EXIST
//! 
//! 
//! Pressure sensor settings, types
#![allow(dead_code, non_camel_case_types)]

/// Pressure sensor settings. Use this struct to configure the sensor.
#[derive(Debug)]
pub struct SensorSettings {    
    /// Output data rate & power mode selection
    pub sample_rate: ODR,    
    //pub auto_addr_inc: bool,

}

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
    pub fn interrupt_cfg(&self) -> u8 {
        let mut result = 0_u8;
        
        result        
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
        (self as u8) << 4
    }
}



#[test]
fn sensor_init_values() {
    let settings = SensorSettings::default();
    //assert_eq!(settings.ctrl_reg5_xl(), 0b0011_1000); // [DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
    //assert_eq!(settings.ctrl_reg6_xl(), 0b0110_0000); // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
    //assert_eq!(settings.ctrl_reg7_xl(), 0b0000_0000); // [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
}
