//! Various functions related to FIFO
//! 
//! TO DO: 
//! - improve watermark level reading (fifo_level)
//! - check if all FIFO-related functions are implemented

use super::*;

/// FIFO settings
#[derive(Debug)]
pub struct FIFOConfig {

    /// Stop on FIFO watermark (enable FIFO watermark use)
    pub enable_watermark: FLAG, // default disabled
    /// Enable decimating output pressure to 1Hz with FIFO Mean mode
    pub enable_decimating: FLAG, // default disabled
    /// Select FIFO operation mode (see Table 22 for details)        
    pub fifo_mode: FIFO_MODE, // default Bypass
    /// Set the watermark level
    pub watermark_level: u8, // default 0
    /// Select sample size for FIFO Mean mode running average (see Table 23 for details)        
    pub fifo_mean_config: FIFO_MEAN, // default 2-sample
}

impl Default for FIFOConfig {
    fn default() -> Self {
        FIFOConfig {
            enable_watermark: FLAG::Disabled,               // disabled
            enable_decimating: FLAG::Disabled,              // disabled
            fifo_mode: FIFO_MODE::Bypass,          // Bypass mode
            watermark_level: 32u8,                  // 0 does not make sense as a default value
            fifo_mean_config: FIFO_MEAN::_2sample, // 2 samples
        }
    }
}

impl FIFOConfig {
    /// Returns values to be written to CTRL_REG2 and FIFO_CTRL:
    fn f_ctrl_reg2(&self) -> u8 {
        let mut data = 0u8;
        // THIS RESULT MUST THEN BE COMBINED WITH THE OTHER BIT SETTINGS
        if self.enable_watermark.status() {
            data |= 1 << 5;
        }
        if self.enable_decimating.status() {
            data |= 1 << 4;
        }
        data
    }
    fn f_fifo_ctrl(&self) -> u8 {
        let mut data = 0u8;

        data |= self.fifo_mode.value();

        let wtm = match self.fifo_mode {
            FIFO_MODE::FIFO_Mean => self.fifo_mean_config.value(),
            _ => self.watermark_level,
        };
        data |= wtm;
        data
    }
}

#[derive(Debug)]
/// Contents of the FIFO_STATUS register (threshold reached, overrun, empty, stored data level)
pub struct FifoStatus {
    pub fifo_thresh_reached: bool,
    pub fifo_overrun: bool,
    pub fifo_empty: bool,
    pub fifo_level: u8,
}

impl<T, E> LPS25HB<T>
where
    T: Interface<Error = E>,
{
    /// Enable and configure FIFO
    pub fn configure_fifo(&mut self, flag: FIFO_ON, config: FIFOConfig) -> Result<(), T::Error> {
        match flag {
            FIFO_ON::Enabled => self.set_register_bit_flag(Registers::CTRL_REG2, Bitmasks::FIFO_EN),
            FIFO_ON::Disabled => self.clear_register_bit_flag(Registers::CTRL_REG2, Bitmasks::FIFO_EN),
        }?;

        let mut reg_data = [0u8];
        self.interface
            .read(Registers::CTRL_REG2.addr(), &mut reg_data)?;
        reg_data[0] |= config.f_ctrl_reg2();
        self.interface
            .write(Registers::CTRL_REG2.addr(), reg_data[0])?;
        self.interface
            .write(Registers::FIFO_CTRL.addr(), config.f_fifo_ctrl())?;

        Ok(())
    }

    /// Get flags and FIFO level from the FIFO_STATUS register
    pub fn get_fifo_status(&mut self) -> Result<FifoStatus, T::Error> {
        
        let reg_value = self.read_register(Registers::FIFO_STATUS)?;

        let status = FifoStatus {
            /// Is FIFO filling equal or higher than the threshold?
            fifo_thresh_reached: match reg_value & Bitmasks::FTH_FIFO {
                0 => false,
                _ => true,
            },
            /// Is FIFO full and at least one sample has been overwritten?
            fifo_overrun: match reg_value & Bitmasks::OVR {
                0 => false,
                _ => true,
            },
            /// Is FIFO empty?
            fifo_empty: match reg_value & Bitmasks::EMPTY_FIFO {
                0 => false,
                _ => true,
            },            
            /// Read FIFO stored data level            
            
            // replace with a bitmask?

            fifo_level: self.read_fifo_level()?,
          };
        
        Ok(status)
    }

    /// Read FIFO stored data level    
    fn read_fifo_level(&mut self) -> Result<u8, T::Error> {
        let mut reg_data = [0u8];
        self.interface
            .read(Registers::FIFO_STATUS.addr(), &mut reg_data)?;

        let fifo_level: u8 = match self.is_register_bit_flag_high(Registers::FIFO_STATUS, Bitmasks::EMPTY_FIFO)? {
            true => 0,
            false => (reg_data[0] & Bitmasks::FSS_MASK) + 1,
        };

        Ok(fifo_level)
    }
}
