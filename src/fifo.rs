//! Various functions related to FIFO
//! 
//! TO DO: 
//! - watermark level selection
//! - FIFO stored data level reading
//! 



use super::*; 

impl<T, E> LPS25HB<T> 
where
    T: Interface<Error = E>,
{   

    /// FIFO enable/disable
    pub fn fifo_control(&mut self, flag: Control) -> Result<(), T::Error> {
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


}