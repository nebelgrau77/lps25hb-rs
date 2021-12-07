//! Various functions related to FIFO
//!
//! TO DO:
//! - watermark level selection (5 bit value) - check how it relates to MEAN settings!
//!

// TO DO (IF POSSIBLE):
// configuration struct similar to the one for the interrupts
// using:
// CTRL_REG2::STOP_ON_FTH
// CTRL_REG2::FIFO_MEAN_DEC
// FIFO_CTRL::F_MODE
// FIFO_CTRL::WTM_POINT (complicated: has to use the specific values if in MEAN mode)


use super::*;

impl<T, E> LPS25HB<T>
where
    T: Interface<Error = E>,
{
    /// FIFO enable/disable
    pub fn fifo_enable(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::CTRL_REG2, Bitmasks::FIFO_EN),
            false => self.clear_register_bit_flag(Registers::CTRL_REG2, Bitmasks::FIFO_EN),
        }
    }

    /// Select FIFO operation mode (see Table 22 for details)        
    pub fn fifo_mode_config(&mut self, mode: FIFO_MODE) -> Result<(), T::Error> {
        let mut reg_data = [0u8];
        self.interface
            .read(Registers::FIFO_CTRL.addr(), &mut reg_data)?;
        let mut payload = reg_data[0];
        payload &= !Bitmasks::F_MODE_MASK;
        payload |= mode.value();
        self.interface.write(Registers::FIFO_CTRL.addr(), payload)?;
        Ok(())
    }

    /// Select sample size for FIFO Mean mode running average (see Table 23 for details)        
    pub fn fifo_mean_config(&mut self, sample: FIFO_MEAN) -> Result<(), T::Error> {
        let mut reg_data = [0u8];
        self.interface
            .read(Registers::FIFO_CTRL.addr(), &mut reg_data)?;
        let mut payload = reg_data[0];
        payload &= !Bitmasks::WTM_POINT_MASK;
        payload |= sample.value();
        self.interface.write(Registers::FIFO_CTRL.addr(), payload)?;
        Ok(())
    }

    /// FIFO empty flag on INT_DRDY pin
    pub fn fifo_empty_drdy_enable(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::CTRL_REG4, Bitmasks::F_EMPTY),
            false => self.clear_register_bit_flag(Registers::CTRL_REG4, Bitmasks::F_EMPTY),
        }
    }

    /// FIFO filled up to threshold (watermark) level on INT_DRDY pin
    pub fn fifo_filled_drdy_enable(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::CTRL_REG4, Bitmasks::F_FTH),
            false => self.clear_register_bit_flag(Registers::CTRL_REG4, Bitmasks::F_FTH),
        }
    }

    /// FIFO overrun interrupt on INT_DRDY pin
    pub fn fifo_overrun_drdy_enable(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::CTRL_REG4, Bitmasks::F_OVR),
            false => self.clear_register_bit_flag(Registers::CTRL_REG4, Bitmasks::F_OVR),
        }
    }

    /// Is FIFO filling equal or higher than the threshold?
    pub fn fifo_threshold_status(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::FIFO_STATUS, Bitmasks::FTH_FIFO)
    }

    /// Is FIFO full and at least one sample has been overwritten?
    pub fn fifo_overrun_status(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::FIFO_STATUS, Bitmasks::OVR)
    }

    /// Is FIFO empty?
    pub fn fifo_empty_status(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::FIFO_STATUS, Bitmasks::EMPTY_FIFO)
    }

    /// Read FIFO stored data level
    pub fn read_fifo_level(&mut self) -> Result<u8, T::Error> {
        let mut reg_data = [0u8];
        self.interface
            .read(Registers::FIFO_STATUS.addr(), &mut reg_data)?;

        let fifo_level: u8 = match self.fifo_empty_status()? {
            true => 0,
            false => (reg_data[0] & Bitmasks::FSS_MASK) + 1,
        };

        Ok(fifo_level)
    }

    /// Stop on FIFO watermark (enable FIFO watermark use)
    pub fn stop_on_fth(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::CTRL_REG2, Bitmasks::STOP_ON_FTH),
            false => self.clear_register_bit_flag(Registers::CTRL_REG2, Bitmasks::STOP_ON_FTH),
        }
    }

    /// Enable decimating output pressure to 1Hz with FIFO Mean mode
    pub fn fifo_decimate_enable(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::CTRL_REG2, Bitmasks::FIFO_MEAN_DEC),
            false => self.clear_register_bit_flag(Registers::CTRL_REG2, Bitmasks::FIFO_MEAN_DEC),
        }
    }

    /*
    /// Set the watermark level
    pub fn set_watermark_level(&mut self, level: u8) -> Result<(), T::Error> {
        let wtm: u8 = match level {
            // if the input value exceeds the capacity, default to maximum
            l if l < 33 => l,
            _ => 32,
        };
        let mut reg_data = [0u8];
        self.interface
            .read(Registers::FIFO_CTRL.addr(), &mut reg_data)?;
        let mut payload = reg_data[0];
        payload &= !Bitmasks::WTM_MASK;
        payload |= mode.value();
        self.interface.write(Registers::FIFO_CTRL.addr(), payload)?;
        Ok(())
    }
     */
}
