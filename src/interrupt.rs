//! Various functions related to interrupts
//!

use super::*;

/// Interrupt pin settings
#[derive(Debug)]
pub struct InterruptConfig {
    /// configure interrupt pin as active high or active low
    pub active_high_or_low: bool,
    /// configure interrupt pin as  push-pull or open drain
    pub pushpull_or_opendrain: bool,
    /// configure data signal on the interrupt pin
    pub data_signal_config: INT_DRDY,
    /// enable FIFO empty flag on interrupt pin
    pub enable_fifo_empty: bool,
    /// enable FIFO watermark flag on interrupt pin
    pub enable_fifo_fth: bool,
    /// enable FIFO overrun flag on interrupt pin
    pub enable_fifo_overrun: bool,
    /// enable data ready signal on interrupt pin
    pub enable_data_ready: bool,
    /// enable latching interrupt request to INT_SOURCE register
    pub enable_latch_interrupt: bool,
    /// enable low pressure event on interrupt pin
    pub enable_low_event: bool, 
    /// enable hihg pressure event on interrupt pin
    pub enable_high_event: bool, 
}

impl Default for InterruptConfig {
    fn default() -> Self {
        InterruptConfig {
            active_high_or_low: false, // active high (CTRL_REG3)
            pushpull_or_opendrain: false, // push-pull (CTRL_REG3)
            data_signal_config: INT_DRDY::DataSignal, // data signal on INT_DRDY pin (CTRL_REG3)
            enable_fifo_empty: false, // disabled (CTRL_REG4)
            enable_fifo_fth: false, // disabled (CTRL_REG4)
            enable_fifo_overrun: false, // disabled (CTRL_REG4)                       
            enable_data_ready: false, // disabled (CTRL_REG4)
            enable_latch_interrupt: false, // inferrupt request not latched (INTERRUPT_CFG)
            enable_low_event: false, // disable interrupt request on low pressure event (INTERRUPT_CFG)
            enable_high_event: false, // disable interrupt request on low pressure event (INTERRUPT_CFG)            
        }
    }
}

impl InterruptConfig {
    // what to do here? it should use the fields to set various registers
    // using the already defined functions (that should be private)
    // if LSM9DS1 crate can be an example, then 
    // functions would be here, but instead of setting up single bits
    // would return values to be written to registers instead?
    // this could actually work, as there are three registers involved:
    // so the functions could be:
    fn ctrl_reg3(&self) -> u8 {
        let mut data = 0u8;
        if self.active_high_or_low {
            data |= 1 << 7;
        }
        if self.pushpull_or_opendrain {
            data |= 1 << 6;
        }
        // MUST USE THE ACTUAL u8 VALUE HERE
        data |= self.data_signal_config.value(); 
        data
    }
    fn ctrl_reg4(&self) -> u8 {
        let mut data = 0u8;
        if self.enable_fifo_empty {
            data |= 1 << 3;
        }
        if self.enable_fifo_fth {
            data |= 1 << 2;
        }
        if self.enable_fifo_overrun {
            data |= 1 << 1;
        }
        if self.enable_data_ready {
            data |= 1;
        }
        data
    }
    fn interrupt_cfg(&self) -> u8 {
        let mut data = 0u8;    
        if self.enable_latch_interrupt {
            data |= 1 << 2;
        }
        if self.enable_low_event {
            data |= 1 << 1;
        }
        if self.enable_high_event {
            data |= 1;
        }
        data
    }
}



impl<T, E> LPS25HB<T>
where
    T: Interface<Error = E>,
{
    /// Configuration of the interrupt generation (enabled/disable)
    pub fn int_generation_enable(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::CTRL_REG1, Bitmasks::DIFF_EN),
            false => self.clear_register_bit_flag(Registers::CTRL_REG1, Bitmasks::DIFF_EN),
        }
    }

    /// Configuration of the interrupt generation (enabled/disable)
    pub fn enable_interrupts(&mut self, flag: bool, config: InterruptConfig) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::CTRL_REG1, Bitmasks::DIFF_EN),
            false => self.clear_register_bit_flag(Registers::CTRL_REG1, Bitmasks::DIFF_EN),
        };
        // MUST USE u8 VALUES OF THE Registers FIELDS
        self.interface.write(Registers::CTRL_REG3.addr(), config.ctrl_reg3())?;
        self.interface.write(Registers::CTRL_REG4.addr(), config.ctrl_reg4())?;
        self.interface.write(Registers::INTERRUPT_CFG.addr(), config.interrupt_cfg())?;
        Ok(())

    }

    /// Interrupt request latching to INT_SOURCE
    pub fn int_latch_enable(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::INTERRUPT_CFG, Bitmasks::LIR),
            false => self.clear_register_bit_flag(Registers::INTERRUPT_CFG, Bitmasks::LIR),
        }
    }

    /// Enable interrupt on differential pressure low event
    pub fn diff_press_low_enable(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::INTERRUPT_CFG, Bitmasks::PL_E),
            false => self.clear_register_bit_flag(Registers::INTERRUPT_CFG, Bitmasks::PL_E),
        }
    }

    /// Enable interrupt on differential pressure high event
    pub fn diff_press_high_enable(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::INTERRUPT_CFG, Bitmasks::PH_E),
            false => self.clear_register_bit_flag(Registers::INTERRUPT_CFG, Bitmasks::PH_E),
        }
    }

    /// Data-ready signal on INT_DRDY pin
    pub fn data_signal_drdy_enable(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::CTRL_REG4, Bitmasks::DRDY),
            false => self.clear_register_bit_flag(Registers::CTRL_REG4, Bitmasks::DRDY),
        }
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
        self.is_register_bit_flag_high(Registers::INT_SOURCE, Bitmasks::PH)
    }

    /// Interrupt active high/low (default active high)
    pub fn interrupt_pin_active(&mut self, setting: INT_ACTIVE) -> Result<(), T::Error> {
        match setting {
            INT_ACTIVE::High => {
                self.clear_register_bit_flag(Registers::CTRL_REG3, Bitmasks::INT_H_L)
            }
            INT_ACTIVE::Low => self.set_register_bit_flag(Registers::CTRL_REG3, Bitmasks::INT_H_L),
        }
    }

    /// Interrupt pin configuration: push-pull (default) or open drain
    pub fn interrupt_pin_config(&mut self, setting: INT_PIN) -> Result<(), T::Error> {
        match setting {
            INT_PIN::PushPull => {
                self.clear_register_bit_flag(Registers::CTRL_REG3, Bitmasks::PP_OD)
            }
            INT_PIN::OpenDrain => self.set_register_bit_flag(Registers::CTRL_REG3, Bitmasks::PP_OD),
        }
    }

    /// Configure INT_DRDY pin
    pub fn int_drdy_config(&mut self, config: INT_DRDY) -> Result<(), T::Error> {
        let mut reg_data = [0u8];
        self.interface
            .read(Registers::CTRL_REG3.addr(), &mut reg_data)?;
        let mut payload = reg_data[0];
        payload &= !Bitmasks::INT_S_MASK;
        payload |= config.value();
        self.interface.write(Registers::CTRL_REG3.addr(), payload)?;
        Ok(())
    }
}
