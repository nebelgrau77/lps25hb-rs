//! Various functions related to interrupts

use super::*;

/// Interrupt pin settings
#[derive(Debug)]
pub struct InterruptConfig {

    /// configure interrupt pin as active high or active low
    pub active_high_or_low: INT_ACTIVE,
    /// configure interrupt pin as  push-pull or open drain
    pub pushpull_or_opendrain: INT_PIN,
    /// configure data signal on the interrupt pin
    pub data_signal_config: INT_DRDY,
    /// enable FIFO empty flag on interrupt pin
    pub enable_fifo_empty: FLAG,
    /// enable FIFO watermark flag on interrupt pin
    pub enable_fifo_fth: FLAG,
    /// enable FIFO overrun flag on interrupt pin
    pub enable_fifo_overrun: FLAG,
    /// enable data ready signal on interrupt pin
    pub enable_data_ready: FLAG,
    /// enable computing of differential pressure output
    pub enable_differential: FLAG,
    /// enable latching interrupt request to INT_SOURCE register
    pub enable_latch_interrupt: FLAG,
    /// enable low pressure event on interrupt pin
    pub enable_low_event: FLAG,
    /// enable hihg pressure event on interrupt pin
    pub enable_high_event: FLAG,
}

impl Default for InterruptConfig {
    fn default() -> Self {
        InterruptConfig {
            active_high_or_low: INT_ACTIVE::High,              // active high (CTRL_REG3)
            pushpull_or_opendrain: INT_PIN::PushPull,          // push-pull (CTRL_REG3)
            data_signal_config: INT_DRDY::DataSignal,          // data signal on INT_DRDY pin (CTRL_REG3)
            enable_fifo_empty: FLAG::Disabled,                 // disabled (CTRL_REG4)
            enable_fifo_fth: FLAG::Disabled,                   // disabled (CTRL_REG4)
            enable_fifo_overrun: FLAG::Disabled,               // disabled (CTRL_REG4)
            enable_data_ready: FLAG::Disabled,                 // disabled (CTRL_REG4)
            enable_differential: FLAG::Disabled,               // disabled (CTRL_REG1)
            enable_latch_interrupt: FLAG::Disabled,            // inferrupt request not latched (INTERRUPT_CFG)
            enable_low_event: FLAG::Disabled,                  // disable interrupt request on low pressure event (INTERRUPT_CFG)
            enable_high_event: FLAG::Disabled,                 // disable interrupt request on low pressure event (INTERRUPT_CFG)
        }
    }
}

impl InterruptConfig {
    /// Returns values to be written to CTRL_REG3, CTRL_REG4 and INTERRUPT_CFG:
    fn int_ctrl_reg3(&self) -> u8 {
        let mut data = 0u8;
        if self.active_high_or_low.status() {
            data |= 1 << 7;
        }
        if self.pushpull_or_opendrain.status() {
            data |= 1 << 6;
        }
        // MUST USE THE ACTUAL u8 VALUE HERE
        data |= self.data_signal_config.value();
        data
    }
    fn int_ctrl_reg4(&self) -> u8 {
        let mut data = 0u8;
        if self.enable_fifo_empty.status() {
            data |= 1 << 3;
        }
        if self.enable_fifo_fth.status() {
            data |= 1 << 2;
        }
        if self.enable_fifo_overrun.status() {
            data |= 1 << 1;
        }
        if self.enable_data_ready.status() {
            data |= 1;
        }
        data
    }
    fn int_interrupt_cfg(&self) -> u8 {
        let mut data = 0u8;
        if self.enable_latch_interrupt.status() {
            data |= 1 << 2;
        }
        if self.enable_low_event.status() {
            data |= 1 << 1;
        }
        if self.enable_high_event.status() {
            data |= 1;
        }
        data
    }
}

#[derive(Debug)]
/// Contents of the INT_SOURCE register (interrupt active and differential pressure events flags)
pub struct IntStatus {
    pub interrupt_active: bool,
    pub diff_press_low: bool,
    pub diff_press_high: bool,    
}

impl<T, E> LPS25HB<T>
where
    T: Interface<Error = E>,
{
    /// Configure interrupt pin and interrupt sources, enable and configure differential pressure interrupts
    pub fn configure_interrupts(
        &mut self,
        //flag: bool,
        config: InterruptConfig,
    ) -> Result<(), T::Error> {
        match config.enable_differential {
            true => self.set_register_bit_flag(Registers::CTRL_REG1, Bitmasks::DIFF_EN),
            false => self.clear_register_bit_flag(Registers::CTRL_REG1, Bitmasks::DIFF_EN),
        }?;

        self.interface
            .write(Registers::CTRL_REG3.addr(), config.int_ctrl_reg3())?;
        self.interface
            .write(Registers::CTRL_REG4.addr(), config.int_ctrl_reg4())?;
        self.interface
            .write(Registers::INTERRUPT_CFG.addr(), config.int_interrupt_cfg())?;
        Ok(())
    }

    // --- THE FOLLOWING SECTION COULD BE REMOVED --- 

    /*

    /// Configuration of the interrupt generation (enabled/disable)
    pub fn int_generation_enable(&mut self, flag: bool) -> Result<(), T::Error> {
        match flag {
            true => self.set_register_bit_flag(Registers::CTRL_REG1, Bitmasks::DIFF_EN),
            false => self.clear_register_bit_flag(Registers::CTRL_REG1, Bitmasks::DIFF_EN),
        }
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

    */

    // --- END OF THE SECTION THAT COULD BE REMOVED --- 


    /// Get all the flags from the INT_SOURCE register (NOTE: INT_SOURCE register is cleared by reading it)
    pub fn get_int_status(&mut self) -> Result<IntStatus, T::Error> {        
                
        let reg_value = self.read_register(Registers::INT_SOURCE)?;

        let status = IntStatus {
            /// Has any interrupt event been generated?
            interrupt_active: match reg_value & Bitmasks::IA {
                0 => false,
                _ => true,
            },
            /// Has low differential pressure event been generated?
            diff_press_low: match reg_value & Bitmasks::PL {
                0 => false,
                _ => true,
            },
            /// Has high differential pressure event been generated?
            diff_press_high: match reg_value & Bitmasks::PH {
                0 => false,
                _ => true,
            },           
        };

        /*
        let status = IntSource {
            /// Has any interrupt event been generated? (self clearing)
            interrupt_active: self.is_register_bit_flag_high(Registers::INT_SOURCE, Bitmasks::IA)?,
            /// Has low differential pressure event been generated? (self clearing)
            diff_press_low: self.is_register_bit_flag_high(Registers::INT_SOURCE, Bitmasks::PL)?,
            /// Has high differential pressure event been generated? (self clearing)
            diff_press_high: self.is_register_bit_flag_high(Registers::INT_SOURCE, Bitmasks::PH)?,
        };
        */
        Ok(status)
    }

    // --- THESE FUNCTIONS COULD BE REMOVED ---

    /*

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
     */
}
