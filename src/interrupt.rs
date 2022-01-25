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
        config: InterruptConfig,
    ) -> Result<(), T::Error> {
        match config.enable_differential {
            FLAG::Enabled => self.set_register_bit_flag(Registers::CTRL_REG1, Bitmasks::DIFF_EN),
            FLAG::Disabled => self.clear_register_bit_flag(Registers::CTRL_REG1, Bitmasks::DIFF_EN),
        }?;
        self.interface
            .write(Registers::CTRL_REG3.addr(), config.int_ctrl_reg3())?;
        self.interface
            .write(Registers::CTRL_REG4.addr(), config.int_ctrl_reg4())?;
        self.interface
            .write(Registers::INTERRUPT_CFG.addr(), config.int_interrupt_cfg())?;
        Ok(())
    }
 
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
        
        Ok(status)
    }
}
