//! TO DO: 
//! - CHECK IF ADDRESSES ARE CORRECT!!! - DONE
//! - CHECK BITMASKS
//! 
//! Register mapping

/// LPS25HB Registers
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
pub enum Registers {
    
    /// Reference pressure register.
    REF_P_XL        = 0x08,
    /// Reference pressure register.
    REF_P_L         = 0x09,
    /// Reference pressure register.
    REF_P_H         = 0x0A,
    /// Who Am I (identifies the chip).
    WHO_AM_I        = 0x0F,    
    /// Resolution configuration.
    RES_CONF        = 0x10,
    /// Control register 1.
    CTRL_REG1       = 0x20,
    /// Control register 2.
    CTRL_REG2       = 0x21,
    /// Control register 3.
    CTRL_REG3       = 0x22,
    /// Control register 4.
    CTRL_REG4       = 0x23,
    /// Interrupt control.
    INTERRUPT_CFG   = 0x24,
    /// Interrupt configuration.
    INT_SOURCE      = 0x25,
    /// Status register.
    STATUS_REG      = 0x27,
    /// Pressure output register.
    PRESS_OUT_XL    = 0x28,
    /// Pressure output register.
    PRESS_OUT_L     = 0x29,
    /// Pressure output register.
    PRESS_OUT_H     = 0x2A,
    /// Temperature output register.
    TEMP_OUT_L      = 0x2B,
    /// Temperature output register.
    TEMP_OUT_H      = 0x2C,
    /// FIFO configuration register.
    FIFO_CTRL       = 0x2E,
    /// FIFO status register.
    FIFO_STATUS     = 0x2F,
    /// Pressure threshold low.
    THS_P_L         = 0x30,
    /// Pressure threshold high.
    THS_P_H         = 0x31,    
    /// Pressure offset register.
    RPDS_L          = 0x39,
    /// Pressure offset register.
    RPDS_H          = 0x3A,
    
    
}

impl Registers {
    pub fn addr(self) -> u8 {
        self as u8
    }
}




/// LPS25HB Bit masks
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
pub enum Bitmasks {
    
    /// Enable single shot to acquire a new dataset
    ONE_SHOT    = 0b0000_0001,
}

impl Bitmasks {
    pub fn bitmask(self) -> u8 {
        self as u8
    }
}