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

  

}


impl Bitmasks {
    pub fn bitmask(self) -> u8 {
        self as u8
    }
}


 // === RES_CONF (0x10) ===
 pub const AVGT_MASK: u8 = 0b0000_1100;
 pub const AVGP_MASK: u8 = 0b0000_0011;

 // === CTRL_REG1 (0x20) ===
 /// Power down control
 pub const PD: u8 = 0b1000_0000;
 /// Output data rate selection
 pub const ODR_MASK: u8 = 0b0111_0000;
 pub const DIFF_EN: u8 = 0b0000_1000;
 pub const BDU: u8 = 0b0000_0100;
 pub const RESET_AZ: u8 = 0b0000_0010;
 pub const SIM: u8 = 0b0000_0001;

 // === CTRL_REG2 (0x21) ===
 pub const BOOT: u8 = 0b1000_0000;
 pub const FIFO_EN: u8 = 0b0100_0000;
 pub const STOP_ON_FTH: u8 = 0b0010_0000;
 pub const FIFO_MEAN_DEC: u8 = 0b0001_0000;
 pub const I2C_DIS: u8 = 0b0000_1000;
 pub const SWRESET: u8 = 0b0000_0100;
 pub const AUTOZERO: u8 = 0b0000_0010;
 /// Enable single shot to acquire a new dataset
 pub const ONE_SHOT: u8 = 0b0000_0001;

 // === CTRL_REG3 (0x22) ===
 pub const INT_H_L: u8 = 0b1000_0000;
 pub const PP_OD: u8 = 0b0100_0000;
 pub const INT_S_MASK: u8 = 0b0000_0011;

 // === CTRL_REG4 (0x23) ===
 pub const F_EMPTY: u8 = 0b0000_1000;
 pub const F_FTH: u8 = 0b0000_0100;
 pub const F_OVR: u8 = 0b0000_0010;
 pub const DRDY: u8 = 0b0000_0001;

 // === INTERRUPT_CFG (0x24) ===
 pub const LIR: u8 = 0b0000_0100;
 pub const PL_E: u8 = 0b0000_0010;
 pub const PH_E: u8 = 0b0000_0001;

 // === FIFO_CTRL (0x2E) ===
 pub const F_MODE_MASK: u8 = 0b1110_0000;
 pub const WTM_POINT_MASK: u8 = 0b0001_1111;

 // === INT_SOURCE (0x25) ===
 pub const IA: u8 = 0b0000_0100;
 pub const PL: u8 = 0b0000_0010;
 pub const PH: u8 = 0b0000_0001;

// === STATUS_REG (0x27) ===
pub const P_OR: u8 = 0b0010_0000;
pub const T_OR: u8 = 0b0001_0000;
pub const P_DA: u8 = 0b0000_0010;
pub const T_DA: u8 = 00000_b0001;

 // === MULTIBYTE READ ===
 pub const MULTIBYTE: u8 = 0b1000_0000;