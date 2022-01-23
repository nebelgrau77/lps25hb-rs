//! TO DO:
//!
//! - reference pressure reading
//! - use MULTIBYTE from the interface (or introduce it directly in the interface)
//! - reference pressure setting

//! - split pressure and temperature reading, as reading it impacts the STATUS_REG values

use super::*;

#[derive(Debug)]
/// Contents of the STATUS_REG register (pressure and temperature overrun and data availability flags)
pub struct DataStatus {
    pub press_overrun: bool,
    pub temp_overrun: bool,
    pub press_available: bool,
    pub temp_available: bool,
}

impl<T, E> LPS25HB<T>
where
    T: Interface<Error = E>,
{
    /// Read the device ID ("who am I")
    pub fn get_device_id(&mut self) -> Result<u8, T::Error> {
        //pub fn get_device_id(&mut self) -> Result<u8, Error<E>> {
        let mut data = [0u8; 1];
        self.interface.read(Registers::WHO_AM_I.addr(), &mut data)?;
        let whoami = data[0];
        Ok(whoami)
    }

    /// Raw sensor reading (3 bytes of pressure data and 2 bytes of temperature data)
    
    // TO DO: split into separate pressure and temperature reading: otherwise it impacts the STATUS_REG

    /*

    fn read_sensor_raw(&mut self) -> Result<(i32, i16), T::Error> {
        let mut data = [0u8; 5];
        self.interface.read(
            Registers::PRESS_OUT_XL.addr() | Bitmasks::MULTIBYTE,
            &mut data,
        )?;
        let p: i32 = (data[2] as i32) << 16 | (data[1] as i32) << 8 | (data[0] as i32);
        let t: i16 = (data[4] as i16) << 8 | (data[3] as i16);
        Ok((p, t))
    }

     */

    /// Calculated pressure reading in hPa
    pub fn read_pressure(&mut self) -> Result<f32, T::Error> {
        let mut data = [0u8; 3];
        self.interface.read(
            Registers::PRESS_OUT_XL.addr() | Bitmasks::MULTIBYTE,
            &mut data,
        )?;
        let p: i32 = (data[2] as i32) << 16 | (data[1] as i32) << 8 | (data[0] as i32);
        let pressure = (p as f32) / PRESS_SCALE; // no need to take care of negative values
        Ok(pressure)
    }

    /// Calculated temperaure reading in degrees Celsius
    pub fn read_temperature(&mut self) -> Result<f32, T::Error> {
        let mut data = [0u8; 2];
        self.interface.read(
            Registers::TEMP_OUT_L.addr() | Bitmasks::MULTIBYTE,
            &mut data,
        )?;
        let t: i16 = (data[1] as i16) << 8 | (data[0] as i16);
        let temperature = (t as f32) / TEMP_SCALE + TEMP_OFFSET;
        Ok(temperature)
    }

    /*

    /// Calculated pressure reading in hPa
    pub fn read_pressure(&mut self) -> Result<f32, T::Error> {
        let (p, _t) = self.read_sensor_raw()?;
        let pressure = (p as f32) / PRESS_SCALE; // no need to take care of negative values
        Ok(pressure)
    }

     */

    /*

    /// Calculated temperaure reading in degrees Celsius
    pub fn read_temperature(&mut self) -> Result<f32, T::Error> {
        let (_p, t) = self.read_sensor_raw()?;
        // negative values taken care of, as the raw value is a signed 16-bit
        let temperature = (t as f32) / TEMP_SCALE + TEMP_OFFSET;
        Ok(temperature)
    }
    */

    /// Calculated reference pressure reading in hPa
    pub fn read_reference_pressure(&mut self) -> Result<f32, T::Error> {
        let mut data = [0u8; 3];
        self.interface.read(Registers::REF_P_XL.addr() | Bitmasks::MULTIBYTE, &mut data)?;
        let p: i32 = (data[2] as i32) << 16 | (data[1] as i32) << 8 | (data[0] as i32);
        let pressure: f32 = (p as f32) / PRESS_SCALE;
        Ok(pressure)
    }

    /// Read pressure offset value, 16-bit data that can be used to implement One-Point Calibration (OPC) after soldering.
    pub fn read_pressure_offset(&mut self) -> Result<i16, T::Error> {
        let mut data = [0u8; 2];
        self.interface
            .read(Registers::RPDS_L.addr() | Bitmasks::MULTIBYTE, &mut data)?;
        let o: i16 = (data[1] as i16) << 8 | (data[0] as i16);
        Ok(o)
    }

    /// Read threshold value for pressure interrupt generation
    pub fn read_threshold(&mut self) -> Result<i16, T::Error> {
        let mut data = [0u8; 2];
        self.interface
            .read(Registers::THS_P_L.addr() | Bitmasks::MULTIBYTE, &mut data)?;
        let ths: i16 = (data[1] as i16) << 8 | (data[0] as i16);
        // Ok(ths * 16) // this is wrong,
        Ok(ths / 16) // this will return value in hPa
    }

    /// Set threshold value for pressure interrupt generation (VALUE IN hPA!)
    pub fn set_threshold(&mut self, threshold: u16) -> Result<(), T::Error> {
        let mut payload = [0u8; 2];
        // The value is expressed as unsigned number: Interrupt threshold(hPA) = (THS_P)/16.
        let threshold = threshold * 16;

        payload[0] = (threshold & 0xff) as u8; // lower byte
        payload[1] = (threshold >> 8) as u8; // upper byte

        // this doesn't really need the multibyte, or it can be written in one go

        self.interface
            .write(Registers::THS_P_L.addr(), payload[0])?;
        self.interface
            .write(Registers::THS_P_H.addr(), payload[1])?;

        Ok(())
    }

    /// Set the pressure offset value (VALUE IN hPA!)
    pub fn set_pressure_offset(&mut self, offset: u16) -> Result<(), T::Error> {
        let mut payload = [0u8; 2];
        let offset = offset * 16;

        payload[0] = (offset & 0xff) as u8; // lower byte
        payload[1] = (offset >> 8) as u8; // upper byte

        self.interface
            .write(Registers::RPDS_L.addr(), payload[0])?;
        self.interface
            .write(Registers::RPDS_H.addr(), payload[1])?;

        Ok(())
    }

    /// Get all the flags from the STATUS_REG register
    pub fn get_data_status(&mut self) -> Result<DataStatus, T::Error> {
        // TO DO: use this value for reading all the bitflags in one go
        // use bitmasks
        let reg_value = self.read_register(Registers::STATUS_REG)?;

        let status = DataStatus {
            /// Has new pressure data overwritten the previous one?
            press_overrun: match reg_value & Bitmasks::P_OR {
                0 => false,
                _ => true,
            },
            /// Has new temperature data overwritten the previous one?
            temp_overrun: match reg_value & Bitmasks::T_OR {
                0 => false,
                _ => true,
            },
            /// Is new pressure data available?
            press_available: match reg_value & Bitmasks::P_DA {
                0 => false,
                _ => true,
            },
            /// Is new temperature data available?
            temp_available: match reg_value & Bitmasks::T_DA {
                0 => false,
                _ => true,
            },
        };

        /*
        let status = DataStatus {
            /// Has new pressure data overwritten the previous one?
            press_overrun: self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::P_OR)?,
            /// Has new temperature data overwritten the previous one?
            temp_overrun: self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::T_OR)?,
            /// Is new pressure data available?
            press_available: self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::P_DA)?,
            /// Is new temperature data available?
            temp_available: self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::T_DA)?,
        };
        */
        Ok(status)
    }

    /// Triggers a single measurement of pressure and temperature.
    /// Once the measurement is done, the ONE_SHOT bit will self-clear, the new data are available in the output registers,
    /// and the STATUS_REG bits are updated.
    pub fn one_shot(&mut self) -> Result<(), T::Error> {
        self.set_datarate(ODR::OneShot)?; // make sure that OneShot mode is enabled
        self.set_register_bit_flag(Registers::CTRL_REG2, Bitmasks::ONE_SHOT)?;
        Ok(())
    }

    // --- THESE FUNCTIONS COULD BE REMOVED ---

    /*
    /// Has new pressure data overwritten the previous one?
    pub fn pressure_data_overrun(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::P_OR)
    }

    /// Has new temperature data overwritten the previous one?
    pub fn temperature_data_overrun(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::T_OR)
    }

    /// Is new pressure data available?
    pub fn pressure_data_available(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::P_DA)
    }

    /// Is new temperature data available?
    pub fn temperature_data_available(&mut self) -> Result<bool, T::Error> {
        self.is_register_bit_flag_high(Registers::STATUS_REG, Bitmasks::T_DA)
    }
     */
}
