//! TO DO:
//! - reference pressure reading
//! - pressure offset reading (what is the pressure offset for?)
//!
//! TO DO: the STATUS_REG could be read to a struct, with a single function
//! the struct would have four fields: 
//! pressure_overrun, temperature_overrun, 
//! pressure_data_available, temperature_data_available

use super::*;

/*
/// Pressure sensor settings. Use this struct to configure the sensor.
#[derive(Debug)]
pub struct SensorSettings {
    /// Output data rate & power mode selection
    pub sample_rate: ODR,
    //pub auto_addr_inc: bool,

}
*/

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

    /// Calculated pressure reading in hPa
    pub fn read_pressure(&mut self) -> Result<f32, T::Error> {
        let (p, _t) = self.read_sensor_raw()?;
        let pressure = (p as f32) / PRESS_SCALE; // no need to take care of negative values
        Ok(pressure)
    }

    /// Calculated temperaure reading in degrees Celsius
    pub fn read_temperature(&mut self) -> Result<f32, T::Error> {
        let (_p, t) = self.read_sensor_raw()?;
        // negative values taken care of, as the raw value is a signed 16-bit
        let temperature = (t as f32) / TEMP_SCALE + TEMP_OFFSET;
        Ok(temperature)
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

        self.interface
            .write(Registers::THS_P_L.addr() | Bitmasks::MULTIBYTE, payload[0])?;
        self.interface
            .write(Registers::THS_P_H.addr() | Bitmasks::MULTIBYTE, payload[1])?;

        Ok(())
    }

    /// Set the pressure offset value (VALUE IN hPA!)
    pub fn set_pressure_offset(&mut self, offset: u16) -> Result<(), T::Error> {
        let mut payload = [0u8; 2];
        let offset = offset * 16;

        payload[0] = (offset & 0xff) as u8; // lower byte
        payload[1] = (offset >> 8) as u8; // upper byte

        self.interface
            .write(Registers::RPDS_L.addr() | Bitmasks::MULTIBYTE, payload[0])?;
        self.interface
            .write(Registers::RPDS_H.addr() | Bitmasks::MULTIBYTE, payload[1])?;

        Ok(())
    }
  
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
}
