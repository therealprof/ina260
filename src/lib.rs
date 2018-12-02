//! A driver for the TI INA260 magnetometer
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.1
//!
//! # Examples
//!
//! None

#![deny(warnings)]
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;

use core::mem;

use cast::{i32, u16, u32};
use hal::blocking::i2c::{Write, WriteRead};

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
pub enum Register {
    // Configuration Register
    CONFIG = 0x00,
    // Contains the value of the current flowing through the shunt resistor
    CURRENT = 0x01,
    // Bus voltage measurement data
    VOLTAGE = 0x02,
    // Contains the value of the calculated power being delivered to the load
    POWER = 0x03,
    // Alert configuration and conversion ready flag
    MASK_ENABLE = 0x06,
    // Contains the limit value to compare to the selected alert function
    ALERT_LIMIT = 0x07,
    // Contains unique manufacturer identification number
    MANUFACTURER_ID = 0xFE,
    // Contains unique die identification number
    DIE_ID = 0xFF,
}

impl Register {
    pub fn addr(self) -> u8 {
        self as u8
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
/// Averaging Mode
/// Determines the number of samples that are collected and averaged.
pub enum Averaging {
    // No averaging (default)
    AVG1 = 0b0000_0000_0000_0000,
    // 4 times averaging
    AVG4 = 0b0000_0010_0000_0000,
    // 16 times averaging
    AVG16 = 0b0000_0100_0000_0000,
    // 64 times averaging
    AVG64 = 0b0000_0110_0000_0000,
    // 128 times averaging
    AVG128 = 0b0000_1000_0000_0000,
    // 256 times averaging
    AVG256 = 0b0000_1010_0000_0000,
    // 512 times averaging
    AVG512 = 0b0000_1100_0000_0000,
    // 1024 times averaging
    AVG1024 = 0b0000_1110_0000_0000,
}

impl Averaging {
    pub fn bits(self) -> u16 {
        self as u16
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
/// Bus Voltage Conversion Time
/// Sets the conversion time for the bus voltage measurement
pub enum BVConvTime {
    // Conversion time = 140 µs
    US140 = 0b0000_0000_0000_0000,
    // Conversion time = 204 µs
    US204 = 0b0000_0000_0100_0000,
    // Conversion time = 332 µs
    US332 = 0b0000_0000_1000_0000,
    // Conversion time = 588 µs
    US588 = 0b0000_0000_1100_0000,
    // Conversion time = 1.1 ms (default)
    MS1_1 = 0b0000_0001_0000_0000,
    // Conversion time = 2.116 ms
    MS2_116 = 0b0000_0001_0100_0000,
    // Conversion time = 4.156 ms
    MS4_156 = 0b0000_0001_1000_0000,
    // Conversion time = 8.244 ms
    MS8_244 = 0b0000_0001_1100_0000,
}

impl BVConvTime {
    pub fn bits(self) -> u16 {
        self as u16
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
/// Shunt Current Conversion Time
/// Sets the conversion time for the shunt current measurement
pub enum SCConvTime {
    // Conversion time = 140 µs
    US140 = 0b0000_0000_0000_0000,
    // Conversion time = 204 µs
    US204 = 0b0000_0000_0000_1000,
    // Conversion time = 332 µs
    US332 = 0b0000_0000_0001_0000,
    // Conversion time = 588 µs
    US588 = 0b0000_0000_0001_1000,
    // Conversion time = 1.1 ms (default)
    MS1_1 = 0b0000_0000_0010_0000,
    // Conversion time = 2.116 ms
    MS2_116 = 0b0000_0000_0010_1000,
    // Conversion time = 4.156 ms
    MS4_156 = 0b0000_0000_0011_0000,
    // Conversion time = 8.244 ms
    MS8_244 = 0b0000_0000_0011_1000,
}

impl SCConvTime {
    pub fn bits(self) -> u16 {
        self as u16
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
/// Operating Mode
/// Selects continuous, triggered, or power-down mode of operation.
pub enum OperMode {
    // Power-Down (or Shutdown)
    SHUTDOWN = 0b0000_0000_0000_0000,
    // = Shunt Current, Triggered
    SCT = 0b0000_0000_0000_0001,
    // = Shunt Current, Triggered
    BVT = 0b0000_0000_0000_0010,
    // = Shunt Current + Bus Voltage, Triggered
    SCBVT = 0b0000_0000_0000_0011,
    // = Shunt Current, Continuous
    SCC = 0b0000_0000_0000_0101,
    // = Bus Voltage, Continuous
    BVC = 0b0000_0000_0000_0110,
    // = Shunt Current + Bus Voltage, Continuous (default)
    SCBVC = 0b0000_0000_0000_0111,
}

impl OperMode {
    pub fn bits(self) -> u16 {
        self as u16
    }
}

pub struct INA260<I2C> {
    i2c: I2C,
    address: u8,
    state: u16,
}

impl<I2C, E> INA260<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Add a new driver for a INA260 chip found on the I2C bus at the specified address
    pub fn new(i2c: I2C, address: u8) -> Result<Self, E> {
        let mut ina260 = Self {
            i2c,
            address,
            state: OperMode::SCBVC.bits()
                | Averaging::AVG1.bits()
                | SCConvTime::MS1_1.bits()
                | BVConvTime::MS1_1.bits(),
        };
        let _ = ina260.write_register(Register::CONFIG, 0x8000);
        Ok(ina260)
    }

    /// Put the INA260 chip managed by the driver in shut down and release I2C resource
    pub fn release(mut self) -> I2C {
        let _ = self.set_operating_mode(OperMode::SHUTDOWN);
        self.i2c
    }

    /// Change the averaging mode of the INA260
    pub fn set_averaging_mode(&mut self, a: Averaging) -> Result<(), E> {
        let bits = a.bits();
        let state = (self.state & !Averaging::AVG1.bits()) | bits;
        self.write_register(Register::CONFIG, state)?;
        self.state = state;
        Ok(())
    }

    /// Change the operating mode of the INA260. Please note that if you change to Triggered mode,
    /// you'll have to call this method again each time you would like to get a new sample.
    pub fn set_operating_mode(&mut self, o: OperMode) -> Result<(), E> {
        let bits = o.bits();
        let state = (self.state & !OperMode::SHUTDOWN.bits()) | bits;
        self.write_register(Register::CONFIG, state)?;
        self.state = state;
        Ok(())
    }

    /// Change the shut current conversion time
    pub fn set_scconvtime_mode(&mut self, s: SCConvTime) -> Result<(), E> {
        let bits = s.bits();
        let state = (self.state & !SCConvTime::US140.bits()) | bits;
        self.write_register(Register::CONFIG, state)?;
        self.state = state;
        Ok(())
    }

    /// Change the bus voltage conversion time
    pub fn set_bvconvtime_mode(&mut self, b: BVConvTime) -> Result<(), E> {
        let bits = b.bits();
        let state = (self.state & !BVConvTime::US140.bits()) | bits;
        self.write_register(Register::CONFIG, state)?;
        self.state = state;
        Ok(())
    }

    /// Delivers the unique chip id
    pub fn did(&mut self) -> Result<u16, E> {
        let mut buffer: [u8; 2] = unsafe { mem::uninitialized() };
        self.i2c
            .write_read(self.address, &[Register::DIE_ID.addr()], &mut buffer)?;

        Ok((u16(buffer[0]) << 8 | u16(buffer[1])) >> 4)
    }

    /// Delivers the die revision id
    pub fn rid(&mut self) -> Result<u16, E> {
        let mut buffer: [u8; 2] = unsafe { mem::uninitialized() };
        self.i2c
            .write_read(self.address, &[Register::DIE_ID.addr()], &mut buffer)?;

        Ok(u16(buffer[1]) & 0b1111)
    }

    /// Delivers the measured raw current in 1.25mA per bit
    pub fn current_raw(&mut self) -> Result<i16, E> {
        let mut buffer: [u8; 2] = unsafe { mem::uninitialized() };
        self.i2c
            .write_read(self.address, &[Register::CURRENT.addr()], &mut buffer)?;

        Ok((u16(buffer[0]) << 8 | u16(buffer[1])) as i16)
    }

    /// Delivers the measured current in uA
    pub fn current(&mut self) -> Result<i32, E> {
        let raw = self.current_raw()?;
        Ok(i32(raw) * 1250)
    }

    /// Delivers the measured current in as tuple of full volts and tenth millivolts
    pub fn current_split(&mut self) -> Result<(i8, u32), E> {
        let raw = i32::from(self.current_raw()?);
        if raw >= 0 {
            let full = (0..=raw).step_by(800).skip(1).count() as i32;
            let rest = (raw - (full * 800)) * 125;
            Ok((full as i8, rest as u32))
        } else
        {
            let full = -((raw..=0).step_by(800).skip(1).count() as i32);
            let rest = -(raw - (full * 800)) * 125;
            Ok((full as i8, rest as u32))
        }
    }

    /// Delivers the measured raw voltage in 1.25mV per bit
    pub fn voltage_raw(&mut self) -> Result<u16, E> {
        let mut buffer: [u8; 2] = unsafe { mem::uninitialized() };
        self.i2c
            .write_read(self.address, &[Register::VOLTAGE.addr()], &mut buffer)?;

        Ok(u16(buffer[0]) << 8 | u16(buffer[1]))
    }

    /// Delivers the measured voltage in uV
    pub fn voltage(&mut self) -> Result<u32, E> {
        let raw = self.voltage_raw()?;
        Ok(u32(raw) * 1250)
    }

    /// Delivers the measured voltage in as tuple of full volts and tenth millivolts
    pub fn voltage_split(&mut self) -> Result<(u8, u32), E> {
        let raw = u32::from(self.voltage_raw()?);
        let full = (0..=raw).step_by(800).skip(1).count() as u32;
        let rest = (raw - (full * 800)) * 125;
        Ok((full as u8, rest))
    }

    /// Delivers the measured power in 10mW per bit
    pub fn power_raw(&mut self) -> Result<u16, E> {
        let mut buffer: [u8; 2] = unsafe { mem::uninitialized() };
        self.i2c
            .write_read(self.address, &[Register::POWER.addr()], &mut buffer)?;

        Ok(u16(buffer[0]) << 8 | u16(buffer[1]))
    }

    /// Delivers the measured raw power in mW
    pub fn power(&mut self) -> Result<u32, E> {
        let raw = self.power_raw()?;
        Ok(u32(raw) * 10)
    }

    /// Delivers the measured power in as tuple of full volts and tenth millivolts
    pub fn power_split(&mut self) -> Result<(u8, u32), E> {
        let raw = u32::from(self.power_raw()?);
        let full = (0..=raw).step_by(100).skip(1).count() as u32;
        let rest = (raw - (full * 100)) * 1000;
        Ok((full as u8, rest))
    }

    fn write_register(&mut self, reg: Register, data: u16) -> Result<(), E> {
        self.i2c.write(
            self.address,
            &[reg.addr(), (data >> 8) as u8, (data & 255) as u8],
        )
    }
}
