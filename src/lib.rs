//! A driver for the TI INA260 magnetometer
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.1
//!
//! # Examples
//!
//! None

#![no_std]

use embedded_hal as hal;

use core::mem;

use crate::hal::blocking::i2c::{Write, WriteRead};
use cast::{i32, u16, u32};

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
    #[inline(always)]
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
    #[inline(always)]
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
    #[inline(always)]
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
    #[inline(always)]
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
    #[inline(always)]
    pub fn bits(self) -> u16 {
        self as u16
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
/// Mask/Enable Register
///
/// The Mask/Enable Register selects the function that is enabled to control the ALERT pin as well as how that pin
/// functions. If multiple functions are enabled, the highest significant bit position Alert Function (D15-D11) takes
/// priority and responds to the Alert Limit Register.
pub enum MaskEnable {
    /// Over Current Limit
    ///
    /// Setting this bit high configures the ALERT pin to be asserted if the current
    /// measurement following a conversion exceeds the value programmed in the Alert
    /// Limit Register.
    OCL = 0b1000_0000_0000_0000,
    /// Under Current Limit
    ///
    /// Setting this bit high configures the ALERT pin to be asserted if the current
    /// measurement following a conversion drops below the value programmed in the
    /// Alert Limit Register.
    UCL = 0b0100_0000_0000_0000,
    /// Bus Voltage Over-Voltage
    ///
    /// Setting this bit high configures the ALERT pin to be asserted if the bus voltage
    /// measurement following a conversion exceeds the value programmed in the Alert
    /// Limit Register.
    BOL = 0b0010_0000_0000_0000,
    /// Bus Voltage Under-Voltage
    ///
    /// Setting this bit high configures the ALERT pin to be asserted if the bus voltage
    /// measurement following a conversion drops below the value programmed in the
    /// Alert Limit Register.
    BUL = 0b0001_0000_0000_0000,
    /// Power Over-Limit
    ///
    /// Setting this bit high configures the ALERT pin to be asserted if the Power
    /// calculation made following a bus voltage measurement exceeds the value
    /// programmed in the Alert Limit Register.
    POL = 0b0000_1000_0000_0000,
    /// Conversion Ready
    ///
    /// Setting this bit high configures the ALERT pin to be asserted when the Conversion
    /// Ready Flag, Bit 3, is asserted indicating that the device is ready for the next
    /// conversion.
    CNVR = 0b0000_0100_0000_0000,
    /// Alert Function Flag
    ///
    /// While only one Alert Function can be monitored at the ALERT pin at a time, the
    /// Conversion Ready can also be enabled to assert the ALERT pin. Reading the Alert
    /// Function Flag following an alert allows the user to determine if the Alert Function
    /// was the source of the Alert.
    ///
    /// When the Alert Latch Enable bit is set to Latch mode, the Alert Function Flag bit
    /// clears only when the Mask/Enable Register is read. When the Alert Latch Enable
    /// bit is set to Transparent mode, the Alert Function Flag bit is cleared following the
    /// next conversion that does not result in an Alert condition.
    AFF = 0b0000_0000_0001_0000,
    /// Conversion Ready
    ///
    /// Although the device can be read at any time, and the data from the last conversion
    /// is available, the Conversion Ready Flag bit is provided to help coordinate one-shot
    /// or triggered conversions. The Conversion Ready Flag bit is set after all
    /// conversions, averaging, and multiplications are complete. Conversion Ready Flag
    /// bit clears under the following conditions:
    ///
    /// 1.) Writing to the Configuration Register (except for Power-Down selection)
    /// 2.) Reading the Mask/Enable Register
    CVRF = 0b0000_0000_0000_1000,
    /// Math Overflow Flag
    ///
    /// This bit is set to '1' if an arithmetic operation resulted in an overflow error. It
    /// indicates that power data may have exceeded the maximum reportable value of
    /// 419.43 W.
    OVF = 0b0000_0000_0000_0100,
    /// Alert Polarity bit
    ///
    /// 1 = Inverted (active-high open collector)
    /// 0 = Normal (active-low open collector) (default)
    APOL = 0b0000_0000_0000_0010,
    /// Alert Latch Enable; configures the latching feature of the ALERT pin and Alert Flag
    /// bits.
    ///
    /// 1 = Latch enabled
    /// 0 = Transparent (default)
    ///
    /// When the Alert Latch Enable bit is set to Transparent mode, the ALERT pin and
    /// Flag bit resets to the idle states when the fault has been cleared. When the Alert
    /// Latch Enable bit is set to Latch mode, the ALERT pin and Alert Flag bit remains
    /// active following a fault until the Mask/Enable Register has been read.
    LEN = 0b0000_0000_0000_0001,
}

impl MaskEnable {
    #[inline(always)]
    pub fn bits(self) -> u16 {
        self as u16
    }
}

#[inline(always)]
fn write_register<I2C>(
    i2c: &mut I2C,
    address: u8,
    reg: Register,
    data: u16,
) -> Result<(), I2C::Error>
where
    I2C: Write,
{
    i2c.write(
        address,
        &[reg.addr(), (data >> 8) as u8, (data & 255) as u8],
    )
}

pub struct INA260NonOwned<I2C> {
    address: u8,
    state: u16,
    _marker: core::marker::PhantomData<I2C>,
}

impl<I2C, E> INA260NonOwned<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Add a new driver for a INA260 chip found on the I2C bus at the specified address
    #[inline(always)]
    pub fn new(i2c: &mut I2C, address: u8) -> Result<Self, E> {
        let ina260 = Self {
            _marker: core::marker::PhantomData,
            address,
            state: OperMode::SCBVC.bits()
                | Averaging::AVG1.bits()
                | SCConvTime::MS1_1.bits()
                | BVConvTime::MS1_1.bits(),
        };
        write_register(i2c, address, Register::CONFIG, 0x8000)?;
        Ok(ina260)
    }

    /// Change the Mask/Enable mode of the INA260
    ///
    /// The Mask/Enable Register selects the function that is enabled to control the ALERT pin as well as how that pin
    /// functions. If multiple functions are enabled, the highest significant bit position Alert Function (D15-D11) takes
    /// priority and responds to the Alert Limit Register.
    #[inline(always)]
    pub fn set_mask_enable(&mut self, i2c: &mut I2C, m: MaskEnable) -> Result<(), E> {
        write_register(i2c, self.address, Register::MASK_ENABLE, m.bits())?;
        Ok(())
    }

    /// Set the alert limit of the INA260
    ///
    /// The Alert Limit Register contains the value used to compare to the register selected in the Mask/Enable Register
    /// to determine if a limit has been exceeded. The format for this register will match the format of the register that is
    /// selected for comparison.
    #[inline(always)]
    pub fn set_alert_limit(&mut self, i2c: &mut I2C, limit: u16) -> Result<(), E> {
        write_register(i2c, self.address, Register::ALERT_LIMIT, limit)?;
        Ok(())
    }

    /// Change the averaging mode of the INA260
    #[inline(always)]
    pub fn set_averaging_mode(&mut self, i2c: &mut I2C, a: Averaging) -> Result<(), E> {
        let bits = a.bits();
        let state = (self.state & !Averaging::AVG1024.bits()) | bits;
        write_register(i2c, self.address, Register::CONFIG, state)?;
        self.state = state;
        Ok(())
    }

    /// Change the operating mode of the INA260. Please note that if you change to Triggered mode,
    /// you'll have to call this method again each time you would like to get a new sample.
    #[inline(always)]
    pub fn set_operating_mode(&mut self, i2c: &mut I2C, o: OperMode) -> Result<(), E> {
        let bits = o.bits();
        let state = (self.state & !OperMode::SCBVC.bits()) | bits;
        write_register(i2c, self.address, Register::CONFIG, state)?;
        self.state = state;
        Ok(())
    }

    /// Change the shut current conversion time
    #[inline(always)]
    pub fn set_scconvtime_mode(&mut self, i2c: &mut I2C, s: SCConvTime) -> Result<(), E> {
        let bits = s.bits();
        let state = (self.state & !SCConvTime::MS8_244.bits()) | bits;
        write_register(i2c, self.address, Register::CONFIG, state)?;
        self.state = state;
        Ok(())
    }

    /// Change the bus voltage conversion time
    #[inline(always)]
    pub fn set_bvconvtime_mode(&mut self, i2c: &mut I2C, b: BVConvTime) -> Result<(), E> {
        let bits = b.bits();
        let state = (self.state & !BVConvTime::MS8_244.bits()) | bits;
        write_register(i2c, self.address, Register::CONFIG, state)?;
        self.state = state;
        Ok(())
    }

    /// Delivers the unique chip id
    #[inline(always)]
    pub fn did(&mut self, i2c: &mut I2C) -> Result<u16, E> {
        let mut buffer: [u8; 2] = unsafe { mem::uninitialized() };
        i2c.write_read(self.address, &[Register::DIE_ID.addr()], &mut buffer)?;

        Ok((u16(buffer[0]) << 8 | u16(buffer[1])) >> 4)
    }

    /// Delivers the die revision id
    #[inline(always)]
    pub fn rid(&mut self, i2c: &mut I2C) -> Result<u16, E> {
        let mut buffer: [u8; 2] = unsafe { mem::uninitialized() };
        i2c.write_read(self.address, &[Register::DIE_ID.addr()], &mut buffer)?;

        Ok(u16(buffer[1]) & 0b1111)
    }

    /// Delivers the measured raw current in 1.25mA per bit
    #[inline(always)]
    pub fn current_raw(&mut self, i2c: &mut I2C) -> Result<i16, E> {
        let mut buffer: [u8; 2] = unsafe { mem::uninitialized() };
        i2c.write_read(self.address, &[Register::CURRENT.addr()], &mut buffer)?;

        Ok((u16(buffer[0]) << 8 | u16(buffer[1])) as i16)
    }

    /// Delivers the measured current in uA
    #[inline(always)]
    pub fn current(&mut self, i2c: &mut I2C) -> Result<i32, E> {
        let raw = self.current_raw(i2c)?;
        Ok(i32(raw) * 1250)
    }

    /// Delivers the measured current in as tuple of full volts and tenth millivolts
    #[inline(always)]
    pub fn current_split(&mut self, i2c: &mut I2C) -> Result<(i8, u32), E> {
        let raw = i32::from(self.current_raw(i2c)?);
        if raw >= 0 {
            let full = (0..=raw).step_by(800).skip(1).count() as i32;
            let rest = (raw - (full * 800)) * 125;
            Ok((full as i8, rest as u32))
        } else {
            let full = -((raw..=0).step_by(800).skip(1).count() as i32);
            let rest = -(raw - (full * 800)) * 125;
            Ok((full as i8, rest as u32))
        }
    }

    /// Delivers the measured raw voltage in 1.25mV per bit
    #[inline(always)]
    pub fn voltage_raw(&mut self, i2c: &mut I2C) -> Result<u16, E> {
        let mut buffer: [u8; 2] = unsafe { mem::uninitialized() };
        i2c.write_read(self.address, &[Register::VOLTAGE.addr()], &mut buffer)?;

        Ok(u16(buffer[0]) << 8 | u16(buffer[1]))
    }

    /// Delivers the measured voltage in uV
    #[inline(always)]
    pub fn voltage(&mut self, i2c: &mut I2C) -> Result<u32, E> {
        let raw = self.voltage_raw(i2c)?;
        Ok(u32(raw) * 1250)
    }

    /// Delivers the measured voltage in as tuple of full volts and tenth millivolts
    #[inline(always)]
    pub fn voltage_split(&mut self, i2c: &mut I2C) -> Result<(u8, u32), E> {
        let raw = u32::from(self.voltage_raw(i2c)?);
        let full = (0..=raw).step_by(800).skip(1).count() as u32;
        let rest = (raw - (full * 800)) * 125;
        Ok((full as u8, rest))
    }

    /// Delivers the measured power in 10mW per bit
    #[inline(always)]
    pub fn power_raw(&mut self, i2c: &mut I2C) -> Result<u16, E> {
        let mut buffer: [u8; 2] = unsafe { mem::uninitialized() };
        i2c.write_read(self.address, &[Register::POWER.addr()], &mut buffer)?;

        Ok(u16(buffer[0]) << 8 | u16(buffer[1]))
    }

    /// Delivers the measured raw power in mW
    #[inline(always)]
    pub fn power(&mut self, i2c: &mut I2C) -> Result<u32, E> {
        let raw = self.power_raw(i2c)?;
        Ok(u32(raw) * 10)
    }

    /// Delivers the measured power in as tuple of full volts and tenth millivolts
    #[inline(always)]
    pub fn power_split(&mut self, i2c: &mut I2C) -> Result<(u8, u32), E> {
        let raw = u32::from(self.power_raw(i2c)?);
        let full = (0..=raw).step_by(100).skip(1).count() as u32;
        let rest = (raw - (full * 100)) * 1000;
        Ok((full as u8, rest))
    }
}

pub struct INA260<I2C>(I2C, INA260NonOwned<I2C>);

impl<I2C, E> INA260<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Add a new driver for a INA260 chip found on the I2C bus at the specified address
    #[inline(always)]
    pub fn new(mut i2c: I2C, address: u8) -> Result<Self, E> {
        let ina260 = INA260NonOwned::new(&mut i2c, address)?;
        Ok(Self(i2c, ina260))
    }

    /// Put the INA260 chip managed by the driver in shut down and release I2C resource
    #[inline(always)]
    pub fn release(mut self) -> I2C {
        let _ = self.set_operating_mode(OperMode::SHUTDOWN);
        self.0
    }

    /// Change the Mask/Enable mode of the INA260
    ///
    /// The Mask/Enable Register selects the function that is enabled to control the ALERT pin as well as how that pin
    /// functions. If multiple functions are enabled, the highest significant bit position Alert Function (D15-D11) takes
    /// priority and responds to the Alert Limit Register.
    #[inline(always)]
    pub fn set_mask_enable(&mut self, m: MaskEnable) -> Result<(), E> {
        self.1.set_mask_enable(&mut self.0, m)
    }

    /// Set the alert limit of the INA260
    ///
    /// The Alert Limit Register contains the value used to compare to the register selected in the Mask/Enable Register
    /// to determine if a limit has been exceeded. The format for this register will match the format of the register that is
    /// selected for comparison.
    #[inline(always)]
    pub fn set_alert_limit(&mut self, l: u16) -> Result<(), E> {
        self.1.set_alert_limit(&mut self.0, l)
    }

    /// Change the averaging mode of the INA260
    #[inline(always)]
    pub fn set_averaging_mode(&mut self, a: Averaging) -> Result<(), E> {
        self.1.set_averaging_mode(&mut self.0, a)
    }

    /// Change the operating mode of the INA260. Please note that if you change to Triggered mode,
    /// you'll have to call this method again each time you would like to get a new sample.
    #[inline(always)]
    pub fn set_operating_mode(&mut self, o: OperMode) -> Result<(), E> {
        self.1.set_operating_mode(&mut self.0, o)
    }

    /// Change the shut current conversion time
    #[inline(always)]
    pub fn set_scconvtime_mode(&mut self, s: SCConvTime) -> Result<(), E> {
        self.1.set_scconvtime_mode(&mut self.0, s)
    }

    /// Change the bus voltage conversion time
    #[inline(always)]
    pub fn set_bvconvtime_mode(&mut self, b: BVConvTime) -> Result<(), E> {
        self.1.set_bvconvtime_mode(&mut self.0, b)
    }

    /// Delivers the unique chip id
    #[inline(always)]
    pub fn did(&mut self) -> Result<u16, E> {
        self.1.did(&mut self.0)
    }

    /// Delivers the die revision id
    #[inline(always)]
    pub fn rid(&mut self) -> Result<u16, E> {
        self.1.rid(&mut self.0)
    }

    /// Delivers the measured raw current in 1.25mA per bit
    #[inline(always)]
    pub fn current_raw(&mut self) -> Result<i16, E> {
        self.1.current_raw(&mut self.0)
    }

    /// Delivers the measured current in uA
    #[inline(always)]
    pub fn current(&mut self) -> Result<i32, E> {
        self.1.current(&mut self.0)
    }

    /// Delivers the measured current in as tuple of full volts and tenth millivolts
    #[inline(always)]
    pub fn current_split(&mut self) -> Result<(i8, u32), E> {
        self.1.current_split(&mut self.0)
    }

    /// Delivers the measured raw voltage in 1.25mV per bit
    #[inline(always)]
    pub fn voltage_raw(&mut self) -> Result<u16, E> {
        self.1.voltage_raw(&mut self.0)
    }

    /// Delivers the measured voltage in uV
    #[inline(always)]
    pub fn voltage(&mut self) -> Result<u32, E> {
        self.1.voltage(&mut self.0)
    }

    /// Delivers the measured voltage in as tuple of full volts and tenth millivolts
    #[inline(always)]
    pub fn voltage_split(&mut self) -> Result<(u8, u32), E> {
        self.1.voltage_split(&mut self.0)
    }

    /// Delivers the measured power in 10mW per bit
    #[inline(always)]
    pub fn power_raw(&mut self) -> Result<u16, E> {
        self.1.power_raw(&mut self.0)
    }

    /// Delivers the measured raw power in mW
    #[inline(always)]
    pub fn power(&mut self) -> Result<u32, E> {
        self.1.power(&mut self.0)
    }

    /// Delivers the measured power in as tuple of full volts and tenth millivolts
    #[inline(always)]
    pub fn power_split(&mut self) -> Result<(u8, u32), E> {
        self.1.power_split(&mut self.0)
    }
}
