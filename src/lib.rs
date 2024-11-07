use micro_rdk::common::config::ConfigType;
use micro_rdk::common::registry::{
    get_board_from_dependencies, ComponentRegistry, Dependency, RegistryError,
};
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::DoCommand;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use core::iter;
use byteorder::ByteOrder as _;
use std::{thread, time};

// correct crates?
use micro_rdk::common::board::Board;
use micro_rdk::common::i2c::{I2CErrors, I2CHandle, I2cHandleType};
use micro_rdk::common::sensor::{
    GenericReadingsResult, Readings, Sensor, SensorError, SensorResult, SensorT, SensorType,
    TypedReadingsResult,
};

pub fn register_models(registry: &mut ComponentRegistry) -> Result<(), RegistryError> {
    registry.register_sensor("nau_7802", &Nau7802::from_config)
}

const NAU7802_I2C_ADDRESS: u8 = 0x2A;

// NAU7802 Register Addresses
pub const PU_CTRL: u8 = 0x00;          // Power control register        
pub const CTRL1: u8 = 0x01;            // Control/config register #1    
pub const CTRL2: u8 = 0x02;            // Control/config register #2    
pub const OCAL1B2: u8 = 0x03;
pub const OCAL1B1: u8 = 0x04;
pub const OCAL1B0: u8 = 0x05;
pub const GCAL1B3: u8 = 0x06;
pub const GCAL1B2: u8 = 0x07;
pub const GCAL1B1: u8 = 0x08;
pub const GCAL1B0: u8 = 0x09;
pub const OCAL2_B2: u8 = 0x0A;
pub const OCAL2_B1: u8 = 0x0B;
pub const OCAL2_B0: u8 = 0x0C;
pub const GCAL2_B3: u8 = 0x0D;
pub const GCAL2_B2: u8 = 0x0E;
pub const GCAL2_B1: u8 = 0x0F;
pub const GCAL2_B0: u8 = 0x10;
pub const I2C_CONTROL: u8 = 0x11;
pub const ADCO_B2: u8 = 0x12;
pub const ADCO_B1: u8 = 0x13;
pub const ADCO_B0: u8 = 0x14;              
pub const ADC: u8 = 0x15;              // ADC / chopper control
pub const PGA: u8 = 0x1B;              // PGA control
pub const POWER: u8 = 0x1C;            // Power control
pub const REVISION_ID: u8 = 0x1F;      // Chip revision ID

// REG0x00 PU_CTRL Bits                DEFAULT             OTHERS
pub const PU_CTRL_BIT_RR: u8 = 0;   // Register Reset               0: Normal Operation 1: Reset all register except RR
pub const PU_CTRL_BIT_PUD: u8 = 1;  // Power Up Digital Circuit     0: 
pub const PU_CTRL_BIT_PUA: u8 = 2;
pub const PU_CTRL_BIT_PUR: u8 = 3;
pub const PU_CTRL_BIT_CS: u8 = 4;
pub const PU_CTRL_BIT_CR: u8 = 5;
pub const PU_CTRL_BIT_OSCS: u8 = 6;
pub const PU_CTRL_BIT_AVDDS: u8 = 7;


// REG0x01CTRL Bts

// PgaRegisterBits Constants
pub const PGA_BIT_CHP_DIS: u8 = 0;
pub const PGA_BIT_INV: u8 = 3;
pub const PGA_BIT_BYPASS_EN: u8 = 4;
pub const PGA_BIT_OUT_EN: u8 = 5;
pub const PGA_BIT_LDO_MODE: u8 = 6;
pub const PGA_BIT_RD_OTP_SEL: u8 = 7;

// PgaPwrRegisterBits Constants
pub const PGA_PWR_BIT_CURR: u8 = 0;
pub const PGA_PWR_BIT_ADC_CURR: u8 = 2;
pub const PGA_PWR_BIT_MSTR_BIAS_CURR: u8 = 4;
pub const PGA_PWR_BIT_CAP_EN: u8 = 7;

// Ctrl2RegisterBits Constants
pub const CTRL2_BIT_CAL_MOD: u8 = 0;
pub const CTRL2_BIT_CALS: u8 = 2;
pub const CTRL2_BIT_CAL_ERROR: u8 = 3;
pub const CTRL2_BIT_CRS: u8 = 4;
pub const CTRL2_BIT_CHS: u8 = 7;

// CTRL2 Calibration Status
#[derive(PartialEq)]
pub enum AfeCalibrationStatus {
    InProgress,
    Failure,
    Success,
}

// LDO Voltage Options
pub const LDO_4V5: u8 = 0;
pub const LDO_4V2: u8 = 1;
pub const LDO_3V9: u8 = 2;
pub const LDO_3V6: u8 = 3;
pub const LDO_3V3: u8 = 4;
pub const LDO_3V0: u8 = 5;
pub const LDO_2V7: u8 = 6;
pub const LDO_2V4: u8 = 7;
pub const LDO_EXTERNAL: u8 = 8;

// Gain Options
pub const GAIN_1: u8 = 0;
pub const GAIN_2: u8 = 1;
pub const GAIN_4: u8 = 2;
pub const GAIN_8: u8 = 3;
pub const GAIN_16: u8 = 4;
pub const GAIN_32: u8 = 5;
pub const GAIN_64: u8 = 6;
pub const GAIN_128: u8 = 7;

// Sample Rates
pub const SPS_RATE_10SPS: u8 = 0;
pub const SPS_RATE_20SPS: u8 = 1;
pub const SPS_RATE_40SPS: u8 = 2;
pub const SPS_RATE_80SPS: u8 = 3;
pub const SPS_RATE_320SPS: u8 = 7;

// Calibration Modes
pub const CALMOD_INTERNAL: u8 = 0;
pub const CALMOD_OFFSET: u8 = 2;
pub const CALMOD_GAIN: u8 = 3;

#[derive(DoCommand)]
pub struct Nau7802 {
    i2c_handle: I2cHandleType,
    i2c_address: u8,
    scale_to_kg: f64,
    ldo: u8,
    gain: u8,
    sps: u8,
}

impl Nau7802 {
    pub fn new(
        i2c_handle: I2cHandleType,
        i2c_address: u8,
        scale_to_kg: f64,
        ldo: u8,
        gain: u8,
        sps: u8,
    ) -> Result<Self, SensorError> {

        let mut sensor = Nau7802 {
            i2c_handle,
            i2c_address,
            scale_to_kg,
            ldo,
            gain,
            sps,
        };

        sensor.start_reset()?;
        // Sleep for 1 millisecond
        thread::sleep(time::Duration::from_millis(1));        
        sensor.finish_reset()?;
        sensor.power_up()?;
        sensor.set_ldo(ldo)?;
        sensor.set_gain(gain)?;
        sensor.set_sample_rate(sps)?;
        sensor.misc_init()?;
        sensor.begin_afe_calibration()?;

        while sensor.poll_afe_calibration_status()? != AfeCalibrationStatus::Success {}

        Ok(sensor)
    }

    pub fn close(&mut self) -> Result<(), SensorError> {
        // reset the device again, is there a standby bit?
        self.i2c_handle
            .write_i2c(NAU7802_I2C_ADDRESS, &[PU_CTRL, 0x01])?;
        Ok(())
    }

    pub fn from_config(cfg: ConfigType, deps: Vec<Dependency>) -> Result<SensorType, SensorError> {
        let board = get_board_from_dependencies(deps);
        if board.is_none() {
            return Err(SensorError::ConfigError("Nau7802 missing board attribute"));
        }
        let board_unwrapped = board.unwrap();
        let i2c_handle: I2cHandleType;

        if let Ok(i2c_name) = cfg.get_attribute::<String>("i2c_bus") {
            i2c_handle = board_unwrapped.get_i2c_by_name(i2c_name)?;
        } else {
            return Err(SensorError::ConfigError(
                "Nau7802 missing i2c_bus attribute in config",
            ));
        };

        let scale_to_kg = cfg.get_attribute::<f64>("scale_to_kg").unwrap_or(1.0); 
        let ldo = cfg.get_attribute::<u8>("ldo").unwrap_or(LDO_3V3); 
        let gain = cfg.get_attribute::<u8>("gain").unwrap_or(GAIN_128); 
        let sps = cfg.get_attribute::<u8>("sps").unwrap_or(SPS_RATE_10SPS); 
        // Default to 1.0 if not provided

        Ok(Arc::new(Mutex::new(Nau7802::new(
            i2c_handle,
            NAU7802_I2C_ADDRESS,
            scale_to_kg,
            ldo,
            gain,
            sps,
        )?)))
    }

    pub fn data_available(&mut self) -> Result<bool, I2CErrors> {
        self.get_bit(PU_CTRL, PU_CTRL_BIT_CR)
    }

    pub fn begin_afe_calibration(&mut self) -> Result<(), I2CErrors> {
        self.set_bit(CTRL2, CTRL2_BIT_CALS)
    }

    pub fn poll_afe_calibration_status(&mut self) -> Result<AfeCalibrationStatus, I2CErrors> {
        if self.get_bit(CTRL2, CTRL2_BIT_CALS)? {
            return Ok(AfeCalibrationStatus::InProgress);
        }

        if self.get_bit(CTRL2, CTRL2_BIT_CAL_ERROR)? {
            return Ok(AfeCalibrationStatus::Failure);
        }

        Ok(AfeCalibrationStatus::Success)
    }

    pub fn set_sample_rate(&mut self, sps: u8) -> Result<(), I2CErrors> {
        const SPS_MASK: u8 = 0b10001111;
        const SPS_START_BIT_IDX: u8 = 4;

        self.set_function_helper(CTRL2, SPS_MASK, SPS_START_BIT_IDX, sps as _)
    }

    pub fn set_gain(&mut self, gain: u8) -> Result<(), I2CErrors> {
        const GAIN_MASK: u8 = 0b11111000;
        const GAIN_START_BIT: u8 = 0;

        self.set_function_helper(CTRL1, GAIN_MASK, GAIN_START_BIT, gain as _)
    }

    pub fn set_ldo(&mut self, ldo: u8) -> Result<(), I2CErrors> {
        const LDO_MASK: u8 = 0b11000111;
        const LDO_START_BIT: u8 = 3;

        self.set_function_helper(CTRL1, LDO_MASK, LDO_START_BIT, ldo as _)?;

        self.set_bit(PU_CTRL, PU_CTRL_BIT_AVDDS)
    }

    pub fn power_up(&mut self) -> Result<(), I2CErrors> {
        const NUM_ATTEMPTS: usize = 100;

        self.set_bit(PU_CTRL, PU_CTRL_BIT_PUD)?;
        self.set_bit(PU_CTRL, PU_CTRL_BIT_PUA)?;

        let check_powered_up = || self.get_bit(PU_CTRL, PU_CTRL_BIT_PUR);

        let powered_up = iter::repeat_with(check_powered_up)
            .take(NUM_ATTEMPTS)
            .filter_map(Result::ok)
            .any(|rdy| rdy == true);

        if powered_up {
            Ok(())
        } else {
            Err(I2CErrors::I2CInvalidArgument("power up failed"))
        }
    }

    pub fn start_reset(&mut self) -> Result<(), I2CErrors> {
        self.set_bit(PU_CTRL, PU_CTRL_BIT_RR )
    }

    pub fn finish_reset(&mut self) -> Result<(), I2CErrors> {
        self.clear_bit(PU_CTRL, PU_CTRL_BIT_RR)
    }

    pub fn misc_init(&mut self) -> Result<(), I2CErrors> {
        const TURN_OFF_CLK_CHPL: u8 = 0x30;

        // Turn off CLK_CHP. From 9.1 power on sequencing
        self.set_register(ADC, TURN_OFF_CLK_CHPL)?;

        // Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note
        self.set_bit(PGA, PGA_PWR_BIT_CAP_EN)
    }

    pub fn set_function_helper(
        &mut self,
        reg: u8,
        mask: u8,
        start_idx: u8,
        new_val: u8,
    ) -> Result<(), I2CErrors> {
        let mut val = self.get_register(reg)?;
        val &= mask;
        val |= new_val << start_idx;

        self.set_register(reg, val)
    }

    pub fn set_bit(&mut self, addr: u8, bit_idx:u8) ->Result<(), I2CErrors>{
        let mut val = self.get_register(addr)?;
        val |= 1 << bit_idx;
        self.set_register(addr, val)
    }

    pub fn clear_bit(&mut self, addr: u8, bit_idx: u8) -> Result<(), I2CErrors>{
        let mut val = self.get_register(addr)?;
        val &= !(1 << bit_idx);
        self.set_register(addr, val)
    }

    pub fn get_bit(&mut self, addr: u8, bit_idx:u8) -> Result<bool, I2CErrors> {
        let mut val = self.get_register(addr)?;
        val &= 1 << bit_idx;
        Ok(val != 0)
    }

    pub fn set_register(&mut self, reg: u8, val:u8) -> Result<(), I2CErrors>{
        let transaction = [reg as _, val];

        self.i2c_handle.write_i2c(self.i2c_address, &transaction)
    }

    pub fn get_register(&mut self, reg: u8) -> Result<u8, I2CErrors> {
        self.request_register(reg)?;
        let val:u8 =0;
        self.i2c_handle.read_i2c(self.i2c_address,&mut [val]);
        Ok(val)
    }

    pub fn request_register(&mut self, reg: u8) -> Result<(), I2CErrors> {
        self.i2c_handle.write_i2c(self.i2c_address, &[reg])
    }
}

impl Status for Nau7802 {
    fn get_status(&self) -> Result<Option<micro_rdk::google::protobuf::Struct>, StatusError> {
        Ok(Some(micro_rdk::google::protobuf::Struct {
            fields: HashMap::new(),
        }))
    }
}

impl Sensor for Nau7802 {}

impl Readings for Nau7802 {
    fn get_generic_readings(&mut self) -> Result<GenericReadingsResult, SensorError> {
        Ok(self
            .get_readings()?
            .into_iter()
            .map(|v| (v.0, SensorResult::<f64> { value: v.1 }.into()))
            .collect())
    }
}

impl SensorT<f64> for Nau7802 {
    fn get_readings(&self) -> Result<TypedReadingsResult<f64>, SensorError> {
        let data_available = self.data_available()?;
    
        if !data_available {
                return Err(SensorError::SensorMethodUnimplemented("Data Unavaible"));
        }

        self.request_register(ADCO_B2)?;

        let mut buf = [0u8; 3]; // will hold an i24
        self.i2c_handle.read_i2c(self.i2c_address, & mut buf);

        let adc_result = byteorder::BigEndian::read_i24(&buf);

        let mut readings = HashMap::new();
        readings.insert("raw_data".to_string(), adc_result as f64); // does this even work, do I have to convert?
                                                                  //log::debug!("getting raw data from nau7802 succeeded!");

        // Store scaled weight in kilograms
        let scaled_reading = (adc_result as f64) * self.scale_to_kg;
        readings.insert("weight_kg".to_string(), scaled_reading);

        Ok(readings)
    }
}