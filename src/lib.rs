use micro_rdk::common::config::ConfigType;
use micro_rdk::common::registry::{
    get_board_from_dependencies, ComponentRegistry, Dependency, RegistryError,
};
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::DoCommand;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
// use core::iter;
// use byteorder::ByteOrder as _;
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


// REG0x01CTRL1 Bts
pub const CTRL1_GAIN: u8 = 2;
pub const CTRL1_VLDO: u8 = 5;
pub const CTRL1_DRDY_SEL: u8 = 6;
pub const CTRL1_CRP: u8 = 7; 

// REG 0x02 CTRL2 Bits
pub const CTRL2_BIT_CAL_MOD: u8 = 0;
pub const CTRL2_BIT_CALS: u8 = 2;
pub const CTRL2_BIT_CAL_ERROR: u8 = 3;
pub const CTRL2_BIT_CRS: u8 = 4;
pub const CTRL2_BIT_CHS: u8 = 7;

// PGA Bits
pub const PGA_BIT_CHP_DIS: u8 = 0;
pub const PGA_BIT_INV: u8 = 3;
pub const PGA_BIT_BYPASS_EN: u8 = 4;
pub const PGA_BIT_OUT_EN: u8 = 5;
pub const PGA_BIT_LDO_MODE: u8 = 6;
pub const PGA_BIT_RD_OTP_SEL: u8 = 7;

// PGAPWR Bits
pub const PGA_PWR_BIT_CURR: u8 = 0;
pub const PGA_PWR_BIT_ADC_CURR: u8 = 2;
pub const PGA_PWR_BIT_MSTR_BIAS_CURR: u8 = 4;
pub const PGA_PWR_BIT_CAP_EN: u8 = 7;



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
    // ldo: u8,
    // gain: u8,
    // sps: u8,
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
            // ldo,
            // gain,
            // sps,
        };

        let r = sensor.rw(PU_CTRL);
        log::info!("1 got {:02X}", r.unwrap());
        sensor.ww(PU_CTRL, 0x1 )?;
        log::warn!("started reset");
        thread::sleep(time::Duration::from_millis(1));        
        
        let r = sensor.rw(PU_CTRL);
        log::info!("2 got {:02X}", r.unwrap());
        
        let next : u8= 1 << 1;
        sensor.ww(PU_CTRL, next )?;
        thread::sleep(time::Duration::from_millis(1));        
        let r = sensor.rw(PU_CTRL).unwrap();
        log::info!("3 got {:02X}", r);

        let next = r | 1<<2 ;
        sensor.ww(PU_CTRL, next )?;
        thread::sleep(time::Duration::from_millis(1));        
        let r = sensor.rw(PU_CTRL).unwrap();
        log::info!("3 got {:02X}", r);

        let next = 1<< 1 | 1<<2 | 1<<7 ;
        sensor.ww(PU_CTRL, next )?;
        thread::sleep(time::Duration::from_millis(1));        
        let r = sensor.rw(PU_CTRL).unwrap();
        log::info!("3 got {:02X}", r);

        let next : u8 = sensor.rw(PU_CTRL).unwrap();
        let next = next | 1<<4;
        sensor.ww(PU_CTRL, next )?;
        thread::sleep(time::Duration::from_millis(1));        
        let r = sensor.rw(PU_CTRL);
        log::info!("7 got {:02X}", r.unwrap());
        

        let next : u8 =  1<<0 | 1<<1 | 1<<2| 1<<5|1<<3 ;
        sensor.ww(CTRL1, next )?;
        thread::sleep(time::Duration::from_millis(1));        
        let r = sensor.rw(CTRL1);
        log::info!("4 got {:02X}", r.unwrap());

        let next = 0x30;
        sensor.ww(ADC, next )?;
        thread::sleep(time::Duration::from_millis(1));        
        let r = sensor.rw(ADC);
        log::info!("7 got {:02X}", r.unwrap());

        let next =  1<<6;
        sensor.ww(PGA, next )?;
        thread::sleep(time::Duration::from_millis(1));        
        let r = sensor.rw(PGA).unwrap();
        log::info!("3 got {:02X}", r);


        for _ in 1..100 {
            let r = sensor.rw(PU_CTRL).unwrap();
            if r & 1<<5 == 1 <<5{
            let r = sensor.rw2(ADCO_B2, 3)?;
            let mut r2:i32 = (r[0] as i32) << 16 | (r[1] as i32) << 8 |(r[2] as i32);
            if r2 & 1<<23 == 1<<23 {
                r2 = r2 |(0xFF000000u32 as i32);
            }
            
            //log::info!("READ {}", r2);
            }     
            thread::sleep(time::Duration::from_millis(5));   
        }

        let next : u8 = 1 <<5;
        sensor.ww(CTRL2, next )?;
        thread::sleep(time::Duration::from_millis(1));        
        let r = sensor.rw(CTRL2);
        log::info!("6 got {:02X}", r.unwrap());

        
        let next : u8 = 1 <<5 |1 <<2;
        sensor.ww(CTRL2, next )?;
        thread::sleep(time::Duration::from_millis(1000));        
        let r = sensor.rw(CTRL2);
        log::info!("6 got {:02X}", r.unwrap());


        let next : u8 = 1 <<5 |1 <<2 | 1 << 1;
        sensor.ww(CTRL2, next )?;
        thread::sleep(time::Duration::from_millis(1000));        
        let r = sensor.rw(CTRL2);
        log::info!("6 got {:02X}", r.unwrap());

        let r = sensor.rw(PU_CTRL);
        log::info!("7 got {:02X}", r.unwrap());

        for _ in 1..100 {
            let r = sensor.rw(PU_CTRL).unwrap();
            if r & 1<<5 == 1 <<5{
            let r = sensor.rw2(ADCO_B2, 3)?;
            let mut r2:i32 = (r[0] as i32) << 16 | (r[1] as i32) << 8 |(r[2] as i32);
            if r2 & 1<<23 == 1<<23 {
                r2 = r2 |(0xFF000000u32 as i32);
            }
            
            log::info!("READ {}", r2);
            }     
            thread::sleep(time::Duration::from_millis(5));   
        }

        Ok(sensor)
    }

    pub fn close(&mut self) -> Result<(), SensorError> {
        // reset the device again, is there a standby bit?
        // reset the device again, is there a standby bit?
        
        let next : u8 = 1 << PU_CTRL_BIT_PUA;
        self.ww(PU_CTRL, next )?;
        let next : u8 = 1 << PU_CTRL_BIT_PUD;
        self.ww(PU_CTRL, next )?;

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


    pub fn rw(&mut self, address_reg: u8) -> Result<u8, I2CErrors> {
        let mut buf: [u8;1] =[0xFF];
        let r = self.i2c_handle.write_read_i2c(NAU7802_I2C_ADDRESS,&[address_reg],&mut buf);
        Ok(buf[0])
    }

    pub fn rw2(&mut self, address_reg: u8, len: u8) -> Result<Vec<u8>, I2CErrors> {
        let mut buf = vec![0xFF;len.into()];
        let r = self.i2c_handle.write_read_i2c(NAU7802_I2C_ADDRESS,&[address_reg],&mut buf);
        Ok(buf)
    }

    pub fn ww(&mut self, addr: u8, payload: u8) -> Result<(), I2CErrors> {
        let buf: [u8;2] = [addr, payload];
        let r = self.i2c_handle.write_i2c(NAU7802_I2C_ADDRESS, &buf);

        Ok(())
    }

    pub fn rwinlock(&self, address_reg: u8) -> Result<u8, I2CErrors> {
        let mut buf: [u8;1] =[0xFF];
        let r = self.i2c_handle.lock().unwrap().write_read_i2c(NAU7802_I2C_ADDRESS,&[address_reg],&mut buf);
        Ok(buf[0])
    }
    
    pub fn rw2inlock(&self, address_reg: u8, len: u8) -> Result<Vec<u8>, I2CErrors> {
        let mut buf = vec![0xFF;len.into()];
        let r = self.i2c_handle.lock().unwrap().write_read_i2c(NAU7802_I2C_ADDRESS,&[address_reg],&mut buf);
        Ok(buf)
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
        let r = self.rwinlock(PU_CTRL).unwrap();
        let mut r2:i32 =0 ;
        if r & 1<<5 == 1 <<5{
        let r = self.rw2inlock(ADCO_B2, 3)?;
        r2 = (r[0] as i32) << 16 | (r[1] as i32) << 8 |(r[2] as i32);
        if r2 & 1<<23 == 1<<23 {
            r2 = r2 |(0xFF000000u32 as i32);
        }
        
        log::info!("READ {}", r2);
        }     
        thread::sleep(time::Duration::from_millis(5));   
        
                
        let mut readings = HashMap::new();
        readings.insert("raw_data".to_string(), r2 as f64); // does this even work, do I have to convert?
                                                                  //log::debug!("getting raw data from nau7802 succeeded!");

        // Store scaled weight in kilograms
        let scaled_reading = (r2 as f64) * self.scale_to_kg;
        readings.insert("weight_kg".to_string(), scaled_reading);

        Ok(readings)
    }
}