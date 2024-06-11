use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use micro_rdk::DoCommand;
use micro_rdk::common::config::ConfigType;
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::common::registry::{ComponentRegistry, RegistryError, Dependency};

// correct crates?
use micro_rdk::board::Board
use micro_rdk::i2c::I2CHandle

use micro_rdk::common::sensor::{Sensor, SensorType, Readings, SensorError};


pub fn register_models(registry: &mut ComponentRegistry) -> Result<(), RegistryError> {
    registry.register_sensor("my_sensor", &Nau7802::from_config)
}

const NAU7802_I2C_ADDRESS: u8 = 0x2A;

const NAU7802_REG_PU_CTRL: u8 = 0x00;
const NAU7802_REG_CTRL1: u8 = 0x01;
const NAU7802_REG_CTRL2: u8 = 0x02;
const NAU7802_REG_ADC: u8 = 0x12;

#[derive(DoCommand)]
pub struct Nau7802 {
    i2c_handle: I2CHandleType,
    i2c_address: u8
}

// TODOs:
// Tare command from DoCommand to reset scale
// Configuration attibute for max weight and add a reading element to return the actual weight 
// in kgs

impl Nau7802 {

    // new wasn't made from the most recent sensor example, following from 
    // https://github.com/viamrobotics/micro-rdk/blob/56615f4ace690f0571bba33d55cb530544c56aae/micro-rdk/src/common/mpu6050.rs#L57
    pub fn new(mut i2c_handle; I2CHandleType, i2c_address: u8) -> Result<Self, SensorError>{
    // Reset the device
    i2c_handle.write_i2c(NAU7802_I2C_ADDRESS, &[NAU7802_REG_PU_CTRL, 0x01])

    // configure
    i2c_handle.write_i2c(NAU7802_I2C_ADDRESS, &[NAU7802_REG_CTRL1, 0x30])?;
    i2c_handle.write_i2c(NAU7802_I2C_ADDRESS, &[NAU7802_REG_CTRL2, 0x07])?;

    Ok(Nau7802 {
        i2c_handle,
        i2c_address,
    })
    }

    pub fn from_config(cfg: ConfigType, deps: Vec<Dependency>) -> Result<SensorType,SensorError> {
        let board = get_board_from_dependencies(dependencies);
        if board.is_none() {
            return Err(SensorError::ConfigError("Nau7802 missing board attribute"));
        }
        let board_unwrapped = board.unwrap();
        let i2c_handle: I2CHandleType;

        if let Ok(i2c_name) = cfg.get_attributes::<String>("i2c_bus"){
            i2c_handle = board_unwrapped.get_i2c_by_name(i2c_name)?;
        } else {
            return Err(SensorError::ConfigError(
                "Nau7802 missing i2c_bus attribute in config",
            ));
        };
    
        Ok(Arc::new(Mutex::new(Nau7802::new(i2c_handle, NAU7802_I2C_ADDRESS)?)))
    }

    pub fn close(&mut self) -> Result<(), SensorError> {
        // reset the device again, is there a standby bit?
        self.i2c_handle.write_i2c(NAU7802_I2C_ADDRESS, &[NAU7802_REG_PU_CTRL, 0x01])?;
        Ok()
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

impl Readingsfor Nau7802{
    fn get_generic_readings(&mut self) -> Result<GenericReadingsResult, SensorError> {
        Ok(self
        .get_readings()?
        .into_iter()
        .map(|v| (v.0, SensorResult::<f64> { value:v.1}.into()))
        .collect())
    }
}

impl SensorT<f64> for Nau7802 {
    fn get_readings(&self) -> Result<TypedReadingsResult<f64>, SensorError> {
    let reading = self.read_adc();
    let mut x = HashMap:new();
    x.insert("raw_data".to_string(), reading as f64); // does this even work, do I have to convert?
    log::debug!("getting raw data from nau7802 succeeded!")
    Ok(x)
    }
}

pub fn read_adc(&mut self) -> Result<i32, E> { // i32 -> f64 needed
    let mut data = [u8; 3];
    self.i2c_handle.write_read(NAU7802_I2C_ADDRESS, &[NAU7802_REG_ADC], &mut data)?;
    let raw_value = ((data[0] as i32) << 16) | ((data[1] as i32) << 8) | (data[2] as i32);

    // Convert to signed 24-bit value
    let adc_value = if raw_value & 0x800000 != 0 {
        raw_value | !0xFFFFFF
    } else {
        raw_value
    };

    Ok(adc_value)
}