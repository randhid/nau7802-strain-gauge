use micro_rdk::common::config::ConfigType;
use micro_rdk::common::registry::{
    get_board_from_dependencies, ComponentRegistry, Dependency, RegistryError,
};
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::DoCommand;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

// correct crates?
use micro_rdk::common::board::Board;
use micro_rdk::common::i2c::{I2CHandle, I2cHandleType};
use micro_rdk::common::sensor::{
    GenericReadingsResult, Readings, Sensor, SensorError, SensorResult, SensorT, SensorType,
    TypedReadingsResult,
};

pub fn register_models(registry: &mut ComponentRegistry) -> Result<(), RegistryError> {
    registry.register_sensor("nau_7802", &Nau7802::from_config)
}

const NAU7802_I2C_ADDRESS: u8 = 0x2A;
const NAU7802_REG_PU_CTRL: u8 = 0x00;
const NAU7802_REG_CTRL1: u8 = 0x01;
const NAU7802_REG_CTRL2: u8 = 0x02;
const NAU7802_REG_ADC: u8 = 0x12;

#[derive(DoCommand)]
pub struct Nau7802 {
    i2c_handle: I2cHandleType,
    i2c_address: u8,
    scale_to_kg: f64,
}

// TODOs:
// Tare command from DoCommand to reset scale
// Configuration attibute for max weight and add a reading element to return the actual weight
// in kgs

impl Nau7802 {
    // new wasn't made from the most recent sensor example, following from
    // https://github.com/viamrobotics/micro-rdk/blob/56615f4ace690f0571bba33d55cb530544c56aae/micro-rdk/src/common/mpu6050.rs#L57
    pub fn new(mut i2c_handle: I2cHandleType, i2c_address: u8) -> Result<Self, SensorError> {
        // Reset the device
        i2c_handle.write_i2c(NAU7802_I2C_ADDRESS, &[NAU7802_REG_PU_CTRL, 0x01])?;
    
        // configure
        i2c_handle.write_i2c(NAU7802_I2C_ADDRESS, &[NAU7802_REG_CTRL1, 0x30])?;
        i2c_handle.write_i2c(NAU7802_I2C_ADDRESS, &[NAU7802_REG_CTRL2, 0x07])?;
    
        Ok(Nau7802 {
            i2c_handle,
            i2c_address,
            scale_to_kg,
        })
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

        let scale_to_kg = cfg.get_attribute::<f64>("scale_to_kg").unwrap_or(1.0); // Default to 1.0 if not provided

        Ok(Arc::new(Mutex::new(Nau7802::new(
            i2c_handle,
            NAU7802_I2C_ADDRESS,
        )?)))
    }

    pub fn close(&mut self) -> Result<(), SensorError> {
        // reset the device again, is there a standby bit?
        self.i2c_handle
            .write_i2c(NAU7802_I2C_ADDRESS, &[NAU7802_REG_PU_CTRL, 0x01])?;
        Ok(())
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
        let reading = &self.read_adc()?;
        let mut readings = HashMap::new();
        readings.insert("raw_data".to_string(), *reading as f64); // does this even work, do I have to convert?
        //log::debug!("getting raw data from nau7802 succeeded!");

        // Store scaled weight in kilograms
        let scaled_reading = (*reading as f64) * self.scale_to_kg;
        readings.insert("weight_kg".to_string(), scaled_reading)
        
        Ok(readings)
    }
}

impl Nau7802 {
// Check the conversion of raw data to signed 24-bit value
pub fn read_adc(&self) -> Result<i32, SensorError> {
    let mut data: [u8; 3] = [0; 3];
    self.i2c_handle.lock().unwrap()
        .write_read_i2c(self.i2c_address, &[NAU7802_REG_ADC], &mut data)?;
    let raw_value = ((data[0] as i32) << 16) | ((data[1] as i32) << 8) | (data[2] as i32);

    // Convert to signed 24-bit value
    let adc_value = if raw_value & 0x800000 != 0 {
        raw_value | !0xFFFFFF
    } else {
        raw_value
    };

    Ok(adc_value)
}
