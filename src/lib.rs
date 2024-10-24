use micro_rdk::common::config::ConfigType;
use micro_rdk::common::registry::{
    get_board_from_dependencies, ComponentRegistry, Dependency, RegistryError,
};
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::DoCommand;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

// Necessary crate imports
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
    gain: u8,
    scale_to_kg: f64,
}

impl Nau7802 {
    pub fn new(i2c_handle: I2cHandleType, i2c_address: u8, scale_to_kg: f64) -> Self {
        Nau7802 {
            i2c_handle,
            i2c_address,
            gain,
            scale_to_kg,
        }
    }

    // Write to a register
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), SensorError> {
        self.i2c_handle.write_i2c(self.i2c_address, &[reg, value])?;
        Ok(())
    }

    // Read from a register
    fn read_register(&mut self, reg: u8) -> Result<u8, SensorError> {
        let mut buf = [0u8; 1];
        self.i2c_handle.write_read_i2c(self.i2c_address, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    // Calibrate the sensor
    pub fn calibrate(&mut self) -> Result<(), SensorError> {
        self.write_register(NAU7802_REG_CTRL2, 0x02)?;

        // Wait for calibration to complete
        loop {
            let status = self.read_register(NAU7802_REG_PU_CTRL)?;
            if (status & 0x08) != 0 { // 0x08 means we're done
                break;
            }
        }

        Ok(())
    }

    // Read the ADC value and convert to a 24-bit signed integer
    pub fn read_adc(&mut self) -> Result<i32, SensorError> {
        let mut data = [0u8; 3];
        self.i2c_handle.write_read_i2c(self.i2c_address, &[NAU7802_REG_ADC], &mut data)?;

        // Convert to 24-bit signed integer
        let raw_value = ((data[0] as i32) << 16) | ((data[1] as i32) << 8) | (data[2] as i32);
        let adc_value = if raw_value & 0x800000 != 0 {
            raw_value | !0xFFFFFF
        } else {
            raw_value
        };

        Ok(adc_value)
    }

    // From configuration: sets up the sensor and returns a SensorType
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
            scale_to_kg,
            3, // Default gain set to 8x (which corresponds to value 3 in CTRL1 register)
        ))))
    }

    // Close the sensor
    pub fn close(&mut self) -> Result<(), SensorError> {
        self.write_register(NAU7802_REG_PU_CTRL, 0x01)?;
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
        let reading = self.read_adc()?;
        let mut readings = HashMap::new();
        readings.insert("raw_data".to_string(), reading as f64);

        // Store scaled weight in kilograms
        let scaled_reading = (reading as f64) * self.scale_to_kg;
        readings.insert("weight_kg".to_string(), scaled_reading);

        Ok(readings)
    }
}
