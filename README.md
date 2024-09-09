# nau7802-strain-gauge

This mpdule implements a nau7802 straing gauge as a sensor for the esp32 using i2c communications to read anad write data.

## Wiring Guide
You can follow some of the steps to wire the Nau7802 to a microcontroller [here](https://learn.sparkfun.com/tutorials/qwiic-scale-hookup-guide), use your imagination to substitute an esp32 for the arduino mentioned in the tutorial. 

## Configure your nau7802 sensor

Navigate to the [**CONFIGURE** tab](https://docs.viam.com/build/configure/) of your [machine](https://docs.viam.com/fleet/machines/) in [the Viam app](https://app.viam.com/).
[Add <INSERT COMPONENT TYPE / INSERT RESOURCE NAME> to your machine](https://docs.viam.com/build/configure/#components).

On the new component panel, copy and paste the following attribute template into your sensors’s attributes field:

```json
{
    "attirbutes" : {
        "i2c_bus" : <String>,
        "scale_to_kg": f64,
        }  
}
```

### Attributes

The following attributes are available for `<INSERT MODEL TRIPLET>` <INSERT API NAME>s:

| Name    | Type   | Inclusion    | Description |
| ------- | ------ | ------------ | ----------- |
| `i2c_bus` | string | **Required** | name of the i2c bus on the esp32 that is used to connect the nau7802 sensor.|
| `max_weight_kg` | string | Optional     | Max weight that the Nau7802 uses to make it's calculation for the calibrated reading output in kilograms.|

### Example configuration

```json
{
    "attirbutes" : {
        "i2c_bus" : "i2c0",
        "max_weight_kg": 10,
        }  
}
```