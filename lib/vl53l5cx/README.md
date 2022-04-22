# Non Blocking VL53L5CX library
The goal of this libarary is to provide a non blocking way to use the vl53l5cx sensors (also refered to as TOF sensors in the context of this project) by modifying ST's standard vl53l5cx library.

ST provides an "Ultra Light Driver" (ULD) to get interface with the TOF sensors but the drivers has the limitation that it does not support a non blocking way to get sensor data without using an interrupt pin. We have extended this ULD with some functions to enable an interrupt based non blocking method of receiving data. Functions implemented by us all contain "async" somewhere in its name and are all found in [vl53l5cx_plugin_async.c](src/vl53l5cx_plugin_async.c) and [vl53l5cx_plugin_async.h](inc/vl53l5cx_plugin_async.h). 

## How to use
### Configuration

Configuration of the sensor is done just like with ST's ULD and docs can be found [here](docs/VL53L5CX_ULD_UM2884.pdf).

### Fetching data blocking

See page 14 of [UM2884](docs/VL53L5CX_ULD_UM2884.pdf).

### Fetching data async

The async API implements two features, one to get the latest measurment, and a second one to check if a new measurment has been made since the last read measurment.
Each such feature is implemented using 3 functions, an "*_async_start()", an "*_async_in_progress()" and an "*_async_finish()" and the workflow of using these function is to

1. Use "*_async_start()" to initlize the call. This will send some i2c commando to the tof sensor.
2. In the I2C interrupt, check for *_async_in_progress()". This only reads a local boolean if the the last initilized call was of this type.
3. If *_async_in_progress()" returns true then you can collect the data with the "*_async_finish()" function.

If one just wants the latest measurment without caring about if a new measurement has been taken since last measurement then you can directly use "vl53l5cx_get_ranging_data_async_start()". Otherwise you can use "vl53l5cx_check_data_ready_async_start()" and if after the steps above "vl53l5cx_check_data_ready_async_finish()" indicated that new data is ready then you can initilize with  "vl53l5cx_get_ranging_data_async_start()" as normal.

### platform
For the library to work you need to provide a struct with the i2c address of the sensor and a pointer to an i2c handler.
```
typedef struct
{
    uint16_t  			address;
    I2C_HandleTypeDef 	*hi2c;

} VL53L5CX_Platform;
```

### Non blocking functions
All non blocking funtions can be found in the vl53l5cx_plugin_async.h file

## examples
An example implementation is availabe for the CM7 core on a stm32H755xx under the [example](example). The whole h755_vl53l5cx folder should be selected when importing into STM32CubeIDE. Our own code in the files specified in the beggining and in the main.c file where it is tagged with "NCS BEGIN" and "NCS END". The example code assumues use of the [53l5A1 breakout board](https://www.st.com/en/evaluation-tools/x-nucleo-53l5a1.html).
