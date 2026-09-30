# Sensors library for usage in MCU projects

## -------------- MLX90640 thermal matrix --------------

A C++ driver for the Melexis **MLX90640** far-infrared thermal sensor array (32x24 pixels) for microcontrollers.

### Features:

* Supports both sensor variants: **MLX90640-BA** (110°×75° FOV) and **MLX90640-BB** (55°×35° FOV).
* Configurable refresh rates ranging from **0.5 Hz up to 64 Hz**.
* Raw data acquisition over I2C (supporting DMA-ready integration) and conversion to pixel temperatures in Celsius (°C).
* Built-in broken/vixen pixel correction based on official Melexis mathematical logic.

### Usage:

* Initialization step:
- Create thermomatrix::Mlx90640 object with its I2C address (0x33) in constructor;
- Subscribe I2C callbacks (etl::delegates) to the I2C periphery driver;
- Invoke init() method with thermomatrix::Mlx90640::RefreshRate value in argument;
- Invoke readParameters() method to read and save sensors EEPROM configurations to the driver class;

* Working process:
- Use isFrameReady() method for waiting untill new frame to be ready (calculated by sensor);
- Invoke frameRequest() to read new frame data;
- By XferReady interruptions sequentally invoke auxRequest() and controlRequest();
- When the last package received, invoke the dataHandle() method. Frames comes only by halves! So dataHandle() returns true if both halves have received.
- If whole the frame is completely ready, use the getMeasurements() to get the results.

```c++
// See code example in samples/Mlx90640.cpp 
```

## -------------- DS18B20 digital thermometer --------------

A C++ driver for **DS18B20** digital temperature sensor over the OneWire protocol on **STM32** microcontrollers.

### Features:

* **Multi-sensor support:** Works with a single sensor or multiple devices on the same bus (ROM code addressing).
* **Configurable Resolution:** Supports 9-bit to 12-bit temperature resolution configuration.
* **CRC verification:** Built-in checksum verification to filter out corrupted data.
* **Measurement time management** Supports any interrupt-based or main-loop (or RTOS) timers to manage measurement delays (according to selected resolution) without while loops.

### Usage:

- Create and initialize OneWire interface driver and Timer instance;
- Create sensor::D18B20 with bus and timer links in constructor;
- Subscribe D18B20 instance for Data Ready callback (it will be invoked after finishing measurement and conversion);
- Set the resolution values to sensors (the same) if it is necessary, or default values will be used;
- Invoke takeMeasure method with requests structs in arguments: this structures contains unique IDs of requested sensors and variables for temperature result writing;

```c++
// See code example in samples/Ds18b20.cpp 
```

## Contacts

relsartmail@gmail.com
