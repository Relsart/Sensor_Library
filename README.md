# Sensors library for usage in MCU projects

## MLX90640

A C++ driver for the Melexis MLX90640 far-infrared thermal sensor array (32x24 pixels) for microcontrollers.

### Features:

* Supports both sensor variants: **MLX90640-BA** (110°×75° FOV) and **MLX90640-BB** (55°×35° FOV).
* Configurable refresh rates ranging from **0.5 Hz up to 64 Hz**.
* Raw data acquisition over I2C (supporting DMA-ready integration) and conversion to pixel temperatures in Celsius (°C).
* Built-in broken/vixen pixel correction based on official Melexis mathematical logic.

### Code Example:

```c++
#include "Mlx90640.h"

/* Instance creating: */
Mlx90640 m_thermoDriver(0x33);  // I2C device address

/* Periphery subscribing: */
m_thermoDriver.setI2cTx(thermomatrix::Mlx90640::I2C_Tx::create<driver::I2C, &driver::I2C::masterTx>(*m_i2c));
m_thermoDriver.setI2cTxRx(thermomatrix::Mlx90640::I2C_TxRx::create<driver::I2C, &driver::I2C::masterTxRx>(*m_i2c));
m_thermoDriver.setI2cRxDMA(thermomatrix::Mlx90640::I2C_RxDMA::create<driver::I2C, &driver::I2C::readRegViaDMA_16bitAddr>(*m_i2c));

/* Initialization and EEPROM parameters reading: */
if (!m_thermoDriver.init(thermomatrix::Mlx90640::RefreshRate::Rate8HZ))
    return false;

if (!m_thermoDriver.readParameters())
    return false;

/* Waiting for data ready: */
while (!m_thermoDriver.isFrameReady()){ /*Timeout*/ }

/* 1st step: Frame request: */
bool isOk = m_thermoDriver.frameRequest();

/* 2nd step: AUX adta request: */
bool isOk = m_thermoDriver.auxRequest();

/* 3rd step: Control register request: */
bool isOk = m_thermoDriver.controlRequest();

/* Check results: */
if (m_thermoDriver.dataHandle())  // Both subpages have calculated:
{
    m_sendReadyData.activ(m_thermoDriver.getMeasurements());    // Send ready frame to consumers
}

```

## Contacts

relsartmail@gmail.com