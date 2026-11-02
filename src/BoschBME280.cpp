#include "BoschBME280.h"
#include "bosch_BME280/bme280.h"
#include <string.h>

namespace sensor
{

static_assert(sizeof(bme280_dev) <= BMx280::m_DevStructStorageSize, "StorageSize in BoschBME280.h is too small!");
static_assert(alignof(bme280_dev) <= alignof(void*), "Storage align in BoschBME280.h is incorrect!");

namespace
{
/**
 * @brief Bridge-functions for binding BMx280 instance with BME280 official APIs:
 */
BME280_INTF_RET_TYPE i2cReadRridge(uint8_t regAddr, uint8_t *regDest, uint32_t regSize, void *context)
{
    if (context && regDest && regSize > 0)
    {
        auto* instance = static_cast<BMx280*>(context);
        return instance->readReg(regAddr, regDest, regSize) ? BME280_OK : BME280_E_COMM_FAIL;
    }
    else
    {
        return BME280_E_COMM_FAIL;
    }
}

BME280_INTF_RET_TYPE i2cWriteBridge(uint8_t regAddr, const uint8_t *regSrc, uint32_t regSize, void *context)
{
    if (context && regSrc && regSize > 0)
    {
        auto* instance = static_cast<BMx280*>(context);
        return instance->writeReg(regAddr, regSrc, regSize) ? BME280_OK : BME280_E_COMM_FAIL;
    }
    else
    {
        return BME280_E_COMM_FAIL;
    }
}

void delayUsBridge(uint32_t period, void *context)
{
    if (context && period > 0)
    {
        auto* instance = static_cast<BMx280*>(context);
        instance->usDelay(period);
    }
}

} // ------------------------------ namespace ------------------------------

BMx280::BMx280(i_face::I_I2C& i2c, i_face::I_Delayer& delayer, uint16_t devAddr) :
    m_i2c(i2c),
    m_i2cAddr(devAddr),
    m_delayer(delayer)
{
    new(m_bme280devStorage) struct bme280_dev();
    m_bmeDev = reinterpret_cast<struct bme280_dev*>(m_bme280devStorage);
}

bool BMx280::readReg(uint8_t regAddr, uint8_t *regDest, uint32_t regSize)
{
    return m_i2c.masterTxRx(m_i2cAddr, &regAddr, 1, regDest, regSize, 100);
}

bool BMx280::writeReg(uint8_t regAddr, const uint8_t *regSrc, uint32_t regSize)
{
    if (regSize >= 5)
        return false;

    uint8_t sendBuff[5]{0};
    sendBuff[0] = regAddr;
    memcpy(&sendBuff[1], regSrc, regSize);
    return m_i2c.masterTx(m_i2cAddr, sendBuff, regSize + 1, 100);
}

void BMx280::usDelay(uint32_t period)
{
    if (period > 0)
        m_delayer.delayUs(period);
}

bool BMx280::isDataReady()
{
    uint8_t stateReg = 0;
    if (!readReg(static_cast<uint8_t>(RegAddr::State), &stateReg, sizeof(stateReg)))
        return false;

    // Bit 3 of State register is "measuring". If it's 1 the sensor is still measuring,
    // if 0 then the data is ready for reading.
    return (stateReg & 0x08) == 0;
}

bool BMx280::init(PressureUnit pressUnit, RefreshRate refreshRate)
{
    if (!m_bmeDev)
        return false;

    uint8_t regbuf = 0;
    if (!readReg(static_cast<uint8_t>(RegAddr::DevId), &regbuf, sizeof(regbuf)))
        return false;

    switch (static_cast<DeviceID>(regbuf))
    {
    case DeviceID::BME:
        m_sensorType = DeviceID::BME;
        break;
    case DeviceID::BMP:
        m_sensorType = DeviceID::BMP;
        break;
    default:
        return false;
    }
    m_pressureUnit = pressUnit;

    // Configurating sensors static struct:
    m_bmeDev->intf = BME280_I2C_INTF;
    m_bmeDev->read = i2cReadRridge;
    m_bmeDev->write = i2cWriteBridge;
    m_bmeDev->delay_us = delayUsBridge;
    m_bmeDev->intf_ptr = this;

    // Reading ID and calibration data:
    if (bme280_init(m_bmeDev) != BME280_OK)
        return false;

    // Put the sensor to sleep (in case it's not a cold start). 
    // From SLEEP mode, the sensor is guaranteed to allow to change its settings:
    if (bme280_set_sensor_mode(BME280_POWERMODE_SLEEP, m_bmeDev) != BME280_OK)
        return false;

    // Setting sensors parameters: oversampling, filter and standby duration:
    struct bme280_settings settings{0};
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_16X;
    settings.osr_t = BME280_OVERSAMPLING_2X;
    settings.filter = BME280_FILTER_COEFF_16;
    settings.standby_time = static_cast<uint8_t>(refreshRate);                    
    if (bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, m_bmeDev) != BME280_OK)
        return false;

    // Waking the sensor:
    return (bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, m_bmeDev) == BME280_OK);
}

bool BMx280::getMeasurements(float& temperature, float& pressure, float& humidity)
{
    if (!m_bmeDev || !isDataReady())
        return false;   // Not ready yet!

    struct bme280_data comp_data{0};
    if (bme280_get_sensor_data(BME280_ALL, &comp_data, m_bmeDev) != BME280_OK)
        return false;

    temperature = static_cast<float>(comp_data.temperature);
    switch (m_pressureUnit)
    {
    case PressureUnit::Pa:
        pressure = static_cast<float>(comp_data.pressure);
        break;
    case PressureUnit::hPa:
        pressure = static_cast<float>(comp_data.pressure / 100.0);
        break;
    case PressureUnit::mmHg:
        pressure = static_cast<float>(comp_data.pressure / 133.3223684);
        break;
    }
    if (m_sensorType == DeviceID::BME)  // For BPM not supported
        humidity = static_cast<float>(comp_data.humidity);
    return true;
}

bool BMx280::getMeasurements(float& temperature, float& pressure)
{
    if (!m_bmeDev || !isDataReady())
        return false;   // Not ready yet!

    struct bme280_data comp_data{0};
    if (bme280_get_sensor_data(BME280_ALL, &comp_data, m_bmeDev) != BME280_OK)
        return false;

    temperature = static_cast<float>(comp_data.temperature);
    switch (m_pressureUnit)
    {
    case PressureUnit::Pa:
        pressure = static_cast<float>(comp_data.pressure);
        break;
    case PressureUnit::hPa:
        pressure = static_cast<float>(comp_data.pressure / 100.0);
        break;
    case PressureUnit::mmHg:
        pressure = static_cast<float>(comp_data.pressure / 133.3223684);
        break;
    }
    return true;
}

}   // namespace sensor
