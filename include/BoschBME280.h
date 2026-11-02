#pragma once

#include <stdint.h>
#include <stddef.h>
#include "Interface.h" // Required from the MCU project!

struct bme280_dev;  // Forward declaration of bme280 device struct

namespace sensor
{

/**
 * @brief Driver for BME280/BMP280
 * 
 * @details BME280 returns temperature, pressure and humidity;
 *          BMP280 returns temperature and pressure.
 *          Original Bosch driver was improved for both types of sensors.
 *          https://github.com/boschsensortec/BME280_SensorAPI
 * 
 */
class BMx280
{
public:
    static constexpr uint8_t I2C_addr = 0x76;               // Default: SDO pin pulled to GND
    static constexpr uint8_t I2C_altAddr = 0x77;            // Alternative: SDO pin pulled to VCC
    static constexpr size_t m_DevStructStorageSize = 64;    // Max size of bme280_dev struct

    enum class PressureUnit { Pa, hPa, mmHg };  // Config of Pressure units
    enum class RefreshRate
    {
        Refresh_0_5Ms =  0,     // 0.5 msec
        Refresh_10Ms =   6,     // 10 msec
        Refresh_20Ms =   7,     // 20 msec
        Refresh_62_5Ms = 1,     // 62.5 msec
        Refresh_125Ms =  2,     // 125 msec
        Refresh_250Ms =  3,     // 250 msec
        Refresh_500Ms =  4,     // 500 msec
        Refresh_1000Ms = 5,     // 1000 msec
    };

    /**
     * @brief Constructor
     * @param [in] i2c communication interface
     * @param [in] delayer uSec delays shaper interface
     * @param [in] devAddr i2c slave address
     */
    BMx280(i_face::I_I2C& i2c, i_face::I_Delayer& delayer, uint16_t devAddr);

    /**
     * @brief Sensor initialization
     * @param [in] pressUnit Pressure units
     * @param [in] refreshRate Standby time
     */
    bool init(PressureUnit pressUnit = PressureUnit::mmHg, RefreshRate refreshRate = RefreshRate::Refresh_500Ms);

    /**
     * @brief Get measurements
     * @param [out] temperature Temperature value in C degrees
     * @param [out] pressure Atmospheric pressure in Pa/gPa/mmHg (depends on settings)
     * @param [out] humidity Available only for BME280 sensor
     * @return True: data valid, False: measurements are not ready (in process)
     */
    bool getMeasurements(float& temperature, float& pressure, float& humidity);
    bool getMeasurements(float& temperature, float& pressure);

    /**
     * @brief Public methods for invoking from the bridge-functions
     */
    bool readReg(uint8_t regAddr, uint8_t *regDest, uint32_t regSize);
    bool writeReg(uint8_t regAddr, const uint8_t *regSrc, uint32_t regSize);
    void usDelay(uint32_t period);

private:
    /**
     * @brief Device ID (BME280 or BMP280), read from 0xD0 register
     */
    enum class DeviceID : uint8_t
    {
        None = 0,
        BME = 0x60,
        BMP = 0x58
    };

    /**
     * @brief Register addresses
     */
    enum class RegAddr : uint8_t
    {
        DevId = 0xD0,   // Device ID register (read-only)
        State = 0xF3,   // State register
    };

    /**
     * @brief Static storage for sensors bme280_dev struct storing
     * @details Separates original Bosch driver and this class declaration 
     */
    alignas(alignof(void*)) uint8_t m_bme280devStorage[m_DevStructStorageSize];
    bme280_dev* m_bmeDev; 

    const uint16_t m_i2cAddr;                   // Sensor I2C slave address
    i_face::I_I2C& m_i2c;                       // I2C interface
    i_face::I_Delayer& m_delayer;               // usec/msec delays shaper
    DeviceID m_sensorType = DeviceID::None;     // Sensor type: BME280 or BMP280
    PressureUnit m_pressureUnit;                // Units for atmospheric pressure

    /**
     * @brief Checks if new valid data is available
     * @return True: data valid and available, 
     *         False: measurements are not ready (in process) or sensor in Sleep mode
     */
    bool isDataReady();
};

}   // namespace sensor
