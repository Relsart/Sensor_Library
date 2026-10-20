#pragma once

#include <stdint.h>
#include <etl/optional.h>
#include "Interface.h" // Required from the MCU project!

namespace sensor
{

/**
 * @brief Dallas D18B20 thermo sensor driver
 */
class D18B20
{
public:
    #ifdef WITH_ETL
    using Callback = etl::delegate<void()>;
    #else
    using Callback = std::function<void()>;
    #endif

    /**
     * @brief Request struct
     * @details Bidirectional: uses for storing request infos and return the output measurements
     */
    struct Request
    {
        const i_face::I_OneWire::DeviceRomId id;
        float measuredResult = 0;
        bool valid = false;
    };

    static constexpr uint8_t m_FamilyCode = 0x28;

    /**
     * @brief OneWire D18B20 specific commands codes
     */
    enum class SpecificCmds : uint8_t
    {
        ConvertStart    = 0x44,     // Start temperature measurement
        ScratchpadRead  = 0xBE,     // Read measurements result from RAM (scratchpad)
        ScratchpadWrite = 0x4E,     // Write settings (alarm limits, resolution) to the RAM (scratchpad)
        ScratchpadCopy  = 0x48,     // Save the settings from RAM to EEPROM
    };

    /**
     * @brief Reslolution codes:
     */
    enum Resolution : uint8_t
    {
        Res_9Bit =  0,   // Conversion time: 93.75 msec, LSB (step) value: 0.5 °C
        Res_10Bit = 1,   // 187.5 msec, 0.25 °C
        Res_11Bit = 2,   // 375 msec, 0.125 °C
        Res_12Bit = 3,   // 750 msec, 0.0625 °C (default)
        ResNone = 0xFF   // Error / not defined
    };

    /**
     * @brief Constructor
     * @param [in] oneWire OneWire driver interface
     * @param [in] timer Timer interface
     */
    D18B20(i_face::I_OneWire& oneWire, i_face::I_Timer& timer);

    /**
     * @brief Subscribe for Measurements Ready callback
     */
    void setCallback(Callback h);

    /**
     * @brief Get sensors resolution value
     * @param [in] sensId unique sensor ID
     * @return resolution value
     */
    Resolution getResolution(const i_face::I_OneWire::DeviceRomId* sensId);

    /**
     * @brief Configure target sensors resolution
     * @param [in] sensId unique sensor ID
     * @param [in] resSet resolution value
     * @return True == Ok (resolution was saved properly)
     */
    bool init(const i_face::I_OneWire::DeviceRomId* sensId, Resolution resSet);

    /**
     * @brief Checks if all conversions are finished (line released)
     * @return True == busy (at least one sensor on the bus)
     */
    bool isBusy();

    /**
     * @brief Requests the new measurement for all sensors on the bus
     * @param [in] request request structs array
     * @param [in] count number of requested sensors (request structs)
     * @return True == Ok
     */
    bool takeMeasure(Request* request, uint32_t count);   

private:
    static constexpr float m_Scale = 0.0625;    //  Scale value

    /**
     * @brief Conversion time values (in msec), depending on resolution:
     */
    inline static const uint16_t m_WaitTimes[4] = {94, 188, 375, 750};

    i_face::I_OneWire& m_bus;   // OneWire bus driver
    i_face::I_Timer& m_timer;   // Timer
    Callback m_readyClbk;       // Measurements ready callback

    static constexpr uint8_t m_ScratchpadSize = 9;
    uint8_t m_scratchBuffer[m_ScratchpadSize]{0};   // Buffer for sensors scratchpad reading

    Request* m_request = nullptr;           // Current request(s) array
    uint32_t m_requestCount = 0;            // Number of requests (sensors to read)
    uint32_t m_timerSet = m_WaitTimes[3];   // Timer set for conversion waiting

    /**
     * @brief Timer handler: get the measurement results from requested sensors
     */
    void timerHandler();

    /**
     * @brief Reads sensors scratchpad bytes to the internal buffer
     * @param [in] sensId unique sensor ID
     * @return Reading result: True == Ok
     */
    bool readScratchpad(const i_face::I_OneWire::DeviceRomId* sensId);

    /**
     * @brief Writes sensors scratchpad bytes from the internal buffer
     * @param [in] sensId unique sensor ID
     * @return Reading result: True == Ok
     */
    bool writeScratchpad(const i_face::I_OneWire::DeviceRomId* sensId);

    /**
     * @brief Calculates the measured temperature value according to sensors resolution
     * @param [in] raw 2-bytes raw value
     * @param [in] resolution sensors resolution
     * @return Ready value or NAN if failed
     */
    etl::optional<float> calcTemperature(uint16_t raw, Resolution resolution);

    /**
     * @brief Selects the sensor by its unique ROM ID value
     * @param [in] sensId unique sensor ID
     */
    void select(const i_face::I_OneWire::DeviceRomId* sensId);
}; 

}   // namespace sensor
