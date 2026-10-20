#pragma once

#include <stdint.h>
#include "Interface.h" // Required from the MCU project!
#include "ParserIface.h"

namespace sensor
{

class MhZ19E : public ParserInterface <MhZ19E, 32>
{
public:
    friend class ParserInterface<MhZ19E, 32>; // For Parser interface!

    #ifdef WITH_ETL
    using Handler = etl::delegate<void()>;
    /* Signature: void(CO2pps, temperature) */
    using Consumer = etl::delegate<void(uint16_t, int8_t)>;
    #else
    using Handler = std::function<void()>;
    using Consumer = std::function<void(uint16_t, int8_t)>;
    #endif

    static constexpr uint32_t baudrate = 9600;  // UART baudrate for this sensor

    /**
     * @brief Constructor
     * @param [in] uart Serial port interface
     */
    MhZ19E(i_face::I_Uart& uart);

    /**
     * @brief Set the Data Rx Event callback
     * @details Used for separating ISR handling from task or main loop execution,
     *          further parsing and consuming realised by invoking handleRawData() from task
     */
    void setRawDataCallback(Handler h);

    /**
     * @brief Subscribe for the valid measuremments consumer
     */
    void setConsumer(Consumer h);

    /**
     * @brief Request new measurements
     */
    void requestMeasurements();

    /**
     * @brief Received data parsing and handling (deferred handler, invokes from task)
     */
    void handleRawData();

private:
    /**
     * @brief MH-Z19E command codes
     */
    enum class CmdCodes : uint8_t
    {
        AutoKalibr      = 0x79,   // Automatic Baseline Correction
        GetMeasurement  = 0x86,   // Read gas concentration (main command)
        ZeroKalibr      = 0x87,   // Zero Point calibration
        SpanCalibr      = 0x88,   // For laboratory calibration (do not use!)
        RangeSet        = 0x99,   // Set measuring range
        GetFwRev        = 0xA1    // Get firmware revision
    };

    /**
     * @brief Measurements request structure
     */
    struct __attribute__((packed)) Request
    {
        const uint8_t Preamble = 0xFF;  // Preamble
        uint8_t sensNo = 1;
        uint8_t cmdCode = 0;
        uint8_t params[5]{0};
        uint8_t checkSum = 0;
    };

    /**
     * @brief Measurements receive structure (from byte 1)
     */
    struct __attribute__((packed)) Measurements
    {
        uint8_t cmdCode;        // Command code (must be 0x86)
        uint16_t co2Conc;       // CO2 concentration
        uint8_t temperature;    // Temperature value
    };
    Measurements m_measur;

    i_face::I_Uart& m_uart;     // Serial port
    Handler m_handler;          // Received data handler (callback for deferred handling)
    Consumer m_consumer;        // Valid measurements consumer

    static constexpr uint8_t m_PackSize = 9;        // Size of request/pesponse packages
    static constexpr uint8_t m_Preamble = 0xFF;     // Preamble, Magic value

    void getDataISR(i_face::I_RingBuffer& data);

    /**
     * @brief Get package size
     * @details Parser interface
     * @return constant 9
     */
    size_t getPackSize(const uint8_t*, size_t);


    /**
     * @brief Check message controlsumm
     * @details Parser interface
     * @param [in] data, size message
     */
    bool checkPackCrc(const uint8_t* data, uint32_t size) ;

    /**
     * @brief Handle valid package
     * @details Parser interface
     * @param [in] data, size message
     */
    void handleMessage(const uint8_t* data, uint32_t size) ;
};

}   // namespace sensor
