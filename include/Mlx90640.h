#pragma once

#include <stdint.h>
#include "MLX90640_API.h"
#ifdef WITH_ETL
#include "etl/delegate.h"
#define funct etl::delegate
#else
#include <functional>
#define funct std::function
#endif

namespace thermomatrix
{

/**
 * @brief Transfer measurements data struct
 */
struct ThermoArray
{
    float* data;    // thermo array[32][24]
    uint16_t size;  // const 768
};

/**
 * @brief Driver for thermosensor MLX90640
 * 
 * @details operations sequence:
 *  1. Create instance and invoke the init method;
 *  2. Invoke the readParameters method (for getting sensors parameters)
 * 
 *  3. Workmode: invoke isFrameReady() till it returns true (maybe with timeout control).
 *  4. Invoke frameRequest, auxRequest and controlRequest for 
 *     data requesting (via DMA), using I2C IRQs for data streams control in upper classes.
 *     Received data will be collected in private rawBuffer;
 *  5. Invoke dataHandle for received data calculating. Ready measurements will be emplased in
 *     private measurements buffer.
 *  6. Use the getMeasurements to get ready measurements in the consumer class.
 * 
 */
class Mlx90640
{
public:
    /* Signature: isOk(slaveAddr, sourcePtr, sourceSize, destPtr, destSize, timeoutMs) */
    using I2C_TxRx = funct<bool(uint16_t, uint8_t*, uint16_t, uint8_t*, uint32_t, uint32_t)>;
    /* Signature: isOk(slaveAddr, sourcePtr, sourceSize, timeoutMs) */
    using I2C_Tx = funct<bool(uint16_t, uint8_t*, uint16_t, uint32_t)>;
    /* Signature: isOk(slaveAddr, regAddr, destPtr, destSize) */
    using I2C_RxDMA = funct<bool(uint16_t, uint16_t, uint8_t*, uint32_t)>;


    /**
     * @brief Sensor refresh rate configuration
     */
    enum class RefreshRate
    {
        Rate2HZ =  2,
        Rate4HZ =  3,
        Rate8HZ =  4,
        Rate16HZ = 5,
        Rate32HZ = 6   // Unstable and not recommended
    };

    /**
     * @brief Constructor
     * @param [in] i2c Periphery driver class
     * @param [in] devAddr Slave i2c address
     */
    Mlx90640(uint16_t devAddr);

    /**
     * @brief Sensor initialisation sequence
     * @param [in] refreshRate Sensor refresh frequency
     */
    bool init(RefreshRate refreshRate);

    /**
     * @brief Read and extract internal parameters from sensor registers
     * @return True == Ok
     */
    bool readParameters();

    /**
     * @brief Check if new thermo frame is ready
     * @details Uses no DMA transactions
     * @return True == Ready
     */
    bool isFrameReady();

    /**
     * @brief New frame request from sensor
     */
    bool frameRequest();

    /**
     * @brief Aux data request from sensor
     */
    bool auxRequest();

    /**
     * @brief Control register value request from sensor
     */
    bool controlRequest();

    /**
     * @brief Received data handling
     * @return True == both of subpages have been calculated 
     */
    bool dataHandle();

    /**
     * @brief Setting i2c driver callbacks
     */
    void setI2cTxRx(I2C_TxRx h);
    void setI2cTx(I2C_Tx h);
    void setI2cRxDMA(I2C_RxDMA h);

    /**
     * @brief Get address of ready measurements buffer
     */
    const ThermoArray* getMeasurements();
    
private:
    I2C_TxRx m_i2cTxRx;     // I2C PIO receive-transmit callback
    I2C_Tx m_i2cTx;         // I2C PIO transmit callback
    I2C_RxDMA m_i2cRxDMA;   // I2C DMA receive callback

    static constexpr uint16_t m_PixelsNum = MLX90640_PIXEL_NUM;
    const uint16_t m_i2cAddr;       // Sensor I2C slave address
    paramsMLX90640 m_params;        // Internal parametersfrom sensor
    uint16_t m_rawBuffer[1000]{0};  // Buffer for receiving raw data from sensor
    uint16_t* const m_pixelsArea;   // Address of Pixels area in the Raw Data Buffer
    uint16_t* const m_auxArea;      // Address of Aux area in the Raw Data Buffer
    uint16_t* const m_controlArea;  // Address of Control Register value in the Raw Data Buffer
    uint16_t* const m_stateArea;    // Address of State Register value in the Raw Data Buffer
    bool m_subpage0 = false;        // Flag: subpage 0 has received
    bool m_subpage1 = false;        // Flag: subpage 1 has received
    ThermoArray m_txMeasuresPack;   // Data for external signal exchanging

    float m_measurBuffer[m_PixelsNum]{0};   // Buffer for ready measurements (pixels values in degC)
    const uint32_t m_FrameTimeoutMs = 2000; // Timeout (in ms) value for waiting sensor measurements are complete
    
    /**
     * @brief Read register from sensor
     * @param [in] regAddr Register address
     * @param [in] buffer Destination address
     * @param [in] size Number of registers
     */
    bool readRegister(uint16_t regAddr, uint16_t* buffer, uint16_t size = 1);

    /**
     * @brief Write register to sensor
     * @param [in] regAddr Register address
     * @param [in] source Source address
     * @param [in] size Number of registers
     */
    bool writeRegister(uint16_t regAddr, uint16_t* source, uint16_t size = 1);

    /**
     * @brief Request reading memory area from sensor (via DMA) 
     * @param [in] startAddr Register address
     * @param [in] buffer Destination address
     * @param [in] size Number of words (uint16_t)
     */
    bool readMemory(uint16_t startAddr, uint16_t* buffer, uint16_t size);

    /**
     * @brief Convert the Raw Buffer units to MSB first format
     */
    void shiftBufferBytes();
};

}   // namespace thermomatrix
