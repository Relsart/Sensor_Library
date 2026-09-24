#include "Mlx90640.h"

namespace thermomatrix
{
/*  0-767 pixels
    768-831 aux
    832 control register
    833 status register */

Mlx90640::Mlx90640(uint16_t devAddr) : m_i2cAddr(devAddr),
                                       m_pixelsArea(&m_rawBuffer[0]),
                                       m_auxArea(&m_rawBuffer[m_PixelsNum]),
                                       m_controlArea(&m_rawBuffer[m_PixelsNum + MLX90640_AUX_NUM]),
                                       m_stateArea(m_controlArea + 1),
                                       m_txMeasuresPack{.data = m_measurBuffer, .size = m_PixelsNum}
{}

void Mlx90640::setI2cTxRx(I2C_TxRx h) { m_i2cTxRx = h; }
void Mlx90640::setI2cTx(I2C_Tx h) { m_i2cTx = h; }
void Mlx90640::setI2cRxDMA(I2C_RxDMA h) { m_i2cRxDMA = h; }

const ThermoArray* Mlx90640::getMeasurements()
{
    return &m_txMeasuresPack;
}

bool Mlx90640::init(RefreshRate refreshRate)
{
    /* Read Control register */
    uint16_t ctrlRegister = 0;
    if (!readRegister(MLX90640_CTRL_REG, &ctrlRegister))
        return false;
    /* Set refresh rate */
    uint16_t value = (static_cast<uint16_t>(refreshRate) << MLX90640_CTRL_REFRESH_SHIFT);
    value &= ~MLX90640_CTRL_REFRESH_MASK;
    value |= (ctrlRegister & MLX90640_CTRL_REFRESH_MASK);
    /* Set Chess mode */
    value |= MLX90640_CTRL_MEAS_MODE_MASK;
    /* Update Control register */
    return writeRegister(MLX90640_CTRL_REG, &value);
}

bool Mlx90640::readParameters()
{
    /* Read dump and extract parameters */
    if (!readRegister(MLX90640_EEPROM_START_ADDRESS, m_rawBuffer, MLX90640_EEPROM_DUMP_NUM))
        return false;

    return (MLX90640_ExtractParameters(m_rawBuffer, &m_params) == MLX90640_NO_ERROR);
}

bool Mlx90640::isFrameReady()
{
    uint16_t tempStateReg = 0;  // Temporary State Register buffer
    if (!readRegister(MLX90640_STATUS_REG, &tempStateReg))
        return false;   // Transfer error
    
    if (MLX90640_GET_DATA_READY(tempStateReg) == 0)
        return false;   // Not ready
    
    // New frame is ready:
    tempStateReg &= 0x0001;         // Mask all state bits, except subpage flag
    *m_stateArea = tempStateReg;    // Put state value (subpage flag) to the raw buffer (for further calculation)
    return true;
}

bool Mlx90640::frameRequest()   // TODO: to separate functions! 
{
    /* Re-init Status Register: */
    uint16_t initState = MLX90640_INIT_STATUS_VALUE;
    if (!writeRegister(MLX90640_STATUS_REG, &initState))
        return false;

    /* Request Pixels Data: */
    return readMemory(MLX90640_PIXEL_DATA_START_ADDRESS, m_pixelsArea, m_PixelsNum);
}

bool Mlx90640::auxRequest()
{
    return readMemory(MLX90640_AUX_DATA_START_ADDRESS, m_auxArea, MLX90640_AUX_NUM);
}

bool Mlx90640::controlRequest()
{
    return readMemory(MLX90640_CTRL_REG, m_controlArea, 1);
}

bool Mlx90640::readRegister(uint16_t regAddr, uint16_t* buffer, uint16_t size)
{
    if (!m_i2cTxRx || !buffer || size == 0)
        return false;

    // Convert register address to MSB first format and read:
    uint8_t addrBytes[sizeof(uint16_t)]{static_cast<uint8_t>((regAddr >> 8) & 0xFF), static_cast<uint8_t>(regAddr & 0xFF)};
    if (!m_i2cTxRx(m_i2cAddr, addrBytes, sizeof(addrBytes), reinterpret_cast<uint8_t*>(buffer), size * sizeof(uint16_t), 200))
        return false;

    // Afret reading it must be converted to MSB first format:
    for (int i = 0; i < size; i++)
    {
        auto dataU8 = reinterpret_cast<uint8_t*>(&buffer[i]);
        buffer[i] = ((dataU8[0] << 8) | dataU8[1]);
    }
    return true;
}

bool Mlx90640::writeRegister(uint16_t regAddr, uint16_t* source, uint16_t size)
{
    if (!m_i2cTx || !source || size == 0)
        return false;

    // Convert register address and data to MSB first format:
    uint8_t dataBuffer[(size + 1) * sizeof(uint16_t)]{0};   // Buffer size = register address + some number of values
    dataBuffer[0] = static_cast<uint8_t>((regAddr >> 8) & 0xFF);
    dataBuffer[1] = static_cast<uint8_t>(regAddr & 0xFF);
    for (int i = 0; i < size; i++)
    {
        dataBuffer[(i+1) * 2] = static_cast<uint8_t>((source[i] >> 8) & 0xFF);
        dataBuffer[(i+1) * 2 + 1] = static_cast<uint8_t>(source[i] & 0xFF);
    }

    return m_i2cTx(m_i2cAddr, reinterpret_cast<uint8_t*>(dataBuffer), sizeof(dataBuffer), 200);
}

bool Mlx90640::readMemory(uint16_t regAddr, uint16_t* buffer, uint16_t size)
{
    if (!m_i2cRxDMA || !buffer || size == 0)
        return false;

    return m_i2cRxDMA(m_i2cAddr, regAddr, reinterpret_cast<uint8_t*>(buffer), size * sizeof(uint16_t));
}

void Mlx90640::shiftBufferBytes()
{
    const uint32_t bufferSize = m_PixelsNum + MLX90640_AUX_NUM + 1; // Pixels + Aux + Control
    for (int i = 0; i < bufferSize; i++)
    {
        auto dataU8 = reinterpret_cast<uint8_t*>(&m_rawBuffer[i]);
        m_rawBuffer[i] = (((dataU8[0] << 8) & 0xFF00) | (dataU8[1] & 0x00FF));
    }
}

bool Mlx90640::dataHandle()
{
    shiftBufferBytes();
    if (ValidateAuxData(m_auxArea) != MLX90640_NO_ERROR)
        return false;
    if (ValidateFrameData(m_pixelsArea) != MLX90640_NO_ERROR)
        return false;

    float emissivity = 0.95;
    float Ta = MLX90640_GetTa(m_pixelsArea, &m_params);
    float tr = Ta - TA_SHIFT;
    MLX90640_CalculateTo(m_pixelsArea, &m_params, emissivity, tr, m_measurBuffer);
    if (*m_stateArea == 0)
        m_subpage0 = true;
    else
        m_subpage1 = true;

    if (m_subpage0 && m_subpage1)  // All picture has been measured
    {
        m_subpage0 = false;
        m_subpage1 = false;
        return true;
    }
    return false;
}

}   // namespace thermomatrix
