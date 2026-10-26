#include "Mlx90640.h"
#include "Signal.h"

void sandbox()
{
    /* Instance creating: */
    Mlx90640 m_thermoDriver(0x33);  // I2C device address
    Signal<const ThermoArray*> m_sendReadyData;

    /* Periphery subscribing: */
    m_thermoDriver.setI2cTx(thermomatrix::Mlx90640::I2C_Tx::create<driver::I2C, &driver::I2C::masterTx>(*m_i2c));
    m_thermoDriver.setI2cTxRx(thermomatrix::Mlx90640::I2C_TxRx::create<driver::I2C, &driver::I2C::masterTxRx>(*m_i2c));
    m_thermoDriver.setI2cRxDMA(thermomatrix::Mlx90640::I2C_RxDMA::create<driver::I2C, &driver::I2C::read16bReg_DMA>(*m_i2c));

    /* Initialization and EEPROM parameters reading: */
    if (!m_thermoDriver.init(thermomatrix::Mlx90640::RefreshRate::Rate8HZ))
        return false;

    if (!m_thermoDriver.readParameters())
        return false;

    /* Working cycle: in RTOS task or in class-handler: */

    /* Waiting for data ready: */
    while (!m_thermoDriver.isFrameReady()){ /*Timeout*/ }

    /* 1st step: Frame request: */
    bool isOk = m_thermoDriver.frameRequest();
    waitForXferCmplt(); // Waiting function or some state maschine in ISR...

    /* 2nd step: AUX adta request: */
    isOk = m_thermoDriver.auxRequest();
    waitForXferCmplt();

    /* 3rd step: Control register request: */
    isOk = m_thermoDriver.controlRequest();
    waitForXferCmplt();

    /* Check results: */
    if (m_thermoDriver.dataHandle())  // Both subpages have calculated:
    {
        m_sendReadyData.activ(m_thermoDriver.getMeasurements());    // Send ready frame to consumers
    }
}
