#include "MH-Z19E.h"
#include "Loger.h"

namespace sensor
{

MhZ19E::MhZ19E(i_face::I_Uart& uart) : m_uart(uart), ParserInterface(&m_Preamble, sizeof(m_Preamble), 0)
{
    // Subscribe to serial port data rx interrupt callback:
    m_uart.setRxCallback(i_face::I_Uart::RxCallback::create<MhZ19E, &MhZ19E::getDataISR>(*this));
}

void MhZ19E::setRawDataCallback(Handler h) { m_handler = h; }
void MhZ19E::setConsumer(Consumer h) { m_consumer = h; }

void MhZ19E::getDataISR(i_face::I_RingBuffer& data)
{
    /* ------------ I S R ------------ */
    if (m_handler)
        m_handler();  // Invoke the deferred handler to separate it from the ISR
}

void MhZ19E::handleRawData()
{
    parseNewData(m_uart.getBuffer());
}

void MhZ19E::handleMessage(const uint8_t* data, uint32_t size)
{
    if (size != m_PackSize)
        return; // Failed size!

    memcpy(&m_measur, &data[1], sizeof(m_measur));
    if (m_measur.cmdCode != static_cast<uint8_t>(CmdCodes::GetMeasurement))
        return; // Not a measurements pack!

    uint16_t co2Concentration = __builtin_bswap16(m_measur.co2Conc);
    int8_t temperature = static_cast<int8_t>(m_measur.temperature - 40);

    Log(lmSystem, Info) << "CO2 concentration is" << co2Concentration;
    if (m_consumer)
        m_consumer(co2Concentration, temperature);
}

void MhZ19E::requestMeasurements()
{
    uint8_t requestPack[9]{0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    m_uart.transmit(requestPack, sizeof(requestPack));
}

size_t MhZ19E::getPackSize(const uint8_t*, size_t)
{
    return m_PackSize;   // Fixed pack size
}

bool MhZ19E::checkPackCrc(const uint8_t* data, uint32_t size)
{
    if (!data || size != m_PackSize)
        return false;

    /* This CRC calculating method is not based on the Cyclic Redundancy Check,
    but the simple arithmetic sum of bytes, so it's impossible to use hardware STM32 CRC module */
    uint8_t crc = 0;
    const uint8_t PackBeginPos = 1;
    const uint8_t ChackSumPos = 8;

    for (int i = PackBeginPos; i < ChackSumPos; i++)
    {
        crc += data[i];
    }
    crc = ~crc + 1;
    return (crc == data[ChackSumPos]);
}

}   // namespace sensor
