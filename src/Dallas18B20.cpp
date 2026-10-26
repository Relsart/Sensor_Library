#include "Dallas18B20.h"
#include "Loger.h"

namespace sensor
{

D18B20::D18B20(i_face::I_OneWire& oneWireDriver, i_face::I_Timer& timer) : 
    m_bus(oneWireDriver), m_timer(timer)
{
    m_timer.setCallback(i_face::I_Timer::Callback::create<D18B20, &D18B20::timerHandler>(*this));
}

void D18B20::setCallback(Callback h)
{
    m_readyClbk = h;
}

bool D18B20::isBusy()
{
    return (!m_bus.readBit());
}

bool D18B20::takeMeasure(Request* request, uint32_t count)
{
    if (!request || count == 0)
        return false;

    /* Save request pointers: */
    m_request = request;
    m_requestCount = count;

    /* Conversion command to all sensors at the bus */
    m_bus.resetBus();
    m_bus.writeByte(static_cast<uint8_t>(i_face::I_OneWire::Comamnds::SkipRom));
    m_bus.writeByte(static_cast<uint8_t>(SpecificCmds::ConvertStart));
    m_timer.stop();
    m_timer.start(m_timerSet);
    return true;
}

void D18B20::timerHandler()
{
    /* Checking: all sensors have finished their measurements (line released) */
    if (!m_bus.readBit())
        Log(lmExtLibs, Error) << "D18B20: not all sensors are ready";

    /* Read the requested measurements results: */
    for (int i = 0; i < m_requestCount; i++)
    {
        m_request[i].valid = false;
        if (!readScratchpad(&m_request[i].id))
        {
            Log(lmExtLibs, Error) << "DS18B20: " << Log::Base::Hex << m_request[i].id.romPack << " reading failed";
            continue;
        }
        uint16_t tempRaw = m_scratchBuffer[0] | (m_scratchBuffer[1] << 8U);;
	    Resolution resolution = static_cast<Resolution>((m_scratchBuffer[4] & 0x60U) >> 5U);
        auto meas = calcTemperature(tempRaw, resolution);
        if (meas.has_value())
        {
            m_request[i].measuredResult = meas.value();
            m_request[i].valid = true;
        }
    }
    /* Notify the consumer about results: */
    if (m_readyClbk)
        m_readyClbk();
}

bool D18B20::readScratchpad(const i_face::I_OneWire::DeviceRomId* sensId)
{
    if (!sensId || sensId->romStruct.familyCode != m_FamilyCode)
        return false;

    m_bus.resetBus();
    select(sensId);
    m_bus.writeByte(static_cast<uint8_t>(SpecificCmds::ScratchpadRead));

    for (uint8_t i = 0; i < m_ScratchpadSize; i++)
    {
        m_scratchBuffer[i] = m_bus.readByte();
    }

    return (m_bus.calcCrc(m_scratchBuffer, 8) == m_scratchBuffer[8]);
}

bool D18B20::writeScratchpad(const i_face::I_OneWire::DeviceRomId* sensId)
{
    if (!sensId || sensId->romStruct.familyCode != m_FamilyCode)
        return false;

    m_bus.resetBus();
    select(sensId);
    m_bus.writeByte(static_cast<uint8_t>(SpecificCmds::ScratchpadWrite));
    for (int i = 2; i < 5; i++) // Only 3 bytes available for writing by this command!
    {
        m_bus.writeByte(m_scratchBuffer[i]);
    }

    m_bus.resetBus();
    select(sensId);
    m_bus.writeByte(static_cast<uint8_t>(SpecificCmds::ScratchpadCopy));
    return true;
}

D18B20::Resolution D18B20::getResolution(const i_face::I_OneWire::DeviceRomId* sensId)
{
    if (!sensId || !readScratchpad(sensId))
        return Resolution::ResNone;
    uint8_t value = ((m_scratchBuffer[4] & 0x60) >> 5U);
    Resolution result = (value <= Resolution::Res_12Bit) ? static_cast<Resolution>(value) : Resolution::ResNone;
    return result;
}

etl::optional<float> D18B20::calcTemperature(uint16_t raw, Resolution resolution)
{
    int16_t signedRaw = static_cast<int16_t>(raw);
    switch (resolution)
    {
    case Resolution::Res_9Bit:
        signedRaw &= 0xFFF8;    // Mask 3 low bits
        break;
    case Resolution::Res_10Bit:
        signedRaw &= 0xFFFC;    // Mask 2 low bits
        break;
    case Resolution::Res_11Bit:
        signedRaw &= 0xFFFE;    // Mask 1 low bit
        break;
    case Resolution::Res_12Bit: break;
    default:
        return etl::nullopt;
    }
    return (static_cast<float>(signedRaw) * m_Scale);
}

bool D18B20::init(const i_face::I_OneWire::DeviceRomId* sensId, Resolution resSet)
{
    if (!sensId || !readScratchpad(sensId))
        return false;
    
    /* Write and save the new resolution value: */
    const uint8_t ResolPos = 5;
    m_scratchBuffer[4] &= ~(3U << ResolPos);
    m_scratchBuffer[4] |= (static_cast<uint8_t>(resSet) << ResolPos);
    uint8_t temp = m_scratchBuffer[4];
    writeScratchpad(sensId);

    /* Read and check resolution to be properly applied: */
    if (!readScratchpad(sensId) || (m_scratchBuffer[4] != temp))
    {
        Log(lmExtLibs, Error) << "DS18B20: resolution saving error";
        return false;
    }
    
    /* Set the waiting interval according to selected resolution: */
    switch (resSet)
    {
    case Resolution::Res_9Bit:  m_timerSet = m_WaitTimes[0];
        break;
    case Resolution::Res_10Bit: m_timerSet = m_WaitTimes[1];
        break;
    case Resolution::Res_11Bit: m_timerSet = m_WaitTimes[2];
        break;
    case Resolution::Res_12Bit: m_timerSet = m_WaitTimes[3];
        break;
    default:
        break;
    }
    return true;
}

void D18B20::select(const i_face::I_OneWire::DeviceRomId* sensId)
{
    if (!sensId)
        return;

    m_bus.writeByte(static_cast<uint8_t>(i_face::I_OneWire::Comamnds::MatchRom));
    for (uint8_t i = 0; i < 8; i++)
    {
        m_bus.writeByte(sensId->romArray[i]);
    }
}

}   // namespace sensor
