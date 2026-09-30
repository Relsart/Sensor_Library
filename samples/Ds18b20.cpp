#include "Dallas18B20.h"
#include "driver/OneWire.h"
#include "Loger.h"

/* Static/global sensors structs storage: */
static constexpr uint8_t sensCount = 2;
sensor::D18B20::Request thermozond[sensCount]
{
    { .id = {.romArray = {0x28, 0x27, 0x7A, 0xAE, 0xB0, 0x21, 0x08, 0x3A}}},  /* Sensor in tube */
    { .id = {.romArray = {0x28, 0xFF, 0x64, 0x0E, 0x74, 0x06, 0xBE, 0xA5}}}   /* Discrete element */
};

driver::OneWire oneWire(GPIOD, 8);                 // OneWire interface driver
SoftTimer termoTimer;                              // Timer (software, RTOS or IRQ-based) for making measurements waiting intervals
sensor::D18B20 thermometer(oneWire, termoTimer);   // Ds18b20 driver instance

/* Ready data handler: shows current temperatures and requests next measurements: */
void showTemp()
{
    /* Sensors unique IDs, measurements results and valid flags are stored in static sensor::D18B20::Request structures */
    for (int i = 0; i < sensCount; i++)
        Log(lmSystem, Info) << "Temperature from sensor #" << i << ": " << thermozond[i].measuredResult << " C";

    thermometer.takeMeasure(thermozond, sensCount);  // Request next measurements cycle
}


int main(void)
{
    /* System clock and periphery init... */

    /* Bus driver initialization: */
    oneWire.init();
    oneWire.resetBus();

    /* Sensor driver initialization: subscribe callback */
    thermometer.setCallback(etl::delegate<void()>::create<showTemp>());

    /* Sensors initialization: set resolution (if necessary): */
    bool ok = false;
    for (int i = 0; i < sensCount; i++)
    {
        ok = thermometer.init(&thermozond[i].id, sensor::D18B20::Resolution::Res_12Bit);
    }

    /* Checking resolution (if needed): */
    auto resol = thermometer.getResolution(&thermozond[0].id);
    resol = thermometer.getResolution(&thermozond[1].id);

    /* Starting the measurements cycle: */
    thermometer.takeMeasure(thermozond, sensCount);

    /* Now measurements will be innvoked with defined intervals according to selected resolution */

    while(true);
    return 1;
}
