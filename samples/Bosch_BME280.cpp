#include "driver/i2c/I2c.h"
#include "driver/Dwt.h"
#include "BoschBME280.h"
#include "Loger.h"

driver::I2C i2cPort(I2C1);

int main(void)
{
    /* System clock and periphery init... */

    /* Init I2C periphery driver: */
    i2cPort.init(driver::I2C::Speed::Std100kHz);

    /* Create two sensors instances (for different i2c slave addresses, by connecting SDO pin of one of them to VCC): */
    sensor::BMx280 bme280sensor(i2cPort, driver::DwtTimer::getInstance(), sensor::BMx280::I2C_addr);
    sensor::BMx280 bmp280sensor(i2cPort, driver::DwtTimer::getInstance(), sensor::BMx280::I2C_altAddr);

    /* Sensors initialization: */
    bool ok = false;
    ok = bme280sensor.init();
    ok = bmp280sensor.init();

    float temperature = 0.0F;
    float pressure = 0.0F;
    float humidity = 0.0F;

    /* Get measurements: */
    if (bme280sensor.getMeasurements(temperature, pressure, humidity))
        Log(lmSystem, Info) << "Temp: " << temperature << ", Pressure: " << pressure << ", Humidity: " << humidity;

    if (bmp280sensor.getMeasurements(temperature, pressure))
        Log(lmSystem, Info) << "Temp: " << temperature << ", Pressure: " << pressure;
}

