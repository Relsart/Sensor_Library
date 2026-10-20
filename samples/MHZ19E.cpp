#include "MH-Z19E.h"
#include "Loger.h"
#include "Signal.h"
#include "driver/uart/Uart.h"

driver::uart::Port co2SensPort(UART8, 128);
sensor::MhZ19E co2sensor(co2SensPort);

/* * * * *  Example for non-RTOS (superloop) project  * * * * */

/* Some upper-layered task class: */
class AirControls : public SlotInterface <uint8_t>
{
public:
    AirControls(sensor::MhZ19E& sensor) : m_co2Sensor(sensor)
    {
        m_timer.setCallback(SoftTimer::Callback::create<AirControls, &AirControls::request>(*this));
        m_timer.start(2000, i_face::I_Timer::WorkingMode::Periodic);
        m_co2CalcSignal.connect(this);
        m_co2Sensor.setRawDataCallback(sensor::MhZ19E::Handler::create<AirControls, &AirControls::handle_MHZ19E>(*this));
        m_co2Sensor.setConsumer(sensor::MhZ19E::Consumer::create<AirControls, &AirControls::showMeasurements>(*this));
    }

private:
    sensor::MhZ19E& m_co2Sensor;                // Sensor driver link
    SoftTimer m_timer;                          // Timer for periodically measurements requests
    SignalMainLoop<uint8_t> m_co2CalcSignal;    // Mainloop signal for separating ISR from deferred handling in Main Loop

    void request()
    {
        // Request measurements by the timer:
        m_co2Sensor.requestMeasurements();
    }

    void handle_MHZ19E()
    {
        // Go out from the ISR here..
        m_co2CalcSignal.activ(1);
    }

    void run(uint8_t device, uint32_t)
    {
        // Deferred handling (parsing & calculating) invokes here:
        m_co2Sensor.handleRawData();
    }

    void showMeasurements(uint16_t co2Conc, int8_t temp)
    {
        // Consume ready measurements:
        Log(lmSystem, Info) << "Carbon dioxide concentration: " << co2Conc << "ppm, temperature: " << (uint16_t)temp << "C";
    }
};


int main(void)
{
    /* System clock and periphery init... */
    /* Set the UART baudrate (const 9600) */
    co2SensPort.init(sensor::MhZ19E::baudrate);

    /* Create task object and start its working: */
    AirControls airControl(co2sensor);

    while(true);
    return 1;
}
