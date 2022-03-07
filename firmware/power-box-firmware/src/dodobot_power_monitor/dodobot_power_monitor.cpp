#include <dodobot_power_monitor/dodobot_power_monitor.h>

DodobotPowerMonitor::DodobotPowerMonitor(DodobotSerial* serial)
{
    ina = new Adafruit_INA219();
    this->serial = serial;
}

void DodobotPowerMonitor::begin()
{
    ina->begin();
}

void DodobotPowerMonitor::update()
{
    if (millis() - update_timer < UPDATE_DELAY_MS) {
        return;
    }
    update_timer = millis();

    shuntvoltage = ina->getShuntVoltage_mV();
    busvoltage = ina->getBusVoltage_V();
    current_mA = ina->getCurrent_mA();
    power_mW = ina->getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    serial->write("power", "fffff", shuntvoltage, busvoltage, current_mA, power_mW, loadvoltage);
}
