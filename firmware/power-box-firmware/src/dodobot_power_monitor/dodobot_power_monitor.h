#pragma once
#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <dodobot_serial/dodobot_serial.h>

class DodobotPowerMonitor
{
public:
    DodobotPowerMonitor(DodobotSerial* serial);
    void begin();
    void update();

private:
    const uint32_t UPDATE_DELAY_MS = 250;
    Adafruit_INA219* ina;
    DodobotSerial* serial;

    float shuntvoltage = 0.0;
    float busvoltage = 0.0;
    float current_mA = 0.0;
    float loadvoltage = 0.0;
    float power_mW = 0.0;

    uint32_t update_timer = 0;
};
