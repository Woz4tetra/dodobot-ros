#ifndef __DODOBOT_POWER_MONITOR_H__
#define __DODOBOT_POWER_MONITOR_H__

#include <Arduino.h>
#include <Adafruit_INA219_Teensy.h>

#include "dodobot.h"
#include "i2c-dodobot.h"


#define INA_SAMPLERATE_DELAY_MS 500
// #define INA_SAMPLERATE_DELAY_MS 10
#define INA_VOLTAGE_THRESHOLD 8.0

/*
 * Adafruit High-side current and voltage meter
 * INA219
 */

namespace dodobot_power_monitor
{
    Adafruit_INA219 ina219;
    float ina219_shuntvoltage = 0.0;
    float ina219_busvoltage = 0.0;
    float ina219_current_mA = 0.0;
    float ina219_loadvoltage = 0.0;
    float ina219_power_mW = 0.0;
    uint32_t ina_report_timer = 0;

    float FULL_VOLTAGE = 12.15;
    float OK_VOLTAGE = 10.5;
    float LOW_VOLTAGE = 9.25;
    float CRITICAL_VOLTAGE = 8.8;

    void setup_INA219()
    {
        ina219.begin(&I2C_BUS_1);
        dodobot_serial::println_info("INA219 initialized.");
    }

    void check_voltage()
    {
        bool status = ina219_loadvoltage > INA_VOLTAGE_THRESHOLD;
        if (dodobot::robot_state.battery_ok != status && !status) {
            dodobot_serial::println_error("INA reports battery is critically low!");
        }
        dodobot::robot_state.battery_ok = status;
    }

    bool read_INA219()
    {
        if (CURRENT_TIME - ina_report_timer < INA_SAMPLERATE_DELAY_MS) {
            return false;
        }
        ina_report_timer = CURRENT_TIME;
        ina219_shuntvoltage = ina219.getShuntVoltage_mV();
        ina219_busvoltage = ina219.getBusVoltage_V();
        ina219_current_mA = ina219.getCurrent_mA();
        ina219_power_mW = ina219.getPower_mW();
        ina219_loadvoltage = ina219_busvoltage + (ina219_shuntvoltage / 1000);

        check_voltage();

        return true;
    }
    void report_INA219()
    {
        if (!dodobot::robot_state.is_reporting_enabled) {
            return;
        }
        // dodobot_serial::data->write("ina", "ufff", CURRENT_TIME, ina219_current_mA, ina219_power_mW, ina219_loadvoltage);
        // dodobot_serial::info->write(dodobot_serial::data->get_written_packet());
        DODOBOT_SERIAL_WRITE_BOTH("batt", "ufff", CURRENT_TIME, ina219_current_mA, ina219_power_mW, ina219_loadvoltage);
    }

    const int max_power_level = 5;
    int power_level() {
        if (ina219_loadvoltage >= FULL_VOLTAGE) {
            return 5;
        }
        else if (ina219_loadvoltage >= OK_VOLTAGE) {
            return 4;
        }
        else if (ina219_loadvoltage > LOW_VOLTAGE) {
            return 3;
        }
        else if (ina219_loadvoltage <= CRITICAL_VOLTAGE) {
            return 1;
        }
        else if (ina219_loadvoltage <= LOW_VOLTAGE) {
            return 2;
        }
        else {
            return 0;
        }
    }
};  // namespace rover6_i2c

#endif  // __DODOBOT_POWER_MONITOR_H__
