
#ifndef __DODOBOT_H__
#define __DODOBOT_H__

#include <Arduino.h>
#include <limits.h>
#include "serial-dodobot.h"

#define CURRENT_TIME millis()
const String ROBOT_NAME = "chansey";
const String ROBOT_AUTHOR = "Ben Warwick";
const String DODOBOT_VERSION = "v0.0";

namespace dodobot
{
    const int MOTOR_STANDBY_PIN = 26;

    uint32_t state_report_timer = 0;
    const uint32_t STATE_SAMPLERATE_DELAY_MS = 500;

    uint32_t prev_loop_timer = 0;
    uint32_t loop_report_sum = 0;
    uint32_t loop_report_counter = 0;

    String network_info;
    const int max_networks_len = 15;
    int network_list_len = 0;
    String* network_list_names = new String[max_networks_len];
    int* network_list_signals = new int[max_networks_len];

    const int max_robot_fns_len = 15;
    int robot_fn_list_len = 0;
    String* robot_functions = new String[max_robot_fns_len];

    uint32_t free_mem()  // for Teensy 3.0
    {
        uint32_t stackTop;
        uint32_t heapTop;

        // current position of the stack.
        stackTop = (uint32_t) &stackTop;

        // current position of heap.
        void* hTop = malloc(1);
        heapTop = (uint32_t) hTop;
        free(hTop);

        // The difference is (approximately) the free, available ram.
        return stackTop - heapTop;
    }
    void soft_restart()
    {
        INFO_SERIAL.end();  // clears the serial monitor if used
        DATA_SERIAL.end();
        SCB_AIRCR = 0x05FA0004;  // write value for restart
    }

    void signal_shutdown() {
        DODOBOT_SERIAL_WRITE_BOTH("control", "sd", "dodobot", 0);
    }

    void signal_reboot() {
        DODOBOT_SERIAL_WRITE_BOTH("control", "sd", "dodobot", 1);
    }

    void signal_restart_ros() {
        DODOBOT_SERIAL_WRITE_BOTH("control", "sd", "dodobot", 2);
    }

    void signal_restart_client() {
        DODOBOT_SERIAL_WRITE_BOTH("control", "sd", "dodobot", 3);
    }

    void signal_restart_microcontroller() {
        DODOBOT_SERIAL_WRITE_BOTH("control", "sd", "dodobot", 4);
        delay(500);
        soft_restart();
    }

    struct state {
        bool is_reporting_enabled;
        bool battery_ok;
        bool motors_active;
        bool is_speed_pid_enabled;
        bool is_ros_ready;
    } robot_state;

    void init_structs() {
        robot_state.is_reporting_enabled = false;
        robot_state.battery_ok = true;
        robot_state.motors_active = false;
        robot_state.is_speed_pid_enabled = false;
        robot_state.is_ros_ready = false;
    }

    void set_motors_active(bool state)
    {
        if (robot_state.motors_active == state) {
            return;
        }
        robot_state.motors_active = state;
        if (state) {  // bring motors out of standby mode
            digitalWrite(MOTOR_STANDBY_PIN, HIGH);
        }
        else {  // set motors to low power
            digitalWrite(MOTOR_STANDBY_PIN, LOW);
        }
        delay(100);  // wait for modules to power off/on
    }

    void setup() {
        init_structs();
        pinMode(MOTOR_STANDBY_PIN, OUTPUT);
        set_motors_active(false);
        // set_motors_active(true);
        // robot_state.is_reporting_enabled = true;
    }

    void report_structs()
    {
        if (!robot_state.is_reporting_enabled) {
            return;
        }
        uint32_t current_time = micros();
        loop_report_sum += current_time - prev_loop_timer;
        loop_report_counter++;
        prev_loop_timer = current_time;

        if (CURRENT_TIME - state_report_timer < STATE_SAMPLERATE_DELAY_MS) {
            return;
        }
        state_report_timer = CURRENT_TIME;
        double loop_rate = (double)loop_report_counter / (double)loop_report_sum * 1E6;

        DODOBOT_SERIAL_WRITE_BOTH("state", "uddfu", CURRENT_TIME,
            robot_state.battery_ok,
            robot_state.motors_active,
            loop_rate,
            free_mem()
        );

        loop_report_sum = 0;
        loop_report_counter = 0;
    }

    void set_network_entry(int index, String network_name, int signal_strength)
    {
        if (index < 0 || index >= max_networks_len) {
            return;
        }
        network_list_names[index] = network_name;
        network_list_signals[index] = signal_strength;
        network_list_len = index + 1;
    }

    void set_robot_fn_entry(int index, String function_name)
    {
        if (index < 0 || index >= max_robot_fns_len) {
            return;
        }
        robot_functions[index] = function_name;
        robot_fn_list_len = index + 1;
    }
}

#endif  // __DODOBOT_H__
