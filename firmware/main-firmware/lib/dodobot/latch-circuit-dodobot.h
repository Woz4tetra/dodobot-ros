#ifndef __DODOBOT_LATCH_CIRCUIT_H__
#define __DODOBOT_LATCH_CIRCUIT_H__

#include <Arduino.h>

#include "dodobot.h"

namespace dodobot_latch_circuit
{
    const int UNLATCH_PIN = 24;
    const int LATCH_STAT_PIN = 25;
    const int BUTTON_LED_PIN = 14;
    const int USB_SENSE_PIN = 7;

    struct state {
        bool usb_connected_once;
        bool usb_voltage_state;
        uint32_t shutdown_timer;
    } state;

    const uint32_t USB_CHECK_INTERVAL_MS = 100;
    uint32_t usb_check_timer = 0;

    const uint32_t POWER_OFF_DEFAULT_THRESHOLD_MS = 3000;
    const uint32_t POWER_OFF_FIRST_TIME_THRESHOLD_MS = 30000;
    uint32_t POWER_OFF_THRESHOLD_MS = POWER_OFF_FIRST_TIME_THRESHOLD_MS;
    uint32_t POWER_OFF_WARN_THRESHOLD_MS = 2500;
    uint32_t power_off_check_timer = 0;

    const uint32_t LED_CYCLE_INTERVAL_US = 1000;
    uint32_t led_cycle_timer = 0;

    int led_val = 0;
    bool button_state = false;
    bool prev_button_state = false;
    bool is_shutting_down = false;

    void unlatch() {
        digitalWrite(UNLATCH_PIN, HIGH);
    }

    bool is_usb_voltage_high() {
        return digitalRead(USB_SENSE_PIN) == HIGH;
    }

    void init_state()
    {
        state.usb_connected_once = false;
        state.usb_voltage_state = false;
        state.shutdown_timer = UINT_MAX;
    }

    void update_state() {
        state.usb_voltage_state = is_usb_voltage_high();
    }

    bool is_usb_connected()
    {
        if (state.usb_voltage_state) {
            power_off_check_timer = CURRENT_TIME;
            state.shutdown_timer = UINT_MAX;
        }
        else {
            state.shutdown_timer = CURRENT_TIME - power_off_check_timer;
            if (CURRENT_TIME - power_off_check_timer > POWER_OFF_WARN_THRESHOLD_MS) {
                DODOBOT_SERIAL_WRITE_BOTH("unlatch", "u", CURRENT_TIME);
            }

            if (CURRENT_TIME - power_off_check_timer > POWER_OFF_THRESHOLD_MS) {
                return false;
            }
        }
        return true;
    }

    bool is_button_pushed() {
        return digitalRead(LATCH_STAT_PIN) == LOW;
    }

    void set_button_led(int val) {
        led_val = val;
        if (led_val < 0) {
            led_val = 0;
        }
        else if (led_val > 255) {
            led_val = 255;
        }
        analogWrite(BUTTON_LED_PIN, val);
    }

    void setup_latch()
    {
        pinMode(UNLATCH_PIN, OUTPUT);
        pinMode(LATCH_STAT_PIN, INPUT_PULLUP);
        pinMode(USB_SENSE_PIN, INPUT_PULLUP);
        pinMode(BUTTON_LED_PIN, OUTPUT);

        digitalWrite(UNLATCH_PIN, LOW);
        set_button_led(255);

        prev_button_state = is_button_pushed();

        dodobot_serial::println_info("Latch ready");
    }

    bool is_incrementing = false;
    void cycle_led()
    {
        if (micros() - led_cycle_timer < LED_CYCLE_INTERVAL_US) {
            return;
        }
        led_cycle_timer = micros();

        if (is_incrementing) {
            set_button_led(led_val + 1);
        }
        else {
            set_button_led(led_val - 1);
        }
        if (led_val == 0) {
            is_incrementing = true;
        }
        else if (led_val == 255) {
            is_incrementing = false;
        }
    }

    void shutdown_callback();

    void shutdown() {
        is_shutting_down = true;
        // dodobot::set_motors_active(false);
        shutdown_callback();
    }

    void update()
    {
        if (button_state) {
            if (!is_shutting_down) {
                cycle_led();
            }
        }
        else if (is_shutting_down) {
            set_button_led(0);
        }
        else {
            set_button_led(255);
        }

        if (CURRENT_TIME - usb_check_timer < USB_CHECK_INTERVAL_MS) {
            return;
        }
        usb_check_timer = CURRENT_TIME;

        update_state();
        if (state.usb_connected_once) {
            if (!is_usb_connected()) {
                unlatch();
            }
        }
        else {
            if (is_usb_voltage_high()) {
                state.usb_connected_once = true;
                POWER_OFF_THRESHOLD_MS = POWER_OFF_DEFAULT_THRESHOLD_MS;
            }
        }

        button_state = is_button_pushed();

        if (prev_button_state != button_state) {
            DODOBOT_SERIAL_WRITE_BOTH("latch_btn", "ud", CURRENT_TIME, button_state);

            prev_button_state = button_state;
        }
    }
};

#endif  // __DODOBOT_LATCH_CIRCUIT_H__
