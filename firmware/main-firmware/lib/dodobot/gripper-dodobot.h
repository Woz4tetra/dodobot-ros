
#ifndef __DODOBOT_GRIPPER_H__
#define __DODOBOT_GRIPPER_H__

#include <Arduino.h>
#include <Servo.h>

#include "dodobot.h"


namespace dodobot_gripper
{
    // Gripper servo
    Servo gripper_servo;
    const int GRIPPER_PIN = 17;

    #define INVERT_GRIPPER_SERVO

    const int MAX_POS = 180;
    const int MIN_POS = 0;
    int OPEN_POS = 0;
    int CLOSE_POS = 180;
    const int DEFAULT_GRIP_THRESHOLD = 40;
    const int NEAR_POS_THRESHOLD = 30;

    uint32_t fsr_timer = 0;
    const uint32_t FSR_UPDATE_DELAY_MS = 1;

    uint32_t grip_report_timer = 0;
    const uint32_t DEFAULT_GRIP_UPDATE_DELAY_MS = 5;
    const uint32_t SLOW_GRIP_UPDATE_DELAY_MS = 15;
    uint32_t GRIP_UPDATE_DELAY_MS = DEFAULT_GRIP_UPDATE_DELAY_MS;

    int gripper_pos = 0;
    bool gripper_attached = false;

    int grip_threshold = 0;
    int pos_threshold = 0;
    bool grip_reached = false;

    // FSRs
    const int FSR_LEFT_PIN = 36;
    const int FSR_RIGHT_PIN = 35;

    uint32_t fsr_report_timer = 0;
    const uint32_t FSR_SAMPLERATE_DELAY_MS = 33;

    int get_left_fsr() {
        return analogRead(FSR_LEFT_PIN);
    }

    int get_right_fsr() {
        return analogRead(FSR_RIGHT_PIN);
    }

    bool fsrs_activated(int threshold) {
        return get_left_fsr() > threshold || get_right_fsr() > threshold;
    }

    // Gripper actions

    void set_servo(int pos)
    {
        pos = max(MIN_POS, min(pos, MAX_POS));

        #ifdef INVERT_GRIPPER_SERVO
        gripper_servo.write(MAX_POS - pos);
        #else
        gripper_servo.write(pos);
        #endif

        gripper_pos = pos;
    }

    void report_gripper_pos();

    void open_gripper(int position = -1)
    {
        if (position == -1) {
            position = OPEN_POS;
        }
        set_servo(position);
        grip_reached = true;
        report_gripper_pos();
    }

    void close_gripper(int threshold = -1, int position = MAX_POS) {
        if (threshold < 0) {
            threshold = DEFAULT_GRIP_THRESHOLD;
        }
        grip_threshold = threshold;
        pos_threshold = position;
        grip_reached = false;
    }

    void toggle_gripper(int threshold = -1)
    {
        if (gripper_pos == OPEN_POS) {
            close_gripper(threshold);
        }
        else {
            open_gripper();
        }
    }

    void set_limits(int open_pos, int close_pos) {
        OPEN_POS = open_pos;
        CLOSE_POS = close_pos;
    }

    void update()
    {
        if (!gripper_attached) {
            return;
        }
        if (!dodobot::robot_state.motors_active) {
            return;
        }

        if (grip_reached) {
            return;
        }

        if (CURRENT_TIME - fsr_report_timer > FSR_UPDATE_DELAY_MS)
        {
            if (fsrs_activated(grip_threshold))
            {
                grip_reached = true;
                report_gripper_pos();
                return;
            }
            fsr_report_timer = CURRENT_TIME;
        }

        if (CURRENT_TIME - grip_report_timer < GRIP_UPDATE_DELAY_MS) {
            return;
        }
        grip_report_timer = CURRENT_TIME;

        if (gripper_pos >= pos_threshold - NEAR_POS_THRESHOLD)
        {
            // if gripper is its goal, slow down
            GRIP_UPDATE_DELAY_MS = SLOW_GRIP_UPDATE_DELAY_MS;
        }
        else {
            GRIP_UPDATE_DELAY_MS = DEFAULT_GRIP_UPDATE_DELAY_MS;
        }
        
        if (gripper_pos >= CLOSE_POS || gripper_pos >= pos_threshold) {
            grip_reached = true;
            report_gripper_pos();
            return;
        }
        set_servo(gripper_pos + 1);
    }

    void set_active(bool state) {
        if (state && dodobot::robot_state.motors_active) {
            if (!gripper_attached) {
                gripper_servo.attach(GRIPPER_PIN);
                gripper_attached = true;
            }
            open_gripper();
        }
    }

    void setup_gripper()
    {
        set_active(false);

        pinMode(FSR_LEFT_PIN, INPUT);
        pinMode(FSR_RIGHT_PIN, INPUT);

        dodobot_serial::println_info("Gripper ready");
    }

    bool read_fsrs()
    {
        if (CURRENT_TIME - fsr_report_timer < FSR_SAMPLERATE_DELAY_MS) {
            return false;
        }
        fsr_report_timer = CURRENT_TIME;
        return true;
    }

    void report_fsrs()
    {
        if (!dodobot::robot_state.is_reporting_enabled) {
            return;
        }
        if (!dodobot::robot_state.motors_active) {
            return;
        }
        dodobot_serial::data->write("fsr", "udd", CURRENT_TIME, get_left_fsr(), get_right_fsr());
        // dodobot_serial::info->write("fsr", "udd", CURRENT_TIME, get_left_fsr(), get_right_fsr());
    }

    void report_gripper_pos()
    {
        if (!dodobot::robot_state.is_reporting_enabled) {
            return;
        }
        DODOBOT_SERIAL_WRITE_BOTH("grip", "ud", CURRENT_TIME, gripper_pos);
        // dodobot_serial::info->write("grip", "ud", CURRENT_TIME, gripper_pos);
    }
};


#endif  // __DODOBOT_GRIPPER_H__
