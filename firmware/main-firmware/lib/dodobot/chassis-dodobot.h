#ifndef __DODOBOT_CHASSIS_H__
#define __DODOBOT_CHASSIS_H__

#include <Arduino.h>
#include <TB6612.h>
#include <Encoder.h>

#include "dodobot.h"

namespace dodobot_chassis
{
    bool is_active = false;

    // DC motors
    const int MOTORA_DR1 = 27;
    const int MOTORA_DR2 = 28;
    const int MOTORB_DR1 = 38;
    const int MOTORB_DR2 = 37;
    const int MOTORA_PWM = 29;
    const int MOTORB_PWM = 30;
    const int BUMP1_PIN = 3;
    const int BUMP2_PIN = 5;

    const int MOTOR_COMMAND_TIMEOUT_MS = 1000;

    uint32_t prev_commandA_time = 0;
    uint32_t prev_commandB_time = 0;

    uint32_t bumper_timer = 0;
    const int BUMPER_SAMPLERATE_DELAY_MS = 100;

    TB6612 motorA(MOTORA_PWM, MOTORA_DR1, MOTORA_DR2);
    TB6612 motorB(MOTORB_PWM, MOTORB_DR2, MOTORB_DR1);

    // Encoders
    const int MOTORA_ENCA = 23;
    const int MOTORA_ENCB = 22;
    const int MOTORB_ENCA = 21;
    const int MOTORB_ENCB = 20;

    const int ENCODER_SAMPLERATE_DELAY_MS = 10;

    Encoder motorA_enc(MOTORA_ENCB, MOTORA_ENCA);
    Encoder motorB_enc(MOTORB_ENCA, MOTORB_ENCB);

    long encA_pos, encB_pos = 0;
    double enc_speedA, enc_speedB = 0.0;  // ticks/s, smoothed
    double enc_speedA_raw, enc_speedB_raw = 0.0;  // ticks/s

    uint32_t prev_enc_time = 0;

    double speed_smooth_kA = 0.9;
    double speed_smooth_kB = 0.9;

    bool bump1_state = false;
    bool bump2_state = false;
    bool debounce_prev_bump1_state = false;
    bool debounce_prev_bump2_state = false;
    uint32_t last_debounce_time_1 = 0;
    uint32_t last_debounce_time_2 = 0;
    uint32_t debounce_delay = 25;

    // Bumpers
    bool is_bump1_active() {
        return digitalRead(BUMP1_PIN) == LOW;
    }

    bool is_bump2_active() {
        return digitalRead(BUMP2_PIN) == LOW;
    }

    void setup_bumpers()
    {
        pinMode(BUMP1_PIN, INPUT_PULLUP);
        pinMode(BUMP2_PIN, INPUT_PULLUP);
        bump1_state = is_bump1_active();
        bump2_state = is_bump2_active();
    }

    bool bump1_changed()
    {
        bool new_state = is_bump1_active();
        if (debounce_prev_bump1_state != new_state) {
            last_debounce_time_1 = CURRENT_TIME;
        }
        debounce_prev_bump1_state = new_state;
        if (CURRENT_TIME - last_debounce_time_1 > debounce_delay)
        {
            if (bump1_state != new_state)
            {
                bump1_state = new_state;
                return true;
            }
        }
        return false;
    }
    bool bump2_changed()
    {
        bool new_state = is_bump2_active();
        if (debounce_prev_bump2_state != new_state) {
            last_debounce_time_2 = CURRENT_TIME;
        }
        debounce_prev_bump2_state = new_state;
        if (CURRENT_TIME - last_debounce_time_2 > debounce_delay)
        {
            if (bump2_state != new_state)
            {
                bump2_state = new_state;
                return true;
            }
        }
        return false;
    }

    // Speed check functions

    bool is_obstacle_in_front() {
        return false;
    }

    bool is_obstacle_in_back() {
        // return bump1_state || bump2_state;  // TEMPORARILY DISABLE BUMPERS UNTIL ISSUE IS FIXED
        return false;
    }

    bool is_moving() {
        return motorA.getSpeed() != 0 || motorB.getSpeed() != 0;
    }
    bool is_moving(int speedA, int speedB) {  // check a command that's about to send
        return speedA != 0 || speedB != 0;
    }
    bool is_moving_forward() {
        return motorA.getSpeed() + motorB.getSpeed() >= 0;
    }
    bool is_moving_forward(int speedA, int speedB) {
        return (speedA + speedB) >> 1 >= 0;
    }

    // DC motor functions

    void setup_motors()
    {
        motorA.begin();
        motorB.begin();
    }

    void reset_motor_timeouts()
    {
        prev_commandA_time = CURRENT_TIME;
        prev_commandB_time = CURRENT_TIME;
    }

    void set_motorA(int speed) {
        if (!is_active) {
            return;
        }
        reset_motor_timeouts();
        motorA.setSpeed(speed);
    }

    void set_motorB(int speed) {
        if (!is_active) {
            return;
        }
        reset_motor_timeouts();
        motorB.setSpeed(speed);
    }

    void set_motors(int speedA, int speedB)
    {
        if (is_obstacle_in_front() && is_moving_forward(speedA, speedB)) {  // if an obstacle is detected in the front, only allow backwards commands
            speedA = 0;
            speedB = 0;
        }
        if (is_obstacle_in_back() && !is_moving_forward(speedA, speedB)) {  // if an obstacle is detected in the back, only allow forwards commands
            speedA = 0;
            speedB = 0;
        }
        set_motorA(speedA);
        set_motorB(speedB);
    }


    bool check_motor_timeout()
    {
        if (!is_active) {
            return true;
        }
        bool timedout = false;
        if (CURRENT_TIME - prev_commandA_time > MOTOR_COMMAND_TIMEOUT_MS) {
            motorA.setSpeed(0);
            timedout = true;
        }
        if (CURRENT_TIME - prev_commandB_time > MOTOR_COMMAND_TIMEOUT_MS) {
            motorB.setSpeed(0);
            timedout = true;
        }

        return timedout;
    }


    void stop_motors() {
        if (!is_active) {
            return;
        }
        motorA.setSpeed(0);
        motorB.setSpeed(0);
    }

    // Encoder functions
    void reset_encoders()
    {
        encA_pos = 0;
        encB_pos = 0;
        motorA_enc.write(0);
        motorB_enc.write(0);
    }

    bool read_encoders()
    {
        if (CURRENT_TIME - prev_enc_time < ENCODER_SAMPLERATE_DELAY_MS) {
            return false;
        }

        long new_encA_pos = motorA_enc.read();
        long new_encB_pos = motorB_enc.read();

        // bool should_report = false;
        // if (new_encA_pos != encA_pos || new_encB_pos != encB_pos) {
        //     should_report = true;
        // }

        enc_speedA_raw = (double)(new_encA_pos - encA_pos) / (CURRENT_TIME - prev_enc_time) * 1000.0;
        enc_speedB_raw = (double)(new_encB_pos - encB_pos) / (CURRENT_TIME - prev_enc_time) * 1000.0;
        enc_speedA += speed_smooth_kA * (enc_speedA_raw - enc_speedA);
        enc_speedB += speed_smooth_kB * (enc_speedB_raw - enc_speedB);

        encA_pos = new_encA_pos;
        encB_pos = new_encB_pos;

        prev_enc_time = CURRENT_TIME;

        // return should_report;
        return true;
    }

    void report_encoders()
    {
        if (!dodobot::robot_state.is_reporting_enabled) {
            return;
        }
        if (!dodobot::robot_state.motors_active) {
            return;
        }
        dodobot_serial::data->write("enc", "uddff", CURRENT_TIME, encA_pos, encB_pos, enc_speedA, enc_speedB);
    }

    void setup_chassis()
    {
        reset_encoders();
        setup_motors();
        setup_bumpers();
        dodobot_serial::println_info("Drive motors ready");
    }

    void update()
    {
        if (read_encoders()) {
            report_encoders();
        }
        if ((CURRENT_TIME - bumper_timer > BUMPER_SAMPLERATE_DELAY_MS) || bump1_changed() || bump2_changed()) {
            bumper_timer = CURRENT_TIME;
            DODOBOT_SERIAL_WRITE_BOTH("bump", "udd", CURRENT_TIME, bump1_state, bump2_state);
        }
        check_motor_timeout();
    }

    void set_active(bool state)
    {
        if (state == is_active) {
            return;
        }
        is_active = state;

        stop_motors();
        reset_encoders();
    }
}

#endif  // __DODOBOT_CHASSIS_H__
