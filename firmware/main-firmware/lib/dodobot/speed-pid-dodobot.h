#ifndef __DODOBOT_SPEED_PID_H__
#define __DODOBOT_SPEED_PID_H__

#include <Arduino.h>
#include "dodobot.h"
#include "chassis-dodobot.h"

namespace dodobot_speed_pid
{
    // 160 rpm @ 6V
    // 135 rpm @ 5V
    // 60 rpm @ 3V
    // double max_rpm = 135.0;
    // double max_linear_speed_cps = max_rpm * 2.0 * PI * wheel_radius_cm / 60.0;  // cm per s, no load
    // double max_linear_speed_cps = 915.0;
    // double cps_to_cmd = 255.0 / max_linear_speed_cps;
    double max_linear_speed_tps_A = 5200.0;  // max speed, with load measured in ticks per second
    double max_linear_speed_tps_B = 5000.0;
    double tps_to_cmd_A = 255.0 / max_linear_speed_tps_A;
    double tps_to_cmd_B = 255.0 / max_linear_speed_tps_B;
    double min_tps = 0.0;
    double max_error_sum = 500000.0;

    const unsigned int NUM_PID_KS = 8;
    double* pid_Ks = new double[NUM_PID_KS];

    const int PID_COMMAND_TIMEOUT_MS = 1000;
    const int PID_UPDATE_DELAY_MS = 10;  // 100 Hz
    uint32_t prev_pid_time = 0;

    int motorA_cmd = 0;
    int motorB_cmd = 0;

    double sign(double x) {
        return (x > 0) - (x < 0);
    }

    class PID {
    private:
        String name;
        double K_ff;  // feedforward constant. (ticks per s to motor command conversion)
        double deadzone;
        double target;
        double error_sum, prev_error;
        double error_sum_clamp;
        double feedforward;
        uint32_t prev_setpoint_time;
        uint32_t current_time, prev_update_time;
        double dt;
        double out;
        bool is_timed_out;

    public:
        double Kp, Ki, Kd;

        PID(String _name, double _deadzone, double _K_ff, double _error_sum_clamp):
            name(_name),
            K_ff(_K_ff),
            deadzone(_deadzone),
            error_sum_clamp(_error_sum_clamp)
        {
            target = 0.0;
            error_sum = 0.0;
            prev_error = 0.0;
            feedforward = 0.0;
            prev_setpoint_time = 0;
            current_time = 0;
            prev_update_time = 0;
            dt = 0.0;
            out = 0.0;
            is_timed_out = false;

            Kp = 0.01;
            Ki = 0.0;
            Kd = 0.0;
        }

        bool timed_out() {
            return is_timed_out;
        }

        void set_target(double _target) {
            feedforward = K_ff * _target;
            target = _target;
            prev_setpoint_time = CURRENT_TIME;
            prev_update_time = micros();
            is_timed_out = false;
        }

        double get_target() {
            return target;
        }

        void reset() {
            prev_error = 0.0;
            error_sum = 0.0;
            set_target(0.0);
        }

        int limit(double value) {
            if (value > 255.0) {
                return 255;
            }
            if (value < -255.0) {
                return -255;
            }
            return (int)(value);
        }

        int compute(double measurement)
        {
            if (!is_timed_out && CURRENT_TIME - prev_setpoint_time > PID_COMMAND_TIMEOUT_MS) {
                reset();
                is_timed_out = true;
                dodobot_serial::println_info("PID '%s' setpoint timed out", name.c_str());
            }

            if (micros() - prev_update_time == 0) {
                return out;
            }
            else if (micros() - prev_update_time < 0) {  // edge case for timer looping
                prev_update_time = micros();
                return out;
            }

            double error = target - measurement;
            current_time = micros();
            dt = (current_time - prev_update_time) * 1E-6;
            prev_update_time = current_time;

            out = 0.0;
            if (abs(target) < deadzone) {
                return 0;
            }
            if (Kp != 0.0) {
                out += Kp * error;
            }
            if (Kd != 0.0) {
                out += Kd * (error - prev_error) / dt;
                prev_error = error;
            }
            if (Ki != 0.0) {
                out += Ki * error_sum * dt;
                error_sum += error;
                if (abs(error_sum) > error_sum_clamp) {
                    error_sum = sign(error_sum) * error_sum_clamp;
                }
            }
            out += feedforward;

            return limit(out);
        }
    };

    PID motorA_pid("A", min_tps, tps_to_cmd_A, max_error_sum);
    PID motorB_pid("B", min_tps, tps_to_cmd_B, max_error_sum);

    void set_Ks()
    {
        motorA_pid.Kp = pid_Ks[0];
        motorA_pid.Ki = pid_Ks[1];
        motorA_pid.Kd = pid_Ks[2];
        motorB_pid.Kp = pid_Ks[3];
        motorB_pid.Ki = pid_Ks[4];
        motorB_pid.Kd = pid_Ks[5];
        dodobot_chassis::speed_smooth_kA = pid_Ks[6];  // defined in chassis-dodobot.h
        dodobot_chassis::speed_smooth_kB = pid_Ks[7];  // defined in chassis-dodobot.h

        DODOBOT_SERIAL_WRITE_BOTH("pidks", "d", 1);

    }

    void failed_to_set_ks() {
        DODOBOT_SERIAL_WRITE_BOTH("pidks", "d", 0);
    }

    void reset_pid() {
        motorA_pid.reset();
        motorB_pid.reset();
    }

    void setup_pid()
    {
        for (size_t index = 0; index < NUM_PID_KS; index++){
            pid_Ks[index] = 0.0;
        }
        pid_Ks[0] = motorA_pid.Kp;
        pid_Ks[1] = motorA_pid.Ki;
        pid_Ks[2] = motorA_pid.Kd;
        pid_Ks[3] = motorB_pid.Kp;
        pid_Ks[4] = motorB_pid.Ki;
        pid_Ks[5] = motorB_pid.Kd;
        reset_pid();

        dodobot_serial::println_info("Speed PID ready");
    }

    void update_setpointA(double new_setpoint) {
        motorA_pid.set_target(new_setpoint);
    }

    void update_setpointB(double new_setpoint) {
        motorB_pid.set_target(new_setpoint);
    }

    void update_speed_pid()
    {
        if (!dodobot::robot_state.is_speed_pid_enabled) {
            return;
        }

        if (CURRENT_TIME - prev_pid_time < PID_UPDATE_DELAY_MS) {
            return;
        }
        prev_pid_time = CURRENT_TIME;

        // inputs: ff_speed (measured - setpoint), ff_setpoint (always 0)
        // output: pid_command (-255..255)

        motorA_cmd = motorA_pid.compute(dodobot_chassis::enc_speedA);
        motorB_cmd = motorB_pid.compute(dodobot_chassis::enc_speedB);

        if (!motorA_pid.timed_out() && !motorB_pid.timed_out()) {
            dodobot_chassis::set_motors(motorA_cmd, motorB_cmd);
        }
    }

    void set_speed_pid(bool enabled) {
        dodobot::robot_state.is_speed_pid_enabled = enabled;
        reset_pid();
    }
};

#endif  // __DODOBOT_SPEED_PID_H__
