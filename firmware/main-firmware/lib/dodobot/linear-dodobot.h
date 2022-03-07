#ifndef __DODOBOT_LINEAR_H__
#define __DODOBOT_LINEAR_H__

#include <Arduino.h>
#include <Tic_Teensy.h>
#include <Encoder.h>

#include "dodobot.h"

#define TIC_SERIAL Serial1

#define RESET_PIN A22


namespace dodobot_linear
{
    enum class LinearEvent
    {
        ACTIVE_TRUE = 1,
        ACTIVE_FALSE = 2,
        HOMING_STARTED = 3,
        HOMING_FINISHED = 4,
        MOVE_STARTED = 5,
        MOVE_FINISHED = 6,
        POSITION_ERROR = 7,
        NOT_HOMED = 8,
        NOT_ACTIVE = 9,
        HOMING_FAILED = 10,
    };

    const int baudrate = 115385;
    const int serial_timeout = 1000;
    // TIC Stepper controller
    TicSerial tic(TIC_SERIAL);
    int32_t stepper_pos = 0;
    int32_t stepper_vel = 0;

    TicStepMode step_mode = TicStepMode::Microstep8;
    // TicStepMode step_mode = TicStepMode::Microstep4;
    // TicStepMode step_mode = TicStepMode::Microstep1;

    const int ERROR_PIN = 15;

    // Homing switch
    const int HOMING_PIN = 39;

    // Positioning constants
    const double STEPPER_GEARBOX_RATIO = 26.0 + 103.0 / 121.0;
    const double ENCODER_TICKS_PER_R_NO_GEARBOX = 160.0;
    const double STEPPER_TICKS_PER_R_NO_GEARBOX = 200.0;
    int microsteps = 1;

    const double ENCODER_TICKS_PER_R = ENCODER_TICKS_PER_R_NO_GEARBOX * STEPPER_GEARBOX_RATIO;
    const double ENCODER_R_PER_TICK = 1.0 / ENCODER_TICKS_PER_R;
    double STEPPER_TICKS_PER_R = 1.0;  // depends on microsteps
    double STEPPER_R_PER_TICK = 1.0;  // depends on microsteps
    double ENC_TO_STEP_TICKS = 1.0;  // depends on microsteps

    // const double BELT_PULLEY_RADIUS_MM = 13.58874;
    const double BELT_PULLEY_RADIUS_MM = 12.1;
    double STEP_TICKS_TO_LINEAR_MM = 1.0;  // depends on microsteps

    // CAD full travel is 178.60mm
    const int MAX_POSITION_FULL_STEP = 10625;
    // const int MAX_POSITION = 85000;

    uint32_t MAX_SPEED_FULL_STEP = 31250000;
    // uint32_t MAX_SPEED_FULL_STEP = 250000000;
    // uint32_t MAX_SPEED_FULL_STEP = 300000000;
    // uint32_t MAX_SPEED_FULL_STEP = 420000000;

    const uint32_t FAST_HOMING_SPEED_FULL_STEP = 31250000;
    const uint32_t HOMING_SPEED_FULL_STEP = 3125000;
    const uint32_t HOMING_ACCEL_FULL_STEP = 1250000;

    uint32_t STEPPER_ACCEL_FULL_STEP = 1250000;
    uint32_t STEPPER_DECEL_FULL_STEP = 1250000;

    const int POSITION_BUFFER_FULL_STEP = 60;
    // const int ENCODER_POSITION_ERROR_FULL_STEP = 500;
    const int ENCODER_POSITION_ERROR_FULL_STEP = 200;
    const int START_POS_FULL_STEP = 1250;
    const int HOMING_POS_OFFSET_FULL_STEP = 125;

    int MAX_POSITION = 1;
    uint32_t MAX_SPEED = 1;
    uint32_t FAST_HOMING_SPEED = 1;
    uint32_t HOMING_SPEED = 1;
    uint32_t HOMING_ACCEL = 1;

    int POSITION_BUFFER = 1;
    int ENCODER_POSITION_ERROR = 1;
    uint32_t STEPPER_ACCEL = 1;
    uint32_t STEPPER_DECEL = 1;

    // Encoder
    const int STEPPER_ENCA = 31;
    const int STEPPER_ENCB = 32;
    Encoder stepper_enc(STEPPER_ENCB, STEPPER_ENCA);
    long encoder_pos = 0;  // encoder ticks converted to stepper ticks
    long raw_encoder_pos = 0;  // encoders ticks as counted

    // general flags and variables
    bool is_homed = false;
    bool is_active = false;
    bool is_moving = false;
    bool is_errored_state = false;
    int target_position = 0;
    int target_velocity = 0;

    uint32_t update_timer = 0;
    const uint32_t UPDATE_DELAY_MS = 30;

    enum TicPlanningMode planning_mode = TicPlanningMode::Off;  // 0 = off, 1 = position, 2 = velocity
    uint32_t planning_mode_timer = 0;
    const uint32_t PLANNING_MODE_DELAY_MS = 100;

    bool is_home_pin_active() {
        return digitalRead(HOMING_PIN) == LOW;
    }

    bool is_errored() {
        return digitalRead(ERROR_PIN) == HIGH;
    }

    void send_event_callback(LinearEvent event);

    void send_event(LinearEvent event) {
        DODOBOT_SERIAL_WRITE_BOTH("le", "ud", CURRENT_TIME, (int)(event));
        send_event_callback(event);
    }

    void reset_encoder()
    {
        encoder_pos = 0;
        stepper_enc.write(0);
    }

    long read_encoder()
    {
        raw_encoder_pos = stepper_enc.read();
        return raw_encoder_pos;
    }

    long enc_as_step_ticks()
    {
        encoder_pos = (long)((double)read_encoder() * ENC_TO_STEP_TICKS);
        return encoder_pos;
    }

    bool has_position_error(int stepper_pos) {
        return abs(enc_as_step_ticks() - stepper_pos) > ENCODER_POSITION_ERROR;
    }

    bool has_been_pushed() {
        return abs(read_encoder() - encoder_pos) > ENCODER_POSITION_ERROR;
    }

    void reset_to_enc_position() {
        tic.haltAndSetPosition(enc_as_step_ticks());
    }

    void reset()
    {
        digitalWrite(RESET_PIN, LOW);
        delay(20);
        digitalWrite(RESET_PIN, HIGH);
    }

    void set_active(bool state) {
        if (is_active == state) {
            return;
        }
        is_active = state;
        if (state && dodobot::robot_state.motors_active) {
            TIC_SERIAL.begin(baudrate);
            TIC_SERIAL.setTimeout(serial_timeout);
            // Give the Tic some time to start up.
            delay(20);
            send_event(LinearEvent::ACTIVE_TRUE);
        }
        else {
            TIC_SERIAL.end();
            is_homed = false;
            send_event(LinearEvent::ACTIVE_FALSE);
        }
    }

    void setup_linear()
    {
        pinMode(HOMING_PIN, INPUT_PULLUP);
        pinMode(ERROR_PIN, INPUT);
        pinMode(RESET_PIN, OUTPUT);

        digitalWrite(RESET_PIN, HIGH);

        set_active(false);
        reset_encoder();
        dodobot_serial::println_info("Linear ready");
    }


    void waitForPosition(int32_t targetPosition)
    {
        tic.setTargetPosition(targetPosition);
        do
        {
            tic.resetCommandTimeout();
        } while (tic.getCurrentPosition() != targetPosition);
        stepper_pos = targetPosition;
    }

    void print_stepper_error(uint16_t errors)
    {
        if (errors & (1 << (uint8_t)TicError::IntentionallyDeenergized))
            dodobot_serial::println_error("Tic Error: IntentionallyDeenergized, %d", errors);
        if (errors & (1 << (uint8_t)TicError::MotorDriverError))
            dodobot_serial::println_error("Tic Error: MotorDriverError, %d", errors);
        if (errors & (1 << (uint8_t)TicError::LowVin))
            dodobot_serial::println_error("Tic Error: LowVin, %d", errors);
        if (errors & (1 << (uint8_t)TicError::KillSwitch))
            dodobot_serial::println_error("Tic Error: KillSwitch, %d", errors);
        if (errors & (1 << (uint8_t)TicError::RequiredInputInvalid))
            dodobot_serial::println_error("Tic Error: RequiredInputInvalid, %d", errors);
        if (errors & (1 << (uint8_t)TicError::SerialError))
            dodobot_serial::println_error("Tic Error: SerialError, %d", errors);
        if (errors & (1 << (uint8_t)TicError::CommandTimeout))
            dodobot_serial::println_error("Tic Error: CommandTimeout, %d", errors);
        if (errors & (1 << (uint8_t)TicError::SafeStartViolation))
            dodobot_serial::println_error("Tic Error: SafeStartViolation, %d", errors);
        if (errors & (1 << (uint8_t)TicError::ErrLineHigh))
            dodobot_serial::println_error("Tic Error: ErrLineHigh, %d", errors);
        if (errors & (1 << (uint8_t)TicError::SerialFraming))
            dodobot_serial::println_error("Tic Error: SerialFraming, %d", errors);
        if (errors & (1 << (uint8_t)TicError::RxOverrun))
            dodobot_serial::println_error("Tic Error: RxOverrun, %d", errors);
        if (errors & (1 << (uint8_t)TicError::Format))
            dodobot_serial::println_error("Tic Error: Format, %d", errors);
        if (errors & (1 << (uint8_t)TicError::Crc))
            dodobot_serial::println_error("Tic Error: Crc, %d", errors);
        if (errors & (1 << (uint8_t)TicError::EncoderSkip))
            dodobot_serial::println_error("Tic Error: EncoderSkip, %d", errors);
    }

    bool check_errors()
    {
        if (dodobot::robot_state.motors_active && is_errored()) {
            uint16_t errors = tic.getErrorStatus();

            if (errors == 0 || errors & (1 << (uint8_t)TicError::SafeStartViolation)) {
                tic.exitSafeStart();
            }
            else {
                dodobot_serial::println_error("Stepper is errored: %d", errors);
                print_stepper_error(errors);
                return true;
            }
        }
        return false;
    }

    void update_microstep_config()
    {
        switch (step_mode) {
        // switch (tic.getStepMode()) {
            case TicStepMode::Microstep1: microsteps = 1; break;
            case TicStepMode::Microstep2_100p:
            case TicStepMode::Microstep2: microsteps = 2; break;
            case TicStepMode::Microstep4: microsteps = 4; break;
            case TicStepMode::Microstep8: microsteps = 8; break;
            case TicStepMode::Microstep16: microsteps = 16; break;
            case TicStepMode::Microstep32: microsteps = 32; break;
            case TicStepMode::Microstep64: microsteps = 64; break;
            case TicStepMode::Microstep128: microsteps = 128; break;
            case TicStepMode::Microstep256: microsteps = 256; break;
        }
    }

    void update_conversions()
    {
        update_microstep_config();
        STEPPER_TICKS_PER_R = STEPPER_TICKS_PER_R_NO_GEARBOX * (double)microsteps * STEPPER_GEARBOX_RATIO;
        STEPPER_R_PER_TICK = 1.0 / STEPPER_TICKS_PER_R;
        ENC_TO_STEP_TICKS = STEPPER_TICKS_PER_R_NO_GEARBOX * (double)microsteps / ENCODER_TICKS_PER_R_NO_GEARBOX;
        STEP_TICKS_TO_LINEAR_MM = STEPPER_R_PER_TICK * BELT_PULLEY_RADIUS_MM * 2 * PI;

        MAX_SPEED = MAX_SPEED_FULL_STEP * microsteps;
        MAX_POSITION = MAX_POSITION_FULL_STEP * microsteps;
        HOMING_SPEED = HOMING_SPEED_FULL_STEP * microsteps;
        FAST_HOMING_SPEED = FAST_HOMING_SPEED_FULL_STEP * microsteps;
        STEPPER_ACCEL = STEPPER_ACCEL_FULL_STEP * microsteps;
        STEPPER_DECEL = STEPPER_DECEL_FULL_STEP * microsteps;
        HOMING_ACCEL = HOMING_ACCEL_FULL_STEP * microsteps;

        POSITION_BUFFER = POSITION_BUFFER_FULL_STEP * microsteps;
        ENCODER_POSITION_ERROR = ENCODER_POSITION_ERROR_FULL_STEP * microsteps;
    }

    void set_max_speed(uint32_t speed) {
        MAX_SPEED_FULL_STEP = speed / microsteps;  // speed is set with microstepping accounted for
        update_conversions();
        tic.setMaxSpeed(MAX_SPEED);
    }

    void set_accel(uint32_t accel) {
        STEPPER_ACCEL_FULL_STEP = accel / microsteps;  // acceleration is set with microstepping accounted for
        STEPPER_DECEL_FULL_STEP = accel / microsteps;  // acceleration is set with microstepping accounted for
        update_conversions();
        tic.setMaxAccel(STEPPER_ACCEL);
        tic.setMaxDecel(STEPPER_DECEL);
    }

    double to_linear_pos(int stepper_ticks) {
        return stepper_ticks * STEP_TICKS_TO_LINEAR_MM;
    }

    bool wait_for_homing_pin(bool goal_state, int32_t target_velocity)
    {
        uint32_t check_pos_error_timer = CURRENT_TIME;
        uint32_t timeout_timer = CURRENT_TIME;

        reset_encoder();
        tic.haltAndSetPosition(0);
        tic.setTargetVelocity(target_velocity);

        while (is_home_pin_active() != goal_state) {
            tic.resetCommandTimeout();
            if (CURRENT_TIME - check_pos_error_timer > 250)
            {
                if (has_position_error(tic.getCurrentPosition()))
                {
                    dodobot_serial::println_error("Homing routine failed. Position error!");
                    send_event(LinearEvent::HOMING_FAILED);
                    tic.haltAndHold();
                    return false;
                }
                check_pos_error_timer = CURRENT_TIME;
            }
            if (CURRENT_TIME - timeout_timer > 10000) {   // 10 second timeout
                dodobot_serial::println_error("Homing routine timed out");
                send_event(LinearEvent::HOMING_FAILED);
                tic.haltAndHold();
                return false;
            }
        }
        return true;
    }

    void home_stepper()
    {
        if (!is_active) {
            dodobot_serial::println_error("Can't home stepper. Active flag not set.");
            send_event(LinearEvent::NOT_ACTIVE);
            return;
        }
        if (check_errors()) {
            dodobot_serial::println_error("Can't home stepper. Stepper is errored.");
            send_event(LinearEvent::HOMING_FAILED);
            return;
        }
        dodobot_serial::println_info("Running home sequence.");
        send_event(LinearEvent::HOMING_STARTED);

        if (!is_homed) {
            // homing routine hasn't been run since setting active to true
            tic.exitSafeStart();
            reset_encoder();
        }

        tic.setStepMode(step_mode);
        update_conversions();
        dodobot_serial::println_info("Step mode set to %d", microsteps);

        tic.setMaxSpeed(FAST_HOMING_SPEED);
        tic.setMaxAccel(HOMING_ACCEL);
        dodobot_serial::println_info("Homing speed: %d, accel: %d, decel: %d", FAST_HOMING_SPEED, STEPPER_ACCEL, STEPPER_DECEL);
        if (check_errors()) {
            dodobot_serial::println_error("Can't home stepper. Stepper is errored after setting motion params.");
            return;
        }

        uint32_t speed;
        for (size_t count = 0; count < 10; count++) {
            delay(20);
            speed = tic.getMaxSpeed();
            if (speed == FAST_HOMING_SPEED) {
                break;
            }
            else {
                delay(100);
            }
        }
        if (speed != FAST_HOMING_SPEED) {
            dodobot_serial::println_error("Homing routine failed to send desired speed. %d != %d", speed, FAST_HOMING_SPEED);
            send_event(LinearEvent::HOMING_FAILED);
            return;
        }
        if (check_errors()) {
            dodobot_serial::println_error("Can't home stepper. Stepper is errored after checking motion params.");
            return;
        }

        // Drive down until the limit switch is found
        if (!wait_for_homing_pin(true, -FAST_HOMING_SPEED)) {
            return;
        }
        tic.haltAndHold();
        delay(50);

        // Move off the homing switch
        if (!wait_for_homing_pin(false, FAST_HOMING_SPEED)) {
            return;
        }
        delay(50);

        // Move down more slowly to get a more accurate reading
        if (!wait_for_homing_pin(true, -HOMING_SPEED)) {
            return;
        }
        tic.haltAndSetPosition(HOMING_POS_OFFSET_FULL_STEP * microsteps);
        delay(50);
        waitForPosition(0);
        tic.haltAndSetPosition(0);
        delay(50);
        reset_encoder();
        delay(50);
        waitForPosition(START_POS_FULL_STEP * microsteps);
        enc_as_step_ticks();  // initialize encoder variables

        tic.setMaxSpeed(MAX_SPEED);
        tic.setMaxAccel(STEPPER_ACCEL);
        tic.setMaxDecel(STEPPER_DECEL);
        dodobot_serial::println_info("Max speed: %d", MAX_SPEED);
        is_homed = true;
        target_position = 0;
        target_velocity = 0;
        dodobot_serial::println_info("Homing complete.");
        send_event(LinearEvent::HOMING_FINISHED);
    }

    void set_position(int position) {
        if (!is_active) {
            send_event(LinearEvent::NOT_ACTIVE);
            return;
        }
        if (!is_homed) {
            send_event(LinearEvent::NOT_HOMED);
            return;
        }
        if (position > MAX_POSITION) {
            position = MAX_POSITION;
        }
        if (position < 0) {
            position = 0;
        }
        target_position = position;
        tic.setTargetPosition(target_position);
        is_moving = true;
        send_event(LinearEvent::MOVE_STARTED);
        if (target_position == tic.getCurrentPosition()) {
            send_event(LinearEvent::MOVE_FINISHED);
        }
    }

    void set_velocity(int velocity) {
        if (!is_active) {
            send_event(LinearEvent::NOT_ACTIVE);
            return;
        }
        if (!is_homed) {
            send_event(LinearEvent::NOT_HOMED);
            return;
        }
        target_velocity = velocity;
        tic.setTargetVelocity(velocity);
        is_moving = true;
    }

    void stop() {
        target_velocity = 0;
        tic.haltAndHold();
    }

    bool is_position_invalid(int position) {
        return (
            (target_position >= MAX_POSITION && position >= MAX_POSITION) ||
            (target_position < 0 && position < 0)
        );
    }

    bool is_velocity_invalid(int position) {
        return (
            (target_velocity > 0 && position >= (MAX_POSITION - POSITION_BUFFER)) ||
            (target_velocity < 0 && position < POSITION_BUFFER)
        );
    }

    enum TicPlanningMode get_planning_mode() {
        if (CURRENT_TIME - planning_mode_timer > PLANNING_MODE_DELAY_MS) {
            planning_mode_timer = CURRENT_TIME;
            TicPlanningMode new_planning_mode = tic.getPlanningMode();
            if (new_planning_mode != planning_mode) {
                if (planning_mode == TicPlanningMode::TargetPosition &&
                        new_planning_mode == TicPlanningMode::Off) {
                    send_event(LinearEvent::MOVE_FINISHED);
                }
                planning_mode = new_planning_mode;
            }
        }
        return planning_mode;
    }

    void update() {
        // If the update timer hasn't exceeded the threshold, do nothing
        if (CURRENT_TIME - update_timer < UPDATE_DELAY_MS) {
            return;
        }
        update_timer = CURRENT_TIME;

        // If reporting is enabled, print various data
        if (dodobot::robot_state.is_reporting_enabled && dodobot::robot_state.motors_active) {
            dodobot_serial::data->write("linear", "udddd", CURRENT_TIME, stepper_pos, is_errored(), is_homed, is_active);
        }

        // If the stepper is not homed and isn't active, do nothing
        if (!is_homed || !is_active) {
            return;
        }

        bool error_state = is_errored();
        if (error_state != is_errored_state && error_state) {
            tic.haltAndHold();
            check_errors();
        }

        // If linear slide hasn't been pushed and the stepper isn't moving, do nothing
        // if (!has_been_pushed() && !is_moving) {
        //     return;
        // }

        // Check if stepper is moving to position, moving with velocity, or not moving
        // Check if stepper position has exceeded the boundaries
        stepper_pos = tic.getCurrentPosition();
        int32_t vel = tic.getCurrentVelocity();
        if (vel != stepper_vel) {
            stepper_vel = vel;
            if (stepper_vel == 0) {
                tic.haltAndHold();
            }
        }
        switch (get_planning_mode()) {
            case TicPlanningMode::Off: is_moving = false; break;
            // case TicPlanningMode::TargetPosition: if (is_position_invalid(stepper_pos))  { stop(); } break;
            case TicPlanningMode::TargetPosition: break;
            case TicPlanningMode::TargetVelocity: if (is_velocity_invalid(stepper_pos))  { stop(); } break;
        }
        // if (is_moving) {
        //     // dodobot_serial::println_info("Step mode: %d", (uint8_t)tic.getStepMode());
        //     // dodobot_serial::println_info("Accel: %d", (int)tic.getMaxAccel());
        //     // dodobot_serial::println_info("Decel: %d", (int)tic.getMaxDecel());
        //     // dodobot_serial::println_info("Current lim: %d", (int)tic.getCurrentLimit());
        //     // dodobot_serial::println_info("%f, %f, %f, %f", STEPPER_TICKS_PER_R, STEPPER_R_PER_TICK, ENC_TO_STEP_TICKS, STEP_TICKS_TO_LINEAR_MM);
        // }

        // If the stepper position has deviated from the encoder position,
        // stop the motor and reset position to the encoder's position
        if (has_position_error(stepper_pos)) {
            reset_to_enc_position();
            dodobot_serial::println_error("Position error!");
            send_event(LinearEvent::POSITION_ERROR);
        }

        tic.resetCommandTimeout();
    }
}

#endif  // __DODOBOT_LINEAR_H__
