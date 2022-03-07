#include <Arduino.h>
#include <Tic_Teensy.h>
#include <Encoder.h>

#define COMM_SERIAL Serial

uint32_t prev_report_time = 0;
const uint32_t report_interval_ms = 250;
#define CURRENT_TIME millis()

// TIC Stepper controller
#define BAUD_RATE 115385
#define TIC_SERIAL Serial1
TicSerial tic(TIC_SERIAL);

// Homing switch
#define HOMING_PIN 39
#define MOTOR_STBY 26

const int MAX_POSITION = 85000;

TicStepMode step_mode = TicStepMode::Microstep8;

// Encoder
#define STEPPER_ENCA 31
#define STEPPER_ENCB 32
long encoder_pos = 0;

Encoder stepper_enc(STEPPER_ENCB, STEPPER_ENCA);

bool is_homed = false;
const int max_position = 85000;
// const uint32_t max_speed = 420000000;
const int max_speed = 200000000;

bool are_motors_active = false;


void reset_encoder()
{
    encoder_pos = 0;
    stepper_enc.write(0);
}


void set_motors_active(bool active)
{
    if (are_motors_active == active) {
        return;
    }
    are_motors_active = active;
    if (active) {  // bring motors out of standby mode
        digitalWrite(MOTOR_STBY, HIGH);
    }
    else {  // set motors to low power
        digitalWrite(MOTOR_STBY, LOW);
    }
}

bool is_home_pin_active() {
    return digitalRead(HOMING_PIN) == LOW;
}

void setup_stepper()
{
    TIC_SERIAL.begin(BAUD_RATE);
    // Give the Tic some time to start up.
    delay(20);
    tic.exitSafeStart();
    tic.setMaxSpeed(max_speed);
    tic.setStepMode(step_mode);

    reset_encoder();

    pinMode(HOMING_PIN, INPUT_PULLUP);
    // pinMode(MOTOR_STBY, OUTPUT);
    // set_motors_active(true);
}

void waitForPosition(int32_t targetPosition)
{
    if (targetPosition > MAX_POSITION) {
        targetPosition = MAX_POSITION;
    }
    if (targetPosition < 0) {
        targetPosition = 0;
    }

    tic.setTargetPosition(targetPosition);
    do
    {
        tic.resetCommandTimeout();
    } while (tic.getCurrentPosition() != targetPosition);
}

void delayWhileResettingCommandTimeout(uint32_t ms)
{
    uint32_t start = millis();
    do
    {
        tic.resetCommandTimeout();
    } while ((uint32_t)(millis() - start) <= ms);
}

void home_stepper()
{
    // Drive down until the limit switch is found
    tic.setTargetVelocity(-200000000);
    while (!is_home_pin_active()) {
        tic.resetCommandTimeout();
    }
    tic.haltAndHold();
    delay(50);

    // Move off the homing switch
    tic.setTargetVelocity(200000000);
    while (is_home_pin_active()) {
        tic.resetCommandTimeout();
    }

    // Move down more slowly to get a more accurate reading
    tic.setTargetVelocity(-50000000);
    while (!is_home_pin_active()) {
        tic.resetCommandTimeout();
    }

    tic.haltAndSetPosition(0);
    delay(50);
    reset_encoder();
    delay(50);
    waitForPosition(10000);

    tic.setMaxSpeed(max_speed);
    is_homed = true;
}

void setup()
{
    COMM_SERIAL.begin(9600);
    setup_stepper();
}

void loop()
{
    if (COMM_SERIAL.available()) {
        String command = COMM_SERIAL.readStringUntil('\n');
        char c = command.charAt(0);

        if (c == 'h') {
            home_stepper();
        }
        else if (c == 'p') {
            if (is_homed) {
                int goal_position = command.substring(1).toInt();
                if (goal_position > max_position) {
                    goal_position = max_position;
                }
                if (goal_position < 0) {
                    goal_position = 0;
                }
                COMM_SERIAL.print("Goal:\t");
                COMM_SERIAL.print(goal_position);
                waitForPosition(goal_position);
            }
            else {
                COMM_SERIAL.println("Stepper isn't homed!");
            }
        }
        else if (c == 'd') {
            // tic.setTargetVelocity(-max_speed);
            // delayWhileResettingCommandTimeout(500);
            // tic.haltAndHold();
            waitForPosition(tic.getCurrentPosition() - 5000);
        }
        else if (c == 'u') {
            // tic.setTargetVelocity(max_speed);
            // delayWhileResettingCommandTimeout(500);
            // tic.haltAndHold();
            waitForPosition(tic.getCurrentPosition() + 5000);
        }
        else {
            tic.haltAndHold();
        }
    }
    if (CURRENT_TIME - prev_report_time > report_interval_ms) {
        encoder_pos = stepper_enc.read();

        COMM_SERIAL.print("homing pin:\t");
        COMM_SERIAL.println(is_home_pin_active());
        COMM_SERIAL.print("encoder:\t");
        COMM_SERIAL.println(encoder_pos);
        COMM_SERIAL.print("position:\t");
        COMM_SERIAL.println(tic.getCurrentPosition());

        prev_report_time = CURRENT_TIME;
    }
}
