#include <Arduino.h>
#include <TB6612.h>

#define COMM_SERIAL Serial

#define MOTOR_STBY 26
#define MOTORA_DR1 27
#define MOTORA_DR2 28
#define MOTORB_DR1 38
#define MOTORB_DR2 37
#define MOTORA_PWM 29
#define MOTORB_PWM 30

#define MOTOR_COMMAND_TIMEOUT_MS 1000

#define CURRENT_TIME millis()

uint32_t prev_commandA_time = 0;
uint32_t prev_commandB_time = 0;

bool are_motors_active = false;

TB6612 motorA(MOTORA_PWM, MOTORA_DR1, MOTORA_DR2);
TB6612 motorB(MOTORB_PWM, MOTORB_DR2, MOTORB_DR1);

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

void setup_motors()
{
    pinMode(MOTOR_STBY, OUTPUT);
    motorA.begin();
    motorB.begin();
    set_motors_active(true);
}

void reset_motor_timeouts()
{
    prev_commandA_time = CURRENT_TIME;
    prev_commandB_time = CURRENT_TIME;
}

void set_motorA(int speed) {
    reset_motor_timeouts();
    motorA.setSpeed(speed);
}

void set_motorB(int speed) {
    reset_motor_timeouts();
    motorB.setSpeed(speed);
}


bool check_motor_timeout()
{
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
    motorA.setSpeed(0);
    motorB.setSpeed(0);
}


void setup()
{
    COMM_SERIAL.begin(9600);
    setup_motors();
}

void loop()
{
    if (COMM_SERIAL.available()) {
        String command = COMM_SERIAL.readStringUntil('\n');

        char c = command.charAt(0);

        if (c == 'f') {
            set_motorA(255);
            set_motorB(255);
        }

        else if (c == 'b') {
            set_motorA(-255);
            set_motorB(-255);
        }

        else if (c == 's') {
            stop_motors();
        }
    }
    check_motor_timeout();
}
