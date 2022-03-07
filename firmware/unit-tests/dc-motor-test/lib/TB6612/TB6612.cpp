#include <TB6612.h>

TB6612::TB6612(int pwm_pin, int dir_pin_1, int dir_pin_2)
{
    TB6612_PWM_MOTOR_PIN = pwm_pin;
    TB6612_MOTOR_DIRECTION_PIN_1 = dir_pin_1;
    TB6612_MOTOR_DIRECTION_PIN_2 = dir_pin_2;

    currentMotorCommand = 0;
}


void TB6612::begin()
{
    pinMode(TB6612_PWM_MOTOR_PIN, OUTPUT);
    pinMode(TB6612_MOTOR_DIRECTION_PIN_1, OUTPUT);
    pinMode(TB6612_MOTOR_DIRECTION_PIN_2, OUTPUT);
}

void TB6612::reset() {
    setSpeed(0);
}

void TB6612::setSpeed(int speed)
{
    currentMotorCommand = speed;
    if (speed > 0) {
        digitalWrite(TB6612_MOTOR_DIRECTION_PIN_1, LOW);
        digitalWrite(TB6612_MOTOR_DIRECTION_PIN_2, HIGH);
    }
    else if (speed < 0) {
        digitalWrite(TB6612_MOTOR_DIRECTION_PIN_1, HIGH);
        digitalWrite(TB6612_MOTOR_DIRECTION_PIN_2, LOW);
    }

    speed = abs(speed);
    if (speed > 255) {
        speed = 255;
    }
    analogWrite(TB6612_PWM_MOTOR_PIN, speed);
}

int TB6612::getSpeed() {
    return currentMotorCommand;
}
