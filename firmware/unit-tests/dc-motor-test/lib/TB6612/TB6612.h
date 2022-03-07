#include <Arduino.h>
class TB6612
{
    public:
        TB6612(int pwm_pin, int dir_pin_1, int dir_pin_2);

        void begin();
        void reset();

        void setSpeed(int speed);
        int getSpeed();

    private:
        int TB6612_PWM_MOTOR_PIN;
        int TB6612_MOTOR_DIRECTION_PIN_1;
        int TB6612_MOTOR_DIRECTION_PIN_2;

        int currentMotorCommand;
};
