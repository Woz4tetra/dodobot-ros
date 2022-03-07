#include <Arduino.h>

#define COMM_SERIAL Serial

#define BUMP1_PIN 3
#define BUMP2_PIN 5

#define CURRENT_TIME millis()

void setup_bumpers()
{
    pinMode(BUMP1_PIN, INPUT_PULLUP);
    pinMode(BUMP2_PIN, INPUT_PULLUP);
}

bool is_bump1_active() {
    return digitalRead(BUMP1_PIN) == LOW;
}

bool is_bump2_active() {
    return digitalRead(BUMP2_PIN) == LOW;
}

void setup()
{
    COMM_SERIAL.begin(9600);
    setup_bumpers();
}

void loop()
{
    COMM_SERIAL.print("1: ");
    COMM_SERIAL.print(is_bump1_active());
    COMM_SERIAL.print("\t2: ");
    COMM_SERIAL.println(is_bump2_active());
    delay(50);
}
