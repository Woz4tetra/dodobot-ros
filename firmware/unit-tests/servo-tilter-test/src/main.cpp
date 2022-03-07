#include <Arduino.h>
#include <Servo.h>

#define COMM_SERIAL Serial

Servo tilter_servo;
#define TILTER_PIN 16

#define TILTER_DOWN 10
#define TILTER_UP 170
int tilter_pos = 0;


void setup()
{
    COMM_SERIAL.begin(9600);
    tilter_servo.attach(TILTER_PIN);
    tilter_servo.write(TILTER_UP);
}


void loop()
{
    if (COMM_SERIAL.available()) {
        String command = COMM_SERIAL.readStringUntil('\n');

        // if (command.length() == 0) {
        //     return;
        // }
        char c = command.charAt(0);
        if (c == 'a') {
            if (tilter_pos == TILTER_UP) {
                tilter_pos = TILTER_DOWN;
            }
            else {
                tilter_pos = TILTER_UP;
            }
            tilter_servo.write(tilter_pos);
        }
        else if (c == 'b') {
            tilter_pos = command.substring(1).toInt();
            tilter_servo.write(tilter_pos);
        }
        Serial.println(tilter_pos);
    }
}
