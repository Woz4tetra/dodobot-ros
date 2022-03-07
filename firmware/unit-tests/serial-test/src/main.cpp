#include <Arduino.h>

#define COMM_SERIAL Serial
#define DATA_SERIAL Serial5

#define CURRENT_TIME millis()

void setup()
{
    COMM_SERIAL.begin(9600);
    DATA_SERIAL.begin(9600);

    while (!COMM_SERIAL.dtr()) {
        delay(1);
    }
}

void loop()
{
    DATA_SERIAL.println(CURRENT_TIME);
    COMM_SERIAL.println(CURRENT_TIME);
    delay(1000);
    // if (COMM_SERIAL.available()) {
    //     DATA_SERIAL.print(CURRENT_TIME);
    //     DATA_SERIAL.print('\t');
    //     DATA_SERIAL.println(COMM_SERIAL.readStringUntil('\n'));
    // }
    // if (DATA_SERIAL.available()) {
    //     COMM_SERIAL.print(CURRENT_TIME);
    //     COMM_SERIAL.print('\t');
    //     COMM_SERIAL.println(DATA_SERIAL.readStringUntil('\n'));
    // }
}
