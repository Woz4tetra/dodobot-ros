#include <Arduino.h>
#include <Tic_Teensy.h>
// #include <SoftwareSerial.h>

#define COMM_SERIAL Serial

// TIC Stepper controller
#define TIC_SERIAL Serial1

// SoftwareSerial TIC_SERIAL(1, 0);
TicSerial tic(TIC_SERIAL);


// Sends a "Reset command timeout" command to the Tic.  We must
// call this at least once per second, or else a command timeout
// error will happen.  The Tic's default command timeout period
// is 1000 ms, but it can be changed or disabled in the Tic
// Control Center.
void resetCommandTimeout()
{
    tic.resetCommandTimeout();
}

// Delays for the specified number of milliseconds while
// resetting the Tic's command timeout so that its movement does
// not get interrupted.
void delayWhileResettingCommandTimeout(uint32_t ms)
{
    uint32_t start = millis();
    do
    {
        resetCommandTimeout();
    } while ((uint32_t)(millis() - start) <= ms);
}

// Polls the Tic, waiting for it to reach the specified target
// position.  Note that if the Tic detects an error, the Tic will
// probably go into safe-start mode and never reach its target
// position, so this function will loop infinitely.  If that
// happens, you will need to reset your Arduino.
void waitForPosition(int32_t targetPosition)
{
    tic.setTargetPosition(targetPosition);
    do
    {
        resetCommandTimeout();
    } while (tic.getCurrentPosition() != targetPosition);
}


void setup()
{
    COMM_SERIAL.begin(9600);
    TIC_SERIAL.begin(115385);
    // Give the Tic some time to start up.
    delay(20);
    tic.exitSafeStart();
    // tic.setMaxSpeed(420000000);
    tic.setMaxSpeed(380000000);
}

void loop()
{
    if (COMM_SERIAL.available()) {
        String command = COMM_SERIAL.readStringUntil('\n');
        char c = command.charAt(0);
        if (c == 'u') {
            // waitForPosition(tic.getCurrentPosition() + 10000);
            tic.setTargetVelocity(420000000);
            delayWhileResettingCommandTimeout(500);
            tic.setTargetVelocity(0);
            Serial.println(tic.getCurrentPosition());
        }
        else if (c == 'd') {
            // waitForPosition(tic.getCurrentPosition() - 10000);
            tic.setTargetVelocity(-420000000);
            delayWhileResettingCommandTimeout(500);
            tic.setTargetVelocity(0);
            Serial.println(tic.getCurrentPosition());
        }
    }
}
