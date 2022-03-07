#include <Arduino.h>
#include <IRremote.h>

#define COMM_SERIAL Serial
#define IR_RECEIVER_PIN 2

IRrecv irrecv(IR_RECEIVER_PIN);

decode_results irresults;
bool ir_result_available = false;
uint8_t ir_type = 0;
uint16_t ir_value = 0;
uint16_t prev_ir_value = 0;

void setup_IR()
{
    irrecv.enableIRIn();
    irrecv.blink13(false);
}

bool read_IR()
{
    if (irrecv.decode(&irresults)) {
        ir_result_available = true;
        ir_type = irresults.decode_type;
        prev_ir_value = ir_value;
        ir_value = irresults.value;
        irrecv.resume(); // Receive the next value
        return true;
    }
    else {
        return false;
    }
}

void setup()
{
    COMM_SERIAL.begin(9600);
    setup_IR();
}

void loop()
{
    if (read_IR()) {
        if (!ir_result_available) {
            return;
        }

        if (ir_value == 0xffff) {  // 0xffff means repeat last command
            ir_value = prev_ir_value;
        }

        switch (ir_value) {
            case 0x00ff: COMM_SERIAL.println("IR: VOL-"); break;  // VOL-
            case 0x807f: COMM_SERIAL.println("IR: Play/Pause"); break;  // Play/Pause
            case 0x40bf: COMM_SERIAL.println("IR: VOL+"); break;  // VOL+
            case 0x20df: COMM_SERIAL.println("IR: SETUP"); break;  // SETUP
            case 0xa05f: COMM_SERIAL.println("IR: ^"); break;  // ^
            case 0x609f: COMM_SERIAL.println("IR: MODE"); break;  // MODE
            case 0x10ef: COMM_SERIAL.println("IR: <"); break;  // <
            case 0x906f: COMM_SERIAL.println("IR: ENTER"); break;  // ENTER
            case 0x50af: COMM_SERIAL.println("IR: >"); break;  // >
            case 0x30cf: COMM_SERIAL.println("IR: 0 10+"); break;  // 0 10+
            case 0xb04f: COMM_SERIAL.println("IR: v"); break;  // v
            case 0x708f: COMM_SERIAL.println("IR: Del"); break;  // Del
            case 0x08f7: COMM_SERIAL.println("IR: 1"); break;  // 1
            case 0x8877: COMM_SERIAL.println("IR: 2"); break;  // 2
            case 0x48B7: COMM_SERIAL.println("IR: 3"); break;  // 3
            case 0x28D7: COMM_SERIAL.println("IR: 4"); break;  // 4
            case 0xA857: COMM_SERIAL.println("IR: 5"); break;  // 5
            case 0x6897: COMM_SERIAL.println("IR: 6"); break;  // 6
            case 0x18E7: COMM_SERIAL.println("IR: 7"); break;  // 7
            case 0x9867: COMM_SERIAL.println("IR: 8"); break;  // 8
            case 0x58A7: COMM_SERIAL.println("IR: 9"); break;  // 9

        }
        // String decode_type;
        // if (irresults->decode_type == NEC) {
        //     decode_type = "NEC";
        // } else if (irresults->decode_type == SONY) {
        //     decode_type = "SONY";
        // } else if (irresults->decode_type == RC5) {
        //     decode_type = "RC5";
        // } else if (irresults->decode_type == RC6) {
        //     decode_type = "RC6";
        // } else if (irresults->decode_type == UNKNOWN) {
        //     decode_type = "???";
        // }

        ir_result_available = false;
        ir_type = 0;
        ir_value = 0;
    }

}
