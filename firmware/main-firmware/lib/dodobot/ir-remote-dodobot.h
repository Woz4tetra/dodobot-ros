#ifndef __DODOBOT_IR_REMOTE_H__
#define __DODOBOT_IR_REMOTE_H__

#include <Arduino.h>
#include <IRremote.h>

#include "dodobot.h"

#define IR_RECEIVER_PIN 2

namespace dodobot_ir_remote
{
    IRrecv irrecv(IR_RECEIVER_PIN);
    decode_results irresults;
    uint8_t ir_type = 0;
    uint16_t ir_value = 0;
    uint16_t prev_ir_value = 0;

    uint32_t ir_check_timer = 0;
    const uint32_t IR_CHECK_DELAY_MS = 33;

    void setup_IR()
    {
        irrecv.enableIRIn();
        irrecv.blink13(false);
        dodobot_serial::println_info("IR ready");
    }

    void callback_ir(uint8_t remote_type, uint16_t value);

    bool read_IR()
    {
        if (CURRENT_TIME - ir_check_timer < IR_CHECK_DELAY_MS) {
            return false;
        }
        ir_check_timer = CURRENT_TIME;

        if (irrecv.decode(&irresults)) {
            ir_type = irresults.decode_type;
            prev_ir_value = ir_value;
            ir_value = irresults.value;
            irrecv.resume(); // Receive the next value
            // dodobot_serial::println_info("IR: %d", ir_value);

            // if (ir_value == 0xffff) {  // 0xffff means repeat last command
            //     ir_value = prev_ir_value;
            // }

            callback_ir(ir_type, ir_value);

            return true;
        }
        else {
            return false;
        }
    }
    void report_IR()
    {
        if (!dodobot::robot_state.is_reporting_enabled) {
            return;
        }
        DODOBOT_SERIAL_WRITE_BOTH("ir", "udd", CURRENT_TIME, ir_type, ir_value);
    }
};  // namespace dodobot_ir_remote


#endif  // __DODOBOT_IR_REMOTE_H__
