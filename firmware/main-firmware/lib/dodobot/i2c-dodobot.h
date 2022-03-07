
#ifndef __DODOBOT_I2C_H__
#define __DODOBOT_I2C_H__

#include <Arduino.h>
#include <i2c_t3.h>

/*
 * I2C
 */

#define I2C_BUS_1 Wire
// #define I2C_BUS_2 Wire1

namespace dodobot_i2c
{
    void setup_i2c()
    {
        I2C_BUS_1.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
        I2C_BUS_1.setDefaultTimeout(200000); // 200ms
        // I2C_BUS_2.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000);
        // I2C_BUS_2.setDefaultTimeout(200000); // 200ms
        dodobot_serial::println_info("I2C ready");
    }
};
#endif // __DODOBOT_I2C_H__
