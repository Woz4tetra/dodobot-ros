#include <Arduino.h>
#include <i2c_t3.h>
#include <Adafruit_INA219_Teensy.h>

#define COMM_SERIAL Serial

#define I2C_BUS_1 Wire


Adafruit_INA219 ina219;
float ina219_shuntvoltage = 0.0;
float ina219_busvoltage = 0.0;
float ina219_current_mA = 0.0;
float ina219_loadvoltage = 0.0;
float ina219_power_mW = 0.0;

void setup()
{
    COMM_SERIAL.begin(9600);

    I2C_BUS_1.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    I2C_BUS_1.setDefaultTimeout(200000); // 200ms

    ina219.begin(&I2C_BUS_1);
}


void loop()
{
    ina219_shuntvoltage = ina219.getShuntVoltage_mV();
    ina219_busvoltage = ina219.getBusVoltage_V();
    ina219_current_mA = ina219.getCurrent_mA();
    ina219_power_mW = ina219.getPower_mW();
    ina219_loadvoltage = ina219_busvoltage + (ina219_shuntvoltage / 1000);

    Serial.print("Shunt Voltage: "); Serial.print(ina219_shuntvoltage); Serial.println(" mV");
    Serial.print("Bus Voltage:   "); Serial.print(ina219_busvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(ina219_current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(ina219_power_mW); Serial.println(" mW");
    Serial.print("Load Voltage:  "); Serial.print(ina219_loadvoltage); Serial.println(" V");
    Serial.print("\n\n");
    delay(500);
}
