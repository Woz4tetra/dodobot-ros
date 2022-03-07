#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>


#define COMM_SERIAL Serial


Adafruit_INA219 ina219;
float ina219_shuntvoltage = 0.0;
float ina219_busvoltage = 0.0;
float ina219_current_mA = 0.0;
float ina219_loadvoltage = 0.0;
float ina219_power_mW = 0.0;

int pin         =  9;
int numPixels   = 24;
int pixelFormat = NEO_GRBW + NEO_KHZ800;

Adafruit_NeoPixel *pixels;

void setup()
{
    COMM_SERIAL.begin(9600);

    pixels = new Adafruit_NeoPixel(numPixels, pin, pixelFormat);

    ina219.begin();
    pixels->begin();
}

int channel_indexer = 0;


void loop()
{
    pixels->clear();

    uint32_t color;
    switch (channel_indexer)
    {
    case 0: color = pixels->Color(255, 0, 0, 100); break;
    case 1: color = pixels->Color(0, 255, 0, 100); break;
    case 2: color = pixels->Color(0, 0, 255, 100); break;
    case 3: color = pixels->Color(255, 255, 255); break;
    case 4: color = pixels->Color(0, 0, 0, 255); break;
    case 5: color = pixels->Color(255, 255, 255, 255); break;
    
    default:  color = pixels->Color(150, 150, 0); break;
        break;
    }
    channel_indexer++;
    if (channel_indexer > 5) {
        channel_indexer = 0;
    }
    for (int i = 0; i < numPixels; i++)
    {
        
        pixels->setPixelColor(i, color);
        pixels->show();
        
        delay(25);
    }

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
