#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <dodobot_serial/dodobot_serial.h>

class DodobotNeopixelRing
{
public:
    DodobotNeopixelRing(DodobotSerial* serial, int pin, int num_pixels, int pixel_format);
    void begin();
    void packet_callback();
    void off();

private:
    const uint32_t UPDATE_DELAY_MS = 10;
    Adafruit_NeoPixel *pixels;
    DodobotSerial* serial;

    int pin;
    int num_pixels;
    int pixel_format;

    uint32_t update_timer = 0;
};
