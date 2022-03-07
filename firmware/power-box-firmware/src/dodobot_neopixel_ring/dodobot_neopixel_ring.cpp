#include <dodobot_neopixel_ring/dodobot_neopixel_ring.h>

DodobotNeopixelRing::DodobotNeopixelRing(DodobotSerial* serial, int pin, int num_pixels, int pixel_format)
{
    this->serial = serial;
    this->pin = pin;
    this->num_pixels = num_pixels;
    this->pixel_format = pixel_format;

    pixels = new Adafruit_NeoPixel(num_pixels, pin, pixel_format);
}

void DodobotNeopixelRing::begin()
{
    pixels->begin();
    pixels->clear();
}

void DodobotNeopixelRing::off()
{
    pixels->clear();
    pixels->show();
}

void DodobotNeopixelRing::packet_callback()
{
    uint32_t color;
    CHECK_SEGMENT(serial, serial->segment_as_uint32(color));
    
    pixels->clear();
    for (int i = 0; i < num_pixels; i++) {
        pixels->setPixelColor(i, color);
    }
    pixels->show();
}
