#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <dodobot_serial/dodobot_serial.h>
#include <dodobot_power_monitor/dodobot_power_monitor.h>
#include <dodobot_neopixel_ring/dodobot_neopixel_ring.h>


#define DEVICE_KEYWORD "dodobot_power_box"

void packet_callback(DodobotSerial* interface, String category);
DodobotSerial* serial_inferface = new DodobotSerial(packet_callback);

DodobotPowerMonitor* power_monitor = new DodobotPowerMonitor(serial_inferface);
DodobotNeopixelRing* neopixel_ring = new DodobotNeopixelRing(serial_inferface, 9, 24, NEO_GRBW + NEO_KHZ800);

void setup()
{
    DODOBOT_SERIAL.begin(38400);
    
    power_monitor->begin();
    neopixel_ring->begin();
}


void loop()
{
    power_monitor->update();
    serial_inferface->read();
}

unsigned int total_checksum = 0;
int32_t prev_segment_index = 0;

void packet_callback(DodobotSerial* interface, String category)
{
    if (category.equals("?"))  // ready command
    {
        String keyword;
        CHECK_SEGMENT(interface, interface->segment_as_string(keyword));
        if (keyword.equals("dodobot"))
        {
            DODOBOT_SERIAL.println("Received ready signal!");
            interface->write("ready", "us", millis(), DEVICE_KEYWORD);
        }
    }
    else if (category.equals("!"))  // stop command
    {
        String keyword;
        CHECK_SEGMENT(interface, interface->segment_as_string(keyword));
        if (keyword.equals("dodobot"))
        {
            neopixel_ring->off();
        }
    }
    else if (category.equals("pix"))
    {
        neopixel_ring->packet_callback();
    }
    else if (category.equals("large-test"))
    {
        String destination;
        int32_t segment_index;
        int32_t num_segments;
        uint16_t length;
        char* data;
        CHECK_SEGMENT(interface, interface->segment_large(destination, segment_index, num_segments, length, prev_segment_index, &data));
        prev_segment_index = segment_index;
        DODOBOT_SERIAL.print("Received large segment ");
        DODOBOT_SERIAL.print(segment_index + 1);
        DODOBOT_SERIAL.print(" of ");
        DODOBOT_SERIAL.println(num_segments);
        DODOBOT_SERIAL.print("Length: ");
        DODOBOT_SERIAL.println(length);

        if (segment_index == 0) {
            total_checksum = 0;
            prev_segment_index = 0;
        }

        if (segment_index == num_segments - 1) {
            for (uint16_t index = 0; index < length - 2; index++) {
                total_checksum += (unsigned int)data[index];
            }
            char recv_checksum_array[2];
            memcpy(recv_checksum_array, data + length - 2, 2);
            uint8_t recv_checksum = strtol(recv_checksum_array, NULL, 16);

             total_checksum &= 0xff;
            
            if (recv_checksum != total_checksum) {
                DODOBOT_SERIAL.print("Large checksum doesn't match: ");
                DODOBOT_SERIAL.print(total_checksum);
                DODOBOT_SERIAL.print(" != ");
                DODOBOT_SERIAL.println(recv_checksum);
            }
            else{
                DODOBOT_SERIAL.println("Large packet received successfully!");
            }
        }
        else {
            for (uint16_t index = 0; index < length; index++) {
                total_checksum += (unsigned int)data[index];
            }
        }
    }
}
