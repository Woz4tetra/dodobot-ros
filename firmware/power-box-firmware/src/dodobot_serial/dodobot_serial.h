#pragma once

#include <Arduino.h>

#define DODOBOT_SERIAL Serial

#define PACKET_START_0 '\x12'
#define PACKET_START_1 '\x13'
#define PACKET_STOP '\n'
#define PACKET_SEGMENT_SEPARATOR '\t'
#define MAX_PACKET_LEN  128
#define MAX_SEGMENT_LEN  64

#define CHECK_SEGMENT(__INTERFACE_OBJ__, __GET_SEGMENT_CALL__)  if (!__GET_SEGMENT_CALL__) {  \
    DODOBOT_SERIAL.print("Not enough segments supplied for segment #");  \
    DODOBOT_SERIAL.println(__INTERFACE_OBJ__->get_segment_num());  \
    return;  }

class DodobotSerial
{
public:
    DodobotSerial(void (*read_callback)(DodobotSerial*, String));

    void write(String name, const char* formats, ...);

    bool write_large(String name, String destination, uint32_t segment_num, uint32_t num_segments, char* buffer);
    bool wait_for_ok();
    
    void read();
    
    void flush_read();
    
    int get_segment_num();

    bool segment_as_string(String& result);
    bool segment_as_char_array(char** result);
    bool segment_as_char_array(char** result, int length);
    bool segment_as_uint16(uint16_t& result);
    bool segment_as_uint32(uint32_t& result);
    bool segment_as_int32(int32_t& result);
    bool segment_as_float(float& result);
    bool segment_large(String& destination, int32_t& segment_index, int32_t& num_segments, uint16_t& length, int32_t prev_segment_index, char** bytes);

    bool ready() {
        return (bool)(DODOBOT_SERIAL);
    }
private:
    void init_variables();
    bool make_packet(String name, const char *formats, va_list args); 

    void parse_packet();

    void write_txrx(uint32_t code);

    bool next_segment(int length);
    bool next_segment();

    void (*read_callback)(DodobotSerial*, String);

    char *segment;

    uint32_t read_packet_num;
    uint32_t write_packet_num;

    uint32_t buffer_index;
    uint32_t read_packet_index;

    int current_segment_num;

    char *recv_char_buffer;
    size_t recv_char_index;
    size_t read_packet_len;
    
    char *write_char_buffer;
    size_t write_char_index;
    
    uint32_t start_wait_time;

    uint32_t recv_txrx_packet_num, recv_txrx_code;

    const uint32_t PACKET_STOP_TIMEOUT = 500;
};