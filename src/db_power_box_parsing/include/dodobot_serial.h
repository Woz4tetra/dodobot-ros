#pragma once

#include <limits.h>
#include "ros/ros.h"
#include "serial/serial.h"

using namespace std;

#define DODOBOT_SERIAL _device

const char PACKET_START_0 = '\x12';
const char PACKET_START_1 = '\x13';
const char PACKET_STOP = '\n';
const char PACKET_SEGMENT_SEPARATOR = '\t';
const unsigned int MAX_PACKET_LEN = 128;
const unsigned int MAX_SEGMENT_LEN = 64;

#define CHECK_SEGMENT(__INTERFACE_OBJ__, __GET_SEGMENT_CALL__)  if (!__GET_SEGMENT_CALL__) {  \
    ROS_INFO("Not enough segments supplied for segment #%d", __INTERFACE_OBJ__->get_segment_num());  \
    return;  }


class ReadyTimeoutException_t : public exception {
    virtual const char* what() const throw() { return "Timeout reached. Never got ready signal from serial device"; }
} ReadyTimeoutException;


typedef struct StructReadyState {
    uint32_t time_ms;
    string robot_name;
    bool is_ready;
} StructReadyState_t;


string format_char(unsigned char c);
string format_packet(string packet);
string format_packet(char *packet, uint32_t length);


class DodobotSerial
{
public:
    DodobotSerial();

    void begin(string address, int baud);
    void close();

    void write(string name, const char* formats, ...);

    bool write_large(string name, string destination, vector<char>* data);
    bool wait_for_ok(uint32_t expected_packet_num);
    bool wait_for_ok();
    
    bool read();
    
    void flush_read();
    
    int get_segment_num()  { return current_segment_num; }
    string get_category()  { return current_category; }

    bool segment_as_string(string& result);
    bool segment_as_char_array(char** result);
    bool segment_as_char_array(char** result, int length);
    bool segment_as_uint16(uint16_t& result);
    bool segment_as_uint32(uint32_t& result);
    bool segment_as_int32(int32_t& result);
    bool segment_as_float(float& result);
    bool segment_large(string& destination, int32_t& segment_index, int32_t& num_segments, uint16_t& length, int32_t prev_segment_index, char** bytes);

    bool ready() {
        return DODOBOT_SERIAL.isOpen();
    }

    void check_device_ready();
    ros::Time get_device_time_as_local(uint32_t device_time);

private:
    void init_variables();
    bool make_packet(string name, const char *formats, va_list args); 

    bool parse_packet();

    void write_txrx(uint32_t code);

    bool next_segment(int length);
    bool next_segment();

    void set_start_time(uint32_t device_time);

    void log_packet_error_code(uint32_t code, uint32_t packet_num);

    serial::Serial DODOBOT_SERIAL;
    const ros::Duration stop_timeout = ros::Duration(0.5);
    const ros::Duration large_packet_segment_delay = ros::Duration(0.05);

    char *segment;

    uint32_t read_packet_num;
    uint32_t write_packet_num;

    uint32_t buffer_index;
    uint32_t read_packet_index;

    int current_segment_num;
    string current_category;

    char *recv_char_buffer;
    size_t recv_char_index;
    size_t read_packet_len;
    
    char *write_char_buffer;
    size_t write_char_index;
    
    ros::Time start_wait_time;

    uint32_t recv_txrx_packet_num, recv_txrx_code;

    std::map<uint32_t, int> wait_for_ok_reqs;

    StructReadyState_t* ready_state;
    ros::Time start_time_local;
    uint32_t start_time_device;
};