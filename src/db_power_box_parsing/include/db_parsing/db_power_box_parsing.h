#ifndef _DODOBOT_POWER_BOX_PARSING_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <queue>
#include <boost/thread/thread.hpp>
#include <iterator>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/BatteryState.h"
#include "serial/serial.h"

using namespace std;

#define CHECK_SEGMENT(PARAM)  if (!getNextSegment(PARAM)) {  ROS_ERROR_STREAM("Not enough segments supplied for #" << getSegmentNum() + 1 << ". Buffer: " << formatPacketToPrint(_recvCharBuffer, _readPacketLen));  return;  }

char PACKET_START_0 = '\x12';
char PACKET_START_1 = '\x13';
char PACKET_STOP = '\n';

typedef union uint16_union
{
    uint16_t integer;
    unsigned char byte[2];
} uint16_union;

typedef union uint32_union
{
    uint32_t integer;
    unsigned char byte[4];
} uint32_union;

typedef union int32_union
{
    int32_t integer;
    unsigned char byte[4];
} int32_union;

typedef union float_union
{
    float floating_point;
    unsigned char byte[8];
} float_union;


#define SERIAL_BUFFER_SIZE 0x4000

class ReadyTimeoutExceptionClass : public exception {
    virtual const char* what() const throw() { return "Timeout reached. Never got ready signal from serial device"; }
} ReadyTimeoutException;

class DodobotPowerBoxParsing {
private:
    ros::NodeHandle nh;  // ROS node handle

    serial::Serial _serialRef;
    string _serialPort;
    int _serialBaud;

    int _serialBufferIndex;
    char* _currentBufferSegment;
    int _currentSegmentNum;

    uint32_t _readPacketNum;
    uint32_t _writePacketNum;

    ros::Time deviceStartTime;
    uint32_t offsetTimeMs;

    size_t _readPacketLen;
    size_t _recvCharIndex;
    char* _recvCharBuffer;

    size_t _writeCharIndex;
    char* _writeCharBuffer;

    size_t large_packet_len;

    std::map<uint32_t, int> wait_for_ok_reqs;
    ros::Duration packet_ok_timeout;

    bool was_reporting;

    ros::Publisher battery_pub;
    sensor_msgs::BatteryState battery_msg;
    void parseBattery();

    void configure();
    void checkReady();
    void setStartTime(uint32_t time_ms);
    ros::Time getDeviceTime(uint32_t time_ms);
    bool getNextSegment(int length);
    bool getNextSegment();
    int getSegmentNum();
    bool waitForPacketStart();
    void processSerialPacket(string category);

    uint16_t segment_as_uint16();
    uint32_t segment_as_uint32();
    int32_t segment_as_int32();
    float segment_as_float();
    string segment_as_string();

    bool readline();
    bool readSerial();
    void writeSerialLarge(string name, vector<unsigned char>* data);
    void writeSerial(string name, const char *formats, ...);
    bool waitForOK(uint32_t packet_num, ros::Duration ok_timeout = ros::Duration(0.0));  // 0.0 -> use default (packet_ok_timeout)
    bool waitForOK(ros::Duration ok_timeout = ros::Duration(0.0));

    bool write_stop_flag;
    int write_thread_rate;
    queue<string> write_queue;
    boost::thread* write_thread;
    void write_packet_from_queue();
    void write_thread_task();

    ros::Time write_timer;

    void setup();
    void loop();
    void stop();

    void logPacketErrorCode(int error_code, uint32_t packet_num);
    void logPacketErrorCode(int error_code, uint32_t packet_num, string message);

    string formatPacketToPrint(char* packet, uint32_t length);
    string formatPacketToPrint(string packet);
    string getPrintChar(unsigned char c);
public:
    DodobotPowerBoxParsing(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _DODOBOT_POWER_BOX_PARSING_H_
