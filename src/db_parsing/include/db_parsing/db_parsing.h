#ifndef _DODOBOT_PARSING_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <queue>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "serial/serial.h"

#include "db_parsing/DodobotDrive.h"
#include "db_parsing/DodobotBumper.h"
#include "db_parsing/DodobotGripper.h"
#include "db_parsing/DodobotFSRs.h"
#include "db_parsing/DodobotLinear.h"
#include "db_parsing/DodobotLinearEvent.h"
#include "db_parsing/DodobotTilter.h"

#include "db_parsing/DodobotPidSrv.h"


using namespace std;

#define CHECK_SEGMENT  if (!getNextSegment()) {  ROS_ERROR_STREAM("Not enough segments supplied for #" << getSegmentNum() + 1 << ". Buffer: " << _serialBuffer);  return;  }

char PACKET_START_0 = '\x12';
char PACKET_START_1 = '\x34';
char PACKET_STOP = '\n';

struct StructReadyState {
    uint32_t time_ms;
    string robot_name;
    bool is_ready;
};

struct StructRobotState {
    uint32_t time_ms;
    bool battery_ok;
    bool motors_active;
    double loop_rate;
};

struct PidKs {
    float kp_A, ki_A, kd_A, kp_B, ki_B, kd_B, speed_kA, speed_kB;
};

class ReadyTimeoutExceptionClass : public exception {
    virtual const char* what() const throw() { return "Timeout reached. Never got ready signal from serial device"; }
} ReadyTimeoutException;

class DodobotParsing {
private:
    ros::NodeHandle nh;  // ROS node handle

    string _serialPort;
    int _serialBaud;
    string _serialBuffer;
    int _serialBufferIndex;
    string _currentBufferSegment;
    int _currentSegmentNum;
    serial::Serial _serialRef;
    unsigned long long _readPacketNum;
    unsigned long long _writePacketNum;
    ros::Time deviceStartTime;
    uint32_t offsetTimeMs;
    size_t _recvCharIndex;
    char* _recvCharBuffer;

    ros::Publisher gripper_pub;
    ros::Subscriber gripper_sub;
    db_parsing::DodobotGripper gripper_msg;
    int gripper_position;
    void parseGripper();
    void gripperCallback(const db_parsing::DodobotGripper::ConstPtr& msg);
    void writeGripper(int command, int force_threshold);

    ros::Publisher fsr_pub;
    db_parsing::DodobotFSRs fsr_msg;
    void parseFSR();

    ros::Publisher tilter_pub;
    ros::Subscriber tilter_sub;
    db_parsing::DodobotTilter tilter_msg;
    void parseTilter();
    void tilterCallback(const db_parsing::DodobotTilter::ConstPtr& msg);
    void writeTilter(uint8_t command, int position);

    ros::Publisher linear_pub;
    ros::Subscriber linear_sub;
    db_parsing::DodobotLinear linear_msg;
    void parseLinear();
    void linearCallback(const db_parsing::DodobotLinear::ConstPtr& msg);

    ros::Publisher linear_event_pub;
    db_parsing::DodobotLinearEvent linear_event_msg;
    void parseLinearEvent();

    ros::Publisher battery_pub;
    sensor_msgs::BatteryState battery_msg;
    void parseBattery();

    string _driveTopicName;
    ros::Publisher drive_pub;
    ros::Subscriber drive_sub;
    db_parsing::DodobotDrive drive_msg;
    float prev_left_setpoint, prev_right_setpoint = 0.0;
    void parseDrive();
    void driveCallback(const db_parsing::DodobotDrive::ConstPtr& msg);
    void writeDriveChassis(float speedA, float speedB);

    ros::Publisher bumper_pub;
    db_parsing::DodobotBumper bumper_msg;
    void parseBumper();

    bool motorsReady();
    bool robotReady();

    StructRobotState* robotState;
    StructReadyState* readyState;

    ros::ServiceServer pid_service;
    PidKs* pidConstants;
    ros::Timer pid_resend_timer;
    void resendPidKs();
    void resendPidKsTimed();

    void configure();
    void checkReady();
    void setStartTime(uint32_t time_ms);
    ros::Time getDeviceTime(uint32_t time_ms);
    bool getNextSegment();
    int getSegmentNum();
    bool waitForPacketStart();
    void processSerialPacket(string category);

    bool readSerial();
    void writeSerial(string name, const char *formats, ...);

    bool write_stop_flag;
    int write_thread_rate;
    queue<string> write_queue;
    boost::thread* write_thread;
    void write_packet_from_queue();
    void write_thread_task();

    void setup();
    void loop();
    void stop();

    bool set_pid(db_parsing::DodobotPidSrv::Request &req, db_parsing::DodobotPidSrv::Response &res);

    void setActive(bool state);
    void softRestart();
    void setReporting(bool state);
    void resetSensors();
    // void writeCurrentState();
    void writeSpeed(float speedA, float speedB);
    void writeK(PidKs* constants);
    void logPacketErrorCode(int error_code, unsigned long long packet_num);

    void parseEncoder();
    double convertTicksToCm(long ticks);

    void parseINA();
    void parseIR();
public:
    DodobotParsing(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _DODOBOT_PARSING_H_
