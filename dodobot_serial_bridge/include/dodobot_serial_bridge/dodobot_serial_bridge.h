#ifndef _ROVER6_SERIAL_BRIDGE_H_

#include <exception>
#include <iostream>
#include <ctime>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "serial/serial.h"

#include "dodobot_serial_bridge/DodobotEncoder.h"
#include "dodobot_serial_bridge/DodobotFSR.h"
#include "dodobot_serial_bridge/DodobotSafety.h"
#include "dodobot_serial_bridge/DodobotTOF.h"
#include "dodobot_serial_bridge/DodobotMotors.h"
#include "dodobot_serial_bridge/DodobotServos.h"
#include "dodobot_serial_bridge/DodobotServoPos.h"

#include "dodobot_serial_bridge/DodobotPidSrv.h"
#include "dodobot_serial_bridge/DodobotSafetySrv.h"
#include "dodobot_serial_bridge/DodobotMenuSrv.h"


using namespace std;

#define CHECK_SEGMENT  if (!getNextSegment()) {  ROS_ERROR_STREAM("Not enough segments supplied for #" << getSegmentNum() << ". Buffer: " << _serialBuffer);  return;  }

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
    bool is_active;
    bool battery_ok;
    bool motors_active;
    double loop_rate;
};

class ReadyTimeoutExceptionClass : public exception {
    virtual const char* what() const throw() { return "Timeout reached. Never got ready signal from serial device"; }
} ReadyTimeoutException;

class DodobotSerialBridge {
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
    dodobot_serial_bridge::DodobotGripper gripper_msg;
    void parseFSR();
    void parseGripper();
    void gripperCallback(const dodobot_serial_bridge::DodobotGripper::ConstPtr& msg);
    void writeGripper(uint8_t command, uint8_t force_threshold);

    ros::Publisher tilter_pub;
    ros::Subscriber tilter_sub;
    dodobot_serial_bridge::DodobotTilter tilter_msg;
    void parseTilter();
    void tilterCallback(const dodobot_serial_bridge::DodobotTilterCmd::ConstPtr& msg);
    void writeTilter(uint8_t command);

    ros::Publisher linear_pub;
    ros::Subscriber linear_sub;
    dodobot_serial_bridge::DodobotLinear linear_msg;
    void parseLinear();
    void linearCallback(const dodobot_serial_bridge::DodobotLinearCmd::ConstPtr& msg);
    void writeLinear(uint8_t command);

    ros::Publisher battery_pub;
    sensor_msgs::BatteryState battery_msg;
    void parseBattery();

    string _encFrameID;
    double _wheelRadiusCm, _ticksPerRotation, _maxRPM, _cmPerTick, _cpsToCmd;
    string _driveTopicName;
    ros::Publisher drive_pub;
    ros::Subscriber drive_sub;
    dodobot_serial_bridge::DodobotDrive drive_msg;
    void parseDrive();
    void driveCallback(const dodobot_serial_bridge::DodobotDrive::ConstPtr& msg);
    void writeDriveChassis(const dodobot_serial_bridge::DodobotDrive::ConstPtr& msg);

    ros::Publisher bumper_pub;
    void parseBumper();

    bool motorsReady();
    bool robotReady();

    StructRobotState* robotState;
    StructReadyState* readyState;

    ros::ServiceServer pid_service;

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

    void setup();
    void loop();
    void stop();

    bool set_pid(dodobot_serial_bridge::DodobotPidSrv::Request &req, dodobot_serial_bridge::DodobotPidSrv::Response &res);

    void setActive(bool state);
    void softRestart();
    void setReporting(bool state);
    void resetSensors();
    // void writeCurrentState();
    void writeSpeed(float speedA, float speedB);
    void writeK(float kp_A, float ki_A, float kd_A, float kp_B, float ki_B, float kd_B, float speed_kA, float speed_kB);
    void logPacketErrorCode(int error_code, unsigned long long packet_num);

    void parseEncoder();
    double convertTicksToCm(long ticks);

    void parseINA();
    void parseIR();
public:
    DodobotSerialBridge(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _ROVER6_SERIAL_BRIDGE_H_
