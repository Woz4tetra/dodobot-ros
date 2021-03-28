#ifndef _DODOBOT_PARSING_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <queue>
#include <boost/thread/thread.hpp>
#include <iterator>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "serial/serial.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/Image.h"

#include "db_parsing/DodobotDrive.h"
#include "db_parsing/DodobotBumper.h"
#include "db_parsing/DodobotGripper.h"
#include "db_parsing/DodobotFSRs.h"
#include "db_parsing/DodobotLinear.h"
#include "db_parsing/DodobotLinearEvent.h"
#include "db_parsing/DodobotTilter.h"
#include "db_parsing/DodobotState.h"
#include "db_parsing/DodobotFunctions.h"
#include "db_parsing/DodobotFunctionsListing.h"
#include "db_parsing/DodobotNotify.h"

#include "db_parsing/DodobotPidSrv.h"
#include "db_parsing/DodobotUploadFile.h"
#include "db_parsing/DodobotListDir.h"
#include "db_parsing/DodobotSetState.h"
#include "db_parsing/DodobotGetState.h"

#include "keyboard_listener/KeyEvent.h"


using namespace std;

#define CHECK_SEGMENT(PARAM)  if (!getNextSegment(PARAM)) {  ROS_ERROR_STREAM("Not enough segments supplied for #" << getSegmentNum() + 1 << ". Buffer: " << formatPacketToPrint(_recvCharBuffer, _readPacketLen));  return;  }

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

class DodobotParsing {
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

    bool use_sensor_msg_time;
    bool active_on_start, reporting_on_start;
    bool was_reporting;

    StructRobotState* robotState;
    StructReadyState* readyState;
    ros::Publisher state_pub;
    db_parsing::DodobotState state_msg;
    void parseState();
    void parseReady();
    void setReadyFlag(bool state);

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
    uint32_t stepper_max_speed, stepper_max_accel, stepper_low_speed, stepper_low_accel;
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

    bool ready_for_images;
    string display_img_topic;
    image_transport::Subscriber image_sub;
    image_transport::ImageTransport image_transport;
    string starter_image_path;
    vector<unsigned char>* display_img_buf;
    vector<int> jpeg_params;
    int jpeg_image_quality;
    int image_resize_width, image_resize_height;
    void imgCallback(const sensor_msgs::ImageConstPtr& msg);
    void writeImage(const cv::Mat& image);

    ros::Publisher bumper_pub;
    db_parsing::DodobotBumper bumper_msg;
    void parseBumper();

    ros::Subscriber keyboard_sub;
    void keyboardCallback(const keyboard_listener::KeyEvent::ConstPtr& msg);

    ros::Subscriber robot_functions_sub;
    void robotFunctionsCallback(const db_parsing::DodobotFunctionsListing::ConstPtr& msg);

    ros::Publisher robot_functions_pub;
    db_parsing::DodobotFunctionsListing selected_fn_msg;
    void parseSelectedRobotFn();

    ros::Subscriber notification_sub;
    void notifyCallback(const db_parsing::DodobotNotify::ConstPtr& msg);

    bool motorsReady();
    bool robotReady();

    ros::ServiceServer pid_service;
    PidKs* pidConstants;
    ros::Timer pid_resend_timer;
    void resendPidKs();
    void resendPidKsTimed();
    bool set_pid(db_parsing::DodobotPidSrv::Request &req, db_parsing::DodobotPidSrv::Response &res);

    ros::ServiceServer file_service;
    bool upload_file(db_parsing::DodobotUploadFile::Request &req, db_parsing::DodobotUploadFile::Response &res);

    ros::ServiceServer listdir_service;
    bool db_listdir(db_parsing::DodobotListDir::Request &req, db_parsing::DodobotListDir::Response &res);

    ros::ServiceServer set_state_service;
    bool set_state(db_parsing::DodobotSetState::Request &req, db_parsing::DodobotSetState::Response &res);

    ros::ServiceServer get_state_service;
    bool get_state(db_parsing::DodobotGetState::Request &req, db_parsing::DodobotGetState::Response &res);

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
    bool waitForOK(uint32_t packet_num);
    bool waitForOK();

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

    void setActive(bool state);
    void softRestart();
    void setReporting(bool state);
    void resetSensors();
    // void writeCurrentState();
    void writeSpeed(float speedA, float speedB);
    void writeK(PidKs* constants);
    void logPacketErrorCode(int error_code, uint32_t packet_num);
    void logPacketErrorCode(int error_code, uint32_t packet_num, string message);

    string formatPacketToPrint(char* packet, uint32_t length);
    string formatPacketToPrint(string packet);
    string getPrintChar(unsigned char c);

    void parseEncoder();
    void parseINA();
    void parseIR();
public:
    DodobotParsing(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _DODOBOT_PARSING_H_
