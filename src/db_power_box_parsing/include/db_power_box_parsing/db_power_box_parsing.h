#pragma once

#include <exception>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/BatteryState.h"
#include "dodobot_serial.h"

using namespace std;

class DodobotPowerBoxParsing {
private:
    ros::NodeHandle nh;  // ROS node handle

    // ROS parameters
    string device_address;
    int device_baud;
    double initial_connect_delay;
    double charge_threshold_mA;
    int connection_attempts;

    // state variables
    bool is_charging;

    ros::Publisher battery_pub;
    sensor_msgs::BatteryState battery_msg;

    ros::Subscriber light_ring_sub;

    ros::Publisher is_charging_pub;
    ros::Timer is_charging_timer;

    DodobotSerial* serial_interface;
    void packet_callback(string category);

    void setup();
    void loop();
    void stop();

    void power_packet_callback(float shunt_voltage, float bus_voltage, float current_mA, float power_mW, float load_voltage);
    void is_charging_timer_callback(const ros::TimerEvent& event);
    void light_ring_callback(const std_msgs::Int32ConstPtr& msg);

    void set_ring_pattern(int pattern_index);
public:
    DodobotPowerBoxParsing(ros::NodeHandle* nodehandle);
    int run();
};
