#pragma once

#include <exception>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/ColorRGBA.h"
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

    ros::Subscriber ring_light_sub;
    ros::Subscriber color_raw_sub;

    ros::Publisher is_charging_pub;
    ros::Timer is_charging_timer;

    DodobotSerial* serial_interface;
    void packet_callback(string category);

    void setup();
    void loop();
    void stop();

    void power_packet_callback(float shunt_voltage, float bus_voltage, float current_mA, float power_mW, float load_voltage);
    void is_charging_timer_callback(const ros::TimerEvent& event);
    void ring_light_callback(const std_msgs::ColorRGBAConstPtr& msg);
    void color_raw_callback(const std_msgs::UInt32ConstPtr& msg);

    void set_ring_color(double r_channel, double g_channel, double b_channel, double a_channel);
    void set_ring_color(uint32_t color);
public:
    DodobotPowerBoxParsing(ros::NodeHandle* nodehandle);
    int run();
};
