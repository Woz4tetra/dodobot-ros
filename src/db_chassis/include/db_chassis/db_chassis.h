#ifndef _DODOBOT_PARSING_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>

#include "db_parsing/DodobotPidSrv.h"

#include "db_parsing/DodobotDrive.h"
#include "db_parsing/DodobotBumper.h"
#include "db_parsing/DodobotGripper.h"
#include "db_parsing/DodobotFSRs.h"
#include "db_parsing/DodobotLinear.h"
#include "db_parsing/DodobotLinearEvent.h"
#include "db_parsing/DodobotTilter.h"
#include "db_parsing/DodobotParallelGripper.h"

#include "db_chassis/LinearPosition.h"
#include "db_chassis/LinearVelocity.h"

#include "db_chassis/DodobotOdomReset.h"
#include "db_chassis/DodobotChassisConfig.h"


using namespace std;

struct OdomState {
    double x;  // x position
    double y;  // y position
    double theta;  // angle

    double v;  // linear velocity
    double w;  // angular velocity

    double vx;  // x component of velocity
    double vy;  // y component of velocity
};

void reset_odom_state(OdomState* state)
{
    state->x = 0.0;
    state->y = 0.0;
    state->theta = 0.0;
    state->v = 0.0;
    state->w = 0.0;
    state->vx = 0.0;
    state->vy = 0.0;
}


OdomState* init_odom_state()
{
    OdomState* state = new OdomState;
    reset_odom_state(state);
    return state;
}

class DodobotChassis {
private:
    ros::NodeHandle nh;  // ROS node handle

    // launch parameters
    double wheel_radius_mm;
    double wheel_distance_mm;
    double ticks_per_rotation;
    double max_speed_tps;

    double min_angular_speed;
    double max_angular_speed;
    double min_linear_speed;
    double max_linear_speed;
    double zero_speed_epsilon;

    string drive_pub_name;
    bool services_enabled;
    bool publish_odom_tf;
    bool use_sensor_msg_time;

    double tilter_lower_angle_deg;
    double tilter_upper_angle_deg;
    int tilter_lower_command;
    int tilter_upper_command;

    double stepper_ticks_per_R_no_gearbox;
    double microsteps;
    double stepper_gearbox_ratio;
    double belt_pulley_radius_m;

    int gripper_open_cmd;
    int gripper_closed_cmd;
    double gripper_open_angle_deg;
    double gripper_closed_angle_deg;

    double armature_length;
    double armature_width;
    double hinge_pin_to_armature_end;
    double hinge_pin_diameter;
    double hinge_pin_to_pad_plane;
    double pad_extension_offset;
    double central_axis_dist;

    string child_frame;
    string odom_parent_frame;
    string tilt_base_frame;
    string camera_rotate_frame;
    string linear_frame;
    string linear_base_frame;

    // converted variables
    double wheel_radius_m;
    double m_to_tick_factor;
    double tick_to_m_factor;
    double max_speed_mps;
    double wheel_distance_m;

    double tilter_upper_angle;
    double tilter_lower_angle;

    double gripper_open_angle;
    double gripper_closed_angle;

    double rotation_offset_x;
    double rotation_offset_y;

    double stepper_ticks_per_R;
    double stepper_R_per_tick;
    double step_ticks_to_linear_m;
    double step_linear_m_to_ticks;
    double step_linear_m_to_speed_ticks;

    double linear_speed_cmd;
    double angular_speed_cmd;

    // State variables
    double camera_tilt_angle;
    double stepper_z_pos;

    ros::Time odom_timestamp;
    ros::Time prev_odom_time;
    int64_t prev_left_ticks;
    int64_t prev_right_ticks;

    // Publishers
    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Publisher odom_pub;
    ros::Publisher drive_pub;
    ros::Publisher gripper_pub;
    ros::Publisher parallel_gripper_pub;
    ros::Publisher linear_pub;
    ros::Publisher linear_pos_pub;
    ros::Publisher linear_joint_pub;
    ros::Publisher tilter_joint_pub;

    // Subscribers
    ros::Subscriber twist_sub;
    ros::Subscriber drive_sub;
    ros::Subscriber tilter_sub;
    ros::Subscriber linear_sub;
    ros::Subscriber gripper_sub;
    ros::Subscriber parallel_gripper_sub;
    ros::Subscriber linear_pos_sub;
    ros::Subscriber linear_vel_sub;

    // Sub callbacks
    void twist_callback(geometry_msgs::Twist msg);
    void drive_callback(db_parsing::DodobotDrive msg);
    void tilter_callback(db_parsing::DodobotTilter msg);
    void linear_callback(db_parsing::DodobotLinear msg);
    void gripper_callback(db_parsing::DodobotGripper msg);
    void parallel_gripper_callback(db_parsing::DodobotParallelGripper msg);
    void linear_pos_callback(db_chassis::LinearPosition msg);
    void linear_vel_callback(db_chassis::LinearVelocity msg);

    // messages
    db_parsing::DodobotDrive drive_sub_msg;
    db_parsing::DodobotDrive drive_pub_msg;

    db_parsing::DodobotParallelGripper parallel_gripper_msg;
    db_parsing::DodobotGripper gripper_msg;

    OdomState* odom_state;
    nav_msgs::Odometry odom_msg;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<db_chassis::DodobotChassisConfig> dyn_cfg;
    dynamic_reconfigure::Server<db_chassis::DodobotChassisConfig>::CallbackType dyn_cfg_wrapped_callback;
    void dynamic_callback(db_chassis::DodobotChassisConfig &config, uint32_t level);

    // Services
    ros::ServiceServer odom_reset_srv;
    string odom_reset_service_name;
    bool odom_reset_callback(db_chassis::DodobotOdomReset::Request &req, db_chassis::DodobotOdomReset::Response &resp);

    ros::ServiceClient set_pid_srv;
    string pid_service_name;
    bool first_time_pid_setup;

    void setup();
    void loop();
    void stop();

    // Joint states
    sensor_msgs::JointState linear_joint;
    sensor_msgs::JointState tilter_joint;
    void publish_joint_states();

    // Data handling/conversion
    size_t num_dist_samples;
    vector<double> angle_samples;
    vector<double> dist_samples;
    double angle_to_parallel_dist(double angle_rad);
    double parallel_dist_to_angle(double parallel_dist);
    void compute_parallel_dist_to_angle_lookup();

    double servo_to_angle(int command, int max_command, int min_command, double min_angle, double max_angle);
    int angle_to_servo(double angle, int max_command, int min_command, double min_angle, double max_angle);
    double tilt_command_to_angle_rad(int command);

    double bound_speed(double value, double lower, double upper, double epsilon);

    double ticks_to_m(int64_t ticks);
    int64_t m_to_ticks(double dist_m);

    // Compute odometry
    void odom_estimator_update(double delta_left, double delta_right, double left_speed, double right_speed, double dt);
    void compute_odometry();
    void publish_chassis_data();

public:
    DodobotChassis(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _DODOBOT_PARSING_H_
