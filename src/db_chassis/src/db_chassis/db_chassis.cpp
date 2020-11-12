#include <db_chassis/db_chassis.h>


DodobotChassis::DodobotChassis(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    string drive_cmd_topic_name = "";

    // Odometry parameters
    ros::param::param<double>("~wheel_radius_mm", wheel_radius_mm, 30.0) ;
    ros::param::param<double>("~wheel_distance_mm", wheel_distance_mm, 183.21);
    ros::param::param<double>("~ticks_per_rotation", ticks_per_rotation, 3840.0);
    ros::param::param<double>("~max_speed_tps", max_speed_tps, 9000.0);

    // Odometry speed parameters
    ros::param::param<double>("~min_angular_speed", min_angular_speed, 0.0);
    ros::param::param<double>("~max_angular_speed", max_angular_speed, 0.0);
    ros::param::param<double>("~min_linear_speed", min_linear_speed, 0.0);
    ros::param::param<double>("~max_linear_speed", max_linear_speed, 0.0);
    ros::param::param<double>("~zero_speed_epsilon", zero_speed_epsilon, 0.01);

    // Topic parameters
    ros::param::param<string>("~drive_pub_name", drive_pub_name, "drive_cmd");
    ros::param::param<bool>("~services_enabled", services_enabled, true);
    ros::param::param<bool>("~publish_odom_tf", publish_odom_tf, true);
    ros::param::param<bool>("~use_sensor_msg_time", use_sensor_msg_time, false);

    // Tilter parameters
    ros::param::param<double>("~tilter_lower_angle_deg", tilter_lower_angle_deg, -60.0);
    ros::param::param<double>("~tilter_upper_angle_deg", tilter_upper_angle_deg, 0.0);
    ros::param::param<int>("~tilter_lower_command", tilter_lower_command, 5);
    ros::param::param<int>("~tilter_upper_command", tilter_upper_command, 180);

    // Linear parameters
    ros::param::param<double>("~stepper_ticks_per_R_no_gearbox", stepper_ticks_per_R_no_gearbox, 200.0);
    ros::param::param<double>("~microsteps", microsteps, 8.0);
    ros::param::param<double>("~stepper_gearbox_ratio", stepper_gearbox_ratio, 26.0 + 103.0 / 121.0);
    ros::param::param<double>("~belt_pulley_radius_m", belt_pulley_radius_m, 0.0121);

    // Gripper parameters
    ros::param::param<int>("~gripper_open_cmd", gripper_open_cmd, 50);
    ros::param::param<int>("~gripper_closed_cmd", gripper_closed_cmd, 162);
    ros::param::param<double>("~gripper_open_angle_deg", gripper_open_angle_deg, 344.89282);
    ros::param::param<double>("~gripper_closed_angle_deg", gripper_closed_angle_deg, 302.560457);

    // Gripper geometry parameters
    ros::param::param<double>("~armature_length", armature_length, 0.06);
    ros::param::param<double>("~armature_width", armature_width, 0.01);
    ros::param::param<double>("~hinge_pin_to_armature_end", hinge_pin_to_armature_end, 0.004);
    ros::param::param<double>("~hinge_pin_diameter", hinge_pin_diameter, 0.0028575);
    ros::param::param<double>("~hinge_pin_to_pad_plane", hinge_pin_to_pad_plane, 0.0055);
    ros::param::param<double>("~pad_extension_offset", pad_extension_offset, 0.01998285);
    ros::param::param<double>("~central_axis_dist", central_axis_dist, 0.015);

    // TF parameters
    ros::param::param<string>("~child_frame", child_frame, "base_link");
    ros::param::param<string>("~odom_parent_frame", odom_parent_frame, "odom");
    ros::param::param<string>("~tilt_base_frame", tilt_base_frame, "tilt_base_link");
    ros::param::param<string>("~camera_rotate_frame", camera_rotate_frame, "camera_rotate_link");
    ros::param::param<string>("~linear_frame", linear_frame, "linear_link");
    ros::param::param<string>("~linear_base_frame", linear_base_frame, "linear_base_link");

    // Odometry conversions
    wheel_radius_m = wheel_radius_mm / 1000.0;
    wheel_distance_m = wheel_distance_mm / 1000.0;

    m_to_tick_factor = ticks_per_rotation / (2.0 * wheel_radius_m * M_PI);
    tick_to_m_factor = 1.0 / m_to_tick_factor;

    max_speed_mps = max_speed_tps * tick_to_m_factor;

    // Tilt conversions
    camera_tilt_angle = tilter_upper_angle;

    tilter_lower_angle = tilter_lower_angle_deg * M_PI/180.0;
    tilter_upper_angle = tilter_upper_angle_deg * M_PI/180.0;

    // Gripper conversions
    gripper_open_angle = gripper_open_angle_deg * M_PI/180.0;
    gripper_closed_angle = gripper_closed_angle_deg * M_PI/180.0;

    rotation_offset_x = armature_length + hinge_pin_to_armature_end;
    rotation_offset_y = (armature_width + hinge_pin_diameter) / 2.0;

    compute_parallel_dist_to_angle_lookup();

    // Linear conversions
    stepper_ticks_per_R = stepper_ticks_per_R_no_gearbox * microsteps * stepper_gearbox_ratio;
    stepper_R_per_tick = 1.0 / stepper_ticks_per_R;
    step_ticks_to_linear_m = stepper_R_per_tick * belt_pulley_radius_m * 2.0 * M_PI;
    step_linear_m_to_ticks = 1.0 / step_ticks_to_linear_m;

    // velocity and acceleration commands at the microcontroller side are
    // 4 orders of magnitude larger for some reason...
    step_linear_m_to_speed_ticks = step_linear_m_to_ticks * 1E4;

    tf2::Quaternion zero_quat;
    zero_quat.setRPY(0.0, 0.0, 0.0);

    ROS_INFO_STREAM("drive_pub_name: " << drive_pub_name);
    ROS_INFO_STREAM("min_angular_speed: " << min_angular_speed);
    ROS_INFO_STREAM("max_angular_speed: " << max_angular_speed);
    ROS_INFO_STREAM("min_linear_speed: " << min_linear_speed);
    ROS_INFO_STREAM("max_linear_speed: " << max_linear_speed);

    // Odometry state
    odom_timestamp = ros::Time::now();
    prev_odom_time = ros::Time::now();
    prev_left_ticks = 0;
    prev_right_ticks = 0;

    odom_state = init_odom_state();

    odom_msg.header.frame_id = odom_parent_frame;
    odom_msg.child_frame_id = child_frame;
    /* [
        1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
    ] */
    odom_msg.pose.covariance[0] = 1e-3;
    odom_msg.pose.covariance[7] = 1e-3;
    odom_msg.pose.covariance[14] = 1e-3;
    odom_msg.pose.covariance[21] = 1e-3;
    odom_msg.pose.covariance[29] = 1e-3;
    odom_msg.pose.covariance[36] = 1e-3;

    // cmd_vel command values
    linear_speed_cmd = 0.0;
    angular_speed_cmd = 0.0;

    // JointState messages

    // Publishers

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    drive_pub = nh.advertise<db_parsing::DodobotDrive>(drive_pub_name, 50);
    gripper_pub = nh.advertise<db_parsing::DodobotGripper>("gripper_cmd", 50);
    parallel_gripper_pub = nh.advertise<db_parsing::DodobotParallelGripper>("parallel_gripper", 50);
    linear_pub = nh.advertise<db_parsing::DodobotLinear>("linear_cmd", 50);
    linear_pos_pub = nh.advertise<db_chassis::LinearPosition>("linear_pos", 50);

    // Subscribers
    twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 50, &DodobotChassis::twist_callback, this);
    drive_sub = nh.subscribe<db_parsing::DodobotDrive>("drive", 50, &DodobotChassis::drive_callback, this);
    tilter_sub = nh.subscribe<db_parsing::DodobotTilter>("tilter", 50, &DodobotChassis::tilter_callback, this);
    linear_sub = nh.subscribe<db_parsing::DodobotLinear>("linear", 50, &DodobotChassis::linear_callback, this);
    gripper_sub = nh.subscribe<db_parsing::DodobotGripper>("gripper", 50, &DodobotChassis::gripper_callback, this);
    parallel_gripper_sub = nh.subscribe<db_parsing::DodobotParallelGripper>("parallel_gripper_cmd", 50, &DodobotChassis::parallel_gripper_callback, this);
    linear_pos_sub = nh.subscribe<db_chassis::LinearPosition>("linear_pos_cmd", 50, &DodobotChassis::linear_pos_callback, this);
    linear_vel_sub = nh.subscribe<db_chassis::LinearVelocity>("linear_vel_cmd", 50, &DodobotChassis::linear_vel_callback, this);

    pid_service_name = "dodobot_pid";
    odom_reset_service_name = "dodobot_odom_reset";
    first_time_pid_setup = false;

    if (services_enabled) {
        set_pid_srv = nh.serviceClient<db_parsing::DodobotPidSrv>(pid_service_name);
        // set_pid_srv.waitForExistence();
        ROS_INFO("%s service is ready", pid_service_name.c_str());

        dyn_cfg_wrapped_callback = boost::bind(&DodobotChassis::dynamic_callback, this, _1, _2);
        dyn_cfg.setCallback(dyn_cfg_wrapped_callback);

        odom_reset_srv = nh.advertiseService(odom_reset_service_name, &DodobotChassis::odom_reset_callback, this);
        ROS_INFO("%s service is ready", odom_reset_service_name.c_str());
    }

    ROS_INFO("db_chassis init done");
}


void DodobotChassis::setup()
{

}

void DodobotChassis::loop()
{
    compute_odometry();
    publish_chassis_data();
}

void DodobotChassis::stop()
{

}


int DodobotChassis::run()
{
    setup();

    ros::Rate clock_rate(60);  // run loop at 60 Hz

    int exit_code = 0;
    while (ros::ok())
    {
        // let ROS process any events
        ros::spinOnce();
        clock_rate.sleep();

        try {
            loop();
        }
        catch (exception& e) {
            ROS_ERROR_STREAM("Exception in main loop: " << e.what());
            exit_code = 1;
            break;
        }
    }
    stop();

    return exit_code;
}

void DodobotChassis::compute_parallel_dist_to_angle_lookup()
{

}

void DodobotChassis::dynamic_callback(db_chassis::DodobotChassisConfig &config, uint32_t level)
{
    if (!services_enabled) {
        ROS_WARN("Services for this node aren't enabled!");
        return;
    }
    if (!first_time_pid_setup) {
        first_time_pid_setup = true;
        return;
    }
    db_parsing::DodobotPidSrv srv;
    srv.request.kp_A = config.kp_A;
    srv.request.ki_A = config.ki_A;
    srv.request.kd_A = config.kd_A;
    srv.request.kp_B = config.kp_B;
    srv.request.ki_B = config.ki_B;
    srv.request.kd_B = config.kd_B;
    srv.request.speed_kA = config.speed_kA;
    srv.request.speed_kB = config.speed_kB;

    set_pid_srv.call(srv);
}

bool DodobotChassis::odom_reset_callback(db_chassis::DodobotOdomReset::Request &req, db_chassis::DodobotOdomReset::Response &resp)
{
    reset_odom_state(odom_state);
    ROS_INFO("Resetting odom to x: %f, y: %f, theta: %f", odom_state->x, odom_state->y, odom_state->theta);
    resp.resp = true;
    return true;
}

//
// Sub callbacks
//

void DodobotChassis::twist_callback(geometry_msgs::Twist msg)
{

}

void DodobotChassis::drive_callback(db_parsing::DodobotDrive msg) {
    drive_sub_msg = msg;
}

void DodobotChassis::tilter_callback(db_parsing::DodobotTilter msg)
{

}

void DodobotChassis::linear_callback(db_parsing::DodobotLinear msg)
{

}

void DodobotChassis::gripper_callback(db_parsing::DodobotGripper msg)
{

}

void DodobotChassis::parallel_gripper_callback(db_parsing::DodobotParallelGripper msg)
{

}

void DodobotChassis::linear_pos_callback(db_chassis::LinearPosition msg)
{

}

void DodobotChassis::linear_vel_callback(db_chassis::LinearVelocity msg)
{

}

//
// Compute odometry
//

double DodobotChassis::ticks_to_m(int64_t ticks) {
    return (double)(ticks) * tick_to_m_factor;
}

void DodobotChassis::odom_estimator_update(double delta_left, double delta_right, double left_speed, double right_speed, double dt)
{
    double theta_n_1 = odom_state->theta;

    double v = (left_speed + right_speed) / 2.0;
    double w = (right_speed - left_speed) / wheel_distance_m;

    // reference: https://www.cs.cmu.edu/16311/current/labs/lab03/
    double k00 = v * cos(theta_n_1);
    double k01 = v * sin(theta_n_1);
    double k02 = w;

    double k10 = v * cos(theta_n_1 + dt / 2.0 * k02);
    double k11 = v * sin(theta_n_1 + dt / 2.0 * k02);
    double k12 = w;

    double k20 = v * cos(theta_n_1 + dt / 2.0 * k12);
    double k21 = v * sin(theta_n_1 + dt / 2.0 * k12);
    double k22 = w;

    double k30 = v * cos(theta_n_1 + dt / 2.0 * k22);
    double k31 = v * sin(theta_n_1 + dt / 2.0 * k22);
    double k32 = w;

    double dx     = dt / 6.0 * (k00 + 2 * (k10 + k20) + k30);
    double dy     = dt / 6.0 * (k01 + 2 * (k11 + k21) + k31);
    double dtheta = dt / 6.0 * (k02 + 2 * (k12 + k22) + k32);

    odom_state->x += dx;
    odom_state->y += dy;
    odom_state->theta += dtheta;
    odom_state->theta = fmod(odom_state->theta, 2.0 * M_PI);

    odom_state->vx = v * cos(odom_state->theta);
    odom_state->vy = v * sin(odom_state->theta);

    odom_state->v = v;
    odom_state->w = w;
}

void DodobotChassis::compute_odometry()
{
    ros::Time now = ros::Time::now();
    odom_timestamp = now;
    double dt = (now - prev_odom_time).toSec();
    prev_odom_time = now;

    double delta_left = ticks_to_m(drive_sub_msg.left_enc_pos - prev_left_ticks);
    double delta_right = ticks_to_m(drive_sub_msg.right_enc_pos - prev_right_ticks);
    double left_speed = ticks_to_m(drive_sub_msg.left_enc_speed);
    double right_speed = ticks_to_m(drive_sub_msg.right_enc_speed);

    prev_left_ticks = drive_sub_msg.left_enc_pos;
    prev_right_ticks = drive_sub_msg.right_enc_pos;

    odom_estimator_update(
        delta_left,
        delta_right,
        left_speed,
        right_speed,
        dt
    );
}

void DodobotChassis::publish_chassis_data()
{
    ros::Time now = ros::Time::now();
    if (use_sensor_msg_time) {
        now = drive_sub_msg.header.stamp;
    }

    tf2::Quaternion odom_quaternion;
    odom_quaternion.setRPY(0.0, 0.0, odom_state->theta);
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat_msg, odom_quaternion);

    if (publish_odom_tf)
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = now;
        transformStamped.header.frame_id = odom_parent_frame;
        transformStamped.child_frame_id = child_frame;
        transformStamped.transform.translation.x = odom_state->x;
        transformStamped.transform.translation.y = odom_state->y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_state->theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tf_broadcaster.sendTransform(transformStamped);
    }

    odom_msg.header.stamp = now;
    odom_msg.pose.pose.position.x = odom_state->x;
    odom_msg.pose.pose.position.y = odom_state->y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation = quat_msg;

    odom_msg.twist.twist.linear.x = odom_state->vx;
    odom_msg.twist.twist.linear.y = odom_state->vy;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = odom_state->w;

    // odom_msg.pose.covariance = odom_covariance;

    odom_pub.publish(odom_msg);
}
