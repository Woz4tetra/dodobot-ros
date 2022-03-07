#include <db_description/db_description.h>


DodobotDescription::DodobotDescription(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    raw_joint_subs = new vector<ros::Subscriber>();
    
    joints_msg.header.frame_id = "base_link";

    string key;
    if (!ros::param::search("joint_names", key)) {
        ROS_ERROR("Failed to find joint_names parameter");
        std::exit(EXIT_FAILURE);
    }
    ROS_DEBUG("Found joint_names: %s", key.c_str());
    nh.getParam(key, _joint_names);

    for (int index = 0; index < _joint_names.size(); index++)
    {
        joints_msg.name.push_back(_joint_names.at(index));
        joints_msg.position.push_back(0.0);

        raw_joint_subs->push_back(
            nh.subscribe<std_msgs::Float64>(
                joints_msg.name.at(index), 50,
                boost::bind(&DodobotDescription::joint_callback, this, _1, index)
            )
        );
    }
    
    joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    
    ROS_INFO("db_description is ready!");
}

void DodobotDescription::joint_callback(const std_msgs::Float64ConstPtr& msg, int joint_index)
{
    joints_msg.position[joint_index] = msg->data;
    joints_msg.header.stamp = ros::Time::now();
}


void DodobotDescription::loop()
{
    joint_pub.publish(joints_msg);
}

int DodobotDescription::run()
{
    ros::Rate clock_rate(60);  // Hz

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
    return exit_code;
}
