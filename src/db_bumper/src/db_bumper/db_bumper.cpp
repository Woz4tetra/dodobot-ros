l1_a#include <db_chassis/db_chassis.h>


DodobotBumper::DodobotBumper(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    // Bumper dimension parameters
    ROS_ASSERT_MSG(!ros::param::has("left_bumper_x_points"), "Failed to find left_bumper_x_points parameter");
    ros::param::param("left_bumper_x_points", left_bumper_x_points);
    ROS_ASSERT_MSG(!ros::param::has("left_bumper_y_points"), "Failed to find left_bumper_y_points parameter");
    ros::param::param("left_bumper_y_points", left_bumper_y_points);

    ROS_ASSERT_MSG(!ros::param::has("right_bumper_x_points"), "Failed to find right_bumper_x_points parameter");
    ros::param::param("right_bumper_x_points", right_bumper_x_points);
    ROS_ASSERT_MSG(!ros::param::has("right_bumper_y_points"), "Failed to find right_bumper_y_points parameter");
    ros::param::param("right_bumper_y_points", right_bumper_y_points);

    ros::param::param<string>("~bumper_scan_topic", bumper_scan_topic, "bumper_occupancy");
    ros::param::param<string>("~bumper_frame", bumper_frame, "bumper");
    ros::param::param<unsigned int>("~scan_count", scan_count, 25);
    ros::param::param<double>("~range_max", range_max, 100.0);

    ROS_ASSERT_MSG(left_bumper_x_points.size() == left_bumper_y_points.size(), "Left bumper points are not equal in size!");
    ROS_ASSERT_MSG(left_bumper_x_points.size() > 0, "Left bumper points have zero points!");

    ROS_ASSERT_MSG(right_bumper_x_points.size() == right_bumper_y_points.size(), "Right bumper points are not equal in size!");
    ROS_ASSERT_MSG(right_bumper_x_points.size() > 0, "Right bumper points have zero points!");

    // Publishers
    scan_pub = nh.advertise<sensor_msgs::LaserScan>(bumper_scan_topic, 5);

    // Subscribers
    bumper_sub = nh.subscribe<db_parsing::DodobotBumper>("bumper", 50, &DodobotBumper::bumper_callback, this);

    // "laser scan" init


    ROS_INFO("db_bumper init done");
}

double get_min_value(vector<double>* points)
{
    vector<double>::iterator result = std::min_element(points->begin(), points->end());
    int index = std::distance(points->begin(), result);
    return points->at(index);
}

double get_max_value(vector<double>* points)
{
    vector<double>::iterator result = std::max_element(points->begin(), points->end());
    int index = std::distance(points->begin(), result);
    return points->at(index);
}

void DodobotBumper::init_scan_msg()
{
    double min_x = std::min(
        get_min_value(&left_bumper_x_points),
        get_min_value(&right_bumper_x_points)
    );
    double max_x = std::max(
        get_max_value(&left_bumper_x_points),
        get_max_value(&right_bumper_x_points)
    );

    double min_y = std::min(
        get_min_value(&left_bumper_y_points),
        get_min_value(&right_bumper_y_points)
    );
    double max_y = std::max(
        get_max_value(&left_bumper_y_points),
        get_max_value(&right_bumper_y_points)
    );

    double angle_min = atan2(min_y, min_x);
    double angle_max = atan2(max_y, max_x);
    double angle_increment = M_PI / scan_count;

    scan.header.frame_id = bumper_frame;
    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = angle_increment;
    // scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.time_increment = 0.0;
    scan.range_min = 0.0;
    scan.range_max = range_max;

    scan.ranges.resize(scan_count);
    scan.intensities.resize(scan_count);

    for (size_t i = 0; i < scan.intensities.size(); i++) {
        scan.intensities[i] = 100.0;
    }

    left_occupied_scan.resize(scan_count);
    right_occupied_scan.resize(scan_count);

    vector<point> left_bumper_points;
    vector<point> right_bumper_points;
    to_points_vector(&left_bumper_x_points, &left_bumper_y_points, &left_bumper_points);
    to_points_vector(&right_bumper_x_points, &right_bumper_y_points, &right_bumper_points);

    point origin = std::make_pair(0.0, 0.0);
    double scan_angle = angle_min;
    for (size_t scan_index = 0; scan_index < scan_count - 1; scan_index++)
    {
        point scan_point;
        scan_point.first = range_max * cos(scan_angle);
        scan_point.second = range_max * sin(scan_angle);

        for (size_t point_index = 0; point_index < left_bumper_points.size() - 1; point_index++)
        {
            point p1 = left_bumper_points[point_index];
            point p2 = left_bumper_points[point_index + 1];
            point intersect_p = get_line_intersection(origin, scan_point, p1, p2);
            left_occupied_scan[scan_index] = get_scan_dist(intersect_p, range_max);
        }

        for (size_t point_index = 0; point_index < right_bumper_points.size() - 1; point_index++)
        {
            point p1 = right_bumper_points[point_index];
            point p2 = right_bumper_points[point_index + 1];
            point intersect_p = get_line_intersection(origin, scan_point, p1, p2);
            right_occupied_scan[scan_index] = get_scan_dist(intersect_p, range_max);
        }

        scan_angle += angle_increment;
    }
}

point DodobotBumper::get_line_intersection(point l1_a, point l1_b, point l2_a, point l2_b)
{
    // source: https://www.geeksforgeeks.org/program-for-point-of-intersection-of-two-lines/
    // Line AB represented as a1x + b1y = c1
    double a1 = l1_b.second - l1_a.second;
    double b1 = l1_a.first - l1_b.first;
    double c1 = a1*(l1_a.first) + b1*(l1_a.second);

    // Line CD represented as a2x + b2y = c2
    double a2 = l2_b.second - l2_a.second;
    double b2 = l2_a.first - l2_b.first;
    double c2 = a2*(l2_a.first)+ b2*(l2_a.second);

    double determinant = a1*b2 - a2*b1;

    if (determinant == 0)
    {
        // The lines are parallel. This is simplified
        // by returning a pair of FLT_MAX
        return std::make_pair(FLT_MAX, FLT_MAX);
    }
    else
    {
        double x = (b2*c1 - b1*c2)/determinant;
        double y = (a1*c2 - a2*c1)/determinant;
        return std::make_pair(x, y);
    }
}

double DodobotBumper::to_points_vector(vector<double>* input_x, vector<double>* input_y, vector<point>* output)
{
    ROS_ASSERT_MSG(input_x->size() == input_y->size(), "X and Y points are not equal in size!");
    output.resize(input_x->size());
    for (size_t i = 0; i < input_x->size(); i++) {
        *output[i] = std::make_pair(input_x->at(i), input_y->at(i));
    }
}

double DodobotBumper::get_scan_dist(point p, double max_range)
{
    if (p.first == FLT_MAX || p.second == FLT_MAX) {
        return max_range;
    }
    else {
        return sqrt(pow(p.first, 2), pow(p.second, 2));
    }
}

void DodobotBumper::apply_scan(bool left, bool right)
{
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        scan.ranges[i] = range_max;
    }
    if (left) {
        for (size_t scan_index = 0; scan_index < left_occupied_scan.size(); scan_index++) {
            if (left_occupied_scan[scan_index] < range_max) {
                scan.ranges[scan_index] = left_occupied_scan[scan_index];
            }
        }
    }
    if (right) {
        for (size_t scan_index = 0; scan_index < right_occupied_scan.size(); scan_index++) {
            if (right_occupied_scan[scan_index] < range_max) {
                scan.ranges[scan_index] = right_occupied_scan[scan_index];
            }
        }
    }
}



int DodobotBumper::run()
{
    // ros::Rate clock_rate(60);  // run loop at 60 Hz
    //
    // int exit_code = 0;
    // while (ros::ok())
    // {
    //     // let ROS process any events
    //     ros::spinOnce();
    //     clock_rate.sleep();
    //
    //     try {
    //
    //     }
    //     catch (exception& e) {
    //         ROS_ERROR_STREAM("Exception in main loop: " << e.what());
    //         exit_code = 1;
    //         break;
    //     }
    // }
    //
    // return exit_code;
    ros::spin();
    return 0;
}

//
// Sub callbacks
//

void DodobotBumper::bumper_callback(db_parsing::DodobotBumper msg)
{
    apply_scan(msg.left, msg.right);

    scan.header.stamp = ros::Time::now();
    scan_pub.publish(scan);
}
