#include <db_bumper/db_bumper.h>


DodobotBumper::DodobotBumper(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    ros::param::param<string>("~bumper_scan_topic", bumper_scan_topic, "bumper_occupancy");
    ros::param::param<string>("~bumper_frame", bumper_frame, "bumper");
    ros::param::param<int>("~scan_count", scan_count, 25);
    ros::param::param<double>("~range_max", range_max, 100.0);

    ROS_INFO("bumper_frame: %s", bumper_frame.c_str());
    ROS_INFO("bumper_scan_topic: %s", bumper_scan_topic.c_str());

    string key;
    // Bumper dimension parameters
    ROS_ASSERT_MSG(ros::param::search("left_bumper_x_points", key), "Failed to find left_bumper_x_points parameter");
    nh.getParam(key, left_bumper_x_points);
    ROS_ASSERT_MSG(ros::param::search("left_bumper_y_points", key), "Failed to find left_bumper_y_points parameter");
    nh.getParam(key, left_bumper_y_points);

    ROS_ASSERT_MSG(ros::param::search("right_bumper_x_points", key), "Failed to find right_bumper_x_points parameter");
    nh.getParam(key, right_bumper_x_points);
    ROS_ASSERT_MSG(ros::param::search("right_bumper_y_points", key), "Failed to find right_bumper_y_points parameter");
    nh.getParam(key, right_bumper_y_points);


    ROS_ASSERT_MSG(left_bumper_x_points.size() == left_bumper_y_points.size(), "Left bumper points are not equal in size!");
    ROS_ASSERT_MSG(left_bumper_x_points.size() > 0, "Left bumper points has zero length!");

    ROS_ASSERT_MSG(right_bumper_x_points.size() == right_bumper_y_points.size(), "Right bumper points are not equal in size!");
    ROS_ASSERT_MSG(right_bumper_x_points.size() > 0, "Right bumper points has zero length!");

    ROS_INFO_STREAM("left_bumper_x_points len: " << left_bumper_x_points.size());
    ROS_INFO_STREAM("left_bumper_y_points len: " << left_bumper_y_points.size());
    ROS_INFO_STREAM("right_bumper_x_points len: " << right_bumper_x_points.size());
    ROS_INFO_STREAM("right_bumper_y_points len: " << right_bumper_y_points.size());

    // Publishers
    scan_pub = nh.advertise<sensor_msgs::LaserScan>(bumper_scan_topic, 5);

    // Subscribers
    bumper_sub = nh.subscribe<db_parsing::DodobotBumper>("bumper", 50, &DodobotBumper::bumper_callback, this);

    // "laser scan" init
    init_scan_msg();

    ROS_INFO("db_bumper init done");
}

double DodobotBumper::get_min_value(vector<double>* points)
{
    vector<double>::iterator result = std::min_element(points->begin(), points->end());
    int index = std::distance(points->begin(), result);
    return points->at(index);
}

double DodobotBumper::get_max_value(vector<double>* points)
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

    double angle_min = -M_PI; //atan2(min_y, min_x);
    double angle_max = M_PI; //atan2(max_y, max_x);
    double angle_increment = 2 * M_PI / scan_count;

    scan.header.frame_id = bumper_frame;
    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = angle_increment;
    // scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.time_increment = 0.0;
    scan.range_min = 0.0;
    scan.range_max = range_max;

    scan.ranges.resize(scan_count, range_max);
    scan.intensities.assign(scan_count, 100.0);

    left_occupied_scan.assign(scan_count, range_max);
    right_occupied_scan.assign(scan_count, range_max);

    vector<point> left_bumper_points;
    vector<point> right_bumper_points;
    to_points_vector(&left_bumper_x_points, &left_bumper_y_points, &left_bumper_points);
    to_points_vector(&right_bumper_x_points, &right_bumper_y_points, &right_bumper_points);

    point origin = std::make_pair(0.0, 0.0);
    double scan_angle = angle_min;
    for (size_t scan_index = 0; scan_index < scan_count; scan_index++)
    {
        point scan_point;
        scan_point.first = range_max * cos(scan_angle);
        scan_point.second = range_max * sin(scan_angle);

        for (size_t point_index = 0; point_index < left_bumper_points.size() - 1; point_index++)
        {
            point p1 = left_bumper_points[point_index];
            point p2 = left_bumper_points[point_index + 1];
            point intersect_p = get_line_intersection(origin, scan_point, p1, p2);
            double dist = get_scan_dist(intersect_p);
            if (dist != FLT_MAX) {
                left_occupied_scan[scan_index] = dist;
            }
        }

        for (size_t point_index = 0; point_index < right_bumper_points.size() - 1; point_index++)
        {
            point p1 = right_bumper_points[point_index];
            point p2 = right_bumper_points[point_index + 1];
            point intersect_p = get_line_intersection(origin, scan_point, p1, p2);
            double dist = get_scan_dist(intersect_p);
            if (dist != FLT_MAX) {
                right_occupied_scan[scan_index] = dist;
            }
        }

        scan_angle += angle_increment;
    }
}

double DodobotBumper::cross(point p1, point p2) {
    return p1.first * p2.second - p1.second * p2.first;
}

double DodobotBumper::dot(point p1, point p2) {
    return p1.first * p1.second + p1.first * p2.second;
}

point DodobotBumper::get_line_intersection(point l11, point l12, point l21, point l22)
{
    // source: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    point p = l11;
    point q = l21;

    point r = std::make_pair(l12.first - l11.first, l12.second - l11.second);
    point s = std::make_pair(l22.first - l21.first, l22.second - l21.second);

    point a = std::make_pair(q.first - p.first, q.second - p.second);

    point nan_point = std::make_pair(FLT_MAX, FLT_MAX);

    if (cross(r, s) == 0.0 && cross(a, r) == 0.0)  // colinear case
    {
        double t0 = dot(a, r) / dot(r, r);
        double t1 = t0 + dot(s, r) / dot(r, r);
        if (dot(s, r) >= 0.0)
        {
            if (t0 <= 0.0 && 0.0 <= t1) {
                return l11;
            }
            else {
                return nan_point;
            }
        }
        else {
            if (t1 <= 0.0 && 0.0 <= t0) {
                return l11;
            }
            else {
                return nan_point;
            }
        }
    }
    else if (cross(r, s) == 0.0 && cross(a, r) != 0.0) {  // parallel case
        return nan_point;
    }

    double t = cross(a, s) / cross(r, s);
    double u = cross(a, r) / cross(r, s);

    if (0.0 <= t && t <= 1.0 && 0.0 <= u && u <= 1.0) {
        point intersect_p = std::make_pair(p.first + t * r.first, p.second + t * r.second);
        return intersect_p;
    }
    else {
        return nan_point;
    }
}

double DodobotBumper::to_points_vector(vector<double>* input_x, vector<double>* input_y, vector<point>* output)
{
    ROS_ASSERT_MSG(input_x->size() == input_y->size(), "X and Y points are not equal in size!");
    output->resize(input_x->size());
    for (size_t i = 0; i < input_x->size(); i++) {
        (*output)[i] = std::make_pair(input_x->at(i), input_y->at(i));
    }
}

double DodobotBumper::get_scan_dist(point p)
{
    if (p.first == FLT_MAX || p.second == FLT_MAX) {
        return FLT_MAX;
    }
    else {
        return sqrt(pow(p.first, 2) + pow(p.second, 2));
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
