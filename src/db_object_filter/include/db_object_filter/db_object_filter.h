#ifndef _DODOBOT_OBJECT_FILTER_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"

#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>
#include <wrappers/rng/rng.h>
#include <filter/bootstrapfilter.h>
#include <model/systemmodel.h>
#include <model/measurementmodel.h>

#include "NonLinearMeasurementPdf.h"
#include "NonLinearSystemPdf.h"
#include "ObjectParticleFilter.h"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_msgs/Detection2DArray.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace std;
using namespace MatrixWrapper;
using namespace BFL;

#define THROW_EXCEPTION(msg)  throw std::runtime_error(msg)

class DodobotObjectFilter {
private:
    ros::NodeHandle nh;  // ROS node handle

public:
    DodobotObjectFilter(ros::NodeHandle* nodehandle);

    ~DodobotObjectFilter();

    // Launch parameters
    string _class_label_param;
    vector<string> _class_labels;
    string _filter_frame;
    vector<double> _sys_noise_cov_param;
    vector<double> _sys_noise_mu_param;
    vector<double> _meas_noise_mu_param;
    vector<double> _meas_noise_cov_param;
    vector<double> _prior_mu_param;
    vector<double> _prior_cov_param;

    void get_vector_param(string vector_name, vector<double> *param, int expected_size);

    // Particle filter containers
    NonlinearSystemPdf *sys_pdf;
    SystemModel<ColumnVector> *sys_model;
    NonlinearMeasurementPdf *meas_pdf;
    MeasurementModel<ColumnVector,ColumnVector> *meas_model;
    MCPdf<ColumnVector> *prior_discr;
    ObjectParticleFilter *filter;

    // Particle filter parameters
    int MEAS_SIZE;
    int STATE_SIZE;
    int NUM_SAMPLES;
    
    ros::Time prev_odom_time;

    void create_particle_filter();
    void publish_particles();
    void publish_pose();

    // Class labels
    string to_label(int index);

    // Subscribers
    ros::Subscriber _detection_sub;
    ros::Subscriber _odom_sub;

    // Publisher
    ros::Publisher _particle_pub;
    ros::Publisher _pose_pub;

    // Callbacks
    void detections_callback(vision_msgs::Detection2DArray msg);
    void odom_callback(nav_msgs::Odometry msg);

    int run();
};

#endif  // _DODOBOT_OBJECT_FILTER_H_
