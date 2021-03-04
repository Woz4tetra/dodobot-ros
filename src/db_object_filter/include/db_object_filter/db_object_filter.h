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

#include <vision_msgs/Detection2DArray.h>
#include <nav_msgs/Odometry.h>


using namespace std;
using namespace MatrixWrapper;
using namespace BFL;

#define GET_VECTOR_PARAM(name, expected_length)  if (!ros::param::search("~##name", key)) { \
        THROW_EXCEPTION("Failed to find ##name parameter"); \
    } \
    nh.getParam(key, _##name_param); \
    if (_##name_param.size() != expected_length) { \
        THROW_EXCEPTION("##name.size() != ##expected_length. %d", _##name_param.size()); \
    }

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
    vector<double> _meas_noise_cov_param;
    vector<double> _prior_mu_param;
    vector<double> _prior_cov_param;

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

    // Callbacks
    void detections_callback(vision_msgs::Detection2DArray msg);
    void odom_callback(nav_msgs::Odometry msg);

    int run();
};

#endif  // _DODOBOT_OBJECT_FILTER_H_
