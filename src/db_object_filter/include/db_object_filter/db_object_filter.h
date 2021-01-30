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

    // Particle filter containers
    NonlinearSystemPdf *sys_pdf;
    SystemModel<ColumnVector> *sys_model;
    NonlinearMeasurementPdf *meas_pdf;
    MeasurementModel<ColumnVector,ColumnVector> *meas_model;
    MCPdf<ColumnVector> *prior_discr;
    ObjectParticleFilter *filter;

    // Particle filter parameters
    double MU_SYSTEM_NOISE_X, MU_SYSTEM_NOISE_Y, MU_SYSTEM_NOISE_Z;
    double SIGMA_SYSTEM_NOISE_X, SIGMA_SYSTEM_NOISE_Y, SIGMA_SYSTEM_NOISE_Z;
    double MU_MEAS_NOISE, SIGMA_MEAS_NOISE;
    int MEAS_SIZE;

    double PRIOR_MU_X, PRIOR_MU_Y, PRIOR_MU_Z;
    double PRIOR_COV_X, PRIOR_COV_Y, PRIOR_COV_Z;
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
