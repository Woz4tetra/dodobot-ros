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

    // Particle filter
    NonlinearSystemPdf *sys_pdf;
    SystemModel<ColumnVector> *sys_model;
    NonlinearMeasurementPdf *meas_pdf;
    MeasurementModel<ColumnVector,ColumnVector> *meas_model;
    MCPdf<ColumnVector> *prior_discr;
    ObjectParticleFilter *filter;
    ros::Time prevDataTime;
    double dt;

    void create_particle_filter();
    void measurement_callback(geometry_msgs::Pose pose);
    void input_callback(geometry_msgs::Twist twist);
    void publish_particles();
    void publish_pose();

    // Class labels
    string to_label(int index);

    // Subscribers
    ros::Subscriber _detection_sub;

    // Callbacks
    void detections_callback(vision_msgs::Detection2DArray msg);

    int run();
};

#endif  // _DODOBOT_OBJECT_FILTER_H_
