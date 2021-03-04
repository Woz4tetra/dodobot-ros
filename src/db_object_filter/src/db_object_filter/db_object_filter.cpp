#include <db_object_filter/db_object_filter.h>


DodobotObjectFilter::DodobotObjectFilter(ros::NodeHandle* nodehandle) :
nh(*nodehandle)
{
    ros::param::param<string>("~class_label_param", _class_label_param, "class_labels");
    ros::param::param<string>("~filter_frame", _filter_frame, "base_link");
    string key;
    while (!ros::param::search(_class_label_param, key)) {
        if (!ros::ok()) {
            ROS_INFO_STREAM("db_object_filter interrupted while looking for " << _class_label_param);
            return;
        }
        ros::Duration(0.05).sleep();
    }

    STATE_SIZE = 3;
    MEAS_SIZE = 3;
    NUM_SAMPLES = 500;

    // if (!ros::param::search("~sys_noise_cov", key)) {
    //     THROW_EXCEPTION("Failed to find sys_noise_cov parameter");
    // }
    // nh.getParam(key, _sys_noise_cov_param);
    // if (_sys_noise_cov_param.size() != 9) {
    //     THROW_EXCEPTION("sys_noise_cov.size() != 9. %d", _sys_noise_cov_param.size());
    // }
    get_vector_param("sys_noise_mu", &_sys_noise_mu_param, STATE_SIZE);
    get_vector_param("sys_noise_cov", &_sys_noise_cov_param, STATE_SIZE * STATE_SIZE);

    get_vector_param("meas_noise_mu", &_meas_noise_mu_param, MEAS_SIZE);
    get_vector_param("meas_noise_cov", &_meas_noise_cov_param, MEAS_SIZE * MEAS_SIZE);

    get_vector_param("prior_mu", &_prior_mu_param, STATE_SIZE);
    get_vector_param("prior_cov", &_prior_cov_param, STATE_SIZE * STATE_SIZE);

    ROS_INFO_STREAM("db_object_filter found the class list parameter: " << key);
    nh.getParam(key, _class_labels);

    _detection_sub = nh.subscribe<vision_msgs::Detection2DArray>("detections", 25, &DodobotObjectFilter::detections_callback, this);
    _odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 25, &DodobotObjectFilter::odom_callback, this);

    _particle_pub = nh.advertise<geometry_msgs::PoseArray>("pf_particles", 25);
    _pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pf_pose", 25);

    create_particle_filter();

    ROS_INFO("db_object_filter init done");
}

DodobotObjectFilter::~DodobotObjectFilter()
{
    delete sys_model;
    delete meas_model;
    delete filter;
}

void DodobotObjectFilter::get_vector_param(string vector_name, vector<double> *param, int expected_size)
{
    string key;
    if (!ros::param::search(vector_name, key)) {
        ROS_ERROR_STREAM("Failed to find " << vector_name << " parameter");
        THROW_EXCEPTION("Failed to find key");
    }
    nh.getParam(key, *param);
    if (param->size() != expected_size) {
        ROS_ERROR_STREAM(vector_name << " size !=  " << expected_size << ". " << param->size());
        THROW_EXCEPTION("Vector was unexpected size");
    }
}


void DodobotObjectFilter::create_particle_filter()
{
    /**************************
    * NonLinear system model *
    **************************/

    // create gaussian
    ColumnVector sys_noise_Mu(STATE_SIZE);
    sys_noise_Mu(1) = _sys_noise_mu_param[0];  // X
    sys_noise_Mu(2) = _sys_noise_mu_param[1];  // Y
    sys_noise_Mu(3) = _sys_noise_mu_param[2];  // Z

    SymmetricMatrix sys_noise_Cov(STATE_SIZE);
    sys_noise_Cov = 0.0;
    sys_noise_Cov(1,1) = _sys_noise_cov_param[0];
    sys_noise_Cov(1,2) = _sys_noise_cov_param[1];
    sys_noise_Cov(1,3) = _sys_noise_cov_param[2];
    sys_noise_Cov(2,1) = _sys_noise_cov_param[3];
    sys_noise_Cov(2,2) = _sys_noise_cov_param[4];
    sys_noise_Cov(2,3) = _sys_noise_cov_param[5];
    sys_noise_Cov(3,1) = _sys_noise_cov_param[6];
    sys_noise_Cov(3,2) = _sys_noise_cov_param[7];
    sys_noise_Cov(3,3) = _sys_noise_cov_param[8];

    Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

    // create the nonlinear system model
    sys_pdf = new NonlinearSystemPdf(system_Uncertainty);
    sys_model = new SystemModel<ColumnVector>(sys_pdf);


    /********************************
    * NonLinear Measurement model  *
    ********************************/


    // Construct the measurement noise (a scalar in this case)
    ColumnVector meas_noise_Mu(MEAS_SIZE);
    meas_noise_Mu(1) = _meas_noise_mu_param[0];  // X
    meas_noise_Mu(2) = _meas_noise_mu_param[1];  // Y
    meas_noise_Mu(3) = _meas_noise_mu_param[2];  // Z

    SymmetricMatrix meas_noise_Cov(MEAS_SIZE);
    meas_noise_Cov(1,1) = _meas_noise_cov_param[0];
    meas_noise_Cov(1,2) = _meas_noise_cov_param[1];
    meas_noise_Cov(1,3) = _meas_noise_cov_param[2];
    meas_noise_Cov(2,1) = _meas_noise_cov_param[3];
    meas_noise_Cov(2,2) = _meas_noise_cov_param[4];
    meas_noise_Cov(2,3) = _meas_noise_cov_param[5];
    meas_noise_Cov(3,1) = _meas_noise_cov_param[6];
    meas_noise_Cov(3,2) = _meas_noise_cov_param[7];
    meas_noise_Cov(3,3) = _meas_noise_cov_param[8];

    Gaussian measurement_Uncertainty(meas_noise_Mu, meas_noise_Cov);


    meas_pdf = new NonlinearMeasurementPdf(measurement_Uncertainty);
    meas_model = new MeasurementModel<ColumnVector,ColumnVector>(meas_pdf);

    /***************************
     * Linear prior DENSITY    *
     ***************************/
    // Continuous Gaussian prior (for Kalman filters)
    ColumnVector prior_Mu(STATE_SIZE);
    prior_Mu(1) = _prior_mu_param[0];
    prior_Mu(2) = _prior_mu_param[1];
    prior_Mu(3) = _prior_mu_param[2];

    SymmetricMatrix prior_Cov(STATE_SIZE);
    prior_Cov(1,1) = _prior_cov_param[0];
    prior_Cov(1,2) = _prior_cov_param[1];
    prior_Cov(1,3) = _prior_cov_param[2];
    prior_Cov(2,1) = _prior_cov_param[3];
    prior_Cov(2,2) = _prior_cov_param[4];
    prior_Cov(2,3) = _prior_cov_param[5];
    prior_Cov(3,1) = _prior_cov_param[6];
    prior_Cov(3,2) = _prior_cov_param[7];
    prior_Cov(3,3) = _prior_cov_param[8];

    Gaussian prior_cont(prior_Mu, prior_Cov);

    // Discrete prior for Particle filter (using the continuous Gaussian prior)
    vector<Sample<ColumnVector> > prior_samples(NUM_SAMPLES);
    prior_discr = new MCPdf<ColumnVector>(NUM_SAMPLES, STATE_SIZE);
    prior_cont.SampleFrom(prior_samples, NUM_SAMPLES, CHOLESKY, NULL);
    prior_discr->ListOfSamplesSet(prior_samples);

    /******************************
    * Construction of the Filter *
    ******************************/
    filter = new ObjectParticleFilter(prior_discr, 0.5, NUM_SAMPLES / 4.0);

    ROS_INFO("Particle filter initialized");
}

void DodobotObjectFilter::publish_particles()
{
    geometry_msgs::PoseArray particles_msg;
    particles_msg.header.stamp = ros::Time::now();
    particles_msg.header.frame_id = _filter_frame;

    vector<WeightedSample<ColumnVector> >::iterator sample_it;
    vector<WeightedSample<ColumnVector> > samples;

    samples = filter->getNewSamples();

    for (sample_it = samples.begin(); sample_it < samples.end(); sample_it++)
    {
        geometry_msgs::Pose pose;
        ColumnVector sample = (*sample_it).ValueGet();

        pose.position.x = sample(1);
        pose.position.y = sample(2);
        pose.position.z = sample(3);

        particles_msg.poses.insert(particles_msg.poses.begin(), pose);
    }
    _particle_pub.publish(particles_msg);
}

void DodobotObjectFilter::publish_pose()
{
    Pdf<ColumnVector> * posterior = filter->PostGet();
    ColumnVector pose = posterior->ExpectedValueGet();
    SymmetricMatrix pose_cov = posterior->CovarianceGet();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = _filter_frame;

    pose_msg.pose.position.x = pose(1);
    pose_msg.pose.position.y = pose(2);
    pose_msg.pose.position.z = pose(3);

    _pose_pub.publish(pose_msg);
}


string DodobotObjectFilter::to_label(int index) {
    return _class_labels[index];
}

void DodobotObjectFilter::detections_callback(vision_msgs::Detection2DArray msg)
{
    for (size_t index = 0; index < msg.detections.size(); index++)
    {
        vision_msgs::ObjectHypothesisWithPose obj = msg.detections[index].results[0];
        ROS_INFO_STREAM("Found object: " << to_label(obj.id));
        if (!to_label(obj.id).compare("blue_cut_sphere")) {
            // TODO: add more object types
            continue;
        }
        geometry_msgs::Pose obj_pose = obj.pose.pose;

        ColumnVector measurement(3);
        measurement(1) = obj_pose.position.x;
        measurement(2) = obj_pose.position.y;
        measurement(3) = obj_pose.position.z;
        filter->Update(meas_model, measurement);
        publish_particles();
        publish_pose();
    }
}

void DodobotObjectFilter::odom_callback(nav_msgs::Odometry msg)
{
    double dt = (msg.header.stamp - prev_odom_time).toSec();
    if (dt == 0.0) {
        return;
    }
    prev_odom_time = msg.header.stamp;

    tf2::Transform rotate_tf, odom_velocity, base_link_velocity;

    // odometry velocities are in base_link frame

    /*
    // convert odometry velocities to be in the base_link frame
    geometry_msgs::Vector3 odom_vel_vector;
    odom_vel_vector.x = -msg.twist.twist.linear.x;
    odom_vel_vector.y = -msg.twist.twist.linear.y;
    odom_vel_vector.z = -msg.twist.twist.linear.z;
    odom_velocity.setOrigin(odom_vel_vector);

    tf2::Quaternion rotate_quat, flip_quat, orig_quat;
    tf2::convert(msg.pose.pose.orientation, orig_quat);
    flip_quat.setRPY(0.0, 0.0, M_PI);  // invert rotation angle
    rotate_quat = flip_quat * orig_quat;
    rotate_quat.normalize();
    rotate_tf.setRotation(rotate_quat);

    base_link_velocity = rotate_tf * odom_velocity;
    
    geometry_msgs::Vector3 base_vel_vector;
    base_vel_vector = base_link_velocity.getOrigin();
    */

    ColumnVector input(2);
    // input(1) = base_vel_vector.x;
    input(1) = msg.twist.twist.linear.x;
    input(2) = msg.twist.twist.angular.z;
    filter->Update(sys_model, input);
}


int DodobotObjectFilter::run()
{
    ros::spin();

    return 0;
}
