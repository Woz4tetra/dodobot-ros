#include <db_object_filter/db_object_filter.h>


DodobotObjectFilter::DodobotObjectFilter(ros::NodeHandle* nodehandle) :
nh(*nodehandle)
{
    ros::param::param<string>("~class_label_param", _class_label_param, "class_labels");
    string key;
    while (!ros::param::search(_class_label_param, key)) {
        if (!ros::ok()) {
            ROS_INFO_STREAM("db_object_filter interrupted while looking for " << _class_label_param);
            return;
        }
        ros::Duration(0.05).sleep();
    }

    ROS_INFO_STREAM("db_object_filter found the class list parameter: " << key);
    nh.getParam(key, _class_labels);

    _detection_sub = nh.subscribe<vision_msgs::Detection2DArray>("detections", 25, &DodobotObjectFilter::detections_callback, this);
    ROS_INFO("db_object_filter init done");
}

~DodobotObjectFilter::DodobotObjectFilter()
{
    delete sys_model;
    delete meas_model;
    delete filter;
}

void DodobotObjectFilter::create_particle_filter()
{
    /**************************
    * NonLinear system model *
    **************************/

    // create gaussian
    ColumnVector sys_noise_Mu(STATE_SIZE);
    sys_noise_Mu(1) = MU_SYSTEM_NOISE_X;
    sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y;
    sys_noise_Mu(3) = MU_SYSTEM_NOISE_THETA;

    SymmetricMatrix sys_noise_Cov(STATE_SIZE);
    sys_noise_Cov = 0.0;
    sys_noise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
    sys_noise_Cov(1,2) = 0.0;
    sys_noise_Cov(1,3) = 0.0;
    sys_noise_Cov(2,1) = 0.0;
    sys_noise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y;
    sys_noise_Cov(2,3) = 0.0;
    sys_noise_Cov(3,1) = 0.0;
    sys_noise_Cov(3,2) = 0.0;
    sys_noise_Cov(3,3) = SIGMA_SYSTEM_NOISE_THETA;

    Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

    // create the nonlinear system model
    sys_pdf = new NonlinearSystemPdf(system_Uncertainty);
    sys_model = new SystemModel<ColumnVector> (sys_pdf);


    /********************************
    * NonLinear Measurement model  *
    ********************************/


    // Construct the measurement noise (a scalar in this case)
    ColumnVector meas_noise_Mu(MEAS_SIZE);
    meas_noise_Mu(1) = MU_MEAS_NOISE;
    meas_noise_Mu(2) = MU_MEAS_NOISE;
    meas_noise_Mu(3) = MU_MEAS_NOISE;
    SymmetricMatrix meas_noise_Cov(MEAS_SIZE);
    meas_noise_Cov(1,1) = SIGMA_MEAS_NOISE;
    meas_noise_Cov(1,2) = 0.0;
    meas_noise_Cov(1,3) = 0.0;
    meas_noise_Cov(2,1) = 0.0;
    meas_noise_Cov(2,2) = SIGMA_MEAS_NOISE;
    meas_noise_Cov(2,3) = 0.0;
    meas_noise_Cov(3,1) = 0.0;
    meas_noise_Cov(3,2) = 0.0;
    meas_noise_Cov(3,3) = SIGMA_MEAS_NOISE;

    Gaussian measurement_Uncertainty(meas_noise_Mu, meas_noise_Cov);


    meas_pdf = new NonlinearMeasurementPdf(measurement_Uncertainty, map_);
    meas_model = new MeasurementModel<ColumnVector,ColumnVector>(meas_pdf);

    /****************************
    * Linear prior DENSITY     *
    ***************************/
    // Continuous Gaussian prior (for Kalman filters)
    ColumnVector prior_Mu(STATE_SIZE);
    prior_Mu(1) = PRIOR_MU_X;
    prior_Mu(2) = PRIOR_MU_Y;
    prior_Mu(3) = PRIOR_MU_THETA;
    SymmetricMatrix prior_Cov(STATE_SIZE);
    prior_Cov(1,1) = PRIOR_COV_X;
    prior_Cov(1,2) = 0.0;
    prior_Cov(1,3) = 0.0;
    prior_Cov(2,1) = 0.0;
    prior_Cov(2,2) = PRIOR_COV_Y;
    prior_Cov(2,3) = 0.0;
    prior_Cov(3,1) = 0.0;
    prior_Cov(3,2) = 0.0;
    prior_Cov(3,3) = PRIOR_COV_THETA;
    Gaussian prior_cont(prior_Mu,prior_Cov);

    // Discrete prior for Particle filter (using the continuous Gaussian prior)
    vector<Sample<ColumnVector> > prior_samples(NUM_SAMPLES);
    prior_discr = new MCPdf<ColumnVector>(NUM_SAMPLES,STATE_SIZE);
    prior_cont.SampleFrom(prior_samples,NUM_SAMPLES,CHOLESKY,NULL);
    prior_discr->ListOfSamplesSet(prior_samples);

    /******************************
    * Construction of the Filter *
    ******************************/
    filter = new CustomParticleFilter (prior_discr, 0.5, NUM_SAMPLES/4.0);
}

void DodobotObjectFilter::measurement_callback(geometry_msgs::Pose pose)
{
    ColumnVector measurement(3);
    measurement(1) = msg.range_front/100;
    measurement(2) = msg.range_left/100;
    measurement(3) = msg.range_right/100;
    //ROS_INFO("Measurement: %f",measurement(1));
    if (LastNavDataMsg.state==3 || LastNavDataMsg.state==7 || LastNavDataMsg.state==4)
    {
        filter->Update(meas_model, measurement);
        PublishParticles();
        PublishPose();
    }
}

void DodobotObjectFilter::input_callback(geometry_msgs::Twist twist)
{
    if (!prevNavDataTime.isZero())  {
        dt = (msg.header.stamp - prevNavDataTime).toSec();
    }
    prevNavDataTime = msg.header.stamp;
    LastNavDataMsg = msg;

    ColumnVector input(2);
    input(1) = msg.vx*dt*0.001;
    input(2) = msg.vy*dt*0.001;
    if (LastNavDataMsg.state==3 || LastNavDataMsg.state==7 || LastNavDataMsg.state==4) {
        filter->Update(sys_model,input);
    }
}

void publish_particles()
{
    geometry_msgs::PoseArray particles_msg;
    particles_msg.header.stamp = ros::Time::now();
    particles_msg.header.frame_id = "/map";

    vector<WeightedSample<ColumnVector> >::iterator sample_it;
    vector<WeightedSample<ColumnVector> > samples;

    samples = filter->getNewSamples();

    for(sample_it = samples.begin(); sample_it<samples.end(); sample_it++)
    {
        geometry_msgs::Pose pose;
        ColumnVector sample = (*sample_it).ValueGet();

        pose.position.x = sample(1);
        pose.position.y = sample(2);
        pose.orientation.z = sample(3);

        particles_msg.poses.insert(particles_msg.poses.begin(), pose);
    }
    particle_pub.publish(particles_msg);
}

void publish_pose()
{
    Pdf<ColumnVector> * posterior = filter->PostGet();
    ColumnVector pose = posterior->ExpectedValueGet();
    SymmetricMatrix pose_cov = posterior->CovarianceGet();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "/map";

    pose_msg.pose.position.x = pose(1);
    pose_msg.pose.position.y = pose(2);
    pose_msg.pose.orientation.z = pose(3);

    pose_pub.publish(pose_msg);
}


string DodobotObjectFilter::to_label(int index) {
    return _class_labels[index];
}

void DodobotObjectFilter::detections_callback(vision_msgs::Detection2DArray msg)
{
    for (size_t index = 0; index < msg.detections.size(); index++) {
        vision_msgs::ObjectHypothesisWithPose obj = msg.detections[index].results[0];
        ROS_INFO_STREAM("Found object: " << to_label(obj.id));
        geometry_msgs::Pose obj_pose = obj.pose.pose.pose;

    }
}

int DodobotObjectFilter::run()
{
    ros::spin();

    return 0;
}
