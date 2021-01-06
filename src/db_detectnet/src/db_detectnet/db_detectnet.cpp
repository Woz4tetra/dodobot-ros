#include <db_detectnet/db_detectnet.h>


DodobotDetectNet::DodobotDetectNet(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh)
{
    ros::param::param<string>("~model_name", _model_name, "ssd-mobilenet-v2");
    ros::param::param<string>("~model_path", _model_path, "");
    ros::param::param<string>("~prototxt_path", _prototxt_path, "");
    ros::param::param<string>("~class_labels_path", _class_labels_path, "");

    ros::param::param<string>("~input_blob", _input_blob, DETECTNET_DEFAULT_INPUT);
    ros::param::param<string>("~output_cvg", _output_cvg, DETECTNET_DEFAULT_COVERAGE);
    ros::param::param<string>("~output_bbox", _output_bbox, DETECTNET_DEFAULT_BBOX);
    ros::param::param<string>("~overlay_flags", _overlay_str, "box,labels,conf");

    ros::param::param<double>("~mean_pixel_value", _mean_pixel, 0.0);
    ros::param::param<double>("~threshold", _threshold, DETECTNET_DEFAULT_THRESHOLD);

    ros::param::param<string>("~depth_topic", _depth_topic, "depth/image_raw");
    ros::param::param<string>("~color_topic", _color_topic, "color/image_raw");
    ros::param::param<string>("~color_info_topic", _color_info_topic, "color/camera_info");
    ros::param::param<string>("~depth_info_topic", _depth_info_topic, "depth/camera_info");
    ros::param::param<int>("~bounding_box_border_px", _bounding_box_border_px, 30);
    ros::param::param<double>("~marker_persistance_s", _marker_persistance_s, 0.5);

    _marker_persistance = ros::Duration(_marker_persistance_s);
    _overlay_flags = detectNet::OverlayFlagsFromStr(_overlay_str.c_str());

    string key;
    if (!ros::param::search("detectnet_marker_colors", key)) {
        THROW_EXCEPTION("Failed to find detectnet_marker_colors parameter");
    }
    nh.getParam(key, _marker_colors_param);

    // _marker_colors_param is a map
    if (_marker_colors_param.getType() != XmlRpc::XmlRpcValue::Type::TypeStruct ||
        _marker_colors_param.size() == 0) {
        THROW_EXCEPTION("detectnet_marker_colors wrong type or size");
    }

    for (XmlRpc::XmlRpcValue::iterator it = _marker_colors_param.begin(); it != _marker_colors_param.end(); ++it)
    {
        if (it->second.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            THROW_EXCEPTION("detectnet_marker_colors element is not a list");
        }
        string label = it->first;

        std_msgs::ColorRGBA color;
        color.r = (double)(it->second[0]);
        color.g = (double)(it->second[1]);
        color.b = (double)(it->second[2]);
        color.a = (double)(it->second[3]);
        _marker_colors[label] = color;
        ROS_INFO("%s: R=%0.2f, G=%0.2f, B=%0.2f, A=%0.2f", label.c_str(), color.r, color.g, color.b, color.a);
    }

    if (_marker_colors.size() == 0) {
        THROW_EXCEPTION("detectnet_marker_colors has zero length!");
    }

    if (!ros::param::search("detectnet_z_depth_estimations", key)) {
        THROW_EXCEPTION("Failed to find detectnet_z_depth_estimations parameter");
    }
    nh.getParam(key, _z_depth_estimations);
    if (_z_depth_estimations.size() == 0) {
        THROW_EXCEPTION("detectnet_z_depth_estimations has zero length!");
    }

    load_detectnet_model();
    load_labels();

    reset_label_counter();

    // image converter objects
	_input_cvt = new imageConverter();
	_overlay_cvt = new imageConverter();

    // Publishers
    _detection_pub = nh.advertise<vision_msgs::Detection2DArray>("detections", 25);
    _marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 25);
    _overlay_pub = _image_transport.advertise("detect_overlay", 2);

    // Subscribers
    color_sub.subscribe(nh, _color_topic, 10);
    color_info_sub.subscribe(nh, _color_info_topic, 10);
    depth_sub.subscribe(nh, _depth_topic, 10);

    sync.reset(new Sync(ApproxSyncPolicy(10), color_sub, color_info_sub, depth_sub));
    // sync.reset(new Sync(ExactSyncPolicy(10), color_sub, color_info_sub, depth_sub));
    sync->registerCallback(boost::bind(&DodobotDetectNet::rgbd_callback, this, _1, _2, _3));

    ROS_INFO("db_detectnet init done");
}


void DodobotDetectNet::load_detectnet_model()
{
    /*
	 * load object detection network
	 */
    if (_model_path.size() > 0)
	{
		// create network using custom model paths
		_net = detectNet::Create(_prototxt_path.c_str(), _model_path.c_str(),
						    _mean_pixel, _class_labels_path.c_str(), _threshold,
						    _input_blob.c_str(), _output_cvg.c_str(), _output_bbox.c_str());
	}
	else
	{
		// determine which built-in model was requested
		detectNet::NetworkType model = detectNet::NetworkTypeFromStr(_model_name.c_str());

		if (model == detectNet::CUSTOM)
		{
			ROS_ERROR("invalid built-in pretrained model name '%s', defaulting to ssd-mobilenet-v2", _model_name.c_str());
			model = detectNet::SSD_MOBILENET_V2;
		}

		// create network using the built-in model
		_net = detectNet::Create(model, _threshold);
	}
}


void DodobotDetectNet::load_labels()
{
    /*
     * create the class labels parameter vector
     */
    std::hash<std::string> model_hasher;  // hash the model path to avoid collisions on the param server
    std::string model_hash_str = std::string(_net->GetModelPath()) + std::string(_net->GetClassPath());
    const size_t model_hash = model_hasher(model_hash_str);

    ROS_INFO("model hash => %zu", model_hash);
    ROS_INFO("hash string => %s", model_hash_str.c_str());

    // obtain the list of class descriptions
    _num_classes = _net->GetNumClasses();
    for (uint32_t n = 0; n < _num_classes; n++) {
        _class_descriptions.push_back(_net->GetClassDesc(n));
    }

    // create the key on the param server
    std::string class_key = std::string("class_labels_") + std::to_string(model_hash);

    if (!nh.hasParam(class_key)) {
        nh.setParam(class_key, _class_descriptions);
    }
}


int DodobotDetectNet::run()
{
    if (!_net)
	{
		ROS_ERROR("failed to load detectNet model");
		return 0;
	}

    if (!_input_cvt || !_overlay_cvt)
	{
		ROS_ERROR("failed to create imageConverter objects");
		return 0;
	}
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

    // free resources
    delete _net;
	delete _input_cvt;
	delete _overlay_cvt;

    return 0;
}

//
// Sub callbacks
//

void DodobotDetectNet::rgbd_callback(
    const ImageConstPtr& color_image, const CameraInfoConstPtr& color_info,
    const ImageConstPtr& depth_image)
{
    ros::Time t0 = ros::Time::now();

    // convert the image to reside on GPU
	if (!_input_cvt || !_input_cvt->Convert(color_image))
	{
		ROS_INFO("failed to convert %ux%u %s image", color_image->width, color_image->height, color_image->encoding.c_str());
		return;
	}

    //
    // Detect bounding boxes in color image
    //
    vision_msgs::Detection2DArray msg;
    const int num_detections = detect(color_image, &msg);
    if (num_detections <= 0) {
        return;
    }

    //
    // Get Z values of bounding boxes
    //
    _camera_model.fromCameraInfo(color_info);

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_image);  // encoding: passthrough
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth_cv_image;
    ROS_ASSERT_MSG(color_image->width != 0, "Color image width is zero!");
    ROS_ASSERT_MSG(color_image->height != 0, "Color image height is zero!");
    if (color_image->width != depth_image->width || color_image->height != depth_image->height) {
        cv::resize(cv_ptr->image, depth_cv_image, cv::Size(color_image->width, color_image->height));
    }
    else {
        depth_cv_image = cv_ptr->image;
    }


    visualization_msgs::MarkerArray markers;

    reset_label_counter();
    for (int n = 0; n < num_detections; n++)
    {
        int class_index = msg.detections[n].results[0].id;
        string label = _class_descriptions[class_index];
        int label_index = _label_counter[label];
        _label_counter[label]++;

        geometry_msgs::PoseWithCovariance pose_with_covar;

        ObjPoseDescription obj_desc = bbox_to_pose(depth_cv_image, msg.detections[n].bbox, depth_image->header.stamp, label, label_index);

        pose_with_covar.pose = obj_desc.pose_stamped.pose;
        msg.detections[n].results[0].pose = pose_with_covar;

        visualization_msgs::Marker sphere_marker = make_marker(&obj_desc);
        visualization_msgs::Marker text_marker = make_marker(&obj_desc);

        sphere_marker.type = visualization_msgs::Marker::SPHERE;
        sphere_marker.ns = "sphere_" + sphere_marker.ns;

        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.ns = "text_" + text_marker.ns;
        text_marker.text = label + "_" + std::to_string(label_index);
        text_marker.scale.x = 0.0;
        text_marker.scale.y = 0.0;

        markers.markers.push_back(sphere_marker);
        markers.markers.push_back(text_marker);
    }

    // populate timestamp in header field
    msg.header.stamp = color_image->header.stamp;

    // publish the detection message
    _detection_pub.publish(msg);
    _marker_pub.publish(markers);

    ros::Time now = ros::Time::now();
    ros::Duration dt_camera = now - color_image->header.stamp;
    ros::Duration dt_process = now - t0;

    ROS_DEBUG("Callback took %fs, %fs since image", dt_process.toSec(), dt_camera.toSec());
}

ObjPoseDescription DodobotDetectNet::bbox_to_pose(cv::Mat depth_cv_image, vision_msgs::BoundingBox2D bbox, ros::Time stamp, string label, int label_index)
{
    ObjPoseDescription desc = init_obj_desc();
    desc.label = label;
    desc.index = label_index;
    desc.center_x = (int)(bbox.center.x);
    desc.center_y = (int)(bbox.center.y);
    desc.obj_radius = std::min(bbox.size_x, bbox.size_y) / 2;
    desc.detect_radius = std::max(desc.obj_radius - _bounding_box_border_px, 1);
    ROS_DEBUG("bbox_to_pose for label %s", desc.label.c_str());

    double z_model = _z_depth_estimations[label];
    ROS_DEBUG("z_model: %f", z_model);

    double z_dist = get_z_dist(depth_cv_image, &desc);
    z_dist += z_model / 2.0;
    ROS_DEBUG("z_dist: %f", z_dist);

    cv::Point2d center_point;
    center_point.x = desc.center_x;
    center_point.y = desc.center_y;
    cv::Point3d ray = _camera_model.projectPixelTo3dRay(center_point);

    desc.x_dist = ray.x * z_dist;
    desc.y_dist = ray.y * z_dist;
    desc.z_dist = z_dist;

    ROS_DEBUG("center; label: %s, X: %0.3f, Y: %0.3f, Z: %0.3f", label.c_str(), desc.x_dist, desc.y_dist, desc.z_dist);

    cv::Point2d edge_point;
    edge_point.x = desc.center_x - (int)(bbox.size_x / 2.0);
    edge_point.y = desc.center_y - (int)(bbox.size_y / 2.0);
    ray = _camera_model.projectPixelTo3dRay(edge_point);

    desc.x_edge = abs(ray.x * z_dist - desc.x_dist) * 2.0;
    desc.y_edge = abs(ray.y * z_dist - desc.y_dist) * 2.0;
    desc.z_edge = z_model;

    ROS_DEBUG("edge;   label: %s, X: %0.3f, Y: %0.3f, Z: %0.3f", label.c_str(), desc.x_edge, desc.y_edge, desc.z_edge);

    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = _camera_model.tfFrame();
    pose.header.stamp = stamp;
    pose.pose.position.x = desc.x_dist;
    pose.pose.position.y = desc.y_dist;
    pose.pose.position.z = desc.z_dist;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    desc.pose_stamped = pose;

    return desc;
}

visualization_msgs::Marker DodobotDetectNet::make_marker(ObjPoseDescription* desc)
{
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = desc->pose_stamped.pose;
    marker.header = desc->pose_stamped.header;
    marker.lifetime = _marker_persistance;
    marker.ns = desc->label;
    marker.id = desc->index;

    geometry_msgs::Vector3 scale_vector;
    scale_vector.x = desc->x_edge;
    scale_vector.y = desc->y_edge;
    scale_vector.z = desc->z_edge;
    marker.scale = scale_vector;
    marker.color = _marker_colors[desc->label];

    return marker;
}


double DodobotDetectNet::get_z_dist(cv::Mat depth_cv_image, ObjPoseDescription* desc)
{
    ROS_DEBUG("Depth image size: w=%d, h=%d", depth_cv_image.rows, depth_cv_image.cols);
    cv::Mat circle_mask = cv::Mat::zeros(depth_cv_image.rows, depth_cv_image.cols, CV_8UC1);
    cv::circle(circle_mask, cv::Size(desc->center_x, desc->center_y), desc->detect_radius, cv::Scalar(255, 255, 255), cv::FILLED);
    ROS_DEBUG("Created circle mask");

    cv::Mat nonzero_mask = (depth_cv_image > 0.0);
    nonzero_mask.convertTo(nonzero_mask, CV_8U);
    ROS_DEBUG("Created nonzero mask");

    cv::Mat target_mask;
    cv::bitwise_and(circle_mask, nonzero_mask, target_mask);
    ROS_DEBUG("Created target mask");

    double z_dist = cv::mean(depth_cv_image, target_mask)[0];  // depth values are in mm
    ROS_DEBUG("z_dist mm: %f", z_dist);

    z_dist /= 1000.0;

    return z_dist;
}

// classify the image
int DodobotDetectNet::detect(const ImageConstPtr& color_image, vision_msgs::Detection2DArray* msg)
{
    detectNet::Detection* detections = NULL;

    ros::Time t0 = ros::Time::now();

    // was _input_cvt->ImageGPU(). Is now _input_cvt->ImageCPU().
    // Using the GPU stored image added ~0.03s of latency on Jetson Nano for some reason...
    const int num_detections = _net->Detect(_input_cvt->ImageCPU(), _input_cvt->GetWidth(), _input_cvt->GetHeight(), &detections, detectNet::OVERLAY_NONE);

    ros::Duration dt = ros::Time::now() - t0;
    ROS_DEBUG("Detect took %fs", dt.toSec());

	// verify success
	if (num_detections < 0)	{
		ROS_ERROR("failed to run object detection on %ux%u image", color_image->width, color_image->height);
	}

	// if objects were detected, update message
	if (num_detections > 0)
	{
		ROS_DEBUG("detected %i objects in %ux%u image", num_detections, color_image->width, color_image->height);

		// create a detection for each bounding box
		for (int n = 0; n < num_detections; n++)
		{
			detectNet::Detection* det = detections + n;

			ROS_DEBUG("object %i class #%u (%s)  confidence=%f", n, det->ClassID, _net->GetClassDesc(det->ClassID), det->Confidence);
			ROS_DEBUG("object %i bounding box (%f, %f)  (%f, %f)  w=%f  h=%f", n, det->Left, det->Top, det->Right, det->Bottom, det->Width(), det->Height());

			// create a detection sub-message
			vision_msgs::Detection2D detMsg;

			detMsg.bbox.size_x = det->Width();
			detMsg.bbox.size_y = det->Height();

			float cx, cy;
			det->Center(&cx, &cy);

			detMsg.bbox.center.x = cx;
			detMsg.bbox.center.y = cy;

			detMsg.bbox.center.theta = 0.0f;

			// create classification hypothesis
			vision_msgs::ObjectHypothesisWithPose hyp;

			hyp.id = det->ClassID;
			hyp.score = det->Confidence;

			detMsg.results.push_back(hyp);
			msg->detections.push_back(detMsg);
		}

        // generate the overlay (if there are subscribers)
    	if (_overlay_pub.getNumSubscribers() > 0) {
            publish_overlay(detections, num_detections);
        }
	}

    return num_detections;
}


void DodobotDetectNet::reset_label_counter()
{
    for (size_t i = 0; i < _class_descriptions.size(); i++) {
        _label_counter[_class_descriptions[i]] = 0;
    }
}

// publish overlay image
bool DodobotDetectNet::publish_overlay(detectNet::Detection* detections, int num_detections)
{
	// get the image dimensions
	const uint32_t width  = _input_cvt->GetWidth();
	const uint32_t height = _input_cvt->GetHeight();

	// assure correct image size
	if (!_overlay_cvt->Resize(width, height, imageConverter::ROSOutputFormat))
		return false;

	// generate the overlay
    // was _input_cvt->ImageGPU(). Is now _input_cvt->ImageCPU().
    // Using the GPU stored image added ~0.03s of latency on Jetson Nano for some reason...
	if (!_net->Overlay(_input_cvt->ImageCPU(), _overlay_cvt->ImageGPU(), width, height,
				   imageConverter::InternalFormat, detections, num_detections, _overlay_flags))
	{
		return false;
	}

	// populate the message
	sensor_msgs::Image msg;

	if( !_overlay_cvt->Convert(msg, imageConverter::ROSOutputFormat) )
		return false;

	// populate timestamp in header field
	msg.header.stamp = ros::Time::now();

	// publish the message
	_overlay_pub.publish(msg);
	ROS_DEBUG("publishing %ux%u overlay image", width, height);
}
