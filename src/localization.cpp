#include "localization.h"
#include "SLAM.h"
#include "klt_point_handling.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>

static const int DUO_QUEUE_SIZE = 30;

Localization::Localization()
: nh_("~"),
  SLAM_reset_flag(1),
  change_reference(false),
  vicon_pos(3, 0.0),
  vicon_quaternion(4, 0.0),
  max_clicks_(0),
  clear_queue_counter(0),
  vio_cnt(0),
  image_visualization_delay(0)
{
	emxInitArray_real_T(&h_u_apo,1);
	emxInitArray_real_T(&map,2);
	emxInitArray_AnchorPose(&anchor_poses, 2);
	SLAM_initialize();

	// initialize structs
	referenceCommand = {{0, 0, 0, 0}, {0, 0, 0, 0}};
	cameraParams = {{},{}};
	noiseParams = {};
	controllerGains = {};
	vioParams = {};

	duo_sub = nh_.subscribe("/duo3d_camera/combined", DUO_QUEUE_SIZE, &Localization::duo3dCb,this);
	joy_sub_ = nh_.subscribe("/joy",1, &Localization::joystickCb, this);
	position_reference_sub_ = nh_.subscribe("/onboard_localization/position_reference",1, &Localization::positionReferenceCb, this);

	controller_pub = nh_.advertise<onboard_localization::ControllerOut>("/onboard_localization/controller_output",10);

	// visualization topics
	vio_vis_pub = nh_.advertise<vio_ros::vio_vis>("/vio_vis/vio_vis", 1);
	vio_vis_reset_pub = nh_.advertise<std_msgs::Empty>("/vio_vis/reset", 1);
	ros::Duration(0.5).sleep(); // otherwise the following message might not be received
	vio_vis_reset_pub.publish(std_msgs::Empty());

	duo_processed_pub = nh_.advertise<std_msgs::UInt32>("/duo3d/msg_processed", 1);

	// Load parameters from launch file
	if(!nh_.getParam("noise_acc", noiseParams.process_noise.qv))
		ROS_WARN("Failed to load parameter noise_acc");
	if(!nh_.getParam("noise_gyro", noiseParams.process_noise.qw))
		ROS_WARN("Failed to load parameter noise_gyro");
	if(!nh_.getParam("noise_gyro_bias", noiseParams.process_noise.qwo))
		ROS_WARN("Failed to load parameter noise_gyro_bias");
	if(!nh_.getParam("noise_acc_bias", noiseParams.process_noise.qao))
		ROS_WARN("Failed to load parameter noise_acc_bias");
	if(!nh_.getParam("noise_R_ci", noiseParams.process_noise.qR_ci))
		ROS_WARN("Failed to load parameter noise_R_ci");
	if(!nh_.getParam("noise_inv_depth_initial_unc", noiseParams.inv_depth_initial_unc))
		ROS_WARN("Failed to load parameter noise_inv_depth_initial_unc");
	if(!nh_.getParam("noise_image", noiseParams.image_noise))
		ROS_WARN("Failed to load parameter noise_image");

	std::vector<double> tmp;
	if (nh_.getParam("noise_gyro_bias_initial_unc", tmp))
	{
		for (int i = 0; i < tmp.size(); i++)
			noiseParams.gyro_bias_initial_unc[i] = tmp[i];
	} else {
		ROS_WARN("Failed to load parameter noise_gyro_bias_initial_unc");
	}
	if (nh_.getParam("noise_acc_bias_initial_unc", tmp))
	{
		for (int i = 0; i < tmp.size(); i++)
			noiseParams.acc_bias_initial_unc[i] = tmp[i];
	} else {
		ROS_WARN("Failed to load parameter noise_acc_bias_initial_unc");
	}

	if(!nh_.getParam("vio_num_points_per_anchor", vioParams.num_points_per_anchor))
		ROS_WARN("Failed to load parameter vio_num_points_per_anchor");
	if(!nh_.getParam ("vio_num_anchors", vioParams.num_anchors))
		ROS_WARN("Failed to load parameter vio_num_anchors");
	if(!nh_.getParam("vio_max_ekf_iterations", vioParams.max_ekf_iterations))
		ROS_WARN("Failed to load parameter vio_max_ekf_iterations");
	if(!nh_.getParam("vio_delayed_initiazation", vioParams.delayed_initialization))
		ROS_WARN("Failed to load parameter vio_delayed_initiazation");
	if(!nh_.getParam("vio_mono", vioParams.mono))
		ROS_WARN("Failed to load parameter vio_mono");
	if(!nh_.getParam("vio_fixed_feature", vioParams.fixed_feature))
		ROS_WARN("Failed to load parameter vio_fixed_feature");
	if(!nh_.getParam("vio_RANSAC", vioParams.RANSAC))
		ROS_WARN("Failed to load parameter vio_RANSAC");

	if(!nh_.getParam("ctrl_Kp_xy", controllerGains.Kp_xy))
		ROS_WARN("Failed to load parameter ctrl_Kp_xy");
	if(!nh_.getParam("ctrl_Ki_xy", controllerGains.Ki_xy))
		ROS_WARN("Failed to load parameter ctrl_Ki_xy");
	if(!nh_.getParam("ctrl_Kd_xy", controllerGains.Kd_xy))
		ROS_WARN("Failed to load parameter ctrl_Kd_xy");
	if(!nh_.getParam("ctrl_Kp_z", controllerGains.Kp_xy))
		ROS_WARN("Failed to load parameter ctrl_Kp_z");
	if(!nh_.getParam("ctrl_Ki_z", controllerGains.Ki_z))
		ROS_WARN("Failed to load parameter ctrl_Ki_z");
	if(!nh_.getParam("ctrl_Kd_z", controllerGains.Kd_z))
		ROS_WARN("Failed to load parameter ctrl_Kd_z");
	if(!nh_.getParam("ctrl_Kp_yaw", controllerGains.Kp_yaw))
		ROS_WARN("Failed to load parameter ctrl_Kp_yaw");
	if(!nh_.getParam("ctrl_Kd_yaw", controllerGains.Kd_yaw))
		ROS_WARN("Failed to load parameter ctrl_Kd_yaw");
	if(!nh_.getParam("ctrl_i_lim", controllerGains.i_lim))
		ROS_WARN("Failed to load parameter ctrl_i_lim");

	std::string camera_name; nh_.param<std::string>("cam_camera_name", camera_name, "NoName");
	std::string lense_type; nh_.param<std::string>("cam_lense_type", lense_type, "NoType");
	int resolution_width; nh_.param<int>("cam_resolution_width", resolution_width, 0);
	int resolution_height; nh_.param<int>("cam_resolution_height", resolution_height, 0);

	std::stringstream res; res << resolution_height << "x" << resolution_width;
	std::string calib_path = ros::package::getPath("vio_ros") + "/calib/" + camera_name + "/" + lense_type + "/" + res.str() + "/cameraParams.yaml";

	ROS_INFO("Reading camera calibration from %s", calib_path.c_str());

	try {
		YAML::Node YamlNode = YAML::LoadFile(calib_path);
		if (YamlNode.IsNull())
		{
			ROS_FATAL("Failed to open camera calibration %s", calib_path.c_str());
			exit(-1);
		}
		cameraParams = parseYaml(YamlNode);
	} catch (YAML::BadFile &e) {
		ROS_FATAL("Failed to open camera calibration %s\nException: %s", calib_path.c_str(), e.what());
		exit(-1);
	}

	if(!nh_.getParam("cam_FPS_duo", fps_duo))
		ROS_WARN("Failed to load parameter cam_FPS_duo");
	if(!nh_.getParam("cam_vision_subsample", vision_subsample))
		ROS_WARN("Failed to load parameter cam_vision_subsample");

	double visualization_freq;
	if(!nh_.getParam("visualization_freq", visualization_freq))
		ROS_WARN("Failed to load parameter visualization_freq");
	vis_publish_delay = fps_duo/vision_subsample/visualization_freq;
	vis_publish_delay = !vis_publish_delay ? 1 : vis_publish_delay;
	if(!nh_.getParam("show_camera_image", show_camera_image_))
		ROS_WARN("Failed to load parameter show_camera_image");
	if(!nh_.getParam("image_visualization_delay", image_visualization_delay))
		ROS_WARN("Failed to load parameter image_visualization_delay");
	image_visualization_delay = !image_visualization_delay ? 1 : image_visualization_delay;

	std::string dark_current_l_path = ros::package::getPath("vio_ros") + "/calib/" + camera_name + "/" + lense_type + "/" + res.str() + "/darkCurrentL.bmp";
	darkCurrentL = cv::imread(dark_current_l_path, CV_LOAD_IMAGE_GRAYSCALE);

	if (!darkCurrentL.data)
	{
		ROS_WARN("Failed to open left dark current image %s!", dark_current_l_path.c_str());
		use_dark_current = false;
	} else if (darkCurrentL.rows != resolution_height || darkCurrentL.cols != resolution_width)
	{
		ROS_WARN("Left dark current image has the wrong dimensions %s!", dark_current_l_path.c_str());
		use_dark_current = false;
	}

	if (use_dark_current)
	{
		std::string dark_current_r_path = ros::package::getPath("vio_ros") + "/calib/" + camera_name + "/" + lense_type + "/" + res.str() + "/darkCurrentR.bmp";
		darkCurrentR = cv::imread(dark_current_r_path, CV_LOAD_IMAGE_GRAYSCALE);
		if (!darkCurrentR.data)
		{
			ROS_WARN("Failed to open right dark current image %s!", dark_current_r_path.c_str());
			use_dark_current = false;
		} else if (darkCurrentR.rows != resolution_height || darkCurrentR.cols != resolution_width)
		{
			ROS_WARN("Right dark current image has the wrong dimensions %s!", dark_current_r_path.c_str());
			use_dark_current = false;
		}
	}

	dynamic_reconfigure::Server<vio_ros::vio_rosConfig>::CallbackType f = boost::bind(&Localization::dynamicReconfigureCb, this, _1, _2);
	dynamic_reconfigure_server.setCallback(f);

	num_points_ = vioParams.num_anchors*vioParams.num_points_per_anchor;

	update_vec_.assign(num_points_, 0);

}

Localization::~Localization()
{
	emxDestroyArray_real_T(h_u_apo);
	emxDestroyArray_real_T(map);
	emxDestroyArray_AnchorPose(anchor_poses);
	SLAM_terminate();

	printf("Longest update duration: %.3f msec, %.3f Hz\n", float(max_clicks_)/CLOCKS_PER_SEC, CLOCKS_PER_SEC/float(max_clicks_));
}

void Localization::duo3dCb(const duo3d_ros::Duo3d& msg)
{
//	ROS_INFO("Received message %d", msg.header.seq);
	// upon reset, catch up with the duo messages before resetting SLAM
	if (SLAM_reset_flag)
	{
		if(clear_queue_counter < DUO_QUEUE_SIZE)
		{
			clear_queue_counter++;
			std_msgs::UInt32 id_msg;
			id_msg.data = msg.header.seq;
			duo_processed_pub.publish( id_msg);
			return;
		} else {
			clear_queue_counter = 0;
			SLAM_reset_flag = false;
			std_msgs::UInt32 id_msg;
			id_msg.data = msg.header.seq;
			duo_processed_pub.publish( id_msg);
			return;
		}
	}

	ros::Time tic_total = ros::Time::now();
	clock_t tic_total_clock = clock();
	double dt;
	// Init time on first call
	if (prev_time_.isZero())
	{
		prev_time_ = msg.header.stamp;
		dt = vision_subsample/fps_duo;
	} else {
		dt = (msg.header.stamp - prev_time_).toSec();
		prev_time_ = msg.header.stamp;
	}

	bool vis_publish = (vio_cnt % vis_publish_delay) == 0;
	bool show_image = false;

	update(dt, msg, vis_publish, show_image);

	double time_measurement = (ros::Time::now() - tic_total).toSec();

	if (0*vis_publish || time_measurement > vision_subsample/fps_duo)
	{
		if (time_measurement > vision_subsample/fps_duo)
			ROS_WARN("Duration: %f ms. Theoretical max frequency: %.3f Hz\n", time_measurement, 1/time_measurement);
		else
			ROS_INFO("Duration: %f ms. Theoretical max frequency: %.3f Hz\n", time_measurement, 1/time_measurement);
	}
	clock_t toc_total_clock = clock();

	if (toc_total_clock - tic_total_clock > max_clicks_)
		max_clicks_ = toc_total_clock - tic_total_clock;

	std_msgs::UInt32 id_msg;
	id_msg.data = msg.header.seq;
	duo_processed_pub.publish( id_msg);
}

void Localization::joystickCb(const sensor_msgs::Joy::ConstPtr& msg)
{
	if (msg->buttons[0] && !SLAM_reset_flag)
	{
		SLAM_reset_flag = true;
		referenceCommand.position[0] = 0;
		referenceCommand.position[1] = 0;
		referenceCommand.position[2] = 0;
		referenceCommand.position[3] = 0;
		referenceCommand.velocity[0] = 0;
		referenceCommand.velocity[1] = 0;
		referenceCommand.velocity[2] = 0;
		referenceCommand.velocity[3] = 0;

		tf::Quaternion quaternion_yaw;
		tf::Transform tf_yaw;
		tf_yaw.setRotation(quaternion_yaw);
		tf::Matrix3x3 rotation_yaw = tf_yaw.getBasis();
		double roll, pitch, yaw;
		rotation_yaw.getRPY(roll, pitch, yaw);
		referenceCommand.position[3] = yaw;

	    geometry_msgs::PoseStamped ref_viz;
	    ref_viz.header.stamp = ros::Time::now();
	    ref_viz.header.frame_id = "world";
	    ref_viz.pose.position.x = referenceCommand.position[0];
	    ref_viz.pose.position.y = referenceCommand.position[1];
	    ref_viz.pose.position.z = referenceCommand.position[2];

	    tf::Quaternion quaternion;
	    quaternion.setRPY(0.0, 0.0, referenceCommand.position[3]);
	    ref_viz.pose.orientation.w = quaternion.getW();
	    ref_viz.pose.orientation.x = quaternion.getX();
	    ref_viz.pose.orientation.y = quaternion.getY();
	    ref_viz.pose.orientation.z = quaternion.getZ();

//	    reference_viz_pub.publish(ref_viz);

	    if (!SLAM_reset_flag)
	    	ROS_INFO("resetting SLAM");

	} else if (msg->buttons[2]) { // auto mode signal
		change_reference = true;
		// set the reference to the current pose

		referenceCommand.position[0] = pose.position.x;
		referenceCommand.position[1] = pose.position.y;
		referenceCommand.position[2] = pose.position.z;

		double yaw = tf::getYaw(pose.orientation) + 1.57;
		referenceCommand.position[3] = yaw;

		referenceCommand.velocity[0] = 0;
		referenceCommand.velocity[1] = 0;
		referenceCommand.velocity[2] = 0;
		referenceCommand.velocity[3] = 0;

		geometry_msgs::PoseStamped ref_viz;
		ref_viz.header.stamp = ros::Time::now();
		ref_viz.header.frame_id = "world";
		ref_viz.pose.position.x = referenceCommand.position[0];
		ref_viz.pose.position.y = referenceCommand.position[1];
		ref_viz.pose.position.z = referenceCommand.position[2];

		tf::Quaternion quaternion;
		quaternion.setRPY(0.0, 0.0, referenceCommand.position[3]);
		ref_viz.pose.orientation.w = quaternion.getW();
		ref_viz.pose.orientation.x = quaternion.getX();
		ref_viz.pose.orientation.y = quaternion.getY();
		ref_viz.pose.orientation.z = quaternion.getZ();

	} else if (msg->buttons[3]) { // leaving auto mode signal
		change_reference = true;
	}
}

void Localization::dynamicReconfigureCb(vio_ros::vio_rosConfig &config, uint32_t level)
{
	controllerGains.Kp_xy  = config.ctrl_Kp_xy;
	controllerGains.Ki_xy  = config.ctrl_Ki_xy;
	controllerGains.Kd_xy  = config.ctrl_Kd_xy;
	controllerGains.Kp_z   = config.ctrl_Kp_z;
	controllerGains.Ki_z   = config.ctrl_Ki_z;
	controllerGains.Kd_z   = config.ctrl_Kd_z;
	controllerGains.Kp_yaw = config.ctrl_Kp_yaw;
	controllerGains.Kd_yaw = config.ctrl_Kd_yaw;
	controllerGains.i_lim  = config.ctrl_i_lim;

	noiseParams.image_noise           = config.noise_image;
	noiseParams.process_noise.qv      = config.noise_acc;
	noiseParams.process_noise.qw      = config.noise_gyro;
	noiseParams.process_noise.qwo     = config.noise_gyro_bias;
	noiseParams.process_noise.qao     = config.noise_acc_bias;
	noiseParams.inv_depth_initial_unc = config.noise_inv_depth_initial_unc;

	vioParams.fixed_feature          = config.vio_fixed_feature;
	vioParams.max_ekf_iterations     = config.vio_max_ekf_iterations;
	vioParams.delayed_initialization = config.vio_delayed_initialization;
	vioParams.mono                   = config.vio_mono;
	vioParams.RANSAC                 = config.vio_RANSAC;

//	show_camera_image_ = config.show_tracker_images;

}

void Localization::positionReferenceCb(const onboard_localization::PositionReference& msg)
{
	if (change_reference)
	{
		double roll, pitch, yaw;
		tf::Matrix3x3(camera2world).getRPY(roll, pitch, yaw);
		tf::Quaternion q;
		q.setRPY(0, 0, yaw + 1.57);
		tf::Vector3 positionChange_world = tf::Transform(q) * tf::Vector3(msg.x, msg.y, msg.z);
		double dt = 0.1; // the loop rate of the joy reference node
		referenceCommand.position[0] += dt * positionChange_world.x();
		referenceCommand.position[1] += dt * positionChange_world.y();
		referenceCommand.position[2] += dt * positionChange_world.z();
		referenceCommand.position[3] += dt * msg.yaw;

		referenceCommand.velocity[0] = positionChange_world.x();
		referenceCommand.velocity[1] = positionChange_world.y();
		referenceCommand.velocity[2] = positionChange_world.z();
		referenceCommand.velocity[3] = msg.yaw;

		geometry_msgs::PoseStamped ref_viz;
		ref_viz.header.stamp = ros::Time::now();
		ref_viz.header.frame_id = "world";
		ref_viz.pose.position.x = referenceCommand.position[0];
		ref_viz.pose.position.y = referenceCommand.position[1];
		ref_viz.pose.position.z = referenceCommand.position[2];

		tf::Quaternion quaternion;
		quaternion.setRPY(0.0, 0.0, referenceCommand.position[3]);
		ref_viz.pose.orientation.w = quaternion.getW();
		ref_viz.pose.orientation.x = quaternion.getX();
		ref_viz.pose.orientation.y = quaternion.getY();
		ref_viz.pose.orientation.z = quaternion.getZ();

//		reference_viz_pub.publish(ref_viz);
	}
}

void Localization::update(double dt, const duo3d_ros::Duo3d &msg, bool update_vis, bool show_image)
{
	std::vector<double> z_all_l(num_points_*2, 0.0);
	std::vector<double> z_all_r(num_points_*2, 0.0);
	VIOMeasurements meas;
	getIMUData(msg.imu, meas);

	double u_out[4];
	double delayedStatus[40];

	//*********************************************************************
	// SLAM prediction
	//*********************************************************************

	SLAM(&update_vec_[0],
			&z_all_l[0],
			&z_all_r[0],
			dt,
			&meas,
			&cameraParams,
			&noiseParams,
			&vioParams,
			0, // predict
			&robot_state,
			h_u_apo,
			map,
			anchor_poses,
			delayedStatus);

	if (vio_cnt % vision_subsample == 0)
	{
		cv_bridge::CvImagePtr left_image;
		cv_bridge::CvImagePtr right_image;
		try
		{
			left_image = cv_bridge::toCvCopy(msg.left_image, "mono8");
			right_image = cv_bridge::toCvCopy(msg.right_image,"mono8");
		}
		catch(cv_bridge::Exception& e)
		{
			ROS_ERROR("Error while converting ROS image to OpenCV: %s", e.what());
			return;
		}

		if(left_image->image.empty() || right_image->image.empty())
		{
			return;
		}
		//*********************************************************************
		// Point tracking
		//*********************************************************************

		ros::Time tic = ros::Time::now();
		ros::Time tic_total = tic;

		//	clock_t bef = clock();
		cv::Mat left, right;
		if (use_dark_current)
		{
			left = left_image->image - darkCurrentL;
			right = right_image->image - darkCurrentR;
		} else {
			left = left_image->image;
			right = right_image->image;
		}
		handle_points_klt(left, right, z_all_l, z_all_r, update_vec_);

		//	clock_t aft = clock();
		//	printf("KLT  took %d clicks, %.3f msec\n", int(aft - bef), 1000*float(aft - bef)/CLOCKS_PER_SEC);

		//*********************************************************************
		// SLAM update
		//*********************************************************************
		SLAM(&update_vec_[0],
				&z_all_l[0],
				&z_all_r[0],
				dt,
				&meas,
				&cameraParams,
				&noiseParams,
				&vioParams,
				1, // vision update
				&robot_state,
				h_u_apo,
				map,
				anchor_poses,
				delayedStatus);

		if (update_vis)
		{
			show_image = display_tracks_cnt % image_visualization_delay == 0;
			display_tracks_cnt++;

			updateVis(robot_state, anchor_poses, map->data, update_vec_, msg, z_all_l, show_image);
		}
	}

	//ROS_INFO("Time SLAM         : %6.2f ms", (ros::Time::now() - tic).toSec()*1000);
	vio_cnt++;
}

void Localization::getIMUData(const sensor_msgs::Imu& imu, VIOMeasurements& meas)
{
	meas.acc_duo[0] = +imu.linear_acceleration.x*9.81;
	meas.acc_duo[1] = -imu.linear_acceleration.y*9.81;
	meas.acc_duo[2] = -imu.linear_acceleration.z*9.81;

	meas.gyr_duo[0] = +imu.angular_velocity.x;
	meas.gyr_duo[1] = -imu.angular_velocity.y;
	meas.gyr_duo[2] = +imu.angular_velocity.z;
}

void Localization::getViconPosition(void)
{

  tf::StampedTransform transform;
  tf_listener_.lookupTransform( "/world", "/drone_base", ros::Time(0), transform);

  tf::Vector3 position = transform.getOrigin();
  tf::Matrix3x3 rotation = transform.getBasis();
  double roll, pitch, yaw;
  rotation.getRPY(roll, pitch, yaw);

  tf::Quaternion world2control_quaternion;
  world2control_quaternion.setRPY(0.0, 0.0, yaw);

  vicon_pos[0] = position.x();
  vicon_pos[1] = position.y();
  vicon_pos[2] = position.z();

  vicon_quaternion[0] = world2control_quaternion.getX();
  vicon_quaternion[1] = world2control_quaternion.getY();
  vicon_quaternion[2] = world2control_quaternion.getZ();
  vicon_quaternion[3] = world2control_quaternion.getW();

}

void Localization::updateVis(RobotState &robot_state,
		emxArray_AnchorPose *anchor_poses,
		double *map, std::vector<int> &updateVect,
		const duo3d_ros::Duo3d &duo_msg,
		std::vector<double> &z_l,
		bool show_image)
{
	vio_ros::vio_vis msg;

	msg.robot_pose.position.x = robot_state.pos[0];
	msg.robot_pose.position.y = robot_state.pos[1];
	msg.robot_pose.position.z = robot_state.pos[2];

	msg.robot_pose.orientation.x = robot_state.att[0];
	msg.robot_pose.orientation.y = robot_state.att[1];
	msg.robot_pose.orientation.z = robot_state.att[2];
	msg.robot_pose.orientation.w = robot_state.att[3];

	for (int i = 0; i < vioParams.num_anchors; i++)
	{
		geometry_msgs::Pose pose;
		pose.position.x =  anchor_poses->data[i].pos[0];
		pose.position.y =  anchor_poses->data[i].pos[1];
		pose.position.z =  anchor_poses->data[i].pos[2];

		pose.orientation.x = anchor_poses->data[i].att[0];
		pose.orientation.y = anchor_poses->data[i].att[1];
		pose.orientation.z = anchor_poses->data[i].att[2];
		pose.orientation.w = anchor_poses->data[i].att[3];

		msg.anchor_poses.poses.push_back(pose);
	}

	for (int i = 0; i < num_points_; i++)
	{
		msg.map.data.push_back(map[i*3 + 0]);
		msg.map.data.push_back(map[i*3 + 1]);
		msg.map.data.push_back(map[i*3 + 2]);

		msg.status_vect.data.push_back(updateVect[i]);
		if (updateVect[i] == 1)
		{
			msg.feature_tracks.data.push_back(z_l[i*2 + 0]);
			msg.feature_tracks.data.push_back(z_l[i*2 + 1]);
		} else {
			msg.feature_tracks.data.push_back(-100);
			msg.feature_tracks.data.push_back(-100);
		}
	}

	if (show_image)
		msg.image = duo_msg.left_image;

	msg.gyro_bias.data.push_back(robot_state.IMU.gyro_bias[0]);
	msg.gyro_bias.data.push_back(robot_state.IMU.gyro_bias[1]);
	msg.gyro_bias.data.push_back(robot_state.IMU.gyro_bias[2]);

	msg.acc_bias.data.push_back(robot_state.IMU.acc_bias[0]);
	msg.acc_bias.data.push_back(robot_state.IMU.acc_bias[1]);
	msg.acc_bias.data.push_back(robot_state.IMU.acc_bias[2]);

	vio_vis_pub.publish(msg);

}

