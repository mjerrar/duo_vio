#include "localization.h"
#include "SLAM.h"

#include "klt_point_handling.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <math.h>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <time.h>

static const int DUO_QUEUE_SIZE = 30;

Localization::Localization()
: nh_("~"),
  t_avg(0.0),
  SLAM_reset_flag(0),
  received_IMU_data(false),
  change_reference(false),
  vicon_pos(3, 0.0),
  vicon_quaternion(4, 0.0),
  max_clicks_(0),
  clear_queue_counter(0)
{
	emxInitArray_real_T(&xt_out,1);
	emxInitArray_real_T(&P_apo_out,2);
	emxInitArray_real_T(&h_u_apo,1);
	emxInitArray_real_T(&map,2);
	SLAM_initialize();
	emxInitArray_real_T(&h_u_apo_,1);

	// initialize struct
	referenceCommand = {{0, 0, 0, 0}, {0, 0, 0, 0}};
	cameraParams = {{},{}};
	noiseParams = {};
	controllerGains = {};
	vioParams = {};

	pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose",1);
	velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/velocity",1);
	combined_sub = nh_.subscribe("/duo3d_camera/combined",DUO_QUEUE_SIZE,
			&Localization::duo3dCb,this);
	point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/vio/features_point_cloud",1); //TODO: add to debug parameter
	path_pub_ = nh_.advertise<nav_msgs::Path>("/vio/SLAM_path",1);
	vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "drone", 0 );
	reference_viz_pub = nh_.advertise<geometry_msgs::PoseStamped>("/onboard_localization/position_reference_viz",1);

	controller_pub = nh_.advertise<onboard_localization::ControllerOut>("/onboard_localization/controller_output",10);

	mavros_imu_sub_ = nh_.subscribe("/mavros/imu/data", 1,
			&Localization::mavrosImuCb, this);
	mavros_mag_sub_ = nh_.subscribe("/mavros/imu/mag", 1,
			&Localization::mavrosMagCb, this);
	mavros_pressure_sub_ = nh_.subscribe("/mavros/imu/atm_pressure", 1,
			&Localization::mavrosPressureCb, this);
	joy_sub_ = nh_.subscribe("/joy",1, &Localization::joystickCb, this);
	position_reference_sub_ = nh_.subscribe("/onboard_localization/position_reference",1, &Localization::positionReferenceCb, this);
	msg_processed_pub = nh_.advertise<std_msgs::Int32>("/vio/msg_processed", 1);


	// Init parameters
	nh_.param<bool>("show_tracker_images", show_tracker_images_, false);

	nh_.param<double>("acc_noise", noiseParams.process_noise[0], 1);
	nh_.param<double>("gyro_noise", noiseParams.process_noise[1], 1);
	nh_.param<double>("gyro_bias_noise", noiseParams.process_noise[2], 1);
	nh_.param<double>("process_noise_3", noiseParams.process_noise[2], 0.0);
	nh_.param<double>("process_noise_4", noiseParams.process_noise[3], 0.0);
	nh_.param<double>("orientation_noise", noiseParams.orientation_noise, 1.0);
	nh_.param<double>("pressure_noise", noiseParams.pressure_noise, 1.0);
	nh_.param<double>("sigma_init", noiseParams.sigmaInit, 0.0001);
	nh_.param<double>("im_noise", noiseParams.image_noise[0], 1.0);
	nh_.param<double>("im_noise", noiseParams.image_noise[1], 1.0);
	nh_.param<double>("ext_pos_noise", noiseParams.ext_pos_noise, 0.1);
	nh_.param<double>("ext_att_noise", noiseParams.ext_att_noise, 0.1);
	nh_.param<double>("gravity_alignment_noise", noiseParams.gravity_alignment_noise, 1);

	nh_.param<int>("num_points_per_anchor", vioParams.num_points_per_anchor, 1);
	nh_.param<int>("num_anchors", vioParams.num_anchors, 1);
	nh_.param<int>("max_ekf_iterations", vioParams.max_ekf_iterations, 1);

	nh_.param<bool>("use_orientation", vioParams.use_orientation, false);
	nh_.param<bool>("use_pressure", vioParams.use_pressure, false);
	nh_.param<bool>("use_magnetometer", vioParams.use_magnetometer, false);
	nh_.param<bool>("use_controller_to_predict", vioParams.use_controller_to_predict, false);
	nh_.param<bool>("use_ext_pose", vioParams.use_ext_pose, false);
	nh_.param<bool>("fixed_anchor", vioParams.fixed_anchor, false);
	nh_.param<bool>("gravity_align", vioParams.gravity_align, false);


	nh_.param<double>("Kp_xy", controllerGains.Kp_xy, 1);
	nh_.param<double>("Ki_xy", controllerGains.Ki_xy, 0);
	nh_.param<double>("Kd_xy", controllerGains.Kd_xy, 1);
	nh_.param<double>("Kp_z", controllerGains.Kp_xy, 1);
	nh_.param<double>("Ki_z", controllerGains.Ki_z, 0);
	nh_.param<double>("Kd_z", controllerGains.Kd_z, 1);
	nh_.param<double>("Kp_yaw", controllerGains.Kp_yaw, 1);
	nh_.param<double>("Kd_yaw", controllerGains.Kd_yaw, 1);
	nh_.param<double>("i_lim", controllerGains.i_lim, 1);

	nh_.param<bool>("use_vicon_for_control", use_vicon_for_control_, false);


	std::string camera_name;
	nh_.param<std::string>("camera_name", camera_name, "NoName");
	std::string lense_type;
	nh_.param<std::string>("lense_type", lense_type, "NoType");
	int resolution_width;
	nh_.param<int>("resolution_width", resolution_width, 0);
	int resolution_height;
	nh_.param<int>("resolution_height", resolution_height, 0);

	std::stringstream res;
	res << resolution_height << "x" << resolution_width;
	std::string path = ros::package::getPath("vio_ros") + "/calib/" + camera_name + "/" + lense_type + "/" + res.str() + "/cameraParams.yaml";

	YAML::Node YamlNode = YAML::LoadFile(path);
	if (YamlNode.IsNull())
	{
		throw std::string("Failed to open camera calibration %s", path.c_str());
	}

	cameraParams = parseYaml(YamlNode);

	double debug_publish_freq;
	nh_.param<double>("debug_publish_freq", debug_publish_freq, 1);
	debug_publish_delay = 1.0/debug_publish_freq;
	last_debug_publish = ros::Time::now();

	nh_.param<bool>("publish_on_debug_topics", publish_on_debug_topics, 1);

	if (publish_on_debug_topics)
	{
		debug_imu_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/vio/debug_imu", 1);
		debug_img_pub_ = nh_.advertise<duo3d_ros::Duo3d>("/vio/debug_img", 1);
	}

	dynamic_reconfigure::Server<vio_ros::vio_rosConfig>::CallbackType f = boost::bind(&Localization::dynamicReconfigureCb, this, _1, _2);
	dynamic_reconfigure_server.setCallback(f);

	num_points_ = vioParams.num_anchors*vioParams.num_points_per_anchor;

	update_vec_.assign(num_points_, 0);


//	// initialize a valid quaternion in case this topic does not publish
//	mavros_imu_data_.orientation.w = 1.0;

}

Localization::~Localization()
{
	emxDestroyArray_real_T(h_u_apo_);
	emxDestroyArray_real_T(xt_out);
	emxDestroyArray_real_T(P_apo_out);
	emxDestroyArray_real_T(h_u_apo);
	emxDestroyArray_real_T(map);
	SLAM_terminate();

	printf("Longest update duration: %.3f msec, %.3f Hz\n", float(max_clicks_)/CLOCKS_PER_SEC, CLOCKS_PER_SEC/float(max_clicks_));
}

void Localization::duo3dCb(const duo3d_ros::Duo3d& msg)
{
	if (!received_IMU_data && vioParams.use_orientation)
	{
		ROS_INFO("No IMU data yet!");
		std_msgs::Int32 m;
		m.data = DUO_QUEUE_SIZE;
		msg_processed_pub.publish(m);
		return;
	}

	// upon reset, catch up with the duo messages before resetting SLAM
	if (SLAM_reset_flag)
	{
		if(clear_queue_counter < DUO_QUEUE_SIZE)
		{
			clear_queue_counter++;
			std_msgs::Int32 m;
			m.data = DUO_QUEUE_SIZE;
			msg_processed_pub.publish(m);
			return;
		} else {
			clear_queue_counter = 0;
		}
	}

	ros::Time tic_total = ros::Time::now();
	clock_t tic_total_clock = clock();

	last_duo_msg_ = msg;

	sensor_msgs::MagneticField mag; // TODO Subscribe to mag topic

	cv_bridge::CvImagePtr cv_left_image;
	cv_bridge::CvImagePtr cv_right_image;
	try
	{
		cv_left_image = cv_bridge::toCvCopy(msg.left_image, "mono8");
		cv_right_image = cv_bridge::toCvCopy(msg.right_image,"mono8");
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("Error while converting ROS image to OpenCV: %s", e.what());
		return;
	}

	if(cv_left_image->image.empty() || cv_right_image->image.empty())
	{
		return;
	}

	double dt;
	// Init time on first call
	if (prev_time_.isZero())
	{
		prev_time_ = msg.header.stamp;
		dt = 1/60.0;
	} else {
		dt = (msg.header.stamp - prev_time_).toSec();
		prev_time_ = msg.header.stamp;
	}

	geometry_msgs::PoseStamped pose_stamped;
	geometry_msgs::Twist velocity;
	pose_stamped.header.stamp = msg.header.stamp;
	pose_stamped.header.frame_id = "world";

	bool debug_publish = (ros::Time::now() - last_debug_publish).toSec() > debug_publish_delay;
	bool debug_display_tracks = false;

	if (debug_publish)
	{
		last_debug_publish = ros::Time::now();

		if (display_tracks_cnt > 10)
		{
			debug_display_tracks = true;
			display_tracks_cnt = 0;
		} else {
			display_tracks_cnt++;
		}
	}

	update(dt, cv_left_image->image, cv_right_image->image, msg.imu, mag, pose, velocity, debug_display_tracks);

	pose_stamped.pose = pose;
	pose_pub_.publish(pose_stamped);

	if(!use_vicon_for_control_)
	{
		velocity_pub_.publish(velocity);
	}


	// Generate and publish pose as transform
	camera2world = tf::Quaternion(pose.orientation.x,pose.orientation.y,
			pose.orientation.z,pose.orientation.w);
	tf::Transform transform;
	transform.setRotation(camera2world);
	transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));

	tf_broadcaster_.sendTransform(tf::StampedTransform(transform, /*pose_stamped.header.stamp*/ ros::Time::now(), "world", "SLAM"));
	if (debug_publish)
	{
		tf_broadcaster_.sendTransform(tf::StampedTransform(transform, /*pose_stamped.header.stamp*/ ros::Time::now(), "world", "SLAM_rviz"));
	}

	updateDronePose(debug_publish);

	if (debug_publish)
	{
		slam_path_.poses.push_back(pose_stamped);
		slam_path_.header = pose_stamped.header;
		path_pub_.publish(slam_path_);

		visMarker();
	}

	double time_measurement = (ros::Time::now() - tic_total).toSec();

	t_avg=0.05*time_measurement+(1-0.05)*t_avg;
	if (debug_publish || time_measurement > 1/60.0)
	{
		if (time_measurement > 1/60.0)
			ROS_WARN("Duration: %f ms. Theoretical max frequency: %.3f Hz\n", time_measurement, 1/time_measurement);
		else
			ROS_INFO("Duration: %f ms. Theoretical max frequency: %.3f Hz\n", time_measurement, 1/time_measurement);
	}
	clock_t toc_total_clock = clock();

	if (toc_total_clock - tic_total_clock > max_clicks_)
		max_clicks_ = toc_total_clock - tic_total_clock;

	std_msgs::Int32 m;
	m.data = DUO_QUEUE_SIZE;
	msg_processed_pub.publish(m);

}

void Localization::mavrosImuCb(const sensor_msgs::Imu msg)
{
	received_IMU_data = true;
	mavros_imu_data_ = msg;
}

void Localization::mavrosMagCb(const sensor_msgs::MagneticField msg)
{
	mavros_mag_data_ = msg;
}

void Localization::mavrosPressureCb(const sensor_msgs::FluidPressure msg)
{
	mavros_pressure_data_ = msg;
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
		quaternion_yaw.setX(mavros_imu_data_.orientation.x);
		quaternion_yaw.setY(mavros_imu_data_.orientation.y);
		quaternion_yaw.setZ(mavros_imu_data_.orientation.z);
		quaternion_yaw.setW(mavros_imu_data_.orientation.w);
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

	    reference_viz_pub.publish(ref_viz);

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

		reference_viz_pub.publish(ref_viz);

	} else if (msg->buttons[3]) { // leaving auto mode signal
		change_reference = true;
	}
}

void Localization::dynamicReconfigureCb(vio_ros::vio_rosConfig &config, uint32_t level)
{
	controllerGains.Kp_xy = config.Kp_xy;
	controllerGains.Ki_xy = config.Ki_xy;
	controllerGains.Kd_xy = config.Kd_xy;
	controllerGains.Kp_z = config.Kp_z;
	controllerGains.Ki_z = config.Ki_z;
	controllerGains.Kd_z = config.Kd_z;
	controllerGains.Kp_yaw = config.Kp_yaw;
	controllerGains.Kd_yaw = config.Kd_yaw;
	controllerGains.i_lim = config.i_lim;

	noiseParams.image_noise[0] = config.im_noise;
	noiseParams.image_noise[1] = config.im_noise;
	noiseParams.process_noise[0] = config.acc_noise;
	noiseParams.process_noise[1] = config.gyro_noise;
	noiseParams.process_noise[2] = config.gyro_bias_noise;
//	noiseParams.orientation_noise = config.orientation_noise;
	noiseParams.pressure_noise = config.pressure_noise;
	noiseParams.sigmaInit = config.sigma_init;
	noiseParams.gravity_alignment_noise = config.gravity_alignment_noise;

//	vioParams.use_orientation = config.use_orientation;
	vioParams.use_pressure = config.use_pressure;
	vioParams.use_magnetometer = config.use_magnetometer;
	vioParams.use_controller_to_predict = config.use_controller_to_predict;
	vioParams.max_ekf_iterations = config.max_ekf_iterations;
	vioParams.use_ext_pose = config.use_ext_pose;
	vioParams.fixed_anchor = config.fixed_anchor;
	vioParams.gravity_align = config.gravity_align;

	show_tracker_images_ = config.show_tracker_images;


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

		reference_viz_pub.publish(ref_viz);
	}
}

void Localization::update(double dt, const cv::Mat& left_image, const cv::Mat& right_image, const sensor_msgs::Imu& imu,
		const sensor_msgs::MagneticField& mag, geometry_msgs::Pose& pose, geometry_msgs::Twist& velocity, bool debug_publish)
{

	//*********************************************************************
	// Point tracking
	//*********************************************************************
	int measurementDim = 4;

	std::vector<double> z_all_l(num_points_*2, 0.0);
	std::vector<double> z_all_r(num_points_*2, 0.0);

	ros::Time tic = ros::Time::now();
	ros::Time tic_total = tic;

//	clock_t bef = clock();

	handle_points_klt(left_image, right_image, z_all_l, z_all_r, update_vec_);

//	clock_t aft = clock();
//	printf("KLT  took %d clicks, %.3f msec\n", int(aft - bef), 1000*float(aft - bef)/CLOCKS_PER_SEC);

	double update_vec_array[num_points_];
	double update_vec_array_out[num_points_];
	for (size_t i = 0; i < num_points_; ++i)
	{
		update_vec_array[i] = update_vec_[i];
	}

	//ROS_INFO("Time point tracker: %6.2f ms", (ros::Time::now() - tic).toSec()*1000);

	//*********************************************************************
	// SLAM
	//*********************************************************************

	VIOMeasurements meas;
	getIMUData(imu, mag, meas);

	if (publish_on_debug_topics)
	{
		debug_img_pub_.publish(last_duo_msg_);
	}

	double u_out[4];

	if(use_vicon_for_control_)
	{
	  getViconPosition();
	}

	// Update SLAM and get pose estimation
	tic = ros::Time::now();

//	bef = clock();

	SLAM(update_vec_array,
			&z_all_l[0],
			&z_all_r[0],
			dt,
			&meas,
			&referenceCommand,
			&vioParams,
			&cameraParams,
			&noiseParams,
			&controllerGains,
			SLAM_reset_flag,
			h_u_apo,
			xt_out,
			P_apo_out,
			map,
			u_out);

//	aft = clock();
//	printf("SLAM took %d clicks, %.3f msec\n", int(aft - bef), 1000*float(aft - bef)/CLOCKS_PER_SEC);

	// failure check
	double dx = abs(xt_out->data[0] - pose.position.x);
	double dy = abs(xt_out->data[1] - pose.position.y);
	double dz = abs(xt_out->data[2] - pose.position.z);
	double failure_threshold = 2/60;
	double bounds = 20; // maximum distance from center


	if (!SLAM_reset_flag && (
			dx > failure_threshold ||
			dy > failure_threshold ||
			dz > failure_threshold ||
			abs(xt_out->data[0]) > bounds ||
			abs(xt_out->data[1]) > bounds ||
			abs(xt_out->data[2]) > bounds))
	{
		ROS_ERROR("VIO FAILING. Resetting");

		SLAM_reset_flag = true;
	} else {
		SLAM_reset_flag = false;

		onboard_localization::ControllerOut controller_out_msg;
		controller_out_msg.x = u_out[0];
		controller_out_msg.y = u_out[1];
		controller_out_msg.z = u_out[2];
		controller_out_msg.yaw = u_out[3];

		controller_pub.publish(controller_out_msg);

		for(int i = 0; i < update_vec_.size(); i++)
		{
			update_vec_[i] = update_vec_array[i];
		}

		// Set the pose
		pose.position.x = xt_out->data[0];
		pose.position.y = xt_out->data[1];
		pose.position.z = xt_out->data[2];

		pose.orientation.x = xt_out->data[3];
		pose.orientation.y = xt_out->data[4];
		pose.orientation.z = xt_out->data[5];
		pose.orientation.w = xt_out->data[6];

		velocity.linear.x = xt_out->data[7];
		velocity.linear.y = xt_out->data[8];
		velocity.linear.z = xt_out->data[9];

		velocity.angular.x = xt_out->data[10];
		velocity.angular.y = xt_out->data[11];
		velocity.angular.z = xt_out->data[12];
	}



	if (debug_publish)
	{
		if (show_tracker_images_)
		{
			displayTracks(left_image, &z_all_l[0], &z_all_r[0], update_vec_, h_u_apo);
		}
		// Publish feature position in world frame
		publishPointCloud(map->data);
	}

	//ROS_INFO("Time SLAM         : %6.2f ms", (ros::Time::now() - tic).toSec()*1000);
}

void Localization::getIMUData(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag, VIOMeasurements& meas)
{
	meas.acc_duo[0] = +imu.linear_acceleration.x*9.81;
	meas.acc_duo[1] = -imu.linear_acceleration.y*9.81;
	meas.acc_duo[2] = -imu.linear_acceleration.z*9.81;

	meas.gyr_duo[0] = +imu.angular_velocity.x;
	meas.gyr_duo[1] = -imu.angular_velocity.y;
	meas.gyr_duo[2] = +imu.angular_velocity.z;

	meas.mag_duo[0] = +mag.magnetic_field.x;
	meas.mag_duo[1] = +mag.magnetic_field.y;
	meas.mag_duo[2] = +mag.magnetic_field.z;

	meas.acc_fmu[0] = mavros_imu_data_.linear_acceleration.x;
	meas.acc_fmu[1] = mavros_imu_data_.linear_acceleration.y;
	meas.acc_fmu[2] = mavros_imu_data_.linear_acceleration.z;

	meas.gyr_fmu[0] = mavros_imu_data_.angular_velocity.x;
	meas.gyr_fmu[1] = mavros_imu_data_.angular_velocity.y;
	meas.gyr_fmu[2] = mavros_imu_data_.angular_velocity.z;

	meas.bar_fmu = mavros_pressure_data_.fluid_pressure;

	meas.mag_fmu[0] = mavros_mag_data_.magnetic_field.x;
	meas.mag_fmu[1] = mavros_mag_data_.magnetic_field.y;
	meas.mag_fmu[2] = mavros_mag_data_.magnetic_field.z;

	meas.att_fmu[0] = mavros_imu_data_.orientation.x;
	meas.att_fmu[1] = mavros_imu_data_.orientation.y;
	meas.att_fmu[2] = mavros_imu_data_.orientation.z;
	meas.att_fmu[3] = mavros_imu_data_.orientation.w;

	// TODO: Write external position measurement

	if (publish_on_debug_topics)
	{
		std_msgs::Float32MultiArray array;
		array.data.clear();

		array.data.push_back(+imu.angular_velocity.x);
		array.data.push_back(-imu.angular_velocity.y);
		array.data.push_back(+imu.angular_velocity.z);

		array.data.push_back(+imu.linear_acceleration.x*9.81);
		array.data.push_back(-imu.linear_acceleration.y*9.81);
		array.data.push_back(-imu.linear_acceleration.z*9.81);

		array.data.push_back(+mag.magnetic_field.x);
		array.data.push_back(+mag.magnetic_field.y);
		array.data.push_back(+mag.magnetic_field.z);

		array.data.push_back(mavros_pressure_data_.fluid_pressure);

		array.data.push_back(mavros_mag_data_.magnetic_field.x);
		array.data.push_back(mavros_mag_data_.magnetic_field.y);
		array.data.push_back(mavros_mag_data_.magnetic_field.z);

		array.data.push_back(mavros_imu_data_.angular_velocity.x);
		array.data.push_back(mavros_imu_data_.angular_velocity.y);
		array.data.push_back(mavros_imu_data_.angular_velocity.z);

		array.data.push_back(mavros_imu_data_.linear_acceleration.x);
		array.data.push_back(mavros_imu_data_.linear_acceleration.y);
		array.data.push_back(mavros_imu_data_.linear_acceleration.z);

		array.data.push_back(mavros_imu_data_.orientation.x);
		array.data.push_back(mavros_imu_data_.orientation.y);
		array.data.push_back(mavros_imu_data_.orientation.z);
		array.data.push_back(mavros_imu_data_.orientation.w);

		debug_imu_pub_.publish(array);
	}
}

void Localization::displayTracks(const cv::Mat& left_image, double z_all_l[], double z_all_r[], std::vector<int> status, emxArray_real_T *h_u)
{
	cv::Mat left;
	cv::cvtColor(left_image,left,cv::COLOR_GRAY2BGR);

	cv::Scalar color_left = cv::Scalar(0,140,255);
	cv::Scalar color_right = cv::Scalar(255,140,0);
	cv::Scalar color_left_pred = cv::Scalar(127,0,127);


	for (unsigned int i = 0; i < num_points_; ++i)
	{
		if (status[i])
		{
			cv::Point left_point(z_all_l[2*i + 0], z_all_l[2*i + 1]);

			std::stringstream ss;
			ss << i+1;
			cv::putText(left, ss.str(), left_point, 1, 1, cvScalar(0,0,255), 1, CV_AA);

			cv::circle(left, left_point, 2, color_left, 2);

			if (z_all_r[2*i + 0] > 0) // plot stereo measurement if available
					{
				cv::Point right_point(z_all_r[2*i + 0], z_all_r[2*i + 1]);
				cv::circle(left, right_point, 2, color_right, 2);
				cv::line(left, left_point, right_point, color_right, 1);
					}

			if(!(h_u == NULL))
			{
				if (h_u->data[4*i + 0] > -100)
				{
					cv::Point left_point_pred(h_u->data[4*i + 0], h_u->data[4*i + 1]);
					cv::Point right_point_pred(h_u->data[4*i + 2], h_u->data[4*i + 3]);
					cv::circle(left, left_point_pred, 1, color_left_pred, 2);
					cv::line(left, left_point, left_point_pred, color_left_pred, 1);
				}
			}
		}
	}
	cv::imshow("left image", left);
	cv::waitKey(10);
}

void Localization::publishPointCloud(double *map)
{
	sensor_msgs::PointCloud features;

	features.header.frame_id = "world";
	features.header.stamp = ros::Time::now();

	for(int cnt = 0; cnt < vioParams.num_points_per_anchor * vioParams.num_anchors; cnt++)
	{
		geometry_msgs::Point32 point;
		point.x = map[cnt*3];
		point.y = map[cnt*3+1];
		point.z = map[cnt*3+2];

		features.points.push_back(point);
	}

	point_cloud_pub_.publish(features);
}



void Localization::updateDronePose(bool debug_publish)
{
	/*tf::Transform camera2drone;
    camera2drone.setOrigin(tf::Vector3(0.0, 0.0, -0.10));
    tf::Quaternion q_c2d;
    q_c2d.setEulerZYX(-M_PI/2, -M_PI/2, 0.0);
    camera2drone.setRotation(q_c2d);
    tf_broadcaster_.sendTransform(tf::StampedTransform(camera2drone, ros::Time::now(), "camera", "drone_base"));*/

	/*tf::Transform slam2drone;
    slam2drone.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q_s2d;
    q_s2d.setEuler(0.0, M_PI, 0.0);// check convention
    slam2drone.setRotation(q_s2d);
    tf_broadcaster_.sendTransform(tf::StampedTransform(slam2drone, ros::Time::now(), "SLAM", "drone_base"));*/

	tf::Transform slam2camera;
	slam2camera.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	tf::Quaternion q_s2d;
	q_s2d.setEuler(0.0, 0.0, 0.0);
	slam2camera.setRotation(q_s2d);
	tf_broadcaster_.sendTransform(tf::StampedTransform(slam2camera, ros::Time::now(), "SLAM", "camera"));

	if (debug_publish)
	{
		tf_broadcaster_.sendTransform(tf::StampedTransform(slam2camera, ros::Time::now(), "SLAM_rviz", "camera_rviz"));
	}

	/*tf::Transform drone2camera;
    drone2camera.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
    tf::Quaternion q_d2c;
    q_d2c.setEuler(M_PI/2, 0.0, M_PI/2);
    drone2camera.setRotation(q_d2c);
    tf_broadcaster_.sendTransform(tf::StampedTransform(drone2camera, ros::Time::now(), "drone_base", "camera"));*/

	tf::Transform camera2drone;
	camera2drone.setOrigin(tf::Vector3(0.0, 0.0, -0.045));
	tf::Quaternion q_d2c;
	q_d2c.setEuler(-M_PI/2, -M_PI/2, 0.0);
	camera2drone.setRotation(q_d2c);

	if(!use_vicon_for_control_)
	{
	  tf_broadcaster_.sendTransform(tf::StampedTransform(camera2drone, ros::Time::now(), "camera", "drone_base"));
	}

	if (debug_publish)
	{
		tf_broadcaster_.sendTransform(tf::StampedTransform(camera2drone, ros::Time::now(), "camera_rviz", "drone_base_rviz"));
	}
}

void Localization::visMarker(void)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "drone_base";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 1.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;
	marker.scale.x = 0.09;
	marker.scale.y = 0.045;
	marker.scale.z = 0.01;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
	//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	vis_pub_.publish(marker);
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

