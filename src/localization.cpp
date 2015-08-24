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

static const unsigned int IMU_delay = 1;

Localization::Localization()
: nh_("~"),
  process_noise_(4,0.0),
  im_noise_(2,0.0),
  t_avg(0.0),
  SLAM_reset_flag(0),
  controller_gains(3,0.0),
  mavros_imu_data_buffer_(IMU_delay),
  received_IMU_data(false),
  body2camera(0.5, 0.5, 0.5, 0.5)
{

	SLAM_initialize();
	emxInitArray_real_T(&h_u_apo_,1);

	pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose",1);
	velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/velocity",1);
	combined_sub = nh_.subscribe("/duo3d_camera/combined",1,
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


	// Init parameters
	// TODO Check default values and give meaningful names
	nh_.param<bool>("show_tracker_images", show_tracker_images_, false);

	nh_.param<double>("process_noise_1", process_noise_[0], 100);
	nh_.param<double>("process_noise_2", process_noise_[1], 1);
	nh_.param<double>("process_noise_3", process_noise_[2], 0.0);
	// nh_.param<double>("process_noise_4", process_noise_[3], 0.0);

	nh_.param<double>("im_noise_1", im_noise_[0], 2.0);
	nh_.param<double>("im_noise_2", im_noise_[1], 2.0);

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
		ROS_ERROR("Failed to open camera calibration at %s", path.c_str());
	}else {
		cameraParams = parseYaml(YamlNode);
	}

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

	int num_points_per_anchor, num_anchors;
	nh_.param<int>("num_points_per_anchor", num_points_per_anchor, 1);
	nh_.param<int>("num_anchors", num_anchors, 32);

	if (num_anchors < 0.0)
	{
		ROS_ERROR("Number of anchors must not be negative!");
		nh_.shutdown();
	}
	else
	{
		num_anchors_ = static_cast<unsigned int>(num_anchors);
	}

	if (num_points_per_anchor < 0.0)
	{
		ROS_ERROR("Number of points per anchors must not be negative!");
		nh_.shutdown();
	}
	else
	{
		num_points_per_anchor_ = static_cast<unsigned int>(num_points_per_anchor);
	}

	dynamic_reconfigure::Server<vio_ros::controllerConfig>::CallbackType f = boost::bind(&Localization::dynamicReconfigureCb, this, _1, _2);
	dynamic_reconfigure_server.setCallback(f);

	num_points_ = num_anchors_*num_points_per_anchor_;

	update_vec_.assign(num_points_, 0);

	// initialize a valid quaternion in case this topic does not publish
	sensor_msgs::Imu mavros_imu_data;
	mavros_imu_data.orientation.x = 0.0;
	mavros_imu_data.orientation.y = 0.0;
	mavros_imu_data.orientation.z = 0.0;
	mavros_imu_data.orientation.w = 1.0;

	for (int i = 0; i < IMU_delay; i++)
	{
		mavros_imu_data_buffer_.push_back(mavros_imu_data);
	}
}

Localization::~Localization()
{
	emxDestroyArray_real_T(h_u_apo_);
	SLAM_terminate();
}

void Localization::duo3dCb(const duo3d_ros::Duo3d& msg)
{
	if (!received_IMU_data)
	{
		ROS_INFO("No IMU data yet!");
		return;
	}

	last_duo_msg_ = msg;

	double tic_total = ros::Time::now().toSec();
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

	// Init time on first call
	if (prev_time_.isZero())
	{
		prev_time_ = msg.header.stamp;
	}

	geometry_msgs::PoseStamped pose_stamped;
	geometry_msgs::Pose pose;
	geometry_msgs::Twist velocity;
	pose_stamped.header.stamp = msg.header.stamp;
	pose_stamped.header.frame_id = "world";
	double dt = (msg.header.stamp - prev_time_).toSec();
	prev_time_ = msg.header.stamp;

	bool debug_publish = (ros::Time::now() - last_debug_publish).toSec() > debug_publish_delay;
	if (debug_publish)
		last_debug_publish = ros::Time::now();

	update(dt, cv_left_image->image, cv_right_image->image, msg.imu, mag, pose, velocity, debug_publish);

	pose_stamped.pose = pose;
	pose_pub_.publish(pose_stamped);
	velocity_pub_.publish(velocity);


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

	double time_measurement = ros::Time::now().toSec() - tic_total;

	t_avg=0.05*time_measurement+(1-0.05)*t_avg;
	if (debug_publish)
		ROS_INFO("Duration: %f ms. Theoretical max frequency: %.3f Hz\n", t_avg, 1/t_avg);
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
		ROS_INFO("resetting SLAM");
	}
}

void Localization::dynamicReconfigureCb(vio_ros::controllerConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure Request: Position: Kp: %.3f, Kd %.3f, Yaw: Kp %.3f", config.Kp_pos, config.Kd_pos, config.Kp_yaw);
	controller_gains[0] = config.Kp_pos;
	controller_gains[1] = config.Kd_pos;
	controller_gains[2] = config.Kp_yaw;
}

void Localization::positionReferenceCb(const onboard_localization::PositionReference& msg)
{
	printf("got position reference change        : (%.3f, %.3f, %.3f, %.3f)\n", msg.x, msg.y, msg.z, msg.yaw);
	tf::Transform body2world = tf::Transform(body2camera) * tf::Transform(camera2world);
	tf::Vector3 positionChange_world = body2world * tf::Vector3(msg.x, msg.y, msg.z);
	pos_reference[0] += positionChange_world.x();
	pos_reference[1] += positionChange_world.y();
	pos_reference[2] += positionChange_world.z();
	pos_reference[3] += msg.yaw;
	printf("got position reference change (world): (%.3f, %.3f, %.3f, %.3f)\n", positionChange_world.x(), positionChange_world.y(), positionChange_world.z(), msg.yaw);
	printf("position reference: (%.3f, %.3f, %.3f, %.3f)\n", pos_reference[0], pos_reference[1], pos_reference[2], pos_reference[3]);

	geometry_msgs::PoseStamped ref_viz;
	ref_viz.header.stamp = ros::Time::now();
	ref_viz.header.frame_id = "world";
	ref_viz.pose.position.x = pos_reference[0];
	ref_viz.pose.position.y = pos_reference[1];
	ref_viz.pose.position.z = pos_reference[2];

	tf::Quaternion quaternion;
	quaternion.setRPY(0.0, 0.0, pos_reference[3]);
	ref_viz.pose.orientation.w = quaternion.getW();
	ref_viz.pose.orientation.x = quaternion.getX();
	ref_viz.pose.orientation.y = quaternion.getY();
	ref_viz.pose.orientation.z = quaternion.getZ();

	reference_viz_pub.publish(ref_viz);
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

	handle_points_klt(left_image, right_image, z_all_l, z_all_r, update_vec_);

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

	std::vector<double> IMU_data(23,0.0);
	getIMUData(imu, mag, IMU_data);

	if (publish_on_debug_topics)
	{
		debug_img_pub_.publish(last_duo_msg_);
	}

	emxArray_real_T *xt_out; // result
	emxArray_real_T *P_apo_out;
	emxArray_real_T *h_u_apo;
	emxArray_real_T *map;

	emxInitArray_real_T(&xt_out,1);
	emxInitArray_real_T(&P_apo_out,2);
	emxInitArray_real_T(&h_u_apo,1);
	emxInitArray_real_T(&map,2);

	double u_out[4];

	// Update SLAM and get pose estimation
	tic = ros::Time::now();

	SLAM(update_vec_array,
			&z_all_l[0],
			&z_all_r[0],
			dt,
			&process_noise_[0],
			&IMU_data[0],
			&im_noise_[0],
			num_points_per_anchor_,
			num_anchors_,
			&cameraParams,
			SLAM_reset_flag,
			&pos_reference[0],
			&controller_gains[0],
			h_u_apo,
			xt_out,
			P_apo_out,
			map,
			u_out);

	onboard_localization::ControllerOut controller_out_msg;
	controller_out_msg.x = u_out[0];
	controller_out_msg.y = u_out[1];
	controller_out_msg.z = u_out[2];
	controller_out_msg.yaw = u_out[3];

	controller_pub.publish(controller_out_msg);

	SLAM_reset_flag = false;

	for(int i = 0; i < update_vec_.size(); i++)
	{
		update_vec_[i] = update_vec_array[i];
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

	emxDestroyArray_real_T(xt_out);
	emxDestroyArray_real_T(P_apo_out);
	emxDestroyArray_real_T(h_u_apo);
	emxDestroyArray_real_T(map);


}

void Localization::getIMUData(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag, std::vector<double>& inertial_vec)
{
	// push the most recent IMU data into the buffer
	mavros_imu_data_buffer_.push_back(mavros_imu_data_);

	inertial_vec.at(0) = +imu.angular_velocity.x;
	inertial_vec.at(1) = -imu.angular_velocity.y;
	inertial_vec.at(2) = +imu.angular_velocity.z;

	inertial_vec.at(3) = +imu.linear_acceleration.x*9.81;
	inertial_vec.at(4) = -imu.linear_acceleration.y*9.81;
	inertial_vec.at(5) = -imu.linear_acceleration.z*9.81;

	inertial_vec.at(6) = +mag.magnetic_field.x;
	inertial_vec.at(7) = +mag.magnetic_field.y;
	inertial_vec.at(8) = +mag.magnetic_field.z;

	inertial_vec.at(9) = mavros_pressure_data_.fluid_pressure;

	inertial_vec.at(10) = mavros_mag_data_.magnetic_field.x;
	inertial_vec.at(11) = mavros_mag_data_.magnetic_field.y;
	inertial_vec.at(12) = mavros_mag_data_.magnetic_field.z;

	inertial_vec.at(13) = mavros_imu_data_.angular_velocity.x;
	inertial_vec.at(14) = mavros_imu_data_.angular_velocity.y;
	inertial_vec.at(15) = mavros_imu_data_.angular_velocity.z;

	inertial_vec.at(16) = mavros_imu_data_.linear_acceleration.x;
	inertial_vec.at(17) = mavros_imu_data_.linear_acceleration.y;
	inertial_vec.at(18) = mavros_imu_data_.linear_acceleration.z;

	inertial_vec.at(19) = mavros_imu_data_.orientation.x;
	inertial_vec.at(20) = mavros_imu_data_.orientation.y;
	inertial_vec.at(21) = mavros_imu_data_.orientation.z;
	inertial_vec.at(22) = mavros_imu_data_.orientation.w;

	if (publish_on_debug_topics)
	{
		std_msgs::Float32MultiArray array;
		array.data.clear();
		for (int i = 0; i < inertial_vec.size(); i++)
			array.data.push_back(inertial_vec[i]);
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

	for(int cnt = 0; cnt < num_points_per_anchor_*num_anchors_; cnt++)
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
	tf_broadcaster_.sendTransform(tf::StampedTransform(camera2drone, ros::Time::now(), "camera", "drone_base"));

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



