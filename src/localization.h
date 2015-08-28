#ifndef _LOCALIZATION_H_
#define _LOCALIZATION_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Path.h>

#include <opencv2/opencv.hpp>

#include "sensor_msgs/Joy.h"

#include "SLAM.h"
#include "SLAM_includes.h"
#include "InterfaceStructs.h"

#include "onboard_localization/ControllerOut.h"

#include <dynamic_reconfigure/server.h>
#include <vio_ros/vio_rosConfig.h>

#include <duo3d_ros/Duo3d.h>

#include <vector>
#include <cstdio>

#include <boost/circular_buffer.hpp>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"

#include "onboard_localization/PositionReference.h"

class Localization
{
public:
	Localization();
	~Localization();

private:
	double t_avg;
	double debug_publish_delay;
	bool SLAM_reset_flag;
	bool received_IMU_data;
	bool publish_on_debug_topics;
	int display_tracks_cnt;
	ros::Time last_debug_publish;

	StereoParameters cameraParams;
	NoiseParameters noiseParams;
	ControllerGains controllerGains;
	VIOParameters vioParams;
	ros::NodeHandle nh_;

	ros::Subscriber combined_sub;
	ros::Subscriber mavros_imu_sub_;
	ros::Subscriber mavros_mag_sub_;
	ros::Subscriber mavros_pressure_sub_;
	ros::Subscriber joy_sub_;
    ros::Subscriber position_reference_sub_;


	ros::Publisher pose_pub_;
	ros::Publisher velocity_pub_;
	ros::Publisher controller_pub;
	ros::Publisher reference_viz_pub;

	ros::Publisher debug_imu_pub_; // publisher that publishes the imu data that is fed to SLAM, for rosbags
	ros::Publisher debug_img_pub_; // publisher that publishes the images that are fed to SLAM, for rosbags

	tf::TransformBroadcaster tf_broadcaster_;

	dynamic_reconfigure::Server<vio_ros::vio_rosConfig> dynamic_reconfigure_server;

	ros::Time prev_time_;
	std::vector<int> update_vec_;

	unsigned int num_points_;
	bool show_tracker_images_;
	emxArray_real_T *h_u_apo_;

	void duo3dCb(const duo3d_ros::Duo3d& msg);
	void mavrosImuCb(const sensor_msgs::Imu msg);
	void mavrosMagCb(const sensor_msgs::MagneticField msg);
	void mavrosPressureCb(const sensor_msgs::FluidPressure msg);
	void joystickCb(const sensor_msgs::Joy::ConstPtr& msg);
	void positionReferenceCb(const onboard_localization::PositionReference& msg);

	void update(double dt, const cv::Mat& left_image, const cv::Mat& right_image, const sensor_msgs::Imu& imu,
			const sensor_msgs::MagneticField& mag, geometry_msgs::Pose& pose, geometry_msgs::Twist& velocity, bool debug_publish);

	void getIMUData(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag, VIOMeasurements& meas);

	void displayTracks(const cv::Mat& left_image, double z_all_l[], double z_all_r[],
			std::vector<int> status, emxArray_real_T *h_u = NULL);

	ros::Publisher point_cloud_pub_;
	void publishPointCloud(double * map );

	void updateDronePose(bool debug_publish);

	ros::Publisher path_pub_;
	nav_msgs::Path slam_path_;
	ros::Publisher vis_pub_;

	sensor_msgs::Imu mavros_imu_data_;
	sensor_msgs::MagneticField mavros_mag_data_;
	sensor_msgs::FluidPressure mavros_pressure_data_;
	duo3d_ros::Duo3d last_duo_msg_;
	ReferenceCommand referenceCommand;
	tf::Quaternion camera2world; // the rotation that transforms a vector in the camera frame to one in the world frame

	bool use_vicon_for_control_;
	tf::TransformListener tf_listener_;
	void getViconPosition(void);
	std::vector<double> vicon_pos;
	std::vector<double> vicon_quaternion;

	void visMarker(void);
	void dynamicReconfigureCb(vio_ros::vio_rosConfig &config, uint32_t level);

};

#endif /* _LOCALIZATION_H_ */
