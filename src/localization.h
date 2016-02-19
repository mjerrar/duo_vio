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
#include <dynamic_reconfigure/server.h>
#include <vio_ros/vio_rosConfig.h>

#include <vio_ros/vio_vis.h>
#include <vio_ros/VioSensorMsg.h>

#include <vector>
#include <cstdio>
#include <fstream>

#include <boost/circular_buffer.hpp>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/String.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt32.h"

#include "parseYaml.h"

#include "Precision.h"

class Localization
{
public:
	Localization();
	~Localization();

private:
	// Visualization topics
	ros::Publisher vio_vis_pub;
	ros::Publisher vio_vis_reset_pub;

	int vis_publish_delay;
	bool SLAM_reset_flag;
	int display_tracks_cnt;
	int max_clicks_;
	int clear_queue_counter;
	double fps_duo;
	int vio_cnt;
	int vision_subsample;

	DUOParameters cameraParams;
	NoiseParameters noiseParams;
	VIOParameters vioParams;
	ros::NodeHandle nh_;

	cv::Mat darkCurrentL, darkCurrentR;
	bool use_dark_current;

	ros::Subscriber duo_sub;
	ros::Subscriber device_serial_nr_sub;
	std::string device_serial_nr;
	bool got_device_serial_nr;
	bool auto_subsample; // if true, predict with messages without image data, otherwise update

	ros::Publisher duo_processed_pub;
	dynamic_reconfigure::Server<vio_ros::vio_rosConfig> dynamic_reconfigure_server;

	ros::Publisher pose_pub;
	ros::Publisher vel_pub;
	ros::Publisher timing_SLAM_pub;
	ros::Publisher timing_feature_tracking_pub;
	ros::Publisher timing_total_pub;
	ros::Publisher vis_pub_;

	tf::TransformBroadcaster tf_broadcaster;
	tf::Transform camera_tf;
	tf::Transform body_tf;
	tf::Quaternion cam2body;

	ros::Time prev_time_;
	std::vector<int> update_vec_;
	geometry_msgs::Pose pose;

	unsigned int num_points_;
	bool show_camera_image_;
	int image_visualization_delay;
	RobotState robot_state;
	double dist;
	double last_pos[3];

	ros::Subscriber reset_sub;
	void resetCb(const std_msgs::Empty &msg);

	std::vector<FloatType> h_u_apo;
	std::vector<FloatType> map;
	std::vector<AnchorPose> anchor_poses;

	void vioSensorMsgCb(const vio_ros::VioSensorMsg &msg);
	void deviceSerialNrCb(const std_msgs::String &msg);
	void loadCustomCameraCalibration(const std::string calib_path);
	void update(double dt, const vio_ros::VioSensorMsg &msg, bool debug_publish, bool show_image, bool reset);

	void getIMUData(const sensor_msgs::Imu& imu, VIOMeasurements& meas);

	void updateVis(RobotState &robot_state, std::vector<AnchorPose> &anchor_poses, std::vector<FloatType> &map, std::vector<int> &updateVect, const vio_ros::VioSensorMsg &msg, std::vector<FloatType> &z_l, bool show_image);

	tf::Quaternion camera2world; // the rotation that transforms a vector in the camera frame to one in the world frame

	void dynamicReconfigureCb(vio_ros::vio_rosConfig &config, uint32_t level);

};

#endif /* _LOCALIZATION_H_ */
