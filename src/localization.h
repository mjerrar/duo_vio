#ifndef _LOCALIZATION_H_
#define _LOCALIZATION_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_broadcaster.h>
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
#include "stereoParameters.h"

#include "onboard_localization/ControllerOut.h"

#include <dynamic_reconfigure/server.h>
#include <vio_ros/controllerConfig.h>

#include <duo3d_ros/Duo3d.h>

#include <vector>
#include <cstdio>

#include <boost/circular_buffer.hpp>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"


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
  ros::Time last_debug_publish;

  stereoParameters cameraParams;
  ros::NodeHandle nh_;

  ros::Subscriber combined_sub;
  ros::Subscriber mavros_imu_sub_;
  ros::Subscriber mavros_mag_sub_;
  ros::Subscriber mavros_pressure_sub_;
  ros::Subscriber joy_sub_;

  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher controller_pub;

  ros::Publisher debug_imu_pub_; // publisher that publishes the imu data that is fed to SLAM, for rosbags
  ros::Publisher debug_img_pub_; // publisher that publishes the images that are fed to SLAM, for rosbags

  tf::TransformBroadcaster tf_broadcaster_;

  dynamic_reconfigure::Server<vio_ros::controllerConfig> dynamic_reconfigure_server;

  ros::Time prev_time_;
  std::vector<int> update_vec_;

  std::vector<double> process_noise_; // qv,qw,qwo,qao
  std::vector<double> im_noise_; // x, y noise
  std::vector<double> controller_gains; // position p, d; yaw p gains

  unsigned int num_points_per_anchor_;
  unsigned int num_anchors_;
  unsigned int num_points_;
  bool show_tracker_images_;
  emxArray_real_T *h_u_apo_;

  void duo3dCb(const duo3d_ros::Duo3d& msg);
  void mavrosImuCb(const sensor_msgs::Imu msg);
  void mavrosMagCb(const sensor_msgs::MagneticField msg);
  void mavrosPressureCb(const sensor_msgs::FluidPressure msg);
  void joystickCb(const sensor_msgs::Joy::ConstPtr& joy);

  void update(double dt, const cv::Mat& left_image, const cv::Mat& right_image, const sensor_msgs::Imu& imu,
      const sensor_msgs::MagneticField& mag, geometry_msgs::Pose& pose, geometry_msgs::Twist& velocity, bool debug_publish);

  void getIMUData(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag, std::vector<double>& inertial_vec);

  void displayTracks(const cv::Mat& left_image, double z_all_l[], double z_all_r[],
		  std::vector<int> status, emxArray_real_T *h_u = NULL);

   ros::Publisher point_cloud_pub_;
   void publishPointCloud(double * map );

  void updateDronePose(bool debug_publish);

  ros::Publisher path_pub_;
  nav_msgs::Path slam_path_;
  ros::Publisher vis_pub_;

  sensor_msgs::Imu mavros_imu_data_;
  boost::circular_buffer<sensor_msgs::Imu> mavros_imu_data_buffer_;
  sensor_msgs::MagneticField mavros_mag_data_;
  sensor_msgs::FluidPressure mavros_pressure_data_;

  void visMarker(void);
  void dynamicReconfigureCb(vio_ros::controllerConfig &config, uint32_t level);

};

#endif /* _LOCALIZATION_H_ */
