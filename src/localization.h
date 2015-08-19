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
#include "cameraParameters.h"

#include "onboard_localization/ControllerOut.h"

#include <duo3d_ros/Duo3d.h>

#include <vector>
#include <cstdio>

class Localization
{
public:
  Localization();
  ~Localization();

private:
  double t_avg;
  double debug_publish_delay;
  bool SLAM_reset_flag;
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

  tf::TransformBroadcaster tf_broadcaster_;

  ros::Time prev_time_;
  std::vector<int> update_vec_;

  // TODO Init subsequent parameters
  std::vector<double> process_noise_; //qv,qw,qwo,qao
  std::vector<double> im_noise_;
  std::vector<double> camera_params_; //f,Cx,Cy,baseline
  unsigned int num_points_per_anchor_;
  unsigned int num_anchors_;
  unsigned int num_points_;
  bool show_tracker_images_;
  emxArray_real_T *h_u_apo_;

  void duo3d_callback(const duo3d_ros::Duo3d& msg);
  void mavrosImuCb(const sensor_msgs::Imu msg);
  void mavrosMagCb(const sensor_msgs::MagneticField msg);
  void mavrosPressureCb(const sensor_msgs::FluidPressure msg);
  void joystickCb(const sensor_msgs::Joy::ConstPtr& joy);

  void update(double dt, const cv::Mat& left_image, const cv::Mat& right_image, const sensor_msgs::Imu& imu,
      const sensor_msgs::MagneticField& mag, geometry_msgs::Pose& pose, geometry_msgs::Twist& velocity, bool debug_publish);

  void get_inertial_vector(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag, std::vector<double>& inertial_vec);

  void display_tracks(const cv::Mat& left_image, double z_all_l[], double z_all_r[],
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

  void visMarker(void);
};

#endif /* _LOCALIZATION_H_ */
