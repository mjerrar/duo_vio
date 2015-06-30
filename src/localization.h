#ifndef _LOCALIZATION_H_
#define _LOCALIZATION_H_

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>

#include <opencv2/opencv.hpp>

#include "SLAM.h"

#include <vector>

class Localization
{
public:
  Localization();
  ~Localization();

private:
  ros::NodeHandle nh_;

  message_filters::Subscriber<sensor_msgs::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> right_image_sub_;
  message_filters::Subscriber<sensor_msgs::Imu>   imu_sub_;

  ros::Subscriber camera_info_sub_;
  
  message_filters::TimeSynchronizer
    <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> time_synchronizer_;

  ros::Publisher pose_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  bool camera_info_initialized_;
  ros::Time prev_time_;
  std::vector<double> update_vec_;

  // TODO Init subsequent parameters
  std::vector<double> process_noise_; //qv,qw,qwo,qao
  std::vector<double> im_noise_;
  std::vector<double> camera_params_; //f,Cx,Cy,baseline
  unsigned int num_points_per_anchor_;
  unsigned int num_anchors_;
  bool show_tracker_images_;
  emxArray_real_T *h_u_apo_;

  void synchronized_callback(const sensor_msgs::ImageConstPtr& left_image,
      const sensor_msgs::ImageConstPtr& right_image,
      const sensor_msgs::ImuConstPtr& imu);

  void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& info);

  void update(double dt, const cv::Mat& left_image, const cv::Mat& right_image, const sensor_msgs::Imu& imu, 
      const sensor_msgs::MagneticField& mag, geometry_msgs::Pose& pose);

  void get_inertial_vector(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag, std::vector<double>& inertial_vec);

  void display_tracks(const cv::Mat& left_image, const cv::Mat& right_image, double z_all[],
      unsigned char status[]);

  ros::Publisher point_cloud_pub_;
  void publishPointCloud(emxArray_real_T *anchor_u_out, emxArray_real_T * xt_out);

  void updateDronePose(void);

  ros::Publisher path_pub_;
  nav_msgs::Path slam_path_;
};

#endif /* _LOCALIZATION_H_ */
