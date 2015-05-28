#ifndef _LOCALIZATION_H_
#define _LOCALIZATION_H_

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <opencv2/opencv.hpp>

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
  
  message_filters::TimeSynchronizer
    <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> time_synchronizer_;

  ros::Publisher pose_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  ros::Time prev_time_;
  double update_vec_[32];

  void synchronized_callback(const sensor_msgs::ImageConstPtr& left_image,
      const sensor_msgs::ImageConstPtr& right_image,
      const sensor_msgs::ImuConstPtr& imu);

  void update(const cv::Mat& left_image, const cv::Mat& right_image, const sensor_msgs::Imu& imu, 
      const sensor_msgs::MagneticField& mag, geometry_msgs::Pose& pose);

  void get_imu_array(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag, std::vector<double>& imu_vec );
};

#endif /* _LOCALIZATION_H_ */
