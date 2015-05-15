#ifndef _DUO_POSE_ESTIMATION_H_
#define _DUO_POSE_ESTIMATION_H_

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>

class DuoPoseEstimation
{
public:
  DuoPoseEstimation();
  ~DuoPoseEstimation() = default;

private:
  ros::NodeHandle nh_;

  message_filters::Subscriber<sensor_msgs::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> right_image_sub_;
  
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> time_synchronizer_;

  void synchronized_callback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image);
};

#endif /* _DUO_POSE_ESTIMATION_H_ */
