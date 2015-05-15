#include "duo_pose_estimation.h"

#include <geometry_msgs/PoseStamped.h>

DuoPoseEstimation::DuoPoseEstimation()
  : nh_("~"), 
    left_image_sub_(nh_, "/left_image", 1),
    right_image_sub_(nh_, "/right_image", 1),
    time_synchronizer_(left_image_sub_, right_image_sub_, 10)
{
  time_synchronizer_.registerCallback(boost::bind(&DuoPoseEstimation::synchronized_callback, this, _1, _2));
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose",1);
}

void DuoPoseEstimation::synchronized_callback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image)
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = left_image->header.stamp;

  // The images are synchronized according to their time stamps
  // TODO Insert pose estimation here

  pose_pub_.publish(pose);

  // Generate and publish pose as transform
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));

  tf::Quaternion q(0,0,0,1);
  // TODO Set quaternion from pose estimation
  // q.setEuler(yaw, pitch, roll);
  // q.setRPY(roll, pitch, yaw);
  // q.setRotation(axis, angle)
  transform.setRotation(q);

  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, pose.header.stamp, "map", "base"));
}
