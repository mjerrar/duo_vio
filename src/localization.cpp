#include "localization.h"

#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>

Localization::Localization(const std::vector<float>& m_focal, const std::vector<float>& m_cc, const std::vector<float>& m_kc)
  : nh_("~"), 
    left_image_sub_(nh_, "/left_image", 1),
    right_image_sub_(nh_, "/right_image", 1),
    imu_sub_(nh_, "/imu", 1),
    time_synchronizer_(left_image_sub_, right_image_sub_, imu_sub_, 10),
    ekf_tracker_(m_focal[0], m_cc[0], m_cc[1], m_kc, 9, 4, 4, 16, 30)
{
	ekf_tracker_.InitFilter();
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose",1);
  time_synchronizer_.registerCallback(boost::bind(&Localization::synchronized_callback,
        this, _1, _2, _3));
}

void Localization::synchronized_callback(const sensor_msgs::ImageConstPtr& left_image,
    const sensor_msgs::ImageConstPtr& right_image,
    const sensor_msgs::ImuConstPtr& imu)
{

  ROS_INFO("Callback");
  cv_bridge::CvImagePtr cv_left_image;
  cv_bridge::CvImagePtr cv_right_image;
  try
  {
    cv_left_image = cv_bridge::toCvCopy(left_image,"mono8");
    cv_right_image = cv_bridge::toCvCopy(right_image,"mono8");
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

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = left_image->header.stamp;

  // Pose estimation
  float acc[3];
  acc[0] = imu->linear_acceleration.x;
  acc[1] = imu->linear_acceleration.y;
  acc[2] = imu->linear_acceleration.z;
  float ang_vel[3];
  ang_vel[0] = imu->angular_velocity.x;
  ang_vel[1] = imu->angular_velocity.y;
  ang_vel[2] = imu->angular_velocity.z;

  ekf_tracker_.ProcessIMUData(0.033, acc, ang_vel);

  //start the filter processing thread after there is imu data
  ekf_tracker_.Start();

  ekf_tracker_.ProcessFrame(cv_left_image->image);

  ekf_tracker_.GetPosition(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

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
