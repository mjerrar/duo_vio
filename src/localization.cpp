#include "localization.h"
#include "SLAM.h"

#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>

Localization::Localization()
  : nh_("~"), 
    left_image_sub_(nh_, "/left_image", 1),
    right_image_sub_(nh_, "/right_image", 1),
    imu_sub_(nh_, "/imu", 1),
    time_synchronizer_(left_image_sub_, right_image_sub_, imu_sub_, 10),
    prev_time_(ros::Time::now()),
    update_vec_{0}
{
  SLAM_initialize();
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose",1);
  time_synchronizer_.registerCallback(boost::bind(&Localization::synchronized_callback,
        this, _1, _2, _3));
}

Localization::~Localization()
{
  SLAM_terminate();
}

void Localization::synchronized_callback(const sensor_msgs::ImageConstPtr& left_image,
    const sensor_msgs::ImageConstPtr& right_image,
    const sensor_msgs::ImuConstPtr& imu)
{

  sensor_msgs::MagneticField mag; // TODO Subscribe to mag topic

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

  geometry_msgs::PoseStamped pose_stamped;
  geometry_msgs::Pose pose;
  pose_stamped.header.stamp = left_image->header.stamp;

  update(cv_left_image->image, cv_right_image->image, *imu, mag, pose);

  pose_stamped.pose = pose;
  pose_pub_.publish(pose);

  // Generate and publish pose as transform
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));

  tf::Quaternion q(0,0,0,1);
  // TODO Set quaternion from pose estimation
  // q.setEuler(yaw, pitch, roll);
  // q.setRPY(roll, pitch, yaw);
  // q.setRotation(axis, angle)
  transform.setRotation(q);

  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, pose_stamped.header.stamp, "map", "base"));
}

void Localization::update(const cv::Mat& left_image, const cv::Mat& right_image, const sensor_msgs::Imu& imu, 
    const sensor_msgs::MagneticField& mag, geometry_msgs::Pose& pose)
{
  // Get time
  ros::Time current = ros::Time::now();
  double dt = (current - prev_time_).toSec();
  prev_time_ = current;

  // TODO Insert point tracker here
  double z_all[96] = {0};
  double camera_params[4] = {0}; //f,Cx,Cy,baseline
  double process_noise[4] = {0}; //qv,qw,qwo,qao
  double im_noise[3] = {0};
  double num_points_per_anchor;
  double num_anchors;

  std::vector<double> imu_vec(9,0.0);
  get_imu_array(imu,mag,imu_vec);

  emxArray_real_T *h_u_apo;
  emxArray_real_T *xt_out; // result
  emxArray_real_T *anchor_u_out;
  emxArray_real_T *anchor_pose_out;

  emxInitArray_real_T(&h_u_apo,1);
  emxInitArray_real_T(&xt_out,1);
  emxInitArray_real_T(&anchor_u_out,1);
  emxInitArray_real_T(&anchor_pose_out,1);

  // Update SLAM and get pose estimation
  SLAM(update_vec_, z_all, camera_params, dt, process_noise, &imu_vec[0], im_noise, num_points_per_anchor, num_anchors,
      h_u_apo, xt_out, update_vec_, anchor_u_out, anchor_pose_out);

  emxDestroyArray_real_T(h_u_apo);
  emxDestroyArray_real_T(xt_out);
  emxDestroyArray_real_T(anchor_u_out);
  emxDestroyArray_real_T(anchor_pose_out);

}

void Localization::get_imu_array(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag, std::vector<double>& imu_vec )
{
  // TODO Check signs of angular velocities
  imu_vec.at(0) = imu.angular_velocity.x;
  imu_vec.at(1) = -imu.angular_velocity.y;
  imu_vec.at(2) = imu.angular_velocity.z;

  // TODO Check signs of linear acceleration
  imu_vec.at(3) = imu.linear_acceleration.x;
  imu_vec.at(4) = -imu.linear_acceleration.y;
  imu_vec.at(5) = -imu.linear_acceleration.z;

  imu_vec.at(6) = mag.magnetic_field.x;
  imu_vec.at(7) = mag.magnetic_field.y;
  imu_vec.at(8) = mag.magnetic_field.z;
}
