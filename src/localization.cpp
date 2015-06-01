#include "localization.h"
#include "SLAM.h"

#include "pointextraction.h"

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
  pointextraction_initialize();
  emxInitArray_real_T(&h_u_apo_,1);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose",1);
  time_synchronizer_.registerCallback(boost::bind(&Localization::synchronized_callback,
        this, _1, _2, _3));

  // Init parameters
  // TODO Check default values and give meaningful names
  nh_.param<double>("process_noise_1", process_noise[0], 10.0);
  nh_.param<double>("process_noise_2", process_noise[1], 10.0);
  nh_.param<double>("process_noise_3", process_noise[2], 0.0);
  nh_.param<double>("process_noise_4", process_noise[3], 0.0);


  nh_.param<double>("im_noise_1", im_noise[0], 10.0);
  nh_.param<double>("im_noise_2", im_noise[1], 10.0);
  nh_.param<double>("im_noise_3", im_noise[2], 10.0);


  nh_.param<int>("num_points_per_anchor", num_points_per_anchor, 1);
  nh_.param<int>("num_anchors", num_anchors, 32);
}

Localization::~Localization()
{
  emxDestroyArray_real_T(h_u_apo_);
  SLAM_terminate();
  pointextraction_terminate();
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
  double bin_x_ = 4;
  double bin_y_ = 4;
  double border_ = 10;
  double min_distance_ = 12;
  int h_u_apo_size = num_anchors * 3;
  double z_all[num_anchors * 3];
  double use_disparity[32];
  double pts_l_arr[64];
  double pts_r_arr[64];

  std::vector<unsigned char>left_image_vector;
  std::vector<unsigned char>right_image_vector;
  left_image_vector.assign(left_image.datastart, left_image.dataend);
  left_image_vector.assign(right_image.datastart, right_image.dataend);
  
  double h_u_apo_data[num_anchors * 3];
  
  pointextraction(&right_image_vector[0], &left_image_vector[0], update_vec_, num_anchors, bin_x_, bin_y_, border_, min_distance_, false,
      h_u_apo_data, &h_u_apo_size, pts_r_arr, pts_l_arr, use_disparity, z_all);

  h_u_apo_->data = h_u_apo_data;
  *h_u_apo_->size = h_u_apo_size;
  
  //*********************************************************************
  // SLAM
  //*********************************************************************

  std::vector<double> inertial(9,0.0);
  get_inertial_vector(imu,mag,inertial);

  emxArray_real_T *xt_out; // result
  emxArray_real_T *anchor_u_out;
  emxArray_real_T *anchor_pose_out;
  emxArray_real_T *P_apo;

  emxInitArray_real_T(&xt_out,1);
  emxInitArray_real_T(&anchor_u_out,1);
  emxInitArray_real_T(&anchor_pose_out,1);
  emxInitArray_real_T(&P_apo,1);

  // Update SLAM and get pose estimation
  SLAM(update_vec_, z_all, camera_params, dt, process_noise, &inertial[0], im_noise, num_points_per_anchor, num_anchors,
      h_u_apo_, xt_out, update_vec_, anchor_u_out, anchor_pose_out, P_apo);

  // Set the pose
  pose.position.x = xt_out->data[0];
  pose.position.y = xt_out->data[1];
  pose.position.z = xt_out->data[2];

  pose.orientation.x = xt_out->data[4];
  pose.orientation.y = xt_out->data[5];
  pose.orientation.z = xt_out->data[6];
  pose.orientation.w = xt_out->data[3];

  // TODO Make velocities ex_out[7] .. ex_out[12] available as ROS message

  emxDestroyArray_real_T(xt_out);
  emxDestroyArray_real_T(anchor_u_out);
  emxDestroyArray_real_T(anchor_pose_out);
  emxDestroyArray_real_T(P_apo);

}

void Localization::get_inertial_vector(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag, std::vector<double>& inertial_vec)
{
  // TODO Check signs of angular velocities
  inertial_vec.at(0) = imu.angular_velocity.x;
  inertial_vec.at(1) = -imu.angular_velocity.y;
  inertial_vec.at(2) = imu.angular_velocity.z;

  // TODO Check signs of linear acceleration
  inertial_vec.at(3) = imu.linear_acceleration.x;
  inertial_vec.at(4) = -imu.linear_acceleration.y;
  inertial_vec.at(5) = -imu.linear_acceleration.z;

  inertial_vec.at(6) = mag.magnetic_field.x;
  inertial_vec.at(7) = mag.magnetic_field.y;
  inertial_vec.at(8) = mag.magnetic_field.z;
}
