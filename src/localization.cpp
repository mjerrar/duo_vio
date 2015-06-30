#include "localization.h"
#include "SLAM.h"

#include "klt_point_handling.h"

#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <math.h>
#include <visualization_msgs/Marker.h>

Localization::Localization()
  : nh_("~"), 
    left_image_sub_(nh_, "/left_image", 1),
    right_image_sub_(nh_, "/right_image", 1),
    imu_sub_(nh_, "/imu", 1),
    time_synchronizer_(left_image_sub_, right_image_sub_, imu_sub_, 10),
    process_noise_(3,0.0),
    im_noise_(3,0.0),
    camera_params_(4,0.0),
    camera_info_initialized_(false)
{
  SLAM_initialize();
  emxInitArray_real_T(&h_u_apo_,1);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose",1);
  time_synchronizer_.registerCallback(boost::bind(&Localization::synchronized_callback,
        this, _1, _2, _3));
  camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>("/duo3d_camera/right/camera_info",1,
      &Localization::camera_info_callback,this);
  //point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/vio/features_point_cloud",1); //TODO: add to debug parameter
  path_pub_ = nh_.advertise<nav_msgs::Path>("/vio/SLAM_path",1);
  vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "drone", 0 );

  // Init parameters
  // TODO Check default values and give meaningful names
  nh_.param<bool>("show_tracker_images", show_tracker_images_, false);

  nh_.param<double>("process_noise_1", process_noise_[0], 10.0);
  nh_.param<double>("process_noise_2", process_noise_[1], 10.0);
  nh_.param<double>("process_noise_3", process_noise_[2], 0.0);
 // nh_.param<double>("process_noise_4", process_noise_[3], 0.0);

  nh_.param<double>("im_noise_1", im_noise_[0], 10.0);
  nh_.param<double>("im_noise_2", im_noise_[1], 10.0);
  nh_.param<double>("im_noise_3", im_noise_[2], 10.0);

  int num_points_per_anchor, num_anchors;
  nh_.param<int>("num_points_per_anchor", num_points_per_anchor, 1);
  nh_.param<int>("num_anchors", num_anchors, 32);

  if (num_anchors < 0.0)
  {
    ROS_ERROR("Number of anchors may not be negative!");
    nh_.shutdown();
  }
  else
  {
    num_anchors_ = static_cast<unsigned int>(num_anchors);
  }

  if (num_points_per_anchor < 0.0)
  {
    ROS_ERROR("Number of points per anchors may not be negative!");
    nh_.shutdown();
  }
  else
  {
    num_points_per_anchor_ = static_cast<unsigned int>(num_points_per_anchor);
  }

  update_vec_.assign(num_anchors_,0.0);
}

Localization::~Localization()
{
  emxDestroyArray_real_T(h_u_apo_);
  SLAM_terminate();
}

void Localization::synchronized_callback(const sensor_msgs::ImageConstPtr& left_image,
    const sensor_msgs::ImageConstPtr& right_image,
    const sensor_msgs::ImuConstPtr& imu)
{

  sensor_msgs::MagneticField mag; // TODO Subscribe to mag topic

  if (!camera_info_initialized_)
  {
    ROS_ERROR("Camera info not initialized!");
    return;
  }

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

  // Init time on first call
  if (prev_time_.isZero())
  {
    prev_time_ = left_image->header.stamp;
  }
  
  double dt = (left_image->header.stamp - prev_time_).toSec();
  prev_time_ = left_image->header.stamp;

  geometry_msgs::PoseStamped pose_stamped;
  geometry_msgs::Pose pose;
  pose_stamped.header.stamp = left_image->header.stamp;
  pose_stamped.header.frame_id = "world";

  update(dt, cv_left_image->image, cv_right_image->image, *imu, mag, pose);

  pose_stamped.pose = pose;
  pose_pub_.publish(pose_stamped);

  // Generate and publish pose as transform
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));

  transform.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,
      pose.orientation.z,pose.orientation.w));

  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, /*pose_stamped.header.stamp*/ ros::Time::now(), "world", "SLAM"));

  slam_path_.poses.push_back(pose_stamped);
  slam_path_.header = pose_stamped.header;
  path_pub_.publish(slam_path_);

  updateDronePose();
  visMarker();
}

void Localization::update(double dt, const cv::Mat& left_image, const cv::Mat& right_image, const sensor_msgs::Imu& imu, 
    const sensor_msgs::MagneticField& mag, geometry_msgs::Pose& pose)
{
  //*********************************************************************
  // Point tracking
  //*********************************************************************

  double z_all[num_anchors_ * 3];
  unsigned char update_vec_char[num_anchors_];

  for (size_t i = 0; i < num_anchors_; ++i)
  {
    update_vec_char[i] = update_vec_[i];
  }

  ros::Time tic = ros::Time::now();
  handle_points_klt(left_image,right_image,num_anchors_,z_all,update_vec_char);
  ROS_INFO("Time Tracking: %f", (ros::Time::now() - tic).toSec());

  if (show_tracker_images_)
  {
    display_tracks(left_image, right_image, z_all, update_vec_char);
  }

  double update_vec_array[num_anchors_];
  for (size_t i = 0; i < num_anchors_; ++i)
  {
    update_vec_array[i] = update_vec_char[i];
    if(z_all[3*i] < 0)
      ROS_ERROR("Negative x: %f",z_all[3*i]);
    if(z_all[3*i+1] < 0)
      ROS_ERROR("Negative y: %f",z_all[3*i+1]);
  }
  
  //*********************************************************************
  // SLAM
  //*********************************************************************

  std::vector<double> inertial(9,0.0);
  get_inertial_vector(imu,mag,inertial);

   emxArray_real_T *xt_out; // result
   emxArray_real_T *P_apo_out;

   emxInitArray_real_T(&xt_out,1);
   emxInitArray_real_T(&P_apo_out,2);

  // Update SLAM and get pose estimation
  tic = ros::Time::now();

  //double updateVect[32];
  
  double b_map[96];
  double h_u_apo[96];

  SLAM(update_vec_array, z_all,  &camera_params_[0], dt, &process_noise_[0], &inertial[0], &im_noise_[0], num_points_per_anchor_,num_anchors_, h_u_apo, xt_out, P_apo_out, b_map);

  update_vec_.assign(update_vec_array, update_vec_array + num_anchors_);
  ROS_INFO("Time SLAM: %f", (ros::Time::now() - tic).toSec());

  // Publish feature position in world frame
  //publishPointCloud(anchor_u_out, xt_out,b_map);

  // Set the pose
  pose.position.x = xt_out->data[0];
  pose.position.y = xt_out->data[1];
  pose.position.z = xt_out->data[2];

  pose.orientation.x = xt_out->data[3];
  pose.orientation.y = xt_out->data[4];
  pose.orientation.z = xt_out->data[5];
  pose.orientation.w = xt_out->data[6];

  // TODO Make velocities ex_out[7] .. ex_out[12] available as ROS message

  emxDestroyArray_real_T(xt_out);
  emxDestroyArray_real_T(P_apo_out);

}

void Localization::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& info)
{
  camera_info_initialized_ = true;
  camera_params_[0] = (info->K[0] + info->K[0]) / 2.0; // focal length
  camera_params_[1] = info->K[2]; // Cx
  camera_params_[2] = info->K[4]; // Cy
  camera_params_[3] = -0.1 * info->P[3] / info->P[0]; // baseline
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

void Localization::display_tracks(const cv::Mat& left_image, const cv::Mat& right_image,
    double z_all[], unsigned char status[])
{
  cv::Mat left;
  cv::cvtColor(left_image,left,cv::COLOR_GRAY2BGR);

  cv::Mat right;
  cv::cvtColor(right_image,right,cv::COLOR_GRAY2RGB);


  for (unsigned int i = 0; i < num_anchors_; ++i)
  {
    if (status[i])
    {
      cv::Point left_point(z_all[3*i + 0] - z_all[3*i + 2], z_all[3*i+1]);
      cv::Point right_point(z_all[3*i + 0],z_all[3*i+1]);
      cv::Scalar color_left;
      if (z_all[3*i + 2] > -100)
      {
        color_left = cv::Scalar(0,255,0);
        cv::circle(right, left_point,1,cv::Scalar(0,255,0),2);
        cv::line(left,left_point,right_point,color_left,1);
      }
      else
      {
        color_left = cv::Scalar(0,0,255);
      }
      cv::circle(left, right_point ,1,color_left,2);
    }
  }
  cv::imshow("left image", left);
  cv::imshow("right image", right);
  cv::waitKey(10);
}

// void Localization::publishPointCloud(emxArray_real_T *anchor_u_out, emxArray_real_T *xt_out,emxArray_real_T *map)
// {
//   sensor_msgs::PointCloud features;

//   features.header.frame_id = "world";

//   for(int cnt = 0; cnt < num_points_per_anchor_*num_anchors_; cnt++)
//   {
// 	// int ind_anchor = cnt/num_points_per_anchor_ + 1;

// 	// //13 = numStatesxt, 7 = numStatesPerAnchorxt - num_points_per_anchor_ TODO(Stefan): change to constants or function inputs
// 	// double x_i = xt_out->data[13 + ind_anchor*(7 + num_points_per_anchor_) - (6 + num_points_per_anchor_) - 1];
// 	// double y_i = xt_out->data[13 + ind_anchor*(7 + num_points_per_anchor_) - (5 + num_points_per_anchor_) - 1];
// 	// double z_i = xt_out->data[13 + ind_anchor*(7 + num_points_per_anchor_) - (4 + num_points_per_anchor_) - 1];

// 	// double fq_cw0 = xt_out->data[13 + ind_anchor*(7 + num_points_per_anchor_) - (3 + num_points_per_anchor_) - 1];
//  //    double fq_cw1 = xt_out->data[13 + ind_anchor*(7 + num_points_per_anchor_) - (2 + num_points_per_anchor_) - 1];
// 	// double fq_cw2 = xt_out->data[13 + ind_anchor*(7 + num_points_per_anchor_) - (1 + num_points_per_anchor_) - 1];
// 	// double fq_cw3 = xt_out->data[13 + ind_anchor*(7 + num_points_per_anchor_) - (0 + num_points_per_anchor_) - 1];

// 	// int feature_offset = cnt % num_points_per_anchor_ + 1;
// 	// double rho = xt_out->data[13 + ind_anchor*(7 + num_points_per_anchor_) - (0 + num_points_per_anchor_) + feature_offset - 1];

// 	// tf::Vector3 fp(x_i, y_i, z_i);
// 	// tf::Quaternion q(fq_cw0, fq_cw1, fq_cw2, fq_cw3);
// 	// //tf::Matrix3x3 R_wc(q);

// 	// tf::Transform cam2world;
// 	// cam2world.setOrigin(fp);
// 	// cam2world.setRotation(q);

// 	// tf::Vector3 m((camera_params_[1] - anchor_u_out->data[cnt*2])/camera_params_[0], (camera_params_[2] - anchor_u_out->data[cnt*2 + 1])/camera_params_[0], 1.0f);
// 	// m.normalize();
// 	// m /= rho;

// 	// tf::Vector3 featurePosition = cam2world*m;

// 	geometry_msgs::Point32 point;
// 	point.x = map(cnt*3);
// 	point.y = map(cnt*3+1);
// 	point.z = map(cnt*3+2);

// 	features.points.push_back(point);
//   }

//   point_cloud_pub_.publish(features);
// }



void Localization::updateDronePose(void)
{
  /*tf::Transform camera2drone;
  camera2drone.setOrigin(tf::Vector3(0.0, 0.0, -0.10));
  tf::Quaternion q_c2d;
  q_c2d.setEulerZYX(-M_PI/2, -M_PI/2, 0.0);
  camera2drone.setRotation(q_c2d);
  tf_broadcaster_.sendTransform(tf::StampedTransform(camera2drone, ros::Time::now(), "camera", "drone_base"));*/

  /*tf::Transform slam2drone;
  slam2drone.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q_s2d;
  q_s2d.setEuler(0.0, M_PI, 0.0);// check convention
  slam2drone.setRotation(q_s2d);
  tf_broadcaster_.sendTransform(tf::StampedTransform(slam2drone, ros::Time::now(), "SLAM", "drone_base"));*/

  tf::Transform slam2camera;
  slam2camera.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q_s2d;
  q_s2d.setEuler(0.0, 0.0, 0.0);
  slam2camera.setRotation(q_s2d);
  tf_broadcaster_.sendTransform(tf::StampedTransform(slam2camera, ros::Time::now(), "SLAM", "camera"));

  /*tf::Transform drone2camera;
  drone2camera.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
  tf::Quaternion q_d2c;
  q_d2c.setEuler(M_PI/2, 0.0, M_PI/2);
  drone2camera.setRotation(q_d2c);
  tf_broadcaster_.sendTransform(tf::StampedTransform(drone2camera, ros::Time::now(), "drone_base", "camera"));*/

  tf::Transform camera2drone;
  camera2drone.setOrigin(tf::Vector3(0.0, 0.0, -1.0));
  tf::Quaternion q_d2c;
  q_d2c.setEuler(-M_PI/2, -M_PI/2, 0.0);
  camera2drone.setRotation(q_d2c);
  tf_broadcaster_.sendTransform(tf::StampedTransform(camera2drone, ros::Time::now(), "camera", "drone_base"));
}

void Localization::visMarker(void)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "drone_base";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 1.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;
	marker.scale.x = 0.7;
	marker.scale.y = 0.7;
	marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
	//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	vis_pub_.publish(marker);
}



