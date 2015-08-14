#include "localization.h"
#include "SLAM.h"

#include "klt_point_handling.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <math.h>
#include <stdio.h>
#include <visualization_msgs/Marker.h>

Localization::Localization()
: nh_("~"),
process_noise_(4,0.0),
im_noise_(4,0.0),
camera_params_(4,0.0),
t_avg(0.0),
plot_cnt(0)
{
    SLAM_initialize();
    emxInitArray_real_T(&h_u_apo_,1);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose",1);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/velocity",1);
    combined_sub = nh_.subscribe("/duo3d_camera/combined",1,
    		&Localization::synchronized_callback,this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/vio/features_point_cloud",1); //TODO: add to debug parameter
    path_pub_ = nh_.advertise<nav_msgs::Path>("/vio/SLAM_path",1);
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "drone", 0 );

    mavros_imu_sub_ = nh_.subscribe("/mavros/imu/data", 1,
            &Localization::mavrosImuCb, this);
    mavros_mag_sub_ = nh_.subscribe("/mavros/imu/mag", 1,
            &Localization::mavrosMagCb, this);
    mavros_pressure_sub_ = nh_.subscribe("/mavros/imu/atm_pressure", 1,
            &Localization::mavrosPressureCb, this);

    // Init parameters
    // TODO Check default values and give meaningful names
    nh_.param<bool>("show_tracker_images", show_tracker_images_, false);

    nh_.param<double>("process_noise_1", process_noise_[0], 100);
    nh_.param<double>("process_noise_2", process_noise_[1], 1);
    nh_.param<double>("process_noise_3", process_noise_[2], 0.0);
    // nh_.param<double>("process_noise_4", process_noise_[3], 0.0);

    nh_.param<double>("im_noise_1", im_noise_[0], 2.0);
    nh_.param<double>("im_noise_2", im_noise_[1], 2.0);
    nh_.param<double>("im_noise_3", im_noise_[2], 2.0);
    nh_.param<double>("im_noise_3", im_noise_[3], 2.0);


    int num_points_per_anchor, num_anchors;
    nh_.param<int>("num_points_per_anchor", num_points_per_anchor, 1);
    nh_.param<int>("num_anchors", num_anchors, 32);

    if (num_anchors < 0.0)
    {
        ROS_ERROR("Number of anchors must not be negative!");
        nh_.shutdown();
    }
    else
    {
        num_anchors_ = static_cast<unsigned int>(num_anchors);
    }

    if (num_points_per_anchor < 0.0)
    {
        ROS_ERROR("Number of points per anchors must not be negative!");
        nh_.shutdown();
    }
    else
    {
        num_points_per_anchor_ = static_cast<unsigned int>(num_points_per_anchor);
    }

    num_points_ = num_anchors_*num_points_per_anchor_;

    update_vec_.assign(num_points_, 0);
//    fill(update_vec_.begin(), update_vec_.begin() + num_points_per_anchor_, 2); // request new features for the first anchor initially
}

Localization::~Localization()
{
    emxDestroyArray_real_T(h_u_apo_);
    SLAM_terminate();


}

void Localization::synchronized_callback(const duo3d_ros::Duo3d& msg)
{
	double tic_total = ros::Time::now().toSec();
    sensor_msgs::MagneticField mag; // TODO Subscribe to mag topic

    cv_bridge::CvImagePtr cv_left_image;
    cv_bridge::CvImagePtr cv_right_image;
    try
    {
        cv_left_image = cv_bridge::toCvCopy(msg.left_image, "mono8");
        cv_right_image = cv_bridge::toCvCopy(msg.right_image,"mono8");
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
        prev_time_ = msg.header.stamp;
    }


    prev_time_ = msg.header.stamp;

    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist velocity;
    pose_stamped.header.stamp = msg.header.stamp;
    pose_stamped.header.frame_id = "world";
    double dt = (msg.header.stamp - prev_time_).toSec();
    update(dt, cv_left_image->image, cv_right_image->image, msg.imu, mag, pose, velocity);

    pose_stamped.pose = pose;
    pose_pub_.publish(pose_stamped);
    velocity_pub_.publish(velocity);

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

double time_measurement = ros::Time::now().toSec() - tic_total;

    t_avg=0.05*time_measurement+(1-0.05)*t_avg;
  // printf("\nMax duration: %f ms. Min frequency: %f Hz\n", t_avg, 1/t_avg);
}

void Localization::mavrosImuCb(const sensor_msgs::Imu msg)
{
  mavros_imu_data_ = msg;
}

void Localization::mavrosMagCb(const sensor_msgs::MagneticField msg)
{
  mavros_mag_data_ = msg;
}

void Localization::mavrosPressureCb(const sensor_msgs::FluidPressure msg)
{
  mavros_pressure_data_ = msg;
}

void Localization::update(double dt, const cv::Mat& left_image, const cv::Mat& right_image, const sensor_msgs::Imu& imu,
    const sensor_msgs::MagneticField& mag, geometry_msgs::Pose& pose, geometry_msgs::Twist& velocity)
{

    //*********************************************************************
    // Point tracking
    //*********************************************************************
	int measurementDim = 4;

//    double z_all_l[num_anchors_ * measurementDim];
//    double z_all_r[num_anchors_ * measurementDim];
    std::vector<double> z_all_l(num_points_*2, 0.0);
    std::vector<double> z_all_r(num_points_*2, 0.0);

    ros::Time tic = ros::Time::now();
    ros::Time tic_total = tic;

    handle_points_klt(left_image, right_image, z_all_l, z_all_r, update_vec_);

    double update_vec_array[num_points_];
    double update_vec_array_out[num_points_];
    for (size_t i = 0; i < num_points_; ++i)
    {
    	update_vec_array[i] = update_vec_[i];
    }

    //ROS_INFO("Time point tracker: %6.2f ms", (ros::Time::now() - tic).toSec()*1000);

    //*********************************************************************
    // SLAM
    //*********************************************************************

    std::vector<double> inertial(23,0.0);
    get_inertial_vector(imu,mag,inertial);

    emxArray_real_T *xt_out; // result
    emxArray_real_T *P_apo_out;
    emxArray_real_T *h_u_apo;
    emxArray_real_T *map;

    emxInitArray_real_T(&xt_out,1);
    emxInitArray_real_T(&P_apo_out,2);
    emxInitArray_real_T(&h_u_apo,1);
    emxInitArray_real_T(&map,2);

    // Update SLAM and get pose estimation
    tic = ros::Time::now();

//    double b_map[96];

//    clock_t t1 = clock();
    SLAM(update_vec_array, &z_all_l[0], &z_all_r[0], dt, &process_noise_[0], &inertial[0], &im_noise_[0], num_points_per_anchor_,num_anchors_, h_u_apo, xt_out, P_apo_out, map);
//    clock_t t2 = clock();
//    printf("SLAM took: %d clicks, %f msec\n", int(t2 - t1), 1000*float(t2 - t1)/CLOCKS_PER_SEC);

    printf("gyro_imu: %f,%f %f \n",inertial[13],inertial[14],inertial[15]);
    printf("gyro_duo: %f,%f %f \n",h_u_apo[0],h_u_apo[1],h_u_apo[2]);
//    update_vec_.assign(update_vec_array_out, update_vec_array_out + num_anchors_);
    for(int i = 0; i < update_vec_.size(); i++)
    {
    	update_vec_[i] = update_vec_array[i];
    }

    if (plot_cnt%1==0)
    {
    	if (show_tracker_images_)
    	{
    		display_tracks(left_image, &z_all_l[0], &z_all_r[0], update_vec_, h_u_apo);
    	}
    }
    plot_cnt++;
    //ROS_INFO("Time SLAM         : %6.2f ms", (ros::Time::now() - tic).toSec()*1000);

    // Publish feature position in world frame
    publishPointCloud(map->data);

    // Set the pose
    pose.position.x = xt_out->data[0];
    pose.position.y = xt_out->data[1];
    pose.position.z = xt_out->data[2];

    pose.orientation.x = xt_out->data[3];
    pose.orientation.y = xt_out->data[4];
    pose.orientation.z = xt_out->data[5];
    pose.orientation.w = xt_out->data[6];

    velocity.linear.x = xt_out->data[7];
    velocity.linear.y = xt_out->data[8];
    velocity.linear.z = xt_out->data[9];

    velocity.angular.x = xt_out->data[10];
    velocity.angular.y = xt_out->data[11];
    velocity.angular.z = xt_out->data[12];

    emxDestroyArray_real_T(xt_out);
    emxDestroyArray_real_T(P_apo_out);
    emxDestroyArray_real_T(h_u_apo);
    emxDestroyArray_real_T(map);


}

void Localization::get_inertial_vector(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag, std::vector<double>& inertial_vec)
{
    // TODO Check signs of angular velocities
    inertial_vec.at(0) = +imu.angular_velocity.x;
    inertial_vec.at(1) = -imu.angular_velocity.y;
    inertial_vec.at(2) = +imu.angular_velocity.z;

    // TODO Check signs of linear acceleration
    inertial_vec.at(3) = +imu.linear_acceleration.x*9.81;
    inertial_vec.at(4) = -imu.linear_acceleration.y*9.81;
    inertial_vec.at(5) = -imu.linear_acceleration.z*9.81;

    inertial_vec.at(6) = +mag.magnetic_field.x;
    inertial_vec.at(7) = +mag.magnetic_field.y;
    inertial_vec.at(8) = +mag.magnetic_field.z;

    inertial_vec.at(9) = mavros_pressure_data_.fluid_pressure;

    inertial_vec.at(10) = mavros_mag_data_.magnetic_field.x;
    inertial_vec.at(11) = mavros_mag_data_.magnetic_field.y;
    inertial_vec.at(12) = mavros_mag_data_.magnetic_field.z;

    inertial_vec.at(13) = mavros_imu_data_.angular_velocity.x;
    inertial_vec.at(14) = mavros_imu_data_.angular_velocity.y;
    inertial_vec.at(15) = mavros_imu_data_.angular_velocity.z;

    inertial_vec.at(16) = mavros_imu_data_.linear_acceleration.x;
    inertial_vec.at(17) = mavros_imu_data_.linear_acceleration.y;
    inertial_vec.at(18) = mavros_imu_data_.linear_acceleration.z;

    //quaternion from attitude controller on FMU TODO: add this to FMU and mavros

    inertial_vec.at(19) = 0.0;
    inertial_vec.at(20) = 0.0;
    inertial_vec.at(21) = 1.0;
    inertial_vec.at(22) = 0.0;

}

void Localization::display_tracks(const cv::Mat& left_image,
    double z_all_l[], double z_all_r[], std::vector<int> status, emxArray_real_T *h_u)
{
    cv::Mat left;
    cv::cvtColor(left_image,left,cv::COLOR_GRAY2BGR);

    cv::Scalar color_left = cv::Scalar(0,140,255);
    cv::Scalar color_right = cv::Scalar(255,140,0);
    cv::Scalar color_left_pred = cv::Scalar(127,0,127);


    for (unsigned int i = 0; i < num_points_; ++i)
    {
        if (status[i])
        {
            cv::Point left_point(z_all_l[2*i + 0], z_all_l[2*i + 1]);

            std::stringstream ss;
            ss << i+1;
            cv::putText(left, ss.str(), left_point, 1, 1, cvScalar(0,0,255), 1, CV_AA);

            cv::circle(left, left_point, 2, color_left, 2);

            if (z_all_r[2*i + 0] > 0) // plot stereo measurement if available
            {
            	cv::Point right_point(z_all_r[2*i + 0], z_all_r[2*i + 1]);
            	cv::circle(left, right_point, 2, color_right, 2);
            	cv::line(left, left_point, right_point, color_right, 1);
            }

            if(!(h_u == NULL))
            {
            	if (h_u->data[4*i + 0] > -100)
            	{
            		cv::Point left_point_pred(h_u->data[4*i + 0], h_u->data[4*i + 1]);
            		cv::Point right_point_pred(h_u->data[4*i + 2], h_u->data[4*i + 3]);
            		cv::circle(left, left_point_pred, 1, color_left_pred, 2);
            		cv::line(left, left_point, left_point_pred, color_left_pred, 1);
            	}
            }
        }
    }
    cv::imshow("left image", left);
    cv::waitKey(10);
}

void Localization::publishPointCloud(double *map)
    {
    sensor_msgs::PointCloud features;

    features.header.frame_id = "world";
    features.header.stamp = ros::Time::now();

    for(int cnt = 0; cnt < num_points_per_anchor_*num_anchors_; cnt++)
    {
            geometry_msgs::Point32 point;
            point.x = map[cnt*3];
            point.y = map[cnt*3+1];
            point.z = map[cnt*3+2];

            features.points.push_back(point);
    }

    point_cloud_pub_.publish(features);
}



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
    camera2drone.setOrigin(tf::Vector3(0.0, 0.0, -0.045));
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
        marker.scale.x = 0.09;
        marker.scale.y = 0.045;
        marker.scale.z = 0.01;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        vis_pub_.publish(marker);
}



