#include "duo_pose_estimation.h"

DuoPoseEstimation::DuoPoseEstimation()
  : nh_("~"), 
    left_image_sub_(nh_, "/left_image", 1),
    right_image_sub_(nh_, "/right_image", 1),
    time_synchronizer_(left_image_sub_, right_image_sub_, 10)
{
  time_synchronizer_.registerCallback(boost::bind(&DuoPoseEstimation::synchronized_callback, this, _1, _2));
}

void DuoPoseEstimation::synchronized_callback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image)
{

}
