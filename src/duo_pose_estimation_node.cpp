#include <ros/ros.h>

#include "duo_pose_estimation.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "duo_pose_estimation_node");

  DuoPoseEstimation estimation;

  ros::spin();
  return 0;
}
