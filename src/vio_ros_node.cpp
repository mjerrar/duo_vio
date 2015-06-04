#include <ros/ros.h>

#include "localization.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vio_ros_node");
  
  Localization estimation();

  ros::spin();
  return 0;
}
