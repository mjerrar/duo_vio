#include <ros/ros.h>

#include "DuoVio.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vio_ros");
  
  DuoVio duo_vio;

  ros::spin();
  return 0;
}
