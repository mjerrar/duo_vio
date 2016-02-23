#!/usr/bin/python
from __future__ import print_function, division
import os
import rospy
from std_msgs.msg import Empty

__author__ = 'nicolas'


if __name__ == "__main__":
    rospy.init_node("vio_reset")

    ros_master_uri = os.environ['ROS_MASTER_URI']

    reset_pub = rospy.Publisher("vio_ros/reset", Empty, queue_size=1)
    msg = Empty()
    while not rospy.is_shutdown():
        raw_input('Press enter to reset VIO at {}:'.format(ros_master_uri))
        reset_pub.publish(msg)