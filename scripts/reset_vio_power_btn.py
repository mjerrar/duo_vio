#!/usr/bin/python
from __future__ import print_function, division
import os
import rospy
from std_msgs.msg import Empty

__author__ = 'nicolas'


if __name__ == "__main__":
    rospy.init_node("vio_reset")

    reset_pub = rospy.Publisher("vio_ros/reset", Empty, queue_size=1)
    msg = Empty()
    while not rospy.is_shutdown():
        if os.path.isfile('/home/odroid/request_reset'):
            reset_pub.publish(msg)
            os.remove('/home/odroid/request_reset')
            rospy.loginfo('Got reset request from power button')
        rospy.sleep(1)