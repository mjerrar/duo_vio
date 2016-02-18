#!/usr/bin/python
from __future__ import print_function, division

__author__ = 'nicolas'

import sys
import rosbag
import rospy
from sensor_msgs.msg import Image
from vio_ros.msg import VioSensorMsg
import time
import numpy as np
import genpy


def vio_sensor_cb(data):
    global left_pub, right_pub

    left_pub.publish(data.left_image)
    right_pub.publish(data.right_image)


if __name__ == "__main__":

    rospy.init_node('image_publisher')

    left_pub = rospy.Publisher("left_image", Image, queue_size=1)
    right_pub = rospy.Publisher("right_image", Image, queue_size=1)
    rospy.Subscriber("/vio_sensor", VioSensorMsg, vio_sensor_cb, queue_size=1)

    rospy.spin()

