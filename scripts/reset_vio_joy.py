#!/usr/bin/python
from __future__ import print_function, division
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
import datetime

__author__ = 'nicolas'


def joy_cb(data):
    global reset_pub, last_time, timeout
    if any(data.buttons) and last_time + datetime.timedelta(seconds=timeout) < datetime.datetime.now():
        reset_pub.publish(Empty())
        last_time = datetime.datetime.now()

if __name__ == "__main__":
    rospy.init_node("vio_reset")

    last_time = datetime.datetime.now()
    timeout = 0.5  # min seconds between two reset sends

    reset_pub = rospy.Publisher("vio_ros/reset", Empty, queue_size=1)

    rospy.Subscriber("/joy", Joy, joy_cb, queue_size=1)

    rospy.spin()
