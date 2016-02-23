#!/usr/bin/python
from __future__ import print_function, division

import rospy
from nav_msgs.msg import Path
import math


def path_cb(data):
    global path_length, last_position
    path_length = 0
    for first, second in zip(data.poses[:-1], data.poses[1:]):
        path_length += math.sqrt((second.pose.position.x - first.pose.position.x)**2 + (second.pose.position.y - first.pose.position.y)**2 + (second.pose.position.z - first.pose.position.z)**2)

    print('path length {}'.format(path_length))

    if len(data.poses):
        last_position = data.poses[-1].pose.position
        print('last position {}, {}, {}'.format(last_position.x, last_position.y, last_position.z))


if __name__ == "__main__":
    rospy.init_node("vio_reset")

    path_length = 0
    last_position = None

    rospy.Subscriber("vio_vis/rviz/robot_path", Path, path_cb, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
