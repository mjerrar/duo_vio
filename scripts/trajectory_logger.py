#!/usr/bin/python
from __future__ import print_function, division

import rospy
import sys
from geometry_msgs.msg import Pose
import time


__author__ = 'nicolas'


def cb(data):
    global out_file

    # print(data)

    out_file.write('{}, {}, {}, {}, {}, {}, {}\n'.format(data.position.x, data.position.y, data.position.z, data.orientation.x,data.orientation.y, data.orientation.z, data.orientation.w))


if __name__ == "__main__":
    rospy.init_node("trajectory_logger")

    if len(sys.argv) < 2:
        out_file_name = 'pose.log'
    else:
        out_file_name = sys.argv[1]

    print('Logging to {}'.format(out_file_name))

    out_file = open(out_file_name, 'w')

    rospy.Subscriber("/vio_ros/pose1", Pose, cb, queue_size=1)

    while not rospy.is_shutdown():
        time.sleep(1)

    print('closing')
    out_file.close()