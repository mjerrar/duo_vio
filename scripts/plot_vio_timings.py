#!/usr/bin/env python
__author__ = 'nicolas'

import os
import sys
import argparse
import rosbag
from vio_ros.msg import VioSensorMsg
import rospy
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Invalid use of {}. Provide at least one argument that is a bag file containing vio_ros/VioSensorMsg messages'.format(sys.argv[0]))
        sys.exit(-1)

    bag_name = sys.argv[1]
    bag = rosbag.Bag(bag_name, 'r')
    times = []
    for topic, msg, t in bag.read_messages(topics=['/vio_sensor']):
        times.extend([float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs)*1e-9])

    time_diffs = [j-i for i, j in zip(times[:-1], times[1:])]
    window = pg.GraphicsWindow(title=bag_name)
    window.show()
    plot = window.addPlot(title='Message times')
    plot.plot(times, pen=(0, 255, 255))

    window.nextRow()
    plot = window.addPlot(title='Message time differences')
    plot.plot(time_diffs, pen=(0, 255, 255))
    QtGui.QApplication.instance().exec_()
