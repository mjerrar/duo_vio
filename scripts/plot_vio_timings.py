#!/usr/bin/python

#   Copyright (c) 2016 AIT, ETH Zurich. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name AIT nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# File: plot_vio_timings.py
# Created on: 09.03.16
# Author: Nicolas de Palezieux

from __future__ import print_function, division

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
