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
# File: integrate_vis_path.py
# Created on: 09.03.16
# Author: Nicolas de Palezieux

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
