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
# File: trajectory_logger.py
# Created on: 09.03.16
# Author: Nicolas de Palezieux

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