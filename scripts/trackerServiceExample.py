#!/usr/bin/python

#   Copyright (c) 2015-2016 AIT, ETH Zurich. All rights reserved.
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

from __future__ import print_function, division

import rospy
import cv2
import cv_bridge
from vio_ros.srv import *
from vio_ros.msg import VioSensorMsg
from sensor_msgs.msg import Image


def cb(data):
    global update_vect
    pass

    track_features = rospy.ServiceProxy('track_features', TrackFeatures)
    resp = track_features(left_image=data.left_image, right_image=Image(encoding='mono8'), update_vect=update_vect, stereo=0)
    update_vect = [1 if x == 1 or x == 2 else 2 for x in resp.update_vect]

    cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data.left_image, "mono8")
    cv_image_c = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

    for i in range(0, len(update_vect)):
        if update_vect[i]:
            cv2.circle(cv_image_c, (int(resp.features_l[i*2]), int(resp.features_l[i*2+1])), 2, (0, 255, 0), -1)

    cv2.imshow('Tracked features', cv_image_c)
    cv2.waitKey(1)


if __name__ == "__main__":

    rospy.init_node("tracker_service_example")

    print('Waiting for service...')
    rospy.wait_for_service('track_features')
    print('... done')

    update_vect = [2]*100

    rospy.Subscriber("/vio_sensor", VioSensorMsg, cb, queue_size=1)

    rospy.spin()

