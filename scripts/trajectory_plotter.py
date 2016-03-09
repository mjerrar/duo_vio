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
# File: trajectory_plotter.py
# Created on: 09.03.16
# Author: Nicolas de Palezieux

from __future__ import print_function, division

__author__ = 'nicolas'

import sys
import rosbag
import rospy
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import tf
import time
import numpy as np
import genpy

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print('Invalid use of {}. Provide exactly one argument that is a ros bag with /vio_ros/pose (and optionally vicon) topics'.format(sys.argv[0]))
        sys.exit(-1)

    rospy.init_node('trajectory_plotter')

    in_bag = rosbag.Bag(sys.argv[1], 'r')

    print(in_bag)
    print('\n')

    paths = dict()

    cam2body = [-0.5, 0.5, -0.5, -0.5]

    start_time = in_bag.get_start_time()
    start_time += 48
    start_time = genpy.Time(start_time)

    for topic, msg, t in in_bag.read_messages(topics=['/vio_ros/pose', '/vicon/Race1/Race1'], start_time=start_time):
        if topic not in paths:
            print('Found new topic: {}'.format(topic))
            paths[topic] = Path()

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = t
        pose_stamped.header.frame_id = 'world'

        if not topic.find('vicon') < 0:
            pose_stamped.pose.position = msg.transform.translation
            pose_stamped.pose.orientation = msg.transform.rotation
            pose_stamped.pose.orientation.w *= -1  # need to convert quaternion from Hamiltonian to JPL

        elif not topic.find('vio_ros') < 0:
            pose_stamped.pose.position = msg.position
            att = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            att = tf.transformations.quaternion_multiply(cam2body, att)
            pose_stamped.pose.orientation.x = att[0]
            pose_stamped.pose.orientation.y = att[1]
            pose_stamped.pose.orientation.z = att[2]
            pose_stamped.pose.orientation.w = att[3]
            # factor = 0.8
            # pose_stamped.pose.position.x *= factor
            # pose_stamped.pose.position.y *= factor
            # pose_stamped.pose.position.z *= factor
            # pose_stamped.pose.position.x += 0.1
            # pose_stamped.pose.position.y += 0.2
            # pose_stamped.pose.position.z += 0.01
        else:
            print('Unexpeced topic: {}'.format(topic))
            continue

        paths[topic].poses.append(pose_stamped)
        paths[topic].header = pose_stamped.header

    # calculate the initial offset between the two frames and transform the VIO poses
    if '/vio_ros/pose' in paths and '/vicon/Race1/Race1' in paths:
        first_vio_pose = paths['/vio_ros/pose'].poses[0].pose
        first_vio_pos = np.array([first_vio_pose.position.x, first_vio_pose.position.y, first_vio_pose.position.z])

        first_vicon_pose = paths['/vicon/Race1/Race1'].poses[0].pose
        first_vicon_pos = np.array([first_vicon_pose.position.x, first_vicon_pose.position.y, first_vicon_pose.position.z])

        offset_pos = first_vicon_pos - first_vio_pos
        print(offset_pos)

        print('quaternions')
        print(first_vio_pose.orientation)
        print('\n')
        print(first_vicon_pose.orientation)

        first_vio_att = tf.transformations.quaternion_matrix([first_vio_pose.orientation.x, first_vio_pose.orientation.y, first_vio_pose.orientation.z, first_vio_pose.orientation.w])
        first_vio_att = first_vio_att[0:3, 0:3]
        print(first_vio_att)

        first_vicon_att = tf.transformations.quaternion_matrix([first_vicon_pose.orientation.x, first_vicon_pose.orientation.y, first_vicon_pose.orientation.z, first_vicon_pose.orientation.w])
        first_vicon_att = first_vicon_att[0:3, 0:3]
        print('\n')
        print(first_vicon_att)

        offset_att = np.transpose(first_vicon_att) * first_vio_att
        offset_att_trafo = np.identity(4)
        offset_att_trafo[:3, :3] = offset_att
        offset_att = tf.transformations.quaternion_from_matrix(offset_att_trafo)

        print(offset_att)

        for pose in paths['/vio_ros/pose'].poses:
            pose.pose.position.x += offset_pos[0]
            pose.pose.position.y += offset_pos[1]
            pose.pose.position.z += offset_pos[2]

    for topic in paths:
        path_topic = "paths{}".format(topic)
        print('Publishing path on {}'.format(path_topic))
        path_pub = rospy.Publisher(path_topic, Path, queue_size=100)
        time.sleep(1)
        path_pub.publish(paths[topic])
