#!/usr/bin/env python
__author__ = 'nicolas'

import os
import sys
import argparse
import rosbag
from vio_ros.msg import VioSensorMsg
from std_msgs.msg import Empty
import rospy

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Invalid use of {}. Provide at least one argument that is a ros bag file.'.format(sys.argv[0]))
        sys.exit(-1)

    inbag_name = sys.argv[1]
    inbag = rosbag.Bag(inbag_name, 'r')

    outbag_cnt = 1
    outbag = rosbag.Bag('{}_{}.bag'.format(inbag_name[:-4], outbag_cnt), 'w')

    out_log = open(inbag_name[:-4] + '.info', 'w')

    start_time = None

    for topic, msg, t in inbag.read_messages():
        if start_time is None:
            start_time = t
        if topic == '/vio_ros/reset':
            print('{}\nFound reset message at time {} s, splitting bag, this was bag nr {}.'.format('='*100, (t-start_time)*1e-9, outbag_cnt))
            out_log.write('{}\nFound reset message at time {} s, splitting bag, this was bag nr {}.'.format('='*100, (t-start_time)*1e-9, outbag_cnt))
            outbag.close()
            outbag_cnt += 1
            outbag = rosbag.Bag('{}_{}.bag'.format(inbag_name[:-4], outbag_cnt), 'w')
            vis_reset = Empty()
            outbag.write('/vio_vis/reset', vis_reset, t)
        else:
            if topic == '/rosout_agg':
                if 'Last position' in msg.msg:
                    print('Found message about last position at time {} s:\n{}'.format((t-start_time)*1e-9, msg.msg))
                    out_log.write('Found message about last position at time {} s:\n{}'.format((t-start_time)*1e-9, msg.msg))
                elif 'Trajectory length' in msg.msg:
                    print('Found message about trajectory length at time {} s:\n{}'.format((t-start_time)*1e-9, msg.msg))
                    out_log.write('Found message about trajectory length at time {} s:\n{}'.format((t-start_time)*1e-9, msg.msg))
            outbag.write(topic, msg, t)

    outbag.close()

    if outbag_cnt == 1:
        print('Did not find any reset signals in this bag. Not splitting it up.')
        os.remove('{}_{}.bag'.format(inbag_name[:-4], outbag_cnt))
    else:
        print('Wrote to the following bags:')
        for i in range(1, outbag_cnt+1):
            print('{}_{}.bag'.format(inbag_name[:-4], i))
