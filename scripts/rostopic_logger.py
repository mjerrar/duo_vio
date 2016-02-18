#!/usr/bin/python
from __future__ import print_function, division

import rospy
from std_msgs.msg import Float32
import argparse
import collections

__author__ = 'nicolas'


def callback(data, name):
    global data_dict, time_offset
    time = rospy.Time.now().to_nsec() - time_offset
    while time in data_dict:
        rospy.logwarn('Got a message from {} at time {}, but there is already a message from {} for that time'.format(name, time, data_dict[time][1]))
        time += 1
    data_dict[time] = [name, data.data]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Log the timing topics from vio_ros to a text file.',
                                     prog='rosrun vio_ros rostopic_logger.py')
    parser.add_argument('--output_file', '-o', action='store', type=str, nargs=1, help='The name of the log file', default=None)
    parser.add_argument('--topics', '-t', action='store', type=str, nargs='*', help='The topics to log. They must be std_msgs/FloatXX or IntXX', default=None)

    rospy.init_node("rostopic_logger")

    args = parser.parse_args()

    if args.output_file is None:
        rospy.loginfo('No output file provided. Log will be saved in out.log')
        out_file_name = 'out.log'
    else:
        out_file_name = args.output_file[0]

    data_dict = collections.OrderedDict()
    subscribers = dict()

    time_offset = rospy.Time.now().to_nsec()

    for topic in args.topics:
        rospy.loginfo('Subscribing to topic {}'.format(topic))

        subscribers[topic] = rospy.Subscriber(topic, Float32, lambda data, topic=topic: callback(data, topic))

    rospy.spin()

    out_file = open(out_file_name, 'w')

    for key, value in data_dict.items():
        out_file.write('{}, {}, {}\n'.format(key, value[0], value[1]))

    out_file.close()
