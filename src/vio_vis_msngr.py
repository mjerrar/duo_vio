#!/usr/bin/python
from __future__ import print_function, division

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud, ChannelFloat32, Image
import tf
import cv2
from cv_bridge import CvBridge

from vio_ros.msg import vio_vis
import pyqtgraph as pg
import numpy as np
import time


class Visualizer(pg.QtCore.QThread):
    newData = pg.QtCore.Signal(object)  # threading signal, used to send plot data to the main thread

    def run(self):
        self.robot_pose_pub = rospy.Publisher("vio_vis/rviz/robot_pose", Marker, queue_size=1)
        self.robot_path_pub = rospy.Publisher("vio_vis/rviz/robot_path", Path, queue_size=1)
        self.map_pub = rospy.Publisher("vio_vis/rviz/point_cloud", PointCloud, queue_size=1)
        self.image_pub = rospy.Publisher("vio_vis/rviz/img", Image, queue_size=1)
        # self.image_cloud_pub = rospy.Publisher("vio_vis/rviz/img_cloud", PointCloud, queue_size=1)
        self.robot_path = Path()

        rospy.Subscriber("vio_vis/vio_vis", vio_vis, self.vis_cb)
        rospy.Subscriber("vio_vis/reset", Empty, self.clear_cb)

    def pose_arrows_from_quaternion(self, pose, ns="_", scale=1.0):
        """
        :param pose: the pose of the coordinate frame to plot
        :param ns: the namespace of this pose
        :param scale: the scale of this pose
        :return: three visualization markers spanning the pose
        """

        rotation = tf.transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, -pose.orientation.w])
        rotation = rotation[0:3, 0:3]

        x_axis = rotation[0, 0:3]
        y_axis = rotation[1, 0:3]
        z_axis = rotation[2, 0:3]

        shaft_diameter = 0.05
        head_diameter = 0.1
        head_length = 0.2

        x_marker = Marker()
        x_marker.header.frame_id = "/world"
        x_marker.ns = "{}_x".format(ns)
        x_marker.id = 0
        x_marker.type = x_marker.ARROW
        x_marker.action = x_marker.ADD
        x_marker.scale.x = shaft_diameter*scale
        x_marker.scale.y = head_diameter*scale
        x_marker.scale.z = head_length*scale
        x_marker.color.r = 1.0
        x_marker.color.a = 1.0
        start = Point()
        end = Point()
        start.x = pose.position.x
        start.y = pose.position.y
        start.z = pose.position.z
        end.x = start.x + x_axis[0]*scale
        end.y = start.y + x_axis[1]*scale
        end.z = start.z + x_axis[2]*scale
        x_marker.points.append(start)
        x_marker.points.append(end)

        y_marker = Marker()
        y_marker.header.frame_id = "/world"
        y_marker.ns = "{}_y".format(ns)
        y_marker.id = 0
        y_marker.type = y_marker.ARROW
        y_marker.action = y_marker.ADD
        y_marker.scale.x = shaft_diameter*scale
        y_marker.scale.y = head_diameter*scale
        y_marker.scale.z = head_length*scale
        y_marker.color.g = 1.0
        y_marker.color.a = 1.0
        start = Point()
        end = Point()
        start.x = pose.position.x
        start.y = pose.position.y
        start.z = pose.position.z
        end.x = start.x + y_axis[0]*scale
        end.y = start.y + y_axis[1]*scale
        end.z = start.z + y_axis[2]*scale
        y_marker.points.append(start)
        y_marker.points.append(end)

        z_marker = Marker()
        z_marker.header.frame_id = "/world"
        z_marker.ns = "{}_z".format(ns)
        z_marker.id = 0
        z_marker.type = z_marker.ARROW
        z_marker.action = z_marker.ADD
        z_marker.scale.x = shaft_diameter*scale
        z_marker.scale.y = head_diameter*scale
        z_marker.scale.z = head_length*scale
        z_marker.color.b = 1.0
        z_marker.color.a = 1.0
        start = Point()
        end = Point()
        start.x = pose.position.x
        start.y = pose.position.y
        start.z = pose.position.z
        end.x = start.x + z_axis[0]*scale
        end.y = start.y + z_axis[1]*scale
        end.z = start.z + z_axis[2]*scale
        z_marker.points.append(start)
        z_marker.points.append(end)

        return x_marker, y_marker, z_marker

    def robot_pose_cb(self, data):
        x_marker, y_marker, z_marker = self.pose_arrows_from_quaternion(data, "robot")

        self.robot_pose_pub.publish(x_marker)
        self.robot_pose_pub.publish(y_marker)
        self.robot_pose_pub.publish(z_marker)

        pose_stamped = PoseStamped()
        pose_stamped.pose = data
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "world"
        self.robot_path.poses.append(pose_stamped)
        self.robot_path.header = pose_stamped.header
        self.robot_path_pub.publish(self.robot_path)

        return

    def anchor_poses_cb(self, data):

        for i, pose in enumerate(data.poses):
            # print("anchor {}".format(i))
            x_marker, y_marker, z_marker = self.pose_arrows_from_quaternion(pose, ns="anchor_{}".format(i), scale=0.5)
            self.robot_pose_pub.publish(x_marker)
            self.robot_pose_pub.publish(y_marker)
            self.robot_pose_pub.publish(z_marker)
        return

    def map_cb(self, feature_map, status_vect):
        cloud = PointCloud()
        cloud.header.frame_id = "world"
        r_channel = ChannelFloat32()
        r_channel.name = 'r'
        g_channel = ChannelFloat32()
        g_channel.name = 'g'
        b_channel = ChannelFloat32()
        b_channel.name = 'b'

        for i in range(0, len(status_vect.data)):
            if status_vect.data[i] > 0:
                point = Point()
                point.x = feature_map.data[i*3 + 0]
                point.y = feature_map.data[i*3 + 1]
                point.z = feature_map.data[i*3 + 2]
                cloud.points.append(point)
                if status_vect.data[i] == 1:
                    r_channel.values.append(0.0)
                    g_channel.values.append(1.0)
                    b_channel.values.append(0.0)
                # elif status_vect.data[i] == 2:
                else:
                    r_channel.values.append(1.0)
                    g_channel.values.append(0.0)
                    b_channel.values.append(1.0)
                # else:
                #     print(data.data[i*3 + 0:i*3 + 3])
                #     r_channel.values.append(0.0)
                #     g_channel.values.append(0.0)
                #     b_channel.values.append(0.0)
        cloud.channels.append(r_channel)
        cloud.channels.append(g_channel)
        cloud.channels.append(b_channel)

        self.map_pub.publish(cloud)
        return

    def img_cb(self, img, feature_tracks, pred_feature_tracks, status_vect):
        if not img.data:  # if image is empty, do not update the plot
            return

        image = CvBridge().imgmsg_to_cv2(img, 'mono8')
        image_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        active_feature_color = (0, 255, 0)
        delayed_feature_color = (255, 0, 255)

        circle_size = int(image_color.shape[0]/160)

        for i in range(0, int(len(feature_tracks.data)/2)):
            if status_vect.data[i] == 1:
                feature_color = active_feature_color
            else:
                feature_color = delayed_feature_color
            cv2.circle(image_color, (int(feature_tracks.data[i*2]), int(feature_tracks.data[i*2+1])), circle_size, feature_color, -1)
            # cv2.putText(image_color, '{}'.format(i), (int(feature_tracks.data[i*2]), int(feature_tracks.data[i*2+1])), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.45, (0, 0, 255))
            # cv2.circle(image_color, (int(pred_feature_tracks.data[i*2]), int(pred_feature_tracks.data[i*2+1])), 2, pred_feature_color, -1)

        self.image_pub.publish(CvBridge().cv2_to_imgmsg(image_color, "bgr8"))

        return

    def update_plots(self, gyro_bias, acc_bias):

        self.newData.emit({'gyro': gyro_bias, 'acc': acc_bias})

    def vis_cb(self, data):
        self.img_cb(data.image, data.feature_tracks, data.pred_feature_tracks, data.status_vect)

        self.map_cb(data.map, data.status_vect)

        self.anchor_poses_cb(data.anchor_poses)

        self.robot_pose_cb(data.robot_pose)

        self.update_plots(data.gyro_bias, data.acc_bias)

    def clear_cb(self, data):
        global plot_idx
        self.robot_path.poses = []
        self.robot_path_pub.publish(self.robot_path)

        plot_idx = 0
        return


def plotter(data):
    global gyro_bias_curves, acc_bias_curves, plot_idx, gyro_bias_data, acc_bias_data

    for i in range(0, 3):
        gyro_bias_data[i][plot_idx] = data['gyro'].data[i]
        gyro_bias_curves[i].setData(gyro_bias_data[i][:plot_idx])

        acc_bias_data[i][plot_idx] = data['acc'].data[i]
        acc_bias_curves[i].setData(acc_bias_data[i][:plot_idx])
    plot_idx += 1
    # pg.QtGui.QApplication.processEvents()

    if plot_idx >= gyro_bias_data[0].shape[0]:
        print('resizing, size was {}'.format(gyro_bias_data[0].shape[0]))
        for i in range(0, 3):
            tmp = gyro_bias_data[i]
            gyro_bias_data[i] = np.empty(gyro_bias_data[i].shape[0]*2)
            gyro_bias_data[i][:tmp.shape[0]] = tmp

            tmp = acc_bias_data[i]
            acc_bias_data[i] = np.empty(acc_bias_data[i].shape[0]*2)
            acc_bias_data[i][:tmp.shape[0]] = tmp


if __name__ == "__main__":

    rospy.init_node("vio_vis")

    win = pg.GraphicsWindow()
    win.setGeometry(992, 0, 927, 560)
    win.setWindowTitle('Estimated parameters')
    gyro_bias_plot = win.addPlot(title='Gyro bias')
    win.nextRow()
    acc_bias_plot = win.addPlot(title='Accelerometer bias')
    gyro_bias_plot.setDownsampling(mode='subsample')
    acc_bias_plot.setDownsampling(mode='subsample')
    gyro_bias_plot.setClipToView(True)
    acc_bias_plot.setClipToView(True)
    acc_bias_plot.addLegend()
    gyro_bias_plot.addLegend()
    gyro_bias_curves = [gyro_bias_plot.plot(pen=(255, 0, 0), name='x'),
                        gyro_bias_plot.plot(pen=(0, 255, 0), name='y'),
                        gyro_bias_plot.plot(pen=(0, 0, 255), name='z')]
    acc_bias_curves = [acc_bias_plot.plot(pen=(255, 0, 0), name='x'),
                       acc_bias_plot.plot(pen=(0, 255, 0), name='y'),
                       acc_bias_plot.plot(pen=(0, 0, 255), name='z')]
    gyro_bias_data = [np.empty(100000), np.empty(100000), np.empty(100000)]
    acc_bias_data = [np.empty(100000), np.empty(100000), np.empty(100000)]
    plot_idx = 0

    vis = Visualizer()
    vis.newData.connect(plotter)
    vis.start()

    # pg.QtGui.QApplication.exec_()
    pg.QtGui.QApplication.processEvents()
    while not rospy.is_shutdown():
        pg.QtGui.QApplication.processEvents()
        time.sleep(0.01)
    pg.QtGui.QApplication.closeAllWindows()
