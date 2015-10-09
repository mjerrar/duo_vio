# VIO_ROS

Launch the visual odometry node and the DUO3D node with:
```
roslaunch vio_ros duo_vio.launch
```

To just launch the visual odometry node (with messages being published by another node, e.g. a rosbag):
```
roslaunch vio_ros vio.launch
```

To launch the VIO visualization (you need RViz and PyQtGraph):
```
roslaunch vio_ros vio_vis.launch
```