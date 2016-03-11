# Installation
The VIO is implemented in ROS. Thus, you first need to install ROS Indigo according to the [official instructions](http://wiki.ros.org/indigo/Installation/Ubuntu).

You will also need PySide for Python 2.7
```bash
sudo apt-get install python-pyside
```
For the visualization, youalso need PyQtGraph. Download the .deb file from the [website](http://pyqtgraph.org/) and double click it to install.

The VIO depends on three ROS packages
- [This package](https://github.com/ethz-ait/duo_vio) for the state estimation
- [`ait_ros_messages`](https://github.com/ethz-ait/ait_ros_messages) for custom ROS messages
- A ROS driver that publishes image and IMU data in VioSensorMsg messages as defined in `ait_ros_messages`. Depending on the camera you are using, this package will differ.
    - If you are using a [DUO3d](http://duo3d.com/) camera, the package you need is [`duo3d_ros`](https://github.com/ethz-ait/duo3d_ros). You will also need to place the DUO SKD in `catkin_ws/src`, which customers can download from the [DUO3d](http://duo3d.com/) website

Clone all git repositories into your `catkin_ws/src` directory.

You are now ready to compile the packages with
```bash
cd catkin_ws
catkin_make
```

## Compiling for embedded devices
A version of the algorithm optimized for ARM Cortex-A devices is provided.
To build this optimized version, open the `CMakeLists.txt` of this package and uncomment the line:
```cmake
set(CORTEX 1)
```

You will need to have [NEON 10](http://projectne10.github.io/Ne10/) installed. Follow the installation instructions provided on the website and modify the `NEON_PATH` variable in `CMakeLists.txt` to point to the appropriate location.

# Calibration
Note: It is recommended that you first start a roscore that is always running. This makes it easier for ROS nodes to communicate with each other if some of them have to be restarted. 
```bash
roscore
```
## Camera Calibration
Calibrate the stereo camera using the DUOCalibrator in the duo3d_ros package. First start the camera with:
```bash
roslaunch duo3d_ros duo.launch
```
Then, run the calibrator:
```bash
rosrun duo3d_ros DUOCalibrator.py
```

Choose the appropriate checkerboard size and dimensions. You can figure out the checkerboard size by pointing the camera at the checkerboard and varying the rows and columns in the app until the whole checkerboard is detected and rainbow colored.

Once the settings are correct, calibrate the camera by pressing the `Start recording` button and capturing the checkerboard form various angles and distances.
It is recommended that you take at least 30 images.
Once you feel that the checkerboard has been recorded from enough poses and appears in all areas of the images, press `Stop recording`.
The cameras will be calibrated (this can take several seconds).

## IMU Calibration
Once you have calibrated the camera, you need to calibrate the IMU.
For this, place the camera on a level surface, such that the camera has zero roll and pitch: The camera should look horizontally forward.

It it's not already running, start the camera with:
```bash
roslaunch duo3d_ros duo.launch
```

Run the calibrator with:
```bash
rosrun duo3d_ros IMUCalibrator.py
```
Follow the command line instructions to calibrate the camera and write the calibration to the appropriate calibration file.

# Using the VIO
Once both the camera and the IMU has been calibrated, we are ready to use the system. Start the visualization (if it isn't already):
```bash
roslaunch duo_vio vis.launch
```
Run the VIO:
```bash
roslauch duo_vio duo_vio.launch
```
