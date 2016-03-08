# Installation
The VIO is implemented in ROS. Thus, you first need to install ROS Indigo according to the []official instructions](http://wiki.ros.org/indigo/Installation/Ubuntu).

You will also need PySide for Python 2.7
```bash
sudo apt-get install python-pyside
```
For the visualization, youalso need PyQtGraph. Download the .deb file from the [website](http://pyqtgraph.org/) and double click it to install.


The VIO with the DUO camera depends on two packages: This package for the state estimation and the [`duo3d_ros`](https://gitlab.inf.ethz.ch/naegelit/DUO3d_ROS) package for the ROS driver of the camera.
Clone both git repositories into your `catkin_ws/src` directory.
Also place the DUO SKD in `catkin_ws/src`.

You are now ready to compile the packages with
```bash
cd catkin_ws
catkin_make
```

## Installing on embedded devices
An optimized version is provided for ARM Cortex devices. To build this optimized version, open the `CMakeLists.txt` of this package and uncomment the line:
```cmake
set(CORTEX 1)
```

You will need to have [NEON 10](http://projectne10.github.io/Ne10/) installed. Follow the installation instructions provided on the website and modify the `NEON_PATH` variable in `CMakeLists.txt` to point to the appropriate location.

# Calibration
It is recommended that you first start a roscore that is always running. This makes it easier for ROS nodes to communicate with each other if some of them have to be restarted. 
```bash
roscore
```
## Camera Calibration
Calibrate the stereo camera using the DUOCalibrator in the duo3d_ros package. First start the camera with 
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
Once you have calibrated the camera, you need to calibrate the IMU. For this, you will use the VIO algorithm. Start the visualization:
```bash
roslaunch vio_ros vis.launch
```
Set the camera down somewhere, where it has a good view of distinctive, relatively close (within 1-2 meters) features.
Then, start the VIO algorithm and the camera. As we are trying to calibrate the IMU's we will use a launch file that sets relatively high IMU bias uncertainties:
```bash
roslauch vio_ros calibrateIMU.launch
```
The PyQtGraph plot will start to plot the estimated biases.
You should see the gyroscope biases converge quite quickly.
Once this has happened, take the camera and set it on all 6 sides (i.e. aligning +/- of all axes with gravity), always making sure that the camera has good features (make sure not to cover the view of the cameras).
Once you feel that the accelerometer values have converged, stop the VIO with `Ctrl + C`.

To store the IMU biases that we just estimated into the calibration file, run:
```bash
roslaunch duo3d_ros merge_IMU_calibration.py
```
Follow the command line instructions to select the appropriate calibration file to merge the estimated biases.

# Using the VIO
Once both the camera and the IMU has been calibrated, we are ready to use the system. Start the visualization (if it isn't already):
```bash
roslaunch vio_ros vis.launch
```
Run the VIO, this time with less IMU bias uncertainty:
```bash
roslauch vio_ros duo_vio.launch
```

