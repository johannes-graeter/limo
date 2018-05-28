# limo

Lidar-Monocular Visual Odometry.
This library is designed to be an open platform for visual odometry algortihm development.
We focus explicitely on the simple integration of the following key methodologies:

* Keyframe selection
* Landmark selection
* Prior estimation
* Depth integration from different sensors.

The core library keyframe_bundle_adjustment is a backend that should faciliate to swap these modules and easily develop those algorithms.

* It is supposed to be an add-on module to do temporal inference of the optimization graph in order to smooth the result
* In order to do that online a windowed approach is used
* Keyframes are instances in time which are used for the bundle adjustment, one keyframe may have several cameras (and therefore images) associated with it
* The selection of Keyframes tries to reduce the amount of redundant information while extending the time span covered by the optimization window to reduce drift
* Methodologies for Keyframe selection:
  * Difference in time
  * Difference in motion

* We use this library for combining Lidar with monocular vision.

## Note

This is work in progress, detailed install instructions and examples will follow.

## Installation

### Requirements

In any case:

* ceres: follow the instructions on [http://ceres-solver.org/installation.html](http://ceres-solver.org/installation.html)
* png++: ```shell sudo apt-get install libpng++-dev```
* install ros https://wiki.ros.org/kinetic/Installation
* install catkin_tools: ```shell sudo apt-get install python-catkin-tools```
* install opencv_apps: ```shell sudo apt-get install ros-kinetic-opencv-apps```

### Build

* initiate a catkin workspace:
    1. ```shell cd *your_catkin_workspace*```
    2. ```shell cd *your_catkin_workspace*/src```
    3. ```shell catkin_init_workspace```

* clone limo into src of workspace:
    1. ```shell cd *your_catkin_workspace*/src```
    2. ```shell git clone https://github.com/johannes-graeter/limo.git```

* clone dependencies and build repos
    1. ```shell cd *your_catkin_workspace*/src/limo```
    2. ```shell bash install_repos.sh```

* unittests:
    1. ```shell cd *your_catkin_workspace*```
    2. ```shell catkin_make run_tests```

* get test data from [https://www.mrt.kit.edu/graeterweb/04.bag](https://www.mrt.kit.edu/graeterweb/04.bag)
    1. this is a bag file generated from Kitti sequence 04 with added semantic labels.
    2. there is more under the same address all named ??.bag (00.bag)

* in different terminals
    1. ```shell roscore```shell
    2. ```shell rosbag play 04.bag -r 0.2 --pause```
    3. ```shell source *your_catkin_workspace*/devel/setup.sh```
      ```shell roslaunch demo_keyframe_bundle_adjustment_meta kitti_standalone.launch```
    4. unpause rosbag (hit space in terminal)
    5. rviz *your_catkin_workspace*/src/demo_keyframe_bundle_adjustment_meta/res/default.rviz

* watch limo trace the trajectory in rviz :)

### Todo

* runtime is ok for individual modules, however communication between nodes must be enhanced to ensure online usage (nodelets...)
* add and try rocc landmark selection

### Try it out

If you just want to give it a quick peek, I prepared a ready-to-use virtualbox image (packed with Ubuntu 16.04.04, ros kinetic, ceres, mrt_cmake_modules and limo).

* download it from [https://www.mrt.kit.edu/graeterweb/limo_core.ova](https://www.mrt.kit.edu/graeterweb/limo_core.ova).
* Find the library in ~/workspaces/limo/src/limo.
* Check out the unittests for examples on simulated data.
* Password for the vm-image is "1234".
