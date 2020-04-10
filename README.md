# Limo

Lidar-Monocular Visual Odometry.
This library is designed to be an open platform for visual odometry algortihm development.
We focus explicitely on the simple integration of the following key methodologies:

* Keyframe selection
* Landmark selection
* Prior estimation
* Depth integration from different sensors.
* Scale integration by groundplane constraint.

The core library keyframe_bundle_adjustment is a backend that should faciliate to swap these modules and easily develop those algorithms.

* It is supposed to be an add-on module to do temporal inference of the optimization graph in order to smooth the result
* In order to do that online a windowed approach is used
* Keyframes are instances in time which are used for the bundle adjustment, one keyframe may have several cameras (and therefore images) associated with it
* The selection of Keyframes tries to reduce the amount of redundant information while extending the time span covered by the optimization window to reduce drift
* Methodologies for Keyframe selection:
  * Difference in time
  * Difference in motion

* We use this library for combining Lidar with monocular vision.
* Limo2 on KITTI is LIDAR with monocular Visual Odometry, supported with groundplane constraint
* Video: https://youtu.be/wRemjJBjp64
* Now we switched from kinetic to melodic

## Details
This work was accepted on IROS 2018.
See https://arxiv.org/pdf/1807.07524.pdf .

If you refer to this work please cite:

```bibtex
@inproceedings{graeter2018limo,
  title={LIMO: Lidar-Monocular Visual Odometry},
  author={Graeter, Johannes and Wilczynski, Alexander and Lauer, Martin},
  booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={7872--7879},
  year={2018},
  organization={IEEE}
}
```


Please note that Limo2 differs from the publication. 
We enhanced the speed a little and added additional groundplane reconstruction for pure monocular visual odometry and a combination of scale from LIDAR and the groundplane (best performing on KITTI).
For information on Limo2, please see my dissertation https://books.google.de/books?hl=en&lr=&id=cZW8DwAAQBAJ&oi .

## Installation

### Docker
To facilitate the development I created a standalone dockerfile.
* Install docker https://phoenixnap.com/kb/how-to-install-docker-on-ubuntu-18-04
* build docker:
```shell
# This is where you put the rosbags this will be available at /limo_data in the container
mkdir $HOME/limo_data
cd limo/docker
docker-compose build limo
```
* You can run the docker and go to the entrypoint with 
```shell
docker-compose run limo bash
```
Go to step Run in this tutorial and use tmux for terminals.
* You can invoke a jupyter notebook with a python interface for limo with
```shell
docker-compose up limo
```
and open the suggested link from the run output in a browser.

### Semantic segmentation

The monocular variant expects semantic segmentation of the images.
You can produce this for example with my fork from NVIDIA's semantic segmentation:
1. Clone my fork
```bash
git clone https://github.com/johannes-graeter/semantic-segmentation
```
2. Download best_kitti.pth as described in the README.md from NVIDIA and put it in the semantic-segmentation folder
3. I installed via their docker, for which you must be logged in on (and register if necessary)
https://ngc.nvidia.com/

4. Build the container with 
```bash
docker-compose build semantic-segmentation
```

5. Run the segmentation with
```bash
docker-copmose run semantic-segmentation
```
Note that without a GPU this will take some time.
With the Nvidia Quadro P2000 on my laptop i took around 6 seconds per image.

### Requirements

In any case:

* ceres: 
  - follow the instructions on [http://ceres-solver.org/installation.html](http://ceres-solver.org/installation.html)
  - you will need ```sudo make install``` to install the headers.
  - tested with libsuitesparse-dev from standard repos.
* png++: 
```shell
 sudo apt-get install libpng++-dev
 ```
* install ros: 
  - follow the instructions on [https://wiki.ros.org/melodic/Installation](https://wiki.ros.org/melodic/Installation).
  - you will need to install ros-perception (for pcl).
  - don't forget to source your ~/.bashrc afterwards.
* install catkin_tools: 
```shell 
sudo apt-get install python-catkin-tools
 ```
* install opencv_apps: 
```shell
sudo apt-get install ros-melodic-opencv-apps
```
* install git: 
```shell
sudo apt-get install git
```

### Build

* initiate a catkin workspace:
    ```shell 
    cd ${your_catkin_workspace}
    catkin init
    ```

* clone limo into src of workspace:
    ```shell 
    mkdir ${your_catkin_workspace}/src
    cd ${your_catkin_workspace}/src
    git clone https://github.com/johannes-graeter/limo.git
    ```

* clone dependencies and build repos
    ```shell 
    cd ${your_catkin_workspace}/src/limo
    bash install_repos.sh
    ```

* unittests:
    ```shell 
    cd ${your_catkin_workspace}/src/limo
    catkin run_tests --profile limo_release
    ```
    
### Run
* get test data [Sequence 04](https://drive.google.com/open?id=16txq5V2RJyJH_VTsbeYOJzSWR5AKOtin) or [Sequence 01](https://drive.google.com/open?id=1u7RFNSvx3IY6l3-hIHBEL1X3wUGri8Tg).
This is a bag file generated from Kitti sequence 04 with added semantic labels.
   
* in different terminals (for example with tmux)
    1. `roscore`
    2. `rosbag play 04.bag -r 0.1 --pause --clock`
    3. ```shell
       source ${your_catkin_workspace}/devel_limo_release/setup.sh
       roslaunch demo_keyframe_bundle_adjustment_meta kitti_standalone.launch
       ```
    4. unpause rosbag (hit space in terminal)
    5. `rviz -d ${your_catkin_workspace}/src/demo_keyframe_bundle_adjustment_meta/res/default.rviz`

* watch limo trace the trajectory in rviz :)
* Before submitting an issue, please have a look at the section [Known issues](#known-issues).

## Known issues
* Unittest of LandmarkSelector.voxel fails with libpcl version 1.7.2 or smaller (just 4 landmarks are selected). 
Since this works with pcl 1.8.1 which is standard for ros melodic, this is ignored. This should lower the performance of the software only by a very small amount.
