Limo works with ROS. In order to make it ros independant, I want to supply a single main that will load KITTI data set and run limo on it. Definition of done:
* all libraries are ros independant
* there is a single main that I can run that reads in  KIITI data, runs the code and generates the poses to be evaluated
* the trajectory, the images and the reconstructed landmarks are shown plotted via rerun
* there is a single repository and a single dockerfile with which users can just checkout and run the container 
* ceres has some deprecated features. Update for a current version.
* mono_lidar_depth is coded very ugly please refactor it in c++23 with a proper test suite.

make sure to test what you implement with unittest and document usage in a README