#!/bin/sh
cd ..
git clone https://github.com/KIT-MRT/feature_tracking.git
git clone https://github.com/johannes-graeter/mono_lidar_depth.git
git clone https://github.com/KIT-MRT/viso2.git
git clone https://github.com/KIT-MRT/mrt_cmake_modules.git
# This hack is needed since rosinterface_handler has problems beeing 
# imported by raw catkin
cd ..
catkin_make > /tmp/first_build_bound_to_fail.txt
cd src
git clone https://github.com/KIT-MRT/rosinterface_handler.git
cd ..
catkin_make