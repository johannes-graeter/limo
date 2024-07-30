#!/bin/sh
# make catkin profile
catkin config --profile limo_release -x _limo_release --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-Wall -Wextra -Wno-unused-parameter -Werror=address -Werror=array-bounds=1 -Werror=bool-compare -Werror=comment -Werror=enum-compare -Werror=format -Werror=init-self -Werror=logical-not-parentheses -Werror=maybe-uninitialized -Werror=memset-transposed-args -Werror=nonnull -Werror=nonnull-compare -Werror=openmp-simd -Werror=parentheses -Werror=return-type -Werror=sequence-point -Werror=sizeof-pointer-memaccess -Werror=switch -Werror=tautological-compare -Werror=trigraphs -Werror=uninitialized -Werror=volatile-register-var

# clone stuff
cd ..
mkdir src
git clone https://github.com/johannes-graeter/feature_tracking.git
git clone https://github.com/johannes-graeter/mono_lidar_depth.git
git clone https://github.com/johannes-graeter/viso2.git
git clone https://github.com/KIT-MRT/mrt_cmake_modules.git
git clone https://github.com/KIT-MRT/rosinterface_handler.git

# build it with profile
catkin build --profile limo_release
