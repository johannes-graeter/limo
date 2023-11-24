#!/bin/bash
SRC_DIR=../../src

pushd SRC_DIR

git clone https://github.com/johannes-graeter/limo.git
git clone https://github.com/johannes-graeter/mrt_cmake_modules.git
git clone https://github.com/johannes-graeter/feature_tracking.git
pushd feature_tracking
git checkout python_binding
popd
git clone https://github.com/johannes-graeter/mono_lidar_depth.git
git clone https://github.com/johannes-graeter/viso2.git
git clone https://github.com/johannes-graeter/rosinterface_handler.git

popd