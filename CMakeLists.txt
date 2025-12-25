cmake_minimum_required(VERSION 3.20)
project(limo VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(PNG REQUIRED)

# Add modules
add_subdirectory(modules/matches_msg_types)
add_subdirectory(modules/robust_optimization)
add_subdirectory(modules/viso2)
add_subdirectory(modules/image_preproc)
add_subdirectory(modules/feature_tracking_core)
add_subdirectory(modules/keyframe_bundle_adjustment)

# Apps
add_subdirectory(modules/apps)
