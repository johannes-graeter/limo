cmake_minimum_required(VERSION 4.2.1)
project(limo VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add modules
add_subdirectory(modules/matches_msg_types)
add_subdirectory(modules/robust_optimization)
add_subdirectory(modules/viso2)
add_subdirectory(modules/image_preproc)
add_subdirectory(modules/feature_tracking_core)
add_subdirectory(modules/keyframe_bundle_adjustment)

# Apps
add_subdirectory(modules/apps)
