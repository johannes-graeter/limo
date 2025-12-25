cmake_minimum_required(VERSION 4.2.1)
project(limo_apps VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(limo_with_kitti limo_with_kitti.cpp)
target_link_libraries(limo_with_kitti PRIVATE 
    feature_tracking_core
    keyframe_bundle_adjustment
    image_preproc
)
