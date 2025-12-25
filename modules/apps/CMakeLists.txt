cmake_minimum_required(VERSION 4.2.1)
project(limo_apps VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(limo_with_kitti limo_with_kitti.cpp)
target_link_libraries(limo_with_kitti PRIVATE 
    feature_tracking_core
    keyframe_bundle_adjustment
    image_preproc
)

add_library(kitti_data_reader
    kitti_data_reader.cpp
)
target_include_directories(kitti_data_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(kitti_data_reader PUBLIC OpenCV::OpenCV PCL::PCL Eigen3::Eigen)
set_target_properties(kitti_data_reader PROPERTIES CXX_STANDARD 26)
