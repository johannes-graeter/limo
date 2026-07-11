cmake_minimum_required(VERSION 3.28.3)
project(limo_apps VERSION 1.0.0 LANGUAGES CXX C)

include(${CMAKE_CURRENT_SOURCE_DIR}/rerun.cmake)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_library(rerun_vis rerun_vis.h rerun_vis.cpp)
target_include_directories(rerun_vis PUBLIC ${rerun_sdk_SOURCE_DIR}/include)
target_link_libraries(rerun_vis
    PUBLIC
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    rerun_sdk
)

add_library(kitti_data_reader kitti_data_reader.h kitti_data_reader.cpp)
target_link_libraries(kitti_data_reader
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

add_executable(limo_with_kitti limo_with_kitti.cpp)
target_link_libraries(limo_with_kitti
    kitti_data_reader
    rerun_vis
    feature_tracking_core
    keyframe_bundle_adjustment
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

