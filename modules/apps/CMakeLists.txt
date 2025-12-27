cmake_minimum_required(VERSION 3.28.3)
project(limo_apps VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(kitti_data_reader
    kitti_data_reader.cpp
)
target_include_directories(kitti_data_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
find_package(PCL REQUIRED)
find_package(MPI REQUIRED)
find_package(OpenCV REQUIRED)
target_link_libraries(kitti_data_reader PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES} Eigen3::Eigen)

add_executable(limo_with_kitti limo_with_kitti.cpp)
target_link_libraries(limo_with_kitti PRIVATE 
    feature_tracking_core
    keyframe_bundle_adjustment
    image_preproc
    kitti_data_reader
)

