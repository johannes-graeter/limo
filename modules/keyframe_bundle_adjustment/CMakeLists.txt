cmake_minimum_required(VERSION 4.2.1)
project(keyframe_bundle_adjustment VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

file(GLOB KEYFRAME_BA_SOURCES src/*.cpp)
add_library(keyframe_bundle_adjustment ${KEYFRAME_BA_SOURCES})
target_include_directories(keyframe_bundle_adjustment PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(keyframe_bundle_adjustment PUBLIC 
    matches_msg_types
    robust_optimization
    Eigen3::Eigen
    Ceres::ceres
    ${PCL_LIBRARIES}
    OpenCV::OpenCV
    Boost::boost
)
