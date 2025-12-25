cmake_minimum_required(VERSION 4.2.1)
project(feature_tracking_core VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB FEATURE_TRACKING_SOURCES src/*.cpp)
add_library(feature_tracking_core ${FEATURE_TRACKING_SOURCES})
target_include_directories(feature_tracking_core PUBLIC include)
target_link_libraries(feature_tracking_core PUBLIC viso2 OpenCV::OpenCV Boost::boost Eigen3::Eigen)
