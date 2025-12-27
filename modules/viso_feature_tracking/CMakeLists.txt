cmake_minimum_required(VERSION 3.28.3)
project(viso_feature_tracking VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB VISO_FEATURE_TRACKING_SOURCES src/*.cpp)
add_library(viso_feature_tracking ${VISO_FEATURE_TRACKING_SOURCES})
target_include_directories(viso_feature_tracking PUBLIC include)
target_link_libraries(viso_feature_tracking PUBLIC ${OpenCV_LIBS} Boost::boost Eigen3::Eigen)
