cmake_minimum_required(VERSION 3.28.3)
project(robust_optimization VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres REQUIRED)

add_library(robust_optimization src/robust_solving.cpp)
target_include_directories(robust_optimization PUBLIC include)
target_link_libraries(robust_optimization PUBLIC Ceres::ceres)

add_subdirectory(test)
