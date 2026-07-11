cmake_minimum_required(VERSION 3.28.3)
project(matches_msg_types VERSION 1.0.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)

add_library(matches_msg_types INTERFACE)
target_include_directories(matches_msg_types INTERFACE include)
target_link_libraries(matches_msg_types INTERFACE Eigen3::Eigen)
