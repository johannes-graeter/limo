cmake_minimum_required(VERSION 4.2.1)
project(image_preproc VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_library(image_preproc src/brightness_correction.cpp src/gamma_corrector.cpp)
target_include_directories(image_preproc PUBLIC include)
target_link_libraries(image_preproc PUBLIC OpenCV::OpenCV)
