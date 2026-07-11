cmake_minimum_required(VERSION 3.28.3)
project(tracklets_depth VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(MPI REQUIRED)

file(GLOB TRACKLETS_DEPTH_SOURCES src/*.cpp)
add_library(tracklets_depth ${TRACKLETS_DEPTH_SOURCES})
target_include_directories(tracklets_depth PUBLIC include)
target_link_libraries(tracklets_depth PUBLIC ${OpenCV_LIBS} Eigen3::Eigen ${PCL_LIBRARIES})
    INCLUDES ${PROJECT_HEADER_FILES_INC} ${PROJECT_SOURCE_FILES_INC}
    SOURCES ${PROJECT_SOURCE_FILES_SRC}
    )

#############
## Install ##
#############
# Install all targets, headers by default and scripts and other files if specified (folders or files).
# This command also exports libraries and config files for dependent packages and this supersedes catkin_package.
mrt_install(PROGRAMS scripts FILES res data ${PROJECT_INSTALL_FILES})

#############
## Testing ##
#############
# Add test targets for cpp and python tests
if (CATKIN_ENABLE_TESTING)
    mrt_add_tests(test)
    mrt_add_nosetests(test)
endif()
