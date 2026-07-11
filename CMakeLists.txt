cmake_minimum_required(VERSION 3.28.3)
project(limo VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()

# Ubuntu's PCL links VTK, whose exported VTK::mpi target requires MPI::MPI_C.
find_package(MPI REQUIRED COMPONENTS C)

# Libraries
add_subdirectory(modules/matches_msg_types)
add_subdirectory(modules/robust_optimization)
add_subdirectory(modules/viso2)
add_subdirectory(modules/image_preproc)
add_subdirectory(modules/feature_tracking_core)
add_subdirectory(modules/keyframe_bundle_adjustment)
add_subdirectory(modules/mono_lidar_depth)

# Apps
add_subdirectory(modules/apps)
