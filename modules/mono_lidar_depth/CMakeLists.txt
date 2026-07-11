find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_library(mono_lidar_depth
    src/depth_estimator.cpp
    src/depth_filter.cpp
    src/ground_plane.cpp
    src/histogram.cpp
    src/line_plane_intersection.cpp
    src/neighbor_finder.cpp
    src/parameters.cpp
    src/pca.cpp
    src/plane_estimation.cpp
    src/road_depth_estimator.cpp
    src/statistics.cpp
    src/tracklet_depth_estimator.cpp
)
target_include_directories(mono_lidar_depth PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(mono_lidar_depth PUBLIC
    matches_msg_types
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

add_subdirectory(test)
