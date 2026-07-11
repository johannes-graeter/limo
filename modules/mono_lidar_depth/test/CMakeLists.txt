add_executable(test_mono_lidar_depth
    test_depth_estimator.cpp
    test_depth_filter.cpp
    test_ground_plane.cpp
    test_histogram.cpp
    test_line_plane_intersection.cpp
    test_neighbor_finder.cpp
    test_pca.cpp
    test_plane_estimation.cpp
    test_tracklet_depth_estimator.cpp
)
target_link_libraries(test_mono_lidar_depth mono_lidar_depth gtest gtest_main pthread)
add_test(NAME test_mono_lidar_depth COMMAND test_mono_lidar_depth)
