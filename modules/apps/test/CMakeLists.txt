add_library(synthetic_kitti synthetic_kitti.h synthetic_kitti.cpp)
target_link_libraries(synthetic_kitti PUBLIC ${OpenCV_LIBS} Eigen3::Eigen)

add_executable(test_kitti_data_reader test_kitti_data_reader.cpp)
target_link_libraries(test_kitti_data_reader kitti_data_reader synthetic_kitti gtest gtest_main pthread)
add_test(NAME test_kitti_data_reader COMMAND test_kitti_data_reader)

add_executable(test_limo_e2e test_limo_e2e.cpp)
target_link_libraries(test_limo_e2e synthetic_kitti gtest gtest_main pthread)
target_compile_definitions(test_limo_e2e PRIVATE LIMO_WITH_KITTI_BINARY="$<TARGET_FILE:limo_with_kitti>")
add_dependencies(test_limo_e2e limo_with_kitti)
add_test(NAME test_limo_e2e COMMAND test_limo_e2e)
set_tests_properties(test_limo_e2e PROPERTIES TIMEOUT 600)
