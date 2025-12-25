file(GLOB KEYFRAME_BA_SOURCES src/*.cpp)
add_library(keyframe_bundle_adjustment ${KEYFRAME_BA_SOURCES})
target_include_directories(keyframe_bundle_adjustment PUBLIC include)
target_link_libraries(keyframe_bundle_adjustment PUBLIC 
    matches_msg_types
    robust_optimization
    Eigen3::Eigen
    Ceres::ceres
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Boost::boost
)
target_include_directories(keyframe_bundle_adjustment PUBLIC ${PCL_INCLUDE_DIRS})
