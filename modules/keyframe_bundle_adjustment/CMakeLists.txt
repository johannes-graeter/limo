find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

file(GLOB KEYFRAME_BA_SOURCES src/*.cpp)
add_library(keyframe_bundle_adjustment ${KEYFRAME_BA_SOURCES})
target_include_directories(keyframe_bundle_adjustment PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(keyframe_bundle_adjustment PUBLIC 
    matches_msg_types
    robust_optimization
    Eigen3::Eigen
    Ceres::ceres
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Boost::boost
)

add_subdirectory(test)
