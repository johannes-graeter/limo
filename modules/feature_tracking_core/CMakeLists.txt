file(GLOB FEATURE_TRACKING_SOURCES src/*.cpp)
add_library(feature_tracking_core ${FEATURE_TRACKING_SOURCES})
target_include_directories(feature_tracking_core PUBLIC include)
target_link_libraries(feature_tracking_core PUBLIC 
    viso2 
    ${OpenCV_LIBS}
    Boost::boost
)
