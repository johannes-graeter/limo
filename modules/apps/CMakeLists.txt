add_executable(limo_with_kitti limo_with_kitti.cpp)
target_link_libraries(limo_with_kitti PRIVATE 
    feature_tracking_core
    keyframe_bundle_adjustment
    image_preproc
)
