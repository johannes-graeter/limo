#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "mono_lidar.hpp"

namespace keyframe_bundle_adjustment_ros_tool {

class MonoLidarNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<MonoLidar>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<MonoLidar> impl_;
};
} // namespace keyframe_bundle_adjustment_ros_tool

PLUGINLIB_EXPORT_CLASS(keyframe_bundle_adjustment_ros_tool::MonoLidarNodelet, nodelet::Nodelet);
