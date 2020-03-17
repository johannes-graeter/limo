#include "static_transform_alias.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace util_nodes_tf2_ros_tool {

class StaticTransformAliasNodelet : public nodelet::Nodelet {

    virtual void onInit();
    boost::shared_ptr<StaticTransformAlias> m_;
};

void StaticTransformAliasNodelet::onInit() {
    m_.reset(new StaticTransformAlias(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace util_nodes_tf2_ros_tool

PLUGINLIB_EXPORT_CLASS(util_nodes_tf2_ros_tool::StaticTransformAliasNodelet, nodelet::Nodelet);
