#include "static_transform_alias.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "static_transform_alias_node");

    util_nodes_tf2_ros_tool::StaticTransformAlias static_transform_alias(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return 0;
}
