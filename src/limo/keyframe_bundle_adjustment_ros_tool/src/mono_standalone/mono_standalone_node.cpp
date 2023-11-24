#include "mono_standalone.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "mono_standalone_node");

    keyframe_bundle_adjustment_ros_tool::MonoStandalone mono_standalone(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
