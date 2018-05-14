#include "mono_lidar.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "mono_lidar_node");

    keyframe_bundle_adjustment_ros_tool::MonoLidar mono_lidar(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
