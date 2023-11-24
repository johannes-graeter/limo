#pragma once

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "util_nodes_tf2_ros_tool/StaticTransformAliasInterface.h"

namespace util_nodes_tf2_ros_tool {

class StaticTransformAlias {
public:
    StaticTransformAlias(ros::NodeHandle, ros::NodeHandle);

private:
    ros::Publisher dummyPub_;
    ros::Subscriber dummySub_;

    StaticTransformAliasInterface params_;

    dynamic_reconfigure::Server<StaticTransformAliasConfig> reconfigSrv_; // Dynamic reconfiguration service

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::StaticTransformBroadcaster tfBroadcaster_;

    /// Diagnostics
    diagnostic_updater::Updater updater_;
    // std::unique_ptr<diagnostic_updater::DiagnosedPublisher<std_msgs::Header>> diagnosed_pub_;
    diagnostic_msgs::DiagnosticStatus diagnosticStatus_;

    void setupDiagnostics();
    void checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper&);
    void diagnostic_msg(diagnostic_updater::DiagnosticStatusWrapper&);
    void diagnoseError();

    void subCallback(const std_msgs::Header::ConstPtr& msg);
    void reconfigureRequest(StaticTransformAliasConfig&, uint32_t);

    // execute aliasing, wait for timeout seconds before failing
    void do_aliasing();
};

} // namespace util_nodes_tf2_ros_tool
