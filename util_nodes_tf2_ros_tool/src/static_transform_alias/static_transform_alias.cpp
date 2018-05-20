#include "static_transform_alias.hpp"
#include <rosinterface_handler/utilities.hpp>
#include <tf/tf.h>

namespace util_nodes_tf2_ros_tool {

StaticTransformAlias::StaticTransformAlias(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */
    rosinterface_handler::setLoggerLevel(private_node_handle);
    params_.fromParamServer();
    setupDiagnostics();

    /**
     * Set up dynamic reconfiguration
     */
    reconfigSrv_.setCallback(boost::bind(&StaticTransformAlias::reconfigureRequest, this, _1, _2));

    /**
     * Publishers & subscriber
     */
    // A diagnosed pub can be used for message types with header.
    // This adds a diagnostics message for the frequency to this topic
    // diagnosed_pub_ = std::make_unique<diagnostic_updater::DiagnosedPublisher<std_msgs::Header>>(
    //     private_node_handle.advertise<std_msgs::Header>(params_.diag_pub_msg_name,
    //                                                     params_.msg_queue_size),
    //     updater_,
    //     diagnostic_updater::FrequencyStatusParam(&params_.diagnostic_updater_rate,
    //                                              &params_.diagnostic_updater_rate,
    //                                              params_.diagnostic_updater_rate_tolerance, 5),
    //     diagnostic_updater::TimeStampStatusParam());

    //    dummyPub_ = private_node_handle.advertise<std_msgs::Header>(params_.publisher_msg_name,
    //    params_.msg_queue_size);
    //    // Instantiate subscriber last, to assure all objects are initialised when first message
    //    is received.
    //    dummySub_ =
    //        private_node_handle.subscribe(params_.subscriber_msg_name, params_.msg_queue_size,
    //                                      &StaticTransformAlias::subCallback, this,
    //                                      ros::TransportHints().tcpNoDelay());

    do_aliasing();

    rosinterface_handler::showNodeInfo();
}

void StaticTransformAlias::do_aliasing() {
    ROS_DEBUG_STREAM("looking up transform from " << params_.from_target_frame_id << " to "
                                                  << params_.to_target_frame_id
                                                  << " timeout="
                                                  << params_.timeout
                                                  << " sec");
    std::string frame_name_target, frame_name_source;
    tf::resolve(params_.from_target_frame_id, frame_name_target);
    tf::resolve(params_.from_source_frame_id, frame_name_source);
    geometry_msgs::TransformStamped transform =
        tfBuffer_.lookupTransform(frame_name_target, frame_name_source, ros::Time(0), ros::Duration(params_.timeout));
    transform.header.frame_id = params_.to_target_frame_id;
    transform.child_frame_id = params_.to_source_frame_id;
    tfBroadcaster_.sendTransform(transform);
}

///*
// * Use const ConstPtr for your callbacks.
// * The 'const' assures that you can not edit incoming messages.
// * The Ptr type guarantees zero copy transportation within nodelets.
// */
// void StaticTransformAlias::subCallback(const std_msgs::Header::ConstPtr& msg) {

//    // do your stuff here...
//    std_msgs::Header new_msg = *msg;
//    dummyPub_.publish(new_msg);
//    // diagnosed_pub_->publish(new_msg);

//    diagnosticStatus_.message = "Valid loop";
//    diagnosticStatus_.level = diagnostic_msgs::DiagnosticStatus::OK;
//    // The updater will take care of publishing at a throttled rate
//    // When calling update, all updater callbacks (defined in setupDiagnostics) will be run
//    updater_.update();
//}

/**
  * This callback is called whenever a change was made in the dynamic_reconfigure window
*/
void StaticTransformAlias::reconfigureRequest(StaticTransformAliasConfig& config, uint32_t level) {
    params_.fromConfig(config);

    do_aliasing();
}

/*
 * Setup the Diagnostic Updater
 */
void StaticTransformAlias::setupDiagnostics() {
    // Give a unique hardware id
    diagnosticStatus_.hardware_id = params_.diagnostic_updater_hardware_id;
    diagnosticStatus_.message = "Starting...";
    diagnosticStatus_.level = diagnostic_msgs::DiagnosticStatus::STALE;
    updater_.setHardwareID(params_.diagnostic_updater_hardware_id);

    // Add further callbacks (or unittests) that should be called regularly
    updater_.add("StaticTransformAlias Sensor Status", this, &StaticTransformAlias::checkSensorStatus);

    updater_.force_update();
}

void StaticTransformAlias::checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper) {
    status_wrapper.summary(diagnosticStatus_);
}

} // namespace util_nodes_tf2_ros_tool
