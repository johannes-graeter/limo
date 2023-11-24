#include "mono_standalone.hpp"

#include <Eigen/Eigen>

#include <chrono>
#include <cv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <nav_msgs/Path.h>
#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/core/eigen.hpp> //attention, eigen must be icluded before that!

#include <chrono>

#include <matches_msg_conversions_ros/convert.hpp>

#include <commons/general_helpers.hpp>
#include <commons/publish_helpers.hpp>
#include <commons/color_by_index_hsv.hpp>

namespace keyframe_bundle_adjustment_ros_tool {


MonoStandalone::MonoStandalone(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : interface_{nh_private}, reconfigure_server_{nh_private}, tf_listener_{tf_buffer_} {

    /**
     * Initialization
     */
    interface_.fromParamServer();
    setupDiagnostics();

    // create ba class, without landmark selection scheme (will be set in reconfigure)
    bundle_adjuster_ = std::make_shared<keyframe_bundle_adjustment::BundleAdjusterKeyframes>();

    // A diagnosed pub can be used for message types with header.
    // This adds a diagnostics message for the frequency to this topic.
    // You can use the publisher in the interface object to create a diagnosed one from it.
    // publisherDiagnosed_ = std::make_unique<diagnostic_updater::DiagnosedPublisher<Msg>>(
    //     interface_.dummy_publisher, updater_,
    //     diagnostic_updater::FrequencyStatusParam(&interface_.diagnostic_updater_rate,
    //                                              &interface_.diagnostic_updater_rate,
    //                                              interface_.diagnostic_updater_rate_tolerance,
    //                                              5),
    //     diagnostic_updater::TimeStampStatusParam());

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/MonoStandalone.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigure_server_.setCallback(boost::bind(&MonoStandalone::reconfigureRequest, this, _1, _2));
    {
        // setup synchronizer
        sync_ = std::make_unique<Synchronizer>(
            ApproximateTime(100), *(interface_.tracklets_subscriber), *(interface_.camera_info_subscriber));
        sync_->registerCallback(boost::bind(&MonoStandalone::callbackSubscriber, this, _1, _2));
    }
    // setup of selectors and bundle adjuster is done in reconfigureCallback

    // get extrinsics from tf, this is the pose from vehicle to camera frame!
    ROS_DEBUG_STREAM("MonoStandalone: Getting transform from vehicle to camera frame");
    try {
        Eigen::Affine3d ext_aff = tf2::transformToEigen(tf_buffer_.lookupTransform(
            interface_.calib_target_frame_id, interface_.calib_source_frame_id, ros::Time::now(), ros::Duration(5.)));
        // attention this only works if affine transform is pure rotation and translation
        trf_camera_vehicle = Eigen::Isometry3d(ext_aff.matrix());
    } catch (const tf2::LookupException& exc) {
        ROS_ERROR_STREAM(exc.what());
        ROS_ERROR_STREAM("set calib to identity and continue");
        trf_camera_vehicle = Eigen::Isometry3d::Identity();
    }
    ROS_DEBUG_STREAM("transform from vehicle to camera frame=\n" << trf_camera_vehicle.matrix());

    rosinterface_handler::showNodeInfo();
}

void MonoStandalone::callbackSubscriber(const TrackletsMsg::ConstPtr& tracklets_msg,
                                        const CameraInfoMsg::ConstPtr& camera_info_msg) {
    auto start_time = std::chrono::steady_clock::now();

    std::stringstream ss;
    ss << std::endl;

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(camera_info_msg);
    assert(model.fx() == model.fy()); // we only support undistorted images

    if (tracklets_msg->stamps.size() == 0) {
        // tracks come only if one or more images have benn seen
        // to not loose the info from the very first image, the first keyframe will be
        // correspodning to the oldest stamp in message.
        ROS_DEBUG_STREAM("In MonoStandalone: no tracklet data yet, return");
        return;
    }

    ros::Time cur_ts_ros = tracklets_msg->stamps.front();
    keyframe_bundle_adjustment::Tracklets tracklets = matches_msg_conversions_ros::Convert(tracklets_msg);

    // initiate camera
    keyframe_bundle_adjustment::Camera::Ptr camera = std::make_shared<keyframe_bundle_adjustment::Camera>(
        model.fx(), Eigen::Vector2d(model.cx(), model.cy()), trf_camera_vehicle);

    if (bundle_adjuster_->keyframes_.size() > 0) {
        // in mono case second pose must be fixed in scale
        keyframe_bundle_adjustment::Keyframe::FixationStatus fixation_status{
            keyframe_bundle_adjustment::Keyframe::FixationStatus::None};

        // get motion corresponding to last keyframe
        auto start_time_5point = std::chrono::steady_clock::now();
        Eigen::Isometry3d motion_prior_vehicle_t1_t0 =
            helpers::getMotionUnscaled(model.fx(),
                                       cv::Point2d(model.cx(), model.cy()),
                                       cur_ts_ros.toNSec(),
                                       bundle_adjuster_->getKeyframe().timestamp_,
                                       tracklets,
                                       trf_camera_vehicle);
        ss << "Duration 5 point="
           << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                    start_time_5point)
                  .count()
           << " ms" << std::endl;

        Eigen::Isometry3d pose_prior_keyframe_origin =
            motion_prior_vehicle_t1_t0 * bundle_adjuster_->getKeyframe().getEigenPose();

        // select keyframe
        auto start_time_adjust_pose = std::chrono::steady_clock::now();
        keyframe_bundle_adjustment::Plane ground_plane;
        ground_plane.distance = interface_.height_over_ground;
        auto cur_frame = std::make_shared<keyframe_bundle_adjustment::Keyframe>(
            cur_ts_ros.toNSec(), tracklets, camera, pose_prior_keyframe_origin, fixation_status, ground_plane);

        std::string summary_motion_only = bundle_adjuster_->adjustPoseOnly(*cur_frame);
        ss << "Duration pose only="
           << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                    start_time_adjust_pose)
                  .count()
           << " ms" << std::endl;
        //        std::cout << "---------------------------- motion only ------------------------" << std::endl;
        //        std::cout << summary_motion_only << std::endl;

        auto start_time_select_kf = std::chrono::steady_clock::now();
        std::set<keyframe_bundle_adjustment::Keyframe::Ptr> selected_frames =
            keyframe_selector_.select({cur_frame}, bundle_adjuster_->getActiveKeyframePtrs());
        assert(selected_frames.size() < 2);
        int64_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                                 start_time_select_kf)
                               .count();
        ss << "Duration select frames=" << duration << " ms\n";

        auto start_time_push = std::chrono::steady_clock::now();
        // add keyframes to bundle adjustment
        for (const auto& kf : selected_frames) {
            bundle_adjuster_->push(*kf);
        }
        ss << "Duration push="
           << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_push)
                  .count()
           << " ms" << std::endl;
        ROS_DEBUG_STREAM("number keyframes " << bundle_adjuster_->keyframes_.size());
        ROS_INFO_STREAM("number selected keyframes " << selected_frames.size());
        ROS_INFO_STREAM("number active keyframes " << bundle_adjuster_->active_keyframe_ids_.size());
        ROS_INFO_STREAM("number active landmarks " << bundle_adjuster_->active_landmark_ids_.size());
        ROS_INFO_STREAM("number selected landmarks " << bundle_adjuster_->selected_landmark_ids_.size());

        // do bundle adjustment
        if (selected_frames.size() > 0 && bundle_adjuster_->keyframes_.size() > 2 &&
            cur_ts_ros.toSec() - last_ts_solved_ > interface_.time_between_keyframes_sec) {

            auto start_time_update_deactivate = std::chrono::steady_clock::now();
            // deactivate keyframes that are not connected to the current frame
            bundle_adjuster_->deactivateKeyframes();
            bundle_adjuster_->updateLabels(tracklets, interface_.shrubbery_weight);
            ss << "Duration update,deactivate="
               << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                        start_time_update_deactivate)
                      .count()
               << " ms" << std::endl;

            // do optimization
            auto start_time_solve = std::chrono::steady_clock::now();
            std::string summary = bundle_adjuster_->solve();
            //            for (const auto& el : bundle_adjuster_->keyframes_) {
            //                auto fix = el.second->fixation_status_;
            //                if (fix == keyframe_bundle_adjustment::Keyframe::FixationStatus::None) {
            //                    std::cout << "none" << std::endl;
            //                } else if (fix == keyframe_bundle_adjustment::Keyframe::FixationStatus::Scale) {
            //                    std::cout << "scale" << std::endl;
            //                } else if (fix == keyframe_bundle_adjustment::Keyframe::FixationStatus::Pose) {
            //                    std::cout << "pose" << std::endl;
            //                } else {
            //                    std::cout << "sth wrong" << std::endl;
            //                }
            //            }
            last_ts_solved_ = cur_ts_ros.toSec();
            ROS_INFO_STREAM("In MonoStandalone:" << summary);
            ss << "In MonoStandalone: Duration solve="
               << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                        start_time_solve)
                      .count()
               << " ms\n";

            auto start_time_publish_stuff = std::chrono::steady_clock::now();
            // convert poses to pose constraint array and publish
            // auto out_msg = helpers::convertToOutmsg(
            //     tracklets_msg->header.stamp, bundle_adjuster_, interface_.calib_source_frame_id);
            // interface_.trajectory_publisher.publish(out_msg);
            // ROS_DEBUG_STREAM("In MonoStandalone: published " << out_msg.constraints.size()
            //                                                  << " pose delta constraints to topic "
            //                                                  << interface_.trajectory_publisher_topic);
            if (interface_.show_debug_image) {
                cv::imshow("flow debug image keyframes", helpers::getFlowImg(bundle_adjuster_));
                cv::waitKey(30);
            }

            if (interface_.path_publisher_topic != "" && interface_.active_path_publisher_topic != "") {
                helpers::publishPaths(interface_.path_publisher,
                                      interface_.active_path_publisher,
                                      bundle_adjuster_,
                                      interface_.tf_parent_frame_id);
            }

            if (interface_.landmarks_publisher_topic != "") {
                helpers::publishLandmarks(
                    interface_.landmarks_publisher, bundle_adjuster_, interface_.tf_parent_frame_id);
            }

            if (interface_.planes_publisher_topic != "") {
                helpers::publishPlanes(interface_.planes_publisher, bundle_adjuster_, interface_.tf_parent_frame_id);
            }

            ss << "Duration publish="
               << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                        start_time_publish_stuff)
                      .count()
               << " ms" << std::endl;
        }
    } else {
        // first received frame is always a keyframe and has fixed pose
        ros::Time ts_oldest_feature = tracklets_msg->stamps.back();
        keyframe_bundle_adjustment::Plane ground_plane;
        ground_plane.distance = interface_.height_over_ground;
        keyframe_bundle_adjustment::Keyframe cur_frame(ts_oldest_feature.toNSec(),
                                                       tracklets,
                                                       camera,
                                                       Eigen::Isometry3d::Identity(),
                                                       keyframe_bundle_adjustment::Keyframe::FixationStatus::Pose,
                                                       ground_plane);
        bundle_adjuster_->push(cur_frame);
        ROS_DEBUG_STREAM("added first keyframe");
    }

    auto start_time_send_pose_tf = std::chrono::steady_clock::now();
    ros::Time timestamp_last_kf;
    timestamp_last_kf.fromNSec(bundle_adjuster_->getKeyframe().timestamp_);
    maybeSendPoseTf(timestamp_last_kf, bundle_adjuster_->getKeyframe().getEigenPose());
    ss << "Duration tf="
       << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                start_time_send_pose_tf)
              .count()
       << " ms" << std::endl;

    // publisherDiagnosed_->publish(new_msg);
    // The updater will take care of publishing at a throttled rate
    // When calling update, all updater callbacks (defined in setupDiagnostics) will be run
    updater_.update();

    auto duration =
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start_time);
    ss << "time callback=" << duration.count() << " sec\n";
    ROS_INFO_STREAM(ss.str());
}


/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure
 * window
*/
void MonoStandalone::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);

    // lm selection scheme without resetting ba
    //    bundle_adjuster_ = std::make_shared<keyframe_bundle_adjustment::BundleAdjusterKeyframes>(
    //        keyframe_bundle_adjustment::LandmarkSelectionSchemeObservability::create(p));
    bundle_adjuster_->landmark_selector_ = std::make_unique<keyframe_bundle_adjustment::LandmarkSelector>();
    // first is always cheirality since it is fast and reliable
    bundle_adjuster_->landmark_selector_->addScheme(
        keyframe_bundle_adjustment::LandmarkRejectionSchemeCheirality::create());
    // add custom lm selection
    //    keyframe_bundle_adjustment::LandmarkSparsificationSchemeObservability::Parameters p;
    //    p.bin_params_.max_num_landmarks_near = interface_.max_number_landmarks_near_bin;
    //    p.bin_params_.max_num_landmarks_middle = interface_.max_number_landmarks_middle_bin;
    //    p.bin_params_.max_num_landmarks_far = interface_.max_number_landmarks_far_bin;
    //    bundle_adjuster_->landmark_selector_->addScheme(
    //        keyframe_bundle_adjustment::LandmarkSparsificationSchemeObservability::create(p));
    keyframe_bundle_adjustment::LandmarkSparsificationSchemeVoxel::Parameters p;
    p.max_num_landmarks_near = interface_.max_number_landmarks_near_bin;
    p.max_num_landmarks_middle = interface_.max_number_landmarks_middle_bin;
    p.max_num_landmarks_far = interface_.max_number_landmarks_far_bin;
    p.roi_middle_xyz = std::array<double, 3>{10., 10., 10.};

    bundle_adjuster_->landmark_selector_->addScheme(
        keyframe_bundle_adjustment::LandmarkSparsificationSchemeVoxel::create(p));

    keyframe_bundle_adjustment::LandmarkSelectionSchemeAddDepth::Parameters p_add_depth;
    auto gp_comparator = [](const keyframe_bundle_adjustment::Landmark::ConstPtr& lm) { return lm->is_ground_plane; };
    auto gp_sorter = [](const keyframe_bundle_adjustment::Measurement& m, const Eigen::Vector3d& local_lm) {
        return local_lm.norm();
    };
    p_add_depth.params_per_keyframe.clear();
    for (int i = 0; i < 30; i = i + 2) {
        p_add_depth.params_per_keyframe.push_back(std::make_tuple(i, 50, gp_comparator, gp_sorter));
    }

    bundle_adjuster_->landmark_selector_->addScheme(
        keyframe_bundle_adjustment::LandmarkSelectionSchemeAddDepth::create(p_add_depth));

    bundle_adjuster_->outlier_rejection_options_.reprojection_thres = interface_.robust_loss_reprojection_thres;
    bundle_adjuster_->outlier_rejection_options_.reprojection_quantile = interface_.outlier_rejection_quantile;
    bundle_adjuster_->outlier_rejection_options_.num_iterations = interface_.outlier_rejection_num_iterations;

    // reset solver time
    bundle_adjuster_->set_solver_time(interface_.max_solver_time);

    // reset keyframe selector
    keyframe_selector_ = keyframe_bundle_adjustment::KeyframeSelector();
    keyframe_selector_.addScheme(
        keyframe_bundle_adjustment::KeyframeRejectionSchemeFlow::create(interface_.min_median_flow));
    keyframe_selector_.addScheme(
        keyframe_bundle_adjustment::KeyframeSelectionSchemePose::create(interface_.critical_rotation_difference));
    keyframe_selector_.addScheme(
        keyframe_bundle_adjustment::KeyframeSparsificationSchemeTime::create(interface_.time_between_keyframes_sec));

    // Read outlier ids and set them in bundle_adjuster
    bundle_adjuster_->labels_["outliers"] =
        keyframe_bundle_adjustment_ros_tool::helpers::loadSetFromYaml(interface_.outlier_labels_yaml, "outlier_labels");
    std::stringstream ss;
    ss << "MonoLidar: outlier labels " << std::endl;
    for (const auto& el : bundle_adjuster_->labels_["outliers"]) {
        ss << " " << el;
    }
    ss << std::endl;
    ROS_DEBUG_STREAM(ss.str());

    bundle_adjuster_->labels_["shrubbery"] = keyframe_bundle_adjustment_ros_tool::helpers::loadSetFromYaml(
        interface_.outlier_labels_yaml, "shrubbery_labels");
    ss.str("");
    ss << "MonoLidar: shrubbery weight: " << interface_.shrubbery_weight << " labels: " << std::endl;
    for (const auto& el : bundle_adjuster_->labels_["shrubbery"]) {
        ss << " " << el;
    }
    ss << std::endl;
    ROS_DEBUG_STREAM(ss.str());

    // If the publisherDiagnosed_ is configurable, you should update it here.
    // This introduces almost no overhead, no need to check if this is actually necessary.
    // publisherDiagnosed_ = std::make_unique<diagnostic_updater::DiagnosedPublisher<Msg>>(
    //     interface_.dummy_publisher, updater_,
    //     diagnostic_updater::FrequencyStatusParam(&interface_.diagnostic_updater_rate,
    //                                              &interface_.diagnostic_updater_rate,
    //                                              interface_.diagnostic_updater_rate_tolerance,
    //                                              5),
    //     diagnostic_updater::TimeStampStatusParam());
}

/*
 * Setup the Diagnostic Updater
 */
void MonoStandalone::setupDiagnostics() {
    // Give a unique hardware id
    diagnostic_status_.hardware_id = interface_.diagnostic_updater_hardware_id;
    diagnostic_status_.message = "Starting...";
    diagnostic_status_.level = diagnostic_msgs::DiagnosticStatus::STALE;
    updater_.setHardwareID(interface_.diagnostic_updater_hardware_id);

    // Add further callbacks (or unittests) that should be called regularly
    updater_.add("MonoStandalone Sensor Status", this, &MonoStandalone::checkSensorStatus);

    updater_.force_update();
}

void MonoStandalone::checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper) {
    status_wrapper.summary(diagnostic_status_);
    diagnostic_status_.message = "Valid operation";
    diagnostic_status_.level = diagnostic_msgs::DiagnosticStatus::OK;
}


void MonoStandalone::maybeSendPoseTf(ros::Time timestamp, Eigen::Isometry3d pose) {
    if (interface_.tf_parent_frame_id != "" && interface_.tf_child_frame_id != "") {

        geometry_msgs::TransformStamped transf;
        helpers::toGeometryMsg(transf.transform, pose);

        // set ids and stamp
        transf.header.stamp = timestamp;
        transf.header.frame_id = interface_.tf_parent_frame_id;
        transf.child_frame_id = interface_.tf_child_frame_id;

        // send it
        tf_broadcaster_.sendTransform(transf);
    }
}


} // namespace keyframe_bundle_adjustment_ros_tool
