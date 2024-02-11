#include "mono_lidar.hpp"
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Eigen>

#include <chrono>
#include <image_geometry/pinhole_camera_model.h>
#include <nav_msgs/Path.h>
#include <commons/color_by_index_hsv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp> //attention, eigen must be icluded before that!

#include <commons/general_helpers.hpp>
#include <commons/publish_helpers.hpp>

#include <matches_msg_conversions_ros/convert.hpp>

#include <chrono>
#include <fstream>


namespace keyframe_bundle_adjustment_ros_tool {

MonoLidar::MonoLidar(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
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
     * New subscribers can be created with "add_subscriber" in "cfg/MonoLidar.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigure_server_.setCallback(boost::bind(&MonoLidar::reconfigureRequest, this, _1, _2));

    // setup synchronizer
    sync_ = std::make_unique<Synchronizer>(
        SyncPolicy(100), *(interface_.tracklets_subscriber), *(interface_.camera_info_subscriber));
    sync_->registerCallback(boost::bind(&MonoLidar::callbackSubscriber, this, _1, _2));

    // get extrinsics from tf, this is the pose from vehicle to camera frame!
    ROS_DEBUG_STREAM("MonoLidar: Getting transform from vehicle to camera frame");
    try {
        Eigen::Affine3d ext_aff = tf2::transformToEigen(tf_buffer_.lookupTransform(
            interface_.calib_target_frame_id, interface_.calib_source_frame_id, ros::Time::now(), ros::Duration(5.)));
        // attention this only works if affine transform is pure rotation and translation
        trf_camera_vehicle = Eigen::Isometry3d(ext_aff.matrix());
    } catch (const tf2::LookupException& exc) {
        ROS_ERROR_STREAM(exc.what());
        ROS_ERROR_STREAM("MonoLidar: Set calib to identity and continue");
        trf_camera_vehicle = Eigen::Isometry3d::Identity();
    }
    ROS_DEBUG_STREAM("MonoLidar: Transform from vehicle to camera frame=\n" << trf_camera_vehicle.matrix());
    last_pose_origin_camera = Eigen::Isometry3d::Identity();
    accumulated_motion = Eigen::Isometry3d::Identity();

    rosinterface_handler::showNodeInfo();
}

MonoLidar::~MonoLidar() {
    // On shutdown write doen hole pointcloud and all poses.
    helpers::dumpMap("/tmp/mono_lidar_map_dump.yaml", bundle_adjuster_);
}


void MonoLidar::callbackSubscriber(const TrackletsMsg::ConstPtr& tracklets_msg,
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
        ROS_DEBUG_STREAM("In MonoLidar: no tracklet data yet, return");
        return;
    }

    ros::Time cur_ts_ros = tracklets_msg->stamps.front();

    keyframe_bundle_adjustment::Tracklets tracklets = matches_msg_conversions_ros::Convert(tracklets_msg);

    // initiate camera
    keyframe_bundle_adjustment::Camera::Ptr camera = std::make_shared<keyframe_bundle_adjustment::Camera>(
        model.fx(), Eigen::Vector2d(model.cx(), model.cy()), trf_camera_vehicle);

    if (bundle_adjuster_->keyframes_.size() > 0) {

        Eigen::Isometry3d pose_prior_keyframe_origin;

        bool has_external_prior = (interface_.prior_vehicle_frame != "");
        if (has_external_prior) {
            // get pose prior to current frame
            auto last_kf = bundle_adjuster_->getKeyframe();

            ros::Time last_ts_ros;
            last_ts_ros.fromNSec(last_kf.timestamp_);

            auto start_get_pose_prior = std::chrono::steady_clock::now();
            Eigen::Affine3d pose_prior_cur_kf, pose_prior_kf_orig;
            pose_prior_kf_orig = last_kf.getEigenPose();
            ROS_DEBUG_STREAM("MonoLidar: Getting prior from tf...");
            geometry_msgs::TransformStamped motion_cur_kf;
            try {
                motion_cur_kf = tf_buffer_.lookupTransform(interface_.prior_vehicle_frame,
                                                           cur_ts_ros,
                                                           interface_.prior_vehicle_frame,
                                                           last_ts_ros,
                                                           interface_.prior_global_frame,
                                                           ros::Duration(interface_.tf_waiting_time));
            } catch (const tf2::TransformException& e) {
                ROS_ERROR_STREAM(e.what());
            }
            tf2::doTransform(pose_prior_kf_orig, pose_prior_cur_kf, motion_cur_kf);
            ss << "In MonoLidar: Duration get pose prior from tf="
               << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                        start_get_pose_prior)
                      .count()
               << " ms" << std::endl;
            ROS_DEBUG_STREAM("In MonoLidar: pose_prior for kf=\n" << pose_prior_cur_kf.matrix());
            pose_prior_keyframe_origin.translation() = pose_prior_cur_kf.translation();
            pose_prior_keyframe_origin.linear() = pose_prior_cur_kf.rotation();
        } else {
            // get motion in camera frame from keyframe to current instant
            // use 5 point algorithm and adjust translation
            auto start_time_5_point = std::chrono::steady_clock::now();
            Eigen::Isometry3d motion_vehicle_t1_t0 =
                helpers::getMotionUnscaled(model.fx(),
                                           cv::Point2d(model.cx(), model.cy()),
                                           cur_ts_ros.toNSec(),
                                           bundle_adjuster_->getKeyframe().timestamp_,
                                           tracklets,
                                           trf_camera_vehicle,
                                           interface_.prior_speed);
            ss << "Duration 5 point="
               << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                        start_time_5_point)
                      .count()
               << " ms" << std::endl;
            auto kf_ptrs = bundle_adjuster_->getSortedActiveKeyframePtrs();
            if (kf_ptrs.size() > 1) {
                // Get vehicle speed from last keyframes.
                auto r_it = kf_ptrs.crbegin();
                auto r_it1 = std::next(r_it);
                Eigen::Vector3d dtrans = ((*r_it)->getEigenPose() * (*r_it1)->getEigenPose().inverse()).translation();
                double dt = keyframe_bundle_adjustment::convert((*r_it)->timestamp_) -
                            keyframe_bundle_adjustment::convert((*r_it1)->timestamp_);
                double speed = dtrans.norm() / dt;
                std::cout << "speed=" << speed << std::endl;

                // Scale translation according to speed
                motion_vehicle_t1_t0.translation() =
                    motion_vehicle_t1_t0.translation() / std::max(0.0001, motion_vehicle_t1_t0.translation().norm());
                motion_vehicle_t1_t0.translation() =
                    motion_vehicle_t1_t0.translation() * speed *
                    (cur_ts_ros.toSec() - keyframe_bundle_adjustment::convert((*r_it)->timestamp_));
            }
            pose_prior_keyframe_origin = motion_vehicle_t1_t0 * kf_ptrs.back()->getEigenPose();
        }

        // select keyframe
        keyframe_bundle_adjustment::Plane ground_plane;
        ground_plane.distance = interface_.height_over_ground;
        auto cur_frame = std::make_shared<keyframe_bundle_adjustment::Keyframe>(
            cur_ts_ros.toNSec(),
            tracklets,
            camera,
            pose_prior_keyframe_origin,
            keyframe_bundle_adjustment::Keyframe::FixationStatus::None,
            ground_plane);

        // If the prior was without scale, adjust it.
        if (!has_external_prior) {
            auto start_time = std::chrono::steady_clock::now();
            std::string summary_motion_only = bundle_adjuster_->adjustPoseOnly(*cur_frame);
            pose_prior_keyframe_origin = cur_frame->getEigenPose();
            std::cout << "---------------------------- motion only ------------------------" << std::endl;
            std::cout << summary_motion_only << std::endl;
            ss << "Duration pose only adjustment="
               << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time)
                      .count()
               << " ms" << std::endl;
        }

        ROS_DEBUG_STREAM(
            "Motion_prior=\n"
            << (pose_prior_keyframe_origin * bundle_adjuster_->getKeyframe().getEigenPose().inverse()).matrix());

        ROS_DEBUG_STREAM("In MonoLidar: select frames");
        auto start_time_select_kf = std::chrono::steady_clock::now();
        std::set<keyframe_bundle_adjustment::Keyframe::Ptr> selected_frames =
            keyframe_selector_.select({cur_frame}, bundle_adjuster_->getActiveKeyframePtrs());
        assert(selected_frames.size() < 2);
        int64_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                                 start_time_select_kf)
                               .count();
        ss << "Duration select keyframes=" << duration << " ms\n";

        // add keyframes to bundle adjustment
        auto start_time_push = std::chrono::steady_clock::now();
        for (const auto& kf : selected_frames) {
            bundle_adjuster_->push(*kf);
        }
        ss << "Duration push="
           << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_push)
                  .count()
           << " ms" << std::endl;
        ROS_DEBUG_STREAM("In MonoLidar: number keyframes " << bundle_adjuster_->keyframes_.size());
        ROS_INFO_STREAM("In MonoLidar: number selected keyframes " << selected_frames.size());
        ROS_INFO_STREAM("In MonoLidar: number active keyframes " << bundle_adjuster_->active_keyframe_ids_.size());
        ROS_INFO_STREAM("In MonoLidar: number active landmarks " << bundle_adjuster_->active_landmark_ids_.size());
        ROS_INFO_STREAM("In MonoLidar: number selected landmarks " << bundle_adjuster_->selected_landmark_ids_.size());

        // do bundle adjustment all interface_.time_between_keyframes
        if (bundle_adjuster_->keyframes_.size() > 2 &&
            cur_ts_ros.toSec() - last_ts_solved_.toSec() > 0.98 * interface_.time_between_keyframes_sec) {

            // deactivate keyframes that are not connected to the current frame
            // It is important that all keyframes shall be maintained in curves.
            // Therefore we only deactivate keyframes if the angle is not smaller than a threshold
            bundle_adjuster_->deactivateKeyframes(
                interface_.min_number_connecting_landmarks, 3, interface_.max_size_optimization_window);
            bundle_adjuster_->updateLabels(tracklets, interface_.shrubbery_weight);

            // do optimization
            auto start_time_solve = std::chrono::steady_clock::now();
            std::string summary = bundle_adjuster_->solve();
            last_ts_solved_ = cur_ts_ros;
            ROS_INFO_STREAM("In MonoLidar:" << summary);
            ss << "In MonoLidar: Duration solve="
               << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                        start_time_solve)
                      .count()
               << " ms" << std::endl;
            for (const auto& el : bundle_adjuster_->getActiveKeyframeConstPtrs()) {
                std::cout << "plane dist=" << el.second->local_ground_plane_.distance << std::endl;
            }

            if (interface_.show_debug_image) {
                cv::imshow("flow debug image keyframes", helpers::getFlowImg(bundle_adjuster_));
                cv::waitKey(30);
            }
        }


        // hack:: dump pose if filepath is non empty
        if (interface_.dump_path != "") {
            std::ofstream file;
            file.open(interface_.dump_path.c_str(), std::ios_base::app);
            file.precision(12);
            Eigen::Isometry3d pose_origin_camera;

            if (bundle_adjuster_->getKeyframe().timestamp_ == tracklets_msg->stamps[0].toNSec()) {
                // if current frame is chosen as keyframe, we can dump that one
                pose_origin_camera = trf_camera_vehicle * bundle_adjuster_->getKeyframe().getEigenPose().inverse() *
                                     trf_camera_vehicle.inverse();
                ROS_DEBUG_STREAM("In MonoLidar: dump_optimized_pose");
            } else {
                // otherwise we dump the current prior
                pose_origin_camera =
                    trf_camera_vehicle * pose_prior_keyframe_origin.inverse() * trf_camera_vehicle.inverse();
                ROS_DEBUG_STREAM("In MonoLidar: dump prior");
            }

            file << std::endl << helpers::poseToString(pose_origin_camera.matrix()) << std::flush;
            file.close();
            ROS_DEBUG_STREAM("In MonoLidar: dumped pose=\n"
                             << pose_origin_camera.matrix()
                             << "\nto "
                             << interface_.dump_path);
            last_pose_origin_camera = pose_origin_camera;
        }
    } else {
        // Since tracker only outputs stuff when we have at least 2 images inserted, the first time this is called,
        // we have 2 measurements already. Set oldest frame as keyframe
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
        ROS_DEBUG_STREAM("In MonoLidar: added first keyframe");

        // hack:: dump pose if filepath is non empty
        if (interface_.dump_path != "") {
            std::ofstream file;
            file.open(interface_.dump_path.c_str());
            file.precision(12);
            file << helpers::poseToString(Eigen::Matrix<double, 4, 4>::Identity());
            file.close();
        }
    }

    auto start_time_publish_stuff = std::chrono::steady_clock::now();
    // convert poses to pose constraint array and publish
    // auto out_msg =
    //     helpers::convertToOutmsg(tracklets_msg->header.stamp, bundle_adjuster_, interface_.calib_source_frame_id);
    // interface_.trajectory_publisher.publish(out_msg);
    // ROS_DEBUG_STREAM("In MonoLidar: published " << out_msg.constraints.size() << " pose delta constraints to topic "
    //                                             << interface_.trajectory_publisher_topic);

    if (interface_.path_publisher_topic != "" && interface_.active_path_publisher_topic != "") {
        helpers::publishPaths(interface_.path_publisher,
                              interface_.active_path_publisher,
                              bundle_adjuster_,
                              interface_.tf_parent_frame_id);
    }

    if (interface_.landmarks_publisher_topic != "") {
        helpers::publishLandmarks(interface_.landmarks_publisher, bundle_adjuster_, interface_.tf_parent_frame_id);
    }

    if (interface_.planes_publisher_topic != "") {
        helpers::publishPlanes(interface_.planes_publisher, bundle_adjuster_, interface_.tf_parent_frame_id);
    }
    ss << "Duration publish="
       << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                start_time_publish_stuff)
              .count()
       << " ms" << std::endl;

    ros::Time timestamp_last_kf;
    timestamp_last_kf.fromNSec(bundle_adjuster_->getKeyframe().timestamp_);
    auto start_time_send_tf = std::chrono::steady_clock::now();
    maybeSendPoseTf(timestamp_last_kf, bundle_adjuster_->getKeyframe().getEigenPose());
    ss << "Duration send pose tf="
       << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_send_tf)
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
void MonoLidar::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);

    // lm selection scheme without resetting ba
    bundle_adjuster_->landmark_selector_ = std::make_unique<keyframe_bundle_adjustment::LandmarkSelector>();

    // first is always cheirality since it is fast and reliable
    bundle_adjuster_->landmark_selector_->addScheme(
        keyframe_bundle_adjustment::LandmarkRejectionSchemeCheirality::create());
    //    {
    //        keyframe_bundle_adjustment::LandmarkRejectionSchemeDimensionPlausibility::Params p_dim;
    //        p_dim.min_z = -5.;
    //        p_dim.max_z = 30.;
    //        bundle_adjuster_->landmark_selector_->addScheme(
    //            keyframe_bundle_adjustment::LandmarkRejectionSchemeDimensionPlausibility::create(p_dim));
    //    }
    // Sparsify landmarks with voxelgrid
    // get parameters
    keyframe_bundle_adjustment::LandmarkSparsificationSchemeVoxel::Parameters p;
    p.max_num_landmarks_near = interface_.max_number_landmarks_near_bin;
    p.max_num_landmarks_middle = interface_.max_number_landmarks_middle_bin;
    p.max_num_landmarks_far = interface_.max_number_landmarks_far_bin;

    p.voxel_size_xyz = std::array<double, 3>{0.5, 0.5, 0.3}; // Voxelsize in x,y,z in meters
    p.roi_far_xyz =
        std::array<double, 3>{40., 40., 40.}; // Roi around current position in hwich points will be considered
    p.roi_middle_xyz =
        std::array<double, 3>{15., 15., 15.}; // Roi around current position in hwich points will be considered

    bundle_adjuster_->landmark_selector_->addScheme(
        keyframe_bundle_adjustment::LandmarkSparsificationSchemeVoxel::create(p));
    // Make parameters for depth assurance.
    // On the zeroth frame (newest frame) we assure 20 measurements with depth measurement.
    keyframe_bundle_adjustment::LandmarkSelectionSchemeAddDepth::Parameters p_add_depth;
    //    p_add_depth.params_per_keyframe.push_back(std::make_tuple(
    //        0,
    //        20,
    //        [](const keyframe_bundle_adjustment::Landmark::ConstPtr& lm) { return lm->has_measured_depth; },
    //        [](const keyframe_bundle_adjustment::Measurement& m, const Eigen::Vector3d& lm) { return m.d; }));
    auto gp_comparator = [](const keyframe_bundle_adjustment::Landmark::ConstPtr& lm) { return lm->is_ground_plane; };
    auto gp_sorter = [](const keyframe_bundle_adjustment::Measurement& m, const Eigen::Vector3d& local_lm) {
        return local_lm.norm();
    };
    p_add_depth.params_per_keyframe.clear();

    for (int i = 0; i < interface_.max_size_optimization_window; i++) {
        p_add_depth.params_per_keyframe.push_back(std::make_tuple(i, 50, gp_comparator, gp_sorter));
    }

    bundle_adjuster_->landmark_selector_->addScheme(
        keyframe_bundle_adjustment::LandmarkSelectionSchemeAddDepth::create(p_add_depth));

    // reset robust loss paramters
    bundle_adjuster_->outlier_rejection_options_.depth_thres = interface_.robust_loss_depth_thres;
    bundle_adjuster_->outlier_rejection_options_.depth_quantile = interface_.outlier_rejection_quantile;
    bundle_adjuster_->outlier_rejection_options_.reprojection_thres = interface_.robust_loss_reprojection_thres;
    bundle_adjuster_->outlier_rejection_options_.reprojection_quantile = interface_.outlier_rejection_quantile;
    bundle_adjuster_->outlier_rejection_options_.num_iterations = interface_.outlier_rejection_num_iterations;

    ROS_DEBUG_STREAM("Depth thres=" << interface_.robust_loss_depth_thres);
    ROS_DEBUG_STREAM("Repr thres=" << interface_.robust_loss_reprojection_thres);
    ROS_DEBUG_STREAM("Quantile=" << interface_.outlier_rejection_quantile);
    ROS_DEBUG_STREAM("Number iterations=" << interface_.outlier_rejection_num_iterations);

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
void MonoLidar::setupDiagnostics() {
    // Give a unique hardware id
    diagnostic_status_.hardware_id = interface_.diagnostic_updater_hardware_id;
    diagnostic_status_.message = "Starting...";
    diagnostic_status_.level = diagnostic_msgs::DiagnosticStatus::STALE;
    updater_.setHardwareID(interface_.diagnostic_updater_hardware_id);

    // Add further callbacks (or unittests) that should be called regularly
    updater_.add("MonoLidar Sensor Status", this, &MonoLidar::checkSensorStatus);

    updater_.force_update();
}

void MonoLidar::checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper) {
    status_wrapper.summary(diagnostic_status_);
    diagnostic_status_.message = "Valid operation";
    diagnostic_status_.level = diagnostic_msgs::DiagnosticStatus::OK;
}

void MonoLidar::maybeSendPoseTf(ros::Time timestamp, Eigen::Isometry3d pose) {
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
