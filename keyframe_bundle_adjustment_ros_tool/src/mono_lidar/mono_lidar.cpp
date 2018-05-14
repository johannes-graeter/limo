#include "mono_lidar.hpp"
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <image_geometry/pinhole_camera_model.h>

#include <commons/general_helpers.hpp>
#include <commons/motion_extrapolator.hpp>
#include <commons/publish_helpers.hpp>

#include <yaml-cpp/yaml.h>

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

    // Init pimpl for scale provider.
    motion_provider_ = std::make_shared<MotionExtrapolator>();


    rosinterface_handler::showNodeInfo();
}

MonoLidar::~MonoLidar() {
    // On shutdown write doen hole pointcloud and all poses.
    std::stringstream ss;
    ss << "/tmp/mono_lidar_map_dump.yaml";
    std::ofstream file(ss.str().c_str());
    file.precision(12);

    file << "landmarks with depth: [";
    for (const auto& id_lm_ptr : bundle_adjuster_->landmarks_) {
        if (id_lm_ptr.second->has_measured_depth) {
            file << "[" << id_lm_ptr.second->pos[0] << ", " << id_lm_ptr.second->pos[1] << ", "
                 << id_lm_ptr.second->pos[2] << ", " << id_lm_ptr.second->weight << "],\n";
        }
    }
    file << "]\n";

    file << "landmarks without depth: [";
    for (const auto& id_lm_ptr : bundle_adjuster_->landmarks_) {
        if (!id_lm_ptr.second->has_measured_depth) {
            file << "[" << id_lm_ptr.second->pos[0] << ", " << id_lm_ptr.second->pos[1] << ", "
                 << id_lm_ptr.second->pos[2] << ", " << id_lm_ptr.second->weight << "],\n";
        }
    }
    file << "]\n";


    file << "poses: {";
    for (const auto& id_kf_ptr : bundle_adjuster_->keyframes_) {
        file << id_kf_ptr.second->timestamp_ << ": [" << id_kf_ptr.second->pose_[0] << ", "
             << id_kf_ptr.second->pose_[1] << ", " << id_kf_ptr.second->pose_[2] << ", " << id_kf_ptr.second->pose_[3]
             << ", " << id_kf_ptr.second->pose_[4] << ", " << id_kf_ptr.second->pose_[5] << ", "
             << id_kf_ptr.second->pose_[6] << "],\n";
    }
    file << "}";

    file.close();

    std::cout << "--------------------------------------\nDumped map to " << ss.str()
              << "--------------------------------------\n"
              << std::endl;
}

// namespace {
// double calcQuaternionDiff(const std::array<double, 7>& p0, const std::array<double, 7>& p1) {
//    Eigen::Quaterniond q0(p0[0], p0[1], p0[2], p0[3]);
//    Eigen::Quaterniond q1(p1[0], p1[1], p1[2], p1[3]);

//    Eigen::AngleAxisd a(q1.inverse() * q0);
//    return a.angle();
//}
//}

void MonoLidar::callbackSubscriber(const TrackletsMsg::ConstPtr& tracklets_msg,
                                   const CameraInfoMsg::ConstPtr& camera_info_msg) {
    auto start_time = std::chrono::steady_clock::now();

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
            ROS_WARN_STREAM(e.what());
            ROS_WARN_STREAM("MonoLidar: interpolate old motion.");
            Eigen::Affine3d cur_motion(motion_provider_->getMotion(cur_ts_ros.toNSec()));
            motion_cur_kf = tf2::eigenToTransform(cur_motion);
        }

        ROS_DEBUG_STREAM("In MonoLidar: prior translation=" << motion_cur_kf.transform.translation.x << " "
                                                            << motion_cur_kf.transform.translation.y
                                                            << " "
                                                            << motion_cur_kf.transform.translation.z);

        //  Interpolate last scales and apply it to prior
        //  Is that better than constant norm?
        // Set current estimations to predict next scale by linear interpolation
        motion_provider_->setLastKeyframePtrs(bundle_adjuster_->getActiveKeyframeConstPtrs());
        //        Eigen::Vector3d transl = motion_provider_->applyScale(cur_ts_ros.toNSec(),
        //                                                              Eigen::Vector3d(motion_cur_kf.transform.translation.x,
        //                                                                              motion_cur_kf.transform.translation.y,
        //                                                                              motion_cur_kf.transform.translation.z));
        //        motion_cur_kf.transform.translation.x = transl[0];
        //        motion_cur_kf.transform.translation.y = transl[1];
        //        motion_cur_kf.transform.translation.z = transl[2];
        //        ROS_DEBUG_STREAM("In MonoLidar: got scale=" << transl.norm());

        tf2::doTransform(pose_prior_kf_orig, pose_prior_cur_kf, motion_cur_kf);
        ROS_INFO_STREAM("In MonoLidar: Duration get pose prior from tf="
                        << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                                 start_get_pose_prior)
                               .count()
                        << " ms");
        ROS_DEBUG_STREAM("In MonoLidar: pose_prior for kf=\n" << pose_prior_cur_kf.matrix());

        Eigen::Isometry3d pose_prior_origin_keyframe;
        pose_prior_origin_keyframe.translation() = pose_prior_cur_kf.translation();
        pose_prior_origin_keyframe.linear() = pose_prior_cur_kf.rotation();

        // select keyframe
        auto start_time_select_kf = std::chrono::steady_clock::now();
        auto cur_frame = std::make_shared<keyframe_bundle_adjustment::Keyframe>(
            cur_ts_ros.toNSec(), tracklets, camera, pose_prior_origin_keyframe);

        std::cout << "----------------Deb kf ros: " << cur_ts_ros.toNSec() << " created=" << cur_frame->timestamp_
                  << std::endl;


        ROS_DEBUG_STREAM("In MonoLidar: select frames");
        std::set<keyframe_bundle_adjustment::Keyframe::Ptr> selected_frames =
            keyframe_selector_.select({cur_frame}, bundle_adjuster_->getActiveKeyframePtrs());
        assert(selected_frames.size() < 2);
        int64_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                                 start_time_select_kf)
                               .count();

        // Calculate difference in angles, to deactivate or not deactivate keyframes.
        ///@todo integrate that in bundler? Better: choose a better connectivity criterium for deactivation
        //        double angle_diff =
        //            calcQuaternionDiff(*(cur_frame->getPosePtr()), *(bundle_adjuster_->getKeyframe().getPosePtr()));

        ROS_INFO_STREAM("In MonoLidar: duration select frames=" << duration << " ms");

        // add keyframes to bundle adjustment
        for (const auto& kf : selected_frames) {
            bundle_adjuster_->push(*kf);
        }
        ROS_DEBUG_STREAM("In MonoLidar: number keyframes " << bundle_adjuster_->keyframes_.size());
        ROS_INFO_STREAM("In MonoLidar: number selected keyframes " << selected_frames.size());
        ROS_INFO_STREAM("In MonoLidar: number active keyframes " << bundle_adjuster_->active_keyframe_ids_.size());
        ROS_INFO_STREAM("In MonoLidar: number active landmarks " << bundle_adjuster_->active_landmark_ids_.size());
        ROS_INFO_STREAM("In MonoLidar: number selected landmarks " << bundle_adjuster_->selected_landmark_ids_.size());

        // do bundle adjustment all interface_.time_between_keyframes
        if (selected_frames.size() > 0 && bundle_adjuster_->keyframes_.size() > 2 &&
            cur_ts_ros.toSec() - last_ts_solved_.toSec() > interface_.time_between_keyframes_sec) {

            // deactivate keyframes that are not connected to the current frame
            auto start_deactivate = std::chrono::steady_clock::now();

            // It is important that all keyframes shall be maintained in curves.
            // Therefore we only deactivate keyframes if the angle is not smaller than a threshold
            //            if (angle_diff < interface_.critical_rotation_difference) {
            bundle_adjuster_->deactivateKeyframes(interface_.min_number_connecting_landmarks);
            //            }
            ROS_INFO_STREAM("In MonoLidar: Duration deactivate kf="
                            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                                     start_deactivate)
                                   .count()
                            << " ms");
            bundle_adjuster_->updateLabels(tracklets, interface_.shrubbery_weight);

            // do optimization
            auto start_time_solve = std::chrono::steady_clock::now();
            std::string summary = bundle_adjuster_->solve();
            last_ts_solved_ = cur_ts_ros;
            ROS_INFO_STREAM("In MonoLidar:" << summary);
            ROS_INFO_STREAM("In MonoLidar: Duration solve=" << std::chrono::duration_cast<std::chrono::milliseconds>(
                                                                   std::chrono::steady_clock::now() - start_time_solve)
                                                                   .count()
                                                            << " ms");

            auto start_debug_publish = std::chrono::steady_clock::now();
            // convert poses to pose constraint array and publish
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

            ROS_INFO_STREAM("In MonoLidar: Duration publishing for rviz="
                            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                                     start_debug_publish)
                                   .count()
                            << " ms");
        }

        // hack:: dump pose if filepath is non empty
        if (interface_.dump_path != "") {
            std::ofstream file;
            file.open(interface_.dump_path.c_str(), std::ios_base::app);
            file.precision(12);
            Eigen::Matrix<double, 4, 4> pose_mat;
            if (bundle_adjuster_->getKeyframe().timestamp_ == tracklets_msg->stamps[0].toNSec()) {
                // if current frame is chosen as keyframe, we can dump that one
                pose_mat = (trf_camera_vehicle * bundle_adjuster_->getKeyframe().getEigenPose().inverse() *
                            trf_camera_vehicle.inverse())
                               .matrix();
                ROS_DEBUG_STREAM("In MonoLidar: dump_optimized_pose");
                std::stringstream ss;
                ss << "/tmp/poses_dump_keyframes.txt";
                std::ofstream file_kf(ss.str().c_str(), std::ios_base::app);
                file_kf << helpers::poseToString(pose_mat) << std::endl;
                file_kf.close();
            } else {
                // otherwise we dump the current prior
                pose_mat =
                    (trf_camera_vehicle * pose_prior_origin_keyframe.inverse() * trf_camera_vehicle.inverse()).matrix();
                ROS_DEBUG_STREAM("In MonoLidar: dump prior");
            }
            file << std::endl << helpers::poseToString(pose_mat) << std::flush;
            file.close();
            ROS_DEBUG_STREAM("In MonoLidar: dumped pose=\n" << pose_mat << "\nto " << interface_.dump_path);

            // Debug
            Eigen::Matrix<double, 4, 4> pose_mat_key =
                (trf_camera_vehicle * bundle_adjuster_->getKeyframe().getEigenPose().inverse() *
                 trf_camera_vehicle.inverse())
                    .matrix();
            std::stringstream ss;
            ss << "/tmp/poses_keyframes.txt";
            std::ofstream file_key(ss.str().c_str(), std::ios_base::app);
            file_key.precision(12);
            file_key << std::endl << helpers::poseToString(pose_mat_key) << std::flush;
            file_key.close();
        }

    } else {
        // Since tracker only outputs stuff when we have at least 2 images inserted, the first time this is called,
        // we have 2 measurements already. Set oldest frame as keyframe
        // first received frame is always a keyframe and has fixed pose
        ros::Time ts_oldest_feature = tracklets_msg->stamps.back();
        keyframe_bundle_adjustment::Keyframe cur_frame(ts_oldest_feature.toNSec(),
                                                       tracklets,
                                                       camera,
                                                       Eigen::Isometry3d::Identity(),
                                                       keyframe_bundle_adjustment::Keyframe::FixationStatus::Pose);
        bundle_adjuster_->push(cur_frame);
        ROS_DEBUG_STREAM("In MonoLidar: added first keyframe");

        // hack:: dump pose if filepath is non empty
        if (interface_.dump_path != "") {
            std::ofstream file;
            file.open(interface_.dump_path.c_str());
            file.precision(12);
            file << helpers::poseToString(Eigen::Matrix<double, 4, 4>::Identity());
            file.close();

            std::stringstream ss;
            ss << "/tmp/poses_keyframes.txt";
            std::ofstream file_key(ss.str().c_str());
            file_key.precision(12);
            file_key << helpers::poseToString(Eigen::Matrix<double, 4, 4>::Identity());
            file_key.close();
        }
    }
    auto start_send_tf = std::chrono::steady_clock::now();

    auto last_kf = bundle_adjuster_->getKeyframe();
    ros::Time timestamp_last_kf;
    timestamp_last_kf.fromNSec(last_kf.timestamp_);
    maybeSendPoseTf(timestamp_last_kf, last_kf.getEigenPose());

    ROS_INFO_STREAM(
        "In MonoLidar: Duration sending pose by tf="
        << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_send_tf)
               .count()
        << " ms");

    // publisherDiagnosed_->publish(new_msg);

    // The updater will take care of publishing at a throttled rate
    // When calling update, all updater callbacks (defined in setupDiagnostics) will be run
    updater_.update();

    auto duration =
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start_time);
    ROS_INFO_STREAM("In MonoLidar: time callback=" << duration.count() << " sec");
}

namespace {

std::set<int> loadSetFromYaml(std::string yaml_path, std::string field_name) {
    std::set<int> out;
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (root[field_name] && root[field_name].IsSequence()) {
        for (const auto& el : root[field_name]) {
            out.insert(el.as<int>());
        }
    } else {
        throw std::runtime_error("LabelReader: vector outlier_labels not defined.");
    }
    return out;
}
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
        keyframe_bundle_adjustment::LandmarkSelectionSchemeCheirality::create());
    // Sparsify landmarks with voxelgrid
    // get parameters
    keyframe_bundle_adjustment::LandmarkSelectionSchemeVoxel::Parameters p;
    //    p.max_num_landmarks_near = interface_.max_number_landmarks_near_bin;
    //    p.max_num_landmarks_middle = interface_.max_number_landmarks_middle_bin;
    //    p.max_num_landmarks_far = interface_.max_number_landmarks_far_bin;
    p.max_num_landmarks_near = 700;
    p.max_num_landmarks_middle = 300;
    p.max_num_landmarks_far = 200;

    p.grid_params_middle.max_x = 40;
    p.grid_params_middle.min_x = -40;
    p.grid_params_middle.max_y = 40;
    p.grid_params_middle.min_y = -40;
    p.grid_params_middle.num_bins_x = 80;
    p.grid_params_middle.num_bins_y = 80;
    p.grid_params_near = p.grid_params_middle;
    p.grid_params_near.num_bins_x = p.grid_params_middle.num_bins_x * 2;
    p.grid_params_near.num_bins_y = p.grid_params_middle.num_bins_y * 2;

    p.roi_far_xyz = std::array<double, 3>{{80, 80, 80}};
    p.roi_middle_xyz = std::array<double, 3>{{20, 20, 20}};

    bundle_adjuster_->landmark_selector_->addScheme(
        keyframe_bundle_adjustment::LandmarkSelectionSchemeVoxel::create(p));
    // Make parameters for depth assurance.
    // On the zeroth frame (newest frame) we assure 20 measurements with depth measurement.
    keyframe_bundle_adjustment::LandmarkSelectionSchemeAddDepth::Parameters p_add_depth;
    p_add_depth.num_depth_meas[0] = 20;
    bundle_adjuster_->landmark_selector_->addScheme(
        keyframe_bundle_adjustment::LandmarkSelectionSchemeAddDepth::create(p_add_depth));

    // reset robust loss paramters
    bundle_adjuster_->outlier_rejection_options_.depth_thres = interface_.robust_loss_depth_thres;
    bundle_adjuster_->outlier_rejection_options_.reprojection_thres = interface_.robust_loss_reprojection_thres;
    bundle_adjuster_->outlier_rejection_options_.depth_quantile = interface_.outlier_rejection_quantile;
    bundle_adjuster_->outlier_rejection_options_.reprojection_quantile = interface_.outlier_rejection_quantile;

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
    bundle_adjuster_->labels_["outliers"] = loadSetFromYaml(interface_.outlier_labels_yaml, "outlier_labels");
    std::stringstream ss;
    ss << "MonoLidar: outlier labels " << std::endl;
    for (const auto& el : bundle_adjuster_->labels_["outliers"]) {
        ss << " " << el;
    }
    ss << std::endl;
    ROS_DEBUG_STREAM(ss.str());

    bundle_adjuster_->labels_["shrubbery"] = loadSetFromYaml(interface_.outlier_labels_yaml, "shrubbery_labels");
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
