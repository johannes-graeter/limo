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

#include <commons/color_by_index_hsv.hpp>
#include <commons/general_helpers.hpp>
#include <commons/publish_helpers.hpp>

namespace keyframe_bundle_adjustment_ros_tool {

namespace {

/**
 * @brief calcMotion5Point, calc motion with 5 point algo from opencv
 * @param motion, motino beetween frames will be changed if successful
 * @param points0, points from first frame
 * @param points1, points from second frame
 * @param focal, focal length
 * @param pp, principal point
 * @return flag if sucessfull==true
 */
bool calcMotion5Point(Eigen::Isometry3d& motion,
                      const CvPoints& points0,
                      const CvPoints& points1,
                      double focal,
                      cv::Point2d pp,
                      double probability) {

    if (points0.size() == 0 || points1.size() == 0) {
        ROS_DEBUG_STREAM("no points for 5 point available!");
        return false;
    }

    cv::Mat essential_mat =
        cv::findEssentialMat(points1, points0, focal, pp, cv::RANSAC, probability, 0.2);
    if (essential_mat.rows != 3 || essential_mat.cols != 3) {
        return false;
    }


    cv::Mat R, t;
    int number_inliers = cv::recoverPose(essential_mat, points1, points0, R, t, focal, pp);

    Eigen::Matrix3d r_eigen;
    Eigen::Vector3d t_eigen;
    cv::cv2eigen(R, r_eigen);
    cv::cv2eigen(t, t_eigen);

    motion.linear() = r_eigen;
    motion.translation() = t_eigen;

    return true;
}
}


MonoStandalone::MonoStandalone(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : interface_{nh_private}, reconfigure_server_{nh_private}, tf_listener_{tf_buffer_},
          motion_prior_camera_t0_t1(Eigen::Isometry3d::Identity()) {

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
        sync_ = std::make_unique<Synchronizer>(ApproximateTime(100),
                                               *(interface_.tracklets_subscriber),
                                               *(interface_.camera_info_subscriber));
        sync_->registerCallback(boost::bind(&MonoStandalone::callbackSubscriber, this, _1, _2));
    }
    // setup of selectors and bundle adjuster is done in reconfigureCallback

    // get extrinsics from tf, this is the pose from vehicle to camera frame!
    try {
        Eigen::Affine3d ext_aff =
            tf2::transformToEigen(tf_buffer_.lookupTransform(interface_.calib_target_frame_id,
                                                             interface_.calib_source_frame_id,
                                                             ros::Time::now(),
                                                             ros::Duration(5.)));
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

    // get camera calib -> with get Cameras?
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(camera_info_msg);
    assert(model.fx() == model.fy()); // we only support undistorted images

    if (tracklets_msg->stamps.size() == 0) {
        // tracks come only if one or more images have benn seen
        // like we loose the info from the very first image, avoiding it is too much trouble
        ROS_DEBUG_STREAM("no tracklet data yet, return");
        return;
    }

    keyframe_bundle_adjustment::TimestampNSec cur_ts = tracklets_msg->stamps.front().toNSec();
    keyframe_bundle_adjustment::Tracklets tracklets =
        matches_msg_conversions_ros::Convert(tracklets_msg);

    ROS_DEBUG_STREAM("num tracklets=" << tracklets.tracks.size());

    // initiate camera
    keyframe_bundle_adjustment::Camera::Ptr camera =
        std::make_shared<keyframe_bundle_adjustment::Camera>(
            model.fx(), Eigen::Vector2d(model.cx(), model.cy()), trf_camera_vehicle);

    if (bundle_adjuster_->keyframes_.size() > 0) {
        // in mono case second pose must be fixed in scale
        keyframe_bundle_adjustment::Keyframe::FixationStatus fixation_status{
            keyframe_bundle_adjustment::Keyframe::FixationStatus::None};
        if (bundle_adjuster_->keyframes_.size() == 1) {
            fixation_status = keyframe_bundle_adjustment::Keyframe::FixationStatus::Scale;
        }

        // get motion corresponding to last keyframe
        auto last_kf = bundle_adjuster_->getKeyframe();
        keyframe_bundle_adjustment::TimestampNSec last_ts = last_kf.timestamp_;

        CvPoints last_points, cur_points;
        std::tie(last_points, cur_points) = helpers::getMatches(tracklets, last_ts, cur_ts);
        //        for (const auto& t : tracklets.tracks) {
        //            if (t.feature_points.size() > 1) {
        //                const auto& cur_m = *t.feature_points.cbegin();
        //                const auto& last_m = *(t.feature_points.cbegin() + 1);

        //                cur_points.push_back(cv::Point(cur_m.u, cur_m.v));
        //                last_points.push_back(cv::Point(last_m.u, last_m.v));
        //            }
        //        }

        // get motion in camera frame from keyframe to current instant

        auto start_time_5point = std::chrono::steady_clock::now();
        calcMotion5Point(motion_prior_camera_t0_t1,
                         last_points,
                         cur_points,
                         model.fx(),
                         cv::Point2d(model.cx(), model.cy()),
                         interface_.motion_prior_ransac_probability);

        ROS_INFO_STREAM("time 5 point="
                        << std::chrono::duration_cast<std::chrono::duration<double>>(
                               std::chrono::steady_clock::now() - start_time_5point)
                               .count()
                        << " sec");
        //        ROS_DEBUG_STREAM("motion from 5 point=\n" << motion_prior_camera_t0_t1.matrix());

        // transform it with known poses to pose of vehicle
        Eigen::Isometry3d motion_prior_vehicle_t1_t0 =
            trf_camera_vehicle.inverse() * motion_prior_camera_t0_t1.inverse() * trf_camera_vehicle;
        Eigen::Isometry3d pose_prior_vehicle_t1_orig =
            motion_prior_vehicle_t1_t0 * last_kf.getEigenPose();

        // select keyframe
        auto start_time_select_kf = std::chrono::steady_clock::now();
        auto cur_frame = std::make_shared<keyframe_bundle_adjustment::Keyframe>(
            cur_ts, tracklets, camera, pose_prior_vehicle_t1_orig, fixation_status);

        std::set<keyframe_bundle_adjustment::Keyframe::Ptr> selected_frames =
            keyframe_selector_.select({cur_frame}, bundle_adjuster_->getActiveKeyframePtrs());
        assert(selected_frames.size() < 2);
        int64_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - start_time_select_kf)
                               .count();
        ROS_INFO_STREAM("duration select frames=" << duration << " ms");

        // add keyframes to bundle adjustment
        for (const auto& kf : selected_frames) {
            bundle_adjuster_->push(*kf);
        }
        ROS_DEBUG_STREAM("number keyframes " << bundle_adjuster_->keyframes_.size());
        ROS_INFO_STREAM("number selected keyframes " << selected_frames.size());
        ROS_INFO_STREAM("number active keyframes "
                        << bundle_adjuster_->active_keyframe_ids_.size());
        ROS_INFO_STREAM("number active landmarks "
                        << bundle_adjuster_->active_landmark_ids_.size());
        ROS_INFO_STREAM("number selected landmarks "
                        << bundle_adjuster_->selected_landmark_ids_.size());

        // do bundle adjustment
        if (selected_frames.size() > 0 && bundle_adjuster_->keyframes_.size() > 2) {

            // deactivate keyframes that are not connected to the current frame
            auto start_deactivate = std::chrono::steady_clock::now();
            bundle_adjuster_->deactivateKeyframes();
            ROS_INFO_STREAM("Duration deactivate kf="
                            << std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now() - start_deactivate)
                                   .count()
                            << " ms");

            // do optimization
            auto start_time_solve = std::chrono::steady_clock::now();
            std::string summary = bundle_adjuster_->solve();
            ROS_INFO_STREAM(summary);
            ROS_INFO_STREAM("Duration solve="
                            << std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now() - start_time_solve)
                                   .count()
                            << " ms");

            // debug
            if (interface_.verbosity == "debug") {
                auto start_time_debug_img = std::chrono::steady_clock::now();

                std::stringstream ss;
                for (const auto& kf : bundle_adjuster_->keyframes_) {

                    ss << "id " << kf.first << ": "
                       << kf.second->getEigenPose().translation().transpose();

                    if (kf.second->is_active_) {
                        ss << " active";
                    } else {
                        ss << " inactive";
                    }

                    if (kf.second->fixation_status_ ==
                        keyframe_bundle_adjustment::Keyframe::FixationStatus::Pose) {
                        ss << " pose";
                    } else if (kf.second->fixation_status_ ==
                               keyframe_bundle_adjustment::Keyframe::FixationStatus::Scale) {
                        ss << " scale";
                    } else if (kf.second->fixation_status_ ==
                               keyframe_bundle_adjustment::Keyframe::FixationStatus::None) {
                        ss << " none";
                    }
                    ss << "\n";
                }
                ROS_DEBUG_STREAM(ss.str());
                ROS_DEBUG_STREAM("Duration plot debug="
                                 << std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now() - start_time_debug_img)
                                        .count()
                                 << " ms");
            }
            if (interface_.show_debug_image) {
                cv::imshow("flow debug image keyframes", helpers::getFlowImg(bundle_adjuster_));
                cv::waitKey(30);
            }

            if (interface_.path_publisher_topic != "" &&
                interface_.active_path_publisher_topic != "") {
                helpers::publishPaths(interface_.path_publisher,
                                      interface_.active_path_publisher,
                                      bundle_adjuster_,
                                      interface_.tf_parent_frame_id);
            }

            if (interface_.landmarks_publisher_topic != "") {
                helpers::publishLandmarks(interface_.landmarks_publisher,
                                          bundle_adjuster_,
                                          interface_.tf_parent_frame_id);
            }
        }
    } else {
        // first received frame is always a keyframe and has fixed pose
        keyframe_bundle_adjustment::Keyframe cur_frame(
            cur_ts,
            tracklets,
            camera,
            Eigen::Isometry3d::Identity(),
            keyframe_bundle_adjustment::Keyframe::FixationStatus::Pose);
        bundle_adjuster_->push(cur_frame);
        ROS_DEBUG_STREAM("added first keyframe");
    }

    auto last_kf = bundle_adjuster_->getKeyframe();
    ros::Time cur_timestamp;
    cur_timestamp.fromNSec(last_kf.timestamp_);
    maybeSendPoseTf(cur_timestamp, last_kf.getEigenPose());

    // The updater will take care of publishing at a throttled rate
    // When calling update, all updater callbacks (defined in setupDiagnostics) will be run
    updater_.update();

    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::steady_clock::now() - start_time);
    ROS_INFO_STREAM("time callback=" << duration.count() << " sec");
}


/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure
 * window
*/
void MonoStandalone::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);

    // lm selection scheme without resetting ba
    keyframe_bundle_adjustment::LandmarkSelectionSchemeObservability::Parameters p;
    p.bin_params_.max_num_landmarks_near = interface_.max_number_landmarks_near_bin;
    p.bin_params_.max_num_landmarks_middle = interface_.max_number_landmarks_middle_bin;
    p.bin_params_.max_num_landmarks_far = interface_.max_number_landmarks_far_bin;
    //    bundle_adjuster_ = std::make_shared<keyframe_bundle_adjustment::BundleAdjusterKeyframes>(
    //        keyframe_bundle_adjustment::LandmarkSelectionSchemeObservability::create(p));
    bundle_adjuster_->landmark_selector_ =
        std::make_unique<keyframe_bundle_adjustment::LandmarkSelector>();
    // first is always cheirality since it is fast and reliable
    bundle_adjuster_->landmark_selector_->addScheme(
        keyframe_bundle_adjustment::LandmarkSelectionSchemeCheirality::create());
    // add custom lm selection
    bundle_adjuster_->landmark_selector_->addScheme(
        keyframe_bundle_adjustment::LandmarkSelectionSchemeObservability::create(p));

    // reset solver time
    bundle_adjuster_->set_solver_time(interface_.max_solver_time);

    // reset keyframe selector
    keyframe_selector_ = keyframe_bundle_adjustment::KeyframeSelector();
    keyframe_selector_.addScheme(keyframe_bundle_adjustment::KeyframeRejectionSchemeFlow::create(
        interface_.min_median_flow));
    keyframe_selector_.addScheme(keyframe_bundle_adjustment::KeyframeSelectionSchemePose::create(
        interface_.critical_rotation_difference));
    keyframe_selector_.addScheme(
        keyframe_bundle_adjustment::KeyframeSparsificationSchemeTime::create(
            interface_.time_between_keyframes_sec));

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

void MonoStandalone::checkSensorStatus(
    diagnostic_updater::DiagnosticStatusWrapper& status_wrapper) {
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
