#include <boost/cstdint.hpp>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

// #define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <numpy/arrayobject.h>

#include <deque>
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>

#include <keyframe_bundle_adjustment/bundle_adjuster_keyframes.hpp>
#include <keyframe_bundle_adjustment/keyframe_selector.hpp>

namespace p = boost::python;
namespace np = p::numpy;

struct CameraData {
  double focal_length;
  double cx;
  double cy;
  std::vector<double> transform_camera_vehicle;
};

struct Config {
  double height_over_ground;
  double time_between_keyframes_sec;
  double shrubbery_weight;
  double min_median_flow;
  double critical_rotation_difference;
  double max_number_landmarks_near_bin;
  double max_number_landmarks_middle_bin;
  double max_number_landmarks_far_bin;
  double robust_loss_reprojection_thres;
  double outlier_rejection_quantile;
  double outlier_rejection_num_iterations;
  double max_solver_time;
  double outlier_labels_yaml;
};

double to_nano_sec(double ts_sec){
  return ts_sec*1e9;
}

void executeMonoBundleAdjustment(
    keyframe_bundle_adjustment::BundleAdjusterKeyframes &bundle_adjuster_,
    const keyframe_bundle_adjustment::Tracklets &tracklets,
    keyframe_bundle_adjustment::KeyframeSelector &keyframe_selector_,
    const CameraData &camera_data, double cur_timestamp_sec,  
    const Config &interface_) {
  // initiate camera
  Eigen::Matrix4d m(camera_data.transform_camera_vehicle.data());
  Eigen::Isometry3d transform_camera_vehicle(m);
  keyframe_bundle_adjustment::Camera::Ptr camera =
      std::make_shared<keyframe_bundle_adjustment::Camera>(
          camera_data.focal_length, Eigen::Vector2d(
              camera_data.cx, camera_data.cy),
          transform_camera_vehicle);

  if (bundle_adjuster_.keyframes_.size() > 0) {
    // in mono case second pose must be fixed in scale
    keyframe_bundle_adjustment::Keyframe::FixationStatus fixation_status{
        keyframe_bundle_adjustment::Keyframe::FixationStatus::None};

    // get motion corresponding to last keyframe
    auto start_time_5point = std::chrono::steady_clock::now();
    Eigen::Isometry3d motion_prior_vehicle_t1_t0 = helpers::getMotionUnscaled(
        camera_data.focal_length, cv::Point2d(camera_data.cx, camera_data.cy),
        to_nano_sec(cur_timestamp_sec),
        bundle_adjuster_.getKeyframe().timestamp_, tracklets,
        transform_camera_vehicle);
    std::cout << "Duration 5 point="
       << std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - start_time_5point)
              .count()
       << " ms" << std::endl;

    Eigen::Isometry3d pose_prior_keyframe_origin =
        motion_prior_vehicle_t1_t0 *
        bundle_adjuster_.getKeyframe().getEigenPose();

    // select keyframe
    auto start_time_adjust_pose = std::chrono::steady_clock::now();
    keyframe_bundle_adjustment::Plane ground_plane;
    ground_plane.distance = interface_.height_over_ground;
    auto cur_frame = std::make_shared<keyframe_bundle_adjustment::Keyframe>(
        to_nano_sec(cur_timestamp_sec), tracklets, camera, pose_prior_keyframe_origin,
        fixation_status, ground_plane);

    std::string summary_motion_only =
        bundle_adjuster_.adjustPoseOnly(*cur_frame);
    std::cout << "Duration pose only="
       << std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - start_time_adjust_pose)
              .count()
       << " ms" << std::endl;
    //        std::cout << "---------------------------- motion only
    //        ------------------------" << std::endl; std::cout <<
    //        summary_motion_only << std::endl;

    auto start_time_select_kf = std::chrono::steady_clock::now();
    std::set<keyframe_bundle_adjustment::Keyframe::Ptr> selected_frames =
        keyframe_selector_.select({cur_frame},
                                  bundle_adjuster_.getActiveKeyframePtrs());
    assert(selected_frames.size() < 2);
    int64_t duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time_select_kf)
            .count();
    std::cout << "Duration select frames=" << duration << " ms\n";

    auto start_time_push = std::chrono::steady_clock::now();
    // add keyframes to bundle adjustment
    for (const auto &kf : selected_frames) {
      bundle_adjuster_.push(*kf);
    }
    std::cout << "Duration push="
       << std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - start_time_push)
              .count()
       << " ms" << std::endl;
    std::cout << "number keyframes " << bundle_adjuster_.keyframes_.size()
              << "\nnumber selected keyframes " << selected_frames.size()
              << "\nnumber active keyframes "
              << bundle_adjuster_.active_keyframe_ids_.size()
              << "\nnumber active landmarks "
              << bundle_adjuster_.active_landmark_ids_.size()
              << "\nnumber selected landmarks "
              << bundle_adjuster_.selected_landmark_ids_.size() << std::endl;

    // do bundle adjustment
    if (selected_frames.size() > 0 && bundle_adjuster_.keyframes_.size() > 2 &&
        cur_timestamp_sec - bundle_adjuster_.getLastSolveTimestamp() >
            interface_.time_between_keyframes_sec) {

      auto start_time_update_deactivate = std::chrono::steady_clock::now();
      // deactivate keyframes that are not connected to the current frame
      bundle_adjuster_.deactivateKeyframes();
      bundle_adjuster_.updateLabels(tracklets, interface_.shrubbery_weight);
      std::cout << "Duration update,deactivate="
         << std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time_update_deactivate)
                .count()
         << " ms" << std::endl;

      // do optimization
      auto start_time_solve = std::chrono::steady_clock::now();
      std::string summary = bundle_adjuster_.solve();
      //            for (const auto& el : bundle_adjuster_->keyframes_) {
      //                auto fix = el.second->fixation_status_;
      //                if (fix ==
      //                keyframe_bundle_adjustment::Keyframe::FixationStatus::None)
      //                {
      //                    std::cout << "none" << std::endl;
      //                } else if (fix ==
      //                keyframe_bundle_adjustment::Keyframe::FixationStatus::Scale)
      //                {
      //                    std::cout << "scale" << std::endl;
      //                } else if (fix ==
      //                keyframe_bundle_adjustment::Keyframe::FixationStatus::Pose)
      //                {
      //                    std::cout << "pose" << std::endl;
      //                } else {
      //                    std::cout << "sth wrong" << std::endl;
      //                }
      //            }
      /// @TODO put this in solve(
      bundle_adjuster_.setLastSolveTimestamp(cur_timestamp_sec);
      std::cout << "In MonoStandalone: Duration solve="
         << std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time_solve)
                .count()
         << " ms\n";
    }
  } else {
    // first received frame is always a keyframe and has fixed pose
    double ts_oldest_feature = cur_timestamp_sec;
    keyframe_bundle_adjustment::Plane ground_plane;
    ground_plane.distance = interface_.height_over_ground;
    keyframe_bundle_adjustment::Keyframe cur_frame(
        to_nano_sec(ts_oldest_feature), tracklets, camera,
        Eigen::Isometry3d::Identity(),
        keyframe_bundle_adjustment::Keyframe::FixationStatus::Pose,
        ground_plane);
    bundle_adjuster_.push(cur_frame);
    std::cout << "added first keyframe" << std::endl;
  }
}

void initBundleAdjuster(
    keyframe_bundle_adjustment::BundleAdjusterKeyframes &bundle_adjuster_,
    const Config &interface_) {
  bundle_adjuster_.landmark_selector_ =
      std::make_shared<keyframe_bundle_adjustment::LandmarkSelector>();
  // first is always cheirality since it is fast and reliable
  bundle_adjuster_.landmark_selector_->addScheme(
      keyframe_bundle_adjustment::LandmarkRejectionSchemeCheirality::create());
  // add custom lm selection
  //    keyframe_bundle_adjustment::LandmarkSparsificationSchemeObservability::Parameters
  //    p; p.bin_params_.max_num_landmarks_near =
  //    interface_.max_number_landmarks_near_bin;
  //    p.bin_params_.max_num_landmarks_middle =
  //    interface_.max_number_landmarks_middle_bin;
  //    p.bin_params_.max_num_landmarks_far =
  //    interface_.max_number_landmarks_far_bin;
  //    bundle_adjuster_.landmark_selector_->addScheme(
  //        keyframe_bundle_adjustment::LandmarkSparsificationSchemeObservability::create(p));
  keyframe_bundle_adjustment::LandmarkSparsificationSchemeVoxel::Parameters p;
  p.max_num_landmarks_near = interface_.max_number_landmarks_near_bin;
  p.max_num_landmarks_middle = interface_.max_number_landmarks_middle_bin;
  p.max_num_landmarks_far = interface_.max_number_landmarks_far_bin;
  p.roi_middle_xyz = std::array<double, 3>{10., 10., 10.};

  bundle_adjuster_.landmark_selector_->addScheme(
      keyframe_bundle_adjustment::LandmarkSparsificationSchemeVoxel::create(p));

  keyframe_bundle_adjustment::LandmarkSelectionSchemeAddDepth::Parameters
      p_add_depth;
  auto gp_comparator =
      [](const keyframe_bundle_adjustment::Landmark::ConstPtr &lm) {
        return lm->is_ground_plane;
      };
  auto gp_sorter = [](const keyframe_bundle_adjustment::Measurement &m,
                      const Eigen::Vector3d &local_lm) {
    return local_lm.norm();
  };
  p_add_depth.params_per_keyframe.clear();
  for (int i = 0; i < 30; i = i + 2) {
    p_add_depth.params_per_keyframe.push_back(
        std::make_tuple(i, 50, gp_comparator, gp_sorter));
  }

  bundle_adjuster_.landmark_selector_->addScheme(
      keyframe_bundle_adjustment::LandmarkSelectionSchemeAddDepth::create(
          p_add_depth));

  bundle_adjuster_.outlier_rejection_options_.reprojection_thres =
      interface_.robust_loss_reprojection_thres;
  bundle_adjuster_.outlier_rejection_options_.reprojection_quantile =
      interface_.outlier_rejection_quantile;
  bundle_adjuster_.outlier_rejection_options_.num_iterations =
      interface_.outlier_rejection_num_iterations;

  // reset solver time
  bundle_adjuster_.set_solver_time(interface_.max_solver_time);

  // Read outlier ids and set them in bundle_adjuster
  bundle_adjuster_.labels_["outliers"] =
      keyframe_bundle_adjustment_ros_tool::helpers::loadSetFromYaml(
          interface_.outlier_labels_yaml, "outlier_labels");

  bundle_adjuster_.labels_["shrubbery"] =
      keyframe_bundle_adjustment_ros_tool::helpers::loadSetFromYaml(
          interface_.outlier_labels_yaml, "shrubbery_labels");
}

void initKeyframeSelector(
    keyframe_bundle_adjustment::KeyframeSelector &keyframe_selector_,
    const Config &interface_) {
  // reset keyframe selector
  keyframe_selector_.addScheme(
      keyframe_bundle_adjustment::KeyframeRejectionSchemeFlow::create(
          interface_.min_median_flow));
  keyframe_selector_.addScheme(
      keyframe_bundle_adjustment::KeyframeSelectionSchemePose::create(
          interface_.critical_rotation_difference));
  keyframe_selector_.addScheme(
      keyframe_bundle_adjustment::KeyframeSparsificationSchemeTime::create(
          interface_.time_between_keyframes_sec));
}

BOOST_PYTHON_MODULE(keyframe_bundle_adjustment_mono) {
  np::initialize(); // have to put this in any module that uses Boost.NumPy

  namespace kfba = keyframe_bundle_adjustment;
  using Transform = np::ndarray;

  // How to bind an enum class?
  p::class_<kfba::Keyframe>("Keyframe", p::init<>())
      .def(p::init<uint64_t, kfba::Tracklets, kfba::Camera::Ptr, Eigen::Isometry3d, kfba::Keyframe::FixationStatus, kfba::Plane>())
      .def_readwrite("FixationStatus", &kfba::Keyframe::fixation_status_);

  p::class_<keyframe_bundle_adjustment::Tracklets>("Tracklets", p::init<>());

  using Keyframes = std::vector<kfba::Keyframe>;
  p::class_<kfba::BundleAdjusterKeyframes>("BundleAdjusterKeyframes", p::init<>());
//.def_readwrite("keyframes", p::vector_indexing_suite<Keyframes>());

  p::class_<kfba::KeyframeSelector>("KeyframeSelector", p::init<>());

  p::def("init_bundle_adjuster", &initBundleAdjuster);
  p::def("init_keyframe_selector", &initKeyframeSelector);
  p::def("execute_mono_bundle_adjustment", &executeMonoBundleAdjustment);
}
