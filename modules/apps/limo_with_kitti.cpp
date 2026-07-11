// Copyright 2026. All rights reserved.
//
// LIMO (lidar-monocular visual odometry) on a KITTI odometry sequence, ROS-free:
// feature tracking (viso2) -> lidar depth for tracked features (mono_lidar_depth) ->
// keyframe bundle adjustment -> poses in KITTI format + rerun visualization.

#include <algorithm>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <ranges>
#include <string>
#include <vector>

#include <feature_tracking_core/tracker_libviso.h>
#include <image_preproc/gamma_corrector.h>
#include <keyframe_bundle_adjustment/bundle_adjuster_keyframes.hpp>
#include <keyframe_bundle_adjustment/keyframe_selector.hpp>
#include <keyframe_bundle_adjustment/keyframe_selection_schemes.hpp>
#include <matches_msg_types/tracklets.hpp>
#include <mono_lidar_depth/tracklet_depth_estimator.hpp>
#include <opencv2/imgproc.hpp>

#include "kitti_data_reader.h"
#include "rerun_vis.h"

namespace {

namespace kfba = keyframe_bundle_adjustment;

struct Options {
    std::filesystem::path sequence_dir;
    std::filesystem::path output_dir{"output"};
    std::string rerun_mode{"spawn"};
    int max_frames{-1};
};

void printUsage(const char* name) {
    std::cerr << "Usage: " << name << " <KITTI sequence dir> [options]\n"
              << "  --output <dir>       output directory for pose files (default: ./output)\n"
              << "  --rerun <mode>       spawn | connect[:<url>] | save:<path> | off (default: spawn)\n"
              << "  --max-frames <n>     stop after n frames (default: all)\n";
}

std::optional<Options> parseOptions(int argc, char** argv) {
    if (argc < 2) {
        return std::nullopt;
    }
    Options options;
    options.sequence_dir = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::optional<std::string> {
            return i + 1 < argc ? std::optional<std::string>(argv[++i]) : std::nullopt;
        };
        if (arg == "--output") {
            const auto value = next();
            if (!value) return std::nullopt;
            options.output_dir = *value;
        } else if (arg == "--rerun") {
            const auto value = next();
            if (!value) return std::nullopt;
            options.rerun_mode = *value;
        } else if (arg == "--max-frames") {
            const auto value = next();
            if (!value) return std::nullopt;
            options.max_frames = std::stoi(*value);
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return std::nullopt;
        }
    }
    return options;
}

/// Converts viso tracklets (deque of matches, newest first) to the pipeline's tracklet type.
/// stamps must also be ordered newest first; feature_points[i] corresponds to stamps[i].
///
/// The label of every tracklet is left at its neutral default. Once semantically segmented
/// images are available, look up each feature's label here — the rest of the pipeline
/// (depth estimation, updateLabels class weights) already forwards them.
matches_msg_types::Tracklets convertTracklets(const feature_tracking::TrackletVector& viso_tracklets,
                                              const std::deque<matches_msg_types::TimestampNSec>& stamps) {
    matches_msg_types::Tracklets out;
    out.stamps.assign(stamps.begin(), stamps.end());
    out.tracks.reserve(viso_tracklets.size());

    for (const auto& viso_tracklet : viso_tracklets) {
        matches_msg_types::Tracklet track;
        track.id = static_cast<unsigned long>(viso_tracklet.id_);
        track.age = static_cast<unsigned long>(viso_tracklet.age_);
        const size_t max_points = std::min(viso_tracklet.size(), stamps.size());
        track.feature_points.reserve(max_points);
        for (size_t i = 0; i < max_points; ++i) {
            const auto& match = viso_tracklet[i];
            track.feature_points.emplace_back(match.p1_.u_, match.p1_.v_);
        }
        out.tracks.push_back(std::move(track));
    }
    return out;
}

/// Writes poses (pose of frame i in the frame of frame 0) in KITTI odometry format:
/// one row per frame with the upper 3x4 of the pose matrix, row-major.
void writeKittiPoses(const std::filesystem::path& file_path, const std::vector<Eigen::Isometry3d>& poses) {
    std::ofstream out(file_path);
    if (!out) {
        throw std::runtime_error("Cannot write " + file_path.string());
    }
    out << std::setprecision(12);
    for (const auto& pose : poses) {
        const auto& m = pose.matrix();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 4; ++col) {
                out << m(row, col) << (row == 2 && col == 3 ? "" : " ");
            }
        }
        out << "\n";
    }
}

/// Landmarks of the current optimization window with a color per kind: depth-measured landmarks
/// orange, ground-plane landmarks green, triangulated-only landmarks cyan.
void logLandmarks(const rr_vis::RerunVisualizer& vis, const kfba::BundleAdjusterKeyframes& ba) {
    std::vector<Eigen::Vector3d> positions;
    std::vector<std::array<uint8_t, 4>> colors;
    for (const auto& [id, landmark] : ba.getActiveLandmarkConstPtrs()) {
        positions.emplace_back(landmark->pos[0], landmark->pos[1], landmark->pos[2]);
        if (landmark->is_ground_plane) {
            colors.push_back({80, 220, 100, 255});
        } else if (landmark->has_measured_depth) {
            colors.push_back({255, 160, 40, 255});
        } else {
            colors.push_back({70, 190, 255, 255});
        }
    }
    vis.logPoints("world/landmarks", positions, colors, 0.12f);
}

} // namespace

int main(int argc, char** argv) {
    const auto options = parseOptions(argc, argv);
    if (!options.has_value()) {
        printUsage(argv[0]);
        return EXIT_FAILURE;
    }

    KittiDataReader reader(options->sequence_dir.string());
    const Eigen::Matrix3d K = reader.intrinsics();
    const double focal_length = K(0, 0);
    const Eigen::Vector2d principal_point(K(0, 2), K(1, 2));

    rr_vis::RerunVisualizer vis("limo_with_kitti", options->rerun_mode);

    // Feature tracking on grayscale images, gamma-corrected for stability.
    feature_tracking::TrackerLibViso tracker;
    image_preproc::GammaCorrector gamma_corrector;

    // The bundle adjustment treats the camera frame as vehicle frame (identity extrinsics).
    auto camera_ba =
        std::make_shared<kfba::Camera>(focal_length, principal_point, Eigen::Isometry3d::Identity());

    kfba::BundleAdjusterKeyframes bundle_adjuster;
    bundle_adjuster.set_solver_time(0.2);
    // Cap the landmark count per solve (voxel grid) but always keep the best depth-measured
    // landmarks — they anchor the scale.
    bundle_adjuster.landmark_selector_->addScheme(kfba::LandmarkSparsificationSchemeVoxel::createConst({}));
    {
        kfba::LandmarkSelectionSchemeAddDepth::Parameters depth_params;
        depth_params.params_per_keyframe.emplace_back(
            0,
            20,
            [](const kfba::Landmark::ConstPtr& lm) { return lm->has_measured_depth; },
            [](const kfba::Measurement& m, const Eigen::Vector3d&) { return static_cast<double>(m.d); });
        bundle_adjuster.landmark_selector_->addScheme(kfba::LandmarkSelectionSchemeAddDepth::createConst(depth_params));
    }

    // Accept every frame with sufficient feature flow as keyframe; frames without flow (e.g. the
    // car is standing) are rejected. The selector output is the union of the sparsification and
    // selection schemes minus rejections, so an accept-everything sparsification scheme is needed.
    kfba::KeyframeSelector keyframe_selector;
    keyframe_selector.addScheme(
        kfba::KeyframeRejectionSchemeBase::ConstPtr(std::make_shared<kfba::KeyframeRejectionSchemeFlow>(3.0)));
    keyframe_selector.addScheme(
        kfba::KeyframeSparsificationSchemeBase::ConstPtr(std::make_shared<kfba::KeyframeSparsificationSchemeTime>(0.)));

    std::optional<mono_lidar::TrackletDepthEstimator> depth_estimator;

    std::deque<matches_msg_types::TimestampNSec> stamps;
    std::vector<Eigen::Isometry3d> frame_poses;          // pose of frame i in frame 0 (camera)
    std::vector<uint64_t> keyframe_stamps_per_frame;     // the keyframe stamp if frame i is one
    std::vector<bool> is_keyframe;
    Eigen::Isometry3d last_motion = Eigen::Isometry3d::Identity(); // frame-to-frame motion (world)
    bool bootstrapped = false;

    size_t processed = 0;
    for (auto gen = reader.frames(); gen.next();) {
        KittiFrame& frame = gen.value();
        if (options->max_frames >= 0 && static_cast<int>(processed) >= options->max_frames) {
            break;
        }

        if (!depth_estimator.has_value()) {
            mono_lidar::DepthEstimatorParameters depth_parameters;
            auto camera_ml = std::make_shared<mono_lidar::CameraPinhole>(
                frame.image.cols, frame.image.rows, focal_length, principal_point.x(), principal_point.y());
            depth_estimator.emplace(depth_parameters, camera_ml, frame.transform_velodyne_to_camera);
        }

        // 1) Track features on the gamma-corrected grayscale image.
        cv::Mat gray;
        cv::cvtColor(frame.image, gray, cv::COLOR_BGR2GRAY);
        cv::Mat gray_corrected;
        gamma_corrector.processImage(gray, gray_corrected);
        tracker.pushBack(gray_corrected);

        feature_tracking::TrackletVector viso_tracklets;
        tracker.getTracklets(viso_tracklets, 0);

        stamps.push_front(frame.timestamp_ns);
        constexpr size_t kMaxTrackletHistory = 30;
        if (stamps.size() > kMaxTrackletHistory) {
            stamps.pop_back();
        }
        auto tracklets = convertTracklets(viso_tracklets, stamps);

        // 2) Lidar depth for the tracked features.
        tracklets = depth_estimator->process(frame.cloud, tracklets);

        // 3) Pose prior by constant velocity, then keyframe selection + bundle adjustment.
        // Identical consecutive pose initializations sit on a singular point of the bundle
        // adjuster's scale regularization (d||t1-t0|| at t1==t0), so nudge a degenerate motion
        // prior slightly forward.
        Eigen::Isometry3d motion_prior = last_motion;
        if (motion_prior.translation().norm() < 0.01) {
            motion_prior.translation() = Eigen::Vector3d(0., 0., 0.25);
        }
        const Eigen::Isometry3d pose_prior =
            frame_poses.empty() ? Eigen::Isometry3d::Identity() : frame_poses.back() * motion_prior;

        Eigen::Isometry3d pose_current = pose_prior;
        bool selected_as_keyframe = false;

        const bool has_tracks = std::ranges::any_of(
            tracklets.tracks, [](const auto& track) { return track.feature_points.size() >= 2; });

        if (!bootstrapped && !has_tracks) {
            // The tracker needs two images before it yields matches; until then there is nothing
            // to estimate.
        } else if (!bootstrapped) {
            // Bootstrap: the tracklets contain observations (and lidar depths) for the previous
            // and the current stamp. Push a fixed keyframe at the previous stamp — it anchors the
            // world in the first frame — plus the current frame, and let the solver estimate the
            // first motion from the depth-anchored landmarks.
            bundle_adjuster.push(kfba::Keyframe(stamps.at(1), tracklets, camera_ba, Eigen::Isometry3d::Identity(),
                                                kfba::Keyframe::FixationStatus::Pose));
            bundle_adjuster.push(kfba::Keyframe(frame.timestamp_ns, tracklets, camera_ba, pose_prior.inverse()));
            bundle_adjuster.updateLabels(tracklets, /*shrubbery_weight=*/1.0);
            // solve() requires three keyframes; the first solve happens with the next one.
            selected_as_keyframe = true;
            bootstrapped = true;
        } else {
            auto candidate = std::make_shared<kfba::Keyframe>(
                frame.timestamp_ns, tracklets, camera_ba, pose_prior.inverse());
            const auto selected = keyframe_selector.select({candidate}, bundle_adjuster.getActiveKeyframePtrs());
            selected_as_keyframe = !selected.empty();

            if (selected_as_keyframe) {
                bundle_adjuster.push(*candidate);
                // Uniform class weights for now; per-class weights return with the semantic input.
                bundle_adjuster.updateLabels(tracklets, /*shrubbery_weight=*/1.0);
                bundle_adjuster.deactivateKeyframes();
                const std::string summary = bundle_adjuster.solve();
                pose_current = bundle_adjuster.getKeyframe().getEigenPose().inverse();
            } else {
                // No parallax (car standing): keep the previous pose.
                pose_current = frame_poses.back();
            }
        }

        if (!frame_poses.empty()) {
            last_motion = frame_poses.back().inverse() * pose_current;
        }
        frame_poses.push_back(pose_current);
        is_keyframe.push_back(selected_as_keyframe);
        keyframe_stamps_per_frame.push_back(frame.timestamp_ns);

        // 4) Visualization.
        vis.setFrame(static_cast<int64_t>(frame.index), frame.timestamp_ns);
        vis.logPose("world/camera", pose_current);
        vis.logCamera("world/camera/image", K, frame.image.cols, frame.image.rows);
        vis.logImage("world/camera/image", frame.image);
        vis.logPointCloud("world/camera/lidar", *frame.cloud, frame.transform_velodyne_to_camera);
        if (selected_as_keyframe) {
            logLandmarks(vis, bundle_adjuster);
            std::vector<Eigen::Vector3d> trajectory;
            trajectory.reserve(frame_poses.size());
            for (const auto& pose : frame_poses) {
                trajectory.push_back(pose.translation());
            }
            vis.logTrajectory("world/trajectory", trajectory);
        }

        if (selected_as_keyframe && depth_estimator.has_value()) {
            std::cout << depth_estimator->depthEstimator().statistics();
        }
        std::cout << "Frame " << frame.index << (selected_as_keyframe ? " [keyframe]" : "")
                  << ": tracks=" << tracklets.tracks.size() << ", pose t=("
                  << pose_current.translation().transpose() << ")" << std::endl;
        ++processed;
    }

    // 5) Final pose output: keyframes take their final optimized pose, frames in between are
    // propagated with the relative motions estimated online.
    std::vector<Eigen::Isometry3d> final_poses;
    final_poses.reserve(frame_poses.size());
    for (size_t i = 0; i < frame_poses.size(); ++i) {
        std::optional<Eigen::Isometry3d> optimized;
        if (is_keyframe[i]) {
            const auto it = bundle_adjuster.keyframes_.find(keyframe_stamps_per_frame[i]);
            if (it != bundle_adjuster.keyframes_.end()) {
                optimized = it->second->getEigenPose().inverse();
            }
        }
        if (optimized.has_value()) {
            final_poses.push_back(*optimized);
        } else if (final_poses.empty()) {
            final_poses.push_back(frame_poses[i]);
        } else {
            const Eigen::Isometry3d relative_motion = frame_poses[i - 1].inverse() * frame_poses[i];
            final_poses.push_back(final_poses.back() * relative_motion);
        }
    }

    std::filesystem::create_directories(options->output_dir);
    writeKittiPoses(options->output_dir / "poses.txt", final_poses);

    std::vector<Eigen::Isometry3d> keyframe_poses;
    for (size_t i = 0; i < final_poses.size(); ++i) {
        if (is_keyframe[i]) {
            keyframe_poses.push_back(final_poses[i]);
        }
    }
    writeKittiPoses(options->output_dir / "poses_keyframes.txt", keyframe_poses);

    std::cout << "Wrote " << final_poses.size() << " poses (" << keyframe_poses.size() << " keyframes) to "
              << options->output_dir << std::endl;
    return EXIT_SUCCESS;
}
