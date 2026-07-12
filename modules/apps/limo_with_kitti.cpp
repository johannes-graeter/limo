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
    std::optional<std::filesystem::path> semantic_dir;
    double shrubbery_weight{0.5};
    bool mono{false};
    double camera_height{1.65};
    bool semantic_gp{true};
};

void printUsage(const char* name) {
    std::cerr
        << "Usage: " << name << " <KITTI sequence dir> [options]\n"
        << "  --output <dir>            output directory for pose files (default: ./output)\n"
        << "  --rerun <mode>            spawn | connect[:<url>] | save:<path> | off (default: spawn)\n"
        << "  --max-frames <n>          stop after n frames (default: all)\n"
        << "  --semantic <dir>          directory with semantically segmented images (000000.png ...,\n"
        << "                            8-bit cityscapes label ids). Enables label-based landmark\n"
        << "                            weighting and the semantic ground plane. Optional.\n"
        << "  --shrubbery-weight <w>    weight for vegetation landmarks with --semantic (default: 0.5)\n"
        << "  --mono                    camera-only mode, no lidar (requires --semantic): the metric\n"
        << "                            scale comes from the camera height over the semantically\n"
        << "                            detected ground plane instead of lidar depth\n"
        << "  --camera-height <m>       camera height over ground (plausibility gate for the ground\n"
        << "                            plane; the scale source in --mono; default: 1.65)\n"
        << "  --semantic-gp <on|off>    ground-plane residuals/regularization with --semantic\n"
        << "                            (default: on; labels still weight landmarks when off)\n";
}

std::optional<Options> parseOptions(int argc, char** argv) {
    if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
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
        } else if (arg == "--semantic") {
            const auto value = next();
            if (!value) return std::nullopt;
            options.semantic_dir = *value;
        } else if (arg == "--shrubbery-weight") {
            const auto value = next();
            if (!value) return std::nullopt;
            options.shrubbery_weight = std::stod(*value);
        } else if (arg == "--mono") {
            options.mono = true;
        } else if (arg == "--camera-height") {
            const auto value = next();
            if (!value) return std::nullopt;
            options.camera_height = std::stod(*value);
        } else if (arg == "--semantic-gp") {
            const auto value = next();
            if (!value || (*value != "on" && *value != "off")) return std::nullopt;
            options.semantic_gp = (*value == "on");
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return std::nullopt;
        }
    }
    if (options.mono && !options.semantic_dir.has_value()) {
        std::cerr << "--mono needs --semantic: without lidar, the scale can only come from the "
                     "camera height over the semantically detected ground plane.\n";
        return std::nullopt;
    }
    return options;
}

/// Loads the semantically segmented image belonging to a frame: <dir>/<%06d frame index>.png,
/// 8-bit grayscale with cityscapes label ids.
cv::Mat loadSemanticImage(const std::filesystem::path& semantic_dir, size_t frame_index) {
    std::ostringstream name;
    name << std::setw(6) << std::setfill('0') << frame_index << ".png";
    const auto path = semantic_dir / name.str();
    cv::Mat labels = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
    if (labels.empty()) {
        throw std::runtime_error("Cannot read semantic image " + path.string());
    }
    return labels;
}

/// Converts viso tracklets (deque of matches, newest first) to the pipeline's tracklet type.
/// stamps must also be ordered newest first; feature_points[i] corresponds to stamps[i].
///
/// With a semantic image, every track is labeled with the cityscapes label id under its newest
/// observation; the bundle adjustment maps labels to landmark weights (vegetation down, dynamic
/// objects and sky to outliers). Without one, labels keep their neutral default.
matches_msg_types::Tracklets convertTracklets(const feature_tracking::TrackletVector& viso_tracklets,
                                              const std::deque<matches_msg_types::TimestampNSec>& stamps,
                                              const std::optional<cv::Mat>& semantic_image) {
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

        if (semantic_image.has_value() && !track.feature_points.empty()) {
            const auto& newest = track.feature_points.front();
            const int u = std::clamp(static_cast<int>(newest.u), 0, semantic_image->cols - 1);
            const int v = std::clamp(static_cast<int>(newest.v), 0, semantic_image->rows - 1);
            track.label = static_cast<int>(semantic_image->at<uchar>(v, u));
        }

        out.tracks.push_back(std::move(track));
    }
    return out;
}

/// Converts the estimated ground plane (lidar coordinates) to the bundle adjustment's local
/// ground plane of a keyframe. Convention there: residual = normal . point + distance for points
/// in the keyframe (camera) frame, with the camera above the plane at positive distance.
///
/// Implausible fits (height far from the camera height, or a normal that is not roughly up) are
/// rejected — a bad plane as hard prior distorts the trajectory more than no plane at all.
kfba::Plane toBaGroundPlane(const mono_lidar::GroundPlane::Ptr& ground_plane,
                            const Eigen::Affine3d& transform_lidar_to_camera,
                            double expected_camera_height) {
    if (ground_plane == nullptr || !ground_plane->isSegmented()) {
        return kfba::Plane(); // negative distance -> unused in the optimization
    }
    const Eigen::Vector4f& coefficients = ground_plane->modelCoeffs();
    const Eigen::Vector3d normal_lidar =
        Eigen::Vector3d(coefficients[0], coefficients[1], coefficients[2]).normalized();
    // A point on the plane (n . p + d = 0) and the normal, both mapped into the camera frame.
    const Eigen::Vector3d point_camera =
        transform_lidar_to_camera * (-normal_lidar * static_cast<double>(coefficients[3]));
    const Eigen::Vector3d normal_camera = transform_lidar_to_camera.rotation() * normal_lidar;

    Eigen::Hyperplane<double, 3> plane(normal_camera, point_camera);
    if (plane.offset() < 0.) {
        plane.coeffs() = -plane.coeffs(); // camera above the plane means positive distance
    }

    const bool height_plausible = plane.offset() > 0.5 * expected_camera_height &&
                                  plane.offset() < 1.5 * expected_camera_height;
    const bool normal_up = plane.normal().dot(-Eigen::Vector3d::UnitY()) > std::cos(20. * M_PI / 180.);
    if (!height_plausible || !normal_up) {
        return kfba::Plane();
    }

    kfba::Plane out;
    out.direction = {plane.normal().x(), plane.normal().y(), plane.normal().z()};
    out.distance = plane.offset();
    return out;
}

/// Rough cityscapes colors for visualizing label images.
cv::Mat colorizeSemanticImage(const cv::Mat& labels) {
    static const std::map<int, cv::Vec3b> kPalette{
        {7, {128, 64, 128}},  // road
        {8, {232, 35, 244}},  // sidewalk
        {11, {70, 70, 70}},   // building
        {13, {153, 153, 190}},// fence
        {17, {153, 153, 153}},// pole
        {21, {35, 142, 107}}, // vegetation
        {22, {152, 251, 152}},// terrain
        {23, {180, 130, 70}}, // sky
        {26, {142, 0, 0}},    // car
    };
    cv::Mat color(labels.size(), CV_8UC3, cv::Scalar(60, 60, 60));
    for (int y = 0; y < labels.rows; ++y) {
        for (int x = 0; x < labels.cols; ++x) {
            const auto it = kPalette.find(labels.at<uchar>(y, x));
            if (it != kPalette.end()) {
                color.at<cv::Vec3b>(y, x) = it->second;
            }
        }
    }
    return color;
}

/// The 1/n-th of a rigid motion (slerp on the rotation, linear on the translation) — used to
/// derive the per-frame velocity from the motion between two keyframes n frames apart.
Eigen::Isometry3d fractionalMotion(const Eigen::Isometry3d& motion, double fraction) {
    const Eigen::Quaterniond rotation(motion.rotation());
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.linear() = Eigen::Quaterniond::Identity().slerp(fraction, rotation).toRotationMatrix();
    out.translation() = fraction * motion.translation();
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

    KittiDataReader reader(options->sequence_dir.string(), /*require_velodyne=*/!options->mono);
    const Eigen::Matrix3d K = reader.intrinsics();
    const double focal_length = K(0, 0);
    const Eigen::Vector2d principal_point(K(0, 2), K(1, 2));

    rr_vis::RerunVisualizer vis("limo_with_kitti", options->rerun_mode);

    // Feature tracking on grayscale images, gamma-corrected for stability. Tracks must span
    // several keyframes (0.25s apart at 10Hz), so keep 1s of history.
    feature_tracking::TrackerLibViso::Parameters tracker_parameters;
    tracker_parameters.maxTracklength = 10;
    feature_tracking::TrackerLibViso tracker(tracker_parameters);
    image_preproc::GammaCorrector gamma_corrector;

    // The bundle adjustment treats the camera frame as vehicle frame (identity extrinsics).
    auto camera_ba =
        std::make_shared<kfba::Camera>(focal_length, principal_point, Eigen::Isometry3d::Identity());

    kfba::BundleAdjusterKeyframes bundle_adjuster;
    bundle_adjuster.set_solver_time(0.3);
    // Note on the depth robust loss (outlier_rejection_options_.depth_thres, default 0.16m):
    // loosening it fixes a small scale underestimation during acceleration (the constant-velocity
    // prior is short and the tight Cauchy loss saturates the depth residuals that would stretch
    // the solution) but destabilizes the plane/pitch estimate late in long sequences. The default
    // measured best overall on KITTI 04.
    // Cap the landmark count per solve (voxel grid) but always keep the best depth-measured
    // landmarks — they anchor the scale.
    bundle_adjuster.landmark_selector_->addScheme(kfba::LandmarkSparsificationSchemeVoxel::createConst({}));
    {
        // Guarantee plenty of depth-anchored landmarks on the newest keyframes: they carry the
        // metric scale, and the solver's scale regularization weight (1000/num_depth_residuals)
        // only becomes negligible when there are enough of them.
        kfba::LandmarkSelectionSchemeAddDepth::Parameters depth_params;
        for (int keyframe_index = 0; keyframe_index < 4; ++keyframe_index) {
            depth_params.params_per_keyframe.emplace_back(
                keyframe_index,
                100,
                [](const kfba::Landmark::ConstPtr& lm) { return lm->has_measured_depth; },
                [](const kfba::Measurement& m, const Eigen::Vector3d&) { return static_cast<double>(m.d); });
            // With semantic labels, also guarantee ground-plane landmarks — they activate the
            // ground-plane residuals and regularization (and carry the scale in --mono).
            depth_params.params_per_keyframe.emplace_back(
                keyframe_index,
                50,
                [](const kfba::Landmark::ConstPtr& lm) { return lm->is_ground_plane; },
                [](const kfba::Measurement&, const Eigen::Vector3d& local_lm) { return local_lm.norm(); });
        }
        bundle_adjuster.landmark_selector_->addScheme(kfba::LandmarkSelectionSchemeAddDepth::createConst(depth_params));
    }

    // Keyframe cadence as in the original limo: at most one keyframe per 0.25s — larger baselines
    // between keyframes condition the bundle adjustment much better than every-frame keyframes —
    // and frames without feature flow (e.g. the car is standing) are rejected. The selector
    // output is the union of the sparsification and selection schemes minus rejections.
    kfba::KeyframeSelector keyframe_selector;
    const auto flow_scheme = std::make_shared<kfba::KeyframeRejectionSchemeFlow>(3.0);
    keyframe_selector.addScheme(kfba::KeyframeRejectionSchemeBase::ConstPtr(flow_scheme));
    keyframe_selector.addScheme(
        kfba::KeyframeSparsificationSchemeBase::ConstPtr(std::make_shared<kfba::KeyframeSparsificationSchemeTime>(0.25)));

    std::optional<mono_lidar::TrackletDepthEstimator> depth_estimator;

    std::deque<matches_msg_types::TimestampNSec> stamps;
    std::vector<Eigen::Isometry3d> frame_poses;          // pose of frame i in frame 0 (camera)
    std::vector<uint64_t> keyframe_stamps_per_frame;     // the keyframe stamp if frame i is one
    std::vector<bool> is_keyframe;
    // Per-frame velocity for the constant-velocity model, derived from the motion between the
    // last two optimized keyframes averaged over the frames in between.
    Eigen::Isometry3d last_motion = Eigen::Isometry3d::Identity();
    std::optional<Eigen::Isometry3d> last_keyframe_pose;
    int frames_since_last_keyframe = 1;
    bool bootstrapped = false;

    size_t processed = 0;
    for (auto gen = reader.frames(); gen.next();) {
        KittiFrame& frame = gen.value();
        if (options->max_frames >= 0 && static_cast<int>(processed) >= options->max_frames) {
            break;
        }

        if (!depth_estimator.has_value() && !options->mono) {
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

        std::optional<cv::Mat> semantic_image;
        if (options->semantic_dir.has_value()) {
            semantic_image = loadSemanticImage(*options->semantic_dir, frame.index);
        }
        auto tracklets = convertTracklets(viso_tracklets, stamps, semantic_image);

        // 2) Lidar depth for the tracked features. With a semantic image, the ground plane comes
        // from the labeled ground pixels instead of RANSAC. In --mono there is no lidar: features
        // keep d = -1 and the ground plane is the camera-height prior (refined per keyframe by
        // the optimization; its distance carries the scale).
        kfba::Plane keyframe_ground_plane;
        if (options->mono) {
            keyframe_ground_plane.direction = {0., -1., 0.};
            keyframe_ground_plane.distance = options->camera_height;
        } else {
            // The road plane for the depth fallback comes from RANSAC — it constrains the plane
            // axis and proved more robust on real data than the unconstrained fit through the
            // labeled ground pixels (SemanticPlane remains available in mono_lidar_depth).
            tracklets = depth_estimator->process(frame.cloud, tracklets);
            if (semantic_image.has_value() && options->semantic_gp) {
                keyframe_ground_plane = toBaGroundPlane(
                    depth_estimator->lastGroundPlane(), frame.transform_velodyne_to_camera, options->camera_height);
            }
        }

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
                                                kfba::Keyframe::FixationStatus::Pose, keyframe_ground_plane));
            bundle_adjuster.push(kfba::Keyframe(frame.timestamp_ns, tracklets, camera_ba, pose_prior.inverse(),
                                                kfba::Keyframe::FixationStatus::None, keyframe_ground_plane));
            bundle_adjuster.updateLabels(tracklets, options->shrubbery_weight);
            // solve() requires three keyframes; the first solve happens with the next one.
            selected_as_keyframe = true;
            bootstrapped = true;
        } else {
            auto candidate = std::make_shared<kfba::Keyframe>(
                frame.timestamp_ns, tracklets, camera_ba, pose_prior.inverse(),
                kfba::Keyframe::FixationStatus::None, keyframe_ground_plane);
            const auto selected = keyframe_selector.select({candidate}, bundle_adjuster.getActiveKeyframePtrs());
            selected_as_keyframe = !selected.empty();

            if (selected_as_keyframe) {
                bundle_adjuster.push(*candidate);
                // With semantic labels, vegetation landmarks are downweighted and dynamic
                // objects/sky become outliers; without them every class weighs the same.
                bundle_adjuster.updateLabels(tracklets, options->shrubbery_weight);
                bundle_adjuster.deactivateKeyframes();
                try {
                    const std::string summary = bundle_adjuster.solve();
                    pose_current = bundle_adjuster.getKeyframe().getEigenPose().inverse();
                } catch (const std::exception& e) {
                    std::cerr << "Bundle adjustment failed at frame " << frame.index << ": " << e.what()
                              << " — continuing with the motion prior." << std::endl;
                }
            } else if (!flow_scheme->isUsable(candidate, bundle_adjuster.getActiveKeyframePtrs())) {
                // No parallax (car standing): keep the previous pose.
                pose_current = frame_poses.back();
            }
            // Otherwise (time-sparsified frame between keyframes): the constant-velocity prior
            // already set in pose_current keeps the per-frame motion alive.
        }

        if (selected_as_keyframe) {
            if (last_keyframe_pose.has_value()) {
                const Eigen::Isometry3d keyframe_motion = last_keyframe_pose->inverse() * pose_current;
                last_motion = fractionalMotion(keyframe_motion, 1. / frames_since_last_keyframe);
            }
            last_keyframe_pose = pose_current;
            frames_since_last_keyframe = 1;
        } else {
            ++frames_since_last_keyframe;
        }
        frame_poses.push_back(pose_current);
        is_keyframe.push_back(selected_as_keyframe);
        keyframe_stamps_per_frame.push_back(frame.timestamp_ns);

        // 4) Visualization.
        vis.setFrame(static_cast<int64_t>(frame.index), frame.timestamp_ns);
        vis.logPose("world/camera", pose_current);
        vis.logCamera("world/camera/image", K, frame.image.cols, frame.image.rows);
        vis.logImage("world/camera/image", frame.image);
        if (semantic_image.has_value()) {
            vis.logImage("world/camera/semantic", colorizeSemanticImage(*semantic_image));
        }
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

    // 5) Final pose output: keyframes take their final optimized pose; frames between two
    // keyframes are interpolated along the keyframe-to-keyframe motion; frames after the last
    // keyframe are propagated with the online relative motions.
    // Every frame whose stamp is a bundle-adjustment keyframe acts as anchor — that includes the
    // fixed bootstrap keyframe, whose frame was not selected through the keyframe selector.
    std::vector<Eigen::Isometry3d> final_poses = frame_poses;
    std::vector<size_t> keyframe_indices;
    for (size_t i = 0; i < frame_poses.size(); ++i) {
        const auto it = bundle_adjuster.keyframes_.find(keyframe_stamps_per_frame[i]);
        if (it != bundle_adjuster.keyframes_.end()) {
            final_poses[i] = it->second->getEigenPose().inverse();
            keyframe_indices.push_back(i);
        }
    }
    for (size_t k = 0; k + 1 < keyframe_indices.size(); ++k) {
        const size_t a = keyframe_indices[k];
        const size_t b = keyframe_indices[k + 1];
        const Eigen::Isometry3d segment_motion = final_poses[a].inverse() * final_poses[b];
        for (size_t i = a + 1; i < b; ++i) {
            final_poses[i] =
                final_poses[a] * fractionalMotion(segment_motion, static_cast<double>(i - a) / (b - a));
        }
    }
    if (!keyframe_indices.empty()) {
        for (size_t i = keyframe_indices.back() + 1; i < final_poses.size(); ++i) {
            final_poses[i] = final_poses[i - 1] * (frame_poses[i - 1].inverse() * frame_poses[i]);
        }
        // Frames before the first keyframe barely move relative to it; anchor them there.
        for (size_t i = 0; i < keyframe_indices.front(); ++i) {
            final_poses[i] = final_poses[keyframe_indices.front()] *
                             (frame_poses[keyframe_indices.front()].inverse() * frame_poses[i]);
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
