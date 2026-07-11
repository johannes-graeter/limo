#include "mono_lidar_depth/tracklet_depth_estimator.hpp"

#include <iostream>
#include <vector>

namespace mono_lidar {

const std::set<int> TrackletDepthEstimator::kDefaultGroundplaneLabels{6, 7, 8, 9};

TrackletDepthEstimator::TrackletDepthEstimator(const DepthEstimatorParameters& parameters,
                                               CameraPinhole::Ptr camera,
                                               const Eigen::Affine3d& transform_lidar_to_camera)
        : depth_estimator_(parameters, camera, transform_lidar_to_camera), camera_(std::move(camera)),
          transform_lidar_to_camera_(transform_lidar_to_camera), parameters_(parameters) {
}

GroundPlane::Ptr TrackletDepthEstimator::makeGroundPlane(const std::optional<cv::Mat>& semantic_image) const {
    if (!semantic_image.has_value()) {
        // Null lets the DepthEstimator create a RansacPlane on the cloud.
        return nullptr;
    }

    SemanticPlane::Camera camera;
    camera.f = camera_->intrinsics()(0, 0);
    camera.cu = camera_->intrinsics()(0, 2);
    camera.cv = camera_->intrinsics()(1, 2);
    camera.transform_cam_lidar = transform_lidar_to_camera_;

    return std::make_shared<SemanticPlane>(
        *semantic_image, camera, kDefaultGroundplaneLabels, parameters_.ransac_plane_refinement_threshold);
}

matches_msg_types::Tracklets TrackletDepthEstimator::process(const Cloud::ConstPtr& cloud,
                                                             const matches_msg_types::Tracklets& tracklets_in,
                                                             const std::optional<cv::Mat>& semantic_image) {
    // Collect the feature points that still need a depth estimate: the newest observation of
    // every track, plus the second-newest for tracks we see for the first time.
    std::vector<NewFeature> new_features;
    new_features.reserve(tracklets_in.tracks.size());
    for (const auto& track : tracklets_in.tracks) {
        NewFeature feature;
        feature.id = track.id;
        feature.point = track.feature_points.at(0).toEigen2d().cast<double>();
        if (!tracklets_.contains(track.id) && track.feature_points.size() > 1) {
            feature.point_last = track.feature_points.at(1).toEigen2d().cast<double>();
        }
        new_features.push_back(feature);
    }

    // Depths of the second-newest observations, estimated with the cloud of the previous frame.
    Eigen::Matrix2Xd points_last(2, static_cast<Eigen::Index>(new_features.size()));
    Eigen::Index last_count = 0;
    for (const auto& feature : new_features) {
        if (feature.point_last.has_value()) {
            points_last.col(last_count++) = *feature.point_last;
        }
    }
    points_last.conservativeResize(2, last_count);

    Eigen::VectorXd depths_last = Eigen::VectorXd::Constant(last_count, -1.);
    if (cloud_last_frame_ != nullptr && last_count > 0) {
        try {
            depths_last = depth_estimator_.calculateDepths(cloud_last_frame_, points_last, ground_plane_last_).depths;
        } catch (const GroundPlane::ExceptionPclInvalid& e) {
            std::cerr << "TrackletDepthEstimator: last frame continues with invalid depths: " << e.what() << "\n";
        }
    }

    // Depths of the newest observations with the current cloud.
    Eigen::Matrix2Xd points_current(2, static_cast<Eigen::Index>(new_features.size()));
    for (size_t i = 0; i < new_features.size(); ++i) {
        points_current.col(static_cast<Eigen::Index>(i)) = new_features[i].point;
    }

    GroundPlane::Ptr ground_plane = makeGroundPlane(semantic_image);
    Eigen::VectorXd depths_current = Eigen::VectorXd::Constant(points_current.cols(), -1.);
    try {
        depths_current = depth_estimator_.calculateDepths(cloud, points_current, ground_plane).depths;
        cloud_last_frame_ = cloud;
    } catch (const GroundPlane::ExceptionPclInvalid& e) {
        std::cerr << "TrackletDepthEstimator: current frame continues with invalid depths: " << e.what() << "\n";
        ground_plane = nullptr;
        cloud_last_frame_ = nullptr;
    }
    ground_plane_last_ = ground_plane;

    // Integrate the estimates into the cache, newest observation first.
    std::vector<TrackletId> updated_ids;
    updated_ids.reserve(new_features.size());
    Eigen::Index current_index = 0;
    Eigen::Index last_index = 0;
    for (const auto& feature : new_features) {
        auto& cached = tracklets_[feature.id];
        if (feature.point_last.has_value()) {
            cached.push_front(Observation{static_cast<float>(feature.point_last->x()),
                                          static_cast<float>(feature.point_last->y()),
                                          static_cast<float>(depths_last(last_index++))});
        }
        cached.push_front(Observation{static_cast<float>(feature.point.x()),
                                      static_cast<float>(feature.point.y()),
                                      static_cast<float>(depths_current(current_index++))});
        updated_ids.push_back(feature.id);
    }

    // Build the output from the cache; stamps and per-track metadata pass through.
    matches_msg_types::Tracklets out;
    out.stamps = tracklets_in.stamps;
    out.tracks.reserve(tracklets_in.tracks.size());
    for (const auto& track_in : tracklets_in.tracks) {
        const auto& cached = tracklets_.at(track_in.id);

        matches_msg_types::Tracklet track_out;
        track_out.id = track_in.id;
        track_out.age = track_in.age;
        track_out.is_outlier = track_in.is_outlier;
        track_out.label = track_in.label;
        track_out.feature_points.reserve(cached.size());
        for (const auto& observation : cached) {
            track_out.feature_points.emplace_back(observation.u, observation.v, observation.depth);
        }

        out.tracks.push_back(std::move(track_out));
    }

    tidyUpTracklets(updated_ids);

    return out;
}

void TrackletDepthEstimator::tidyUpTracklets(const std::vector<TrackletId>& updated_ids) {
    const std::set<TrackletId> updated(updated_ids.begin(), updated_ids.end());
    std::erase_if(tracklets_, [&](const auto& entry) { return !updated.contains(entry.first); });
}

} // namespace mono_lidar
