// Copyright 2017-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <optional>

#include <Eigen/Eigen>
#include <matches_msg_types/tracklets.hpp>
#include <opencv2/core/mat.hpp>

#include "depth_estimator.hpp"
#include "ground_plane.hpp"
#include "parameters.hpp"

namespace mono_lidar {

/// Adds lidar depth to feature tracklets.
///
/// Per frame, the newly observed feature points of the incoming tracklets (the newest point of
/// every track, plus the second-newest for tracks seen for the first time) are depth-estimated
/// with the DepthEstimator; estimated depths of older observations are cached, so each
/// observation is estimated exactly once with the cloud of its own frame.
///
/// The ground plane for the road fallback comes from RANSAC on the cloud. Once semantically
/// segmented images are available, pass them to process(): ground points are then taken from the
/// segmentation instead (SemanticPlane); the rest of the pipeline is unchanged.
class TrackletDepthEstimator {
public:
    using Cloud = DepthEstimator::Cloud;

    /// Labels of a typical semantic segmentation that correspond to the ground plane.
    static const std::set<int> kDefaultGroundplaneLabels;

    /// @param transform_lidar_to_camera pose of the lidar in camera coordinates
    TrackletDepthEstimator(const DepthEstimatorParameters& parameters,
                           CameraPinhole::Ptr camera,
                           const Eigen::Affine3d& transform_lidar_to_camera);

    /// Estimates depth for all new feature points of tracklets_in and returns the tracklets with
    /// the depth field d filled for every cached observation (-1 where estimation failed).
    ///
    /// @param cloud        lidar cloud synchronized with the tracklets' newest frame
    /// @param tracklets_in tracklets from the feature tracker, newest observation first
    /// @param semantic_image optional semantically segmented image (ground plane from labels
    ///                       instead of RANSAC) — the seam for the upcoming segmentation input
    matches_msg_types::Tracklets process(const Cloud::ConstPtr& cloud,
                                         const matches_msg_types::Tracklets& tracklets_in,
                                         const std::optional<cv::Mat>& semantic_image = std::nullopt);

    const DepthEstimator& depthEstimator() const {
        return depth_estimator_;
    }

    /// Ground plane of the last processed frame (in lidar coordinates), null before the first
    /// frame or if the last segmentation failed.
    GroundPlane::Ptr lastGroundPlane() const {
        return ground_plane_last_;
    }

private:
    using TrackletId = uint64_t;

    /// One cached observation: image point plus its estimated depth.
    struct Observation {
        float u;
        float v;
        float depth{-1.f};
    };

    /// Feature points of the current frame that still need a depth estimate.
    struct NewFeature {
        TrackletId id;
        Eigen::Vector2d point;                        ///< newest observation
        std::optional<Eigen::Vector2d> point_last;    ///< second-newest, only for new tracklets
    };

    GroundPlane::Ptr makeGroundPlane(const std::optional<cv::Mat>& semantic_image) const;

    /// Removes cached tracklets that were not updated in this frame.
    void tidyUpTracklets(const std::vector<TrackletId>& updated_ids);

    DepthEstimator depth_estimator_;
    CameraPinhole::Ptr camera_;
    Eigen::Affine3d transform_lidar_to_camera_;
    DepthEstimatorParameters parameters_;

    /// Cache of tracklets with estimated depths, newest observation first.
    std::map<TrackletId, std::deque<Observation>> tracklets_;

    Cloud::ConstPtr cloud_last_frame_;
    GroundPlane::Ptr ground_plane_last_;
};

} // namespace mono_lidar
