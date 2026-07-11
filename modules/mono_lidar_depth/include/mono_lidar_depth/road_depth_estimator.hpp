// Copyright 2017-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <expected>
#include <memory>
#include <optional>

#include <Eigen/Eigen>

#include "camera_pinhole.hpp"
#include "depth_filter.hpp"
#include "line_plane_intersection.hpp"
#include "parameters.hpp"
#include "types.hpp"

namespace mono_lidar {

/// Depth estimation for features on the road: fits a plane through ground points around the
/// feature (strategy per RoadPlaneFitMode) and intersects the viewing ray with it.
class RoadDepthEstimator {
public:
    RoadDepthEstimator(RoadPlaneFitMode mode, double triangle_min_z_x_relation);

    void enableThresholdGlobal(std::shared_ptr<const DepthThresholdGlobal> threshold) {
        threshold_global_ = std::move(threshold);
    }
    void enableThresholdLocal(std::shared_ptr<const DepthThresholdLocal> threshold) {
        threshold_local_ = std::move(threshold);
    }

    /// Prior for the m_estimator strategy (usually the RANSAC ground plane).
    void setPlanePrior(const Eigen::Hyperplane<double, 3>& plane) {
        prior_plane_ = plane;
    }

    /// @param feature_image_cs feature point in image coordinates
    /// @param ground_points    neighboring ground-plane inlier points (camera coordinates)
    /// @return the estimated depth, or the failure reason
    std::expected<double, DepthResultType> calculateDepth(const Eigen::Vector2d& feature_image_cs,
                                                          const CameraPinhole& camera,
                                                          const VecOfVec3d& ground_points) const;

private:
    std::expected<Eigen::Hyperplane<double, 3>, DepthResultType> fitPlane(const VecOfVec3d& ground_points) const;

    RoadPlaneFitMode mode_;
    double triangle_min_z_x_relation_;
    LinePlaneIntersection intersection_;

    std::shared_ptr<const DepthThresholdGlobal> threshold_global_;
    std::shared_ptr<const DepthThresholdLocal> threshold_local_;
    std::optional<Eigen::Hyperplane<double, 3>> prior_plane_;
};

} // namespace mono_lidar
