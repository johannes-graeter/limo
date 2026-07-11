#include "mono_lidar_depth/road_depth_estimator.hpp"

#include <stdexcept>

#include "mono_lidar_depth/plane_estimation.hpp"

namespace mono_lidar {

RoadDepthEstimator::RoadDepthEstimator(RoadPlaneFitMode mode, double triangle_min_z_x_relation)
        : mode_(mode), triangle_min_z_x_relation_(triangle_min_z_x_relation) {
    if (mode == RoadPlaneFitMode::none) {
        throw std::invalid_argument("RoadDepthEstimator: fit mode must not be 'none'");
    }
}

std::expected<Eigen::Hyperplane<double, 3>, DepthResultType> RoadDepthEstimator::fitPlane(
    const VecOfVec3d& ground_points) const {
    switch (mode_) {
    case RoadPlaneFitMode::max_spanning_triangle: {
        const auto triangle = maxSpanningTriangle(ground_points);
        if (!triangle.has_value()) {
            return std::unexpected(DepthResultType::insufficient_neighbors);
        }
        // Reject patches that barely extend in driving direction; their plane fit tilts easily.
        if (!checkZxRatio(ground_points, triangle_min_z_x_relation_)) {
            return std::unexpected(DepthResultType::insufficient_road_points);
        }
        return Eigen::Hyperplane<double, 3>::Through((*triangle)[0], (*triangle)[1], (*triangle)[2]);
    }
    case RoadPlaneFitMode::least_squares: {
        const auto plane = fitPlaneLeastSquares(ground_points);
        if (!plane.has_value()) {
            return std::unexpected(DepthResultType::insufficient_road_points);
        }
        return *plane;
    }
    case RoadPlaneFitMode::m_estimator: {
        if (!prior_plane_.has_value()) {
            throw std::logic_error("RoadDepthEstimator: m_estimator requires a plane prior (setPlanePrior)");
        }
        const auto plane = fitPlaneMEstimator(ground_points, *prior_plane_);
        if (!plane.has_value()) {
            return std::unexpected(DepthResultType::insufficient_road_points);
        }
        return *plane;
    }
    case RoadPlaneFitMode::none:
        break;
    }
    throw std::logic_error("RoadDepthEstimator: invalid fit mode");
}

std::expected<double, DepthResultType> RoadDepthEstimator::calculateDepth(const Eigen::Vector2d& feature_image_cs,
                                                                          const CameraPinhole& camera,
                                                                          const VecOfVec3d& ground_points) const {
    const auto plane = fitPlane(ground_points);
    if (!plane.has_value()) {
        return std::unexpected(plane.error());
    }

    Eigen::Vector3d ray_support;
    Eigen::Vector3d ray_direction;
    camera.getViewingRays(feature_image_cs, ray_support, ray_direction);
    if (ray_direction.z() < 0.) {
        ray_direction *= -1.;
    }

    const auto intersection = intersection_.intersect(*plane, ray_support, ray_direction);
    if (!intersection.has_value()) {
        return std::unexpected(DepthResultType::plane_viewray_not_orthogonal);
    }
    double depth = intersection->depth;

    if (threshold_global_ != nullptr) {
        const auto result = threshold_global_->check(depth);
        if (result == ThresholdResult::smaller_min) {
            return std::unexpected(DepthResultType::threshold_depth_global_smaller_min);
        }
        if (result == ThresholdResult::greater_max) {
            return std::unexpected(DepthResultType::threshold_depth_global_greater_max);
        }
    }

    if (threshold_local_ != nullptr) {
        const auto result = threshold_local_->check(ground_points, depth);
        if (result == ThresholdResult::smaller_min) {
            return std::unexpected(DepthResultType::threshold_depth_local_smaller_min);
        }
        if (result == ThresholdResult::greater_max) {
            return std::unexpected(DepthResultType::threshold_depth_local_greater_max);
        }
    }

    return depth;
}

} // namespace mono_lidar
