// Copyright 2017-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <string_view>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace mono_lidar {

using VecOfVec3d = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

/// Outcome of the depth estimation of a single feature point.
enum class DepthResultType {
    unspecified,
    success,
    success_road,
    insufficient_neighbors,
    histogram_no_local_max,
    threshold_depth_global_greater_max,
    threshold_depth_global_smaller_min,
    threshold_depth_local_greater_max,
    threshold_depth_local_smaller_min,
    triangle_not_planar,
    triangle_not_planar_insufficient_points,
    corner_behind_camera,
    plane_viewray_not_orthogonal,
    pca_is_point,
    pca_is_line,
    pca_is_cubic,
    insufficient_road_points,
};

constexpr std::string_view toString(DepthResultType type) {
    switch (type) {
    case DepthResultType::unspecified:
        return "unspecified";
    case DepthResultType::success:
        return "success";
    case DepthResultType::success_road:
        return "success_road";
    case DepthResultType::insufficient_neighbors:
        return "insufficient_neighbors";
    case DepthResultType::histogram_no_local_max:
        return "histogram_no_local_max";
    case DepthResultType::threshold_depth_global_greater_max:
        return "threshold_depth_global_greater_max";
    case DepthResultType::threshold_depth_global_smaller_min:
        return "threshold_depth_global_smaller_min";
    case DepthResultType::threshold_depth_local_greater_max:
        return "threshold_depth_local_greater_max";
    case DepthResultType::threshold_depth_local_smaller_min:
        return "threshold_depth_local_smaller_min";
    case DepthResultType::triangle_not_planar:
        return "triangle_not_planar";
    case DepthResultType::triangle_not_planar_insufficient_points:
        return "triangle_not_planar_insufficient_points";
    case DepthResultType::corner_behind_camera:
        return "corner_behind_camera";
    case DepthResultType::plane_viewray_not_orthogonal:
        return "plane_viewray_not_orthogonal";
    case DepthResultType::pca_is_point:
        return "pca_is_point";
    case DepthResultType::pca_is_line:
        return "pca_is_line";
    case DepthResultType::pca_is_cubic:
        return "pca_is_cubic";
    case DepthResultType::insufficient_road_points:
        return "insufficient_road_points";
    }
    return "unknown";
}

constexpr bool isSuccess(DepthResultType type) {
    return type == DepthResultType::success || type == DepthResultType::success_road;
}

/// What to do with a depth estimate that violates a threshold: discard it or clamp it to the bound.
enum class ThresholdMode { dispose, adjust };

/// How the tolerance of the local depth threshold is interpreted.
enum class ThresholdToleranceType { absolute, relative };

enum class ThresholdResult { in_bounds, smaller_min, greater_max };

/// A lidar point cloud, transformed into camera coordinates and projected to the image.
///
/// "visible" refers to the subset of points that project into the image bounds and lie in front
/// of the camera; visibleToCloudIndex maps a visible-point index back to the original cloud.
struct ProjectedPointcloud {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3Xd points_lidar_cs;   ///< all points in lidar coordinates
    Eigen::Matrix3Xd points_camera_cs;  ///< all points in camera coordinates
    Eigen::Matrix2Xd points_image_cs;   ///< all points projected to the image plane
    Eigen::Matrix2Xd visible_image_cs;  ///< image-plane coordinates of the visible subset
    std::vector<int> visible_to_cloud;  ///< maps visible-point index -> original cloud index

    int toCloudIndex(int visible_index) const {
        return visible_to_cloud.at(visible_index);
    }

    Eigen::Vector3d pointCameraCs(int cloud_index) const {
        return points_camera_cs.col(cloud_index);
    }

    Eigen::Vector3d visiblePointCameraCs(int visible_index) const {
        return pointCameraCs(toCloudIndex(visible_index));
    }
};

} // namespace mono_lidar
