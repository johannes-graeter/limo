// Copyright 2016-2026. All rights reserved.
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
#include <vector>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "camera_pinhole.hpp"
#include "depth_filter.hpp"
#include "ground_plane.hpp"
#include "histogram.hpp"
#include "line_plane_intersection.hpp"
#include "neighbor_finder.hpp"
#include "parameters.hpp"
#include "road_depth_estimator.hpp"
#include "statistics.hpp"
#include "types.hpp"

namespace mono_lidar {

/// Estimates the depth of 2d image feature points from a synchronized lidar point cloud.
///
/// Per feature point: the projected lidar points inside a pixel rectangle around the feature are
/// segmented by depth (histogram) to isolate the foreground object; a local plane is spanned
/// through the segmented points (max spanning triangle or PCA); the viewing ray is intersected
/// with that plane; the resulting depth is validated against global and local thresholds. For
/// features that fail this (typically points on the road, where lidar rows are far apart), a
/// wider search against the ground plane inliers is used as fallback.
class DepthEstimator {
public:
    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<Point>;

    struct DepthResults {
        Eigen::VectorXd depths;                ///< estimated depth per input point, -1 on failure
        std::vector<DepthResultType> results;  ///< outcome per input point
    };

    /// @param camera               intrinsics of the (undistorted) camera
    /// @param transform_lidar_to_cam pose of the lidar in camera coordinates
    DepthEstimator(const DepthEstimatorParameters& parameters,
                   CameraPinhole::Ptr camera,
                   const Eigen::Affine3d& transform_lidar_to_cam);

    /// Set the lidar cloud of the current frame; transforms it to camera coordinates, projects it
    /// to the image and (if enabled) segments the ground plane. If ground_plane is null and RANSAC
    /// is enabled, a RansacPlane is created in place.
    void setInputCloud(const Cloud::ConstPtr& cloud, GroundPlane::Ptr& ground_plane);

    /// Depth for each column of feature_points_image_cs. Requires setInputCloud for this frame.
    DepthResults calculateDepths(const Eigen::Matrix2Xd& feature_points_image_cs,
                                 const GroundPlane::Ptr& ground_plane);

    /// Convenience overload: setInputCloud + calculateDepths.
    DepthResults calculateDepths(const Cloud::ConstPtr& cloud,
                                 const Eigen::Matrix2Xd& feature_points_image_cs,
                                 GroundPlane::Ptr& ground_plane);

    /// Depth of a single feature point, or the failure reason.
    std::expected<double, DepthResultType> calculateDepth(const Eigen::Vector2d& feature_image_cs,
                                                          const GroundPlane::Ptr& ground_plane) const;

    const DepthEstimatorParameters& parameters() const {
        return parameters_;
    }

    const CameraPinhole& camera() const {
        return *camera_;
    }

    const DepthCalculationStatistics& statistics() const {
        return statistics_;
    }

    /// Image-plane coordinates of the lidar points visible in the camera (for visualization).
    const Eigen::Matrix2Xd& visiblePointsImageCs() const {
        return points_.visible_image_cs;
    }

    /// The current cloud in camera coordinates (for visualization).
    const Eigen::Matrix3Xd& cloudCameraCs() const {
        return points_.points_camera_cs;
    }

private:
    /// Neighbors around the feature point: indices into the visible subset plus their 3d points.
    struct Neighborhood {
        std::vector<int> visible_indices;
        VecOfVec3d points;
    };

    std::optional<Neighborhood> findNeighbors(const Eigen::Vector2d& feature_image_cs,
                                              float scale_width = 1.f,
                                              float scale_height = 1.f) const;

    /// Depth-segments the neighborhood (histogram blob) if enabled.
    std::optional<VecOfVec3d> segmentNeighbors(const Neighborhood& neighborhood) const;

    /// Ground-plane inliers among the neighbors; nullopt if the patch is unusable.
    std::optional<VecOfVec3d> segmentGroundPlaneNeighbors(const Neighborhood& neighborhood,
                                                          const GroundPlane::Ptr& ground_plane) const;

    /// Spans a local plane through the segmented points and intersects the viewing ray.
    std::expected<double, DepthResultType> calculateDepthSegmented(const Eigen::Vector2d& feature_image_cs,
                                                                   const VecOfVec3d& points_segmented) const;

    DepthEstimatorParameters parameters_;
    CameraPinhole::Ptr camera_;
    Eigen::Affine3d transform_lidar_to_cam_;
    Eigen::Affine3d transform_cam_to_lidar_;

    PixelNeighborFinder neighbor_finder_;
    LinePlaneIntersection line_plane_intersection_;
    std::shared_ptr<const DepthThresholdGlobal> threshold_global_;
    std::shared_ptr<const DepthThresholdLocal> threshold_local_;
    std::unique_ptr<RoadDepthEstimator> road_depth_estimator_;

    ProjectedPointcloud points_;
    bool cloud_set_{false};

    DepthCalculationStatistics statistics_;
};

} // namespace mono_lidar
