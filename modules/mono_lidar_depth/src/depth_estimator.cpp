#include "mono_lidar_depth/depth_estimator.hpp"

#include <stdexcept>

#include "mono_lidar_depth/pca.hpp"
#include "mono_lidar_depth/plane_estimation.hpp"

namespace mono_lidar {

namespace {

/// Transforms the cloud into camera coordinates, projects it to the image and collects the
/// visible subset (inside the image bounds; behind-camera points are rejected later during
/// rasterization).
ProjectedPointcloud projectCloudToCamera(const DepthEstimator::Cloud::ConstPtr& cloud,
                                         const Eigen::Affine3d& lidar_to_cam,
                                         const CameraPinhole& camera) {
    ProjectedPointcloud points;

    points.points_lidar_cs = cloud->getMatrixXfMap().cast<double>().topRows<3>();
    const auto point_count = static_cast<int>(points.points_lidar_cs.cols());

    points.points_camera_cs = lidar_to_cam * points.points_lidar_cs;

    points.points_image_cs.resize(2, point_count);
    const auto in_range = camera.getImagePoints(points.points_camera_cs, points.points_image_cs);

    const auto is_visible = [&](int i) {
        return in_range[i] && points.points_image_cs(0, i) > 0. &&
               points.points_image_cs(0, i) < static_cast<double>(camera.width()) &&
               points.points_image_cs(1, i) > 0. && points.points_image_cs(1, i) < static_cast<double>(camera.height());
    };

    int visible_count = 0;
    for (int i = 0; i < point_count; ++i) {
        if (is_visible(i)) {
            ++visible_count;
        }
    }

    points.visible_image_cs.resize(2, visible_count);
    points.visible_to_cloud.clear();
    points.visible_to_cloud.reserve(static_cast<size_t>(visible_count));

    int visible_index = 0;
    for (int i = 0; i < point_count; ++i) {
        if (is_visible(i)) {
            points.visible_image_cs.col(visible_index) = points.points_image_cs.col(i);
            points.visible_to_cloud.push_back(i);
            ++visible_index;
        }
    }

    return points;
}

} // namespace

DepthEstimator::DepthEstimator(const DepthEstimatorParameters& parameters,
                               CameraPinhole::Ptr camera,
                               const Eigen::Affine3d& transform_lidar_to_cam)
        : parameters_(parameters), camera_(std::move(camera)), transform_lidar_to_cam_(transform_lidar_to_cam),
          transform_cam_to_lidar_(transform_lidar_to_cam.inverse()),
          neighbor_finder_(camera_->width(), camera_->height(), parameters.pixel_search_width,
                           parameters.pixel_search_height),
          line_plane_intersection_(parameters.viewray_plane_orthogonality_threshold > 0.
                                       ? LinePlaneIntersection(parameters.viewray_plane_orthogonality_threshold)
                                       : LinePlaneIntersection()) {
    if (parameters_.threshold_depth_enabled) {
        threshold_global_ = std::make_shared<DepthThresholdGlobal>(
            parameters_.threshold_depth_mode, parameters_.threshold_depth_min, parameters_.threshold_depth_max);
    }
    if (parameters_.threshold_depth_local_enabled) {
        threshold_local_ = std::make_shared<DepthThresholdLocal>(parameters_.threshold_depth_local_mode,
                                                                 parameters_.threshold_depth_local_tolerance_type,
                                                                 parameters_.threshold_depth_local_tolerance);
    }
    if (parameters_.use_ransac_plane && parameters_.road_plane_fit_mode != RoadPlaneFitMode::none) {
        road_depth_estimator_ =
            std::make_unique<RoadDepthEstimator>(parameters_.road_plane_fit_mode, parameters_.road_triangle_min_z_x_relation);
        if (threshold_global_ != nullptr) {
            road_depth_estimator_->enableThresholdGlobal(threshold_global_);
        }
        if (threshold_local_ != nullptr) {
            road_depth_estimator_->enableThresholdLocal(threshold_local_);
        }
    }
}

void DepthEstimator::setInputCloud(const Cloud::ConstPtr& cloud, GroundPlane::Ptr& ground_plane) {
    points_ = projectCloudToCamera(cloud, transform_lidar_to_cam_, *camera_);
    neighbor_finder_.initializeProjection(points_);
    cloud_set_ = true;

    if (!parameters_.use_ransac_plane) {
        return;
    }

    if (ground_plane == nullptr) {
        ground_plane = std::make_shared<RansacPlane>(parameters_);
    }
    if (!ground_plane->isSegmented()) {
        ground_plane->calculateInliersPlane(cloud, parameters_.ransac_plane_min_z, parameters_.ransac_plane_max_z);
    }

    if (road_depth_estimator_ != nullptr && parameters_.road_plane_fit_mode == RoadPlaneFitMode::m_estimator) {
        const auto& coeffs = ground_plane->modelCoeffs();
        const Eigen::Vector3d normal(coeffs[0], coeffs[1], coeffs[2]);
        road_depth_estimator_->setPlanePrior(
            Eigen::Hyperplane<double, 3>(normal.normalized(), static_cast<double>(coeffs[3])));
    }
}

DepthEstimator::DepthResults DepthEstimator::calculateDepths(const Cloud::ConstPtr& cloud,
                                                             const Eigen::Matrix2Xd& feature_points_image_cs,
                                                             GroundPlane::Ptr& ground_plane) {
    setInputCloud(cloud, ground_plane);
    return calculateDepths(feature_points_image_cs, ground_plane);
}

DepthEstimator::DepthResults DepthEstimator::calculateDepths(const Eigen::Matrix2Xd& feature_points_image_cs,
                                                             const GroundPlane::Ptr& ground_plane) {
    if (!cloud_set_) {
        throw std::logic_error("DepthEstimator: calculateDepths called before setInputCloud");
    }

    const auto point_count = static_cast<int>(feature_points_image_cs.cols());
    DepthResults results;
    results.depths.resize(point_count);
    results.results.resize(static_cast<size_t>(point_count));

    for (int i = 0; i < point_count; ++i) {
        const auto result = calculateDepth(feature_points_image_cs.col(i), ground_plane);
        if (result.has_value()) {
            results.depths(i) = *result;
            // Distinguishing plain and road success is only relevant for the statistics below.
            results.results[static_cast<size_t>(i)] = DepthResultType::success;
        } else {
            results.depths(i) = -1.;
            results.results[static_cast<size_t>(i)] = result.error();
        }
    }

    if (parameters_.collect_statistics) {
        statistics_.startFrame();
        for (const auto result : results.results) {
            statistics_.add(result);
        }
    }

    return results;
}

std::expected<double, DepthResultType> DepthEstimator::calculateDepth(const Eigen::Vector2d& feature_image_cs,
                                                                      const GroundPlane::Ptr& ground_plane) const {
    const auto neighborhood = findNeighbors(feature_image_cs);
    if (!neighborhood.has_value()) {
        return std::unexpected(DepthResultType::insufficient_neighbors);
    }

    // Isolate the foreground object the feature sits on, then span a local plane through it.
    auto failure = DepthResultType::histogram_no_local_max;
    const auto points_segmented = segmentNeighbors(*neighborhood);
    if (points_segmented.has_value()) {
        const auto depth = calculateDepthSegmented(feature_image_cs, *points_segmented);
        if (depth.has_value()) {
            return *depth;
        }
        failure = depth.error();
    }

    // Fallback for features on the road: wider search, restricted to ground plane inliers.
    if (ground_plane != nullptr && road_depth_estimator_ != nullptr) {
        const auto road_neighborhood = findNeighbors(feature_image_cs, 2.f, 1.5f);
        if (!road_neighborhood.has_value()) {
            return std::unexpected(DepthResultType::insufficient_neighbors);
        }

        const auto ground_points = segmentGroundPlaneNeighbors(*road_neighborhood, ground_plane);
        if (!ground_points.has_value()) {
            return std::unexpected(failure);
        }

        return road_depth_estimator_->calculateDepth(feature_image_cs, *camera_, *ground_points);
    }

    return std::unexpected(failure);
}

std::optional<DepthEstimator::Neighborhood> DepthEstimator::findNeighbors(const Eigen::Vector2d& feature_image_cs,
                                                                          float scale_width,
                                                                          float scale_height) const {
    Neighborhood neighborhood;
    neighborhood.visible_indices = neighbor_finder_.findNeighborIndices(feature_image_cs, scale_width, scale_height);
    neighborhood.points = PixelNeighborFinder::collectPoints(points_, neighborhood.visible_indices);

    if (static_cast<int>(neighborhood.points.size()) < parameters_.neighbor_count_min) {
        return std::nullopt;
    }
    return neighborhood;
}

std::optional<VecOfVec3d> DepthEstimator::segmentNeighbors(const Neighborhood& neighborhood) const {
    if (!parameters_.use_histogram_segmentation) {
        return neighborhood.points;
    }

    Eigen::VectorXd depths(static_cast<Eigen::Index>(neighborhood.points.size()));
    for (size_t i = 0; i < neighborhood.points.size(); ++i) {
        depths(static_cast<Eigen::Index>(i)) = std::min(neighborhood.points[i].z(), 999.);
    }

    auto blob = segmentDepthBlob(neighborhood.points,
                                 neighborhood.visible_indices,
                                 depths,
                                 parameters_.histogram_bin_width,
                                 parameters_.histogram_min_pointcount);
    if (!blob.has_value()) {
        return std::nullopt;
    }
    return std::move(blob->points);
}

std::optional<VecOfVec3d> DepthEstimator::segmentGroundPlaneNeighbors(const Neighborhood& neighborhood,
                                                                      const GroundPlane::Ptr& ground_plane) const {
    const Eigen::Vector4f& coeffs = ground_plane->modelCoeffs();
    const Eigen::Hyperplane<double, 3> plane_lidar_cs(
        Eigen::Vector3d(coeffs[0], coeffs[1], coeffs[2]).normalized(), static_cast<double>(coeffs[3]));

    VecOfVec3d ground_points;
    for (size_t i = 0; i < neighborhood.points.size(); ++i) {
        // The plane and its inlier indices live in the lidar frame / original cloud.
        const Eigen::Vector3d point_lidar_cs = transform_cam_to_lidar_ * neighborhood.points[i];
        if (plane_lidar_cs.absDistance(point_lidar_cs) > parameters_.ransac_plane_point_distance_threshold) {
            return std::nullopt;
        }

        const int cloud_index = points_.toCloudIndex(neighborhood.visible_indices[i]);
        if (ground_plane->isPointInPlane(cloud_index)) {
            ground_points.push_back(neighborhood.points[i]);
        }
    }

    if (ground_points.size() < 3) {
        return std::nullopt;
    }
    return ground_points;
}

std::expected<double, DepthResultType> DepthEstimator::calculateDepthSegmented(
    const Eigen::Vector2d& feature_image_cs, const VecOfVec3d& points_segmented) const {
    Eigen::Vector3d ray_support;
    Eigen::Vector3d ray_direction;
    camera_->getViewingRays(feature_image_cs, ray_support, ray_direction);
    if (ray_direction.z() < 0.) {
        ray_direction *= -1.;
    }

    std::optional<RayPlaneIntersection> intersection;

    if (parameters_.use_pca) {
        Eigen::MatrixXd point_matrix(3, static_cast<Eigen::Index>(points_segmented.size()));
        for (size_t i = 0; i < points_segmented.size(); ++i) {
            point_matrix.col(static_cast<Eigen::Index>(i)) = points_segmented[i];
        }

        const PlanarityPca pca(point_matrix,
                               parameters_.pca_threshold_point_max,
                               parameters_.pca_threshold_line_min,
                               parameters_.pca_threshold_cubic_max);
        switch (pca.result()) {
        case PcaResult::point:
            return std::unexpected(DepthResultType::pca_is_point);
        case PcaResult::linear:
            return std::unexpected(DepthResultType::pca_is_line);
        case PcaResult::cubic:
            return std::unexpected(DepthResultType::pca_is_cubic);
        case PcaResult::plane:
            break;
        }

        intersection =
            line_plane_intersection_.intersect(pca.planeNormal(), pca.planeAnchorPoint(), ray_support, ray_direction);
    } else {
        const auto triangle = maxSpanningTriangle(points_segmented);
        if (!triangle.has_value()) {
            return std::unexpected(DepthResultType::triangle_not_planar_insufficient_points);
        }

        if (parameters_.check_triangle_planar &&
            !isTrianglePlanar(
                (*triangle)[0], (*triangle)[1], (*triangle)[2], parameters_.triangle_planar_cross_norm_threshold)) {
            return std::unexpected(DepthResultType::triangle_not_planar);
        }

        intersection = line_plane_intersection_.intersect(
            (*triangle)[0], (*triangle)[1], (*triangle)[2], ray_support, ray_direction);
    }

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
        const auto result = threshold_local_->check(points_segmented, depth);
        if (result == ThresholdResult::smaller_min) {
            return std::unexpected(DepthResultType::threshold_depth_local_smaller_min);
        }
        if (result == ThresholdResult::greater_max) {
            return std::unexpected(DepthResultType::threshold_depth_local_greater_max);
        }
    }

    if (depth < 0. && parameters_.discard_depth_behind_camera) {
        return std::unexpected(DepthResultType::corner_behind_camera);
    }

    return depth;
}

} // namespace mono_lidar
