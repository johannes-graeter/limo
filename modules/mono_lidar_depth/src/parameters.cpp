#include "mono_lidar_depth/parameters.hpp"

#include <ostream>

namespace mono_lidar {

namespace {

constexpr std::string_view toString(RoadPlaneFitMode mode) {
    switch (mode) {
    case RoadPlaneFitMode::none:
        return "none";
    case RoadPlaneFitMode::max_spanning_triangle:
        return "max_spanning_triangle";
    case RoadPlaneFitMode::least_squares:
        return "least_squares";
    case RoadPlaneFitMode::m_estimator:
        return "m_estimator";
    }
    return "unknown";
}

} // namespace

std::ostream& operator<<(std::ostream& os, const DepthEstimatorParameters& p) {
    os << "--- depth estimator parameters ---\n";
    os << "pixel_search_width: " << p.pixel_search_width << "\n";
    os << "pixel_search_height: " << p.pixel_search_height << "\n";
    os << "neighbor_count_min: " << p.neighbor_count_min << "\n";
    os << "use_histogram_segmentation: " << p.use_histogram_segmentation << "\n";
    os << "histogram_bin_width: " << p.histogram_bin_width << "\n";
    os << "histogram_min_pointcount: " << p.histogram_min_pointcount << "\n";
    os << "threshold_depth_enabled: " << p.threshold_depth_enabled << "\n";
    os << "threshold_depth_min: " << p.threshold_depth_min << "\n";
    os << "threshold_depth_max: " << p.threshold_depth_max << "\n";
    os << "threshold_depth_local_enabled: " << p.threshold_depth_local_enabled << "\n";
    os << "threshold_depth_local_tolerance: " << p.threshold_depth_local_tolerance << "\n";
    os << "use_pca: " << p.use_pca << "\n";
    os << "check_triangle_planar: " << p.check_triangle_planar << "\n";
    os << "triangle_planar_cross_norm_threshold: " << p.triangle_planar_cross_norm_threshold << "\n";
    os << "viewray_plane_orthogonality_threshold: " << p.viewray_plane_orthogonality_threshold << "\n";
    os << "use_ransac_plane: " << p.use_ransac_plane << "\n";
    os << "ransac_plane_distance_threshold: " << p.ransac_plane_distance_threshold << "\n";
    os << "ransac_plane_max_iterations: " << p.ransac_plane_max_iterations << "\n";
    os << "road_plane_fit_mode: " << toString(p.road_plane_fit_mode) << "\n";
    return os;
}

} // namespace mono_lidar
