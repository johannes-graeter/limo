// Copyright 2017-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <iosfwd>

#include "types.hpp"

namespace mono_lidar {

/// How the plane for road (ground) features is fitted from the neighboring lidar points.
enum class RoadPlaneFitMode {
    none,               ///< no special treatment of road features
    max_spanning_triangle,
    least_squares,
    m_estimator,        ///< weighted least squares with the RANSAC plane as prior (default)
};

/// Parameters of the depth estimation, defaults tuned for KITTI-like setups (HDL-64, ~0.4°
/// vertical resolution at 1242x375 px). All lengths are in meters.
struct DepthEstimatorParameters {
    // Neighbor search: pixel rect around the feature point in which projected lidar points count
    // as neighbor candidates.
    int pixel_search_width{12};
    int pixel_search_height{15};
    int neighbor_count_min{3}; ///< minimum number of neighbors to attempt depth estimation

    // Histogram segmentation: neighbors are binned by depth, the nearest sufficiently populated
    // blob is kept to separate foreground from background.
    bool use_histogram_segmentation{true};
    double histogram_bin_width{0.5};
    int histogram_min_pointcount{3};

    // Global depth thresholds applied to every estimate.
    bool threshold_depth_enabled{true};
    ThresholdMode threshold_depth_mode{ThresholdMode::dispose};
    double threshold_depth_min{0.};
    double threshold_depth_max{100.};

    // Local depth threshold: the estimate must stay inside the depth interval spanned by its
    // supporting points, extended by a tolerance.
    bool threshold_depth_local_enabled{true};
    ThresholdMode threshold_depth_local_mode{ThresholdMode::dispose};
    ThresholdToleranceType threshold_depth_local_tolerance_type{ThresholdToleranceType::relative};
    double threshold_depth_local_tolerance{0.5};

    // Local patch reconstruction: either a maximum spanning triangle through the segmented
    // neighbors, or a PCA plane fit through all of them.
    bool use_pca{false};
    double pca_threshold_point_max{0.005}; ///< largest eigenvalue below -> degenerate point
    double pca_threshold_line_min{15.};    ///< largest/middle eigenvalue ratio above -> degenerate line
    double pca_threshold_cubic_max{0.5};   ///< planarity below -> not planar enough
    bool check_triangle_planar{true};
    double triangle_planar_cross_norm_threshold{0.1};
    double viewray_plane_orthogonality_threshold{0.1};
    bool discard_depth_behind_camera{true};

    // RANSAC ground plane estimation (used for the road-feature fallback).
    bool use_ransac_plane{true};
    double ransac_plane_distance_threshold{0.2};
    double ransac_plane_min_z{-10000.};
    double ransac_plane_max_z{10000.};
    int ransac_plane_max_iterations{10000};
    double ransac_plane_probability{0.999};
    bool ransac_plane_use_refinement{true};
    double ransac_plane_refinement_threshold{10.2};
    double ransac_plane_point_distance_threshold{0.2};
    int ransac_plane_sample_points{6000}; ///< random subsample size before RANSAC (<=100 disables)

    // Road-feature depth estimation.
    RoadPlaneFitMode road_plane_fit_mode{RoadPlaneFitMode::m_estimator};
    double road_triangle_min_z_x_relation{0.}; ///< min z/x extent ratio of the road patch

    bool collect_statistics{true};
};

std::ostream& operator<<(std::ostream& os, const DepthEstimatorParameters& p);

} // namespace mono_lidar
