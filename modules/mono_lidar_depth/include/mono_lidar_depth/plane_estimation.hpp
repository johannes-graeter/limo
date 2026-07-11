// Copyright 2017-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <array>
#include <optional>

#include <Eigen/Eigen>

#include "types.hpp"

namespace mono_lidar {

/// Selects the three points spanning the (approximately) largest triangle: the two points with
/// the greatest mutual distance plus the point maximizing the summed distance to both. Point
/// pairs closer than min_distance are rejected.
std::optional<std::array<Eigen::Vector3d, 3>> maxSpanningTriangle(const VecOfVec3d& points,
                                                                  double min_distance = 0.);

/// Checks that the triangle spanned by three points is not degenerate (corners neither coincide
/// nor lie on a line): the cross-product norms of all normalized edge pairs must reach the
/// threshold (in [0, 1]).
bool isTrianglePlanar(const Eigen::Vector3d& corner1,
                      const Eigen::Vector3d& corner2,
                      const Eigen::Vector3d& corner3,
                      double threshold);

/// Least-squares plane fit through points (total least squares via SVD).
///
/// Replaces the former ceres-based ErrorPlane fit, whose residual dropped a parenthesis (only the
/// z term was divided by the normal's norm), read the distance from out-of-bounds ext[4], and
/// started from a zero normal (division by zero). The SVD fit is the closed-form optimum of the
/// same cost.
///
/// @return the plane, or std::nullopt for fewer than 3 points
std::optional<Eigen::Hyperplane<double, 3>> fitPlaneLeastSquares(const VecOfVec3d& points);

/// Weighted least-squares plane fit; points are weighted by the inverse of their distance to the
/// prior plane, so outliers relative to the prior are downweighted (M-estimator, one iteration).
///
/// @return the plane, or std::nullopt for fewer than 3 points
std::optional<Eigen::Hyperplane<double, 3>> fitPlaneMEstimator(const VecOfVec3d& points,
                                                               const Eigen::Hyperplane<double, 3>& prior);

} // namespace mono_lidar
