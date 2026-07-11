// Copyright 2017-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <optional>

#include <Eigen/Eigen>

#include "types.hpp"

namespace mono_lidar {

/// Intersection point of a viewing ray with a plane, plus the resulting depth (the z coordinate
/// of the intersection in camera coordinates).
struct RayPlaneIntersection {
    Eigen::Vector3d point;
    double depth;
};

/// Intersects camera viewing rays with planes. If an orthogonality threshold is set, rays that
/// are too parallel to the plane (|normal . ray| below the threshold) are rejected — those
/// intersections are numerically unstable and produce large depth errors.
class LinePlaneIntersection {
public:
    LinePlaneIntersection() = default;

    explicit LinePlaneIntersection(double orthogonality_threshold)
            : orthogonality_threshold_(orthogonality_threshold) {
    }

    std::optional<RayPlaneIntersection> intersect(const Eigen::Hyperplane<double, 3>& plane,
                                                  const Eigen::Vector3d& ray_support,
                                                  const Eigen::Vector3d& ray_direction) const;

    std::optional<RayPlaneIntersection> intersect(const Eigen::Vector3d& plane_normal,
                                                  const Eigen::Vector3d& plane_point,
                                                  const Eigen::Vector3d& ray_support,
                                                  const Eigen::Vector3d& ray_direction) const {
        return intersect(Eigen::Hyperplane<double, 3>(plane_normal, plane_point), ray_support, ray_direction);
    }

    /// Plane through three points.
    std::optional<RayPlaneIntersection> intersect(const Eigen::Vector3d& p1,
                                                  const Eigen::Vector3d& p2,
                                                  const Eigen::Vector3d& p3,
                                                  const Eigen::Vector3d& ray_support,
                                                  const Eigen::Vector3d& ray_direction) const {
        return intersect(Eigen::Hyperplane<double, 3>::Through(p1, p2, p3), ray_support, ray_direction);
    }

private:
    std::optional<double> orthogonality_threshold_;
};

/// Checks that a road patch extends sufficiently in driving direction: the z extent of the points
/// must be at least `threshold` times the x extent. A larger z/x ratio reduces the angular error
/// of the fitted road plane around the camera x axis.
bool checkZxRatio(const VecOfVec3d& points, double threshold);

} // namespace mono_lidar
