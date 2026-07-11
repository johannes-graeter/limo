// Copyright 2017-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <Eigen/Eigen>

namespace mono_lidar {

enum class PcaResult { point, linear, plane, cubic };

/// Principal component analysis of a local point cloud patch, used to decide whether the patch is
/// (approximately) a plane and to extract that plane.
class PlanarityPca {
public:
    /// @param points one 3d point per column
    /// @param threshold_point_max   patch degenerates to a point if the largest eigenvalue is below
    /// @param threshold_line_min    patch degenerates to a line if largest/middle eigenvalue exceeds this
    /// @param threshold_cubic_max   patch is volumetric (not planar) if the planarity is below
    PlanarityPca(const Eigen::MatrixXd& points,
                 double threshold_point_max,
                 double threshold_line_min,
                 double threshold_cubic_max);

    PcaResult result() const {
        return result_;
    }

    /// Anchor point of the fitted plane (the centroid).
    const Eigen::Vector3d& planeAnchorPoint() const {
        return mean_;
    }

    /// Normal of the fitted plane (eigenvector of the smallest eigenvalue).
    const Eigen::Vector3d& planeNormal() const {
        return plane_normal_;
    }

private:
    Eigen::Vector3d mean_;
    Eigen::Vector3d plane_normal_;
    PcaResult result_;
};

} // namespace mono_lidar
