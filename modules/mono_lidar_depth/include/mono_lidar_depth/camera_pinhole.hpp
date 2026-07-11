// Copyright 2018-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <memory>

#include <Eigen/Eigen>

namespace mono_lidar {

/// Pinhole camera with a single focal length (single-viewpoint, undistorted images).
class CameraPinhole final {
public:
    using Ptr = std::shared_ptr<CameraPinhole>;
    using ConstPtr = std::shared_ptr<const CameraPinhole>;

    CameraPinhole(int width, int height, double focal_length, double principal_point_x, double principal_point_y)
            : width_(width), height_(height), focal_length_(focal_length), principal_point_x_(principal_point_x),
              principal_point_y_(principal_point_y) {
    }

    int width() const {
        return width_;
    }
    int height() const {
        return height_;
    }

    Eigen::Matrix3d intrinsics() const {
        Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
        intrinsics(0, 0) = intrinsics(1, 1) = focal_length_;
        intrinsics(0, 2) = principal_point_x_;
        intrinsics(1, 2) = principal_point_y_;
        return intrinsics;
    }

    /// Viewing rays through the given image points: support point (camera center, always the
    /// origin for a single-viewpoint camera) plus normalized direction.
    template <int N>
    void getViewingRays(const Eigen::Matrix<double, 2, N>& image_points,
                        Eigen::Matrix<double, 3, N>& support_points,
                        Eigen::Matrix<double, 3, N>& directions) const {
        directions = intrinsics().inverse() * image_points.colwise().homogeneous();
        directions = directions.colwise().normalized();
        support_points.setZero(3, directions.cols());
    }

    /// Project camera-frame points to the image plane. Returns per-point whether the projection
    /// lies inside the image bounds.
    template <int N>
    Eigen::Array<bool, 1, N> getImagePoints(const Eigen::Matrix<double, 3, N>& points3d,
                                            Eigen::Matrix<double, 2, N>& image_points) const {
        Eigen::Matrix<double, 3, N> image_points_3d = intrinsics() * points3d;
        image_points = image_points_3d.colwise().hnormalized();

        Eigen::Array<bool, 1, N> is_in_bounds =
            (image_points.row(0).array() >= 0.) && (image_points.row(0).array() <= static_cast<double>(width_)) &&
            (image_points.row(1).array() >= 0.) && (image_points.row(1).array() <= static_cast<double>(height_));
        return is_in_bounds;
    }

private:
    int width_;
    int height_;
    double focal_length_;
    double principal_point_x_;
    double principal_point_y_;
};

} // namespace mono_lidar
