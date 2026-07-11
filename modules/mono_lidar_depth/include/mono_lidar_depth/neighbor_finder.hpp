// Copyright 2017-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "types.hpp"

namespace mono_lidar {

/// Finds the projected lidar points inside a pixel rectangle around a feature point.
///
/// The visible lidar points are rasterized into a width x height index image once per frame
/// (initializeProjection); lookups then simply scan the search rectangle.
class PixelNeighborFinder {
public:
    PixelNeighborFinder(int img_width, int img_height, int pixel_search_width, int pixel_search_height);

    /// Rasterize the visible lidar points. If several points fall onto the same pixel, the first
    /// one in front of the camera wins.
    void initializeProjection(const ProjectedPointcloud& points);

    /// Indices (into the visible subset) of the lidar points inside the search rectangle around
    /// the feature point. The rectangle can be enlarged with the scale factors (used for the wider
    /// road search).
    std::vector<int> findNeighborIndices(const Eigen::Vector2d& feature_image_cs,
                                         float scale_width = 1.f,
                                         float scale_height = 1.f) const;

    /// 3d points (camera coordinates) for visible-point indices.
    static VecOfVec3d collectPoints(const ProjectedPointcloud& points, const std::vector<int>& visible_indices);

private:
    static constexpr int kNoPoint = -1;

    Eigen::MatrixXi pixel_to_visible_index_; ///< (width x height), visible-point index per pixel or kNoPoint

    int img_width_;
    int img_height_;
    int pixel_search_width_;
    int pixel_search_height_;
};

} // namespace mono_lidar
