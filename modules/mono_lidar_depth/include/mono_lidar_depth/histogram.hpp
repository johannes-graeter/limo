// Copyright 2016-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <optional>
#include <vector>

#include <Eigen/Eigen>

#include "types.hpp"

namespace mono_lidar {

/// Fixed-bin-width histogram over non-negative values, used to segment lidar neighbors by depth.
class DepthHistogram {
public:
    DepthHistogram(double bin_width, int bin_count);

    void addElement(double value);

    int binElementCount(int bin_index) const {
        return static_cast<int>(bins_.at(static_cast<size_t>(bin_index)).size());
    }

    int binCount() const {
        return static_cast<int>(bins_.size());
    }

private:
    double bin_width_;
    std::vector<std::vector<double>> bins_;
};

/// Result of segmentDepthBlob: the points of the nearest sufficiently populated depth blob.
struct DepthBlob {
    VecOfVec3d points;
    std::vector<int> point_indices;
    double lower_border{-1.};
    double higher_border{-1.};
};

/// Segments points by depth with a histogram and keeps the nearest local-maximum bin ("blob").
/// This separates the foreground object a feature sits on from background points inside the same
/// search rectangle.
///
/// @param points        neighbor points (camera coordinates)
/// @param point_indices indices belonging to points (same length), forwarded to the output
/// @param depths        depth of each point (same length)
/// @param bin_width     histogram bin size in meters
/// @param min_local_max_size minimum population for a bin to count as local maximum
/// @return the blob, or std::nullopt if no local maximum of sufficient size exists
std::optional<DepthBlob> segmentDepthBlob(const VecOfVec3d& points,
                                          const std::vector<int>& point_indices,
                                          const Eigen::VectorXd& depths,
                                          double bin_width,
                                          int min_local_max_size);

} // namespace mono_lidar
