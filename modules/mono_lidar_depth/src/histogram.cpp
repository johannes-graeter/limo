#include "mono_lidar_depth/histogram.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mono_lidar {

DepthHistogram::DepthHistogram(double bin_width, int bin_count) : bin_width_(bin_width) {
    if (bin_width <= 0. || bin_count <= 0) {
        throw std::invalid_argument("DepthHistogram: bin_width and bin_count must be positive");
    }
    bins_.resize(static_cast<size_t>(bin_count));
}

void DepthHistogram::addElement(double value) {
    // Very big values go into the last bin (guards against numerical instabilities).
    value = std::min(value, 1e10);
    const auto bin_index =
        static_cast<size_t>(std::min(std::abs(value / bin_width_), static_cast<double>(bins_.size()) - 1.));
    bins_[bin_index].push_back(value);
}

std::optional<DepthBlob> segmentDepthBlob(const VecOfVec3d& points,
                                          const std::vector<int>& point_indices,
                                          const Eigen::VectorXd& depths,
                                          double bin_width,
                                          int min_local_max_size) {
    if (static_cast<Eigen::Index>(points.size()) != depths.rows() || points.size() != point_indices.size()) {
        throw std::invalid_argument("segmentDepthBlob: points, point_indices and depths must have the same size");
    }

    double max_depth = 0.;
    for (Eigen::Index i = 0; i < depths.rows(); ++i) {
        max_depth = std::max(max_depth, std::ceil(depths(i)));
    }
    const int bin_count = static_cast<int>(max_depth / bin_width) + 1;
    if (bin_count <= 1) {
        return std::nullopt;
    }

    DepthHistogram histogram(bin_width, bin_count);
    for (Eigen::Index i = 0; i < depths.rows(); ++i) {
        histogram.addElement(depths(i));
    }

    // Walk the bins from near to far and take the first local maximum of sufficient size. An
    // empty bin directly after a populated one that was below the minimum size means an isolated
    // speckle in front of the object — give up in that case.
    int bin_max_id = -1;
    int bin_max_count = -1;
    int bin_value = 0;
    for (int i = 0; i < bin_count; ++i) {
        const int last_bin_value = bin_value;
        bin_value = histogram.binElementCount(i);

        if (bin_value > bin_max_count && bin_value >= min_local_max_size) {
            bin_max_count = bin_value;
            bin_max_id = i;
        } else if (bin_value < bin_max_count) {
            break; // first local maximum found
        }

        if (last_bin_value > 0 && bin_value == 0) {
            return std::nullopt;
        }
    }

    if (bin_max_id < 0) {
        return std::nullopt;
    }

    DepthBlob blob;
    blob.lower_border = bin_max_id * bin_width;
    blob.higher_border = (bin_max_id + 1) * bin_width;

    for (size_t i = 0; i < points.size(); ++i) {
        const double depth = depths(static_cast<Eigen::Index>(i));
        if (depth >= blob.lower_border && depth < blob.higher_border) {
            blob.points.push_back(points[i]);
            blob.point_indices.push_back(point_indices[i]);
        }
    }

    return blob;
}

} // namespace mono_lidar
