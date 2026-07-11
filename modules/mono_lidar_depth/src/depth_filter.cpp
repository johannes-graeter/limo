#include "mono_lidar_depth/depth_filter.hpp"

#include <algorithm>
#include <limits>

namespace mono_lidar {

namespace {

ThresholdResult applyBounds(ThresholdMode mode, double border_low, double border_high, double& depth) {
    if (depth < border_low) {
        if (mode == ThresholdMode::dispose) {
            depth = -1.;
            return ThresholdResult::smaller_min;
        }
        depth = border_low;
    } else if (depth > border_high) {
        if (mode == ThresholdMode::dispose) {
            depth = -1.;
            return ThresholdResult::greater_max;
        }
        depth = border_high;
    }
    return ThresholdResult::in_bounds;
}

} // namespace

ThresholdResult DepthThresholdGlobal::check(double& depth) const {
    return applyBounds(mode_, min_value_, max_value_, depth);
}

ThresholdResult DepthThresholdLocal::check(const VecOfVec3d& supporting_points, double& depth) const {
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    for (const auto& point : supporting_points) {
        min_z = std::min(min_z, point.z());
        max_z = std::max(max_z, point.z());
    }

    const double tolerance =
        tolerance_type_ == ThresholdToleranceType::relative ? (max_z - min_z) * tolerance_ : tolerance_;

    return applyBounds(mode_, min_z - tolerance, max_z + tolerance, depth);
}

} // namespace mono_lidar
