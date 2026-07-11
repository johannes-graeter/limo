// Copyright 2017-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include "types.hpp"

namespace mono_lidar {

/// Validates a depth estimate against fixed global bounds.
///
/// mode == dispose: an out-of-bounds depth is set to -1 and reported.
/// mode == adjust:  an out-of-bounds depth is clamped to the violated bound.
class DepthThresholdGlobal {
public:
    DepthThresholdGlobal(ThresholdMode mode, double min_value, double max_value)
            : mode_(mode), min_value_(min_value), max_value_(max_value) {
    }

    ThresholdResult check(double& depth) const;

private:
    ThresholdMode mode_;
    double min_value_;
    double max_value_;
};

/// Validates a depth estimate against the depth interval spanned by its supporting points,
/// extended by a tolerance (absolute in meters, or relative to the interval size).
class DepthThresholdLocal {
public:
    DepthThresholdLocal(ThresholdMode mode, ThresholdToleranceType tolerance_type, double tolerance)
            : mode_(mode), tolerance_type_(tolerance_type), tolerance_(tolerance) {
    }

    ThresholdResult check(const VecOfVec3d& supporting_points, double& depth) const;

private:
    ThresholdMode mode_;
    ThresholdToleranceType tolerance_type_;
    double tolerance_;
};

} // namespace mono_lidar
