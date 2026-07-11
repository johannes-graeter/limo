// Copyright 2017-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <iosfwd>
#include <map>

#include "types.hpp"

namespace mono_lidar {

/// Per-frame and accumulated counts of depth-calculation outcomes.
class DepthCalculationStatistics {
public:
    void startFrame();

    void add(DepthResultType result);

    int frameCount(DepthResultType result) const;
    int frameTotal() const {
        return frame_total_;
    }

    int accumulatedCount(DepthResultType result) const;
    int accumulatedTotal() const {
        return accumulated_total_;
    }

    friend std::ostream& operator<<(std::ostream& os, const DepthCalculationStatistics& s);

private:
    std::map<DepthResultType, int> frame_counts_;
    std::map<DepthResultType, int> accumulated_counts_;
    int frame_total_{0};
    int accumulated_total_{0};
};

} // namespace mono_lidar
