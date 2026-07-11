#include "mono_lidar_depth/statistics.hpp"

#include <ostream>

namespace mono_lidar {

void DepthCalculationStatistics::startFrame() {
    frame_counts_.clear();
    frame_total_ = 0;
}

void DepthCalculationStatistics::add(DepthResultType result) {
    ++frame_counts_[result];
    ++frame_total_;
    ++accumulated_counts_[result];
    ++accumulated_total_;
}

int DepthCalculationStatistics::frameCount(DepthResultType result) const {
    const auto it = frame_counts_.find(result);
    return it != frame_counts_.end() ? it->second : 0;
}

int DepthCalculationStatistics::accumulatedCount(DepthResultType result) const {
    const auto it = accumulated_counts_.find(result);
    return it != accumulated_counts_.end() ? it->second : 0;
}

std::ostream& operator<<(std::ostream& os, const DepthCalculationStatistics& s) {
    os << "--- depth calculation statistics (frame / accumulated) ---\n";
    os << "total: " << s.frame_total_ << " / " << s.accumulated_total_ << "\n";
    for (const auto& [result, accumulated] : s.accumulated_counts_) {
        os << toString(result) << ": " << s.frameCount(result) << " / " << accumulated << "\n";
    }
    return os;
}

} // namespace mono_lidar
