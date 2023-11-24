// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/keyframe_sparsification_scheme_time.hpp"

namespace keyframe_bundle_adjustment {

bool KeyframeSparsificationSchemeTime::isUsable(const Keyframe::Ptr& new_frame,
                                                const std::map<KeyframeId, Keyframe::Ptr>& last_frames) const {
    // nothing to test -> return true
    if (last_frames.size() == 0) {
        return true;
    }

    // first have look at the time diff between last keyframes and new ones
    auto it = std::max_element(last_frames.cbegin(), last_frames.cend(), [](const auto& a, const auto& b) {
        return a.second->timestamp_ < b.second->timestamp_;
    });
    const TimestampNSec max_ts = it->second->timestamp_;

    return (new_frame->timestamp_ - max_ts) > time_difference_nano_sec_;
}

KeyframeSparsificationSchemeBase::ConstPtr KeyframeSparsificationSchemeTime::createConst(
    double time_difference_nano_sec_) {
    return KeyframeSparsificationSchemeBase::ConstPtr(new KeyframeSparsificationSchemeTime(time_difference_nano_sec_));
}

KeyframeSparsificationSchemeBase::Ptr KeyframeSparsificationSchemeTime::create(double time_difference_nano_sec_) {
    return KeyframeSparsificationSchemeBase::Ptr(new KeyframeSparsificationSchemeTime(time_difference_nano_sec_));
}
}
