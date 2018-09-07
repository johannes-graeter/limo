// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/keyframe_selection_scheme_pose.hpp"

namespace keyframe_bundle_adjustment {

KeyframeSelectionSchemePose::KeyframeSelectionSchemePose(double critical_quaternion_difference)
        : critical_quaternion_diff_(critical_quaternion_difference) {
    ;
}

bool KeyframeSelectionSchemePose::isUsable(const Keyframe::Ptr& new_frame,
                                           const std::map<KeyframeId, Keyframe::Ptr>& last_frames) const {

    // nothing to test return false, otherwise empty ones would always be taken
    if (last_frames.size() == 0) {
        return false;
    }
    // get frame with last timestamp
    auto it = std::max_element(last_frames.cbegin(), last_frames.cend(), [](const auto& a, const auto& b) {
        return a.second->timestamp_ < b.second->timestamp_;
    });
    const Keyframe::Ptr& last_keyframe = it->second;

    // get quaternion distance
    double quat_diff = calcQuaternionDiff(new_frame->pose_, last_keyframe->pose_);
    std::cout << "quaternion diff=" << quat_diff << std::endl;

    // use if quaternion difference is bigger than threshold
    return quat_diff > critical_quaternion_diff_;
}

KeyframeSelectionSchemeBase::ConstPtr KeyframeSelectionSchemePose::createConst(double critical_quaternion_difference) {
    return KeyframeSelectionSchemeBase::ConstPtr(new KeyframeSelectionSchemePose(critical_quaternion_difference));
}

KeyframeSelectionSchemeBase::Ptr KeyframeSelectionSchemePose::create(double critical_quaternion_difference) {
    return KeyframeSelectionSchemeBase::Ptr(new KeyframeSelectionSchemePose(critical_quaternion_difference));
}
}
