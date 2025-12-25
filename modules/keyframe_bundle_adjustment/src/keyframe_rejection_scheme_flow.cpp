// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/keyframe_rejection_scheme_flow.hpp"

namespace keyframe_bundle_adjustment {
KeyframeRejectionSchemeFlow::KeyframeRejectionSchemeFlow(double min_median_flow)
        : min_median_flow_squared_(min_median_flow * min_median_flow) {
    ;
}

bool KeyframeRejectionSchemeFlow::isUsable(const Keyframe::Ptr& new_frame,
                                           const std::map<KeyframeId, Keyframe::Ptr>& last_frames) const {
    // nothing to test return true
    if (last_frames.size() == 0) {
        return true;
    }
    // no measurements, don't use
    if (new_frame->measurements_.size() == 0) {
        return false;
    }
    // get measurements of last keyframe
    auto it = std::max_element(last_frames.cbegin(), last_frames.cend(), [](const auto& a, const auto& b) {
        return a.second->timestamp_ < b.second->timestamp_;
    });
    const Keyframe::Ptr& last_keyframe = it->second;

    // calculate flow for all cameras
    std::vector<double> flow_squared;
    flow_squared.reserve(new_frame->measurements_.size());

    for (const auto& m : new_frame->measurements_) {
        const auto& lm_id = m.first;
        const auto& map_cam_meas = m.second;

        for (const auto& cam_id_meas : map_cam_meas) {
            if (last_keyframe->hasMeasurement(lm_id, cam_id_meas.first)) {
                const Measurement& last_meas = last_keyframe->getMeasurement(lm_id, cam_id_meas.first);
                Eigen::Vector2d diff = cam_id_meas.second.toEigen2d() - last_meas.toEigen2d();
                flow_squared.push_back(diff.squaredNorm());
            }
        }
    }

    //    // get median flow
    //    std::nth_element(flow_squared.begin(),
    //                     flow_squared.begin() + flow_squared.size() / 2,
    //                     flow_squared.end(),
    //                     [](const auto& a, const auto& b) { return a < b; });

    //    double median_flow = flow_squared[flow_squared.size() / 2];

    // get mean flow
    double sum = 0.;
    for (const auto& el : flow_squared) {
        sum += std::sqrt(el);
    }
    sum /= static_cast<double>(flow_squared.size());
    // This is not median but mean,
    ///@todo change names.
    double median_flow = sum * sum;

    if (median_flow < min_median_flow_squared_) {
        std::cout << "---- Not enough flow, reject frame." << std::endl;
    }

    // accept if median flow is bigger than threshold
    return median_flow > min_median_flow_squared_;
}

KeyframeRejectionSchemeBase::ConstPtr KeyframeRejectionSchemeFlow::createConst(double min_median_flow) {
    return KeyframeRejectionSchemeBase::ConstPtr(new KeyframeRejectionSchemeFlow(min_median_flow));
}

KeyframeRejectionSchemeBase::Ptr KeyframeRejectionSchemeFlow::create(double min_median_flow) {
    return KeyframeRejectionSchemeBase::Ptr(new KeyframeRejectionSchemeFlow(min_median_flow));
}
}
