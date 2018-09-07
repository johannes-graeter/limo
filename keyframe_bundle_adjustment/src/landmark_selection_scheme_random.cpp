// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/landmark_selection_scheme_random.hpp"

namespace keyframe_bundle_adjustment {

std::set<LandmarkId> LandmarkSparsificationSchemeRandom::getSelection(const LandmarkMap& landmarks,
                                                                      const KeyframeMap& keyframes) const {

    // copy indices from landmarks
    std::vector<LandmarkId> lm_ids;
    lm_ids.resize(landmarks.size());
    std::transform(landmarks.cbegin(), landmarks.cend(), lm_ids.begin(), [](const auto& a) { return a.first; });

    // shuffle them and take first num_landmarks
    std::random_shuffle(lm_ids.begin(), lm_ids.end());

    std::set<LandmarkId> selected_landmark_ids;
    for (auto it = lm_ids.cbegin(); it != lm_ids.cend() && it != lm_ids.cbegin() + num_landmarks_; it++) {
        selected_landmark_ids.insert(*it);
    }

    return selected_landmark_ids;
}


LandmarkSparsificationSchemeBase::ConstPtr LandmarkSparsificationSchemeRandom::createConst(size_t num_landmarks) {

    return LandmarkSparsificationSchemeBase::ConstPtr(new LandmarkSparsificationSchemeRandom(num_landmarks));
}

LandmarkSparsificationSchemeBase::Ptr LandmarkSparsificationSchemeRandom::create(size_t num_landmarks) {

    return LandmarkSparsificationSchemeBase::Ptr(new LandmarkSparsificationSchemeRandom(num_landmarks));
}
}
