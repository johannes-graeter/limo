// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/landmark_selection_scheme_add_depth.hpp"
#include <algorithm>
#include "internal/landmark_selection_scheme_helpers.hpp"

namespace keyframe_bundle_adjustment {


std::set<LandmarkId> LandmarkSelectionSchemeAddDepth::getSelection(
    const LandmarkSchemeBase::LandmarkMap& landmarks, const LandmarkSchemeBase::KeyframeMap& keyframes) const {
    // Define output.
    std::set<LandmarkId> out;

    // Put keyframes into vector and sort them.
    std::vector<Keyframe::ConstPtr> kf_ptrs_sorted_reverse = keyframe_helpers::getSortedKeyframes(keyframes);
    std::reverse(kf_ptrs_sorted_reverse.begin(), kf_ptrs_sorted_reverse.end());


    // For every Frame index assure num of lms with depth.
    for (const auto& el : params_.params_per_keyframe) {
        FrameIndex ind;
        NumberLandmarks num_lms_to_add;
        Comparator comparator;
        Sorter sorter;
        std::tie(ind, num_lms_to_add, comparator, sorter) = el;

        if (ind > static_cast<int>(kf_ptrs_sorted_reverse.size() - 1)) {
            continue;
        }

        // Get landmarks that fullfill the condition defined by comparator.
        std::vector<LandmarkId> ids;
        for (const auto& lm : kf_ptrs_sorted_reverse[ind]->measurements_) {
            if (landmarks.find(lm.first) != landmarks.cend() && comparator(landmarks.at(lm.first))) {
                ids.push_back(lm.first);
            }
        }

        // Get landmarks ids with cost for this keyframe.
        std::vector<std::pair<LandmarkId, double>> ids_sort_val;
        ids_sort_val.reserve(ids.size());
        for (const auto& id : ids) {
            Eigen::Vector3d local_lm =
                kf_ptrs_sorted_reverse[ind]->getEigenPose() * LmMap(landmarks.at(id)->pos.data());
            std::vector<double> cost;
            for (const auto& cam_meas : kf_ptrs_sorted_reverse[ind]->measurements_.at(id)) {
                cost.push_back(sorter(cam_meas.second, local_lm));
            }
            auto max_it =
                std::max_element(cost.cbegin(), cost.cend(), [](const auto& a, const auto& b) { return a < b; });
            ids_sort_val.push_back(std::make_pair(id, *max_it));
        }

        // Add lms with lowest cost to selection.
        int incr = std::min(num_lms_to_add, static_cast<int>(ids_sort_val.size()));
        std::partial_sort(ids_sort_val.begin(),
                          std::next(ids_sort_val.begin(), incr),
                          ids_sort_val.end(),
                          [](const auto& a, const auto& b) { return a.second < b.second; });

        // Insert them in selection.
        std::transform(ids_sort_val.begin(),
                       std::next(ids_sort_val.begin(), incr),
                       std::inserter(out, out.begin()),
                       [](const auto& a) { return a.first; });
    }

    return out;
}

LandmarkSelectionSchemeBase::ConstPtr LandmarkSelectionSchemeAddDepth::createConst(
    LandmarkSelectionSchemeAddDepth::Parameters p) {
    return LandmarkSelectionSchemeBase::ConstPtr(new LandmarkSelectionSchemeAddDepth(p));
}

LandmarkSelectionSchemeBase::Ptr LandmarkSelectionSchemeAddDepth::create(
    LandmarkSelectionSchemeAddDepth::Parameters p) {
    return LandmarkSelectionSchemeBase::Ptr(new LandmarkSelectionSchemeAddDepth(p));
}
}
