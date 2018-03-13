// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/landmark_selection_scheme_add_depth.hpp"
#include <algorithm>

namespace keyframe_bundle_adjustment {

std::set<LandmarkId> LandmarkSelectionSchemeAddDepth::getSelection(
    const LandmarkSelectionSchemeBase::LandmarkMap& landmarks,
    const LandmarkSelectionSchemeBase::KeyframeMap& keyframes) const {

    // This function makes number of landmarks only bigger.
    // First add already existing landmarks.
    std::set<LandmarkId> out;
    for (const auto& el : landmarks) {
        out.insert(el.first);
    }

    // Put keyframes into vector and sort them
    std::vector<Keyframe::ConstPtr> kf_ptrs_sorted;
    kf_ptrs_sorted.reserve(keyframes.size());

    for (const auto& kf : keyframes) {
        // only use active keyframes
        if (kf.second->is_active_) {
            kf_ptrs_sorted.push_back(kf.second);
        }
    }

    // Newest keyframe is first.
    std::sort(kf_ptrs_sorted.begin(), kf_ptrs_sorted.end(), [](const auto& a, const auto& b) {
        return a->timestamp_ > b->timestamp_;
    });

    // For every Frame index assure num of lms with depth.
    for (const auto& frame_index_num_lms : params_.num_depth_meas) {
        const auto& cur_kf_ptr = kf_ptrs_sorted[frame_index_num_lms.first];
        // Get landmarks ids with valid depth for this keyframe.
        std::map<LandmarkId, double> ids_depth;
        for (const auto& lm : cur_kf_ptr->measurements_) {
            for (const auto& cam_meas : lm.second) {
                if (cam_meas.second.d > 0.) {
                    ids_depth[lm.first] = cam_meas.second.d;
                }
            }
        }

        // Get landmark ids with depth that are not in landmark selection.
        std::vector<std::pair<LandmarkId, double>> intersection;
        std::set_difference(ids_depth.cbegin(),
                            ids_depth.cend(),
                            landmarks.cbegin(),
                            landmarks.cend(),
                            std::back_inserter(intersection),
                            [](const auto& a, const auto& b) { return a.first < b.first; });

        // Get num of lms to add.
        int num_lms_id_inselection = ids_depth.size() - intersection.size();
        int num_lms_to_add = frame_index_num_lms.second - num_lms_id_inselection;

        std::cout << "LandmarkSelectionSchemeAddDepth: add " << num_lms_to_add << " lms" << std::endl;

        if (num_lms_to_add > 0) {
            // Get element until whcih vector shall be sorted.
            int incr = std::min(num_lms_to_add, static_cast<int>(intersection.size()));
            auto it = std::next(intersection.begin(), incr);
            // Add nearest Lms to output.
            // Get n nearest values.
            std::partial_sort(intersection.begin(), it, intersection.end(), [](const auto& a, const auto& b) {
                return a.second < b.second;
            });
            // insert them to ouptut
            std::transform(
                intersection.begin(), it, std::inserter(out, out.begin()), [](const auto& a) { return a.first; });
        }
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
