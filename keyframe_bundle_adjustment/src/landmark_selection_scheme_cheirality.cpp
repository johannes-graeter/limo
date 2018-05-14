// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/landmark_selection_scheme_cheirality.hpp"

namespace keyframe_bundle_adjustment {

namespace {
/**
 * @brief is_landmark_cheiral, test if landmark fullfills cheirality constraint for all cameras on
 * all keyframes
 * @param keyframes
 * @param lm
 * @return
 */
bool is_landmark_cheiral(const LandmarkSelectionSchemeBase::KeyframeMap& keyframes,
                         const std::pair<LandmarkId, Landmark::ConstPtr>& lm) {
    for (const auto& id_kf : keyframes) {
        const auto& kf = id_kf.second;
        if (kf->is_active_) {
            const std::map<CameraId, Eigen::Vector3d> projected_lms_per_cam = kf->getProjectedLandmarkPosition(lm);
            // test cheirality for every projected landmark in cams
            for (const auto& cam_lm : projected_lms_per_cam) {
                if (cam_lm.second.z() < 0.) {
                    return false;
                }
            }
        }
    }
    return true;
}
}

std::set<LandmarkId> LandmarkSelectionSchemeCheirality::getSelection(
    const LandmarkSelectionSchemeBase::LandmarkMap& landmarks,
    const LandmarkSelectionSchemeBase::KeyframeMap& keyframes) const {

    std::set<LandmarkId> out;

    // transform landmarks into frames where they were observed
    // if all landmarks are in front of the camera, select landmark
    for (const auto& lm_el : landmarks) {
        if (is_landmark_cheiral(keyframes, lm_el)) {
            out.insert(lm_el.first);
        }
    }

    return out;
}

LandmarkSelectionSchemeBase::ConstPtr LandmarkSelectionSchemeCheirality::createConst() {
    return LandmarkSelectionSchemeBase::ConstPtr(new LandmarkSelectionSchemeCheirality());
}

LandmarkSelectionSchemeBase::Ptr LandmarkSelectionSchemeCheirality::create() {

    return LandmarkSelectionSchemeBase::Ptr(new LandmarkSelectionSchemeCheirality());
}
}
