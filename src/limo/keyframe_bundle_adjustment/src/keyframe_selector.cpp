// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "keyframe_selector.hpp"
#include <chrono>

namespace keyframe_bundle_adjustment {

void KeyframeSelector::addScheme(KeyframeSelectionSchemeBase::ConstPtr scheme) {
    selection_schemes_.push_back(static_cast<KeyframeSchemeBase::ConstPtr>(scheme));
}

void KeyframeSelector::addScheme(KeyframeRejectionSchemeBase::ConstPtr scheme) {
    rejection_schemes_.push_back(static_cast<KeyframeSchemeBase::ConstPtr>(scheme));
}

void KeyframeSelector::addScheme(KeyframeSparsificationSchemeBase::ConstPtr scheme) {
    sparsification_schemes_.push_back(static_cast<KeyframeSchemeBase::ConstPtr>(scheme));
}

namespace {
/**
 * @brief applyRejectionScheme, apply and keyframeScheme to reject or sparsify keyframes
 * @param selected_keyframe, output which is written here
 * @param id, id of the last selected keyframe
 * @param schemes, schemes to be applied
 */
std::map<KeyframeId, Keyframe::Ptr> applyRejectionScheme(const KeyframeSelector::Keyframes& frames,
                                                         std::map<KeyframeId, Keyframe::Ptr> buffer_selected_frames,
                                                         const std::vector<KeyframeSchemeBase::ConstPtr>& schemes) {

    std::map<KeyframeId, Keyframe::Ptr> selected_keyframes;
    KeyframeId id = 0;
    for (const auto& frame : frames) {
        bool is_rejected = false;
        for (const auto& scheme : schemes) {
            // test schemes until frame is usable
            if (!scheme->isUsable(frame, buffer_selected_frames) || !scheme->isUsable(frame, selected_keyframes)) {
                is_rejected = true;
                break;
            }
        }

        if (!is_rejected) {
            selected_keyframes[id] = frame;
            id++;
        }
    }

    return selected_keyframes;
}

/**
 * @brief applySelectionScheme, apply and keyframeScheme to select keyframes
 * @todo this is not so nice, how to assure that only selection schemes are put here?
 * assert with std::inherits_from would be an option
 * @param selected_keyframe, output which is written here
 * @param id, id of the last selected keyframe
 * @param schemes, schemes to be applied
 */
std::map<KeyframeId, Keyframe::Ptr> applySelectionScheme(const KeyframeSelector::Keyframes& frames,
                                                         std::map<KeyframeId, Keyframe::Ptr> buffer_selected_frames,
                                                         const std::vector<KeyframeSchemeBase::ConstPtr>& schemes) {

    std::map<KeyframeId, Keyframe::Ptr> selected_keyframes;
    KeyframeId id = 0;
    for (const auto& frame : frames) {
        for (const auto& scheme : schemes) {
            // test schemes until frame is usable
            if (scheme->isUsable(frame, buffer_selected_frames) || scheme->isUsable(frame, selected_keyframes)) {
                selected_keyframes[id] = frame;
                id++;
                break;
            }
        }
    }

    return selected_keyframes;
}
void eraseRejected(std::map<KeyframeId, Keyframe::Ptr>& cur_keyframes,
                   const std::map<KeyframeId, Keyframe::Ptr>& non_rejected_keyframes) {
    // no non rejected means no frame can be valid
    if (non_rejected_keyframes.size() == 0) {
        cur_keyframes = std::map<KeyframeId, Keyframe::Ptr>();
    }
    // is this test necessary?
    if (cur_keyframes.size() == 0) {
        return;
    }

    // erase keyframes if they were rejected
    auto it = cur_keyframes.begin();
    for (; it != cur_keyframes.end(); it++) {
        // if keyframe is rejected erase it
        if (non_rejected_keyframes.find(it->first) == non_rejected_keyframes.cend()) {
            it = cur_keyframes.erase(it);
        }
    }
}
}

KeyframeSelector::Keyframes KeyframeSelector::select(const Keyframes& frames,
                                                     std::map<KeyframeId, Keyframe::Ptr> buffer_selected_frames) {
    using KeyframeMap = std::map<KeyframeId, Keyframe::Ptr>;

    // reject all frames that we can not have due to stability
    KeyframeMap non_rejected_keyframes = applyRejectionScheme(frames, buffer_selected_frames, rejection_schemes_);

    // select all frames that we need for stability
    KeyframeMap selected_keyframes = applySelectionScheme(frames, buffer_selected_frames, selection_schemes_);
    eraseRejected(selected_keyframes, non_rejected_keyframes);

    // the ones
    // sparsify frames that we do not essentially need
    KeyframeMap sparsified_keyframes = applyRejectionScheme(frames, buffer_selected_frames, sparsification_schemes_);
    eraseRejected(sparsified_keyframes, non_rejected_keyframes);

    // copy map to et
    Keyframes out;
    for (const auto& el : selected_keyframes) {
        out.insert(el.second);
    }
    for (const auto& el : sparsified_keyframes) {
        out.insert(el.second);
    }

    return out;
}
}
