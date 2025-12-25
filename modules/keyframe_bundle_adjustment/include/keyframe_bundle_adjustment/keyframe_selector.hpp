// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

// standard includes
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "keyframe.hpp"
#include "keyframe_selection_schemes.hpp"


namespace keyframe_bundle_adjustment {

/**
*  @class KeyframeSelector
*  @par
*
*  Keyframe selector which executes different selection schemes one after the other
*/
class KeyframeSelector {
public: // public classes/enums/types etc...
    using Keyframes = std::set<Keyframe::Ptr>;

public: // public methods
    // default constructor
    KeyframeSelector() = default;

    // default destructor
    virtual ~KeyframeSelector() = default;

    // default move
    KeyframeSelector(KeyframeSelector&& other) = default;
    KeyframeSelector& operator=(KeyframeSelector&& other) = default;

    // default copy
    KeyframeSelector(const KeyframeSelector& other) = default;
    KeyframeSelector& operator=(const KeyframeSelector& other) = default;


    /**
     * @brief addScheme, add scheme to be used, overloaded for selection rejection and
     * sparsificatino
     * @param scheme
     */
    void addScheme(KeyframeSelectionSchemeBase::ConstPtr scheme);
    void addScheme(KeyframeRejectionSchemeBase::ConstPtr scheme);
    void addScheme(KeyframeSparsificationSchemeBase::ConstPtr scheme);

    /**
     * @brief select, select keyframes by comparing to last selected frames with SelectionSchemes
     * @param frames, reference to frames that shall be selected
     */
    Keyframes select(const Keyframes& frames, std::map<KeyframeId, Keyframe::Ptr> buffer_selected_frames);


public:                                                           // attributes
    std::vector<KeyframeSchemeBase::ConstPtr> selection_schemes_; ///< schemes that are
                                                                  /// used for keyframe
                                                                  /// selection(frames
                                                                  /// that are needed
                                                                  /// for stability)
    std::vector<KeyframeSchemeBase::ConstPtr>
        rejection_schemes_; ///< schemes for rejection of keyframes for stability ///selection, (so
    std::vector<KeyframeSchemeBase::ConstPtr>
        sparsification_schemes_; ///< schemes for sparsification of keyframes to reduce amount of
                                 /// redundant information ///selection, (so
};
}
