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
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "definitions.hpp"
#include "../keyframe.hpp"

namespace keyframe_bundle_adjustment {

/**
*  @class KeyframeSelectionSchemeBase
*  @par
*
*  Base class for keyframe selection schemes
*/
class LandmarkSelectionSchemeBase {
public: // public classes/enums/types etc...
    using Ptr = std::shared_ptr<LandmarkSelectionSchemeBase>;
    using ConstPtr = std::shared_ptr<const LandmarkSelectionSchemeBase>;
    using LandmarkMap = std::map<LandmarkId, Landmark::ConstPtr>;
    using KeyframeMap = std::map<KeyframeId, Keyframe::ConstPtr>;

public: // attributes
public: // public methods
    // default constructor
    LandmarkSelectionSchemeBase() = default;

    // default destructor
    virtual ~LandmarkSelectionSchemeBase() = default;

    // default move
    LandmarkSelectionSchemeBase(LandmarkSelectionSchemeBase&& other) = default;
    LandmarkSelectionSchemeBase& operator=(LandmarkSelectionSchemeBase&& other) = default;

    // default copy
    LandmarkSelectionSchemeBase(const LandmarkSelectionSchemeBase& other) = default;
    LandmarkSelectionSchemeBase& operator=(const LandmarkSelectionSchemeBase& other) = default;

    /**
     * @brief getSelection, abstract class for interface to schemes
     * @param landmarks, whole set of landmarks
     * @return selected set of landmarks
     */
    virtual std::set<LandmarkId> getSelection(const LandmarkMap& landmarks, const KeyframeMap& keyframes) const = 0;
};
}
