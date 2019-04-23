// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include "landmark_selection_scheme_base.hpp"

namespace keyframe_bundle_adjustment {

/**
*  @class Landmark selection scheme random
*  @par
*
* Concrete LandmarkSelectionScheme; select num_landmarks randomly
*/
class LandmarkSparsificationSchemeRandom : public LandmarkSparsificationSchemeBase {
public: // public classes/enums/types etc...
public: // public methods
    // default constructor
    LandmarkSparsificationSchemeRandom(size_t num_landmarks) : num_landmarks_(num_landmarks) {
        identifier = "random";
    }

    // default destructor
    virtual ~LandmarkSparsificationSchemeRandom() = default;

    // default move
    LandmarkSparsificationSchemeRandom(LandmarkSparsificationSchemeRandom&& other) = default;
    LandmarkSparsificationSchemeRandom& operator=(LandmarkSparsificationSchemeRandom&& other) = default;

    // default copy
    LandmarkSparsificationSchemeRandom(const LandmarkSparsificationSchemeRandom& other) = default;
    LandmarkSparsificationSchemeRandom& operator=(const LandmarkSparsificationSchemeRandom& other) = default;

    std::set<LandmarkId> getSelection(const LandmarkMap& landmarks, const KeyframeMap& keyframes) const override;

    //////////////////////////////////////////////////
    /// \brief createConst, factory function for const pointer
    /// \return
    ///
    static ConstPtr createConst(size_t num_landmarks);

    //////////////////////////////////////////////////
    /// \brief create, factory function
    /// \return
    ///
    static Ptr create(size_t num_landmarks);


public: // attributes
    size_t num_landmarks_;
};
}
