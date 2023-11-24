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
*  @class LandmarkSelectionSchemeCheirality
*  @par
*
*  Evaluates the cheirality constraint for each camera.
*  Cheirality is a very fancy word for the test if a 3d landmark could have been observed by the
* camera.
*  In this simple version this is equal to the test if the 3d landmark is behin the image plane or
* not
*/
class LandmarkRejectionSchemeCheirality : public LandmarkRejectionSchemeBase {
public: // public classes/enums/types etc...
public: // attributes
public: // public methods
    // default constructor
    LandmarkRejectionSchemeCheirality() {
        identifier = "cheirality";
    }

    // default destructor
    virtual ~LandmarkRejectionSchemeCheirality() = default;

    // default move
    LandmarkRejectionSchemeCheirality(LandmarkRejectionSchemeCheirality&& other) = default;
    LandmarkRejectionSchemeCheirality& operator=(LandmarkRejectionSchemeCheirality&& other) = default;

    // default copy
    LandmarkRejectionSchemeCheirality(const LandmarkRejectionSchemeCheirality& other) = default;
    LandmarkRejectionSchemeCheirality& operator=(const LandmarkRejectionSchemeCheirality& other) = default;

    std::set<LandmarkId> getSelection(const LandmarkMap& landmarks, const KeyframeMap& keyframes) const override;

    //////////////////////////////////////////////////
    /// \brief createConst, factory function for const pointer
    /// \return
    ///
    static LandmarkRejectionSchemeBase::ConstPtr createConst();

    //////////////////////////////////////////////////
    /// \brief create, factory function
    /// \return
    ///
    static LandmarkRejectionSchemeBase::Ptr create();
};
}
