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
class LandmarkSelectionSchemeCheirality : public LandmarkSelectionSchemeBase {
public: // public classes/enums/types etc...
public: // attributes
public: // public methods
    // default constructor
    LandmarkSelectionSchemeCheirality() = default;

    // default destructor
    virtual ~LandmarkSelectionSchemeCheirality() = default;

    // default move
    LandmarkSelectionSchemeCheirality(LandmarkSelectionSchemeCheirality&& other) = default;
    LandmarkSelectionSchemeCheirality& operator=(LandmarkSelectionSchemeCheirality&& other) = default;

    // default copy
    LandmarkSelectionSchemeCheirality(const LandmarkSelectionSchemeCheirality& other) = default;
    LandmarkSelectionSchemeCheirality& operator=(const LandmarkSelectionSchemeCheirality& other) = default;

    std::set<LandmarkId> getSelection(const LandmarkMap& landmarks, const KeyframeMap& keyframes) const override;

    //////////////////////////////////////////////////
    /// \brief createConst, factory function for const pointer
    /// \return
    ///
    static ConstPtr createConst();

    //////////////////////////////////////////////////
    /// \brief create, factory function
    /// \return
    ///
    static Ptr create();
};
}
