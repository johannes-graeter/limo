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
*  @class LandmarkSelectionSchemeAddWDepth
*  @par
*
*  Through all the seelction algorithms it can happen, that we don not have enough depth measurements.
*  This class assures that we have enough depth measruements in the selection and adds ids of landmarks
*  even though they have been removed before.
*/
class LandmarkSelectionSchemeAddDepth : public LandmarkSelectionSchemeBase {
public:                     // public classes/enums/types etc...
    using FrameIndex = int; ///< Index of frame, 0 means first frame.
    struct Parameters {
        std::map<FrameIndex, int> num_depth_meas{
            {0, 20}}; ///< Determines how many depth points are assured on what keyframes.
    };

public: // public methods
    // default constructor
    LandmarkSelectionSchemeAddDepth(Parameters p) : params_(p) {
        ;
    }

    // default destructor
    ~LandmarkSelectionSchemeAddDepth() = default;

    // default move
    LandmarkSelectionSchemeAddDepth(LandmarkSelectionSchemeAddDepth&& other) = default;
    LandmarkSelectionSchemeAddDepth& operator=(LandmarkSelectionSchemeAddDepth&& other) = default;

    // default copy
    LandmarkSelectionSchemeAddDepth(const LandmarkSelectionSchemeAddDepth& other) = default;
    LandmarkSelectionSchemeAddDepth& operator=(const LandmarkSelectionSchemeAddDepth& other) = default;

    std::set<LandmarkId> getSelection(const LandmarkMap& landmarks, const KeyframeMap& keyframes) const override;

    //////////////////////////////////////////////////
    /// \brief createConst, factory function for const pointer
    /// \return
    ///
    static ConstPtr createConst(Parameters p);

    //////////////////////////////////////////////////
    /// \brief create, factory function
    /// \return
    ///
    static Ptr create(Parameters p);


public:                 // attributes
    Parameters params_; ///< paramters
};
}
