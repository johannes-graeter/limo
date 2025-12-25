// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <functional>
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
public: // public classes/enums/types etc...
    using LmMap = Eigen::Map<const Eigen::Vector3d>;
    using FrameIndex = int;      ///< Index of frame, 0 means first frame.
    using NumberLandmarks = int; ///< Number of landmarks to take
    using Comparator =
        std::function<bool(const Landmark::ConstPtr&)>; ///< Function which determines if shall be adde dor not.
    using Sorter = std::function<float(const Measurement&, const Eigen::Vector3d&)>; ///< Function that
                                                                                     /// gives a double for
    /// which the elements
    /// are sorted.
    struct Parameters {
        std::vector<std::tuple<FrameIndex, NumberLandmarks, Comparator, Sorter>>
            params_per_keyframe; ///< Determines how many depth points are assured on what keyframes.
                                 //        {
                                 //            std::make_tuple(0,
                                 //                            20,
        //                            [](const Landmark::ConstPtr& lm) { return lm->has_measured_depth; },
        //                            [](const Measurement& m, const Eigen::Vector3d& lm) { return m.d; }),
        //            std::make_tuple(0,
        //                            20,
        //                            [](const Landmark::ConstPtr& lm) { return lm->is_ground_plane; },
        //                            [](const Measurement& m, const Eigen::Vector3d& local_lm) {
        //                                return local_lm.norm();
        //                            })};
    };

public: // public methods
    // default constructor
    LandmarkSelectionSchemeAddDepth(Parameters p) : params_(p) {
        identifier = "add depth";
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
