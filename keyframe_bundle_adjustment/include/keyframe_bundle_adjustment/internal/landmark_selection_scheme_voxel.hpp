// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include "landmark_categorization_interface.hpp"
#include "landmark_selection_scheme_base.hpp"

namespace keyframe_bundle_adjustment {
/**
*  @class LandmarkSelectionSchemeVoxel
*  @par
*
*  Concrete LandmarkSelectionScheme; sparsify landmarks by running a voxelfilter
*  of landmark pointcloud.
*/
class LandmarkSparsificationSchemeVoxel : public LandmarkSparsificationSchemeBase,
                                          public LandmarkCategorizatonInterface {
public: // Public classes/enums/types etc...
    struct Parameters {
        Parameters() {
            ;
        }
        std::array<double, 3> voxel_size_xyz{{1.0, 1.0, 0.5}}; ///< Voxelsize in x,y,z in meters
        std::array<double, 3> roi_far_xyz{{
            50., 50., 50.}}; ///< Roi around current position in hwich points will be considered
        std::array<double, 3> roi_middle_xyz{{
            25., 25., 25.}};                         ///< Roi around current position in hwich points will be considered
        unsigned int max_num_landmarks_near{300};   ///< maximum number of landmarks in near bin
        unsigned int max_num_landmarks_middle{300}; ///< maximum number of landmarks in middle bin
        unsigned int max_num_landmarks_far{300};    ///< maximum number of landmarks in far bin
    };

public: // Attributes.
public: // Public methods.
    // Default constructor.
    explicit LandmarkSparsificationSchemeVoxel(Parameters p) : params_(p) {
        ;
    }

    // Default destructor.
    ~LandmarkSparsificationSchemeVoxel() = default;

    // Default move.
    LandmarkSparsificationSchemeVoxel(LandmarkSparsificationSchemeVoxel&& other) = default;
    LandmarkSparsificationSchemeVoxel& operator=(LandmarkSparsificationSchemeVoxel&& other) = default;

    // Default copy.
    LandmarkSparsificationSchemeVoxel(const LandmarkSparsificationSchemeVoxel& other) = default;
    LandmarkSparsificationSchemeVoxel& operator=(const LandmarkSparsificationSchemeVoxel& other) = default;

    std::set<LandmarkId> getSelection(const LandmarkMap& new_frame, const KeyframeMap& keyframes) const override;

    std::map<LandmarkId, LandmarkCategorizatonInterface::Category> getCategorizedSelection(
        const LandmarkMap& new_frame, const KeyframeMap& keyframes) const override;

    //////////////////////////////////////////////////
    /// \brief createConst, factory function for const pointer
    /// \return
    ///
    static ConstPtr createConst(Parameters p = Parameters()) {
        return LandmarkSparsificationSchemeBase::ConstPtr(new LandmarkSparsificationSchemeVoxel(p));
    }

    //////////////////////////////////////////////////
    /// \brief create, factory function
    /// \return
    ///
    static Ptr create(Parameters p = Parameters()) {
        return LandmarkSparsificationSchemeBase::Ptr(new LandmarkSparsificationSchemeVoxel(p));
    }


public:                 // attributes
    Parameters params_; ///< paramters
};
}
