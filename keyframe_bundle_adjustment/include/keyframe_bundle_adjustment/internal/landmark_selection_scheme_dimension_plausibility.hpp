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
class LandmarkRejectionSchemeDimensionPlausibility : public LandmarkRejectionSchemeBase {
public: // public classes/enums/types etc...
    struct Params {
        double min_x{std::numeric_limits<double>::min()}, max_x{std::numeric_limits<double>::max()};
        double min_y{std::numeric_limits<double>::min()}, max_y{std::numeric_limits<double>::max()};
        double min_z{std::numeric_limits<double>::min()}, max_z{std::numeric_limits<double>::max()};
    };

public: // public methods
    // default constructor
    LandmarkRejectionSchemeDimensionPlausibility(const Params& p) : params_(p) {
        ;
    }

    // default destructor
    virtual ~LandmarkRejectionSchemeDimensionPlausibility() = default;

    // default move
    LandmarkRejectionSchemeDimensionPlausibility(LandmarkRejectionSchemeDimensionPlausibility&& other) = default;
    LandmarkRejectionSchemeDimensionPlausibility& operator=(LandmarkRejectionSchemeDimensionPlausibility&& other) =
        default;

    // default copy
    LandmarkRejectionSchemeDimensionPlausibility(const LandmarkRejectionSchemeDimensionPlausibility& other) = default;
    LandmarkRejectionSchemeDimensionPlausibility& operator=(const LandmarkRejectionSchemeDimensionPlausibility& other) =
        default;

    std::set<LandmarkId> getSelection(const LandmarkMap& landmarks, const KeyframeMap& keyframes) const override {
        auto it =
            std::max_element(keyframes.cbegin(), keyframes.cend(), [](const auto& a, const auto& b) { return a < b; });
        const auto& newest_kf = *it;

        std::set<LandmarkId> out;
        for (const auto& el : landmarks) {
            Eigen::Vector3d p =
                newest_kf.second->getEigenPose() * Eigen::Map<const Eigen::Vector3d>(el.second->pos.data());
            if (p[0] > params_.min_x && p[0] < params_.max_x && p[1] > params_.min_y && p[1] < params_.max_y &&
                p[2] > params_.min_z && p[2] < params_.max_z) {
                out.insert(el.first);
            }
        }

        return out;
    }

    //////////////////////////////////////////////////
    /// \brief createConst, factory function for const pointer
    /// \return
    ///
    static ConstPtr createConst(const Params& p) {
        return LandmarkRejectionSchemeDimensionPlausibility::ConstPtr(
            new LandmarkRejectionSchemeDimensionPlausibility(p));
    }

    //////////////////////////////////////////////////
    /// \brief create, factory function
    /// \return
    ///
    static Ptr create(const Params& p) {
        return LandmarkRejectionSchemeDimensionPlausibility::Ptr(new LandmarkRejectionSchemeDimensionPlausibility(p));
    }


private: // attributes
    Params params_;
};
}
