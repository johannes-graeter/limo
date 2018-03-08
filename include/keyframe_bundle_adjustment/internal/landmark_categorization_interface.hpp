// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include "definitions.hpp"
#include "../keyframe.hpp"

namespace keyframe_bundle_adjustment {

/**
*  @class LandmarkCategorizatonInterface
*  @par
*
*  A class inherting from this will be forced to have an interface that supplies categories for
*  landmarks
*/
struct LandmarkCategorizatonInterface {
    enum class Category { NearField, MiddleField, FarField };

    virtual std::map<LandmarkId, Category> getCategorizedSelection(
        const std::map<LandmarkId, Landmark::ConstPtr>& landmarks,
        const std::map<KeyframeId, Keyframe::ConstPtr>& keyframes) const = 0;
};
}
