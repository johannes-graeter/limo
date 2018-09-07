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
*  @class LandmarkSchemeBase
*  @par
*
*  Base class for landmark selection, rejection and sparsification schemes
*/
class LandmarkSchemeBase {
public: // public classes/enums/types etc...
        //    using Ptr = std::shared_ptr<LandmarkSchemeBase>;
        //    using ConstPtr = std::shared_ptr<const LandmarkSchemeBase>;
    using LandmarkMap = std::map<LandmarkId, Landmark::ConstPtr>;
    using KeyframeMap = std::map<KeyframeId, Keyframe::ConstPtr>;

public: // attributes
public: // public methods
    // default constructor
    LandmarkSchemeBase() = default;

    // default destructor
    virtual ~LandmarkSchemeBase() = default;

    // default move
    LandmarkSchemeBase(LandmarkSchemeBase&& other) = default;
    LandmarkSchemeBase& operator=(LandmarkSchemeBase&& other) = default;

    // default copy
    LandmarkSchemeBase(const LandmarkSchemeBase& other) = default;
    LandmarkSchemeBase& operator=(const LandmarkSchemeBase& other) = default;

    /**
     * @brief getSelection, abstract class for interface to schemes
     * @param landmarks, whole set of landmarks
     * @return selected set of landmarks
     */
    virtual std::set<LandmarkId> getSelection(const LandmarkMap& landmarks, const KeyframeMap& keyframes) const = 0;
};

/**
*  @class LandmarkSelectionSchemeBase
*  @par
*
*  Selection scheme, all landmarks selected by this scheme get definitely accepted
*/
class LandmarkSelectionSchemeBase : public LandmarkSchemeBase {
public: // Public classes/enums/types etc...
    using Ptr = std::shared_ptr<LandmarkSelectionSchemeBase>;
    using ConstPtr = std::shared_ptr<const LandmarkSelectionSchemeBase>;

public: // Attributes.
public: // Public methods.
    // Default constructor.
    explicit LandmarkSelectionSchemeBase() = default;

    // Default destructor.
    virtual ~LandmarkSelectionSchemeBase() = default;

    // Default move.
    LandmarkSelectionSchemeBase(LandmarkSelectionSchemeBase&& other) = default;
    LandmarkSelectionSchemeBase& operator=(LandmarkSelectionSchemeBase&& other) = default;

    // Default copy.
    LandmarkSelectionSchemeBase(const LandmarkSelectionSchemeBase& other) = default;
    LandmarkSelectionSchemeBase& operator=(const LandmarkSelectionSchemeBase& other) = default;
};

/**
*  @class LandmarkRejectionSchemeBase
*  @par
*
*  Rejection scheme, all landmarks that get not selected by this scheme get definitely rejected.
*/
class LandmarkRejectionSchemeBase : public LandmarkSchemeBase {
public: // Public classes/enums/types etc...
    using Ptr = std::shared_ptr<LandmarkRejectionSchemeBase>;
    using ConstPtr = std::shared_ptr<const LandmarkRejectionSchemeBase>;

public: // Attributes.
public: // Public methods.
    // Default constructor.
    explicit LandmarkRejectionSchemeBase() = default;

    // Default destructor.
    virtual ~LandmarkRejectionSchemeBase() = default;

    // Default move.
    LandmarkRejectionSchemeBase(LandmarkRejectionSchemeBase&& other) = default;
    LandmarkRejectionSchemeBase& operator=(LandmarkRejectionSchemeBase&& other) = default;

    // Default copy.
    LandmarkRejectionSchemeBase(const LandmarkRejectionSchemeBase& other) = default;
    LandmarkRejectionSchemeBase& operator=(const LandmarkRejectionSchemeBase& other) = default;
};

/**
*  @class LandmarkSparsificationSchemeBase
*  @par
*
*  After selection and rejection rest of landmarks is sparsified by these methods.
*/
class LandmarkSparsificationSchemeBase : public LandmarkSchemeBase {
public: // Public classes/enums/types etc...
    using Ptr = std::shared_ptr<LandmarkSparsificationSchemeBase>;
    using ConstPtr = std::shared_ptr<const LandmarkSparsificationSchemeBase>;

public: // Attributes.
public: // Public methods.
    // Default constructor.
    explicit LandmarkSparsificationSchemeBase() = default;

    // Default destructor.
    virtual ~LandmarkSparsificationSchemeBase() = default;

    // Default move.
    LandmarkSparsificationSchemeBase(LandmarkSparsificationSchemeBase&& other) = default;
    LandmarkSparsificationSchemeBase& operator=(LandmarkSparsificationSchemeBase&& other) = default;

    // Default copy.
    LandmarkSparsificationSchemeBase(const LandmarkSparsificationSchemeBase& other) = default;
    LandmarkSparsificationSchemeBase& operator=(const LandmarkSparsificationSchemeBase& other) = default;
};
}
