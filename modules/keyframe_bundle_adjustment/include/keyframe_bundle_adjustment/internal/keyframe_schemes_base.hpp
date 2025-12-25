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

#include "../keyframe.hpp"

namespace keyframe_bundle_adjustment {

/**
*  @class KeyframeSchemeBase
*  @par
*
*  interface for keyframe selection, rejection and sparsification schemes.
*  making the difference between the 3 of them is important to apply them in the correct oreder in selector
*/
class KeyframeSchemeBase {
public: // public classes/enums/types etc...
    using Ptr = std::shared_ptr<KeyframeSchemeBase>;
    using ConstPtr = std::shared_ptr<const KeyframeSchemeBase>;

public: // attributes
public: // public methods
    // default constructor
    KeyframeSchemeBase() = default;

    // default destructor
    virtual ~KeyframeSchemeBase() = default;

    // default move
    KeyframeSchemeBase(KeyframeSchemeBase&& other) = default;
    KeyframeSchemeBase& operator=(KeyframeSchemeBase&& other) = default;

    // default copy
    KeyframeSchemeBase(const KeyframeSchemeBase& other) = default;
    KeyframeSchemeBase& operator=(const KeyframeSchemeBase& other) = default;

    /**
    * @brief isUsable, abstract class for interface to schemes: if true the "new_frame" is
    * accpeted by scheme as a keyframe
    * @param new_frame, new keyframe that is teste
    * @param last_selected_keyframes, all previously selected keyframes against which new_frame is
    * compared
    * @return boolean if new_frame is accepted by scheme or not
    */
    virtual bool isUsable(const Keyframe::Ptr& new_frame,
                          const std::map<KeyframeId, Keyframe::Ptr>& last_selected_keyframes) const = 0;
};

/**
*  @class KeyframeSelectionSchemeBase
*  @par
*
*  Base class for keyframe selection schemes.
*  Selection means that keyframes that fullfill this metric have to be used otherwise estimation
*  will get unstable
*/
class KeyframeSelectionSchemeBase : public KeyframeSchemeBase {
public: // public classes/enums/types etc...
    using Ptr = std::shared_ptr<KeyframeSelectionSchemeBase>;
    using ConstPtr = std::shared_ptr<const KeyframeSelectionSchemeBase>;

public: // attributes
public: // public methods
    // default constructor
    KeyframeSelectionSchemeBase() = default;

    // default destructor
    virtual ~KeyframeSelectionSchemeBase() = default;

    // default move
    KeyframeSelectionSchemeBase(KeyframeSelectionSchemeBase&& other) = default;
    KeyframeSelectionSchemeBase& operator=(KeyframeSelectionSchemeBase&& other) = default;

    // default copy
    KeyframeSelectionSchemeBase(const KeyframeSelectionSchemeBase& other) = default;
    KeyframeSelectionSchemeBase& operator=(const KeyframeSelectionSchemeBase& other) = default;
};

/**
*  @class KeyframeRejectionSchemeBase
*  @par
*
*  these are schemes that reject keyframes with which result would be instable
*/
class KeyframeRejectionSchemeBase : public KeyframeSchemeBase {
public: // public classes/enums/types etc...
    using Ptr = std::shared_ptr<KeyframeRejectionSchemeBase>;
    using ConstPtr = std::shared_ptr<const KeyframeRejectionSchemeBase>;

public: // attributes
public: // public methods
    // default constructor
    KeyframeRejectionSchemeBase() = default;

    // default destructor
    virtual ~KeyframeRejectionSchemeBase() = default;

    // default move
    KeyframeRejectionSchemeBase(KeyframeRejectionSchemeBase&& other) = default;
    KeyframeRejectionSchemeBase& operator=(KeyframeRejectionSchemeBase&& other) = default;

    // default copy
    KeyframeRejectionSchemeBase(const KeyframeRejectionSchemeBase& other) = default;
    KeyframeRejectionSchemeBase& operator=(const KeyframeRejectionSchemeBase& other) = default;
};

/**
*  @class KeyframeSparsificationSchemeBase
*  @par
*
*  these are schemes that reject keyframes if their information does not harm but doesn't bring
* great benefit. It is a special kind of rejection scheme
*/
class KeyframeSparsificationSchemeBase : public KeyframeSchemeBase {
public: // public classes/enums/types etc...
    using Ptr = std::shared_ptr<KeyframeSparsificationSchemeBase>;
    using ConstPtr = std::shared_ptr<const KeyframeSparsificationSchemeBase>;

public: // attributes
public: // public methods
    // default constructor
    KeyframeSparsificationSchemeBase() = default;

    // default destructor
    virtual ~KeyframeSparsificationSchemeBase() = default;

    // default move
    KeyframeSparsificationSchemeBase(KeyframeSparsificationSchemeBase&& other) = default;
    KeyframeSparsificationSchemeBase& operator=(KeyframeSparsificationSchemeBase&& other) = default;

    // default copy
    KeyframeSparsificationSchemeBase(const KeyframeSparsificationSchemeBase& other) = default;
    KeyframeSparsificationSchemeBase& operator=(const KeyframeSparsificationSchemeBase& other) = default;
};
}
