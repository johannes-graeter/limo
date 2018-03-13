// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include "keyframe_schemes_base.hpp"

namespace keyframe_bundle_adjustment {

/**
*  @class KeyframeSelectionSchemeTime
*  @par
*
* Concrete KeyframeSelectionScheme; select keyframes on the time between them
*/
class KeyframeSparsificationSchemeTime : public KeyframeSparsificationSchemeBase {
public: // public classes/enums/types etc...
    using DurationNSec = unsigned long;

public: // public methods
    // default constructor
    KeyframeSparsificationSchemeTime(double time_difference_sec)
            : time_difference_nano_sec_(convert(time_difference_sec)) {
        ;
    }

    // default destructor
    virtual ~KeyframeSparsificationSchemeTime() = default;

    // default move
    KeyframeSparsificationSchemeTime(KeyframeSparsificationSchemeTime&& other) = default;
    KeyframeSparsificationSchemeTime& operator=(KeyframeSparsificationSchemeTime&& other) = default;

    // default copy
    KeyframeSparsificationSchemeTime(const KeyframeSparsificationSchemeTime& other) = default;
    KeyframeSparsificationSchemeTime& operator=(const KeyframeSparsificationSchemeTime& other) = default;

    //////////////////////////////////////////////////
    /// \brief isSelected, concrete selection scheme, according to interface in base
    /// \param
    /// \param
    ///
    bool isUsable(const Keyframe::Ptr& new_frame,
                  const std::map<KeyframeId, Keyframe::Ptr>& last_frames) const override;

    //////////////////////////////////////////////////
    /// \brief create_const, factory function for const pointer
    /// \param time_difference_nano_sec_
    /// \return
    ///
    static ConstPtr createConst(double time_difference_nano_sec_);

    //////////////////////////////////////////////////
    /// \brief create, factory function
    /// \param time_difference_nano_sec_
    /// \return
    ///
    static Ptr create(double time_difference_nano_sec_);


public: // attributes
    DurationNSec time_difference_nano_sec_;
};
}
