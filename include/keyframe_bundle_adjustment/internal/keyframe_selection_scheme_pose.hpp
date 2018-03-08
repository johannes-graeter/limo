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
*  @class KeyframeSelectionSchemePose
*  @par
*
* This scheme selects keyframes that have a pose difference over a certain threshold. This is
* important in curves where many keyframes are needed to not loose track.
*/
class KeyframeSelectionSchemePose : public KeyframeSelectionSchemeBase {
public: // public classes/enums/types etc...
public: // attributes
public: // public methods
    // default constructor
    KeyframeSelectionSchemePose(double critical_quaternion_difference);

    // default destructor
    virtual ~KeyframeSelectionSchemePose() = default;

    // default move
    KeyframeSelectionSchemePose(KeyframeSelectionSchemePose&& other) = default;
    KeyframeSelectionSchemePose& operator=(KeyframeSelectionSchemePose&& other) = default;

    // default copy
    KeyframeSelectionSchemePose(const KeyframeSelectionSchemePose& other) = default;
    KeyframeSelectionSchemePose& operator=(const KeyframeSelectionSchemePose& other) = default;

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
    static ConstPtr createConst(double critical_quaternion_difference);

    //////////////////////////////////////////////////
    /// \brief create, factory function
    /// \param time_difference_nano_sec_
    /// \return
    ///
    static Ptr create(double critical_quaternion_difference);

private:
    double critical_quaternion_diff_; ///< if difference angle between quaternions is
};
}
