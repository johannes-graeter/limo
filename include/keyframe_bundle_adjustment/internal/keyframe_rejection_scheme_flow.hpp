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
*  @class KeyframeSelectionSchemeFlow
*  @par
*
*  select keyframes from flow, this is particularly important for stand still detection to not add
* frames between which flow is too small
*/
class KeyframeRejectionSchemeFlow : public KeyframeRejectionSchemeBase {
public: // public classes/enums/types etc...
public: // attributes
public: // public methods
    // default constructor
    KeyframeRejectionSchemeFlow(double min_median_flow);

    // default destructor
    virtual ~KeyframeRejectionSchemeFlow() = default;

    // default move
    KeyframeRejectionSchemeFlow(KeyframeRejectionSchemeFlow&& other) = default;
    KeyframeRejectionSchemeFlow& operator=(KeyframeRejectionSchemeFlow&& other) = default;

    // default copy
    KeyframeRejectionSchemeFlow(const KeyframeRejectionSchemeFlow& other) = default;
    KeyframeRejectionSchemeFlow& operator=(const KeyframeRejectionSchemeFlow& other) = default;

    //////////////////////////////////////////////////
    /// \brief isSelected, concrete selection scheme, according to interface in base
    /// calculates medioan of flow to most recent keyframe from last_frames and from all cameras
    /// \param new_frame, frame that should be tested for selection
    /// \param last_frames, frames, against which new_frame is tested
    /// \return true if median flow is bigger than min_median_flow_
    ///
    bool isUsable(const Keyframe::Ptr& new_frame,
                  const std::map<KeyframeId, Keyframe::Ptr>& last_frames) const override;

    //////////////////////////////////////////////////
    /// \brief create_const, factory function for const pointer
    /// \param time_difference_nano_sec_
    /// \return
    ///
    static ConstPtr createConst(double min_median_flow);

    //////////////////////////////////////////////////
    /// \brief create, factory function
    /// \param time_difference_nano_sec_
    /// \return
    ///
    static Ptr create(double min_median_flow);

public:
    double min_median_flow_squared_; ///< when median of flow most recent keyframe is smaller than
                                     /// this threshold, reject frame
};
}
