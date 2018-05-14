// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once
#include <algorithm>
#include <functional>

#include "keyframe_bundle_adjustment/keyframe.hpp"

namespace keyframe_bundle_adjustment_ros_tool {

/**
*  @class ScaleProvider
*  @par
*
*  Class to provide scale if it is not estimated explicitely.
*
*  Sometimes bundle adjustment is initialized with a prior
*  without scale (f.e. momo). In that case we save last poses
*  in this class and interpolate their velocity for the
*  current frame.
*
*/
class MotionExtrapolator {
public:
    using KeyframesTsMap =
        std::map<keyframe_bundle_adjustment::TimestampNSec, keyframe_bundle_adjustment::Keyframe::ConstPtr>;

    struct NotEnoughDataException : public std::exception {
        virtual const char* what() const throw() {
            return "In ScaleProvider: Not enough data for interpolation";
        }
    };

    struct InvalidTimestampException : public std::exception {
        virtual const char* what() const throw() {
            return "In ScaleProvider: The timestamp given for interpolation is invalid.";
        }
    };

public:                          // attributes
    int interpolation_size_ = 2; ///< Windowsize for linear interpolation
public:                          // public methods
    // default constructor
    explicit MotionExtrapolator() = default;

    // default destructor
    virtual ~MotionExtrapolator() = default;

    // default move
    MotionExtrapolator(MotionExtrapolator&& other) = default;
    MotionExtrapolator& operator=(MotionExtrapolator&& other) = default;

    // default copy
    MotionExtrapolator(const MotionExtrapolator& other) = default;
    MotionExtrapolator& operator=(const MotionExtrapolator& other) = default;

    Eigen::Isometry3d getMotion(keyframe_bundle_adjustment::TimestampNSec cur_ts) const {
        if (last_kfs_.size() < 2) {
            throw NotEnoughDataException();
        }
        // Get average of velocity.
        const auto& cur_kf_ptr = last_kfs_.crbegin()->second;
        auto it = last_kfs_.crbegin();
        std::advance(it, 1);
        const auto& last_kf_ptr = it->second;
        Eigen::Isometry3d delta_motion = cur_kf_ptr->getEigenPose() * last_kf_ptr->getEigenPose().inverse();
        keyframe_bundle_adjustment::TimestampSec delta_time_sec =
            keyframe_bundle_adjustment::convert(cur_kf_ptr->timestamp_ - last_kf_ptr->timestamp_);

        // Interpolate scale
        keyframe_bundle_adjustment::TimestampSec cur_dtime =
            keyframe_bundle_adjustment::convert(cur_ts - cur_kf_ptr->timestamp_);
        if (cur_dtime < 0.) {
            throw InvalidTimestampException();
        }

        Eigen::Isometry3d out;
        out.translation() = delta_motion.translation() * cur_dtime / delta_time_sec;

        Eigen::AngleAxisd rot_aa(delta_motion.rotation());
        rot_aa.angle() = rot_aa.angle() * cur_dtime / delta_time_sec;
        out.linear() = rot_aa.matrix();
        return out;
    }

    Eigen::Vector3d applyScale(keyframe_bundle_adjustment::TimestampNSec cur_ts, Eigen::Vector3d translation) const {
        Eigen::Vector3d out;
        try {
            out = translation * getScale(cur_ts) / translation.norm();
        } catch (const NotEnoughDataException& e) {
            std::cout << e.what() << "; continuing" << std::endl;
            out = translation;
        }

        return out;
    }

    double getScale(keyframe_bundle_adjustment::TimestampNSec cur_ts) const {
        if (last_kfs_.size() < 2) {
            throw NotEnoughDataException();
        }
        // Get average of velocity.
        double mean_velocity = 0.;
        auto last_iter = last_kfs_.cbegin();
        auto iter = last_kfs_.cbegin();
        iter++;
        for (; iter != last_kfs_.cend(); ++last_iter, ++iter) {
            Eigen::Vector3d delta_trans =
                iter->second->getEigenPose().translation() - last_iter->second->getEigenPose().translation();
            keyframe_bundle_adjustment::TimestampSec delta_time_sec =
                keyframe_bundle_adjustment::convert(iter->second->timestamp_ - last_iter->second->timestamp_);
            mean_velocity += delta_trans.norm() / delta_time_sec;
        }
        mean_velocity /= static_cast<double>(last_kfs_.size());

        // Interpolate scale
        keyframe_bundle_adjustment::TimestampSec cur_dtime =
            keyframe_bundle_adjustment::convert(cur_ts - last_kfs_.crbegin()->second->timestamp_);
        if (cur_dtime < 0.) {
            throw InvalidTimestampException();
        }
        return mean_velocity * cur_dtime;
    }

    void setLastKeyframePtrs(const KeyframesTsMap& kfs_map) {
        // Set data.
        for (const auto& el : kfs_map) {
            last_kfs_[el.second->timestamp_] = el.second;
        }

        // Get timestamp that corresponds to interpolation size.
        int adv_num = std::min(interpolation_size_, static_cast<int>(last_kfs_.size() - 1));
        auto r_it = last_kfs_.rbegin();
        std::advance(r_it, adv_num);
        keyframe_bundle_adjustment::TimestampNSec oldest_timestamp = r_it->second->timestamp_;

        // Erase keyframes that are smaller than the oldest possible timestamp.
        auto it = last_kfs_.begin();
        for (; it != last_kfs_.end();) {
            if (oldest_timestamp >= it->first) {
                it = last_kfs_.erase(it);
            } else {
                ++it;
            }
        }
    }

    KeyframesTsMap getLastKfs() const {
        return last_kfs_;
    }

private:
    ///@brief Keyframes with scale from epesarlier points in time.
    KeyframesTsMap last_kfs_;
};
}
