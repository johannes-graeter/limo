// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <cv.hpp>
#include <keyframe_bundle_adjustment/bundle_adjuster_keyframes.hpp>
#include <matches_msg_types/tracklets.hpp>

#include "color_by_index_hsv.hpp"

namespace keyframe_bundle_adjustment_ros_tool {

using CvPoints = std::vector<cv::Point2d>;

namespace helpers {

std::string poseToString(Eigen::Matrix<double, 4, 4> m) {
    std::stringstream ss;
    ss << m(0, 0) << " " << m(0, 1) << " " << m(0, 2) << " " << m(0, 3) << " " << m(1, 0) << " "
       << m(1, 1) << " " << m(1, 2) << " " << m(1, 3) << " " << m(2, 0) << " " << m(2, 1) << " "
       << m(2, 2) << " " << m(2, 3);
    return ss.str();
}

std::tuple<CvPoints, CvPoints> getMatches(const matches_msg_types::Tracklets& tracklets,
                                          keyframe_bundle_adjustment::TimestampNSec ts0,
                                          keyframe_bundle_adjustment::TimestampNSec ts1) {

    // get indices of stamps
    int index0;
    {
        auto iter = std::find(tracklets.stamps.cbegin(), tracklets.stamps.cend(), ts0);
        index0 = std::distance(tracklets.stamps.cbegin(), iter);
    }

    int index1;
    {
        auto iter = std::find(tracklets.stamps.cbegin(), tracklets.stamps.cend(), ts1);
        index1 = std::distance(tracklets.stamps.cbegin(), iter);
    }


    // get matches corresponding to timest
    CvPoints points0;
    CvPoints points1;

    for (const auto& track : tracklets.tracks) {
        if (int(track.feature_points.size()) > index0 &&
            int(track.feature_points.size()) > index1) {
            points0.push_back(
                cv::Point2d(track.feature_points[index0].u, track.feature_points[index0].v));
            points1.push_back(
                cv::Point2d(track.feature_points[index1].u, track.feature_points[index1].v));
        }
    }

    return std::make_tuple(points0, points1);
}


/**
 * @brief getFlowImg, plot selected measurements from keyframe
 * @param bundle_adjuster
 * @return img with a lot of dots
 */
cv::Mat getFlowImg(keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster) {
    cv::Mat debug_img(600, 1300, CV_8UC3);
    debug_img.setTo(0);

    int num_colors = 10;

    for (const auto& kf : bundle_adjuster->keyframes_) {
        if (kf.second->is_active_) {
            for (const auto& m : kf.second->measurements_) {
                const auto& m_id = m.first;
                const auto& meas = m.second.cbegin()->second;

                auto color = util_image::get_color(m_id, num_colors);

                cv::circle(debug_img, cv::Point(meas.u, meas.v), 1, color, -1);
            }
        }
    }

    return debug_img;
}
}
}
