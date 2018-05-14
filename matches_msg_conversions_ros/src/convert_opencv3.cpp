/*
 * Copyright 2017. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */

#include "convert_opencv3.hpp"

namespace matches_msg_conversions_ros {

LastCurPoints ConvertF2F(const matches_msg_ros::MatchesMsg::ConstPtr m) {
    PointsVec last_points, cur_points;
    last_points.reserve(m->tracks.size());
    cur_points.reserve(m->tracks.size());
    for (const auto& tracklet : m->tracks) {
        if (tracklet.feature_points.size() < 2)
            continue;

        auto cur_match = tracklet.feature_points.front();
        cur_points.push_back(cv::Point2f(cur_match.u, cur_match.v));

        cur_match = *std::next(tracklet.feature_points.cbegin());
        last_points.push_back(cv::Point2f(cur_match.u, cur_match.v));
    }

    return std::make_pair(last_points,cur_points);
}
} // end of ns
