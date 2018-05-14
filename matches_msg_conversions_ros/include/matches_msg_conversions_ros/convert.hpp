/*
 * Copyright 2017. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */
#pragma once

#include <matches_msg_depth_ros/MatchesMsg.h>
#include <matches_msg_depth_ros/MatchesMsgWithOutlierFlag.h>
#include <matches_msg_ros/MatchesMsg.h>
#include <matches_msg_ros/MatchesMsgWithOutlierFlag.h>

#include <keyframe_bundle_adjustment/matches_msg_types.hpp>

#include <type_traits>

namespace matches_msg_conversions_ros {

using OutlierFlags = std::vector<bool>;
using Errors = std::vector<double>;

///@brief Define custom assert
void Assert(bool condition, std::string s);

///@brief add outlier flags to existing message
template <typename T, typename TOut>
TOut Convert(const typename T::ConstPtr m, const OutlierFlags& outlier_flags) {

    TOut out;

    out.header = m->header;
    out.stamps = m->stamps;

    auto i1 = m->tracks.cbegin();
    auto i2 = outlier_flags.cbegin();
    Assert(m->tracks.size() == outlier_flags.size(),
           "matches_msg.data.size()=" + std::to_string(m->tracks.size()) + " !=  outlier_flags.size()=" +
               std::to_string(outlier_flags.size()));

    using TrackletVecType = decltype(out.tracks); // should be something like std::vector<matches_msg_ros::TrackletsMsg>
    using TrackletType = typename TrackletVecType::value_type; // matches_msg_ros::TrackletsMsg

    for (; i1 != m->tracks.cend(); ++i1, ++i2) {
        TrackletType converted_tracklet;
        converted_tracklet.feature_points = i1->feature_points;
        converted_tracklet.id = i1->id;
        converted_tracklet.is_outlier = *i2;
        converted_tracklet.label = -2;


        out.tracks.emplace_back(std::move(converted_tracklet));
    }
    return out;
}

///@brief Overload instead of specialization
matches_msg_depth_ros::MatchesMsgWithOutlierFlag Convert(const matches_msg_depth_ros::MatchesMsg::ConstPtr m,
                                                         const OutlierFlags& out_f) {
    return Convert<matches_msg_depth_ros::MatchesMsg, matches_msg_depth_ros::MatchesMsgWithOutlierFlag>(m, out_f);
}

matches_msg_ros::MatchesMsgWithOutlierFlag Convert(const matches_msg_ros::MatchesMsg::ConstPtr m,
                                                   const OutlierFlags& out_f) {
    return Convert<matches_msg_ros::MatchesMsg, matches_msg_ros::MatchesMsgWithOutlierFlag>(m, out_f);
}

///@brief add outlier flags to existing msg with additional field for errors, f.e. loss from
/// optimization
template <typename T, typename TOut>
TOut Convert(const typename T::ConstPtr m, const OutlierFlags& outlier_flags, const Errors& errs) {
    TOut out;

    out.header = m->header;
    out.stamps = m->stamps;

    auto i1 = m->tracks.cbegin();
    auto i2 = outlier_flags.cbegin();
    auto i3 = errs.cbegin();
    Assert(m->tracks.size() == outlier_flags.size() && outlier_flags.size() == errs.size(),
           "matches_msg.data.size()=" + std::to_string(m->tracks.size()) + " !=  outlier_flags.size()=" +
               std::to_string(outlier_flags.size()));

    using TrackletVecType = decltype(out.tracks); // should be something like std::vector<matches_msg_ros::TrackletsMsg>
    using TrackletType = typename TrackletVecType::value_type; // matches_msg_ros::TrackletsMsg

    for (; i1 != m->tracks.cend(); ++i1, ++i2, ++i3) {
        TrackletType converted_tracklet;
        converted_tracklet.feature_points = i1->feature_points;
        converted_tracklet.id = i1->id;
        converted_tracklet.is_outlier = *i2;
        converted_tracklet.error = *i3;
        converted_tracklet.label = -2;

        out.tracks.emplace_back(std::move(converted_tracklet));
    }
    return out;
}

///@brief Overload instead of specialization
matches_msg_depth_ros::MatchesMsgWithOutlierFlag Convert(const matches_msg_depth_ros::MatchesMsg::ConstPtr m,
                                                         const OutlierFlags& out_f,
                                                         const Errors& errs) {
    return Convert<matches_msg_depth_ros::MatchesMsg, matches_msg_depth_ros::MatchesMsgWithOutlierFlag>(m, out_f, errs);
}

matches_msg_ros::MatchesMsgWithOutlierFlag Convert(const matches_msg_ros::MatchesMsg::ConstPtr m,
                                                   const OutlierFlags& out_f,
                                                   const Errors& errs) {
    return Convert<matches_msg_ros::MatchesMsg, matches_msg_ros::MatchesMsgWithOutlierFlag>(m, out_f, errs);
}

// Overload function for usage of feature points with and without depth.
matches_msg_types::FeaturePoint getFeaturePoint(const matches_msg_depth_ros::FeaturePoint& f) {
    return matches_msg_types::FeaturePoint(f.u, f.v, f.d);
}

matches_msg_types::FeaturePoint getFeaturePoint(const matches_msg_ros::FeaturePoint& f) {
    return matches_msg_types::FeaturePoint(f.u, f.v);
}

// Templates for testing if method has outlier flag with SFINAE.
template <typename T, typename = void>
struct HasOutlierFlag : std::false_type {};

template <typename T>
struct HasOutlierFlag<T, decltype(std::declval<T>().is_outlier, void())> : std::true_type {};

// Enable functions, SFINAE style
// If the tracklet msg has a field .is_outlier, we add data to it otherwise we do nothing
// First is the default declaration, second is only activated if std::enable_if<true, return_type>
template <typename T>
typename std::enable_if<HasOutlierFlag<T>::value == false, void>::type MaybeAddOutlier(
    const T& msg_track, matches_msg_types::Tracklet& cur_track) {
    ;
}
template <typename T>
typename std::enable_if<HasOutlierFlag<T>::value == true, void>::type MaybeAddOutlier(
    const T& msg_track, matches_msg_types::Tracklet& cur_track) {
    cur_track.is_outlier = msg_track.is_outlier;
    cur_track.label = msg_track.label;
}


///@brief Convert function for MatchesMsg with/without depth and with/without outlier Flags.
template <typename T>
matches_msg_types::Tracklets Convert(const T msg_const_ptr) {
    matches_msg_types::Tracklets out;

    // Convert tracks.
    out.tracks.reserve(msg_const_ptr->tracks.size());
    for (const auto& msg_track : msg_const_ptr->tracks) {
        matches_msg_types::Tracklet cur_track;
        // Standard set.
        cur_track.id = msg_track.id;
        cur_track.age = msg_track.age;

        // Test if msg has outlier flag and add outlier if this is the case.
        // We don't use overloading such as for getFeaturePoint since we then would have to
        // implement already 4 function (with/without outliers for msgs with/without depth)
        MaybeAddOutlier<decltype(msg_track)>(msg_track, cur_track);

        // Convert feature points.
        // Overloaded for tracks with and without depth
        cur_track.feature_points.reserve(msg_track.feature_points.size());
        for (const auto& f : msg_track.feature_points) {
            cur_track.feature_points.push_back(getFeaturePoint(f));
        }

        out.tracks.push_back(cur_track);
    }

    // Convert timestamps.
    out.stamps.reserve(msg_const_ptr->stamps.size());
    for (const auto& stamp : msg_const_ptr->stamps) {
        out.stamps.push_back(matches_msg_types::TimestampNSec(stamp.toNSec()));
    }

    return out;
}

} // end of ns
