// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
// List of some basic tests fuctions:
// Fatal assertion                      Nonfatal assertion                   Verifies / Description
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// ASSERT_EQ(expected, actual);         EXPECT_EQ(expected, actual);         expected == actual
// ASSERT_NE(val1, val2);               EXPECT_NE(val1, val2);               val1 != val2
// ASSERT_LT(val1, val2);               EXPECT_LT(val1, val2);               val1 < val2
// ASSERT_LE(val1, val2);               EXPECT_LE(val1, val2);               val1 <= val2
// ASSERT_GT(val1, val2);               EXPECT_GT(val1, val2);               val1 > val2
// ASSERT_GE(val1, val2);               EXPECT_GE(val1, val2);               val1 >= val2
//
// ASSERT_FLOAT_EQ(expected, actual);   EXPECT_FLOAT_EQ(expected, actual);   the two float values
// are almost equal (4 ULPs)
// ASSERT_DOUBLE_EQ(expected, actual);  EXPECT_DOUBLE_EQ(expected, actual);  the two double values
// are almost equal (4 ULPs)
// ASSERT_NEAR(val1, val2, abs_error);  EXPECT_NEAR(val1, val2, abs_error);  the difference between
// val1 and val2 doesn't exceed the given absolute error
//
// Note: more information about ULPs can be found here:
// http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
//
// Example of two unit test:
// TEST(Math, Add) {
//    ASSERT_EQ(10, 5+ 5);
//}
//
// TEST(Math, Float) {
//	  ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f)
//}
//=======================================================================================================================================================
#include <matches_msg_depth_ros/MatchesMsg.h>
#include <matches_msg_depth_ros/MatchesMsgWithOutlierFlag.h>
#include <matches_msg_ros/MatchesMsg.h>
#include <matches_msg_ros/MatchesMsgWithOutlierFlag.h>
#include <ros/ros.h>
#include "convert.hpp"
#include "gtest/gtest.h"

namespace {
void getData(matches_msg_ros::MatchesMsg& msg) {
    matches_msg_ros::Tracklet track;
    matches_msg_ros::FeaturePoint p;
    p.u = 10.;
    p.v = 15.;
    for (float i = 0.; i < 50.; i += 10.) {
        track.feature_points.push_back(p);
        msg.stamps.push_back(ros::Time(i));
        p.u = p.u + i;
        p.v = p.v + i / 2.;
    }

    msg.tracks.push_back(track);
}
void getData(matches_msg_depth_ros::MatchesMsg& msg) {
    matches_msg_depth_ros::Tracklet track;
    matches_msg_depth_ros::FeaturePoint p;
    p.u = 10.;
    p.v = 15.;
    p.d = 2.;
    for (float i = 0.; i < 50.; i += 10.) {
        track.feature_points.push_back(p);
        msg.stamps.push_back(ros::Time(i));
        p.u = p.u + i;
        p.v = p.v + i / 2.;
        p.d = p.d + 2. * i;
    }

    msg.tracks.push_back(track);
}

void getData(matches_msg_ros::MatchesMsgWithOutlierFlag& msg) {
    matches_msg_ros::TrackletWithOutlierFlag track;
    matches_msg_ros::FeaturePoint p;
    p.u = 10.;
    p.v = 15.;
    for (float i = 0.; i < 50.; i += 10.) {
        track.feature_points.push_back(p);
        msg.stamps.push_back(ros::Time(i));
        p.u = p.u + i;
        p.v = p.v + i / 2.;
    }

    msg.tracks.push_back(track);
    track.is_outlier = true;
    msg.tracks.push_back(track);
}
void getData(matches_msg_depth_ros::MatchesMsgWithOutlierFlag& msg) {
    matches_msg_depth_ros::TrackletWithOutlierFlag track;
    matches_msg_depth_ros::FeaturePoint p;
    p.u = 10.;
    p.v = 15.;
    p.d = 2.;
    for (float i = 0.; i < 50.; i += 10.) {
        track.feature_points.push_back(p);
        msg.stamps.push_back(ros::Time(i));
        p.u = p.u + i;
        p.v = p.v + i / 2.;
        p.d = p.d + 2. * i;
    }

    msg.tracks.push_back(track);
    track.is_outlier = true;
    msg.tracks.push_back(track);
}
}
// A google test function (uncomment the next function, add code and
// change the names TestGroupName and TestName)
TEST(Convert, MatchesMsg) {
    matches_msg_ros::MatchesMsg msg;
    getData(msg);
    matches_msg_depth_ros::MatchesMsg msg_depth;
    getData(msg_depth);
    matches_msg_ros::MatchesMsgWithOutlierFlag msg_w;
    getData(msg_w);
    matches_msg_depth_ros::MatchesMsgWithOutlierFlag msg_depth_w;
    getData(msg_depth_w);

    auto res = matches_msg_conversions_ros::Convert(std::make_shared<const matches_msg_ros::MatchesMsg>(msg));
    auto res_depth =
        matches_msg_conversions_ros::Convert(std::make_shared<const matches_msg_depth_ros::MatchesMsg>(msg_depth));
    auto res_w =
        matches_msg_conversions_ros::Convert(std::make_shared<const matches_msg_ros::MatchesMsgWithOutlierFlag>(msg_w));
    auto res_depth_w = matches_msg_conversions_ros::Convert(
        std::make_shared<const matches_msg_depth_ros::MatchesMsgWithOutlierFlag>(msg_depth_w));

    ASSERT_EQ(res.tracks[0].feature_points.size(), res_depth.tracks[0].feature_points.size());
    ASSERT_EQ(res.tracks.size(), res_depth.tracks.size());
    ASSERT_EQ(res.stamps.size(), res.tracks[0].feature_points.size());
    ASSERT_EQ(res_depth.stamps.size(), res_depth.tracks[0].feature_points.size());
    ASSERT_GT(res_depth.tracks[0].feature_points[0].d, 0.);
    ASSERT_EQ(res_depth_w.tracks[0].is_outlier, false);
    ASSERT_EQ(res_w.tracks[0].is_outlier, false);
    ASSERT_EQ(res_depth_w.tracks[1].is_outlier, true);
    ASSERT_EQ(res_w.tracks[1].is_outlier, true);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "unittest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
