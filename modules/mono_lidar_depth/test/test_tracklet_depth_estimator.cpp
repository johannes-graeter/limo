#include <gtest/gtest.h>

#include <mono_lidar_depth/tracklet_depth_estimator.hpp>

namespace {

using matches_msg_types::Tracklet;
using matches_msg_types::Tracklets;
using mono_lidar::CameraPinhole;
using mono_lidar::DepthEstimatorParameters;
using mono_lidar::TrackletDepthEstimator;

using Cloud = TrackletDepthEstimator::Cloud;

/// Fronto-parallel wall at z = 10 in camera coordinates (identity transform).
Cloud::Ptr makeWallCloud() {
    Cloud::Ptr cloud(new Cloud());
    for (float x = -2.f; x <= 2.f; x += 0.25f) {
        for (float y = -2.f; y <= 2.f; y += 0.25f) {
            pcl::PointXYZI point;
            point.x = x;
            point.y = y;
            point.z = 10.f;
            point.intensity = 1.f;
            cloud->push_back(point);
        }
    }
    return cloud;
}

TrackletDepthEstimator makeEstimator() {
    DepthEstimatorParameters parameters;
    parameters.use_ransac_plane = false; // wall scene has no ground plane
    auto camera = std::make_shared<CameraPinhole>(100, 100, 100., 50., 50.);
    return TrackletDepthEstimator(parameters, camera, Eigen::Affine3d::Identity());
}

Tracklet makeTracklet(uint64_t id, const std::vector<std::pair<float, float>>& points_newest_first) {
    Tracklet track;
    track.id = id;
    track.age = static_cast<unsigned long>(points_newest_first.size());
    track.label = 42;
    for (const auto& [u, v] : points_newest_first) {
        track.feature_points.emplace_back(u, v);
    }
    return track;
}

TEST(TrackletDepthEstimator, estimatesBothObservationsOfNewTracklet) {
    auto estimator = makeEstimator();

    Tracklets input;
    input.stamps = {1000u};
    input.tracks.push_back(makeTracklet(1, {{50.f, 50.f}, {52.f, 50.f}}));

    const auto out = estimator.process(makeWallCloud(), input);

    ASSERT_EQ(out.tracks.size(), 1u);
    ASSERT_EQ(out.tracks[0].feature_points.size(), 2u);
    // Newest observation: estimated with the current cloud.
    EXPECT_NEAR(out.tracks[0].feature_points[0].d, 10.f, 1e-3f);
    // Second-newest: no cloud of the previous frame available yet.
    EXPECT_FLOAT_EQ(out.tracks[0].feature_points[1].d, -1.f);
    // Metadata passes through (the seam for later semantic labels).
    EXPECT_EQ(out.tracks[0].label, 42);
    EXPECT_EQ(out.stamps, input.stamps);
}

TEST(TrackletDepthEstimator, cachesDepthsAcrossFrames) {
    auto estimator = makeEstimator();

    Tracklets frame1;
    frame1.stamps = {1000u};
    frame1.tracks.push_back(makeTracklet(1, {{50.f, 50.f}, {52.f, 50.f}}));
    estimator.process(makeWallCloud(), frame1);

    // Frame 2: track 1 extended by a new observation; track 2 appears for the first time, so its
    // second-newest observation is estimated with the cached cloud of frame 1.
    Tracklets frame2;
    frame2.stamps = {2000u, 1000u};
    frame2.tracks.push_back(makeTracklet(1, {{48.f, 50.f}, {50.f, 50.f}, {52.f, 50.f}}));
    frame2.tracks.push_back(makeTracklet(2, {{60.f, 60.f}, {62.f, 60.f}}));

    const auto out = estimator.process(makeWallCloud(), frame2);

    ASSERT_EQ(out.tracks.size(), 2u);
    // Track 1: three cached observations, all with valid depth except the very first one.
    ASSERT_EQ(out.tracks[0].feature_points.size(), 3u);
    EXPECT_NEAR(out.tracks[0].feature_points[0].d, 10.f, 1e-3f);
    EXPECT_NEAR(out.tracks[0].feature_points[1].d, 10.f, 1e-3f);
    EXPECT_FLOAT_EQ(out.tracks[0].feature_points[2].d, -1.f);
    // Track 2: both observations valid, the older one via the cached frame-1 cloud.
    ASSERT_EQ(out.tracks[1].feature_points.size(), 2u);
    EXPECT_NEAR(out.tracks[1].feature_points[0].d, 10.f, 1e-3f);
    EXPECT_NEAR(out.tracks[1].feature_points[1].d, 10.f, 1e-3f);
}

TEST(TrackletDepthEstimator, evictsTrackletsThatWereNotUpdated) {
    auto estimator = makeEstimator();

    Tracklets frame1;
    frame1.stamps = {1000u};
    frame1.tracks.push_back(makeTracklet(1, {{50.f, 50.f}, {52.f, 50.f}}));
    frame1.tracks.push_back(makeTracklet(2, {{60.f, 60.f}, {62.f, 60.f}}));
    estimator.process(makeWallCloud(), frame1);

    // Track 2 disappears -> evicted from the cache.
    Tracklets frame2;
    frame2.stamps = {2000u, 1000u};
    frame2.tracks.push_back(makeTracklet(1, {{48.f, 50.f}, {50.f, 50.f}, {52.f, 50.f}}));
    estimator.process(makeWallCloud(), frame2);

    // Track 2 reappears: it must be treated as new (only its two newest observations cached),
    // proving the cache was purged.
    Tracklets frame3;
    frame3.stamps = {3000u, 2000u, 1000u};
    frame3.tracks.push_back(makeTracklet(2, {{58.f, 60.f}, {60.f, 60.f}, {62.f, 60.f}}));

    const auto out = estimator.process(makeWallCloud(), frame3);

    ASSERT_EQ(out.tracks.size(), 1u);
    EXPECT_EQ(out.tracks[0].feature_points.size(), 2u);
}

} // namespace
