#include <gtest/gtest.h>

#include <mono_lidar_depth/depth_filter.hpp>

namespace {

using mono_lidar::DepthThresholdGlobal;
using mono_lidar::DepthThresholdLocal;
using mono_lidar::ThresholdMode;
using mono_lidar::ThresholdResult;
using mono_lidar::ThresholdToleranceType;
using mono_lidar::VecOfVec3d;

TEST(DepthThresholdGlobal, disposeInvalidatesOutOfBoundsDepth) {
    const DepthThresholdGlobal threshold(ThresholdMode::dispose, 1., 100.);

    double depth = 50.;
    EXPECT_EQ(threshold.check(depth), ThresholdResult::in_bounds);
    EXPECT_DOUBLE_EQ(depth, 50.);

    depth = 0.5;
    EXPECT_EQ(threshold.check(depth), ThresholdResult::smaller_min);
    EXPECT_DOUBLE_EQ(depth, -1.);

    depth = 150.;
    EXPECT_EQ(threshold.check(depth), ThresholdResult::greater_max);
    EXPECT_DOUBLE_EQ(depth, -1.);
}

TEST(DepthThresholdGlobal, adjustClampsToBounds) {
    const DepthThresholdGlobal threshold(ThresholdMode::adjust, 1., 100.);

    double depth = 0.5;
    EXPECT_EQ(threshold.check(depth), ThresholdResult::in_bounds);
    EXPECT_DOUBLE_EQ(depth, 1.);

    depth = 150.;
    EXPECT_EQ(threshold.check(depth), ThresholdResult::in_bounds);
    EXPECT_DOUBLE_EQ(depth, 100.);
}

VecOfVec3d supportingPoints() {
    // Depth interval spanned by the points: [10, 12].
    VecOfVec3d points;
    points.emplace_back(0., 0., 10.);
    points.emplace_back(1., 0., 11.);
    points.emplace_back(0., 1., 12.);
    return points;
}

TEST(DepthThresholdLocal, absoluteToleranceBoundsDepthToSupportInterval) {
    const DepthThresholdLocal threshold(ThresholdMode::dispose, ThresholdToleranceType::absolute, 1.);

    double depth = 12.5; // inside [9, 13]
    EXPECT_EQ(threshold.check(supportingPoints(), depth), ThresholdResult::in_bounds);
    EXPECT_DOUBLE_EQ(depth, 12.5);

    depth = 14.; // outside
    EXPECT_EQ(threshold.check(supportingPoints(), depth), ThresholdResult::greater_max);
    EXPECT_DOUBLE_EQ(depth, -1.);

    depth = 8.; // outside
    EXPECT_EQ(threshold.check(supportingPoints(), depth), ThresholdResult::smaller_min);
    EXPECT_DOUBLE_EQ(depth, -1.);
}

TEST(DepthThresholdLocal, relativeToleranceScalesWithIntervalSize) {
    // Interval [10, 12] (size 2), tolerance 0.5 relative -> [9, 13].
    const DepthThresholdLocal threshold(ThresholdMode::dispose, ThresholdToleranceType::relative, 0.5);

    double depth = 9.5;
    EXPECT_EQ(threshold.check(supportingPoints(), depth), ThresholdResult::in_bounds);

    depth = 13.5;
    EXPECT_EQ(threshold.check(supportingPoints(), depth), ThresholdResult::greater_max);
}

} // namespace
