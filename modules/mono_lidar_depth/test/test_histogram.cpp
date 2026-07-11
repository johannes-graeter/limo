#include <gtest/gtest.h>

#include <mono_lidar_depth/histogram.hpp>

namespace {

using mono_lidar::segmentDepthBlob;
using mono_lidar::VecOfVec3d;

VecOfVec3d pointsAtDepths(const std::vector<double>& depths) {
    VecOfVec3d points;
    for (const double d : depths) {
        points.emplace_back(0., 0., d);
    }
    return points;
}

std::vector<int> iota(int n) {
    std::vector<int> indices(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        indices[static_cast<size_t>(i)] = i;
    }
    return indices;
}

Eigen::VectorXd toVector(const std::vector<double>& depths) {
    return Eigen::Map<const Eigen::VectorXd>(depths.data(), static_cast<Eigen::Index>(depths.size()));
}

TEST(DepthHistogram, binsElementsByWidth) {
    mono_lidar::DepthHistogram histogram(0.5, 10);
    histogram.addElement(0.1);  // bin 0
    histogram.addElement(0.6);  // bin 1
    histogram.addElement(0.7);  // bin 1
    histogram.addElement(99.0); // clamped into last bin

    EXPECT_EQ(histogram.binElementCount(0), 1);
    EXPECT_EQ(histogram.binElementCount(1), 2);
    EXPECT_EQ(histogram.binElementCount(9), 1);
}

TEST(DepthHistogram, rejectsInvalidConfiguration) {
    EXPECT_THROW(mono_lidar::DepthHistogram(0., 10), std::invalid_argument);
    EXPECT_THROW(mono_lidar::DepthHistogram(0.5, 0), std::invalid_argument);
}

TEST(SegmentDepthBlob, keepsNearestSufficientlyPopulatedBlob) {
    // Foreground object at ~5m (4 points), background at ~20m (3 points).
    const std::vector<double> depths{5.0, 5.1, 5.2, 5.3, 20.0, 20.1, 20.2};
    const auto points = pointsAtDepths(depths);

    const auto blob = segmentDepthBlob(points, iota(7), toVector(depths), 1.0, 3);

    ASSERT_TRUE(blob.has_value());
    ASSERT_EQ(blob->points.size(), 4u);
    for (const auto& point : blob->points) {
        EXPECT_LT(point.z(), 6.0);
    }
    EXPECT_EQ(blob->point_indices, (std::vector<int>{0, 1, 2, 3}));
}

TEST(SegmentDepthBlob, failsWithoutLocalMaximum) {
    // Two points per bin, minimum size 3: no local maximum qualifies.
    const std::vector<double> depths{5.0, 5.1, 20.0, 20.1};
    const auto points = pointsAtDepths(depths);

    const auto blob = segmentDepthBlob(points, iota(4), toVector(depths), 1.0, 3);

    EXPECT_FALSE(blob.has_value());
}

TEST(SegmentDepthBlob, rejectsIsolatedSpeckleInFront) {
    // A lone point in front of the object, separated by an empty bin, aborts the search.
    const std::vector<double> depths{2.0, 20.0, 20.1, 20.2, 20.3};
    const auto points = pointsAtDepths(depths);

    const auto blob = segmentDepthBlob(points, iota(5), toVector(depths), 1.0, 3);

    EXPECT_FALSE(blob.has_value());
}

TEST(SegmentDepthBlob, throwsOnSizeMismatch) {
    const std::vector<double> depths{1.0, 2.0};
    const auto points = pointsAtDepths({1.0});

    EXPECT_THROW(segmentDepthBlob(points, iota(1), toVector(depths), 1.0, 1), std::invalid_argument);
}

} // namespace
