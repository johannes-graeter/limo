#include <gtest/gtest.h>

#include <mono_lidar_depth/neighbor_finder.hpp>

namespace {

using mono_lidar::PixelNeighborFinder;
using mono_lidar::ProjectedPointcloud;

/// Cloud with points on a pixel grid at the given image coordinates, all at depth z = 10.
ProjectedPointcloud makeProjected(const std::vector<std::pair<double, double>>& image_points) {
    ProjectedPointcloud points;
    const auto n = static_cast<Eigen::Index>(image_points.size());
    points.points_camera_cs.resize(3, n);
    points.visible_image_cs.resize(2, n);
    for (Eigen::Index i = 0; i < n; ++i) {
        points.visible_image_cs(0, i) = image_points[static_cast<size_t>(i)].first;
        points.visible_image_cs(1, i) = image_points[static_cast<size_t>(i)].second;
        points.points_camera_cs.col(i) = Eigen::Vector3d(0., 0., 10.);
        points.visible_to_cloud.push_back(static_cast<int>(i));
    }
    return points;
}

TEST(PixelNeighborFinder, findsPointsInsideSearchRectangleOnly) {
    PixelNeighborFinder finder(100, 100, 4, 4);

    const auto points = makeProjected({{50., 50.}, {51., 51.}, {49., 48.}, {80., 80.}});
    finder.initializeProjection(points);

    const auto indices = finder.findNeighborIndices(Eigen::Vector2d(50., 50.));

    EXPECT_EQ(indices.size(), 3u); // the far point at (80, 80) is excluded
    for (const int index : indices) {
        EXPECT_NE(index, 3);
    }
}

TEST(PixelNeighborFinder, scaleEnlargesSearchRectangle) {
    PixelNeighborFinder finder(100, 100, 4, 4);

    const auto points = makeProjected({{50., 50.}, {56., 50.}});
    finder.initializeProjection(points);

    EXPECT_EQ(finder.findNeighborIndices(Eigen::Vector2d(50., 50.)).size(), 1u);
    EXPECT_EQ(finder.findNeighborIndices(Eigen::Vector2d(50., 50.), 4.f, 1.f).size(), 2u);
}

TEST(PixelNeighborFinder, ignoresPointsBehindCamera) {
    PixelNeighborFinder finder(100, 100, 4, 4);

    auto points = makeProjected({{50., 50.}});
    points.points_camera_cs.col(0) = Eigen::Vector3d(0., 0., -10.);
    finder.initializeProjection(points);

    EXPECT_TRUE(finder.findNeighborIndices(Eigen::Vector2d(50., 50.)).empty());
}

TEST(PixelNeighborFinder, clampsSearchRectangleToImageBorder) {
    PixelNeighborFinder finder(100, 100, 10, 10);

    const auto points = makeProjected({{1., 1.}, {99., 99.}});
    finder.initializeProjection(points);

    EXPECT_EQ(finder.findNeighborIndices(Eigen::Vector2d(0., 0.)).size(), 1u);
    EXPECT_EQ(finder.findNeighborIndices(Eigen::Vector2d(99., 99.)).size(), 1u);
}

TEST(PixelNeighborFinder, collectPointsReturnsCameraCoordinates) {
    const auto points = makeProjected({{10., 10.}, {20., 20.}});

    const auto collected = PixelNeighborFinder::collectPoints(points, {1});

    ASSERT_EQ(collected.size(), 1u);
    EXPECT_EQ(collected[0], Eigen::Vector3d(0., 0., 10.));
}

} // namespace
