#include <gtest/gtest.h>

#include <mono_lidar_depth/line_plane_intersection.hpp>

namespace {

using mono_lidar::checkZxRatio;
using mono_lidar::LinePlaneIntersection;
using mono_lidar::VecOfVec3d;

TEST(LinePlaneIntersection, intersectsFrontoParallelPlane) {
    // Plane z = 10, ray along the optical axis.
    const LinePlaneIntersection intersection;
    const auto result = intersection.intersect(
        Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., 10.), Eigen::Vector3d::Zero(), Eigen::Vector3d(0., 0., 1.));

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->depth, 10., 1e-12);
    EXPECT_NEAR((result->point - Eigen::Vector3d(0., 0., 10.)).norm(), 0., 1e-12);
}

TEST(LinePlaneIntersection, depthIsZCoordinateForObliqueRays) {
    // Plane z = 10, oblique ray: the intersection z stays 10 (the depth convention), while the
    // euclidean distance along the ray is larger.
    const LinePlaneIntersection intersection;
    const Eigen::Vector3d direction = Eigen::Vector3d(1., 0., 1.).normalized();
    const auto result = intersection.intersect(
        Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., 10.), Eigen::Vector3d::Zero(), direction);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->depth, 10., 1e-12);
    EXPECT_NEAR(result->point.x(), 10., 1e-12);
}

TEST(LinePlaneIntersection, intersectsPlaneThroughThreePoints) {
    const LinePlaneIntersection intersection;
    const auto result = intersection.intersect(Eigen::Vector3d(1., 0., 5.),
                                               Eigen::Vector3d(-1., 0., 5.),
                                               Eigen::Vector3d(0., 1., 5.),
                                               Eigen::Vector3d::Zero(),
                                               Eigen::Vector3d(0., 0., 1.));

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->depth, 5., 1e-12);
}

TEST(LinePlaneIntersection, rejectsNearParallelRayWithThreshold) {
    // Ray direction almost perpendicular to the plane normal.
    const LinePlaneIntersection intersection(0.5);
    const Eigen::Vector3d ray = Eigen::Vector3d(1., 0., 0.01).normalized();
    const auto result =
        intersection.intersect(Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., 10.), Eigen::Vector3d::Zero(), ray);

    EXPECT_FALSE(result.has_value());
}

TEST(CheckZxRatio, comparesPatchExtents) {
    VecOfVec3d points;
    points.emplace_back(0., 0., 0.);
    points.emplace_back(1., 0., 3.);

    EXPECT_TRUE(checkZxRatio(points, 2.));  // z/x extent = 3
    EXPECT_FALSE(checkZxRatio(points, 4.));
}

} // namespace
