#include <random>

#include <gtest/gtest.h>

#include <mono_lidar_depth/plane_estimation.hpp>

namespace {

using mono_lidar::fitPlaneLeastSquares;
using mono_lidar::fitPlaneMEstimator;
using mono_lidar::isTrianglePlanar;
using mono_lidar::maxSpanningTriangle;
using mono_lidar::VecOfVec3d;

TEST(MaxSpanningTriangle, picksTheExtremePoints) {
    VecOfVec3d points;
    points.emplace_back(0., 0., 0.);
    points.emplace_back(0.1, 0., 0.);
    points.emplace_back(10., 0., 0.); // farthest pair: (0,0,0) - (10,0,0)
    points.emplace_back(5., 8., 0.);  // maximizes summed distance to the pair

    const auto triangle = maxSpanningTriangle(points);

    ASSERT_TRUE(triangle.has_value());
    EXPECT_EQ((*triangle)[0], points[0]);
    EXPECT_EQ((*triangle)[1], points[2]);
    EXPECT_EQ((*triangle)[2], points[3]);
}

TEST(MaxSpanningTriangle, failsForFewerThanThreePoints) {
    VecOfVec3d points;
    points.emplace_back(0., 0., 0.);
    points.emplace_back(1., 0., 0.);

    EXPECT_FALSE(maxSpanningTriangle(points).has_value());
}

TEST(MaxSpanningTriangle, failsWhenAllPointsCoincide) {
    VecOfVec3d points(4, Eigen::Vector3d(1., 2., 3.));

    EXPECT_FALSE(maxSpanningTriangle(points, 0.01).has_value());
}

TEST(IsTrianglePlanar, acceptsProperTriangleRejectsDegenerate) {
    EXPECT_TRUE(isTrianglePlanar(
        Eigen::Vector3d(0., 0., 0.), Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 1., 0.), 0.1));

    // Collinear corners span no plane.
    EXPECT_FALSE(isTrianglePlanar(
        Eigen::Vector3d(0., 0., 0.), Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(2., 0., 0.), 0.1));
}

// Regression test for the former ceres-based ErrorPlane fit: its residual dropped a parenthesis
// (only the z term was divided by the normal's norm), read the plane distance from out-of-bounds
// memory and started from a zero normal. The replacement must recover a known plane exactly.
TEST(FitPlaneLeastSquares, recoversKnownPlane) {
    // Points on the plane z = 2 + 0.1x - 0.2y.
    VecOfVec3d points;
    for (int x = -3; x <= 3; ++x) {
        for (int y = -3; y <= 3; ++y) {
            points.emplace_back(x, y, 2. + 0.1 * x - 0.2 * y);
        }
    }

    const auto plane = fitPlaneLeastSquares(points);

    ASSERT_TRUE(plane.has_value());
    for (const auto& point : points) {
        EXPECT_NEAR(plane->absDistance(point), 0., 1e-10);
    }
}

TEST(FitPlaneLeastSquares, failsForFewerThanThreePoints) {
    VecOfVec3d points;
    points.emplace_back(0., 0., 0.);
    points.emplace_back(1., 0., 0.);

    EXPECT_FALSE(fitPlaneLeastSquares(points).has_value());
}

TEST(FitPlaneMEstimator, downweightsOutliersRelativeToPrior) {
    // Ground plane y = 1.5 (camera coordinates, y down) plus a strong off-plane outlier.
    VecOfVec3d points;
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> jitter(-0.01, 0.01);
    for (int i = 0; i < 30; ++i) {
        points.emplace_back(jitter(rng) * 100., 1.5 + jitter(rng), 5. + 0.3 * i);
    }
    points.emplace_back(0., -3., 10.); // outlier far from the prior

    const Eigen::Hyperplane<double, 3> prior(Eigen::Vector3d(0., 1., 0.), -1.5);
    const auto plane = fitPlaneMEstimator(points, prior);

    ASSERT_TRUE(plane.has_value());
    // The fitted plane must stay close to the true ground plane despite the outlier.
    EXPECT_NEAR(std::abs(plane->normal().y()), 1., 1e-3);
    EXPECT_NEAR(plane->absDistance(Eigen::Vector3d(0., 1.5, 20.)), 0., 0.05);
}

} // namespace
