#include <gtest/gtest.h>

#include <mono_lidar_depth/depth_estimator.hpp>

namespace {

using mono_lidar::CameraPinhole;
using mono_lidar::DepthEstimator;
using mono_lidar::DepthEstimatorParameters;
using mono_lidar::DepthResultType;
using mono_lidar::GroundPlane;

using Cloud = DepthEstimator::Cloud;

Cloud::Ptr makeCloud(const std::vector<Eigen::Vector3f>& points) {
    Cloud::Ptr cloud(new Cloud());
    for (const auto& p : points) {
        pcl::PointXYZI point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        point.intensity = 1.f;
        cloud->push_back(point);
    }
    return cloud;
}

CameraPinhole::Ptr makeCamera() {
    return std::make_shared<CameraPinhole>(100, 100, 100., 50., 50.);
}

/// Fronto-parallel wall of lidar points at z = 10 (identity lidar-to-camera transform).
TEST(DepthEstimator, estimatesDepthOfFrontoParallelWall) {
    std::vector<Eigen::Vector3f> points;
    for (float x = -2.f; x <= 2.f; x += 0.25f) {
        for (float y = -2.f; y <= 2.f; y += 0.25f) {
            points.emplace_back(x, y, 10.f);
        }
    }

    DepthEstimatorParameters parameters;
    parameters.use_ransac_plane = false; // scene has no ground plane
    DepthEstimator estimator(parameters, makeCamera(), Eigen::Affine3d::Identity());

    GroundPlane::Ptr no_ground_plane;
    Eigen::Matrix2Xd features(2, 2);
    features.col(0) = Eigen::Vector2d(50., 50.); // center of the wall
    features.col(1) = Eigen::Vector2d(65., 50.); // off-center, still on the wall

    const auto results = estimator.calculateDepths(makeCloud(points), features, no_ground_plane);

    ASSERT_EQ(results.depths.size(), 2);
    EXPECT_NEAR(results.depths(0), 10., 1e-6);
    EXPECT_NEAR(results.depths(1), 10., 1e-6);
    EXPECT_EQ(results.results[0], DepthResultType::success);

    EXPECT_EQ(estimator.statistics().frameCount(DepthResultType::success), 2);
    EXPECT_EQ(estimator.statistics().frameTotal(), 2);
}

TEST(DepthEstimator, failsWithoutEnoughNeighbors) {
    DepthEstimatorParameters parameters;
    parameters.use_ransac_plane = false;
    DepthEstimator estimator(parameters, makeCamera(), Eigen::Affine3d::Identity());

    GroundPlane::Ptr no_ground_plane;
    Eigen::Matrix2Xd features(2, 1);
    features.col(0) = Eigen::Vector2d(20., 20.); // far away from the only points

    const auto results =
        estimator.calculateDepths(makeCloud({{0.f, 0.f, 10.f}, {0.1f, 0.f, 10.f}}), features, no_ground_plane);

    EXPECT_DOUBLE_EQ(results.depths(0), -1.);
    EXPECT_EQ(results.results[0], DepthResultType::insufficient_neighbors);
}

TEST(DepthEstimator, throwsWhenCloudNotSet) {
    DepthEstimatorParameters parameters;
    parameters.use_ransac_plane = false;
    DepthEstimator estimator(parameters, makeCamera(), Eigen::Affine3d::Identity());

    Eigen::Matrix2Xd features(2, 1);
    features.col(0) = Eigen::Vector2d(50., 50.);

    EXPECT_THROW(estimator.calculateDepths(features, nullptr), std::logic_error);
}

/// Road scene in the lidar frame (x forward, z up): sparse ground rows force the road fallback,
/// which intersects the viewing ray with the RANSAC ground plane.
TEST(DepthEstimator, roadFallbackRecoversGroundDepth) {
    // Ground plane at z = -1.5 with laterally narrow rows -> the max spanning triangle of a
    // single row is degenerate, so the plain path fails and the road fallback kicks in.
    std::vector<Eigen::Vector3f> points;
    for (float x = 4.f; x <= 25.f; x += 0.5f) {
        for (float y = -0.2f; y <= 0.2f; y += 0.2f) {
            points.emplace_back(x, y, -1.5f);
        }
    }

    // cam_x = -y_lidar, cam_y = -z_lidar, cam_z = x_lidar
    Eigen::Matrix3d rotation;
    rotation << 0., -1., 0., 0., 0., -1., 1., 0., 0.;
    const Eigen::Affine3d transform_lidar_to_cam(rotation);

    DepthEstimatorParameters parameters;
    DepthEstimator estimator(parameters, makeCamera(), transform_lidar_to_cam);

    // Feature at the projection of the ground point 10m ahead: v = f * 1.5 / 10 + cv = 65.
    Eigen::Matrix2Xd features(2, 1);
    features.col(0) = Eigen::Vector2d(50., 65.);

    GroundPlane::Ptr ground_plane; // created internally via RANSAC
    const auto results = estimator.calculateDepths(makeCloud(points), features, ground_plane);

    ASSERT_NE(ground_plane, nullptr);
    EXPECT_TRUE(ground_plane->isSegmented());
    EXPECT_NEAR(results.depths(0), 10., 0.1);
}

} // namespace
