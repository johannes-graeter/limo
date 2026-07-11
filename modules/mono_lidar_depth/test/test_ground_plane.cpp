#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <mono_lidar_depth/ground_plane.hpp>

namespace {

using Cloud = mono_lidar::GroundPlane::Cloud;
using Point = mono_lidar::GroundPlane::Point;

Point makePoint(float x, float y, float z) {
    Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 1.f;
    return point;
}

/// Lidar frame (z up): dense ground plane at z = -1.5 plus a vertical wall at x = 10 that stays
/// above the sensor horizon (z >= 0.5).
Cloud::Ptr makeGroundAndWallCloud() {
    Cloud::Ptr cloud(new Cloud());
    for (float x = 2.f; x < 9.f; x += 0.25f) {
        for (float y = -3.f; y <= 3.f; y += 0.25f) {
            cloud->push_back(makePoint(x, y, -1.5f));
        }
    }
    for (float y = -3.f; y <= 3.f; y += 0.5f) {
        for (float z = 0.5f; z <= 2.5f; z += 0.5f) {
            cloud->push_back(makePoint(10.f, y, z));
        }
    }
    return cloud;
}

TEST(RansacPlane, findsGroundPlaneAmongWallPoints) {
    mono_lidar::DepthEstimatorParameters parameters;
    // The default refinement threshold (10.2m) deliberately keeps almost everything as inlier;
    // tighten it so inlier classification is testable.
    parameters.ransac_plane_refinement_threshold = 0.3;
    mono_lidar::RansacPlane plane(parameters);

    const auto cloud = makeGroundAndWallCloud();
    plane.calculateInliersPlane(cloud, parameters.ransac_plane_min_z, parameters.ransac_plane_max_z);

    ASSERT_TRUE(plane.isSegmented());

    // Normal along z, plane at z = -1.5 (up to sign convention).
    const Eigen::Vector4f& coeffs = plane.modelCoeffs();
    EXPECT_NEAR(std::abs(coeffs[2]), 1.f, 0.05f);
    EXPECT_NEAR(std::abs(coeffs[3]), 1.5f, 0.1f);

    // Ground points are inliers, wall points are not.
    EXPECT_TRUE(plane.isPointInPlane(0));
    EXPECT_FALSE(plane.inlierIndices().empty());
    const int wall_index = static_cast<int>(cloud->size()) - 1;
    EXPECT_FALSE(plane.isPointInPlane(wall_index));
}

TEST(RansacPlane, throwsOnDegenerateCloud) {
    mono_lidar::DepthEstimatorParameters parameters;
    mono_lidar::RansacPlane plane(parameters);

    Cloud::Ptr cloud(new Cloud());
    cloud->push_back(makePoint(0.f, 0.f, 0.f));

    EXPECT_THROW(plane.calculateInliersPlane(cloud), mono_lidar::GroundPlane::ExceptionPclInvalid);
}

TEST(SemanticPlane, fitsPlaneThroughLabeledGroundPoints) {
    // Camera looking forward (lidar x): cam_x = -y_lidar, cam_y = -z_lidar, cam_z = x_lidar.
    mono_lidar::SemanticPlane::Camera camera;
    camera.f = 100.;
    camera.cu = 50.;
    camera.cv = 50.;
    Eigen::Matrix3d rotation;
    rotation << 0., -1., 0., 0., 0., -1., 1., 0., 0.;
    camera.transform_cam_lidar = Eigen::Affine3d(rotation);

    // Ground pixels (label 7) in the lower image half, everything else labeled as building (11).
    cv::Mat semantic_image(100, 100, CV_8UC1, cv::Scalar(11));
    semantic_image.rowRange(50, 100).setTo(7);

    const auto cloud = makeGroundAndWallCloud();
    mono_lidar::SemanticPlane plane(semantic_image, camera, {6, 7, 8, 9}, 0.2);
    plane.calculateInliersPlane(cloud);

    ASSERT_TRUE(plane.isSegmented());
    const Eigen::Vector4f& coeffs = plane.modelCoeffs();
    EXPECT_NEAR(std::abs(coeffs[2]), 1.f, 0.05f);
    EXPECT_NEAR(std::abs(coeffs[3]), 1.5f, 0.1f);
}

} // namespace
