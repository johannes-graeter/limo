#include <random>

#include <gtest/gtest.h>

#include <mono_lidar_depth/pca.hpp>

namespace {

using mono_lidar::PcaResult;
using mono_lidar::PlanarityPca;

constexpr double kThresholdPointMax = 0.005;
constexpr double kThresholdLineMin = 15.;
constexpr double kThresholdCubicMax = 0.5;

PlanarityPca makePca(const Eigen::MatrixXd& points) {
    return PlanarityPca(points, kThresholdPointMax, kThresholdLineMin, kThresholdCubicMax);
}

TEST(PlanarityPca, classifiesPlanarPatch) {
    // Points spread in x and y, flat in z.
    Eigen::MatrixXd points(3, 9);
    int col = 0;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            points.col(col++) = Eigen::Vector3d(x, y, 5. + 0.001 * x);
        }
    }

    const auto pca = makePca(points);

    EXPECT_EQ(pca.result(), PcaResult::plane);
    EXPECT_NEAR(std::abs(pca.planeNormal().z()), 1., 1e-3);
    EXPECT_NEAR(pca.planeAnchorPoint().z(), 5., 1e-2);
}

TEST(PlanarityPca, classifiesLine) {
    Eigen::MatrixXd points(3, 10);
    for (int i = 0; i < 10; ++i) {
        points.col(i) = Eigen::Vector3d(i, 0., 5.);
    }

    EXPECT_EQ(makePca(points).result(), PcaResult::linear);
}

TEST(PlanarityPca, classifiesVolumetricPatchAsCubic) {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> spread(-1., 1.);
    Eigen::MatrixXd points(3, 40);
    for (int i = 0; i < 40; ++i) {
        points.col(i) = Eigen::Vector3d(spread(rng), spread(rng), 5. + spread(rng));
    }

    EXPECT_EQ(makePca(points).result(), PcaResult::cubic);
}

} // namespace
