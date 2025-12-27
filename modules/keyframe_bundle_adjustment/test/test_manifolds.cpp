#include <gtest/gtest.h>
#include "keyframe_bundle_adjustment/internal/local_parameterizations.hpp"
#include <Eigen/Core>

using namespace keyframe_bundle_adjustment::local_parameterizations;

TEST(FixScaleVectorPlusManifold, PlusPreservesNorm) {
    FixScaleVectorPlus manifold(2.0);
    double x[3] = {1.0, 0.0, 0.0};
    double delta[3] = {1.0, 1.0, 0.0};
    double x_plus_delta[3];
    manifold.Plus(x, delta, x_plus_delta);
    double norm = std::sqrt(x_plus_delta[0]*x_plus_delta[0] + x_plus_delta[1]*x_plus_delta[1] + x_plus_delta[2]*x_plus_delta[2]);
    EXPECT_NEAR(norm, 2.0, 1e-9);
}

TEST(CircularMotionPlus2dManifold, PlusOutputSize) {
    CircularMotionPlus2d manifold;
    double x[7] = {1,0,0,0,0,0,0};
    double delta[2] = {0.1, 1.0};
    double x_plus_delta[7];
    bool ok = manifold.Plus(x, delta, x_plus_delta);
    EXPECT_TRUE(ok);
}
