// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
// List of some basic tests fuctions:
// Fatal assertion                      Nonfatal assertion                   Verifies / Description
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// ASSERT_EQ(expected, actual);         EXPECT_EQ(expected, actual);         expected == actual
// ASSERT_NE(val1, val2);               EXPECT_NE(val1, val2);               val1 != val2
// ASSERT_LT(val1, val2);               EXPECT_LT(val1, val2);               val1 < val2
// ASSERT_LE(val1, val2);               EXPECT_LE(val1, val2);               val1 <= val2
// ASSERT_GT(val1, val2);               EXPECT_GT(val1, val2);               val1 > val2
// ASSERT_GE(val1, val2);               EXPECT_GE(val1, val2);               val1 >= val2
//
// ASSERT_FLOAT_EQ(expected, actual);   EXPECT_FLOAT_EQ(expected, actual);   the two float values are almost equal (4
// ULPs)
// ASSERT_DOUBLE_EQ(expected, actual);  EXPECT_DOUBLE_EQ(expected, actual);  the two double values are almost equal (4
// ULPs)
// ASSERT_NEAR(val1, val2, abs_error);  EXPECT_NEAR(val1, val2, abs_error);  the difference between val1 and val2
// doesn't exceed the given absolute error
//
// Note: more information about ULPs can be found here:
// http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
//
// Example of two unit test:
// TEST(Math, Add) {
//    ASSERT_EQ(10, 5+ 5);
//}
//
// TEST(Math, Float) {
//	  ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f)
//}
//=======================================================================================================================================================
#include <random>
#include <robust_solving.hpp>
#include <ceres/ceres.h>
#include <internal/apply_trimmer.hpp>
#include "gtest/gtest.h"

using IdResMap = std::map<int, double>;

namespace {
IdResMap makeResiduals(int start_id, int num, double mean, double dev, double min, double max) {
    // construct normal distribution
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> Distribution(mean, dev);

    IdResMap out;
    for (int i = 0; i < int(num); ++i) {
        // Generate number from distribution
        double val = Distribution(gen);
        val = std::min(val, min);
        val = std::max(val, max);

        out[start_id + i] = val;
    }
    return out;
}

IdResMap makeData() {
    double res_thres = 3.5;
    int num_outliers = 10;
    auto outliers = makeResiduals(0, num_outliers, 5., 1., res_thres + 0.1, 100.);
    auto inliers = makeResiduals(num_outliers, 100, 0., 1., 0., res_thres - 0.1);
    IdResMap input = outliers;
    std::copy(inliers.cbegin(), inliers.cend(), std::inserter(input, input.end()));

    return input;
}

void printData(const IdResMap& data) {
    for (const auto& el : data) {
        std::cout << el.first << " " << el.second;
    }
    std::cout << std::endl;
}

void printData(const std::vector<int>& data) {
    for (const auto& el : data) {
        std::cout << el << " ";
    }
    std::cout << std::endl;
}
}
TEST(Trimmers, TrimmerFix) {
    auto input = makeData();
    printData(input);

    auto outliers = robust_optimization::getOutliers(input, robust_optimization::TrimmerType::Fix, 3.5);
    ASSERT_EQ(outliers.size(), 10);
    std::cout << "Trimmer fix outliers:" << std::endl;
    printData(outliers);
}

TEST(Trimmers, TrimmerQuantile) {
    auto input = makeData();
    auto outliers = robust_optimization::getOutliers(input, robust_optimization::TrimmerType::Quantile, 0.9);

    ASSERT_EQ(outliers.size(), 11);

    std::cout << "after trimmer quantile" << std::endl;
    printData(outliers);
}

namespace {
struct TestFuncInlier {
    template <typename T>
    bool operator()(const T* const param, T* res) const {
        res[0] = T(3.0) * param[0];
        return true;
    }

    static ceres::CostFunction* Create() {
        return (new ceres::AutoDiffCostFunction<TestFuncInlier, 1, 1>(new TestFuncInlier()));
    }
};

struct TestFuncOutlier {
    template <typename T>
    bool operator()(const T* const param, T* res) const {
        res[0] = T(10.);
        return true;
    }

    static ceres::CostFunction* Create() {
        return (new ceres::AutoDiffCostFunction<TestFuncOutlier, 1, 1>(new TestFuncOutlier()));
    }
};
}
TEST(Solve, trimmed) {
    ceres::Problem prob;
    int num_inliers = 90;
    int num_outliers = 10;
    double x = 2.0;
    robust_optimization::ResidualIds ids;
    for (int i = 0; i < int(num_inliers); ++i) {
        ceres::CostFunction* test_func = TestFuncInlier::Create();
        auto id = prob.AddResidualBlock(test_func, NULL, &x);
        ids.push_back(std::make_pair(id, 1));
    }

    for (int i = 0; i < int(num_outliers); ++i) {
        ceres::CostFunction* test_func = TestFuncOutlier::Create();
        auto id = prob.AddResidualBlock(test_func, NULL, &x);
        ids.push_back(std::make_pair(id, 1));
    }

    robust_optimization::TrimmerSpecification spec{robust_optimization::TrimmerType::Quantile, 0.9};
    robust_optimization::solveTrimmed(std::vector<int>{0, 2}, ids, spec, prob);

    ASSERT_NEAR(x, 0., 0.001);
}
