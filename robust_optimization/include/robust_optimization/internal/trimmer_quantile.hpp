// Copyright 2018. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once
#include <algorithm>
#include <map>
#include <vector>

namespace robust_optimization {
/**
* @class TrimmerQuantile
* @par
*
* Trimms data with quantiles.
* if quantile=0.9, the highest 10 % of redisuals are outliers
*/
class TrimmerQuantile {
public: // Public classes/enums/types etc...
public: // Attributes.
public: // Public methods.
    // Default constructor.
    explicit TrimmerQuantile(double quantile) : quantile_(quantile) {
        ;
    }

    // Default move.
    TrimmerQuantile(TrimmerQuantile&& other) = default;
    TrimmerQuantile& operator=(TrimmerQuantile&& other) = default;

    // Default copy.
    TrimmerQuantile(const TrimmerQuantile& other) = default;
    TrimmerQuantile& operator=(const TrimmerQuantile& other) = default;

    template <typename Id>
    std::vector<Id> getOutliers(const std::map<Id, double>& residuals_input) {
        // Copy map to vector for sorting.
        std::vector<std::pair<Id, double>> input_vec;
        input_vec.resize(residuals_input.size());
        std::transform(
            residuals_input.cbegin(), residuals_input.cend(), input_vec.begin(), [](const auto& a) { return a; });

        // Calc element number that corresponds to quantile.
        int num = static_cast<int>(static_cast<double>(input_vec.size()) * quantile_);

        // From http://en.cppreference.com/w/cpp/algorithm/nth_element
        // All of the elements before this new nth element are less than or equal to the elements after the new nth
        // element.
        std::nth_element(input_vec.begin(), input_vec.begin() + num, input_vec.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

        std::vector<Id> outliers;
        for (auto it = input_vec.cbegin() + num; it != input_vec.cend(); it++) {
            outliers.push_back(it->first);
        }
        return outliers;
    }

private:
    double quantile_; ///< Quantile for rejection, between 0. and 1.0, 0.9 means 10% largest residuals are rejected.
};
}
