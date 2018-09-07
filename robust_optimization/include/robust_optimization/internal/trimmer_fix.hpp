// Copyright 2018. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <map>
#include <vector>

namespace robust_optimization {
/**
* @class TrimmerFix
* @par
*
* Most simple concrete trimmer.
* Uses fix residual threshold for outlier rejection.
*/
class TrimmerFix {
public: // Public methods.
    // Default constructor.
    explicit TrimmerFix(double outlier_threshold) : outlier_thres_(outlier_threshold) {
        ;
    }

    // Default move.
    TrimmerFix(TrimmerFix&& other) = default;
    TrimmerFix& operator=(TrimmerFix&& other) = default;

    // Default copy.
    TrimmerFix(const TrimmerFix& other) = default;
    TrimmerFix& operator=(const TrimmerFix& other) = default;

    template <typename Id>
    std::vector<Id> getOutliers(const std::map<Id, double>& residuals_input) {
        std::vector<Id> outliers;
        for (const auto& el : residuals_input) {
            if (el.second > outlier_thres_) {
                outliers.push_back(el.first);
            }
        }
        return outliers;
    }

private:
    double outlier_thres_; ///< Fix threshold. Residuals > thres will be rejected.
};
} // end of namespace
