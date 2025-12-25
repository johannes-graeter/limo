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

#include "definitions.hpp"

#include "trimmer_fix.hpp"
#include "trimmer_quantile.hpp"

namespace robust_optimization {

/**
 * @brief This a factory that constructs a trimmer and applies it.
 *  if you implement a new trimmer, register it as a type and
 *  in apply_trimmer.
 * @param data; data to be trimmed,
 *              templated since we do not care about ids.
 * @param type; filter type to be applied.
 * @return outliers; return Ids of outliers.
 */
template <typename Id, typename... Args>
std::vector<Id> getOutliers(const std::map<Id, double>& data, TrimmerType type, Args... filter_args) {
    switch (type) {
    case TrimmerType::Fix: {
        TrimmerFix t(filter_args...);
        return t.getOutliers(data);
    }
    case TrimmerType::Quantile: {
        TrimmerQuantile t(filter_args...);
        return t.getOutliers(data);
    }
    default:
        std::runtime_error("In apply_trimmer: TrimmerType not defined");
    }

    return std::vector<Id>{};
}
}
