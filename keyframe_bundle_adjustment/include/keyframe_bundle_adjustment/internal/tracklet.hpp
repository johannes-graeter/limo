#pragma once

#include "feature_point.hpp"
namespace matches_msg_types {
struct Tracklet {
    std::vector<FeaturePoint> feature_points;

    unsigned long id;
    unsigned long age;
    bool is_outlier{false};
    int label{-2};
};
}
