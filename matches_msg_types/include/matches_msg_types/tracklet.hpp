#pragma once

#include "feature_point.hpp"
namespace matches_msg_types {
struct Tracklet {
    std::vector<FeaturePoint> feature_points;

    unsigned long id;
    unsigned long age;
    bool is_outlier{false};
    int label{-2};

    bool operator==(const Tracklet& rhs) const {
        return feature_points == rhs.feature_points && id == rhs.id 
	       && age == rhs.age && is_outlier == rhs.is_outlier && label == rhs.label;
    }
};
}
