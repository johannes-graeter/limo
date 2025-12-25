#pragma once
#include <Eigen/Eigen>
#include "feature_point.hpp"
namespace matches_msg_types {
struct FeaturePointDepth : public FeaturePoint {
    FeaturePointDepth() {
        ;
    }

    FeaturePointDepth(Eigen::Vector3d p)
	: FeaturePoint(p[0], p[1]), d(p[2]) {
        ;
    }

    FeaturePointDepth(float u, float v, float d)
	: FeaturePoint(u, v), d(d) {
        ;
    }

	float d;
};
}
