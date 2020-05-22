#pragma once
#include <Eigen/Eigen>
namespace matches_msg_types {
struct FeaturePoint {
    FeaturePoint() {
        ;
    }
    FeaturePoint(Eigen::Vector2d p) : u(p[0]), v(p[1]), d(-1) {
        ;
    }

    FeaturePoint(Eigen::Vector3d p) : u(p[0]), v(p[1]), d(p[2]) {
        ;
    }

    FeaturePoint(float u, float v) : u(u), v(v), d(-1) {
        ;
    }

    FeaturePoint(float u, float v, float d) : u(u), v(v), d(d) {
            ;
        }
    
    bool operator==(const FeaturePoint& rhs) const{
        return u==rhs.u && v==rhs.v && d==rhs.d;
    }

    float u;
    float v;
    float d{-1};

    Eigen::Vector2d toEigen2d() const {
        return Eigen::Vector2d(u, v);
    }

    Eigen::Vector3d toEigen3d() const {
        return Eigen::Vector3d(u, v, d);
    }

};
}
