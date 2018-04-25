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

    FeaturePoint(double u, double v) : u(u), v(v), d(-1) {
        ;
    }

    FeaturePoint(double u, double v, double d) : u(u), v(v), d(d) {
            ;
        }

    double u;
    double v;
    double d;

    Eigen::Vector2d toEigen2d() const {
        return Eigen::Vector2d(u, v);
    }

    Eigen::Vector3d toEigen3d() const {
        return Eigen::Vector3d(u, v, d);
    }

};
}
