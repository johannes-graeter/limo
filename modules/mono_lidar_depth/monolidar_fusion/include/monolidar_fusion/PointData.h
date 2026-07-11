#pragma once
#include <Eigen/Eigen>

class PointData {
public:
    // Specify Eigen Alignment, should be obsolete with c++17
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    using Ptr = std::shared_ptr<PointData>();
    using ConstPtr = std::shared_ptr<const PointData>();

public:
    Eigen::Matrix3Xf points;
    Eigen::Matrix2Xf coordinates;

    PointData() {
        points.setZero();
        coordinates.setZero();
    }

    PointData& operator=(PointData& in) {
        this->points = in.points;
        this->coordinates = in.coordinates;
        return *this;
    }
};
