// Copyright 2018. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include "definitions.hpp"

namespace keyframe_bundle_adjustment {
namespace regularization {

namespace {
namespace center_of_rotation {
template <typename T>
T getDeltaY(const T& d_yaw, const T& d_x) {
    // Radius of rotation r=arclength/yaw, movement=[x,y]=[r*sin(yaw), r*(1-cos(yaw))], residual is hence
    // y_{measured}-y_{motion_model} with y_{motion_model}=r*(1-cos(yaw))=x/sin(yaw)*(1-cos(yaw))
    if (ceres::abs(d_yaw - T(0.)) < T(1.e-6)) {
        return T(0.);
    }
    return d_x / ceres::sin(d_yaw) * (T(1.) - ceres::cos(d_yaw));
}
}
}

class MotionModelRegularization {
public: // public methods
    //    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // This is not needed since no eigen stuff is a member.
    // default constructor
    MotionModelRegularization() = default;

    template <typename T>
    bool operator()(const T* const pose_origin_from_keyframe1,
                    const T* const pose_origin_from_keyframe0,
                    T* residual) const {
        using Pose = Eigen::Transform<T, 3, Eigen::Isometry>;
        Pose p_O_K0 = convert(pose_origin_from_keyframe0);
        Pose p_O_K1 = convert(pose_origin_from_keyframe1);

        Pose motion_K0_K1 = p_O_K0.inverse() * p_O_K1;

        // Eigen::eulerAngles is aorund moving axes however dynamic "ZYX" should be identical to static "XYZ" accoridng
        // to this
        // https://stackoverflow.com/questions/27508242/roll-pitch-and-yaw-from-rotation-matrix-with-eigen-library
        Eigen::Matrix<T, 3, 1> yrp = motion_K0_K1.rotation().eulerAngles(2, 1, 0);
        //        std::cout << yrp[0] << " " << yrp[1] << " " << yrp[2] << " " << std::endl;
        T d_y_motion_model = center_of_rotation::getDeltaY(yrp[0], motion_K0_K1.translation()[0]);

        std::cout << d_y_motion_model << std::endl;
        std::cout << motion_K0_K1.translation()[0] << " " << motion_K0_K1.translation()[1] << std::endl;
        residual[0] = motion_K0_K1.translation()[1] - d_y_motion_model;

        return true;
    }

    static ceres::CostFunction* Create() {
        return (new ceres::AutoDiffCostFunction<MotionModelRegularization, 1, 7, 7>(new MotionModelRegularization()));
    }
};
}
}
