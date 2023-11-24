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
    bool operator()(const T* const pose_keyframe1_origin, const T* const pose_keyframe0_origin, T* residual) const {
        using Pose = Eigen::Transform<T, 3, Eigen::Isometry>;
        Pose p_K0_O = convert(pose_keyframe0_origin);
        Pose p_K1_O = convert(pose_keyframe1_origin);

        Pose motion_K1_K0 = p_K1_O * p_K0_O.inverse();

        // Eigen::eulerAngles is aorund moving axes however dynamic "ZYX" should be identical to static "XYZ" accoridng
        // to this
        // https://stackoverflow.com/questions/27508242/roll-pitch-and-yaw-from-rotation-matrix-with-eigen-library
        //        Eigen::Matrix<T, 3, 1> yrp = motion_K0_K1.rotation().eulerAngles(2, 1, 0);

        // Get yaw from quaternion.
        // https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        Eigen::Quaternion<T> q(motion_K1_K0.rotation());
        q.x() = T(0.);
        q.y() = T(0.);
        q.normalize();

        T sign = q.z() < T(0.) ? T(-1.) : T(1.);
        T yaw = sign * 2. * ceres::acos(q.w());

        T d_y_motion_model = center_of_rotation::getDeltaY(yaw, motion_K1_K0.translation()[0]);

        if (std::is_same<T, double>::value) {
            std::cout << "y motion_model=" << d_y_motion_model << std::endl;
            std::cout << "delta_motion translation=" << motion_K1_K0.translation()[0] << " "
                      << motion_K1_K0.translation()[1] << std::endl;
            std::cout << "yaw=" << yaw << std::endl;
        }
        residual[0] = motion_K1_K0.translation()[1] - d_y_motion_model;
        residual[1] = motion_K1_K0.translation()[2] - 0.;

        return true;
    }

    static ceres::CostFunction* Create() {
        return (new ceres::AutoDiffCostFunction<MotionModelRegularization, 2, 7, 7>(new MotionModelRegularization()));
    }
};
}
}
