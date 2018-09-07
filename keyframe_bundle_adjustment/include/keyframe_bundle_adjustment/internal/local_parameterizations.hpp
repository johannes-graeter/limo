// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once
#include <array>
#include <Eigen/Eigen>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/local_parameterization.h>
#include <ceres/rotation.h>

#include "definitions.hpp"

namespace keyframe_bundle_adjustment {
namespace local_parameterizations {

///@brief apply ceres functions to convert from euler angles(rotation around x,y,z axes
/// respectively) to a quaternion
template <typename T>
void EulerAnglesToQuaternion(const T* euler_angles, T* quaternion) {

    std::array<T, 9> R;
    // euler angles must be in degree for function
    T fact(180. / M_PI);
    T angles_rad[3] = {euler_angles[0] * fact, euler_angles[1] * fact, euler_angles[2] * fact};

    ceres::EulerAnglesToRotationMatrix(angles_rad, 3, R.data());

    std::array<T, 3> aa;
    ceres::RotationMatrixToAngleAxis(R.data(), aa.data());

    ceres::AngleAxisToQuaternion(aa.data(), quaternion);
}


/**
 * @brief plus operator for local parameterization for 6 degrees of freedom
 *
 * this is for testing purposes for applicatino use ceres built-ins:
 * ceres::LocalParameterization* motion_parameterization = new ceres::ProductParameterization(
 * new ceres::QuaternionParameterization(), new ceres::IdentityParameterization(3));
 */
struct FullDofsPlus {
    template <typename T>
    bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
        // x is input state, delta is how varibales propagate to state, x_plus_delta is the
        // result
        // delta: angle1,angle2,angle3 (i guess angleaxis), x,y,z
        // x: quaternion[0:4], x, y, z

        // quaternion plus copied from
        // https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/local_parameterization_test.cc
        const T squared_norm_delta = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];

        T q_delta[4];
        if (squared_norm_delta > T(0.0)) {
            T norm_delta = ceres::sqrt(squared_norm_delta);
            const T sin_delta_by_delta = ceres::sin(norm_delta) / norm_delta;
            q_delta[0] = ceres::cos(norm_delta);
            q_delta[1] = sin_delta_by_delta * delta[0];
            q_delta[2] = sin_delta_by_delta * delta[1];
            q_delta[3] = sin_delta_by_delta * delta[2];
        } else {
            // We do not just use q_delta = [1,0,0,0] here because that is a
            // constant and when used for automatic differentiation will
            // lead to a zero derivative. Instead we take a first order
            // approximation and evaluate it at zero.
            q_delta[0] = T(1.0);
            q_delta[1] = delta[0];
            q_delta[2] = delta[1];
            q_delta[3] = delta[2];
        }

        T out_rot[4];
        ceres::QuaternionProduct(q_delta, x, out_rot);

        // add it to output
        x_plus_delta[0] = out_rot[0];
        x_plus_delta[1] = out_rot[1];
        x_plus_delta[2] = out_rot[2];
        x_plus_delta[3] = out_rot[3];
        x_plus_delta[4] = x[4] + delta[3];
        x_plus_delta[5] = x[5] + delta[4];
        x_plus_delta[6] = x[6] + delta[5];

        return true;
    }
};


///**
// * NOT WORKING YET: x_plus_delta must lie on sphere, not the delta
// * @brief The FixScaleVectorPlus struct, vector with fix norm, can be used for first pose
// */
// struct FixScaleVectorPlus {
//    FixScaleVectorPlus(double scale = 1.) : scale_(scale) {
//        ;
//    }
//    /**
//     * @brief functor operator
//     * @param x: 3 dims, x,y,z
//     * @param delta: 2 dims, two angles of a sphere
//     * @param x_plus_delta: 3 dims result of plus
//     */
//    template <typename T>
//    bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
//        // use sphere coordinates:
//        // https://en.wikipedia.org/wiki/Spherical_coordinate_system

//        T r(scale_);

//        T sin_theta = ceres::sin(delta[0]);
//        x_plus_delta[0] = x[0] + r * sin_theta * ceres::cos(delta[1]);
//        x_plus_delta[1] = x[1] + r * sin_theta * ceres::sin(delta[1]);
//        x_plus_delta[2] = x[2] + r * ceres::cos(delta[0]);

//        return true;
//    }

//    double scale_; ///< scale of the vector
//};

/**
 * @brief The FixScaleVectorPlus struct, vector with fix norm, can be used for first pose
 * seems strange but should work according to
 * https://groups.google.com/forum/#!topic/ceres-solver/pFJDfAWR3dY
 *
 * For global cos that could be a problem if the reference pose is not at zero, middle of shpere
 * should lie in last pose->optimise motion not pose?
 */
struct FixScaleVectorPlus {
    FixScaleVectorPlus(double scale = 1.) : scale_(scale) {
        ;
    }
    /**
     * @brief functor operator
     * @param x: 3 dims, x,y,z
     * @param delta: 3 dims, just norm it
     * @param x_plus_delta: 3 dims result of plus
     */
    template <typename T>
    bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
        // use sphere coordinates:

        x_plus_delta[0] = x[0] + delta[0];
        x_plus_delta[1] = x[1] + delta[1];
        x_plus_delta[2] = x[2] + delta[2];

        T norm = ceres::sqrt(x_plus_delta[0] * x_plus_delta[0] + x_plus_delta[1] * x_plus_delta[1] +
                             x_plus_delta[2] * x_plus_delta[2]);
        T factor = T(scale_) / norm;

        x_plus_delta[0] *= factor;
        x_plus_delta[1] *= factor;
        x_plus_delta[2] *= factor;

        return true;
    }

    double scale_; ///< scale of the vector
};

/**
 * @brief plus operator for local parametrization
 */
struct CircularMotionPlus2d {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CircularMotionPlus2d() = default;
    /**
    * @brief plus functor, x is input state, delta is how varibales propagate to state, x_plus_delta
    * is the result
     * @param delta:  yaw, arc_length (angle around z axiss)
     * @param x: quaternion[0:4], x, y, z
     */
    template <typename T>
    bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
        using Transf = Eigen::Transform<T, 3, Eigen::Isometry>;

        // Add delta as yaw and arc length
        T yaw = delta[0];
        T arc = delta[1];

        // Calculate pose on circle from ref to current.
        // default values for dx and dy when yaw=0.
        T dx = arc;
        T dy = T(0.);

        // if yaw is not zero, dx and dy can be calculated by curvature
        if (ceres::abs(yaw) > T(0.001)) {
            T radius = arc / yaw;
            dx = radius * ceres::sin(yaw);
            dy = radius * (T(1.) - ceres::cos(yaw));
        }

        Transf delta_pose = Transf::Identity();
        delta_pose.translation()[0] = dx;
        delta_pose.translation()[1] = dy;
        delta_pose.translation()[2] = T(0.);

        Eigen::Matrix<T, 3, 3> rot_mat =
            (Eigen::AngleAxis<T>(yaw, Eigen::Matrix<T, 3, 1>(T(0.), T(0.), T(1.)))).toRotationMatrix();
        delta_pose.linear() = rot_mat;

        Transf pose_x_origin = convert(x);
        Transf out = delta_pose * pose_x_origin;
        Eigen::Quaternion<T> q(out.rotation());

        // add it to output
        x_plus_delta[0] = q.w();
        x_plus_delta[1] = q.x();
        x_plus_delta[2] = q.y();
        x_plus_delta[3] = q.z();
        x_plus_delta[4] = out.translation().x();
        x_plus_delta[5] = out.translation().y();
        x_plus_delta[6] = out.translation().z();

        return true;
    }

    static ceres::LocalParameterization* Create() {
        return new ceres::AutoDiffLocalParameterization<CircularMotionPlus2d, 7, 2>(new CircularMotionPlus2d());
    }
};

struct FixScaleCircularMotionPlus {
    FixScaleCircularMotionPlus(double fix_value = 1.) : fix_value_(fix_value) {
        ;
    }

    template <typename T>
    bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
        // x is input state, delta is how varibales propagate to state, x_plus_delta is the
        // result
        // delta: pitch, yaw, roll (angles around x,y,z axes)
        // x: quaternion[0:4], x, curvature, z

        // default values for dx and dy
        T dx = T(fix_value_);
        T dy = T(0.);

        // if yaw is not zero, dx and dy can be calculated by curvature
        if (ceres::abs(delta[1]) > T(0.001)) {
            dx = ceres::sin(delta[1]) / delta[1] * T(fix_value_);
            dy = (T(1.) - ceres::cos(delta[1])) / delta[1] * T(fix_value_);
        }

        // get quaternion from euler angles
        std::array<T, 4> q;
        EulerAnglesToQuaternion(delta, q.data());

        // propagate rotations
        std::array<T, 4> out_rot;
        ceres::QuaternionProduct(x, q.data(), out_rot.data());

        // add it to output
        x_plus_delta[0] = out_rot[0];
        x_plus_delta[1] = out_rot[1];
        x_plus_delta[2] = out_rot[2];
        x_plus_delta[3] = out_rot[3];
        x_plus_delta[4] = x[4] + dy;
        x_plus_delta[5] = x[5] + T(0.);
        x_plus_delta[6] = x[6] + dx;

        return true;
    }

    double fix_value_;
};
}
}
