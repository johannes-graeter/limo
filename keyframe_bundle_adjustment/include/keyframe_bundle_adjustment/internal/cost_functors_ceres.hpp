/*
 * Copyright 2017. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */

#pragma once

#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "definitions.hpp"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <Eigen/Eigen>

namespace keyframe_bundle_adjustment {
namespace cost_functors_ceres {
// Templated pinhole camera model for use with Ceres.  The camera is
// parameterized using 7 parameters. 4 for rotation, 3 for
// translation.

struct ReprojectionErrorWithQuaternions {
    // this is basically from
    // https://github.com/ceres-solver/ceres-solver/blob/master/examples/snavely_reprojection_error.h
    // without distortions
    // (u, v): the position of the observation with respect to the image
    // center point.
    ReprojectionErrorWithQuaternions(double observed_x,
                                     double observed_y,
                                     double focal_length,
                                     double principal_point_x,
                                     double principal_point_y,
                                     std::array<double, 7> pose_cam_veh)
            : observed_x(observed_x), observed_y(observed_y), focal_length(focal_length),
              principal_point_x(principal_point_x), principal_point_y(principal_point_y), pose_C_X(pose_cam_veh) {
    }

    template <typename T>
    bool operator()(const T* const pose_X_O, const T* const point_O, T* residuals) const {
        // pose is defined from keyframe X to origin in origin coordinate system
        // camera frame is called C
        // point is defined in Origin

        // pose_O_X[0,1,2,3] is are the rotation of the camera as a quaternion.
        //
        // We use QuaternionRotatePoint as it does not assume that the
        // quaternion is normalized, since one of the ways to run the
        // bundle adjuster is to let Ceres optimize all 4 quaternion
        // parameters without a local parameterization.

        // normaly a local parametrization should be used
        // with that you can also easily implement motion models

        // transform landmark to landmarks
        // poses are defined from camera x to cos in which landmarks are defined
        //        T p[3] = {point_O[0], point_O[1], point_O[2]};

        // transformation of the point p and poses P is the following :
        //  * p_Vehicle = P_Camera_KEyframe*P_KEyframe_Origin * p_Origin

        // is this faster with ceres functions functions?
        // convert point from vehicle frame to camera frame
        std::array<T, 7> pose_C_X_cast;
        std::transform(
            std::cbegin(pose_C_X), std::cend(pose_C_X), std::begin(pose_C_X_cast), [](const auto& a) { return T(a); });

        std::array<T, 7> pose_X_O_cast;
        pose_X_O_cast[0] = T(pose_X_O[0]);
        pose_X_O_cast[1] = T(pose_X_O[1]);
        pose_X_O_cast[2] = T(pose_X_O[2]);
        pose_X_O_cast[3] = T(pose_X_O[3]);
        pose_X_O_cast[4] = T(pose_X_O[4]);
        pose_X_O_cast[5] = T(pose_X_O[5]);
        pose_X_O_cast[6] = T(pose_X_O[6]);


        Eigen::Transform<T, 3, Eigen::Isometry> pose_X_O_eigen = convert(pose_X_O_cast);
        Eigen::Transform<T, 3, Eigen::Isometry> pose_C_X_eigen = convert(pose_C_X_cast);

        Eigen::Matrix<T, 3, 1> point_C = pose_C_X_eigen * pose_X_O_eigen * Eigen::Matrix<T, 3, 1>(point_O);

        // Compute final projected point position.
        T x_cam = point_C[0];
        T y_cam = point_C[1];

        if (ceres::abs(point_C[2]) >= T(0.01)) {
            x_cam /= point_C[2];
            y_cam /= point_C[2];
        } else {
            return false;
        }

        const T predicted_x = static_cast<T>(focal_length) * x_cam + static_cast<T>(principal_point_x);
        const T predicted_y = static_cast<T>(focal_length) * y_cam + static_cast<T>(principal_point_y);

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);


        //        if (std::is_same<T, double>::value) {
        //            std::cout << "pose_X_O=" << pose_X_O[0] << " " << pose_X_O[1] << " " <<
        //            pose_X_O[2]
        //                      << " " << pose_X_O[3] << " " << pose_X_O[4] << " " << pose_X_O[5] <<
        //                      " "
        //                      << pose_X_O[6] << " " << std::endl;

        //            std::cout << "pose_C_X=" << pose_C_X[0] << " " << pose_C_X[1] << " " <<
        //            pose_C_X[2]
        //                      << " " << pose_C_X[3] << " " << pose_C_X[4] << " " << pose_C_X[5] <<
        //                      " "
        //                      << pose_C_X[6] << " " << std::endl;
        //            std::cout << "point_O=" << point_O[0] << " " << point_O[1] << " " <<
        //            point_O[2]
        //                      << std::endl;
        //            std::cout << "point_X=" << point_X[0] << " " << point_X[1] << " " <<
        //            point_X[2]
        //                      << std::endl;
        //            std::cout << predicted_x << " " << observed_x << "\t" << predicted_y << " "
        //                      << observed_y << std::endl;
        //        }

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y,
                                       const double focal_length,
                                       const double principal_point_x,
                                       const double principal_point_y,
                                       std::array<double, 7> pose_veh_cam) {
        return (new ceres::AutoDiffCostFunction<ReprojectionErrorWithQuaternions, 2, 7, 3>(
            new ReprojectionErrorWithQuaternions(
                observed_x, observed_y, focal_length, principal_point_x, principal_point_y, pose_veh_cam)));
    }

    double observed_x;
    double observed_y;
    double focal_length;
    double principal_point_x;
    double principal_point_y;
    std::array<double, 7> pose_C_X;
};


// if you want to do motion only better deactivate parameters that making different cost functor

struct LandmarkDepthError {

    LandmarkDepthError(const double depth, std::array<double, 7> pose_cam_veh)
            : depth_(depth), pose_C_X_(pose_cam_veh) {
    }

    template <typename T>
    bool operator()(const T* const pose_X_O, const T* const point_O, T* residuals) const {

        // convert point from vehicle frame to camera frame
        std::array<T, 7> pose_C_X_cast;
        std::transform(std::cbegin(pose_C_X_), std::cend(pose_C_X_), std::begin(pose_C_X_cast), [](const auto& a) {
            return T(a);
        });

        std::array<T, 7> pose_X_O_cast;
        pose_X_O_cast[0] = T(pose_X_O[0]);
        pose_X_O_cast[1] = T(pose_X_O[1]);
        pose_X_O_cast[2] = T(pose_X_O[2]);
        pose_X_O_cast[3] = T(pose_X_O[3]);
        pose_X_O_cast[4] = T(pose_X_O[4]);
        pose_X_O_cast[5] = T(pose_X_O[5]);
        pose_X_O_cast[6] = T(pose_X_O[6]);

        Eigen::Transform<T, 3, Eigen::Isometry> pose_X_O_eigen = convert(pose_X_O_cast);
        Eigen::Transform<T, 3, Eigen::Isometry> pose_C_X_eigen = convert(pose_C_X_cast);

        Eigen::Matrix<T, 3, 1> point_C = pose_C_X_eigen * pose_X_O_eigen * Eigen::Matrix<T, 3, 1>(point_O);

        // the depth of the landmark is the z value of the point in the camera frame
        T point_C_z = point_C(2, 0);

        residuals[0] = point_C_z - T(depth_);

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double depth, std::array<double, 7> pose_cam_veh) {
        return (
            new ceres::AutoDiffCostFunction<LandmarkDepthError, 1, 7, 3>(new LandmarkDepthError(depth, pose_cam_veh)));
    }

    const double depth_;
    std::array<double, 7> pose_C_X_;
};

struct PoseRegularizationPose {
    PoseRegularizationPose(double scale) : scale_(scale) {
        ;
    }

    template <typename T>
    bool operator()(const T* const pose1, const T* const pose0, T* residuals) const {

        using v3t = Eigen::Matrix<T, 3, 1>;

        v3t trans0, trans1;
        trans1 << pose1[4], pose1[5], pose1[6];
        trans0 << pose0[4], pose0[5], pose0[6];

        T res = (trans1 - trans0).squaredNorm();
        // Could be possible that sqaure root is instable near 0.
        if (res < T(0.001)) {
            residuals[0] = T(0.) - static_cast<T>(scale_);
        } else {
            residuals[0] = ceres::sqrt(res) - static_cast<T>(scale_);
        }

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(double scale) {
        return (new ceres::AutoDiffCostFunction<PoseRegularizationPose, 1, 7, 7>(new PoseRegularizationPose(scale)));
    }

    double scale_;
};


struct PoseRegularizationSpeed {
    PoseRegularizationSpeed(double ts_cur, double ts_before, double ts_before2)
            : dt_cur(ts_cur - ts_before), dt_before(ts_before - ts_before2) {
        if (dt_cur <= 0. || dt_before <= 0.) {
            throw std::runtime_error("In PoseRegularizationSpeed: invalid timestamps");
        }
    }

    template <typename T>
    bool operator()(const T* const pose_cur,
                    const T* const pose_before,
                    const T* const pose_before2,
                    T* residuals) const {

        using v3t = Eigen::Matrix<T, 3, 1>;

        v3t trans_cur, trans_before, trans_before2;
        trans_cur << pose_cur[4], pose_cur[5], pose_cur[6];
        trans_before << pose_before[4], pose_before[5], pose_before[6];
        trans_before2 << pose_before2[4], pose_before2[5], pose_before2[6];

        v3t vel_cur = (trans_cur - trans_before) / static_cast<T>(dt_cur);
        v3t vel_before = (trans_before - trans_before2) / static_cast<T>(dt_before);

        v3t d_vel = vel_cur - vel_before;
        residuals[0] = d_vel[0];
        residuals[1] = d_vel[1];
        residuals[2] = d_vel[2];

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(double ts_cur, double ts_before, double ts_before2) {
        return (new ceres::AutoDiffCostFunction<PoseRegularizationSpeed, 3, 7, 7, 7>(
            new PoseRegularizationSpeed(ts_cur, ts_before, ts_before2)));
    }

    double dt_cur;
    double dt_before;
};
// struct ReprojectionErrorWithQuaternionsMotionOnly {
//    ReprojectionErrorWithQuaternionsMotionOnly(double observed_x,
//                                               double observed_y,
//                                               double focal_length,
//                                               double principal_point_x,
//                                               double principal_point_y,
//                                               Eigen::Vector3d landmark_O)
//            : landmark_O_(landmark_O) {
//        cost_functor_full_ba_ = std::make_unique<ReprojectionErrorWithQuaternions>(
//            observed_x, observed_y, focal_length, principal_point_x, principal_point_y);
//    }

//    /**
//     * this cost functor is a wrapper around full ba cst funtor without landmarks
//     */
//    template <typename T>
//    bool operator()(const T* const pose_O_X, T* residuals) const {
//        return cost_functor_full_ba_->operator()(
//            pose_O_X, landmark_O_.cast<T>().eval().data(), residuals);
//    }

//    // Factory to hide the construction of the CostFunction object from
//    // the client code.
//    static ceres::CostFunction* Create(const double observed_x,
//                                       const double observed_y,
//                                       const double focal_length,
//                                       const double principal_point_x,
//                                       const double principal_point_y,
//                                       const Eigen::Vector3d landmark) {
//        return (new ceres::AutoDiffCostFunction<ReprojectionErrorWithQuaternionsMotionOnly, 2,
//        7>(
//            new ReprojectionErrorWithQuaternionsMotionOnly(observed_x,
//                                                           observed_y,
//                                                           focal_length,
//                                                           principal_point_x,
//                                                           principal_point_y,
//                                                           landmark)));
//    }


//    Eigen::Vector3d landmark_O_;
//    std::unique_ptr<ReprojectionErrorWithQuaternions> cost_functor_full_ba_;
//};
}
}
