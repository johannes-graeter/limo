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

namespace commons {
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> getEigenPose(const T* const pose_X_O) {
    std::array<T, 7> pose_X_O_cast;
    pose_X_O_cast[0] = T(pose_X_O[0]);
    pose_X_O_cast[1] = T(pose_X_O[1]);
    pose_X_O_cast[2] = T(pose_X_O[2]);
    pose_X_O_cast[3] = T(pose_X_O[3]);
    pose_X_O_cast[4] = T(pose_X_O[4]);
    pose_X_O_cast[5] = T(pose_X_O[5]);
    pose_X_O_cast[6] = T(pose_X_O[6]);

    Eigen::Transform<T, 3, Eigen::Isometry> pose_X_O_eigen = convert(pose_X_O_cast);
    return pose_X_O_eigen;
}
}

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
                                     const std::array<double, 7>& pose_cam_veh,
                                     std::shared_ptr<bool> compensate_rotation = std::make_shared<bool>(false))
            : observed_x(observed_x), observed_y(observed_y), focal_length(focal_length),
              principal_point_x(principal_point_x), principal_point_y(principal_point_y), pose_C_X(pose_cam_veh),
              compensate_rotation_(compensate_rotation) {
    }

    template <typename T>
    bool project(const Eigen::Matrix<T, 3, 1>& point_C, T& predicted_x, T& predicted_y) const {

        // Compute final projected point position.
        T x_cam = point_C[0];
        T y_cam = point_C[1];

        if (ceres::abs(point_C[2]) >= T(0.01)) {
            x_cam /= point_C[2];
            y_cam /= point_C[2];
        } else {
            return false;
        }

        predicted_x = static_cast<T>(focal_length) * x_cam + static_cast<T>(principal_point_x);
        predicted_y = static_cast<T>(focal_length) * y_cam + static_cast<T>(principal_point_y);

        return true;
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
        Eigen::Transform<T, 3, Eigen::Isometry> pose_C_X_eigen = convert(pose_C_X_cast);
        Eigen::Transform<T, 3, Eigen::Isometry> pose_X_O_eigen = commons::getEigenPose(pose_X_O);

        Eigen::Matrix<T, 3, 1> point_C = pose_C_X_eigen * pose_X_O_eigen * Eigen::Matrix<T, 3, 1>(point_O);

        T predicted_x;
        T predicted_y;
        if (!project(point_C, predicted_x, predicted_y)) {
            return false;
        }

        // Account for rotation compensation (RotRocc from Buzco et al.).
        T rot_comp{1.};
        if (*compensate_rotation_) {
            Eigen::Transform<T, 3, Eigen::Isometry> transform_rot_only =
                pose_C_X_eigen * Eigen::Quaternion<T>(pose_X_O_eigen.rotation());
            Eigen::Matrix<T, 3, 1> point_rot_C = transform_rot_only * Eigen::Matrix<T, 3, 1>(point_O);
            T predicted_x_rot;
            T predicted_y_rot;
            if (!project(point_rot_C, predicted_x_rot, predicted_y_rot)) {
                return false;
            }
            T dx_rot = predicted_x_rot - T(observed_x);
            T dy_rot = predicted_y_rot - T(observed_y);
            rot_comp = dx_rot * dx_rot + dy_rot * dy_rot;
            if (rot_comp < T(0.01)) {
                return false;
            }
            rot_comp = ceres::sqrt(rot_comp);
        }

        // The error is the difference between the predicted and observed position.
        residuals[0] = (predicted_x - T(observed_x)) / rot_comp;
        residuals[1] = (predicted_y - T(observed_y)) / rot_comp;

        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y,
                                       const double focal_length,
                                       const double principal_point_x,
                                       const double principal_point_y,
                                       const std::array<double, 7>& pose_veh_cam,
                                       std::shared_ptr<bool> compensate_rotation = std::make_shared<bool>(false)) {
        return (new ceres::AutoDiffCostFunction<ReprojectionErrorWithQuaternions, 2, 7, 3>(
            new ReprojectionErrorWithQuaternions(observed_x,
                                                 observed_y,
                                                 focal_length,
                                                 principal_point_x,
                                                 principal_point_y,
                                                 pose_veh_cam,
                                                 compensate_rotation)));
    }

    double observed_x;
    double observed_y;
    double focal_length;
    double principal_point_x;
    double principal_point_y;
    std::array<double, 7> pose_C_X;
    std::shared_ptr<bool> compensate_rotation_; ///< If true, use RotRoccErrorMetric
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
        Eigen::Transform<T, 3, Eigen::Isometry> pose_C_X_eigen = convert(pose_C_X_cast);
        Eigen::Transform<T, 3, Eigen::Isometry> pose_X_O_eigen = commons::getEigenPose(pose_X_O);

        Eigen::Matrix<T, 3, 1> point_C = pose_C_X_eigen * pose_X_O_eigen * Eigen::Matrix<T, 3, 1>(point_O);

        // the depth of the landmark is the z value of the point in the camera frame
        T point_C_z = point_C(2, 0);

        residuals[0] = point_C_z - T(depth_);

        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const double depth, std::array<double, 7> pose_cam_veh) {
        return (
            new ceres::AutoDiffCostFunction<LandmarkDepthError, 1, 7, 3>(new LandmarkDepthError(depth, pose_cam_veh)));
    }

    const double depth_;
    std::array<double, 7> pose_C_X_;
};

struct PoseRegularization {
    PoseRegularization(double scale) : scale_(scale) {
        ;
    }

    template <typename T>
    bool operator()(const T* const pose1, const T* const pose0, T* residuals) const {

        using Transf = Eigen::Transform<T, 3, Eigen::Isometry>;
        Transf pose1_eigen = commons::getEigenPose(pose1);
        Transf pose0_eigen = commons::getEigenPose(pose0);

        Transf diff = pose1_eigen * pose0_eigen.inverse();

        T res = diff.translation().norm();
        residuals[0] = res - static_cast<T>(scale_);

        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(double scale) {
        return (new ceres::AutoDiffCostFunction<PoseRegularization, 1, 7, 7>(new PoseRegularization(scale)));
    }

    double scale_;
};


struct SpeedRegularization {
    SpeedRegularization(double ts_cur, double ts_before, double ts_before2)
            : dt_cur(ts_cur - ts_before), dt_before(ts_before - ts_before2) {
        if (dt_cur <= 0. || dt_before <= 0.) {
            throw std::runtime_error("In PoseRegularizationSpeed: invalid timestamps");
        }
    }

    template <typename T>
    bool operator()(const T* const pose_cur_origin,
                    const T* const pose_before_origin,
                    const T* const pose_before2_origin,
                    T* residuals) const {

        using Transf = Eigen::Transform<T, 3, Eigen::Isometry>;

        Transf pose_cur_origin_eigen = commons::getEigenPose(pose_cur_origin);
        Transf pose_before_origin_eigen = commons::getEigenPose(pose_before_origin);
        Transf pose_before2_origin_eigen = commons::getEigenPose(pose_before2_origin);

        Transf d_pose_cur_before = pose_cur_origin_eigen * pose_before_origin_eigen.inverse();
        Transf d_pose_before_before2 = pose_before_origin_eigen * pose_before2_origin_eigen.inverse();

        T vel_cur = d_pose_cur_before.translation().norm() / static_cast<T>(dt_cur);
        T vel_before = d_pose_before_before2.translation().norm() / static_cast<T>(dt_before);

        //        if (std::is_same<T, double>::value) {
        //            std::cout << "cur vel=" << vel_cur << std::endl;
        //            std::cout << "before vel=" << vel_before << std::endl;
        //        }

        residuals[0] = vel_cur - vel_before;

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(double ts_cur, double ts_before, double ts_before2) {
        return (new ceres::AutoDiffCostFunction<SpeedRegularization, 1, 7, 7, 7>(
            new SpeedRegularization(ts_cur, ts_before, ts_before2)));
    }

    double dt_cur;
    double dt_before;
};

struct SpeedRegularizationVector2 {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SpeedRegularizationVector2(double ts_cur, double ts_before, double ts_before2, Pose pose_before, Pose pose_before2)
            : dt_cur_(ts_cur - ts_before) {
        double dt_before = ts_before - ts_before2;
        if (dt_cur_ <= 0. || dt_before <= 0.) {
            throw std::runtime_error("In PoseRegularizationSpeed: invalid timestamps");
        }

        // Get old speed.
        Eigen::Isometry3d pose_before_eigen = convert(pose_before);
        Eigen::Isometry3d pose_before2_eigen = convert(pose_before2);
        Eigen::Isometry3d pose_before_before2_eigen = pose_before_eigen * pose_before2_eigen.inverse();
        vel_before_before2_ = pose_before_before2_eigen.translation() / dt_before;

        // Save inverse of last pose.
        pose_origin_before_eigen_ = pose_before_eigen.inverse();
    }

    template <typename T>
    bool operator()(const T* const pose_cur_origin, T* residuals) const {
        using Transf = Eigen::Transform<T, 3, Eigen::Isometry>;
        Transf pose_cur_origin_eigen = commons::getEigenPose(pose_cur_origin);
        Transf pose_cur_before = pose_cur_origin_eigen * pose_origin_before_eigen_.cast<T>();
        Eigen::Matrix<T, 3, 1> vel_cur_before = pose_cur_before.translation() / static_cast<T>(dt_cur_);

        //        if (std::is_same<T, double>::value) {
        //            std::cout << "cur vel=" << vel_cur_before[0] << " " << vel_cur_before[1] << " " <<
        //            vel_cur_before[2] << " "
        //                      << std::endl;
        //            std::cout << "before vel=" << vel_before_before2_[0] << " " << vel_before_before2_[1] << " "
        //                      << vel_before_before2_[2] << std::endl;
        //        }

        Eigen::Matrix<T, 3, 1> dv = vel_cur_before - vel_before_before2_.cast<T>();
        residuals[0] = dv[0];
        residuals[1] = dv[1];
        residuals[2] = dv[2];

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(
        double ts_cur, double ts_before, double ts_before2, Pose pose_before, Pose pose_before2) {
        return (new ceres::AutoDiffCostFunction<SpeedRegularizationVector2, 3, 7>(
            new SpeedRegularizationVector2(ts_cur, ts_before, ts_before2, pose_before, pose_before2)));
    }

    Eigen::Vector3d vel_before_before2_;
    double dt_cur_;
    Eigen::Isometry3d pose_origin_before_eigen_;
};

struct GroundPlaneHeightRegularization {
    GroundPlaneHeightRegularization() = default;

    template <typename T>
    bool operator()(const T* const pose_X_O,
                    const T* const plane_dir,
                    const T* const dist,
                    const T* const point_O,
                    T* residuals) const {
        Eigen::Transform<T, 3, Eigen::Isometry> pose_X_O_eigen = commons::getEigenPose(pose_X_O);
        Eigen::Matrix<T, 3, 1> point_X = pose_X_O_eigen * Eigen::Map<const Eigen::Matrix<T, 3, 1>>(point_O);

        // Residual is distance to ground plane.
        // Cos: Normal in direction of vehicle coordinates, Kamera over gp is positive (this is against mathematical
        // formulation but more intuitive.)
        residuals[0] = plane_dir[0] * point_X[0] + plane_dir[1] * point_X[1] + plane_dir[2] * point_X[2] + dist[0];

        //        if (std::is_same<T, double>::value) {
        //            std::cout << "Plane dist=" << plane_dist_ << std::endl;
        //            std::cout << "Plane dir=" << plane_dir[0] << " " << plane_dir[1] << " " << plane_dir[2] <<
        //            std::endl;
        //            std::cout << "pointO=" << point_O[0] << " " << point_O[1] << " " << point_O[2] << std::endl;
        //            std::cout << "point=" << point_X[0] << " " << point_X[1] << " " << point_X[2] << std::endl;
        //            std::cout << "residual=" << residuals[0] << std::endl;
        //            std::cout << "pose_X_O=" << pose_X_O[0] << " " << pose_X_O[1] << " " << pose_X_O[2] << " " <<
        //            pose_X_O[3]
        //                      << " " << pose_X_O[4] << " " << pose_X_O[5] << " " << pose_X_O[6] << std::endl;
        //        }

        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create() {
        return (new ceres::AutoDiffCostFunction<GroundPlaneHeightRegularization, 1, 7, 3, 1, 3>(
            new GroundPlaneHeightRegularization()));
    }
};

struct VectorDifferenceRegularization {
    VectorDifferenceRegularization() {
        ;
    }

    template <typename T>
    bool operator()(const T* const plane_dir0, const T* const plane_dir1, T* residuals) const {
        // Residual is distance to ground plane.
        residuals[0] = plane_dir0[0] - plane_dir1[0];
        residuals[1] = plane_dir0[1] - plane_dir1[1];
        residuals[2] = plane_dir0[2] - plane_dir1[2];

        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create() {
        return (new ceres::AutoDiffCostFunction<VectorDifferenceRegularization, 3, 3, 3>(
            new VectorDifferenceRegularization()));
    }
};

struct VectorDifferenceRegularization2 {
    VectorDifferenceRegularization2(Direction dir) : plane_dir0_(dir) {
        ;
    }

    template <typename T>
    bool operator()(const T* const plane_dir1, T* residuals) const {
        // Residual is distance to ground plane.
        residuals[0] = plane_dir0_[0] - plane_dir1[0];
        residuals[1] = plane_dir0_[1] - plane_dir1[1];
        residuals[2] = plane_dir0_[2] - plane_dir1[2];

        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(Direction dir) {
        return (new ceres::AutoDiffCostFunction<VectorDifferenceRegularization2, 3, 3>(
            new VectorDifferenceRegularization2(dir)));
    }

    Direction plane_dir0_;
};

struct TranslationDifferenceRegularization {
    TranslationDifferenceRegularization() {
        ;
    }

    template <typename T>
    bool operator()(const T* const pose0, const T* const pose1, const T* const pose2, T* residuals) const {
        using Transf = Eigen::Transform<T, 3, Eigen::Isometry>;
        Transf pose0_origin_eigen = commons::getEigenPose(pose0);
        Transf pose1_origin_eigen = commons::getEigenPose(pose1);
        Transf pose2_origin_eigen = commons::getEigenPose(pose2);

        Transf diff10 = pose1_origin_eigen * pose0_origin_eigen.inverse();
        Transf diff21 = pose2_origin_eigen * pose1_origin_eigen.inverse();

        Eigen::Matrix<T, 3, 1> trans_diff = diff21.translation() - diff10.translation();

        residuals[0] = trans_diff[0];
        residuals[1] = trans_diff[1];
        residuals[2] = trans_diff[2];

        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create() {
        return (new ceres::AutoDiffCostFunction<TranslationDifferenceRegularization, 3, 7, 7, 7>(
            new TranslationDifferenceRegularization()));
    }
};

struct TranslationDifferenceRegularization2 {
    TranslationDifferenceRegularization2(std::array<double, 7> pose0_origin, std::array<double, 7> pose1_origin) {
        Eigen::Isometry3d pose1_origin_eigen = commons::getEigenPose(pose1_origin.data());
        pose_origin_1_eigen_ = pose1_origin_eigen.inverse();

        Eigen::Isometry3d pose0_origin_eigen = commons::getEigenPose(pose0_origin.data());
        pose1_0_eigen_ = pose1_origin_eigen * pose0_origin_eigen.inverse();
    }

    template <typename T>
    bool operator()(const T* const pose2_origin, T* residuals) const {
        using Transf = Eigen::Transform<T, 3, Eigen::Isometry>;
        Transf pose2_origin_eigen = commons::getEigenPose(pose2_origin);

        Transf diff10 = pose1_0_eigen_.cast<T>();
        Transf diff21 = pose2_origin_eigen * pose_origin_1_eigen_.cast<T>();

        Eigen::Matrix<T, 3, 1> trans_diff = diff21.translation() - diff10.translation();

        residuals[0] = trans_diff[0];
        residuals[1] = trans_diff[1];
        residuals[2] = trans_diff[2];

        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(std::array<double, 7> pose0_origin, std::array<double, 7> pose1_origin) {
        return (new ceres::AutoDiffCostFunction<TranslationDifferenceRegularization2, 3, 7>(
            new TranslationDifferenceRegularization2(pose0_origin, pose1_origin)));
    }

    Eigen::Isometry3d pose1_0_eigen_;
    Eigen::Isometry3d pose_origin_1_eigen_;
};

struct GroundPlaneDistanceRegularization {
    GroundPlaneDistanceRegularization() {
        ;
    }

    template <typename T>
    bool operator()(const T* const dist0, const T* const dist1, T* residuals) const {
        // Residual is distance to ground plane.
        residuals[0] = dist0[0] - dist1[0];

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create() {
        return (new ceres::AutoDiffCostFunction<GroundPlaneDistanceRegularization, 1, 1, 1>(
            new GroundPlaneDistanceRegularization()));
    }
};

struct GroundPlaneMotionRegularization {
    GroundPlaneMotionRegularization() {
        ;
    }

    template <typename T>
    bool operator()(const T* const pose_0, const T* const pose_1, const T* const plane_dir0, T* residuals) const {
        Eigen::Transform<T, 3, Eigen::Isometry> pose_eigen_0 = commons::getEigenPose(pose_0);
        Eigen::Transform<T, 3, Eigen::Isometry> pose_eigen_1 = commons::getEigenPose(pose_1);

        // Inverse delta translation. Should be positive for forward movement.
        Eigen::Matrix<T, 3, 1> delta_trans = (pose_eigen_0 * pose_eigen_1.inverse()).translation();

        // Normalize trans to 1, otherwise this cost functor will draw translation to zero.
        delta_trans.normalize();

        // Residual is distance distance to plane at 0.
        residuals[0] = plane_dir0[0] * delta_trans[0] + plane_dir0[1] * delta_trans[1] + plane_dir0[2] * delta_trans[2];

        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create() {
        return (new ceres::AutoDiffCostFunction<GroundPlaneMotionRegularization, 1, 7, 7, 3>(
            new GroundPlaneMotionRegularization()));
    }
};
}
}
