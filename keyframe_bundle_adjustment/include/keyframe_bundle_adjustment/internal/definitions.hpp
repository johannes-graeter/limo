//
// Created by graeter on 06/07/16.
//
#pragma once
#include <cv.hpp>
#include <iostream>
#include <Eigen/Eigen>
#include <ceres/problem.h>
#include <matches_msg_types/tracklets.hpp>

namespace keyframe_bundle_adjustment {

using CameraId = unsigned long;                         ///< id of the cameras
using TimestampNSec = matches_msg_types::TimestampNSec; ///< Timestamp in unix system time
using TimestampSec = double;                            ///< Timestamp in unix system time
using LandmarkId = unsigned long;                       ///< id for landmarks, must be global
// using LandmarkIds = std::vector<LandmarkIds>;    ///< many landmark ids
using KeyframeId = unsigned long;          ///< ids for keyframes, hence for poses
using CameraId = unsigned long;            ///< ids for cameras
using CameraIds = std::vector<CameraId>;   ///< ids for cameras
using PoseId = KeyframeId;                 ///< poses are linked to keyframes so ids should be identical
using EigenPose = Eigen::Isometry3d;       ///< poses are isometries not affine transformaions
using Pose = std::array<double, 7>;        ///< poses are stored as quaternion(w,x,y,z) and translation(x,y,z)
using ResidualId = ceres::ResidualBlockId; ///< ids given by ceres to identify residuals so we can
using Direction = std::array<double, 3>;   ///< direction vector
///@brief plane, consisting of normal vector and distance to zero.
struct Plane {
    Plane() {
        direction = std::array<double, 3>{{0., 0., 1.}};
        distance = -std::numeric_limits<double>::max(); // negative distance means no gp will be used in optimization.
    }
    Direction direction;
    double distance;
};

using FeaturePoint = matches_msg_types::FeaturePoint;
using Tracklet = matches_msg_types::Tracklet;
using Tracklets = matches_msg_types::Tracklets;

using Measurement = FeaturePoint; ///< use feature points since vector2d is a pain

struct Landmark {
    using Ptr = std::shared_ptr<Landmark>;
    using ConstPtr = std::shared_ptr<const Landmark>;

    using Residual = double;
    using ResidualWithoutRobustLoss = double;

    using Residuals = std::pair<Residual, ResidualWithoutRobustLoss>;

    /**
     * @brief Landmark, empty constructor for use with map
     */
    Landmark();
    /**
     * @brief Landmark, constructor for interface with eigen
     * @param p, position
     * @param has_depth, flag if depth is observed or if it is a triangulated landmark from mono
     */
    Landmark(const Eigen::Vector3d& p, bool has_depth = false);
    std::array<double, 3> pos; ///< position of the landmark, x,y,z
    //    std::array<double, 3> unc; ///< uncertainty of the landmark in x,y,z direction

    bool has_measured_depth{false}; ///< flag if depth was observed
    bool is_ground_plane{false};    ///< flag for ground plane points

    double weight{1.}; ///< weight for landmark, can be set by covariance, label, ...
};

Pose convert(EigenPose p);

TimestampSec convert(const TimestampNSec& ts);
TimestampNSec convert(const TimestampSec& ts);

template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> convert(const T* const pose) {
    // eigen quaternions are strange, there order is: x,y,z,w in map, in constructor it is w,x,y,z
    Eigen::Transform<T, 3, Eigen::Isometry> p = Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
    p.translate(Eigen::Matrix<T, 3, 1>(pose[4], pose[5], pose[6]));
    p.rotate(Eigen::Quaternion<T>(pose[0], pose[1], pose[2], pose[3]));

    return p;
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> convert(const std::array<T, 7>& pose) {
    return convert(pose.data());
}

/**
 * @brief The Camera struct; save camera calibration with extrinsics and intrinsics
 */
struct Camera {
    using Ptr = std::shared_ptr<Camera>;

    Camera(double f, const Eigen::Vector2d& pp, const EigenPose& pose_cam_veh);

    /**
     * @brief getIntrinsicMatrix, convenience function to acces intrinsics as matrix
     * @return intrinsics as Eigen Matrix
     */
    Eigen::Matrix3d getIntrinsicMatrix() const;

    /**
     * @brief getEigenPose, convenience function to get eigen pose from array
     * @return eigen format of poses
     */
    EigenPose getEigenPose() const;

    /**
     * @brief getViewingRay, convenience function to convert a measurement to a viewing ray
     * @param m, measruement u,v
     * @return viewing ray with norm 1.
     */
    Eigen::Vector3d getViewingRay(const Measurement& m);

    double focal_length;             ///< focal length
    Eigen::Vector2d principal_point; ///< principal point

    Pose pose_camera_vehicle;   ///< pose from camera coordinate system to vehicle coordinate system
                                /// = extrinsics calibration
                                ///
    Eigen::Matrix3d intrin_inv; ///< inverse intrinsics for ray calculation
};

template <typename T, size_t n>
void print_array(const std::array<T, n>& a) {
    for (size_t j = 0; j < n; ++j) {
        std::cout << a[n] << " ";
    }
    std::cout << std::endl;
}

template <typename T, size_t n>
std::array<T, n> convert_eigen_to_array(const Eigen::Matrix<T, n, 1>& v) {
    std::array<T, n> out;
    for (size_t j = 0; j < n; ++j) {
        out[n] = v[n];
    }
    return out;
}

template <typename T, size_t n>
Eigen::Matrix<T, n, 1> convert_array_to_eigen(const std::array<T, n>& v) {
    Eigen::Matrix<T, n, 1> out;
    for (size_t j = 0; j < n; ++j) {
        out[n] = v[n];
    }
    return out;
}

template <typename T>
Eigen::Vector2d reproject(const T& transform_1_0, const Eigen::Matrix3d& intrin, const Eigen::Vector3d& lm_0) {
    Eigen::Vector3d tmp_1 = intrin * (transform_1_0 * lm_0);
    return tmp_1.colwise().hnormalized();
}

double calcRotRoccMetric(const Eigen::Isometry3d& transform_1_0,
                         const Eigen::Matrix3d& intrinsics,
                         const Eigen::Vector3d& lm_0,
                         const Eigen::Vector2d& measurement_1);

double getCost(const std::vector<ResidualId>& ids, ceres::Problem& problem);

double getCost(const std::map<ResidualId, std::pair<LandmarkId, int>>& ids, ceres::Problem& problem);

Eigen::Matrix<double, 3, 1> convertMeasurementToRay(const Eigen::Matrix3d& intrin_inv, const Measurement& m);

double calcQuaternionDiff(const Pose& p0, const Pose& p1);

} // end of namespace
