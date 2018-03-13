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

using FeaturePoint = matches_msg_types::FeaturePoint;
using Tracklet = matches_msg_types::Tracklet;
using Tracklets = matches_msg_types::Tracklets;

using Measurement = FeaturePoint; ///< use feature points since vector2d is a pain


// struct ResidualHistory {

//    struct Residual{
//        ResidualId residual_id;
//        LandmarkId landmark_id;
//        double value;
//        double value_without_loss;
//    }


//    void assignResidualId(LandmarkId lm_id, ResidualId res_id, CameraId cam_id) {
//        residual_ids[lm_id][cam_id] = res_id;
//    }

//    void resetResidualIds() {
//        residual_ids.clear();
//    }

//    void evaluateResiduals(std::shared_ptr<ceres::Problem> problem) {
//        // get residuals ids
//        std::vector<ResidualId> block_ids;
//        std::vector<LandmarkId> lm_ids;
//        for (const auto& lm_camresid : residual_ids) {
//            for (const auto& cam_resid : lm_camresid.second) {
//                // only add residual of first camera
//                block_ids.push_back(cam_resid.second);
//                lm_ids.push_back(lm_camresid.first);
//                break;
//            }
//        }

//        // evaluate with and wihtout robust loss function
//        ceres::Problem::EvaluateOptions eval_options;
//        eval_options.residual_blocks = block_ids;

//        std::vector<double> residuals_without_loss;
//        eval_options.apply_loss_function = false;
//        double cost = -1.;
//        problem->Evaluate(eval_options, &cost, &residuals_without_loss, nullptr, nullptr);

//        std::vector<double> residuals_with_loss;
//        eval_options.apply_loss_function = true;
//        problem->Evaluate(eval_options, &cost, &residuals_with_loss, nullptr, nullptr);

//        // store residuals on landmarks
//        assert(lm_ids.size() == block_ids.size() &&
//               2 * lm_ids.size() == residuals_without_loss.size() &&
//               2 * lm_ids.size() == residuals_with_loss.size());
//        TimestampNSec cur_stamp = getKeyframe();
//        for (int i = 0; i < int(lm_ids.size()); ++i) {
//            residual_history_[lm_ids].assignResiduals(m.first,
//                                                      res_id_reprojection,
//                                                      cam_id_meas.first,
//                                                      residuals_with_loss[i],
//                                                      residuals_without_loss[i]);
//        }
//    }

//    std::map<LandmarkId, std::map<CameraId, std::vector<ResidualId>>> residual_ids;
//};

struct Landmark {
    using Ptr = std::shared_ptr<Landmark>;
    using ConstPtr = std::shared_ptr<const Landmark>;

    using Residual = double;
    using ResidualWithoutRobustLoss = double;

    using Residuals = std::pair<Residual, ResidualWithoutRobustLoss>;

    /**
     * @brief Landmark, empty constructor for use with map
     */
    Landmark() {
        ;
    }
    /**
     * @brief Landmark, constructor for interface with eigen
     * @param p, position
     * @param has_depth, flag if depth is observed or if it is a triangulated landmark from mono
     */
    Landmark(Eigen::Vector3d p, bool has_depth = false) : has_measured_depth(has_depth) {
        pos[0] = p[0];
        pos[1] = p[1];
        pos[2] = p[2];
    }
    std::array<double, 3> pos; ///< position of the landmark, x,y,z
    //    std::array<double, 3> unc; ///< uncertainty of the landmark in x,y,z direction

    bool has_measured_depth{false}; ///< flag if depth was observed

    double weight{1.}; ///< weight for landmark, can be set by covariance, label, ...
};

namespace {
Pose convert(const EigenPose& p) {
    Eigen::Quaterniond q(p.rotation());
    // why does q.normalize() not work?
    Pose pose;
    pose[0] = q.w();
    pose[1] = q.x();
    pose[2] = q.y();
    pose[3] = q.z();

    pose[4] = p.translation()[0];
    pose[5] = p.translation()[1];
    pose[6] = p.translation()[2];

    return pose;
}

// These functions are currently unused.
TimestampSec convert(const TimestampNSec& ts) {
    return static_cast<TimestampSec>(ts * 1e-09);
}

TimestampNSec convert(const TimestampSec& ts) {
    return static_cast<TimestampNSec>(ts * 1e09);
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> convert(const std::array<T, 7>& pose) {
    // eigen quaternions are strange, there order is: x,y,z,w in map, in constructor it is w,x,y,z
    Eigen::Transform<T, 3, Eigen::Isometry> p = Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
    p.translate(Eigen::Matrix<T, 3, 1>(pose[4], pose[5], pose[6]));
    p.rotate(Eigen::Quaternion<T>(pose[0], pose[1], pose[2], pose[3]));

    return p;
}
}

/**
 * @brief The Camera struct; save camera calibration with extrinsics and intrinsics
 */
struct Camera {
    using Ptr = std::shared_ptr<Camera>;

    Camera(double f, Eigen::Vector2d pp, EigenPose pose_cam_veh) : focal_length(f), principal_point(pp) {
        pose_camera_vehicle = convert(pose_cam_veh);
        intrin_inv = getIntrinsicMatrix().inverse();
    }

    /**
     * @brief getIntrinsicMatrix, convenience function to acces intrinsics as matrix
     * @return intrinsics as Eigen Matrix
     */
    Eigen::Matrix3d getIntrinsicMatrix() const {
        Eigen::Matrix3d intrin;
        intrin << focal_length, 0., principal_point[0], 0., focal_length, principal_point[1], 0., 0., 1.;
        return intrin;
    }

    /**
     * @brief getEigenPose, convenience function to get eigen pose from array
     * @return eigen format of poses
     */
    EigenPose getEigenPose() const {
        return convert(pose_camera_vehicle);
    }

    /**
     * @brief getViewingRay, convenience function to convert a measurement to a viewing ray
     * @param m, measruement u,v
     * @return viewing ray with norm 1.
     */
    Eigen::Vector3d getViewingRay(const Measurement& m) {
        Eigen::Vector3d meas_hom{m.u, m.v, 1.};

        meas_hom = intrin_inv * meas_hom;
        meas_hom.normalize();

        return meas_hom;
    }

    double focal_length;             ///< focal length
    Eigen::Vector2d principal_point; ///< principal point

    Pose pose_camera_vehicle;   ///< pose from camera coordinate system to vehicle coordinate system
                                /// = extrinsics calibration
                                ///
    Eigen::Matrix3d intrin_inv; ///< inverse intrinsics for ray calculation
};

//#include <matches_msg_ros/MatchesMsg.h>
/////@brief define forward declaration as matchesMsg
///// taken from
/// https://stackoverflow.com/questions/18834645/c-forward-declaration-and-alias-with-using-or-typedef
// struct Tracklets : MatchesMsg {
//    using MatchesMsg::MatchesMsg; // inherit constructors
//};


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
}
