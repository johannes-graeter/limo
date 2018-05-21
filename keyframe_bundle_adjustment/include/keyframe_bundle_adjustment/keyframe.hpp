// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

// standard includes
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

#include "internal/definitions.hpp"

namespace keyframe_bundle_adjustment {

class Keyframe {
public: // defs
    enum class FixationStatus { Pose, Scale, None };
    using Ptr = std::shared_ptr<Keyframe>;
    using ConstPtr = std::shared_ptr<const Keyframe>;

public:
    /**
     * @brief Keyframe, empty constructor needed for std::map
     */
    Keyframe() {
        ;
    }

    /**
     * @brief Keyframe constructor
     * @param t, timstamp at which keyframe is taken, used for identification with measurements
     * @param tracklets, all measurements, corresponding measurements will be found and assigned to
     * keyframe
     * @param cameras, map with pointers to all cameras, with intrinsics and extrinsics
     * @param landmark_to_cameras, lookup to define which landmark is seen from which camera
     * @param p, pose of keyframe
     * @param fix_stat, define if full pose of Keyframe shall be optimized or not
     */
    Keyframe(TimestampNSec timestamp,
             const Tracklets& tracklets,
             std::map<CameraId, Camera::Ptr> cameras,
             std::map<LandmarkId, CameraIds> landmark_to_cameras,
             EigenPose p,
             FixationStatus fix_stat = FixationStatus::None);

    /**
     * @brief Keyframe constructor, mono
     * @param timestamp, timestamp at which keyframe is taken, used for identification with
     * measurements
     * @param tracklets, all measurements, corresponding measurements will be found and assigned to
     * keyframe
     * @param cameras, pointer to camera, with intrinsics and extrinsics, id is given internally
     * @param p, pose of keyframe
     */
    Keyframe(TimestampNSec timestamp,
             const Tracklets& tracklets,
             Camera::Ptr camera,
             EigenPose p,
             FixationStatus fix_stat = FixationStatus::None);

    /**
     * @brief operator <, assort by timestamp, usefull for sets
     * @param kf
     * @return
     */
    bool operator<(const Keyframe& kf) const;

    /**
     * @brief assignMeasurements to keyframes; this will find measurements in tracklets that
     * correspond to keyframe and save them
     * @param Tracklets
     */
    void assignMeasurements(const Tracklets&, const CameraId&);

    /**
     * @brief assignMeasurements
     * @param tracklets, measured tracklets
     * @param landmark_lookup, defines what landmark is seen in which camera
     */
    void assignMeasurements(const Tracklets& tracklets, const std::map<LandmarkId, CameraIds>& landmark_lookup);

    /**
     * @brief assignPose to keyframe
     * @param Pose in eigen format
     */
    void assignPose(const EigenPose& p);

    //    /**
    //     * @brief assignResidualId, setter for residual ids
    //     * @param lm_id
    //     * @param res_id
    //     */
    //    void assignResidualId(LandmarkId lm_id, ResidualId res_id, CameraId cam_id);

    /**
     * @brief Getter for measurement from this keyframe that
     * corresponds to a certain landmark and camera
     * @param LandmarkId, id of the landmark given by Tracklets
     * @param CameraId, id of the camera on which landmark was observed
     * @return measurement
     */
    Measurement& getMeasurement(LandmarkId lm_id, CameraId cam_id);

    ///@brief const version
    const Measurement& getMeasurement(LandmarkId lm_id, CameraId cam_id) const;

    /**
     * @brief getMeasurements, get measurements rom all cams given a landmark id
     * @param lm_id
     * @return map with cameraIds and measurements
     */
    std::map<CameraId, Measurement> getMeasurements(LandmarkId lm_id) const;

    /**
     * @brief hasMeasurement, test if measurement is present for landmark and cam
     * @param lm_id, id of the landmark given by Tracklets
     * @param cam_id, id of the camera on which landmark was observed
     * @return
     */
    bool hasMeasurement(const LandmarkId& lm_id, const CameraId& cam_id) const;

    /**
     * @brief hasMeasurements, test if any of the cameras has a measurement
     * @param lm_id, landmark id
     * @return bool
     */
    bool hasMeasurement(LandmarkId lm_id) const;

    /**
     * @brief getProjectedLandmark, convenience function to get landmarks in keyframe coordinates if
     * the where measured there
     * @param LandmarkId and landmark, coordinates of landmark to project and id, corrdinates in
     * origin frame
     * @return projected landmarks for each of the cameras
     */
    //    std::map<CameraId, Eigen::Vector3d> getProjectedLandmarkPosition(
    //        const std::pair<LandmarkId, Landmark>& landmark_origin) const;

    ///@brief overload for pointers to landmarks
    std::map<CameraId, Eigen::Vector3d> getProjectedLandmarkPosition(
        const std::pair<LandmarkId, Landmark::ConstPtr>& landmark_origin) const;

    /**
     * @brief getPose, copy keyframe pose to eigen transform
     * @todo get pointer from reference to pose
     * @return pointer to pose
     */
    EigenPose getEigenPose() const;

    /**
     * @brief getPosePtr, get pointer to keyframe pose
     * @return pointer to pose
     */
    std::shared_ptr<Pose> getPosePtr() const;


public:
    ///@brief timestamp of keyframe, used for finding measurements in tracklets
    TimestampNSec timestamp_;

    ///@brief a keyframe can possess several cameras with known calibration
    std::map<CameraId, Camera::Ptr> cameras_;

    ///@brief define what of the keyframe is fixed
    FixationStatus fixation_status_;

    ///@brief pose from keyframe to origin
    Pose pose_;

    ///@brief storage for measurement, identified by landmark ids
    std::map<LandmarkId, std::map<CameraId, Measurement>> measurements_;

    //    ///@brief storage for residual ids, stored by landmarks, these can be used to activate or
    //    /// deactivate resiualds
    //    std::map<LandmarkId, ResidualHistory> residual_history;

    ///@brief flag if keyframe is active and will be optimized
    bool is_active_;
};
}
