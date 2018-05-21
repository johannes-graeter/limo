// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "keyframe.hpp"
namespace keyframe_bundle_adjustment {

Keyframe::Keyframe(TimestampNSec timestamp,
                   const Tracklets& tracklets,
                   std::map<CameraId, Camera::Ptr> cameras,
                   std::map<LandmarkId, CameraIds> landmark_to_cameras,
                   EigenPose p,
                   FixationStatus fix_stat)
        : timestamp_(timestamp), cameras_(cameras), fixation_status_(fix_stat), is_active_(true) {
    assignMeasurements(tracklets, landmark_to_cameras);
    assignPose(p);
}

Keyframe::Keyframe(TimestampNSec timestamp,
                   const Tracklets& tracklets,
                   Camera::Ptr camera,
                   EigenPose p,
                   Keyframe::FixationStatus fix_stat)
        : timestamp_(timestamp), fixation_status_(fix_stat), is_active_(true) {
    CameraId cam_id = 0;
    cameras_[cam_id] = camera;
    assignMeasurements(tracklets, cam_id);
    assignPose(p);
}

bool Keyframe::operator<(const Keyframe& kf) const {
    return this->timestamp_ < kf.timestamp_;
}


void Keyframe::assignMeasurements(const Tracklets& tracklets, const std::map<LandmarkId, CameraIds>& landmark_lookup) {
    std::map<CameraId, Tracklets> out;
    for (const auto& track : tracklets.tracks) {
        // add tracklets to all cameras from which it is seen
        for (const auto& cam_id : landmark_lookup.at(track.id)) {
            // add track to tracklet per cam
            out[cam_id].stamps = tracklets.stamps; // this should be always the same since non measured
                                                   // tracklets are discarded
            out[cam_id].tracks.push_back(track);
        }
    }

    // assign them
    for (const auto& el : out) {
        assignMeasurements(el.second, el.first);
    }
}

void Keyframe::assignMeasurements(const Tracklets& tracklets, const CameraId& cam_id) {
    // find stamp corresponding to keyframe
    auto iter = std::find(tracklets.stamps.begin(), tracklets.stamps.end(), this->timestamp_);
    int index = std::distance(tracklets.stamps.begin(), iter);

    // assign measurements from tracklets
    for (const auto& track : tracklets.tracks) {
        // it is possible that feature track is shorter than .stamps so check if track has
        // measurement at timestamp
        if (index < int(track.feature_points.size())) {
            //            measurements[track.id] = m;
            measurements_[track.id][cam_id] = track.feature_points[index];
        }
    }
}

void Keyframe::assignPose(const EigenPose& p) {
    pose_ = convert(p);
}

std::map<CameraId, Eigen::Vector3d> Keyframe::getProjectedLandmarkPosition(
    const std::pair<LandmarkId, Landmark::ConstPtr>& id_lm) const {
    auto it = measurements_.find(id_lm.first);

    // In this case lm_id was never observed in keyframe, return empty map.
    if (it == measurements_.cend()) {
        return std::map<CameraId, Eigen::Vector3d>();
    }

    // Get landmark in vehicle coords.
    // Save copy by using Eigen::Map.
    const Eigen::Vector3d& p_vehicle =
        this->getEigenPose() * Eigen::Map<const Eigen::Vector3d>(id_lm.second->pos.data());

    // Transform landmark into all camera frames.
    std::map<CameraId, Eigen::Vector3d> out;
    for (const auto& cam_meas : it->second) {
        const auto& cam_id = cam_meas.first;
        Eigen::Vector3d p_cam = cameras_.at(cam_id)->getEigenPose() * p_vehicle;
        out[cam_id] = p_cam;
    }

    return out;
}


Measurement& Keyframe::getMeasurement(LandmarkId lm_id, CameraId cam_id) {
    return measurements_.at(lm_id).at(cam_id);
}

const Measurement& Keyframe::getMeasurement(LandmarkId lm_id, CameraId cam_id) const {
    return measurements_.at(lm_id).at(cam_id);
}

std::map<CameraId, Measurement> Keyframe::getMeasurements(LandmarkId lm_id) const {
    std::map<CameraId, Measurement> out;
    for (const auto& cam : cameras_) {
        if (hasMeasurement(lm_id, cam.first)) {
            out[cam.first] = getMeasurement(lm_id, cam.first);
        }
    }

    return out;
}

bool Keyframe::hasMeasurement(const LandmarkId& lm_id, const CameraId& cam_id) const {
    auto it_lm = measurements_.find(lm_id);
    if (it_lm != measurements_.cend()) {

        auto it_cam = measurements_.at(lm_id).find(cam_id);

        if (it_cam != measurements_.at(lm_id).cend()) {
            return true;
        }
    }

    return false;
}

bool Keyframe::hasMeasurement(LandmarkId lm_id) const {
    for (const auto& cam : cameras_) {
        if (hasMeasurement(lm_id, cam.first)) {
            return true;
        }
    }

    return false;
}

EigenPose Keyframe::getEigenPose() const {
    return convert(pose_);
}

std::shared_ptr<Pose> Keyframe::getPosePtr() const {
    return std::make_shared<Pose>(pose_);
}
}
