// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include "definitions.hpp"
#include "../keyframe.hpp"

namespace keyframe_bundle_adjustment {

namespace landmark_helpers {


template <typename iterator_type>
void getMeasurementFromKf(iterator_type it,
                          iterator_type end_iterator,
                          LandmarkId lm_id,
                          std::map<CameraId, Measurement>& meas_per_cam,
                          TimestampNSec& ts) {
    for (; it != end_iterator; it++) {
        for (const auto& cam : (*it)->cameras_) {
            const auto& cam_id = cam.first;
            if ((*it)->hasMeasurement(lm_id, cam_id)) {
                // calculate ray from measurement and poses
                auto map_meas = (*it)->getMeasurements(lm_id);
                assert(map_meas.size() == 1);
                auto first_measurement = map_meas.cbegin()->second;
                //                const auto& cam_id = map_meas.cbegin()->first;
                //                // get vewing ray no transformation needed, angle is independant of
                //                // frame
                //                const auto& cam = (*it)->cameras_.at(cam_id);
                //                viewing_ray_per_cam[cam_id] = cam->getViewingRay(first_measurement);
                meas_per_cam[cam_id] = first_measurement;
                ts = (*it)->timestamp_;
            }
        }

        // only get first hit -> return
        if (!meas_per_cam.empty()) {
            break;
        }
    }
}

std::vector<LandmarkId> chooseNearLmIds(int max_num_lms,
                                        const std::vector<LandmarkId>& near_ids,
                                        const std::map<LandmarkId, double>& map_flow);

std::vector<LandmarkId> chooseMiddleLmIds(int max_num, const std::vector<LandmarkId>& middle_ids);

std::vector<LandmarkId> chooseFarLmIds(int max_num,
                                       const std::vector<LandmarkId>& ids_far,
                                       const std::map<KeyframeId, Keyframe::ConstPtr>& keyframes);

std::map<LandmarkId, double> calcMeanFlow(const std::map<LandmarkId, Landmark::ConstPtr>& landmarks,
                                          const std::map<KeyframeId, Keyframe::ConstPtr>& keyframes);
}
}
