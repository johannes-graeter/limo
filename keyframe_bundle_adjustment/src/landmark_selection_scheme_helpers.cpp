// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/landmark_selection_scheme_helpers.hpp"

namespace keyframe_bundle_adjustment {
namespace landmark_helpers {

std::vector<LandmarkId> chooseNearLmIds(int max_num_lms,
                                        const std::vector<LandmarkId>& near_ids,
                                        const std::map<LandmarkId, double>& map_flow) {
    // It is possible, that landmarks are not present in map since f.e. tracks are too short.
    // Sort those out first.
    ///@todo How long does that take?
    std::vector<LandmarkId> ids;
    for (const auto& el : near_ids) {
        if (map_flow.find(el) != map_flow.cend()) {
            ids.push_back(el);
        }
    }

    // Init output.
    int cur_num_lms = std::min(max_num_lms, static_cast<int>(ids.size()));
    std::vector<LandmarkId> out;
    out.resize(cur_num_lms);

    // Get the ids woth biggest flow.
    auto it = std::partial_sort_copy(
        ids.cbegin(), ids.cend(), out.begin(), out.end(), [&map_flow](const auto& a, const auto& b) {
            return map_flow.at(a) > map_flow.at(b);
        });
    if (it != out.end()) {
        throw std::runtime_error(
            "In LandmarkSelectionSchemeHelpers: Not all chosen ids of near field have been copied!");
    }

    return out;
}

std::vector<LandmarkId> chooseMiddleLmIds(int max_num, const std::vector<LandmarkId>& middle_ids) {
    // shuffle vector and copy first elements
    int num = std::min(max_num, static_cast<int>(middle_ids.size()));
    std::vector<LandmarkId> a;
    a.reserve(middle_ids.size());
    std::copy(middle_ids.cbegin(), middle_ids.cend(), std::back_inserter(a));

    std::random_shuffle(a.begin(), a.end());
    std::vector<LandmarkId> out;
    out.reserve(num);
    std::copy(a.cbegin(), a.cbegin() + num, std::back_inserter(out));
    return out;
}

std::vector<LandmarkId> chooseFarLmIds(int max_num,
                                       const std::vector<LandmarkId>& ids_far,
                                       const std::map<KeyframeId, Keyframe::ConstPtr>& keyframes) {

    // far: measruements with highest number of measurements
    // Here we don't distinguigh between measurements with and without depth
    // count occurency in keyframes for each landmark

    std::map<LandmarkId, unsigned int> count_map;
    for (const auto& id : ids_far) {
        count_map[id] = 0;
        for (const auto& kf : keyframes) {
            if (kf.second->hasMeasurement(id)) {
                count_map.at(id) = count_map.at(id) + 1;
            }
        }
    }

    // Add n landmarks with longest tracks to selection.
    std::vector<LandmarkId> chosen_ids_far;
    {
        int num = std::min(max_num, static_cast<int>(ids_far.size()));

        chosen_ids_far.resize(num);

        auto it = std::partial_sort_copy(
            ids_far.cbegin(),
            ids_far.cend(),
            chosen_ids_far.begin(),
            chosen_ids_far.end(),
            [&count_map](const auto& a, const auto& b) { return count_map.at(a) > count_map.at(b); });
    }

    return chosen_ids_far;
}

std::map<LandmarkId, double> calcMeanFlow(const std::map<LandmarkId, Landmark::ConstPtr>& landmarks,
                                          const std::map<KeyframeId, Keyframe::ConstPtr>& keyframes) {
    std::map<LandmarkId, double> out;

    // put keyframes into vector and sort them
    std::vector<Keyframe::ConstPtr> kf_ptrs;
    kf_ptrs.reserve(keyframes.size());

    for (const auto& kf : keyframes) {
        // only use active keyframes
        if (kf.second->is_active_) {
            kf_ptrs.push_back(kf.second);
        }
    }

    std::sort(
        kf_ptrs.begin(), kf_ptrs.end(), [](const auto& a, const auto& b) { return a->timestamp_ < b->timestamp_; });
    for (const auto& lm_el : landmarks) {
        const auto& lm_id = lm_el.first;
        // get measruements from keyframes with min and max timestamps
        std::map<CameraId, Measurement> first_measurement_per_cam;
        TimestampNSec first_ts = 0;
        getMeasurementFromKf(kf_ptrs.cbegin(), kf_ptrs.cend(), lm_id, first_measurement_per_cam, first_ts);

        std::map<CameraId, Measurement> last_measurement_per_cam;
        TimestampNSec last_ts = 0;
        getMeasurementFromKf(kf_ptrs.crbegin(), kf_ptrs.crend(), lm_id, last_measurement_per_cam, last_ts);

        // don't add if lm not on keyframes
        if (last_measurement_per_cam.empty() || first_measurement_per_cam.empty()) {
            continue;
        }

        // evaluate flow per time
        double dt_sec = convert(last_ts - first_ts);
        if (dt_sec <= 0.) {
            continue;
        }

        // calc all permutations with all cams for this lm id and take max
        std::vector<double> data;
        data.reserve(first_measurement_per_cam.size() * last_measurement_per_cam.size());
        for (const auto& el_last : last_measurement_per_cam) {
            for (const auto& el_first : first_measurement_per_cam) {
                //                // dot product can be bigger than 1. because of numeric inaccuracy
                //                // -> round to 1.0 if it is bigger
                //                Eigen::Vector3d last_ray = el_last.second;
                //                last_ray.normalize();
                //                Eigen::Vector3d first_ray = el_first.second;
                //                first_ray.normalize();
                //                double d = last_ray.dot(first_ray);
                //                double sign = d > 0. ? 1. : -1.;
                //                double angle = std::acos(sign * std::min(std::abs(d), 1.0));
                //                angles.push_back(angle);

                data.push_back((el_last.second.toEigen2d() - el_first.second.toEigen2d()).norm() / dt_sec);
            }
        }
        // get max and assign flow as pixel per second
        auto it = std::max_element(data.begin(), data.end(), [](const auto& a, const auto& b) { return a < b; });
        out[lm_id] = (*it);
    }

    return out;
}
}
}
