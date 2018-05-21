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

std::vector<LandmarkId> chooseNearLmIds(size_t max_num_lms,
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
    size_t cur_num_lms = std::min(max_num_lms, ids.size());
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

std::vector<LandmarkId> chooseMiddleLmIds(size_t max_num, const std::vector<LandmarkId>& middle_ids) {
    // shuffle vector and copy first elements
    size_t num = std::min(max_num, middle_ids.size());
    std::vector<LandmarkId> a;
    a.reserve(middle_ids.size());
    std::copy(middle_ids.cbegin(), middle_ids.cend(), std::back_inserter(a));

    std::random_shuffle(a.begin(), a.end());
    std::vector<LandmarkId> out;
    out.reserve(num);
    std::copy(a.cbegin(), std::next(a.cbegin(), num), std::back_inserter(out));
    return out;
}

std::vector<LandmarkId> chooseFarLmIds(size_t max_num,
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
        size_t num = std::min(max_num, ids_far.size());

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

std::map<LandmarkId, double> calcMeanFlow(const std::vector<LandmarkId>& valid_lm_ids,
                                          const std::vector<Keyframe::ConstPtr>& sorted_kf_ptrs) {
    std::map<LandmarkId, double> out;
    for (const auto& lm_id : valid_lm_ids) {
        // get measruements from keyframes with min and max timestamps
        std::map<CameraId, Measurement> last_meas;
        std::map<CameraId, double> mean;
        std::map<CameraId, int> occurence;
        for (const auto& el : sorted_kf_ptrs) {
            std::map<CameraId, Measurement> cur_meas = el->getMeasurements(lm_id);
            for (const auto& cam_meas : cur_meas) {
                if (last_meas.find(cam_meas.first) != last_meas.cend()) {
                    double flow = (last_meas.at(cam_meas.first).toEigen2d() - cam_meas.second.toEigen2d()).norm();
                    mean[cam_meas.first] = mean[cam_meas.first] + flow;
                    occurence[cam_meas.first] = occurence[cam_meas.first] + 1;
                }
                last_meas[cam_meas.first] = cam_meas.second;
            }
        }
        auto mean_iter = mean.begin();
        auto occurence_iter = occurence.cbegin();
        if (mean.size() != occurence.size()) {
            throw std::runtime_error("mean.size()=" + std::to_string(mean.size()) + " != occurence.size()=" +
                                     std::to_string(occurence.size()));
        }
        for (; mean_iter != mean.end() && occurence_iter != occurence.cend(); ++mean_iter, ++occurence_iter) {
            mean_iter->second /= occurence_iter->second;
        }

        // get max and assign flow as pixel per second
        auto it = std::max_element(
            mean.cbegin(), mean.cend(), [](const auto& a, const auto& b) { return a.second < b.second; });
        out[lm_id] = it->second;
    }

    return out;
}

std::map<LandmarkId, double> calcMeanFlow2(const std::vector<LandmarkId>& valid_lm_ids,
                                           const std::vector<Keyframe::ConstPtr>& sorted_kf_ptrs) {
    std::map<LandmarkId, double> out;

    std::cout << "oldest ts=" << sorted_kf_ptrs.front()->timestamp_ << std::endl;
    std::cout << "newest ts=" << sorted_kf_ptrs.back()->timestamp_ << std::endl;

    for (const auto& lm_id : valid_lm_ids) {
        // get measruements from keyframes with min and max timestamps
        std::map<CameraId, Measurement> oldest_measurement_per_cam;
        TimestampNSec oldest_ts = 0;
        getMeasurementFromKf(
            sorted_kf_ptrs.cbegin(), sorted_kf_ptrs.cend(), lm_id, oldest_measurement_per_cam, oldest_ts);

        std::map<CameraId, Measurement> newest_measurement_per_cam;
        TimestampNSec newest_ts = 0;
        getMeasurementFromKf(
            sorted_kf_ptrs.crbegin(), sorted_kf_ptrs.crend(), lm_id, newest_measurement_per_cam, newest_ts);

        // don't add if lm not on keyframes
        if (newest_measurement_per_cam.empty() || oldest_measurement_per_cam.empty()) {
            continue;
        }

        // evaluate flow per time
        double dt_sec = convert(newest_ts - oldest_ts);
        if (dt_sec <= 0.) {
            std::cout << "dt=" << dt_sec << std::endl;
            std::cout << "ts=" << oldest_ts << std::endl;
            std::cout << "---------1-----------" << std::endl;
            continue;
        }

        // calc all permutations with all cams for this lm id and take max
        std::vector<double> data;
        data.reserve(oldest_measurement_per_cam.size() * newest_measurement_per_cam.size());
        for (const auto& el_last : newest_measurement_per_cam) {
            for (const auto& el_first : oldest_measurement_per_cam) {
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

std::map<LandmarkId, double> calcMeanFlow(const std::vector<LandmarkId>& valid_lm_ids,
                                          const std::map<KeyframeId, Keyframe::ConstPtr>& keyframes) {
    // Get all keyframes and sort them by time stamps.
    std::vector<Keyframe::ConstPtr> kf_ptrs;
    kf_ptrs.reserve(keyframes.size());
    for (const auto& el : keyframes) {
        kf_ptrs.push_back(el.second);
    }

    std::sort(kf_ptrs.begin(), kf_ptrs.end(), [](const auto& a, const auto& b) { return a < b; });
    return calcMeanFlow(valid_lm_ids, kf_ptrs);
}
}
}
