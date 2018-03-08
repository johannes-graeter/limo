// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/landmark_selection_scheme_observability.hpp"

#include "internal/landmark_selection_scheme_helpers.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include <chrono>

namespace keyframe_bundle_adjustment {

namespace {

using LmIdVec = std::vector<LandmarkId>;

void assignMeasure(double bound_near_middle,
                   double bound_middle_far,
                   double data,
                   const LandmarkId& lm_id,
                   LmIdVec& near_ids,
                   LmIdVec& middle_ids,
                   LmIdVec& far_ids) {
    if (bound_near_middle <= data) {
        near_ids.push_back(lm_id);
    } else if (bound_near_middle > data && data > bound_middle_far) {
        middle_ids.push_back(lm_id);
    } else if (bound_middle_far >= data) {
        far_ids.push_back(lm_id);
    } else {
        std::stringstream ss;
        ss << "there is something wrong with the bins, angle=" << data;
        throw std::runtime_error(ss.str());
    }
}

void addVec(const LmIdVec& vec_to_add, LmIdVec& vec) {
    // Put all in one vector
    vec.reserve(vec.size() + std::distance(vec_to_add.cbegin(), vec_to_add.cend()));
    vec.insert(vec.end(), vec_to_add.cbegin(), vec_to_add.cend());
}
} // end of anonymous namespace

std::set<LandmarkId> LandmarkSelectionSchemeObservability::getSelection(const LandmarkMap& landmarks,
                                                                        const KeyframeMap& keyframes) const {
    auto categorized_lms = getCategorizedSelection(landmarks, keyframes);
    std::set<LandmarkId> out;

    for (const auto& el : categorized_lms) {
        out.insert(el.first);
    }
    return out;
}

std::map<LandmarkId, LandmarkCategorizatonInterface::Category> LandmarkSelectionSchemeObservability::
    getCategorizedSelection(const LandmarkMap& landmarks, const KeyframeMap& keyframes) const {

    auto start_calc_angles = std::chrono::steady_clock::now();
    std::map<LandmarkId, double> map_data = landmark_helpers::calcMeanFlow(landmarks, keyframes);
    std::cout << "Duration lm_sel::scheme_obs::calc_angles="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_calc_angles)
                     .count()
              << " ms" << std::endl;

    auto start_sort_lms = std::chrono::steady_clock::now();
    // split landmarks in 3 bins + 2 rest bins
    // so we have bins: lower outliers, bin1, bin2, bin3 ,upper outliers
    // bin.first is the lower boundary of the bin
    // all that is smaller than bin 2: far field
    // between bin2 and bin3: middle field
    // bigger than bin3: near field
    //
    // We also distinguish between landmarks with depth and wiithout depth in middle
    // and near field in order to prefer landmarks with depth measurement
    std::vector<LandmarkId> near_ids;
    std::vector<LandmarkId> middle_ids;
    std::vector<LandmarkId> far_ids;

    std::vector<LandmarkId> near_ids_with_depth;
    std::vector<LandmarkId> middle_ids_with_depth;

    auto start_make_lm_sets = std::chrono::steady_clock::now();
    for (const auto& lm_id_vec : landmarks) {
        const auto& lm_id = lm_id_vec.first;
        const auto& cur_data = map_data.at(lm_id);

        if (lm_id_vec.second->has_measured_depth) {
            assignMeasure(params_.bin_params_.bound_near_middle,
                          params_.bin_params_.bound_middle_far,
                          cur_data,
                          lm_id,
                          near_ids_with_depth,
                          middle_ids_with_depth,
                          far_ids);
        } else {
            assignMeasure(params_.bin_params_.bound_near_middle,
                          params_.bin_params_.bound_middle_far,
                          cur_data,
                          lm_id,
                          near_ids,
                          middle_ids,
                          far_ids);
        }
    }
    std::cout << "num near with depth=" << near_ids_with_depth.size() << "\n"
              << "num near without depth=" << near_ids.size() << "\n"
              << "num middle with depth=" << middle_ids_with_depth.size() << "\n"
              << "num middle without depth=" << middle_ids.size() << "\n"
              << "num far with/without depth=" << far_ids.size() << std::endl;

    std::cout << "Duration lm_sel::scheme_obs::copy ids to lm sets="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_make_lm_sets)
                     .count()
              << " ms" << std::endl;

    auto start_choose_near = std::chrono::steady_clock::now();
    std::vector<LandmarkId> chosen_ids_near;
    {
        // near: measruements with biggest angles
        // First choose those with depth then without
        chosen_ids_near = landmark_helpers::chooseNearLmIds(
            params_.bin_params_.max_num_landmarks_near, near_ids_with_depth, map_data);

        std::vector<LandmarkId> chosen_ids_near_wo_depth;
        int num = params_.bin_params_.max_num_landmarks_near - chosen_ids_near.size();
        chosen_ids_near_wo_depth = landmark_helpers::chooseNearLmIds(num, near_ids, map_data);

        addVec(chosen_ids_near_wo_depth, chosen_ids_near);
    }
    std::cout << "Duration lm_sel::scheme_obs::choose_near="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_choose_near)
                     .count()
              << " ms" << std::endl;

    auto start_choose_middle = std::chrono::steady_clock::now();
    std::vector<LandmarkId> chosen_ids_middle;
    {
        // middle: random
        // choose those with depth
        chosen_ids_middle =
            landmark_helpers::chooseMiddleLmIds(params_.bin_params_.max_num_landmarks_middle, middle_ids_with_depth);
        // if there is a pensum left add measrues without depth
        std::vector<LandmarkId> chosen_ids_middle_wo_depth;
        int num = params_.bin_params_.max_num_landmarks_middle - chosen_ids_middle.size();
        chosen_ids_middle_wo_depth = landmark_helpers::chooseMiddleLmIds(num, middle_ids);

        addVec(chosen_ids_middle_wo_depth, chosen_ids_middle);
    }
    std::cout << "Duration lm_sel::scheme_obs::choose_middle"
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_choose_middle)
                     .count()
              << " ms" << std::endl;


    auto start_choose_far = std::chrono::steady_clock::now();
    // Choose far ids
    std::vector<LandmarkId> chosen_ids_far =
        landmark_helpers::chooseFarLmIds(params_.bin_params_.max_num_landmarks_far, far_ids, keyframes);

    std::cout << "Duration lm_sel::scheme_obs::choose_far="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_choose_far)
                     .count()
              << " ms" << std::endl;

    auto start_out = std::chrono::steady_clock::now();

    std::map<LandmarkId, LandmarkCategorizatonInterface::Category> out;
    // write the chosen lms to selection
    for (const auto& el : chosen_ids_near) {
        out[el] = LandmarkCategorizatonInterface::Category::NearField;
    }
    for (const auto& el : chosen_ids_middle) {
        out[el] = LandmarkCategorizatonInterface::Category::MiddleField;
    }
    for (const auto& el : chosen_ids_far) {
        out[el] = LandmarkCategorizatonInterface::Category::FarField;
    }
    std::cout
        << "Duration lm_sel::scheme_obs::write_to_sets="
        << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_out).count()
        << " ms" << std::endl;

    std::cout << "Duration lm_sel::scheme_obs::sort_lms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_sort_lms)
                     .count()
              << " ms" << std::endl;

    return out;
}

LandmarkSelectionSchemeBase::ConstPtr LandmarkSelectionSchemeObservability::createConst(Parameters p) {

    return LandmarkSelectionSchemeBase::ConstPtr(new LandmarkSelectionSchemeObservability(p));
}

LandmarkSelectionSchemeBase::Ptr LandmarkSelectionSchemeObservability::create(Parameters p) {

    return LandmarkSelectionSchemeBase::Ptr(new LandmarkSelectionSchemeObservability(p));
}
}
