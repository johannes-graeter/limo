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
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "keyframe.hpp"
#include "landmark_selection_schemes.hpp"
#include "internal/definitions.hpp"

#include <chrono>


namespace keyframe_bundle_adjustment {

/**
*  @class LandmarkSelector
*  @par
*
*  select landmarks
*
*/
class LandmarkSelector {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public: // attributes
public: // public methods
    // default constructor
    LandmarkSelector() = default;

    // default destructor
    virtual ~LandmarkSelector() = default;

    // default move
    LandmarkSelector(LandmarkSelector&& other) = default;
    LandmarkSelector& operator=(LandmarkSelector&& other) = default;

    // default copy
    LandmarkSelector(const LandmarkSelector& other) = default;
    LandmarkSelector& operator=(const LandmarkSelector& other) = default;

    /**
    * @brief addScheme, add scheme to be used
    * @param scheme
    */
    void addScheme(LandmarkSelectionSchemeBase::ConstPtr scheme) {
        schemes_.push_back(scheme);
    }

    /**
     * @brief select, select keyframes by comparing to last selected frames with SelectionSchemes
     * @param frames, reference to frames that shall be selected
     */
    std::set<LandmarkId> select(const std::map<LandmarkId, Landmark::ConstPtr>& landmarks,
                                const std::map<KeyframeId, Keyframe::ConstPtr>& kfs) {

        // init as selection
        std::map<LandmarkId, Landmark::ConstPtr> selected_lms = landmarks;

        // reject outliers marked by setOutlier()
        for (const auto& id : outlier_ids_) {
            auto it = selected_lms.find(id);
            if (it != selected_lms.cend()) {
                selected_lms.erase(it);
            }
        }

        auto start_sel_schemes = std::chrono::steady_clock::now();
        // apply selection schemes to selection
        for (const auto& scheme : schemes_) {
            auto start_indiv_scheme = std::chrono::steady_clock::now();

            std::set<LandmarkId> cur_selection;
            // If the scheme is a Categorizer save categories

            auto categorizer_scheme = std::dynamic_pointer_cast<const LandmarkCategorizatonInterface>(scheme);
            if (categorizer_scheme == nullptr) {
                cur_selection = scheme->getSelection(selected_lms, kfs);
            } else {
                // we have only one categorizer implemented yet, so we can copy
                ///@todo if more are implemented, make a logic to fuse categories
                landmark_categories_ = categorizer_scheme->getCategorizedSelection(selected_lms, kfs);

                // make set
                for (const auto& el : landmark_categories_) {
                    cur_selection.insert(el.first);
                }
            }
            std::cout << "Duration lm_sel::apply_individual_scheme="
                      << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                               start_indiv_scheme)
                             .count()
                      << " ms" << std::endl;

            // push selected items to map and select with next scheme
            selected_lms.clear();
            for (const auto& id : cur_selection) {
                if (landmarks.find(id) != landmarks.cend()) {
                    selected_lms[id] = landmarks.at(id);
                } else {
                    // This can happen if landmarks are rejected before but still saved on keyframes as measruements.
                    // For example LandmarkSelectionSchemeAddDepth may add them again.
                    std::cout << "LandmarkSelector: Attention! Id=" << id << " is not in landmarks." << std::endl;
                }
            }
        }
        std::cout << "Duration lm_sel::apply_sel_schemes="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                           start_sel_schemes)
                         .count()
                  << " ms" << std::endl;

        std::set<LandmarkId> selection;
        // takes elements from first vector, does something with it and stores it in second vector
        std::transform(selected_lms.cbegin(),
                       selected_lms.cend(),
                       std::inserter(selection, selection.end()),
                       [](const auto& a) { return a.first; });

        auto start_get_non_sel = std::chrono::steady_clock::now();
        // get not selected landmarks
        std::set<LandmarkId> all_landmarks;
        for (const auto& lm : landmarks) {
            all_landmarks.insert(lm.first);
        }
        std::cout << "Duration lm_sel::get_non_sel_lms="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                           start_get_non_sel)
                         .count()
                  << " ms" << std::endl;

        auto start_get_diff = std::chrono::steady_clock::now();
        std::set<LandmarkId> diff;
        // see https://www.fluentcpp.com/2017/01/09/know-your-algorithms-algos-on-sets/
        std::set_difference(all_landmarks.cbegin(),
                            all_landmarks.cend(),
                            selection.cbegin(),
                            selection.cend(),
                            std::inserter(diff, diff.begin()));
        std::cout << "Duration lm_sel::get_lm_diff="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                           start_get_diff)
                         .count()
                  << " ms" << std::endl;
        // mark non selected
        for (const auto& el : diff) {
            markUnselected(el);
        }


        return selection;
    }

    /**
     * @brief markUnselected, remember unselected landmarks(f.e. to remove them once they have a
     * certain count)
     * @param lm_id, landmark id that shall be unselected
     */
    void markUnselected(LandmarkId lm_id) {
        if (unselected_lms.find(lm_id) == unselected_lms.cend()) {
            unselected_lms[lm_id] = 1;
        } else {
            unselected_lms.at(lm_id)++;
        }
    }

    /**
     * @brief getUnselectedLandmarks, getter for unselected landmarks
     * @return unselected landmarks with id and count how often they where unselected
     */
    const std::map<LandmarkId, unsigned int>& getUnselectedLandmarks() const {
        return unselected_lms;
    }

    /**
     * @brief getLandmarkCategories, get for landmark categories; if no scheme with categorizer was
     * added,
     *        this is empty.
     * @return Map with landmark ids and categories.
     */
    std::map<LandmarkId, LandmarkCategorizatonInterface::Category> getLandmarkCategories() const {
        return landmark_categories_;
    }

    void clearOutliers() {
        outlier_ids_.clear();
    }

    const std::set<LandmarkId>& getOutliers() const {
        return outlier_ids_;
    }

    void setOutlier(LandmarkId id) {
        outlier_ids_.insert(id);
    }

    void setOutlier(const std::set<LandmarkId>& ids) {
        for (const auto& el : ids) {
            setOutlier(el);
        }
    }

private:                                               // attributes
    std::map<LandmarkId, unsigned int> unselected_lms; ///< save which landmarks where not selected
    /// and count how often they were unseletected

    std::map<LandmarkId, LandmarkCategorizatonInterface::Category> landmark_categories_;

    ///@brief labels from semantic labeling that should be treated as outliers
    const std::set<int> outlier_labels_{
        23, // sky
        24, // kind of vehicle
        25, // motorcycle
        26  // vehicle
    };

public:                                                          // attributes
    std::vector<LandmarkSelectionSchemeBase::ConstPtr> schemes_; ///< schemes that are used for landmark selection
    std::set<LandmarkId> outlier_ids_;                           ///< ids of outliers
};
}
