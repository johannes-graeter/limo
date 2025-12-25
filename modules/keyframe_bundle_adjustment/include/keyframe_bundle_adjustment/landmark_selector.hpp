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

#include <type_traits>
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
        selection_schemes_.push_back(scheme);
    }
    void addScheme(LandmarkSparsificationSchemeBase::ConstPtr scheme) {
        sparsification_schemes_.push_back(scheme);
    }
    void addScheme(LandmarkRejectionSchemeBase::ConstPtr scheme) {
        rejection_schemes_.push_back(scheme);
    }

    template <typename T>
    std::set<LandmarkId> getSelection(std::shared_ptr<const T> scheme,
                                      const std::map<LandmarkId, Landmark::ConstPtr>& selected_lms,
                                      const std::map<KeyframeId, Keyframe::ConstPtr>& kfs) {
        if (!std::is_base_of<LandmarkSchemeBase, T>::value) {
            throw std::runtime_error("In landmark_selector: Scheme does not inherit from base.");
        }
        // Get selection for current scheme.
        std::set<LandmarkId> cur_selection;
        auto categorizer_scheme = std::dynamic_pointer_cast<const LandmarkCategorizatonInterface>(scheme);
        if (categorizer_scheme == nullptr) {
            cur_selection = scheme->getSelection(selected_lms, kfs);
        } else {
            // If the scheme is a Categorizer save categories
            // we have only one categorizer implemented yet, so we can copy
            ///@todo if more are implemented, make a logic to fuse categories
            landmark_categories_ = categorizer_scheme->getCategorizedSelection(selected_lms, kfs);

            // make set
            for (const auto& el : landmark_categories_) {
                cur_selection.insert(el.first);
            }
        }
        return cur_selection;
    }

    void addToMap(const std::map<LandmarkId, Landmark::ConstPtr>& landmarks,
                  const std::set<LandmarkId>& cur_selection,
                  std::map<LandmarkId, Landmark::ConstPtr>& selected_lms) {
        // push selected items to map and select with next scheme
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

    /**
     * @brief select, select keyframes by comparing to last selected frames with SelectionSchemes
     * @param frames, reference to frames that shall be selected
     */
    std::set<LandmarkId> select(const std::map<LandmarkId, Landmark::ConstPtr>& landmarks,
                                const std::map<KeyframeId, Keyframe::ConstPtr>& kfs) {

        // init as selection
        std::map<LandmarkId, Landmark::ConstPtr> non_rejected_lms = landmarks;

        // reject outliers marked by setOutlier()
        for (const auto& id : outlier_ids_) {
            auto it = non_rejected_lms.find(id);
            if (it != non_rejected_lms.cend()) {
                non_rejected_lms.erase(it);
            }
        }

        //        // Init selection as all landmarks that heven't been unselected (selected + new ones)
        //        std::map<LandmarkId, Landmark::ConstPtr> selected_lms;

        //        for (const auto& el : landmarks) {
        //            const auto& id = el.first;
        //            // If id is neither unselected nor outlier, add it to possible selection.
        //            if (unselected_lms.find(id) == unselected_lms.cend() && outlier_ids_.find(id) ==
        //            outlier_ids_.cend()) {
        //                selected_lms[id] = el.second;
        //            }
        //        }

        auto start_sel_schemes = std::chrono::steady_clock::now();
        // Apply rejection schemes to all landmarks.
        // Size of lms will decrease.
        for (const auto& scheme : rejection_schemes_) {
            //            auto start_indiv_scheme = std::chrono::steady_clock::now();
            auto cur_selection = getSelection(scheme, non_rejected_lms, kfs);
            non_rejected_lms.clear();
            addToMap(landmarks, cur_selection, non_rejected_lms);
            //            std::cout << "Duration lm_sel::apply individual scheme and add selected="
            //                      <<
            //                      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()
            //                      -
            //                                                                               start_indiv_scheme)
            //                             .count()
            //                      << " ms" << std::endl;
        }

        // From the non rejected ones get those that must be included.
        // Size of lms will increase.
        std::map<LandmarkId, Landmark::ConstPtr> selected_lms;
        for (const auto& scheme : selection_schemes_) {
            //            auto start_indiv_scheme = std::chrono::steady_clock::now();
            //            auto cur_selection = getSelection(scheme, non_rejected_lms, kfs);
            auto cur_selection = getSelection(scheme, non_rejected_lms, kfs);
            addToMap(non_rejected_lms, cur_selection, selected_lms);
            //            std::cout << "Duration lm_sel::apply individual scheme and add selected="
            //                      <<
            //                      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()
            //                      -
            //                                                                               start_indiv_scheme)
            //                             .count()
            //                      << " ms" << std::endl;
        }

        // From the non rejected ones, sparsify.
        // Size of lms will decrease.
        std::map<LandmarkId, Landmark::ConstPtr> sparsified_lms = non_rejected_lms;
        for (const auto& scheme : sparsification_schemes_) {
            //            auto start_indiv_scheme = std::chrono::steady_clock::now();
            auto cur_selection = getSelection(scheme, sparsified_lms, kfs);
            sparsified_lms.clear();
            addToMap(non_rejected_lms, cur_selection, sparsified_lms);
            //            std::cout << "Duration lm_sel::apply individual scheme and add selected="
            //                      <<
            //                      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()
            //                      -
            //                                                                               start_indiv_scheme)
            //                             .count()
            //                      << " ms" << std::endl;
        }

        // Add sparsified to selected.
        for (const auto& el : selected_lms) {
            sparsified_lms[el.first] = el.second;
        }

        std::cout << "Duration lm_sel::apply all selection schemes="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                           start_sel_schemes)
                         .count()
                  << " ms" << std::endl;

        std::set<LandmarkId> selection;
        // takes elements from first vector, does something with it and stores it in second vector
        std::transform(sparsified_lms.cbegin(),
                       sparsified_lms.cend(),
                       std::inserter(selection, selection.end()),
                       [](const auto& a) { return a.first; });

        // get not selected landmarks
        auto start_get_non_sel = std::chrono::steady_clock::now();
        std::set<LandmarkId> all_landmarks;
        for (const auto& lm : landmarks) {
            all_landmarks.insert(lm.first);
        }
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
        auto it = std::max_element(kfs.cbegin(), kfs.cend(), [](const auto& a, const auto& b) {
            return a.second->timestamp_ < b.second->timestamp_;
        });
        const auto& cur_ts = it->second->timestamp_;
        for (const auto& el : diff) {
            markUnselected(el, cur_ts);
        }
        clean(cur_ts - convert(10.));

        std::cout << "Duration lm_sel::get_non_sel_lms="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                           start_get_non_sel)
                         .count()
                  << " ms" << std::endl;

        // Remember last selection.
        last_selected_lms_ = selection;

        return selection;
    }

    /**
     * @brief markUnselected, remember unselected landmarks(f.e. to remove them once they have a
     * certain count)
     * @param lm_id, landmark id that shall be unselected
     */
    void markUnselected(LandmarkId lm_id, TimestampNSec last_time_seen) {
        unselected_lms_[lm_id] += 1;
        last_time_seen_[lm_id] = last_time_seen;
    }

    void clean(TimestampNSec oldestTs) {
        auto it = last_time_seen_.begin();
        for (; it != last_time_seen_.end();) {
            if (it->second < oldestTs) {
                unselected_lms_.erase(it->first);
                it = last_time_seen_.erase(it);
            } else {
                it = std::next(it);
            }
        }
    }

    /**
     * @brief getUnselectedLandmarks, getter for unselected landmarks
     * @return unselected landmarks with id and count how often they where unselected
     */
    const std::map<LandmarkId, unsigned int>& getUnselectedLandmarks() const {
        return unselected_lms_;
    }

    /**
     * @brief getLandmarkCategories, get for landmark categories; if no scheme with categorizer was
     * added,
     *        this is empty.
     * @return Map with landmark ids and categories.
     */
    const std::map<LandmarkId, LandmarkCategorizatonInterface::Category>& getLandmarkCategories() const {
        return landmark_categories_;
    }

    /**
     * @brief getLastSelection, get last selection, f.e. for pose only estimation.
     * @return last set of landmarks that was produced by select.
     */
    std::set<LandmarkId> getLastSelection() const {
        return last_selected_lms_;
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

private:                                                // attributes
    std::map<LandmarkId, unsigned int> unselected_lms_; ///< save which landmarks where not selected
    /// and count how often they were unseletected
    std::map<LandmarkId, TimestampNSec> last_time_seen_; ///< save last time lm was unselected
    std::set<LandmarkId> last_selected_lms_;             ///< save which landmarks where not selected

    std::map<LandmarkId, LandmarkCategorizatonInterface::Category> landmark_categories_;

    ///@brief labels from semantic labeling that should be treated as outliers
    const std::set<int> outlier_labels_{
        23, // sky
        24, // kind of vehicle
        25, // motorcycle
        26  // vehicle
    };

public: // attributes
    std::vector<LandmarkSelectionSchemeBase::ConstPtr>
        selection_schemes_; ///< schemes that select landmarks that will definitely be taken.
    std::vector<LandmarkSparsificationSchemeBase::ConstPtr>
        sparsification_schemes_; ///< schemes that sparsifiy all landmarks that are left.
    std::vector<LandmarkRejectionSchemeBase::ConstPtr>
        rejection_schemes_;            ///< schemes non selected lms will definitely not be taken.
    std::set<LandmarkId> outlier_ids_; ///< ids of outliers
};
}
