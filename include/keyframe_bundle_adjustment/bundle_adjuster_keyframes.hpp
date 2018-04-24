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

#include "keyframe.hpp"
#include "landmark_selector.hpp"
#include "internal/triangulator.hpp"

///@brief forward declaration of ceres problem, to enforce encapsulation
namespace ceres {
class Problem;
}

namespace keyframe_bundle_adjustment {

/**
 * @brief The BundleAdjusterKeyframes class
 * Used to do the bundle adjustment with keyframes.
 * Push keyframes to problem, triangulate landmarks that are not yet initialized, build ceres
 * problem and solve it.
 */
class BundleAdjusterKeyframes {
public:
    using UPtr = std::unique_ptr<keyframe_bundle_adjustment::BundleAdjusterKeyframes>;
    using Ptr = std::shared_ptr<keyframe_bundle_adjustment::BundleAdjusterKeyframes>;

private:                                                     // structs
    using v3 = Eigen::Matrix<double, 3, 1>;                  ///< def for 3d vectors
    using t3 = Eigen::Transform<double, 3, Eigen::Isometry>; ///< def for 3d poses

    ///@brief input for triangulator
    using PoseAndRay = std::pair<std::shared_ptr<t3>, v3>;

    using ResidualBlockSize = int;
    using ResidualIdMap = std::map<ResidualId, std::pair<LandmarkId, ResidualBlockSize>>;

    enum class OptimizationFlags { MotionOnly };

public: // exceptions
    struct NotEnoughKeyframesException : public std::exception {

        NotEnoughKeyframesException(int num_is, int num_should_be) : num_is(num_is), num_should_be(num_should_be) {
            ;
        }
        virtual const char* what() const throw() {
            std::stringstream ss;
            ss << "Not enough keyframes available in bundle_adjuster_keyframes. Should be " << num_should_be << " is "
               << num_is;
            return ss.str().c_str();
        }

        int num_is;        ///< number of keyframes that is present
        int num_should_be; ///< number of keyframes that should be present
    };
    struct KeyframeNotFoundException : public std::exception {
        KeyframeNotFoundException(TimestampNSec timestamp) : ts_(timestamp) {
            ;
        }
        virtual const char* what() const throw() {
            std::stringstream ss;
            ss << "keyframe corresponding to timestamp " << ts_ << " nano seconds not found";
            return ss.str().c_str();
        }

        TimestampNSec ts_; ///< timestamp for which keyframe was not found
    };

    struct OutlierRejectionOptions {
        double depth_thres{0.16};       ///< threshold in meters for cauchy loss used to robustify depth reidual
        double reprojection_thres{1.6}; ///< threshold in pixel for huber loss used to robustify reprojection error
        double depth_quantile{0.95};    ///< Quantile for outlier rejection in depth 0.9 means that the 10% highest
                                        /// residuals are rejected each
        /// iteration.
        double reprojection_quantile{0.95}; ///< Quantile for outlier rejection in reprojection 0.9 means that the 10%
                                            /// highest residuals are rejected each
        /// iteration.
        int num_iterations{1}; ///< Number of iterations for qunatile outlier rejection (==trimmed least squares).
    };

public: // methods
        /**
         * @brief BundleAdjusterKeyframes, default -> take all landmarks
         */
    BundleAdjusterKeyframes();

    /**
     * @brief BundleAdjusterKeyframes, constructor
     */
    BundleAdjusterKeyframes(LandmarkSelectionSchemeBase::ConstPtr);

    //    virtual ~BundleAdjusterKeyframes() = default;

    /**
     * @brief push, add keyframe to storage, triangulate landmarks if they are not yet known and
     * add residuals to ceres problem
     * @param kf, keyframe that shall be added
     */
    void push(const Keyframe& kf);

    /**
     * @brief push, wrapper for several keyframes
     * @param kfs, keyframes that shall be added
     */
    void push(const std::vector<Keyframe>& kfs);

    /**
     * @brief Checks if a landmark obversed in a given keyframe contains depth information
     * @param kf, keyframe
     * @Kandmark lId, id of the landmark which s observed by the given keyframe
     * @return True, if the observed measurement of the landmark has a valid depth value
     */
    bool containsDepth(const Keyframe& kf, const LandmarkId lId) const;

    /**
    * @brief optimize, use ceres to optimize poses and landmarks
    * @return full summary
    */
    std::string solve();

    /**
     * @brief deactivateKeyframes, find keyframes that do not have any new measurements (graphcut?)
     * and remove cost for landmarks and poses
     * @param min_num_connecting_landmarks, if number of tracklets that connect first to current frame is smaller than
     * this, we set the end of the optimization window
     * @param min_time_sec, minimum optimization window size in delta time (seconds)
     * @param max_time_sec, maximum optimization window size in delta time (seconds)
     */
    void deactivateKeyframes(int min_num_connecting_landmarks = 3,
                             double min_time_sec = 3.0,
                             double max_time_sec = 5.0);

    /**
     * @brief getKeyframe, const getter for keyframe at timestamp
     * @param timestamp, timestamp in seconds, if smaller 0., get latest timestamp
     * @return keyframe
     */
    const Keyframe& getKeyframe(TimestampSec timestamp = -1.) const;

    /**
     * @brief getActiveKeyframes, const getter for active keyframes
     * @return keyframes
     */
    std::map<KeyframeId, Keyframe::Ptr> getActiveKeyframePtrs() const;
    std::map<KeyframeId, Keyframe::ConstPtr> getActiveKeyframeConstPtrs() const;

    /**
     * @brief getSortedKeyframePtrs,
     * @return sorted vector of keyframes, begins with the oldest frame
     */
    std::vector<Keyframe::Ptr> getSortedActiveKeyframePtrs() const;

    /**
     * @brief getActiveLandmarks
     * @return
     */
    std::map<LandmarkId, Landmark::ConstPtr> getActiveLandmarkConstPtrs() const;

    /**
     * @brief getSelectedLandmarks, get landmarks that where selected for alst optimization. This
     * should be a subset of getActiveLandmarks
     * @return
     */
    std::map<LandmarkId, Landmark::ConstPtr> getSelectedLandmarkConstPtrs() const;

    /**
     * @brief evaluateResiduals, evaluate last ceres problem two times with and without loss to get
     * outliers and residuals
     */
    void evaluateResiduals();

    /**
     * @brief calculates landmark position in world space using prior depth information of
     * measurements
     * @param kf keyFrame from which Pose the landmark is calculated using the measured depth
     * @param lId Unique Id of the new landmark
     * @poseAbs calculated position of the new landmark in world frame
     * @return true, if calculation successful
     */
    bool calculateLandmark(const Keyframe& kf, const LandmarkId& lId, v3& posAbs);

    /**
     * @brief calculates landmark position in world space using triangulation
     * @param lId Unique Id of the new landmark
     * @poseAbs calculated position of the new landmark in world frame
     * @return true, if calculation successful
     */
    bool calculateLandmark(const LandmarkId& lId, v3& posAbs);

    /**
     * @brief set_solver_time, set solver time of problem
     * @param solver_time_sec
     */
    void set_solver_time(double solver_time_sec);

    /**
     * @brief updateOutliers, update outlier flags
     * @param t
     */
    void updateLabels(const Tracklets& t, double shrubbery_weight = 1.);

private: // methods
         /**
          * @brief getMeasurementsAndPoses, convenience function to get measurements and poses from
          * keyframes for trinagulation
          * @param id, landmark id
          * @return input for triangulator
          */
    std::vector<PoseAndRay> getMeasurementsAndPoses(const LandmarkId& id);

public:
    std::map<KeyframeId, Keyframe::Ptr> keyframes_; ///< storage for all keyframes
    std::map<LandmarkId, Landmark::Ptr> landmarks_; ///< storage for all landmarks
    std::set<KeyframeId> active_keyframe_ids_;      ///< save ids of active keyframes
    std::set<LandmarkId> active_landmark_ids_;      ///< save ids of active landmarks (landmarks that could be selected)
    std::set<LandmarkId> selected_landmark_ids_;    ///< landmarks that were selected for optimization
    OutlierRejectionOptions outlier_rejection_options_;   ///< options for robustifiers
    std::unique_ptr<LandmarkSelector> landmark_selector_; ///< pointer to landmark selector which
                                                          /// follows one or more selection schemes

    ///@brief labels from semantic labeling that should be treated as outliers and weighted down
    /// If you want to set more, edit them from outside.
    std::map<std::string, std::set<int>> labels_{{"outliers",
                                                  {
                                                      23, // sky
                                                      24, // kind of vehicle
                                                      25, // motorcycle
                                                      26  // vehicle
                                                  }},
                                                 {"shrubbery", {21}}};

private:                                      // attributes
    Triangulator<double> triangulator_;       ///< triangulator instance
    std::shared_ptr<ceres::Problem> problem_; ///< instance of ceres problem
    double solver_time_sec;                   ///< solver time in seconds for ceres

private: // methods
         /**
          * @brief deactivateParameters; deactivate parameters for the optimization, f.e. deactivate
          * first pose as a parameter for mono or landmarks for motion only bundle adjustment
          * @param vector with flags for deactivation
          */
    void deactivateParameters(const std::vector<OptimizationFlags>& = std::vector<OptimizationFlags>{});
    /**
     * @brief setParameterization, set local parameterization for the problem
     */
    void setParameterization();

    /**
     * @brief addScaleRegularization, add regularization for scale
     * @param weight
     */
    void addScaleRegularization(double weight);

    /**
     * @brief filterLandmarksById, fundtion to filter landmarks use in getSelectedLandmarks and
     * getActiveLandmarks
     * @param landmark_ids_
     * @return
     */
    std::map<LandmarkId, Landmark::ConstPtr> filterLandmarksById(const std::set<KeyframeId>& landmark_ids_) const;

    /**
     * @brief addBlocksToProblem, add residual block and parameter blocks to problem
     */
    std::tuple<ResidualIdMap, ResidualIdMap> addBlocksToProblem();

    /**
     * @brief removeLandmarksByLoss, evaluate problems with loss and without loss and reject landmarks that have high
     * difference
     * @param quantile, qunatile to reject. 0.9 means highest 10% are rejected.
     */
    void removeLandmarksByLoss(double quantile, ResidualIdMap& res_ids);
};
}
