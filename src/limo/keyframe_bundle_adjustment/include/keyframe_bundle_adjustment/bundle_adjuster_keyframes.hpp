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

    enum class MotionParameterizationType { FixRotation, FullDOF, Bycicle };


public: // exceptions
    struct NotEnoughKeyframesException : public std::exception {

        NotEnoughKeyframesException(size_t num_is, size_t num_should_be)
                : num_is(num_is), num_should_be(num_should_be) {
            ;
        }
        const char* what() const noexcept;

        size_t num_is;        ///< number of keyframes that is present
        size_t num_should_be; ///< number of keyframes that should be present
    };
    struct KeyframeNotFoundException : public std::exception {
        KeyframeNotFoundException(TimestampNSec timestamp) : ts_(timestamp) {
            ;
        }
        const char* what() const noexcept;

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
         * @brief BundleAdjusterKeyframes, default -> take all landmarks, which fullfill cheirality constraint.
         */
    BundleAdjusterKeyframes();

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
                             int min_size_optimization_window = 4,
                             int max_size_optimization_window = 20);

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
    std::vector<std::pair<KeyframeId, Keyframe::Ptr>> getSortedIdsWithActiveKeyframePtrs() const;

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
     * @brief adjust_pose_only, adjust keyframe without adding it to memory or overall bundle_adjustment.
     * @param Keyframe to be adjusted.
     */
    std::string adjustPoseOnly(Keyframe&);

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
     * @param poseAbs, calculated position of the new landmark in world frame
     * @return true, if calculation successful
     */
    bool calculateLandmark(const Keyframe& kf, const LandmarkId& lId, v3& posAbs);

    /**
     * @brief calculates landmark position in world space using triangulation
     * @param lId Unique Id of the new landmark
     * @param poseAbs calculated position of the new landmark in world frame
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

    ///@brief labels from semantic labeling that should be treated as outliers and weighted down.
    ///       The values set here are from cityscapes labels
    ///       (https://github.com/mcordts/cityscapesScripts/).
    ///       If you want to set more, edit them from outside.
    std::map<std::string, std::set<int>> labels_{{"outliers",
                                                  {
                                                      23, // sky
                                                      24, // kind of vehicle
                                                      25, // motorcycle
                                                      26, // vehicle
                                                      27, // truck
                                                      28, // bus
                                                      29, // caravan
                                                      30, // trailer
                                                      31, // train
                                                      32, // motorcycle
                                                      33  // bicycle
                                                  }},
                                                 {"shrubbery",
					          {
						      21 // vegetation
						  }},
                                                 {"ground",
					          {
					              6, // ground
						      7, // road
						      8, // sidewalk
						      9, // parking
						      10 // rail track
						  }}};

private:                                      // attributes
    Triangulator<double> triangulator_;       ///< triangulator instance
    std::shared_ptr<ceres::Problem> problem_; ///< instance of ceres problem
    double solver_time_sec;                   ///< solver time in seconds for ceres

private: // methods
    /// //////////////////////////////////////////////
    /// \brief deactivatePoseParameters
    /// Set pose parameters constant for the optimization, f.e. deactivate first pose
    /// paramters for mono (set={Keyframe::FixationStatus::Pose}) or do structure only.
    ///
    void deactivatePoseParameters(const std::set<Keyframe::FixationStatus>& flags);

    //    /// //////////////////////////////////////////////
    //    /// \brief activatePoseParameters
    //    /// Set pose parameters variable for the optimization.
    //    ///
    //    void activatePoseParameters(const std::set<Keyframe::FixationStatus>& flags);

    /// //////////////////////////////////////////////
    /// \brief deactivateLandmarks
    /// Set landmark parameters constant. Calling this results in motion only optimization.
    ///
    void deactivateLandmarks(double keyframe_fraction = 1.0, double landmark_fraction = 1.0);

    /// /////////////////////////////////////////////
    /// \brief activateLandmarks
    /// Set parameters of landmarks non constant.
    void activateLandmarks();

    /**
     * @brief setParameterization, set local parameterization for the problem
     */
    void setParameterization(Keyframe& kf, MotionParameterizationType type);

    /**
     * @brief addScaleRegularization, add regularization for scale
     * @param weight
     */
    void addScaleRegularization(double weight);

    void addGroundplaneRegularization(double weight);

    /**
     * @brief filterLandmarksById, fundtion to filter landmarks use in getSelectedLandmarks and
     * getActiveLandmarks
     * @param landmark_ids_
     * @return
     */
    std::map<LandmarkId, Landmark::ConstPtr> filterLandmarksById(const std::set<LandmarkId>& landmark_ids_) const;

    /**
     * @brief addKeyframeToProblem, add residual blocks and parameter blocks from one keyframe to problem
     */
    void addKeyframeToProblem(Keyframe& kf,
                              ResidualIdMap& residual_block_ids_depth,
                              ResidualIdMap& residual_block_ids_repr,
                              std::shared_ptr<bool> compensate_rotation = std::make_shared<bool>(false));
    /**
     * @brief addKeyframesToProblem, add residuals and parameters for all keyframes.
     * @return
     */
    std::vector<ResidualIdMap> addActiveKeyframesToProblem();

    /**
     * @brief removeLandmarksByLoss, evaluate problems with loss and without loss and reject landmarks that have high
     * difference
     * @param quantile, qunatile to reject. 0.9 means highest 10% are rejected.
     */
    void removeLandmarksByLoss(double quantile, ResidualIdMap& res_ids);

    /**
    * @brief addGroundPlaneResiduals, add residuals for ground plane to problem. Residuals can be deactviated by setting
    * height < -10.;
    * @param weight, weight of the ehgith regularization. Scale of residual =weight/distance(landmark to next pose)
    * @param residual_block_ids_gp, residual id map (residual ids to group and residual size)
    */
    void addGroundPlaneResiduals(double weight, BundleAdjusterKeyframes::ResidualIdMap& residual_block_ids_gp);
};
}
