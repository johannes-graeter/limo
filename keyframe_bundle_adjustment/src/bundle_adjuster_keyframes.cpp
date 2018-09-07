// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "bundle_adjuster_keyframes.hpp"

#include <chrono>
#include <exception>
#include <fstream>
#include <random>
#include <sstream>
#include <ceres/ceres.h>
#include "internal/cost_functors_ceres.hpp"
#include "internal/definitions.hpp"
#include "internal/local_parameterizations.hpp"
#include "internal/motion_model_regularization.hpp"

#include "landmark_selection_schemes.hpp"

#include <robust_optimization/robust_solving.hpp>


namespace keyframe_bundle_adjustment {

namespace {

/**
 * @brief Checks if a landmark obversed in a given keyframe contains depth information
 * @param kf, keyframe
 * @param LandmarkId, id of the landmark which s observed by the given keyframe
 * @return True, if the observed measurement of the landmark has a valid depth value
 */
bool containsDepth(const Keyframe& kf, const LandmarkId lId) {
    // Get all measuremets of the given landmark in the keyframe
    const auto& landmarkMeasurements = kf.measurements_.at(lId);

    for (const auto& cam_meas : landmarkMeasurements) {
        // check if the measurement of minimum 1 camera contains a valid depth value
        if (cam_meas.second.d >= 0)
            return true;
    }

    return false;
}

ceres::Solver::Options getSolverOptions(double solver_time_sec) {
    ceres::Solver::Options options;
    // how to choose solver http://ceres-solver.org/solving_faqs.html
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //    solverOptions_.num_threads = ceres::CERES_NUM_THREADS_DEFAULT;
    //    solverOptions_.num_linear_solver_threads = CERES_NUM_THREADS_DEFAULT;
    options.max_num_iterations = 100;
    // options.parameter_tolerance = 1.e-10;
    // options.function_tolerance = 1.e-10;
    // options.gradient_tolerance = 1.e-10;
    options.max_solver_time_in_seconds = solver_time_sec;
    options.minimizer_progress_to_stdout = true;
    //    options.update_state_every_iteration = true; // needed for callbacks
    return options;
}

ceres::Solver::Options getSolverOptionsMotionOnly() {
    ceres::Solver::Options options;
    // how to choose solver http://ceres-solver.org/solving_faqs.html
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 30;
    options.max_num_iterations = 4;
    //    options.max_solver_time_in_seconds = 0.1 ;
    options.minimizer_progress_to_stdout = true;
    return options;
}

ceres::Solver::Options getSolverOptionsNumIter(int num_iterations) {
    ceres::Solver::Options options = getSolverOptions(100.);
    options.max_num_iterations = num_iterations;
    return options;
}
/**
 * @brief getCommonLandmarks, get set of keyframes that have connecting landmarks
 * @param cur_kf, keyframe that shall be tested
 * @param newest_kf, keyframe with biggest timestamp
 * @return vector with common landmarks
 */
std::vector<LandmarkId> getCommonLandmarkIds(const Keyframe& cur_kf, const Keyframe& newest_kf) {
    // copy ids to vector
    std::vector<LandmarkId> cur_lm_id;
    cur_lm_id.resize(cur_kf.measurements_.size());
    std::transform(cur_kf.measurements_.cbegin(), cur_kf.measurements_.cend(), cur_lm_id.begin(), [](const auto& a) {
        return a.first;
    });

    std::vector<LandmarkId> newest_lm_id;
    newest_lm_id.resize(newest_kf.measurements_.size());
    std::transform(newest_kf.measurements_.cbegin(),
                   newest_kf.measurements_.cend(),
                   newest_lm_id.begin(),
                   [](const auto& a) { return a.first; });

    // get intersection
    std::vector<LandmarkId> id_intersection;
    std::set_intersection(newest_lm_id.cbegin(),
                          newest_lm_id.cend(),
                          cur_lm_id.cbegin(),
                          cur_lm_id.cend(),
                          std::back_inserter(id_intersection));
    return id_intersection;
}
}


BundleAdjusterKeyframes::BundleAdjusterKeyframes() : solver_time_sec(0.2) {
    landmark_selector_ = std::make_unique<LandmarkSelector>();
    // first is always cheirality since it is fast and reliable
    landmark_selector_->addScheme(LandmarkRejectionSchemeCheirality::create());

    // instantiate ceres problem
    problem_ = std::make_shared<ceres::Problem>();
}


std::vector<BundleAdjusterKeyframes::PoseAndRay> BundleAdjusterKeyframes::getMeasurementsAndPoses(
    const LandmarkId& id) {
    std::vector<PoseAndRay> out;
    // get measurements from all keyframes for all cameras as seperate rays for triangulation
    for (auto& active_kf_id : active_keyframe_ids_) {
        auto& kf = *(keyframes_.at(active_kf_id));
        for (const auto& id_and_cam : kf.cameras_) {
            const auto& cam_id = id_and_cam.first;
            Camera::Ptr cam = id_and_cam.second;

            if (kf.hasMeasurement(id, cam_id)) {
                assert(std::isfinite(kf.getMeasurement(id, cam_id).u) &&
                       std::isfinite(kf.getMeasurement(id, cam_id).v));

                // poses are defined pointing to origin
                EigenPose pose_cam_origin = cam->getEigenPose() * kf.getEigenPose();

                //                std::cout << "cam pose: " << std::endl <<
                //                cam->getEigenPose().matrix() << std::endl;
                //                std::cout << "kf pose: " << std::endl <<
                //                kf.getEigenPose().matrix() << std::endl;
                //                std::cout << "pose_cam_origin: " << std::endl <<
                //                pose_cam_origin.matrix() << std::endl;
                //                std::cout << "inverse: " << std::endl <<
                //                pose_cam_origin.inverse().matrix() << std::endl;

                Eigen::Matrix<double, 3, 1> ray_cam =
                    convertMeasurementToRay(cam->intrin_inv, kf.getMeasurement(id, cam_id));
                out.push_back(std::make_pair(std::make_shared<t3>(pose_cam_origin.inverse()), ray_cam));
            }
        }
    }

    return out;
}

void BundleAdjusterKeyframes::setParameterization(Keyframe& kf,
                                                  BundleAdjusterKeyframes::MotionParameterizationType type) {
    if (problem_->HasParameterBlock(kf.pose_.data()) && problem_->GetParameterization(kf.pose_.data()) == NULL) {
        // add local parametrization to implement motion model if it pose was added as variable
        // and
        // parameterization was not set.
        // -> less parameters while being able to use convenient x,y,z representation

        // full 6 dofs
        ceres::LocalParameterization* motion_parameterization;
        // if keyframe is fixed no contraint is added -> return
        if (type == MotionParameterizationType::FixRotation) {
            motion_parameterization = new ceres::ProductParameterization(
                new ceres::SubsetParameterization(4, {0, 1, 2, 3}), new ceres::IdentityParameterization(3));
        } else if (type == MotionParameterizationType::Bycicle) {
            //            const auto& last_kf = getKeyframe();
            //            motion_parameterization =
            //            local_parameterizations::CircularMotionPlus2d::Create(last_kf.getEigenPose());
            motion_parameterization = local_parameterizations::CircularMotionPlus2d::Create();
        } else {
            motion_parameterization = new ceres::ProductParameterization(new ceres::QuaternionParameterization(),
                                                                         new ceres::IdentityParameterization(3));
        }

        problem_->SetParameterization(kf.pose_.data(), motion_parameterization);

        if (problem_->HasParameterBlock(kf.local_ground_plane_.direction.data()) &&
            problem_->GetParameterization(kf.local_ground_plane_.direction.data()) == NULL) {
            // Plane parameterization.
            ceres::LocalParameterization* plane_parameterization =
                new ceres::AutoDiffLocalParameterization<local_parameterizations::FixScaleVectorPlus, 3, 3>(
                    new local_parameterizations::FixScaleVectorPlus(1.0));
            problem_->SetParameterization(kf.local_ground_plane_.direction.data(), plane_parameterization);
        }
    }
}

void BundleAdjusterKeyframes::deactivatePoseParameters(const std::set<Keyframe::FixationStatus>& flags) {
    // set parameters constant if pose fixation status is flagged.
    for (const auto& id : active_keyframe_ids_) {
        auto& kf = *keyframes_[id];
        if (flags.find(kf.fixation_status_) != flags.cend()) {
            // Pose is constant.
            if (problem_->HasParameterBlock(kf.pose_.data())) {
                problem_->SetParameterBlockConstant(kf.pose_.data());
            }

            // Groundplane is constant.
            // This is important for startup.
            if (problem_->HasParameterBlock(kf.local_ground_plane_.direction.data())) {
                problem_->SetParameterBlockConstant(kf.local_ground_plane_.direction.data());
            }

            if (problem_->HasParameterBlock(&kf.local_ground_plane_.distance)) {
                problem_->SetParameterBlockConstant(&kf.local_ground_plane_.distance);
            }
        }
    }
}

void BundleAdjusterKeyframes::deactivateLandmarks(double keyframe_fraction, double landmark_fraction) {

    std::set<LandmarkId> all_lm_ids_to_deactivate;
    // Get all lm ids in lower part of keyframes (lower fraction*100. percent of the keyframes).
    if (keyframe_fraction == 1.0) {
        all_lm_ids_to_deactivate = active_landmark_ids_;
    } else {
        long max_ind =
            static_cast<long>(std::floor(static_cast<double>(active_keyframe_ids_.size()) * keyframe_fraction));
        auto it = active_keyframe_ids_.cbegin();
        auto end_it = std::next(active_keyframe_ids_.cbegin(), max_ind);
        for (; it != end_it; it++) {
            const auto& kf_ptr = keyframes_.at(*it);
            for (const auto& m : kf_ptr->measurements_) {
                all_lm_ids_to_deactivate.insert(m.first);
            }
        }
    }

    // Draw landmark_fraction*100. % of the lm_ids randomly to fix them.
    std::set<LandmarkId> lm_ids_to_deactivate;
    if (landmark_fraction == 1.0) {
        lm_ids_to_deactivate = all_lm_ids_to_deactivate;
    } else {
        // Copy to vec so we do not have to access lm_ids in set.
        std::vector<LandmarkId> all_lm_ids_to_deactivate_vec;
        all_lm_ids_to_deactivate_vec.reserve(all_lm_ids_to_deactivate.size());
        std::copy(all_lm_ids_to_deactivate.cbegin(),
                  all_lm_ids_to_deactivate.cend(),
                  std::back_inserter(all_lm_ids_to_deactivate_vec));
        // Draw randomly.
        std::default_random_engine generator;
        std::uniform_int_distribution<size_t> distribution(0, all_lm_ids_to_deactivate_vec.size());
        size_t num_lms = static_cast<size_t>(
            std::floor(static_cast<double>(all_lm_ids_to_deactivate_vec.size()) * landmark_fraction));
        for (size_t i = 0; i < num_lms; ++i) {
            size_t rand_num = distribution(generator);
            lm_ids_to_deactivate.insert(all_lm_ids_to_deactivate_vec[rand_num]);
        }
    }

    // Deactivate landmark parameters for motion only bundle adjustment.
    for (const auto& lm_id : lm_ids_to_deactivate) {
        if (landmarks_.find(lm_id) != landmarks_.cend() && // can this be economized?
            problem_->HasParameterBlock(landmarks_.at(lm_id)->pos.data()) &&
            !problem_->IsParameterBlockConstant(landmarks_.at(lm_id)->pos.data())) {
            problem_->SetParameterBlockConstant(landmarks_.at(lm_id)->pos.data());
        }
    }
}

void BundleAdjusterKeyframes::activateLandmarks() {
    for (auto& lm_id : active_landmark_ids_) {
        if (landmarks_.find(lm_id) != landmarks_.cend() &&
            problem_->HasParameterBlock(landmarks_.at(lm_id)->pos.data()) &&
            problem_->IsParameterBlockConstant(landmarks_.at(lm_id)->pos.data())) {
            problem_->SetParameterBlockVariable(landmarks_.at(lm_id)->pos.data());
        }
    }
}

void BundleAdjusterKeyframes::push(const std::vector<Keyframe>& kfs) {
    for (const auto& kf : kfs) {
        push(kf);
    }
}


void BundleAdjusterKeyframes::push(const Keyframe& kf) {
    // copy keyframe since we want to assign residual ids
    ///@todo move keyframe here

    // add keyframe
    // here use timestamps to identify, but can be any unique id
    keyframes_[kf.timestamp_] = std::make_shared<Keyframe>(kf);

    // add keyframe id to active frames to optimize it
    active_keyframe_ids_.insert(kf.timestamp_);

    // create landmarks if they do not exist
    for (const auto& m : kf.measurements_) {
        auto it = landmarks_.find(m.first);
        // if no landmark was found: initialize new one by triangulation
        // else use existing landmark
        if (it == landmarks_.cend()) {
            // create new landmark with depth information if available, else triangulate it
            v3 p;
            bool has_depth = containsDepth(kf, m.first);

            bool success = has_depth ? calculateLandmark(kf, m.first, p) : calculateLandmark(m.first, p);

            if (!success) {
                continue;
            }

            // add landmark to storage and get iterator
            bool was_inserted;
            std::tie(it, was_inserted) =
                landmarks_.insert(std::make_pair(m.first, std::make_shared<Landmark>(p, has_depth)));
            assert(was_inserted == true && "new calculated landmark couldn't be inserted");

            // set triangulated landmark as active, may be deactivated
            active_landmark_ids_.insert(m.first);
        } else {
            // set triangulated landmark as active, may be deactivated
            active_landmark_ids_.insert(m.first);
        }
    }
}


bool BundleAdjusterKeyframes::calculateLandmark(const Keyframe& kf, const LandmarkId& lId, v3& posAbs) {
    // Get all measuremets of the given landmark in the keyframe
    const auto& landmarkMeasurements = kf.measurements_.at(lId);

    for (const auto& m : landmarkMeasurements) {
        // check if the measurement of minimum 1 camera contains a valid depth value
        if (m.second.d < 0) {
            continue;
        }
        const auto cam = kf.cameras_.at(m.first);

        // calculate landmark position in camera frame
        const double z = static_cast<double>(m.second.d);
        const double x = (static_cast<double>(m.second.u) - cam->principal_point.x()) * z / cam->focal_length;
        const double y = (static_cast<double>(m.second.v) - cam->principal_point.y()) * z / cam->focal_length;

        // transform landmark position from camera frame into world frame
        posAbs = (cam->getEigenPose() * kf.getEigenPose()).inverse() * v3(x, y, z);

        return true;
    }

    return false;
}


bool BundleAdjusterKeyframes::calculateLandmark(const LandmarkId& lId, v3& posAbs) {
    // search correspondant measures and triangulate
    const auto& measurements_and_poses = getMeasurementsAndPoses(lId);

    // don't add to problem in case we don't have enough measurements to triangulate
    if (measurements_and_poses.size() < 2) {
        return false;
    }

    // triangulate landmark
    // they origin in the same coordinate system as the poses
    posAbs = triangulator_.triangulate_rays(measurements_and_poses);

    return true;
}

void BundleAdjusterKeyframes::set_solver_time(double solver_time_sec) {
    this->solver_time_sec = solver_time_sec;
}

void BundleAdjusterKeyframes::updateLabels(const Tracklets& t, double shrubbery_weight) {
    // Mark outliers.
    {
        // Retain outliers that are still in optimization window.
        std::set<LandmarkId> outlier_ids;
        for (const auto& outlier_id : landmark_selector_->getOutliers()) {
            if (active_landmark_ids_.find(outlier_id) != active_landmark_ids_.cend()) {
                outlier_ids.insert(outlier_id);
            }
        }

        // Add new outliers if they are marked as outlier
        // or if their label should be treated as outlier.
        for (const auto& track : t.tracks) {
            bool is_outlier_label = (labels_["outliers"].find(track.label) != labels_["outliers"].cend());
            if (track.is_outlier || is_outlier_label) {
                outlier_ids.insert(track.id);
            }
        }

        // Delete old outliers.
        landmark_selector_->clearOutliers();

        // Set new outliers.
        landmark_selector_->setOutlier(outlier_ids);
    }

    // Update weights for landmarks according to label.
    {
        for (const auto& track : t.tracks) {
            bool is_shrubbery = (labels_["shrubbery"].find(track.label) != labels_["shrubbery"].cend());
            bool is_lm_initialized = active_landmark_ids_.find(track.id) != active_landmark_ids_.cend();
            if (is_lm_initialized) {
                if (is_shrubbery) {
                    landmarks_.at(track.id)->weight = shrubbery_weight;
                }

                // Shouldn't be true if is_shrubbery.
                landmarks_.at(track.id)->is_ground_plane =
                    (labels_["ground"].find(track.label) != labels_["ground"].cend());
            }
        }
    }
}

std::map<LandmarkId, Landmark::ConstPtr> BundleAdjusterKeyframes::getActiveLandmarkConstPtrs() const {
    return filterLandmarksById(active_landmark_ids_);
}

std::map<LandmarkId, Landmark::ConstPtr> BundleAdjusterKeyframes::getSelectedLandmarkConstPtrs() const {
    return filterLandmarksById(selected_landmark_ids_);
}

std::map<LandmarkId, Landmark::ConstPtr> BundleAdjusterKeyframes::filterLandmarksById(
    const std::set<LandmarkId>& landmark_ids_) const {
    std::map<LandmarkId, Landmark::ConstPtr> filtered_landmarks;
    for (const auto& id : landmark_ids_) {
        auto it = landmarks_.find(id); // it may be possible that active landmarks couldn't be reconstructed
        if (it != landmarks_.cend()) {
            filtered_landmarks[id] = std::const_pointer_cast<const Landmark>(it->second);
        }
    }

    return filtered_landmarks;
}

std::map<KeyframeId, Keyframe::Ptr> BundleAdjusterKeyframes::getActiveKeyframePtrs() const {
    std::map<KeyframeId, Keyframe::Ptr> filtered_kfs;
    for (const auto& id : active_keyframe_ids_) {
        filtered_kfs[id] = keyframes_.at(id);
    }
    return filtered_kfs;
}

std::map<KeyframeId, Keyframe::ConstPtr> BundleAdjusterKeyframes::getActiveKeyframeConstPtrs() const {
    std::map<KeyframeId, Keyframe::ConstPtr> filtered_kfs;
    for (const auto& id : active_keyframe_ids_) {
        filtered_kfs[id] = std::const_pointer_cast<const Keyframe>(keyframes_.at(id));
    }
    return filtered_kfs;
}


std::vector<std::pair<KeyframeId, Keyframe::Ptr>> BundleAdjusterKeyframes::getSortedIdsWithActiveKeyframePtrs() const {
    // Newest will be last.
    // Put keyframes into vector and sort them.
    std::vector<std::pair<KeyframeId, Keyframe::Ptr>> kf_ptrs;
    auto active_kf_ptrs = getActiveKeyframePtrs();
    kf_ptrs.reserve(active_kf_ptrs.size());
    for (const auto& kf : active_kf_ptrs) {
        // Only use active keyframes.
        kf_ptrs.push_back(std::make_pair(kf.first, kf.second));
    }

    // Keyframes have < operator defined.
    std::sort(kf_ptrs.begin(), kf_ptrs.end(), [](const auto& a, const auto& b) { return *(a.second) < *(b.second); });

    return kf_ptrs;
}

std::vector<Keyframe::Ptr> BundleAdjusterKeyframes::getSortedActiveKeyframePtrs() const {
    auto ids_kfs = getSortedIdsWithActiveKeyframePtrs();
    std::vector<Keyframe::Ptr> out;
    out.reserve(ids_kfs.size());
    for (const auto& el : ids_kfs) {
        out.push_back(el.second);
    }
    return out;
}

std::vector<BundleAdjusterKeyframes::ResidualIdMap> BundleAdjusterKeyframes::addActiveKeyframesToProblem() {
    // Save residual block ids from depth.
    ResidualIdMap residual_block_ids_depth;
    ResidualIdMap residual_block_ids_repr;
    ResidualIdMap residual_block_ids_gp;

    // Add residuals for all landmarks in all keyframes.
    for (auto& id_kf : active_keyframe_ids_) {
        auto& kf = *keyframes_.at(id_kf);
        addKeyframeToProblem(kf, residual_block_ids_depth, residual_block_ids_repr);
    }

    // Optimize distance to groundplane if scale was observed by landmark depth.
    double scale_gp_reg = 10.; // Add ground plane residual if landmark is in vicinity of pose.
    addGroundPlaneResiduals(scale_gp_reg, residual_block_ids_gp);

    return {residual_block_ids_depth, residual_block_ids_repr, residual_block_ids_gp};
}

void BundleAdjusterKeyframes::addGroundPlaneResiduals(double weight,
                                                      BundleAdjusterKeyframes::ResidualIdMap& residual_block_ids_gp) {
    for (const auto& lm_id : selected_landmark_ids_) {
        if (landmarks_.at(lm_id)->is_ground_plane) {
            Eigen::Map<Eigen::Vector3d> cur_lm(landmarks_.at(lm_id)->pos.data());

            // Get closest keyframe to landmark.
            double min_dist = std::numeric_limits<double>::max();
            KeyframeId kf_id;
            for (const auto& cur_kf_id : active_keyframe_ids_) {
                if (keyframes_.at(cur_kf_id)->local_ground_plane_.distance < -10.) {
                    continue;
                }
                double dist = (keyframes_.at(cur_kf_id)->getEigenPose() * cur_lm).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    kf_id = cur_kf_id;
                }
            }
            // Pass condition: is valid (not deactivated and there is a pose that is close enough)
            if (min_dist == std::numeric_limits<double>::max()) {
                continue;
            }

            // Add residual.
            ceres::CostFunction* cost_functor_gp = cost_functors_ceres::GroundPlaneHeightRegularization::Create();

            // Lower weight for very distant features->better for local plane assumption.
            // Do that only outside of optimization, since otherwise all landmark distances would tend to zero.
            // Only add if weight is big enough, otherwise very distant features could crash scale.
            double max_valid_dist = 25.;
            if (min_dist < max_valid_dist) {
                double robust_loss_scale = 0.1;
                double loss_weight = weight * (1. - min_dist / max_valid_dist);
                ceres::ResidualBlockId res_id_gp = problem_->AddResidualBlock(
                    cost_functor_gp,
                    new ceres::ScaledLoss(new ceres::HuberLoss(robust_loss_scale), loss_weight, ceres::TAKE_OWNERSHIP),
                    keyframes_.at(kf_id)->pose_.data(),
                    keyframes_.at(kf_id)->local_ground_plane_.direction.data(),
                    &(keyframes_.at(kf_id)->local_ground_plane_.distance),
                    landmarks_.at(lm_id)->pos.data());
                residual_block_ids_gp[res_id_gp] = std::make_pair(lm_id, 1);
            }
        }
    }
}

void BundleAdjusterKeyframes::addKeyframeToProblem(Keyframe& kf,
                                                   BundleAdjusterKeyframes::ResidualIdMap& residual_block_ids_depth,
                                                   BundleAdjusterKeyframes::ResidualIdMap& residual_block_ids_repr,
                                                   std::shared_ptr<bool> compensate_rotation) {
    // Add measruement constraints.
    for (const auto& m : kf.measurements_) {
        // Add to problem only if landmark is selected.
        if (selected_landmark_ids_.find(m.first) != selected_landmark_ids_.cend()) {

            for (const auto& cam_id_meas : m.second) {
                const Camera::Ptr& cam = kf.cameras_.at(cam_id_meas.first);
                Pose pose_camera_veh = cam->pose_camera_vehicle;

                // Add cost functor if has measured depth.
                if (cam_id_meas.second.d > 0.0f) {
                    // Landmark depth was measured.
                    // Add cost functor with landmark, measurement and pose to estimation problem.
                    // Each residual block takes a point and a camera as input and outputs a 1 dimensional residual.
                    // Internally, the cost function stores the measured depth of the landmark (distance between camera
                    // and landmark) and compares the measured depth with the current depth.
                    ceres::CostFunction* cost_functor_depth = cost_functors_ceres::LandmarkDepthError::Create(
                        static_cast<double>(cam_id_meas.second.d), pose_camera_veh);

                    ceres::ResidualBlockId res_id_depth = problem_->AddResidualBlock(
                        cost_functor_depth,
                        new ceres::ScaledLoss(new ceres::CauchyLoss(outlier_rejection_options_.depth_thres),
                                              landmarks_.at(m.first)->weight,
                                              ceres::TAKE_OWNERSHIP),
                        kf.pose_.data(),
                        landmarks_.at(m.first)->pos.data());
                    residual_block_ids_depth[res_id_depth] = std::make_pair(m.first, 1);
                }

                //                bool is_far = landmark_selector_->getLandmarkCategories().find(m.first) !=
                //                                  landmark_selector_->getLandmarkCategories().cend() &&
                //                              landmark_selector_->getLandmarkCategories().at(m.first) ==
                //                                  LandmarkCategorizatonInterface::Category::FarField;

                // Add reprojection cost functor for all landmarks (both with and without measured depth).
                // Poses are stored on keyframe as quaternion & translation, landmarks are stored as std::array<double>.
                ceres::CostFunction* cost_functor_reprojection =
                    cost_functors_ceres::ReprojectionErrorWithQuaternions::Create(
                        static_cast<double>(cam_id_meas.second.u),
                        static_cast<double>(cam_id_meas.second.v),
                        cam->focal_length,
                        cam->principal_point[0],
                        cam->principal_point[1],
                        pose_camera_veh,
                        compensate_rotation);

                ceres::ResidualBlockId res_id_repr = problem_->AddResidualBlock(
                    cost_functor_reprojection,
                    new ceres::ScaledLoss(new ceres::CauchyLoss(outlier_rejection_options_.reprojection_thres),
                                          landmarks_.at(m.first)->weight,
                                          ceres::TAKE_OWNERSHIP),
                    kf.pose_.data(),
                    landmarks_.at(m.first)->pos.data());

                // Store residual id on keyframe for later deactivation of residuals.
                residual_block_ids_repr[res_id_repr] = std::make_pair(m.first, 2);
            }
        }
    }
}

std::string BundleAdjusterKeyframes::solve() {
    if (keyframes_.size() < 3) {
        throw NotEnoughKeyframesException(keyframes_.size(), 3);
    }

    // Reinitialize ceres::Problem to get rid of old landmarks.
    ceres::Problem::Options options{};
    options.enable_fast_removal = true; // Important for removing ost functors efficiently.
    problem_ = std::make_shared<ceres::Problem>(options);

    auto start_time_lm_sel = std::chrono::steady_clock::now();
    // Process landmark selector, that was assigned in tool, on landmarks that are not deactivated.
    auto active_landmarks = getActiveLandmarkConstPtrs();
    auto active_keyframes = getActiveKeyframeConstPtrs();
    selected_landmark_ids_ = landmark_selector_->select(active_landmarks, active_keyframes);

    auto categories = landmark_selector_->getLandmarkCategories();
    int num_near = 0;
    int num_middle = 0;
    for (const auto& el : categories) {
        if (el.second == LandmarkCategorizatonInterface::Category::NearField) {
            num_near++;
        } else if (el.second == LandmarkCategorizatonInterface::Category::MiddleField) {
            num_middle++;
        }
    }

    // Reset landmarks if not enough near lms.
    /*
        if (num_near < 15 || num_middle < 20) {
            auto start_time_recovery = std::chrono::steady_clock::now();

            std::vector<LandmarkId> lms_with_gp;
            for (const auto& el : active_landmarks) {
                if (el.second->is_ground_plane) {
                    lms_with_gp.push_back(el.first);
                }
            }
            landmarks_.clear();
            for (const auto& el : active_keyframes) {
                push(*el.second);
            }
            std::cout << "--------------------- recover -------------------" << std::endl;
            active_landmarks = getActiveLandmarkConstPtrs();
            selected_landmark_ids_ = landmark_selector_->select(active_landmarks, active_keyframes);

            for (const auto& el : lms_with_gp) {
                if (landmarks_.find(el) != landmarks_.cend()) {
                    landmarks_.at(el)->is_ground_plane = true;
                }
            }
            std::cout << "Duration recovery="
                      << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                               start_time_recovery)
                             .count()
                      << " ms" << std::endl;
        }
    */
    std::cout << "duration solver:lms_selection="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_time_lm_sel)
                     .count()
              << " ms" << std::endl;

    //    deactivateKeyframes(3, 3., 100.);
    // Add all residual blocks and parameters to ceres problem_.
    auto ids = addActiveKeyframesToProblem();
    ResidualIdMap residual_block_ids_depth = ids[0];
    ResidualIdMap residual_block_ids_repr = ids[1];
    ResidualIdMap residual_block_ids_gp = ids[2];

    std::cout << "Size residual block ids gp size=" << residual_block_ids_gp.size() << std::endl;
    std::cout << "Cost gp res=" << getCost(residual_block_ids_gp, *problem_) << std::endl;

    // If scale was observed, don't fix scale but add regularization.
    if (residual_block_ids_depth.size() > 10 || residual_block_ids_gp.size() > 10) {
        // Add regularization of first pose diff for scale.
        // Weight is in function of number of observed depth measruements.
        if (residual_block_ids_gp.size() < 30) {
            double scale_reg_weight = 1000. / (static_cast<double>(residual_block_ids_depth.size() +
                                                                   static_cast<double>(residual_block_ids_gp.size())));
            addScaleRegularization(scale_reg_weight);
        }
    } else {
        // For fixed scale, do not use local parameterization, but use very high cost.
        addScaleRegularization(1000.);
        std::cout << "Add scale reg." << std::endl;
    }
    if (residual_block_ids_gp.size() > 0) {
        addGroundplaneRegularization(10.);
    }

    // Fix plane distance if it is the only scale info we got.
    if (residual_block_ids_depth.size() < 10) {
        for (const auto& el : getActiveKeyframePtrs()) {
            if (problem_->HasParameterBlock(&(el.second->local_ground_plane_.distance))) {
                problem_->SetParameterBlockConstant(&(el.second->local_ground_plane_.distance));
            }
        }
    }

    // Set local parametrizations.
    for (const auto& id_kf : keyframes_) {
        setParameterization(*id_kf.second, MotionParameterizationType::FullDOF);
    }

    // Deactivate optimization parameters, f.e. fixed poses or landmarks.
    deactivatePoseParameters({Keyframe::FixationStatus::Pose});

    // Optimization is done with trimmed least squares, with qunatile outlier rejection.
    // Only do rejection if number of measurements is sufficient.
    std::vector<int> number_iterations{};
    if (selected_landmark_ids_.size() > 100) {
        for (int i = 0; i < int(outlier_rejection_options_.num_iterations); i++) {
            number_iterations.push_back(2);
        }
    }
    std::vector<std::pair<ResidualIdMap, robust_optimization::TrimmerSpecification>> input;
    input.push_back(
        std::make_pair(residual_block_ids_depth,
                       robust_optimization::TrimmerSpecification(robust_optimization::TrimmerType::Quantile,
                                                                 outlier_rejection_options_.depth_quantile)));
    input.push_back(
        std::make_pair(residual_block_ids_repr,
                       robust_optimization::TrimmerSpecification(robust_optimization::TrimmerType::Quantile,
                                                                 outlier_rejection_options_.reprojection_quantile)));

    input.push_back(
        std::make_pair(residual_block_ids_gp,
                       robust_optimization::TrimmerSpecification(robust_optimization::TrimmerType::Quantile, 1.0)));

    robust_optimization::Options opt = robust_optimization::getStandardSolverOptions(solver_time_sec);
    opt.max_solver_time_refinement_in_seconds = solver_time_sec;
    opt.minimum_number_residual_groups = 30;
    opt.trust_region_relaxation_factor = -10.; // Reset trust region at each iteration.
    opt.num_threads = 3;
    auto final_summary = robust_optimization::solveTrimmed(number_iterations, input, *problem_, opt);
    return final_summary.FullReport();
}

void BundleAdjusterKeyframes::addGroundplaneRegularization(double weight) {
    std::cout << "Size keyframes=" << active_keyframe_ids_.size() << std::endl;
    if (active_keyframe_ids_.size() > 1) {
        std::vector<ResidualId> ids_normal;
        std::vector<ResidualId> ids_dist;
        std::vector<ResidualId> ids_motion;
        for (auto it0 = active_keyframe_ids_.cbegin(); it0 != std::prev(active_keyframe_ids_.cend()); it0++) {
            auto it1 = std::next(it0);
            //            if (problem_->HasParameterBlock(keyframes_.at(*it1)->local_ground_plane_.direction.data()) &&
            //                problem_->HasParameterBlock(keyframes_.at(*it0)->local_ground_plane_.direction.data())) {
            ResidualId normal_id = problem_->AddResidualBlock(
                cost_functors_ceres::VectorDifferenceRegularization::Create(),
                new ceres::ScaledLoss(new ceres::TrivialLoss(), 3. * weight, ceres::TAKE_OWNERSHIP),
                keyframes_.at(*it1)->local_ground_plane_.direction.data(),
                keyframes_.at(*it0)->local_ground_plane_.direction.data());
            ids_normal.push_back(normal_id);

            ResidualId dist_id = problem_->AddResidualBlock(
                cost_functors_ceres::GroundPlaneDistanceRegularization::Create(),
                new ceres::ScaledLoss(new ceres::TrivialLoss(), weight, ceres::TAKE_OWNERSHIP),
                &keyframes_.at(*it1)->local_ground_plane_.distance,
                &keyframes_.at(*it0)->local_ground_plane_.distance);
            ids_dist.push_back(dist_id);


            ResidualId motion_id = problem_->AddResidualBlock(
                cost_functors_ceres::GroundPlaneMotionRegularization::Create(),
                new ceres::ScaledLoss(new ceres::TrivialLoss(), 2. * weight, ceres::TAKE_OWNERSHIP),
                keyframes_.at(*it0)->pose_.data(),
                keyframes_.at(*it1)->pose_.data(),
                keyframes_.at(*it0)->local_ground_plane_.direction.data());
            ids_motion.push_back(motion_id);
            //        }
        }
        {
            std::cout << "Normal reg has inital cost=" << getCost(ids_normal, *problem_) << std::endl;
            std::cout << "Distance reg has inital cost=" << getCost(ids_dist, *problem_) << std::endl;
            std::cout << "Motion reg has inital cost=" << getCost(ids_motion, *problem_) << std::endl;
        }

        std::vector<ResidualId> global_normal_ids;
        for (const auto& el : active_keyframe_ids_) {
            ResidualId global_plane_dir_id = problem_->AddResidualBlock(
                cost_functors_ceres::VectorDifferenceRegularization2::Create(std::array<double, 3>{{0., 0., 1.}}),
                new ceres::ScaledLoss(new ceres::TrivialLoss(), weight, ceres::TAKE_OWNERSHIP),
                keyframes_.at(el)->local_ground_plane_.direction.data());
            global_normal_ids.push_back(global_plane_dir_id);
        }
    }
}

std::string BundleAdjusterKeyframes::adjustPoseOnly(Keyframe& kf) {
    // Reinitialiaize to get rid of old landmarks.
    ceres::Problem::Options options{};
    options.enable_fast_removal = true;
    problem_ = std::make_shared<ceres::Problem>(options);
    auto compensate_rotation = std::make_shared<bool>(false);

    // Get last selection -> faster.
    selected_landmark_ids_ = landmark_selector_->getLastSelection();

    // Add all residual block and parameters to ceres problem_.
    ResidualIdMap residual_block_ids_depth, residual_block_ids_repr;
    addKeyframeToProblem(kf, residual_block_ids_depth, residual_block_ids_repr, compensate_rotation);

    // Add motion constraint for this Keyframe.
    if (active_keyframe_ids_.size() > 2) {
        std::vector<Keyframe::Ptr> sorted_active_kf_ptrs = getSortedActiveKeyframePtrs();
        auto it0 = sorted_active_kf_ptrs.crbegin();
        auto it1 = std::next(it0);
        double rot_diff = calcQuaternionDiff((*it0)->pose_, (*it1)->pose_);

        // Linearly decrease weight until 0.03, then no weight at all.
        if (rot_diff < 0.03) {
            double weight = 1. * (1 - rot_diff / 0.03);
            double cur_ts = convert(kf.timestamp_);
            double cur_before = convert((*it0)->timestamp_);
            double cur_before2 = convert((*it1)->timestamp_);
            ceres::CostFunction* cost_func = cost_functors_ceres::SpeedRegularizationVector2::Create(
                cur_ts, cur_before, cur_before2, (*it0)->pose_, (*it1)->pose_);
            problem_->AddResidualBlock(cost_func,
                                       new ceres::ScaledLoss(new ceres::TrivialLoss(), weight, ceres::TAKE_OWNERSHIP),
                                       kf.pose_.data());
        }
    }

    // Set local parametrizations.
    setParameterization(kf, MotionParameterizationType::FullDOF);

    // Deactivate optimization parameters, f.e. fixed poses or landmarks.
    deactivatePoseParameters({Keyframe::FixationStatus::Pose});

    // Deactivate landmarks.
    deactivateLandmarks();

    std::vector<int> number_iterations{};
    if (selected_landmark_ids_.size() > 30) {
        for (int i = 0; i < int(outlier_rejection_options_.num_iterations); i++) {
            number_iterations.push_back(2);
        }
    }

    std::vector<std::pair<ResidualIdMap, robust_optimization::TrimmerSpecification>> input;
    input.push_back(
        std::make_pair(residual_block_ids_depth,
                       robust_optimization::TrimmerSpecification(robust_optimization::TrimmerType::Quantile,
                                                                 outlier_rejection_options_.depth_quantile)));
    input.push_back(
        std::make_pair(residual_block_ids_repr,
                       robust_optimization::TrimmerSpecification(robust_optimization::TrimmerType::Quantile,
                                                                 outlier_rejection_options_.reprojection_quantile)));

    robust_optimization::Options opt = robust_optimization::getStandardSolverOptions(solver_time_sec);
    opt.max_solver_time_refinement_in_seconds = solver_time_sec;

    opt.minimum_number_residual_groups = 30;
    opt.trust_region_relaxation_factor = -10.; // Reset trust region at each iteration.
    auto final_summary = robust_optimization::solveTrimmed(number_iterations, input, *problem_, opt);
    return final_summary.FullReport();
}

void BundleAdjusterKeyframes::addScaleRegularization(double weight) {
    if (active_keyframe_ids_.size() > 1) {
        auto it0 = active_keyframe_ids_.cbegin();
        auto it1 = std::next(it0);

        double current_scale =
            (keyframes_.at(*it1)->getEigenPose() * keyframes_.at(*it0)->getEigenPose().inverse()).translation().norm();

        ceres::CostFunction* cost_functor_reg_scale = cost_functors_ceres::PoseRegularization::Create(current_scale);
        problem_->AddResidualBlock(cost_functor_reg_scale,
                                   new ceres::ScaledLoss(new ceres::TrivialLoss(), weight, ceres::TAKE_OWNERSHIP),
                                   keyframes_.at(*it1)->pose_.data(),
                                   keyframes_.at(*it0)->pose_.data());
    }
}


void BundleAdjusterKeyframes::deactivateKeyframes(int min_num_connecting_landmarks,
                                                  int min_size_optimization_window,
                                                  int max_size_optimization_window) {
    // Find edges for all pushed keyframes.
    // Go through active frames and test with last pushed keyframe.
    // If does not belong to same landmark deactivate keyframe.

    // min_num_connecting_landmarks may not be bigger than the total number of landmarks
    //    min_num_connecting_landmarks =
    //        std::min(min_num_connecting_landmarks, static_cast<int>(active_landmark_ids_.size()));
    auto sorted_kf_ptrs = getSortedIdsWithActiveKeyframePtrs();
    auto newest_kf_ptr = sorted_kf_ptrs.back().second;

    // Go through all active keyframes, test connection between them by landmarks and save or erase
    // corresponding data.
    for (auto it = sorted_kf_ptrs.crbegin(); it != sorted_kf_ptrs.crend(); it++) {
        // Index of keyframe.
        int n = -std::distance(it, sorted_kf_ptrs.crbegin()); // reverted iterators have negative distance.
        std::cout << "n=" << n << std::endl;
        auto& cur_kf = *it->second;

        if (n > max_size_optimization_window - 1) {
            // Deactivate in any case.
            cur_kf.is_active_ = false;
        } else if (n < min_size_optimization_window - 1) {
            // Activate in any case.
            cur_kf.is_active_ = true;
        } else {
            // Get landmarks that connect the two frames.
            const auto& connecting_lm_ids = getCommonLandmarkIds(cur_kf, *newest_kf_ptr);

            std::cout << "Num connecting lms=" << connecting_lm_ids.size() << std::endl;

            // Ff there are landmarks that connect the keyframes mark as active otherwise inactive.
            cur_kf.is_active_ = static_cast<int>(connecting_lm_ids.size()) > min_num_connecting_landmarks;
        }

        // Erase id from memory if inactive.
        if (!cur_kf.is_active_) {
            active_keyframe_ids_.erase(it->first);
        }
    }

    // Deactivate landmarks so that selector can work only on landmarks that are important:
    // Go over active keyframes and add all landmark ids that have a measurement.
    std::set<LandmarkId> new_active_landmark_ids;
    for (const auto& kf_id : active_keyframe_ids_) {
        for (const auto& lm_id_meas : keyframes_.at(kf_id)->measurements_) {
            if (active_landmark_ids_.find(lm_id_meas.first) != active_landmark_ids_.cend()) {
                new_active_landmark_ids.insert(lm_id_meas.first);
            }
        }
    }
    active_landmark_ids_ = new_active_landmark_ids;

    // One non deactivated frame must be fixed otherwise problem is unstable.
    // Take oldest one since it has the best estimate.

    // Copy keyframes to vector for sorting.
    std::vector<std::pair<KeyframeId, Keyframe::Ptr>> kf_ptrs;
    kf_ptrs.resize(active_keyframe_ids_.size());
    std::transform(active_keyframe_ids_.cbegin(), active_keyframe_ids_.cend(), kf_ptrs.begin(), [this](const auto& id) {
        return std::make_pair(id, keyframes_.at(id));
    });

    // Partial sort to get kf_ids of the first two oldest timestamps.
    std::partial_sort(kf_ptrs.begin(), kf_ptrs.begin() + 2, kf_ptrs.end(), [](const auto& a, const auto& b) {
        return a.second->timestamp_ < b.second->timestamp_;
    });
    const auto& oldest_kf_id = kf_ptrs[0].first;
    const auto& second_oldest_kf_id = kf_ptrs[1].first;

    // Mark oldest for fixation.
    keyframes_.at(oldest_kf_id)->fixation_status_ = Keyframe::FixationStatus::Pose;

    // Mark second oldest for scale fixation.
    // This is done by punishing scale deviations with high weight rather than local parameterziation.
    // That can only reasonably be done if poses are expressed as relative motino from keyframe to keyframe.
    // If scale is observed by plane or landmark depth, this weight will be reduced.
    keyframes_.at(second_oldest_kf_id)->fixation_status_ = Keyframe::FixationStatus::Scale;
}

const Keyframe& BundleAdjusterKeyframes::getKeyframe(TimestampSec timestamp) const {
    // throw exception if no keyframes are available
    if (keyframes_.size() == 0) {
        throw NotEnoughKeyframesException(keyframes_.size(), 1);
    }

    if (timestamp < 0.) {
        // Get last keyframe.
        auto it = std::max_element(
            active_keyframe_ids_.cbegin(), active_keyframe_ids_.cend(), [&](const auto& a, const auto& b) {
                return keyframes_.at(a)->timestamp_ < keyframes_.at(b)->timestamp_;
            });
        return *keyframes_.at(*it);
    } else {
        // Get keyframe at point in time.
        TimestampNSec ts_nsec = convert(timestamp);

        // First search in active ids than in all of them.
        auto iter0 = std::find_if(active_keyframe_ids_.cbegin(), active_keyframe_ids_.cend(), [&](const auto& a) {
            return keyframes_.at(a)->timestamp_ == ts_nsec;
        });
        if (iter0 != active_keyframe_ids_.cend()) {
            return *keyframes_.at(*iter0);
        }

        auto iter = std::find_if(
            keyframes_.cbegin(), keyframes_.cend(), [&](const auto& a) { return a.second->timestamp_ == ts_nsec; });
        if (iter == keyframes_.cend()) {
            throw KeyframeNotFoundException(ts_nsec);
        }
        return *(iter->second);
    }
}

const char* BundleAdjusterKeyframes::NotEnoughKeyframesException::what() const noexcept {
    std::stringstream ss;
    ss << "Not enough keyframes available in bundle_adjuster_keyframes. Should be " << num_should_be << " is "
       << num_is;
    return ss.str().c_str();
}

const char* BundleAdjusterKeyframes::KeyframeNotFoundException::what() const noexcept {
    std::stringstream ss;
    ss << "keyframe corresponding to timestamp " << ts_ << " nano seconds not found";
    return ss.str().c_str();
}
}
