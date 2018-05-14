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
#include <sstream>
#include <ceres/ceres.h>
#include "internal/cost_functors_ceres.hpp"
#include "internal/definitions.hpp"
#include "internal/local_parameterizations.hpp"

#include "landmark_selection_schemes.hpp"


namespace keyframe_bundle_adjustment {

namespace {
Eigen::Matrix<double, 3, 1> convertMeasurementToRay(const Eigen::Matrix3d& intrin_inv, const Measurement& m) {
    Eigen::Matrix<double, 3, 1> ray =
        intrin_inv * Eigen::Vector3d(static_cast<double>(m.u), static_cast<double>(m.v), 1.);
    ray.normalize();
    return ray;
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

BundleAdjusterKeyframes::BundleAdjusterKeyframes()
        : BundleAdjusterKeyframes(LandmarkSelectionSchemeRandom::createConst(std::numeric_limits<size_t>::max())) {
    // this selector is taking all landmarks available
}


BundleAdjusterKeyframes::BundleAdjusterKeyframes(LandmarkSelectionSchemeBase::ConstPtr landmark_selection_scheme)
        : solver_time_sec(0.2) {
    landmark_selector_ = std::make_unique<LandmarkSelector>();
    // first is always cheirality since it is fast and reliable
    landmark_selector_->addScheme(LandmarkSelectionSchemeCheirality::create());
    // add custom lm selection
    landmark_selector_->addScheme(landmark_selection_scheme);

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

void BundleAdjusterKeyframes::setParameterization() {
    for (const auto& id_kf : keyframes_) {
        auto& kf = *id_kf.second;
        if (problem_->HasParameterBlock(kf.pose_.data()) && problem_->GetParameterization(kf.pose_.data()) == NULL) {
            // add local parametrization to implement motion model if it pose was added as variable
            // and
            // parameterization was not set.
            // -> less parameters while being able to use convenient x,y,z representation

            // full 6 dofs
            ceres::LocalParameterization* motion_parameterization;
            // if keyframe is fixed no contraint is added -> return

            if (kf.fixation_status_ == Keyframe::FixationStatus::Scale) {
                double scale =
                    std::sqrt(kf.pose_[4] * kf.pose_[4] + kf.pose_[5] * kf.pose_[5] + kf.pose_[6] * kf.pose_[6]);
                // quaternion parameterization and const norm of translation
                // this is suboptimal (one DOF too much), but works
                motion_parameterization = new ceres::ProductParameterization(
                    new ceres::QuaternionParameterization(),
                    new ceres::AutoDiffLocalParameterization<local_parameterizations::FixScaleVectorPlus2, 3, 3>(
                        new local_parameterizations::FixScaleVectorPlus2(scale)));

                // quaternion parameterization and constant z to hold scale
                // make z fix, suboptimal since cos dependant (if components value is too small it
                // doesnt' have effect.)
                //                motion_parameterization = new ceres::ProductParameterization(
                //                    new ceres::QuaternionParameterization(),
                //                    new ceres::SubsetParameterization(3,{2}));

                //                motion_parameterization = new
                //                ceres::AutoDiffLocalParameterization<
                //                    local_parameterizations::FixScaleCircularMotionPlus,
                //                    7,
                //                    3>(new
                //                    local_parameterizations::FixScaleCircularMotionPlus(scale));
            } else {
                // quaternion parameterization
                motion_parameterization = new ceres::ProductParameterization(new ceres::QuaternionParameterization(),
                                                                             new ceres::IdentityParameterization(3));
                // this is the normal parameterization for circular movement
                //                motion_parameterization = new
                //                ceres::AutoDiffLocalParameterization<
                //                    local_parameterizations::CircularMotionPlus,
                //                    7,
                //                    4>();
            }

            problem_->SetParameterization(kf.pose_.data(), motion_parameterization);
        }
    }
}

void BundleAdjusterKeyframes::deactivateParameters(
    const std::vector<BundleAdjusterKeyframes::OptimizationFlags>& flags) {
    // set parameters constant if pose is fixed
    for (const auto& id_kf : keyframes_) {
        auto& kf = *id_kf.second;
        if (kf.fixation_status_ == Keyframe::FixationStatus::Pose && problem_->HasParameterBlock(kf.pose_.data())) {
            problem_->SetParameterBlockConstant(kf.pose_.data());
        }
    }

    // deactivate landmark parameters for motion only bundle adjustment
    for (const auto& f : flags) {
        if (f == OptimizationFlags::MotionOnly) {
            for (auto& lm : landmarks_) {
                if (!problem_->IsParameterBlockConstant(lm.second->pos.data())) {
                    problem_->SetParameterBlockConstant(lm.second->pos.data());
                }
            }
        }
    }
}

void BundleAdjusterKeyframes::push(const std::vector<Keyframe>& kfs) {
    for (const auto& kf : kfs) {
        push(kf);
    }
}


void BundleAdjusterKeyframes::push(const Keyframe& kf) {
    auto start_time_push = std::chrono::steady_clock::now();
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
        if (it == landmarks_.end()) {
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
        }

        // set triangulated landmark as active, may be deactivated
        active_landmark_ids_.insert(m.first);
    }
    std::cout << "Duration push="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_time_push)
                     .count()
              << " ms" << std::endl;
}


bool BundleAdjusterKeyframes::containsDepth(const Keyframe& kf, const LandmarkId lId) const {
    // Get all measuremets of the given landmark in the keyframe
    const auto& landmarkMeasurements = kf.measurements_.at(lId);

    for (const auto& cam : landmarkMeasurements) {
        // check if the measurement of minimum 1 camera contains a valid depth value
        if (cam.second.d >= 0)
            return true;
    }

    return false;
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
        const double z = m.second.d;
        const double x = (m.second.u - cam->principal_point.x()) * z / cam->focal_length;
        const double y = (m.second.v - cam->principal_point.y()) * z / cam->focal_length;

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
            bool is_lm_initialized = active_keyframe_ids_.find(track.id) != active_keyframe_ids_.cend();
            if (is_shrubbery && is_lm_initialized) {
                landmarks_.at(track.id)->weight = shrubbery_weight;
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
    const std::set<KeyframeId>& landmark_ids_) const {
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

std::vector<Keyframe::Ptr> BundleAdjusterKeyframes::getSortedActiveKeyframePtrs() const {
    // put keyframes into vector and sort them
    std::vector<Keyframe::Ptr> kf_ptrs;
    auto active_kf_ptrs = getActiveKeyframePtrs();
    kf_ptrs.reserve(active_kf_ptrs.size());
    for (const auto& kf : active_kf_ptrs) {
        // only use active keyframes
        kf_ptrs.push_back(kf.second);
    }

    // Keyframes have < operator defined
    std::sort(kf_ptrs.begin(), kf_ptrs.end(), [](const auto& a, const auto& b) { return *a < *b; });

    return kf_ptrs;
}


std::tuple<BundleAdjusterKeyframes::ResidualIdMap, BundleAdjusterKeyframes::ResidualIdMap> BundleAdjusterKeyframes::
    addBlocksToProblem() {

    // save residual block ids from depth
    // also used to determin if scale shall be fixed or not
    ResidualIdMap residual_block_ids_depth;
    ResidualIdMap residual_block_ids_repr;

    // add residuals for all landmarks in all keyframes
    for (auto& id_kf : active_keyframe_ids_) {
        // rever to instance
        auto& kf = *keyframes_.at(id_kf);
        //        // new problem so the residual ids are reset
        //        kf.resetResidualIds();

        // Add measruement constraints
        for (const auto& m : kf.measurements_) {
            // add to problem only if landmark is selected
            if (selected_landmark_ids_.find(m.first) != selected_landmark_ids_.cend()) {

                for (const auto& cam_id_meas : m.second) {
                    // pointer to current camera
                    const Camera::Ptr& cam = kf.cameras_.at(cam_id_meas.first);

                    // get extrinsics
                    Pose pose_camera_veh = cam->pose_camera_vehicle;

                    // add cost functor if has measured depth
                    if (cam_id_meas.second.d > 0.) {
                        // landmark depth is available as prior
                        // add cost functor with landmark, measurement and pose to estimation
                        // problem
                        // Each residual block takes a point and a camera as input and outputs a
                        // 1 dimensional residual.
                        // Internally, the cost function stores the measured depth of the
                        // landmark (distance between camera and landmark) and compares the
                        // measured depth with the current depth.
                        ceres::CostFunction* cost_functor_depth =
                            cost_functors_ceres::LandmarkDepthError::Create(cam_id_meas.second.d, pose_camera_veh);

                        // add residual block to problem for depth
                        ceres::ResidualBlockId res_id_depth = problem_->AddResidualBlock(
                            cost_functor_depth,
                            new ceres::ScaledLoss(new ceres::CauchyLoss(outlier_rejection_options_.depth_thres),
                                                  landmarks_.at(m.first)->weight,
                                                  ceres::TAKE_OWNERSHIP),
                            kf.pose_.data(),
                            landmarks_.at(m.first)->pos.data());
                        //                        // store residual id on keyframe for later
                        //                        deactivation of residuals
                        //                        residual_history_.assignResidualId(
                        //                            m.first, res_id_depth, cam_id_meas.first);
                        residual_block_ids_depth[res_id_depth] = std::make_pair(m.first, 1);
                    }

                    // landmark depth not available as prior; landmark has been triangulated
                    // add cost functor with landmark, measurement and pose to estimation
                    // problem
                    // Each Residual block takes a point and a camera as input and outputs a 2
                    // dimensional residual. Internally, the cost function stores the observed
                    // image location and compares the reprojection against the observation.
                    ceres::CostFunction* cost_functor_reprojection =
                        cost_functors_ceres::ReprojectionErrorWithQuaternions::Create(cam_id_meas.second.u,
                                                                                      cam_id_meas.second.v,
                                                                                      cam->focal_length,
                                                                                      cam->principal_point[0],
                                                                                      cam->principal_point[1],
                                                                                      pose_camera_veh);
                    // add residual block to problem poses are stored on keyframe as
                    // quaternion&translation landmarks are stored as Vector3d: Attention, Eigen
                    // stores matrices columnmajor!
                    // However here this is not a problem since we have only one column
                    ceres::ResidualBlockId res_id_repr = problem_->AddResidualBlock(
                        cost_functor_reprojection,
                        new ceres::ScaledLoss(new ceres::CauchyLoss(outlier_rejection_options_.reprojection_thres),
                                              landmarks_.at(m.first)->weight,
                                              ceres::TAKE_OWNERSHIP),
                        kf.pose_.data(),
                        landmarks_.at(m.first)->pos.data());

                    // store residual id on keyframe for later deactivation of residuals
                    residual_block_ids_repr[res_id_repr] = std::make_pair(m.first, 2);
                }
            }
        }
    }
    return std::make_tuple(residual_block_ids_depth, residual_block_ids_repr);
}

namespace {
std::map<LandmarkId, std::vector<Eigen::VectorXd>> calculateResiduals(
    const std::map<ResidualId, std::pair<LandmarkId, int>>& res_ids, bool apply_loss, ceres::Problem& problem) {

    // Put residual blocks in ceres format.
    ceres::Problem::EvaluateOptions eval_opt{};
    eval_opt.apply_loss_function = apply_loss;

    std::map<ResidualId, int> access_index;
    int ind = 0;
    for (const auto& res_id : res_ids) {
        const auto& res_size = res_id.second.second;

        // Put res id to stuff to evaluate.
        eval_opt.residual_blocks.push_back(res_id.first);

        // Add access index for later calling the residual vector.
        access_index[res_id.first] = ind;
        ind += res_size;
    }

    // Do evaluation.
    double cost = 0.0; // default
    std::vector<double> residuals;
    problem.Evaluate(eval_opt, &cost, &residuals, NULL, NULL);

    // Get residuals corresponding to landmark
    std::map<LandmarkId, std::vector<Eigen::VectorXd>> out;
    for (const auto& res_id_ind : access_index) {
        // Get residual size.
        const int& res_size = res_ids.at(res_id_ind.first).second;

        // Get residuals from vector and store them as Eigen::VectorXd.
        int base_ind = res_id_ind.second;
        Eigen::VectorXd res_vec(res_size);
        for (int i = 0; i < res_size; ++i) {
            res_vec[i] = residuals[static_cast<size_t>(base_ind + i)];
        }

        // Push back to output.s
        const auto& lm_id = res_ids.at(res_id_ind.first).first;
        out[lm_id].push_back(res_vec);
    }
    return out;
}

// double calcLoss(const Landmark::Ptr& lm, ceres::Problem& problem) {
//    //    auto residuals_loss = evaluateLandmark(lm, true, problem);
//    auto residuals_wo_loss = evaluateLandmark(lm, false, problem);

//    //    // Make difference and take max
//    //    std::vector<double> diff;
//    //    diff.resize(residuals_loss.size());
//    //    std::transform(residuals_loss.cbegin(),
//    //                   residuals_loss.cend(),
//    //                   residuals_wo_loss.cbegin(),
//    //                   diff.begin(),
//    //                   [](const auto& a, const auto& b) { return std::abs(a - b); });

//    //    auto it = std::max_element(diff.cbegin(), diff.cend(), [](const auto& a, const auto& b) { return a < b; });
//    auto it = std::max_element(
//        residuals_wo_loss.cbegin(), residuals_wo_loss.cend(), [](const auto& a, const auto& b) { return a < b; });
//    return *it;
//}

///@brief reject outliers by relative error, quantile=0.9 means the highest 10% error will be cut.
/// @return reject landmark ids
std::set<LandmarkId> rejectQuantile(double quantile, std::vector<std::pair<LandmarkId, double>>& loss) {
    int num = static_cast<int>(static_cast<double>(loss.size()) * quantile);
    // From http://en.cppreference.com/w/cpp/algorithm/nth_element
    // All of the elements before this new nth element are less than or equal to the elements after the new nth element.
    std::nth_element(
        loss.begin(), loss.begin() + num, loss.end(), [](const auto& a, const auto& b) { return a.second < b.second; });

    std::set<LandmarkId> out;
    for (auto it = loss.begin() + num; it != loss.end(); it++) {
        out.insert(it->first);
    }
    return out;
}

///@brief Merge summaries from solver.
std::string mergeSummaries(const std::vector<ceres::Solver::Summary>& summaries) {
    std::stringstream ss;
    ss << "Merged summaries of BundleAdjusterKeyframes:\n";
    for (size_t i = 0; i < summaries.size(); ++i) {
        ss << "--------------------------------------------------\nIteration No." << i << "\n";
        ss << summaries[i].FullReport();
    }
    return ss.str();
}

std::vector<std::pair<LandmarkId, double>> reduceResiduals(
    const std::map<LandmarkId, std::vector<Eigen::VectorXd>>& lm_res) {
    std::vector<std::pair<LandmarkId, double>> out;
    out.reserve(lm_res.size());

    for (const auto& el : lm_res) {
        auto it = std::max_element(
            el.second.cbegin(), el.second.cend(), [](const auto& a, const auto& b) { return a.norm() < b.norm(); });
        out.push_back(std::make_pair(el.first, it->norm()));
    }

    return out;
}
}

void BundleAdjusterKeyframes::removeLandmarksByLoss(double quantile, ResidualIdMap& res_ids) {
    // evaluate problem for each  active landmark and reject landmarks that have high downweight
    auto start_time_calc_loss = std::chrono::steady_clock::now();

    std::map<LandmarkId, std::vector<Eigen::VectorXd>> lm_residuals = calculateResiduals(res_ids, false, *problem_);

    std::cout << "Duration calc_res="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_time_calc_loss)
                     .count()
              << " ms" << std::endl;

    std::vector<std::pair<LandmarkId, double>> loss = reduceResiduals(lm_residuals);

    //    // dump residuals
    //    {
    //        std::stringstream ss;
    //        ss << "/tmp/res_" << std::chrono::system_clock::now().time_since_epoch().count() << ".txt";
    //        std::ofstream file(ss.str().c_str());
    //        for (const auto& el : loss) {
    //            file << el.second << "\n";
    //        }
    //        file.close();
    //    }

    std::cout << "Duration calc_res and reduce="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_time_calc_loss)
                     .count()
              << " ms" << std::endl;


    // reject lms by loss
    std::set<LandmarkId> rejected_lms = rejectQuantile(quantile, loss);

    auto start_time_remove_params = std::chrono::steady_clock::now();
    // Remove landmark
    for (const auto& lm_id : rejected_lms) {
        // Remove residual blockss corresponding to landmark from optimization problem.
        // We do not use RemoveParameterBlock since that would remove all residuals so also the ones for reprojection
        // error or depth error.
        auto it = res_ids.begin();
        for (; it != res_ids.end();) {
            if (it->second.first == lm_id) {
                problem_->RemoveResidualBlock(it->first);
                // Erase val from id map for next turn.
                it = res_ids.erase(it);
            } else {
                it++;
            }
        }
        //        problem_->RemoveParameterBlock(landmarks_.at(id)->pos.data());
        // Remove landmarks from selection id
        //        selected_landmark_ids_.erase(selected_landmark_ids_.find(lm_id));
    }
    std::cout << "Duration remove_params="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_time_remove_params)
                     .count()
              << " ms" << std::endl;
}


std::string BundleAdjusterKeyframes::solve() {
    if (keyframes_.size() < 3) {
        throw NotEnoughKeyframesException(keyframes_.size(), 3);
    }

    // Add several summaries. For wach outlier rejection iteration one.
    std::vector<ceres::Solver::Summary> summaries;

    // Reinitialiaize to get rid of old landmarks.
    ceres::Problem::Options options{};
    options.enable_fast_removal = true;
    problem_ = std::make_shared<ceres::Problem>(options);

    auto start_time_lm_sel = std::chrono::steady_clock::now();
    // Process landmark selector that was assigned in tool on landmarks that are not deactivated.
    auto start_get_active = std::chrono::steady_clock::now();
    auto active_landmarks = getActiveLandmarkConstPtrs();
    auto active_keyframes = getActiveKeyframeConstPtrs();
    std::cout << "Duration solver:lm_selection:get_active="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_get_active)
                     .count()
              << " ms" << std::endl;

    selected_landmark_ids_ = landmark_selector_->select(active_landmarks, active_keyframes);
    std::cout << "duration solver:lms_selection="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_time_lm_sel)
                     .count()
              << " ms" << std::endl;

    // Add all residual block and parameters to ceres problem_.
    ResidualIdMap residual_block_ids_depth, residual_block_ids_repr;
    std::tie(residual_block_ids_depth, residual_block_ids_repr) = addBlocksToProblem();


    // If scale was observed, don't fix scale but add regularization.
    if (residual_block_ids_depth.size() > 3) {
        for (const auto& act_kf : getActiveKeyframePtrs()) {
            if (act_kf.second->fixation_status_ == Keyframe::FixationStatus::Scale) {
                act_kf.second->fixation_status_ = Keyframe::FixationStatus::None;
            }
        }

        // Add regularization of first pose diff for scale with weight in function of number of observed
        // depth measruements.
        double regularization_loss_weight = 10. / static_cast<double>(residual_block_ids_depth.size());
        std::cout << "KeyframeBundleAdjuster: regularization weight=" << regularization_loss_weight << std::endl;
        addScaleRegularization(regularization_loss_weight);
    }

    // Set local parametrizations.
    setParameterization();
    // Deactivate optimization parameters, f.e. fixed poses or landmarks.
    deactivateParameters();

    auto start_time_outlier = std::chrono::steady_clock::now();
    // Alternate optimization and outlier rejection by diff between residuals
    // and downweighted residuals. For now do it 2 times.
    for (int i = 0; i < outlier_rejection_options_.num_iterations; ++i) {
        // Only do rejection if number of measeruements is sufficient.
        if (selected_landmark_ids_.size() < 30) {
            break;
        }

        // At ceres sometimes diverges, if so calculate with more iterations.
        ceres::Solver::Summary cur_summary;
        int num_outlier_iter = 2;
        ceres::Solve(getSolverOptionsNumIter(num_outlier_iter), problem_.get(), &cur_summary);
        double cost_change = cur_summary.initial_cost - cur_summary.final_cost;

        if (cost_change <= 0.) {
            ceres::Solve(getSolverOptionsNumIter(3 * num_outlier_iter), problem_.get(), &cur_summary);
            cost_change = cur_summary.initial_cost - cur_summary.final_cost;
            if (cost_change <= 0.) {
                std::cout << "--------------------------------------------------" << std::endl;
                std::cout << "Outlier rejection is instable, problem didnt' reduce error after " << 3 * num_outlier_iter
                          << " iterations" << std::endl;
            }
        }
        // Remember summary for later.
        summaries.push_back(cur_summary);

        auto start_time_quant = std::chrono::steady_clock::now();
        // Reject landmarks by quantile.
        removeLandmarksByLoss(outlier_rejection_options_.depth_quantile, residual_block_ids_depth);
        removeLandmarksByLoss(outlier_rejection_options_.reprojection_quantile, residual_block_ids_repr);
        std::cout << "Duration quant="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                           start_time_quant)
                         .count()
                  << " ms" << std::endl;
        auto start_time_clean = std::chrono::steady_clock::now();
        std::cout << "Duration clean up parameter blocks="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                           start_time_clean)
                         .count()
                  << " ms" << std::endl;
        // Clean up unused parameter blocks.
        std::vector<double*> param_blocks;
        problem_->GetParameterBlocks(&param_blocks);
        for (const auto& pb : param_blocks) {
            std::vector<ceres::ResidualBlockId> residual_blocks;
            problem_->GetResidualBlocksForParameterBlock(pb, &residual_blocks);
            if (residual_blocks.size() == 0) {
                problem_->RemoveParameterBlock(pb);
            }
        }
    }
    std::cout << "Duration outlier rejection="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_time_outlier)
                     .count()
              << " ms" << std::endl;

    // Solve problem.
    ceres::Solver::Summary summary;
    ceres::Solve(getSolverOptions(solver_time_sec), problem_.get(), &summary);
    summaries.push_back(summary);

    return mergeSummaries(summaries);
}

void BundleAdjusterKeyframes::addScaleRegularization(double weight) {
    if (active_keyframe_ids_.size() > 1) {
        auto it1 = active_keyframe_ids_.cbegin();
        std::advance(it1, 1);
        auto it0 = active_keyframe_ids_.cbegin();

        double current_scale =
            (keyframes_.at(*it1)->getEigenPose().translation() - keyframes_.at(*it0)->getEigenPose().translation())
                .norm();

        ceres::CostFunction* cost_functor_reg_scale =
            cost_functors_ceres::PoseRegularizationPose::Create(current_scale);
        problem_->AddResidualBlock(cost_functor_reg_scale,
                                   new ceres::ScaledLoss(new ceres::TrivialLoss(), weight, ceres::TAKE_OWNERSHIP),
                                   keyframes_.at(*it1)->pose_.data(),
                                   keyframes_.at(*it0)->pose_.data());
    }
}


void BundleAdjusterKeyframes::deactivateKeyframes(int min_num_connecting_landmarks,
                                                  double min_time_sec,
                                                  double max_time_sec) {
    // that number may not be bigger than the total number of landmarks
    //    min_num_connecting_landmarks =
    //        std::min(min_num_connecting_landmarks, static_cast<int>(active_landmark_ids_.size()));
    // find edges for all pushed keyframes
    // go through active frames and test with last pushed keyframe
    // if not belongs to same landmark deactivate keyframe
    const auto& newest_kf = getKeyframe();

    // go through all active keyframes, test connection between them by landmarks and save or erase
    // corresponding data
    for (auto it = active_keyframe_ids_.cbegin(); it != active_keyframe_ids_.cend();
         /*no incrementation*/) {
        const auto& kf_id = *it;
        auto& cur_kf = *keyframes_.at(kf_id);

        double dt_sec = std::abs(convert(newest_kf.timestamp_) - convert(cur_kf.timestamp_));
        if (dt_sec > min_time_sec) {
            // only test connectivity if bigger than minimum optimization window
            // get landmarks that connect the two frames.
            const auto& connecting_lm_ids = getCommonLandmarkIds(cur_kf, newest_kf);

            // if there are landmarks that connect the keyframes mark as active otherwise inactive
            cur_kf.is_active_ = static_cast<int>(connecting_lm_ids.size()) > min_num_connecting_landmarks;
        } else if (dt_sec > max_time_sec) {
            // deactivate in any case
            cur_kf.is_active_ = false;
        }


        // erase id from memory if inactive
        if (cur_kf.is_active_) {
            ++it;
        } else {
            // erase keyframe and advance iterator
            it = active_keyframe_ids_.erase(it); // since c+11 erase returns next element
        }
    }

    // deactivate landmarks so that selector can work only on landmarks that are important
    // go over active keyframes and add all landmark ids that have a measurement
    active_landmark_ids_.clear();
    for (const auto& kf_id : active_keyframe_ids_) {
        for (const auto& lm_id_meas : keyframes_.at(kf_id)->measurements_) {
            active_landmark_ids_.insert(lm_id_meas.first);
        }
    }

    // one non deactivated frame must be fixed otherwise problem is unstable
    // take oldest one since it has the best estimate

    // copy keyframes to vector for sorting
    std::vector<std::pair<KeyframeId, Keyframe::Ptr>> kf_ptrs;
    kf_ptrs.resize(active_keyframe_ids_.size());
    std::transform(active_keyframe_ids_.cbegin(), active_keyframe_ids_.cend(), kf_ptrs.begin(), [this](const auto& id) {
        return std::make_pair(id, keyframes_.at(id));
    });

    // partial sort to get kf_ids of the first two oldest timestamps
    std::partial_sort(kf_ptrs.begin(), kf_ptrs.begin() + 2, kf_ptrs.end(), [](const auto& a, const auto& b) {
        return a.second->timestamp_ < b.second->timestamp_;
    });
    const auto& oldest_kf_id = kf_ptrs[0].first;
    const auto& second_oldest_kf_id = kf_ptrs[1].first;

    // fix oldest
    keyframes_.at(oldest_kf_id)->fixation_status_ = Keyframe::FixationStatus::Pose;

    // fix scale
    // if we have a depth measurement this fixation will be deactivated
    keyframes_.at(second_oldest_kf_id)->fixation_status_ = Keyframe::FixationStatus::Scale;
}

const Keyframe& BundleAdjusterKeyframes::getKeyframe(TimestampSec timestamp) const {
    // throw exception if no keyframes are available
    if (keyframes_.size() == 0) {
        throw NotEnoughKeyframesException(keyframes_.size(), 1);
    }

    if (timestamp < 0.) {
        // get last keyframe
        auto it = std::max_element(keyframes_.cbegin(), keyframes_.cend(), [](const auto& a, const auto& b) {
            return a.second->timestamp_ < b.second->timestamp_;
        });
        return *(it->second);
    } else {
        // get keyframe at point in time
        TimestampNSec ts_nsec = convert(timestamp);
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
