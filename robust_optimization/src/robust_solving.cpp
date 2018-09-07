// Copyright 2018. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include <robust_solving.hpp>
#include <internal/apply_trimmer.hpp>

namespace robust_optimization {
namespace {

std::map<ResidualGroupId, std::vector<Eigen::VectorXd>> calculateResiduals(const ResidualIdMap& res_ids,
                                                                           bool apply_loss,
                                                                           ceres::Problem& problem) {
    if (res_ids.size() == 0) {
        return std::map<ResidualGroupId, std::vector<Eigen::VectorXd>>();
    }

    // Put residual blocks in ceres format.
    ceres::Problem::EvaluateOptions eval_opt{};
    eval_opt.apply_loss_function = apply_loss;
    eval_opt.num_threads = 2;

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
    std::map<ResidualGroupId, std::vector<Eigen::VectorXd>> out;
    for (const auto& res_id_ind : access_index) {
        // Get residual size.
        const auto& res_size = res_ids.at(res_id_ind.first).second;

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

std::map<ResidualGroupId, std::vector<double>> reduceResidualsNorm(
    const std::map<ResidualGroupId, std::vector<Eigen::VectorXd>>& lm_res) {
    std::map<ResidualGroupId, std::vector<double>> out;

    for (const auto& el : lm_res) {
        std::vector<double> res_vec;
        res_vec.reserve(el.second.size());
        for (const auto& lm : el.second) {
            res_vec.push_back(lm.norm());
        }
        out[el.first] = res_vec;
    }
    return out;
}

std::map<ResidualGroupId, double> getMaximumResidual(const std::map<ResidualGroupId, std::vector<double>>& lm_res) {
    std::map<ResidualGroupId, double> out;

    for (const auto& el : lm_res) {
        auto it =
            std::max_element(el.second.cbegin(), el.second.cend(), [](const auto& a, const auto& b) { return a < b; });
        out[el.first] = *it;
    }
    return out;
}


/**
 * @brief removeResidualsByLoss, remove residuals with the biggest loss
 * @param res_ids, ids with residual block size
 * @param trimmer_specs, trimmer specs define type and paramter of the trimmer algorithm that shall be used
 * @param problem, ceres problem to remove params from
 */
void getResidualsToRemove(TrimmerSpecification trimmer_specs,
                          size_t minimum_number_groups,
                          ceres::Problem& problem,
                          const ResidualIdMap& res_ids,
                          std::set<ResidualGroupId>& residuals_to_remove) {
    // Evaluate problem for each  active landmark and reject landmarks that have high downweight.
    std::map<ResidualGroupId, std::vector<Eigen::VectorXd>> residuals = calculateResiduals(res_ids, false, problem);

    // Don't remove if number of groups is too small.
    if (residuals.size() < minimum_number_groups) {
        return;
    }

    // To compare we need a scalar residual.
    auto reduced_residuals_vec = reduceResidualsNorm(residuals);
    auto reduced_residuals = getMaximumResidual(reduced_residuals_vec);

    // Reject lms by residual.
    std::vector<ResidualGroupId> rejected_res =
        getOutliers(reduced_residuals, trimmer_specs.type, trimmer_specs.parameter);

    // Add to output.
    for (const auto& el : rejected_res) {
        residuals_to_remove.insert(el);
    }
}

void removeUnconstraintParameters(ceres::Problem& problem) {
    std::vector<double*> param_blocks;
    problem.GetParameterBlocks(&param_blocks);
    for (const auto& pb : param_blocks) {
        std::vector<ceres::ResidualBlockId> residual_blocks;
        problem.GetResidualBlocksForParameterBlock(pb, &residual_blocks);
        if (residual_blocks.size() == 0) {
            problem.RemoveParameterBlock(pb);
        }
    }
}
}

Summary solveTrimmed(const std::vector<int>& number_iterations,
                     std::vector<std::pair<ResidualIdMap, TrimmerSpecification>>& ids_trimmer_specs,
                     ceres::Problem& problem,
                     Options options) {
    auto start_time = std::chrono::steady_clock::now();

    // Add several summaries. For watch outlier rejection iterations.
    std::vector<ceres::Solver::Summary> summaries;

    // Save number of iterations for later.
    int number_iterations_final = options.max_num_iterations;

    // Remember the last trust_region_radius after each step and use it for new iterations.
    double last_trust_region_radius =
        options.initial_trust_region_radius / options.trust_region_relaxation_factor; // Default ceres value

    // Alternate optimization and outlier rejection by diff between residuals
    // and downweighted residual.
    for (const auto& num_outlier_iter : number_iterations) {
        // Set number of iterations.
        options.max_num_iterations = num_outlier_iter;
        // Set trust_region_radius
        if (options.trust_region_relaxation_factor > 0.) {
            options.initial_trust_region_radius = last_trust_region_radius * options.trust_region_relaxation_factor;
            options.max_trust_region_radius = std::pow(options.initial_trust_region_radius, 4);
        }

        // At ceres sometimes diverges, if so calcualte with more iterations.
        ceres::Solver::Summary cur_summary;
        ceres::Solve(options, &problem, &cur_summary);
        double cost_change = cur_summary.initial_cost - cur_summary.final_cost;

        if (cost_change <= 0.) {
            options.max_num_iterations = 3 * num_outlier_iter;
            ceres::Solve(options, &problem, &cur_summary);
            cost_change = cur_summary.initial_cost - cur_summary.final_cost;
            if (cost_change <= 0.) {
                std::cout << "--------------------------------------------------" << std::endl;
                std::cout << "Outlier rejection is instable, problem didnt' reduce error after " << 3 * num_outlier_iter
                          << " iterations" << std::endl;
            }
        }

        // Remember summary for later.
        summaries.push_back(cur_summary);

        // Execute defined pre action.
        options.pre_action();

        // Reject landmarks by quantile.
        std::set<ResidualGroupId> residual_groups_to_remove;
        for (auto& el : ids_trimmer_specs) {
            getResidualsToRemove(
                el.second, options.minimum_number_residual_groups, problem, el.first, residual_groups_to_remove);
        }

        // Remove residual groups in all input data from problem.
        // This can lead to more data removed than intended, but is better than "dangling" parameters, such as landmarks
        // without pose constraints.
        for (auto& ids_spec : ids_trimmer_specs) {
            auto& map_ids = ids_spec.first;

            // For each id, timmer pair, remove residual if group id should be removed.
            auto it = map_ids.begin();
            for (; it != map_ids.end();) {
                const auto& residual_id = it->first;
                const auto& residual_group_id = it->second.first;
                if (residual_groups_to_remove.find(residual_group_id) != residual_groups_to_remove.cend()) {
                    problem.RemoveResidualBlock(residual_id);
                    it = map_ids.erase(it);
                } else {
                    it = std::next(it);
                }
            }
        }

        // Clean up unused parameter blocks.
        removeUnconstraintParameters(problem);

        // Save trust region for last sucessfull iteration.
        // Next iteration will start there (and will possibly expand it).
        for (auto it = cur_summary.iterations.crbegin(); it != cur_summary.iterations.crend(); it++) {
            if (it->step_is_successful && it->step_is_valid) {
                last_trust_region_radius = it->trust_region_radius;
                std::cout << "Robust solving tieration number: " << it->iteration << std::endl;
                break;
            }
        }

        // Execute defined post action.
        options.post_action();
    }

    // Set original number of iterations.
    options.max_num_iterations = number_iterations_final;
    options.max_solver_time_in_seconds = options.max_solver_time_refinement_in_seconds;

    // Solve problem.
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    summaries.push_back(summary);
    Summary trimmer_summary(summaries);
    trimmer_summary.time_sec = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                       std::chrono::steady_clock::now() - start_time)
                                                       .count()) *
                               1e-3;

    return trimmer_summary;
}
}
