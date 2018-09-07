// Copyright 2018. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <chrono>
#include <functional>
#include <ceres/ceres.h>
#include "internal/definitions.hpp"

namespace robust_optimization {

using ResidualId = ceres::ResidualBlockId;
using ResidualBlockSize = int;
using ResidualIds = std::vector<std::pair<ResidualId, ResidualBlockSize>>;
using ResidualGroupId = unsigned long; ///< Ids of element to w

///@brief Map to identify residual ids to residual group ids, makes sense for problems where one outlier residual shall
/// lead to rejection of all measruements correponding to that(f.e. each landmarks in BA has one group id)
using ResidualIdMap = std::map<ResidualId, std::pair<ResidualGroupId, ResidualBlockSize>>;

///@brief Specifications for trimmer.
/// @param type of the trimming algorithm
/// @param parameter for algo (up to now only 1)
struct TrimmerSpecification {
    TrimmerSpecification() = default;
    TrimmerSpecification(TrimmerType type, double parameter) : type(type), parameter(parameter) {
        ;
    }
    TrimmerType type;
    double parameter;
};

/**
 * @brief The Summary struct
 * Stores several ceres summaries from solveTrimmed.
 * Imitates interfaces from ceres summaries (thats why we have Google method annotation.)
 */
struct Summary {
    Summary(const std::vector<ceres::Solver::Summary>& summaries) {
        Set(summaries);
    }
    std::vector<ceres::Solver::Summary> all_summaries;
    std::string merged_full_summaries;
    double initial_cost;
    double final_cost;
    double time_sec;

    std::string FullReport() const {
        std::stringstream ss;
        ss << merged_full_summaries << std::endl;
        ss << "Duration solveTrimmed=" << time_sec << " sec" << std::endl;
        return ss.str();
    }

    ///@brief Merge summaries from solver.
    void Set(const std::vector<ceres::Solver::Summary>& summaries) {
        std::stringstream ss;
        ss << "Merged summaries:\n";
        for (int i = 0; i < int(summaries.size()); ++i) {
            ss << "--------------------------------------------------\nIteration No." << i << "\n";
            ss << summaries[i].FullReport();
        }
        all_summaries = summaries;
        merged_full_summaries = ss.str();
        initial_cost = summaries.front().initial_cost;
        final_cost = summaries.back().final_cost;
    }
};

struct Options : public ceres::Solver::Options {
    Options() = default;

    ///@brief Maximum solver time for last optimization step.
    double max_solver_time_refinement_in_seconds{this->max_solver_time_in_seconds * 4.};

    ///@brief Relaxation factor increases size of last radius a bit (we remove outliers -> bigger steps should be
    /// possible.). Negative value means it is reset to defualt every iteration.
    double trust_region_relaxation_factor{3.};

    size_t minimum_number_residual_groups{30};

    std::function<void(void)> pre_action{[]() { return; }};
    std::function<void(void)> post_action{[]() { return; }};
};

///@brief Get typical options for ceres problem, if solver time is the issue.
Options getStandardSolverOptions(double solver_time_sec) {
    Options options;
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

///@brief Get typical options for ceres problem fix number of iterations.
Options getSolverOptionsNumIter(int num_iterations) {
    Options options = getStandardSolverOptions(100.);
    options.max_num_iterations = num_iterations;
    return options;
}


/**
*  @brief solve_trimmed; Solver, that achieves robustness by trimming.
*  Trimming means that residuals that are outside of a threshold are
*  rejected. That threshold is determined by the Trimmer instance, with
*  which different trimming techniques can be realized.
*
*  @param number_iterations; defines number of iterations of the trimmer(.size) and number of iterations of the
* solver.
* @param ids_trimmer_specs; pairs of data to trim. First is map from residual ids to group ids and residual size.
*                           ResidualGrouId is used for problems where it makes sense to reject the all residuals
*                           corresponding to one group of residuals, if one of them is an outlier (as for landmarks
* in
*                           bundle adjustment). Second is specifications of the trimming algorithm to use (type and
*                           parameter).
* @param problem; ceres problem for which residuals and parameters shall be reduced.
*
*/
Summary solveTrimmed(const std::vector<int>& number_iterations,
                     std::vector<std::pair<ResidualIdMap, TrimmerSpecification>>& ids,
                     ceres::Problem& problem,
                     Options options = getStandardSolverOptions(0.1));

///@brief Overload for case with only residual ids of one kind.
Summary solveTrimmed(const std::vector<int>& number_iterations,
                     const ResidualIds& ids,
                     const TrimmerSpecification& trimmer_specs,
                     ceres::Problem& problem,
                     Options options = getStandardSolverOptions(0.1)) {
    // If any residual shall be treated individually, we give each residual id an independant group id.
    ResidualIdMap group_ids;
    unsigned int i = 0;
    for (const auto& el : ids) {
        group_ids[el.first] = std::make_pair(i, el.second);
        i++;
    }

    std::vector<std::pair<ResidualIdMap, TrimmerSpecification>> ids_map{std::make_pair(group_ids, trimmer_specs)};
    return solveTrimmed(number_iterations, ids_map, problem, options);
}
}
