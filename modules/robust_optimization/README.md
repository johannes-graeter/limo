# ROBUST OPTIMIZATION
Wrapper library around ceres to implement trimmed least squares algorithm.

## Usage
* Find a working example in ./test/robust_optimization.cpp
* Given:
  * ceres problem (name: problem) as std::unique_ptr<ceres::Problem>
  * residual block ids (name: residual_block_ids) as robust_optimization::ResidualIds

```cpp
// Input for trimmed least squares.
int number_iterations = 3;  // Set number of iterations of trimmed least squares.
int number_steps = 2;

std::vector<int> number_iterations{};
for (int i = 0; i < number_iterations; i++) {
    number_iterations.push_back(number_steps);
}
std::vector<std::pair<ResidualIdMap, robust_optimization::TrimmerSpecification>> input;
input.push_back(
    std::make_pair(residual_block_ids,
                   robust_optimization::TrimmerSpecification(robust_optimization::TrimmerType::Quantile,
                                                             outlier_rejection_options_.depth_quantile)));
                                                             
robust_optimization::Options opt = robust_optimization::getStandardSolverOptions(solver_time_sec);
opt.max_solver_time_refinement_in_seconds = solver_time_sec;
opt.minimum_number_residual_groups = 30;
opt.trust_region_relaxation_factor = -10.; // Reset trust region at each iteration.
opt.num_threads = 3;
auto final_summary = robust_optimization::solveTrimmed(number_iterations, input, *problem, opt);
std::cout << final_summary.FullReport() << std::endl;
```
