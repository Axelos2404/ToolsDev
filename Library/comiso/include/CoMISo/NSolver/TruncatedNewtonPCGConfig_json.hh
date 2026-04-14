#pragma once

#include <CoMISo/NSolver/TruncatedNewtonPCGConfig.hh>
#include <nlohmann/json.hpp>

namespace COMISO {
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TruncatedNewtonPCGConfig,
        matrix_decomposition_algo,
        eps,
        eps_ls,
        eps_gdx,
        max_iters,
        max_pcg_iters,
        pcg_tolerance,
        alpha_ls,
        beta_ls,
        line_search_feasibility_step,
        rho_merit,
        eps_constraints_violation,
        eps_constraints_violation_desirable,
        min_preconditioner_value,
        max_preconditioner_value,
        max_feasible_step_safety_factor,
        max_infeasibility_step_safety_factor,
        adaptive_tolerance,
        always_update_preconditioner,
        allow_warmstart,
        hessian_max_skips,
        hessian_min_acceptable_alpha,
        hessian_min_acceptable_rel_objective_decrease,
        adaptive_tolerance_modifier,
        compute_dual_variables,
        enable_iterative_refinement,
        max_iterative_refinement_iters,
        iterative_refinement_cos_angle_threshold,
        project_dx_before_update,
        silent
)

} // namespace COMISO
