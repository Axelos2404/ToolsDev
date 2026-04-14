#pragma once

#include <CoMISo/Utils/MatrixDecomposition.hh>

namespace COMISO {

/// Config for TruncatedNewtonPCG solver
struct TruncatedNewtonPCGConfig
{
  MatrixDecompositionAlgorithm matrix_decomposition_algo = MatrixDecompositionAlgorithm::Default;
  double eps           = 1e-3;
  /// Line search epsilon
  double eps_ls        = 1e-8;
  double eps_gdx       = 1e-3; // Note: 0.1 in alternate constructor
  int    max_iters     = 500;
  int    max_pcg_iters = 500;
  double pcg_tolerance = 1e-4;

  double alpha_ls      = 0.1;
  double beta_ls       = 0.8;

  // parameters of L1 Merit function line search
  bool   line_search_feasibility_step = true;
  double rho_merit     = 0.5;

  // max inf-norm constraint violation allowed for feasibility
  double eps_constraints_violation = 1e-6;
  // max inf-norm constraint violation above which feasibility steps are still performed
  double eps_constraints_violation_desirable = 1e-9;

  // minimal acceptable relative decrease of constraint violation (cv_old-cv_min)/c_old
  double min_acceptable_rel_constraint_violation_decrease = 1e-4;
  // minimal acceptable relative decrease of objective function (obj_old-opj_min)/obj_old (used to detec convergenc to infeasible point)
  double min_acceptable_rel_objective_decrease = 1e-4;

  // set limits for preconditioner values
//  double min_preconditioner_value = 1e-12;
//  double max_preconditioner_value = 1e12;
  double min_preconditioner_value = 1e-6;
  double max_preconditioner_value = 1e6;

  double max_feasible_step_safety_factor = 0.5;
  double max_infeasibility_step_safety_factor = 0.6;

  // adaptively choose tolerance of CG optimization?
  bool adaptive_tolerance = true;
  bool always_update_preconditioner = true;
  bool allow_warmstart = false;

  // parameters to adaptively skip hessian matrix updates:
  int    hessian_max_skips = 10; // maximum number of skipped hessian updates
  double hessian_min_acceptable_alpha = 0.1; // minimal acceptable line-search step length to skip hessian update
  double hessian_min_acceptable_rel_objective_decrease = 0.1; // minimal acceptable relative objective descrease to skip hessian update

  double adaptive_tolerance_modifier = 1.0;

  bool compute_dual_variables = false;

  // iterative refinement parameters
  bool enable_iterative_refinement = true;
  int  max_iterative_refinement_iters = 5;
  double iterative_refinement_cos_angle_threshold = 1e-12;
  // perform additional projection for Newton search direction (should not be necessary when iterative refinement is used)
  bool project_dx_before_update = false;
  bool silent = false;
};
} // namespace COMISO

