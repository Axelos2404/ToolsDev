#pragma once
/*===========================================================================*\
 *                                                                           *
 *                            TruncatedNewtonPCG                             *
 *      Copyright (C) 2024 by Computer Graphics Group, University of Bern    *
 *                           http://cgg.unibe.ch                             *
 *                                                                           *
 *      Author: David Bommes                                                 *
 *                                                                           *
\*===========================================================================*/




//== COMPILE-TIME PACKAGE REQUIREMENTS ========================================
#include <CoMISo/Config/config.hh>

//== INCLUDES =================================================================

#include <CoMISo/Config/CoMISoDefines.hh>
#include <CoMISo/NSolver/NProblemInterface.hh>
#include <CoMISo/NSolver/NConstraintInterface.hh>
#include <CoMISo/NSolver/LinearConstraint.hh>
#include <CoMISo/NSolver/TruncatedNewtonPCGConfig.hh>

#include <Eigen/Dense>
#include <Eigen/Sparse>

//== NAMESPACES ===============================================================

namespace COMISO {

//== CLASS DEFINITION =========================================================

class COMISODLLEXPORT TruncatedNewtonPCG
{
public:

  typedef Eigen::VectorXd             VectorD;
  typedef Eigen::SparseMatrix<double> SMatrixD;
  typedef Eigen::Triplet<double>      Triplet;

  struct OptimizerStatus
  {
    // check whether optimizer converged to a locally optimal point
    bool converged_to_local_optimum() const
    {
      return (     feasible
                && !negative_curvature_step
//                && line_search_t == 1.0
                && (newton_decrement_within_tolerance || projected_gradient_norm_within_tolerance) );
    }

    // check whether optimizer converged to a locally infeasible point
    bool converged_to_infeasible_point() const
    {
      return (!feasible && !feasibility_step_productive && !optimization_step_productive);
    }

    // objective function value at last iterate
    double objective_function = DBL_MAX;
    // relative decrease of objective function in last iteration ( (objective_old-objective_new)/objective_old)
    double rel_objective_function_decrease = DBL_MAX;
    // did optimization step of last iterate improve objective sufficiently?
    bool optimization_step_productive = false;

    // was last iterate feasible w.r.t. eps_constraints_violation_, i.e. ||residual||_max <= eps_constraints_violation_
    bool feasible = false;
    // constraint violation ||residual||_max of last iterate
    double constraint_violation_inf_norm = DBL_MAX;
    // relative decrease of constraint violation in last iteration ( (constraint_violation_inf_norm_old-constraint_violation_inf_norm_new)/constraint_violation_inf_norm_old)
    double rel_constraint_violation_inf_norm_decrease = DBL_MAX;
    // did feasibility step of last iterate improve feasbility sufficiently?
    bool feasibility_step_productive = false;

    // was Newton decrement of last iterate within specified tolerance eps_gdx_?
    bool   newton_decrement_within_tolerance = false;
    // Newton decrement of last iterate
    double newton_decrement = DBL_MAX;

    // was projected gradient norm of last iterate within specified tolerance eps_?
    bool projected_gradient_norm_within_tolerance = false;
    // projected gradient norm of last iterate
    double projected_gradient_norm = DBL_MAX;

    // did last iterate perform a step into negative curvature direction?
    bool negative_curvature_step = false;

    // line search parameter of last step in [0,1]
    double line_search_t = 0.0;
    double line_search_t_max_feasible = 0.0;
    int    line_search_iterations = 0;

    // line search parameter of last step in [0,1]
    double line_search_t_inf = 0.0;
    double line_search_t_inf_max_feasible = 0.0;
    int    line_search_inf_iterations = 0;

    // number of refinement iters in current Newton iter
    int refinement_iters = 0;
    // total number of iterative refinement iterations
    int refinement_iters_total = 0;

    // ConjugateGradient data
    bool cg_converged = false;
    int  cg_iterations = 0;
    int  cg_iterations_total = 0;

    // Hessian updated in last iteration?
    bool hessian_updated = false;

    // total number of performed Newton iterations
    int n_newton_iters = 0;
    // total number of negative curvature iterations
    int n_negative_curvature_iters = 0;
  };


  using Config = TruncatedNewtonPCGConfig;

  explicit TruncatedNewtonPCG(Config const& _config = {})
      : config_(_config)
  {}

  /// deprecated: old constructor provided for backwards compatibility
  [[deprecated("Use TruncatedNewtonPCG(TruncatedNewtonPCGConfig const&) instead")]]
  TruncatedNewtonPCG(const double _eps, const double _eps_line_search = 1e-8,
                     const int _max_iters = 500, const double _alpha_ls = 0.2,
                     const double _beta_ls = 0.6)
  {
      config_.eps = _eps;
      config_.eps_ls = _eps_line_search;
      config_.max_iters = _max_iters;
      config_.alpha_ls = _alpha_ls;
      config_.beta_ls = _beta_ls;
      // old defaults:
      config_.max_pcg_iters = 500;
      config_.max_feasible_step_safety_factor = 0.5;
      config_.eps_gdx = 0.1; // !! Different from TruncatedNewtonPCGConfig default
      config_.adaptive_tolerance_modifier = 1.0;
      config_.allow_warmstart = false;
      config_.always_update_preconditioner = true;
      config_.adaptive_tolerance = true;
  }

  // optimize unconstrained problem
  int solve(NProblemInterface* _problem);

  // optimize with linear constraints
  // Note: It is required that the constraints are linearly independent
  int solve( NProblemInterface* _problem, const SMatrixD& _A, const VectorD& _b );
  int solve( NProblemInterface* _problem, std::vector<LinearConstraint>& _constraints);
  int solve( NProblemInterface* _problem, std::vector<NConstraintInterface*>& _constraints);
  // deprecated naming: identical to above but kept for backward compatibility
  int solve_projected_normal_equation( NProblemInterface* _problem, const SMatrixD& _A, const VectorD& _b );
  int solve_projected_normal_equation(NProblemInterface* _problem, std::vector<LinearConstraint>& _constraints);
  int solve_projected_normal_equation(NProblemInterface* _problem, std::vector<NConstraintInterface*>& _constraints);

//  // solve with linear constraints
//  // Warning: so far only feasible starting points with (_A*_problem->initial_x() == b) are supported!
//  // It is also required that the constraints are linearly independent
//  int solve_experimental(NProblemInterface* _problem, const SMatrixD& _A, const VectorD& _b)
//  int solve_reduced_system( NProblemInterface* _problem, const SMatrixD& _A, const VectorD& _b )
//  int solve_reduced_system_EigenCG( NProblemInterface* _problem, const SMatrixD& _A, const VectorD& _b )
//  int solve_experimental(NProblemInterface* _problem, std::vector<LinearConstraint>& _constraints)
//  int solve_reduced_system(NProblemInterface* _problem, std::vector<LinearConstraint>& _constraints)
//  int solve_reduced_system_EigenCG(NProblemInterface* _problem, std::vector<LinearConstraint>& _constraints)


  // obtain deatiled information of optimization status
  OptimizerStatus& status() {return status_;}

  bool converged() { return status_.converged_to_local_optimum(); }
  bool feasible_solution_found() { return status_.feasible; }

  TruncatedNewtonPCGConfig const& config() const {return config_;}
  TruncatedNewtonPCGConfig & config() {return config_;}

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  void set_silent(const bool _silent) { config_.silent = _silent;}

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  void set_eps_constraints_violation(double eps){ config_.eps_constraints_violation = eps; }

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  void set_eps_constraints_violation_desirable(double eps){ config_.eps_constraints_violation_desirable = eps; }

  double reduced_gradient_norm() { return status_.projected_gradient_norm;};

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  bool& always_update_preconditioner() { return config_.always_update_preconditioner;};

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  double& adaptive_tolerance_modifier() { return config_.adaptive_tolerance_modifier;};

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  double& tolerance_newton_decrement() { return config_.eps_gdx;}
  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  double& tolerance_gdx() { return config_.eps_gdx;}

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  double& tolerance_reduced_gradient_norm() { return config_.eps;}

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  int& max_iters()     { return config_.max_iters;}
  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  int& max_pcg_iters() { return config_.max_pcg_iters;}

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  bool& allow_warmstart() { return config_.allow_warmstart;};

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  double& max_feasible_step_safety_factor() { return config_.max_feasible_step_safety_factor;}

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  int&    hessian_max_skips() {return config_.hessian_max_skips;}
  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  double& hessian_min_acceptable_alpha() {return config_.hessian_min_acceptable_alpha;}
  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  double& hessian_min_acceptable_rel_objective_decrease() {return config_.hessian_min_acceptable_rel_objective_decrease;}

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  double& min_preconditioner_value() { return config_.min_preconditioner_value;}
  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  double& max_preconditioner_value() { return config_.max_preconditioner_value;}

  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  double& alpha_ls() {return config_.alpha_ls;}
  [[deprecated("Pass appropriate config to constructor, or use config()")]]
  double& beta_ls()  {return config_.beta_ls;}

  size_t n_iterations_used() const {return n_iterations_used_;}

  bool line_search_feasibility_step() {return config_.line_search_feasibility_step;}

  bool& compute_dual_variables() {return config_.compute_dual_variables;}

  VectorD& nue() {return nue_;}

  double backtracking_line_search(NProblemInterface* _problem,
                           const VectorD& _x, const double _fx,
                           const VectorD& _dx,
                           const double _gdx, const double _t_max, const int _max_iter_ls,
                           VectorD& _x_new, double& _fx_new, int& _iter_ls) const;

  double line_search_negative_curvature( NProblemInterface* _problem,
                                         const VectorD& _x, const double _fx,
                                         const VectorD& _dx,
                                         const double _t_max, const int _max_iter_ls,
                                         VectorD& _x_new, double& _fx_new, int& _iter_ls) const;


  double backtracking_line_search_infeasible_merit_l1(NProblemInterface* _problem, const SMatrixD& _H,
                                               const SMatrixD& _A, const VectorD& _b,
                                               const VectorD& _x, const double& _fx,
                                               const VectorD& _g, VectorD& _dx,
                                               VectorD& _x_new, double& _fx_new,
                                               double& _mu_merit,
                                               const double _t_start, const int _max_ls_iters, int& _n_iters) const;

  double max_abs_cos_angle(const SMatrixD& _A, const VectorD& _A_inv_row_norm, const VectorD& _v) const;

  void print_iteration_data(const OptimizerStatus& _status) const;
  void print_summary(const OptimizerStatus& _status) const;


private:
  TruncatedNewtonPCGConfig config_;
  // dual variables
  VectorD nue_;

  // deprecated: use status_
  size_t n_iterations_used_ = 0;

  // Optimizer Status for last iterate
  OptimizerStatus status_;

  // deprecated ---> remove and only use status_
  bool feasible_solution_found_ = false;
};


//=============================================================================
} // namespace COMISO
//=============================================================================

