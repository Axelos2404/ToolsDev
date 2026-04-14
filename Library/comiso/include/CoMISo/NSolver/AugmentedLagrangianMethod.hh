//=============================================================================
//
//  CLASS AugmentedLagrangianMethod
//
//=============================================================================


#ifndef COMISO_AUGMENTEDLAGRANGIANMETHOD_HH
#define COMISO_AUGMENTEDLAGRANGIANMETHOD_HH

//== COMPILE-TIME PACKAGE REQUIREMENTS ========================================
#include <CoMISo/Config/config.hh>

//== INCLUDES =================================================================

#include <CoMISo/Config/CoMISoDefines.hh>
#include <CoMISo/Utils/StopWatch.hh>
#include "NProblemInterface.hh"
#include "NConstraintInterface.hh"
#include "AugmentedLagrangianProblem.hh"
#include "TruncatedNewtonPCG.hh"

#include <Base/Debug/DebTime.hh>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace COMISO {

//== CLASS DEFINITION =========================================================

class COMISODLLEXPORT AugmentedLagrangianMethod
{
public:

  using SVectorNC = NConstraintInterface::SVectorNC;
  using SMatrixNC = NConstraintInterface::SMatrixNC;

  using Triplet = Eigen::Triplet<double>;


  /// Default constructor
  AugmentedLagrangianMethod(const double _mu0 = 10.0, const double _eps_grad = 1e-6, const double _eps_constraints = 1e-9,
      const int _max_iters = 200)
      : mu0_(_mu0), eps_grad_(_eps_grad), eps_constraints_(_eps_constraints), max_iters_(_max_iters), converged_(false), silent_(false)
  {
  }

  int solve(NProblemInterface* _problem, std::vector<NConstraintInterface*>& _constraints)
  {
    DEB_time_func_def;
    converged_ = false;

    DEB_line_if(!silent_, 2, "optimize via AugmentedLagrangianMethod with " << _problem->n_unknowns() << " unknowns and " << _constraints.size() << " constraints");

    // set penalty and tolerances (mu = penalty parameter, tau = gradient threshold, eta = constraint threshold)
    double mu = mu0_;
    double tau = std::max(1.0/mu, eps_grad_);
    double eta = std::max(1.0/std::pow(mu, 0.1), eps_constraints_);

    // create augmented lagrangian function (dual variables are zero by default)
    AugmentedLagrangianProblem alp(_problem, _constraints, mu, 0.0);

    int iter=0;
    for(; iter < max_iters_; ++iter)
    {
      // update penalty parameter in alp
      alp.mu() = mu;

      // optimize unconstrained
      TruncatedNewtonPCG tn{TruncatedNewtonPCGConfig{.eps=tau, .always_update_preconditioner=true, .silent = silent_}};
      tn.solve(&alp);

      // current solution
      double *x = alp.primal_variables().data();
      // constraint violation
      double cv = alp.constraint_violation();
      // gradient norm
      double gn = alp.gradient_norm();

      if( cv < eta)
      {
        // found solution?
        if(cv < eps_constraints_ && gn < eps_grad_)
        {
          converged_ = true;
          DEB_line_if(!silent_,2, "*** ALM converged with grad_norm = " << gn << " and constraint_violation = " << cv);
          // store result
          _problem->store_result( x );
          return converged_;
        }

        // update multipliers, tighten tolerances
        for(size_t i=0; i< _constraints.size(); ++i)
          alp.dual_variables()[i] += mu*_constraints[i]->eval_constraint(x);

        // tighten tolerances but never more than final requirement
        eta = std::max(eta/std::pow(mu, 0.9), eps_constraints_);
        tau = std::max(tau/mu, eps_grad_);

        DEB_line_if(!silent_,2, "*** ALM iter = " << iter << " ---> UPDATE MULTIPLIERS, TIGHTEN TOLERANCES" << ", grad_norm = " << gn << ", constraint_violation = " << cv << ", penalty = " << mu << ", grad_eps = " << tau << ", constraint_eps = " << eta);
      }
      else
      {
        // increase penalty parameter
        mu *= 100.0;
        // reset tolerances
        eta = std::max(1.0/std::pow(mu,0.1), eps_constraints_);
        tau = std::max(1.0/mu, eps_grad_);

        DEB_line_if(!silent_,2, "ALM iter = " << iter << " ---> INCREASE PENALTY" << ", grad_norm = " << gn << ", constraint_violation = " << cv << ", penalty = " << mu << ", grad_eps = " << tau << ", constraint_eps = " << eta);
      }

      // stop if penalty gets too large
      if(mu > 1e10)
      {
        break;
      }

    }

    // constraint violation
    double cv = alp.constraint_violation();
    // gradient norm
    double gn = alp.gradient_norm();

    DEB_line_if(!silent_,2, "ALM did not converge after " << iter << "iterations, grad_norm = " << gn << " and constraint_violation = " << cv);

    // store result
    _problem->store_result( alp.primal_variables().data() );

    return converged_;
  }


  int solve(NProblemInterface* _problem, std::vector<NConstraintInterface*>& _constraints, std::vector<LinearConstraint>& _linear_constraints)
  {
    DEB_time_func_def;
    converged_ = false;

    DEB_line_if(!silent_,2, "optimize via AugmentedLagrangianMethod with " << _problem->n_unknowns() << " unknowns and " << _constraints.size() << " constraints");

    // set penalty and tolerances (mu = penalty parameter, tau = gradient threshold, eta = constraint threshold)
    double mu = mu0_;
    double tau = std::max(1.0/mu, eps_grad_);
    double eta = std::max(1.0/std::pow(mu, 0.1), eps_constraints_);

    // create augmented lagrangian function (dual variables are zero by default)
    AugmentedLagrangianProblem alp(_problem, _constraints, mu, 0.0);

    int iter=0;
    for(; iter < max_iters_; ++iter)
    {
      // update penalty parameter in alp
      alp.mu() = mu;

      // optimize unconstrained
      TruncatedNewtonPCG tn{TruncatedNewtonPCGConfig{.eps=tau, .always_update_preconditioner=true, .silent = silent_}};
      tn.solve_projected_normal_equation(&alp, _linear_constraints);

      // current solution
      double *x = alp.primal_variables().data();
      // constraint violation
      double cv = alp.max_constraint_violation();
      // gradient norm
//      double gn = alp.gradient_norm();
      double gn = tn.reduced_gradient_norm();

      if( cv < eta)
      {
        // found solution?
        if(cv < eps_constraints_ && gn < eps_grad_)
        {
          converged_ = true;
          DEB_line_if(!silent_,2, "*** ALM converged with grad_norm = " << gn << " and constraint_violation = " << cv);
          // store result
          _problem->store_result( x );
          return converged_;
        }

        // update multipliers, tighten tolerances
        for(size_t i=0; i< _constraints.size(); ++i)
          alp.dual_variables()[i] += mu*_constraints[i]->eval_constraint(x);

        // tighten tolerances but never more than final requirement
        eta = std::max(eta/std::pow(mu, 0.9), eps_constraints_);
        tau = std::max(tau/mu, eps_grad_);

        DEB_line_if(!silent_,2, "*** ALM iter = " << iter << " ---> UPDATE MULTIPLIERS, TIGHTEN TOLERANCES" << ", grad_norm = " << gn << ", constraint_violation = " << cv << ", penalty = " << mu << ", grad_eps = " << tau << ", constraint_eps = " << eta);
      }
      else
      {
        // increase penalty parameter
        mu *= 100.0;
        // reset tolerances
        eta = std::max(1.0/std::pow(mu,0.1), eps_constraints_);
        tau = std::max(1.0/mu, eps_grad_);

        DEB_line_if(!silent_,2, "ALM iter = " << iter << " ---> INCREASE PENALTY" << ", grad_norm = " << gn << ", constraint_violation = " << cv << ", penalty = " << mu << ", grad_eps = " << tau << ", constraint_eps = " << eta);
      }

      // stop if penalty gets too large
      if(mu > 1e10)
      {
        break;
      }

    }

    // constraint violation
    double cv = alp.constraint_violation();
    // gradient norm
    double gn = alp.gradient_norm();

    DEB_line_if(!silent_,2, "ALM did not converge after " << iter << "iterations, grad_norm = " << gn << " and constraint_violation = " << cv);

    // store result
    _problem->store_result( alp.primal_variables().data() );

    return converged_;
  }


  int solve_experimental(NProblemInterface* _problem, std::vector<NConstraintInterface*>& _constraints, std::vector<LinearConstraint>& _linear_constraints)
  {
    DEB_enter_func;
//    DEB_time_func_def;
    converged_ = false;

    DEB_line_if(!silent_,2, "*** optimize via AugmentedLagrangianMethod with " << _problem->n_unknowns() << " unknowns and " << _constraints.size() << " constraints");

    // set penalty and tolerances (mu = penalty parameter, tau = gradient threshold, eta = constraint threshold)
    double mu = mu0_;
    double tau = eps_grad_;
    double eta = eps_constraints_;

    // create augmented lagrangian function (dual variables are zero by default)
    AugmentedLagrangianProblem alp(_problem, _constraints, mu, 0.0);

    double gn(0.0), cv(0.0);
    int iter=0;
    for(; iter < max_iters_; ++iter)
    {
      // update penalty parameter in alp
      alp.mu() = mu;

      // optimize unconstrained
      TruncatedNewtonPCG tn{TruncatedNewtonPCGConfig{.eps=tau, .always_update_preconditioner=true, .silent = silent_}};
      tn.solve_projected_normal_equation(&alp, _linear_constraints);

      // current solution
      double *x = alp.primal_variables().data();
      // constraint violation
      cv = alp.max_constraint_violation();
      // gradient norm
      gn = tn.reduced_gradient_norm();

      // found solution?
      if(cv < eps_constraints_ && gn < eps_grad_)
      {
        converged_ = true;
        DEB_line_if(!silent_,2, "*** ALM converged with grad_norm = " << gn << " and constraint_violation = " << cv);
        // store result
        _problem->store_result( x );
        return converged_;
      }

      DEB_line_if(!silent_,2, "*** ALM iter = " << iter << ", grad_norm = " << gn << ", constraint_violation = " << cv << ", penalty = " << mu << ", grad_eps = " << tau << ", constraint_eps = " << eta);

      // update multipliers, tighten tolerances
      for(size_t i=0; i< _constraints.size(); ++i)
        alp.dual_variables()[i] += mu*_constraints[i]->eval_constraint(x);

      // increase penalty parameter
      mu *= double(iter+1);

    }

    DEB_line_if(!silent_,2, "*** ALM did not converge after " << iter << "iterations, grad_norm = " << gn << " and constraint_violation = " << cv);

    // store result
    _problem->store_result( alp.primal_variables().data() );

    return converged_;
  }

  bool converged() { return converged_; }

  void set_silent(const bool _silent) { silent_ = _silent;}

private:

  double mu0_;
  double eps_grad_;
  double eps_constraints_;
  int    max_iters_;

  bool converged_;
  bool silent_;
};


//=============================================================================
} // namespace COMISO
//=============================================================================
#endif // COMISO_AUGMENTEDLAGRANGIANMETHOD_HH defined
//=============================================================================

