//=============================================================================
//
//  CLASS TrustregionNewtonPCG
//
//=============================================================================


#ifndef COMISO_TRUSTREGIONNEWTONPCG_HH
#define COMISO_TRUSTREGIONNEWTONPCG_HH

//== COMPILE-TIME PACKAGE REQUIREMENTS ========================================
#include <CoMISo/Config/config.hh>

//== INCLUDES =================================================================

#include <CoMISo/Config/CoMISoDefines.hh>
#include <CoMISo/Utils/StopWatch.hh>
#include <CoMISo/NSolver/NProblemInterface.hh>
#include <CoMISo/NSolver/NConstraintInterface.hh>
#include <CoMISo/NSolver/LinearConstraintConverter.hh>

#include <Base/Debug/DebTime.hh>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace COMISO {

//== CLASS DEFINITION =========================================================

	      

/** \class TrustregionNewtonPCG TrustregionNewtonPCG.hh <CoMISo/.../TrustregionNewtonPCG.hh>

    Brief Description.
  
    A more elaborate description follows.
*/
class COMISODLLEXPORT TrustregionNewtonPCG
{
public:

  typedef Eigen::VectorXd             VectorD;
  typedef Eigen::SparseMatrix<double> SMatrixD;
  typedef Eigen::Triplet<double>      Triplet;

  /// Default constructor
  TrustregionNewtonPCG(const double _eps = 1e-3,
                       const double _eps_trust_region_radius = 1e-8,
                       const int    _max_iters = 200,
                       const int    _max_pcg_iters = 300,
                       const double _eta_shrink_radius = 0.25,
                       const double _eta_grow_radius = 0.75,
                       const double _eta_accept_step = 0.1,
                       const double _shrink_radius_scale = 0.25,
                       const double _grow_radius_scale = 2.0,
                       const double _max_trust_region_radius = 1e6 )
      : eps_(_eps), eps_trust_region_radius_( _eps_trust_region_radius), max_iters_(_max_iters), max_pcg_iters_(_max_pcg_iters),
        eta_shrink_radius_(_eta_shrink_radius), eta_grow_radius_(_eta_grow_radius), eta_accept_step_(_eta_accept_step),
        shrink_radius_scale_(_shrink_radius_scale), grow_radius_scale_(_grow_radius_scale), max_trust_region_radius_(_max_trust_region_radius),
        adaptive_tolerance_(true), always_update_preconditioner_(true), try_shortened_step_(true)
  {
  }

  // solve with linear constraints
  // Warning: so far only feasible starting points with (_A*_problem->initial_x() == b) are supported!
  // It is also required that the constraints are linearly independent
  int solve( NProblemInterface* _problem, const SMatrixD& _A, const VectorD& _b )
  {
    DEB_time_func_def;
    converged_ = false;

    // number of unknowns
    size_t n = _problem->n_unknowns();
    size_t m = _A.rows();

    DEB_line(2, "optimize via TrustregiondNewtonProjectedNormalEquationsPCG with " << n << " unknowns and " << m << " linear constraints");

    // Newton parameters
    const int    max_iters  = max_iters_;
    const double newton_tol = eps_; // norm of gradient of reduced (unconstrained) problem
    // CG parameters
    const int    max_pcg_iters = max_pcg_iters_;
    const double pcg_tol = 1e-4;

    // initialize vectors of unknowns
    VectorD x(n);
    _problem->initial_x(x.data());

    double constraint_violation = (_A*x-_b).norm();
    DEB_line(2, "initial constraint violation = " << constraint_violation);
    if(constraint_violation > 1e-6)
      std::cerr << "Warning: this method expects a feasible starting point!" << std::endl;

    // storage of update vector dx and gradient
    VectorD g(n), dx(n), xn(n), gz(n);

    // storage CG
    VectorD  r(n), v(n), q(n), p(n), r2(n), q2(n), Hp(n), W2dx(n), W2p(n);

    // hessian matrix
    SMatrixD H(n,n);

    // Diagonal Preconditioner
    VectorD W(n), W2(n), Wi(n);

    // cholesky decomposition of projection
    Eigen::SimplicialLDLT<SMatrixD> ldlt;

    // statistics
    int n_pcg_iters(0);

    // choose precision of CG solve
    double eta = pcg_tol;

    bool ls_succeeded = true;

    bool cg_converged = false;

    // get function value at current point
    double fx = _problem->eval_f(x.data());

    // trustregion radius
    double trr(0.0);

    for(int iter=0; iter<max_iters; ++iter)
    {
      // get gradient and Hessian
      _problem->eval_gradient(x.data(), g.data());
      _problem->eval_hessian(x.data(), H);

      // pre-factor projection matrix and update preconditioner
      if(iter == 0 || always_update_preconditioner_)
      {
        // setup preconditioner W = (|diag(H)|)
        // and Wi = (|diag(H)|)^-1
        for(int j=0; j<n ;++j)
        {
          W[j]  = std::abs(H.coeffRef(j,j));
          W2[j] = std::sqrt(W[j]);
          Wi[j] = 1.0/W[j];

//          // no preconditioner
//          W[j]  = 1.0;
//          W2[j] = 1.0;
//          Wi[j] = 1.0;
        }

        // prepare constraint projection
//        DEB_line(2, "prepare constraint projection ..." );
        constraint_violation = (_A*x-_b).norm();
//        DEB_line(2, "initial constraint violation = " << constraint_violation);
        SMatrixD AWiAt = _A*Wi.asDiagonal()*_A.transpose();
        ldlt.compute(AWiAt);
        if(ldlt.info() != Eigen::Success)
        {
          for(unsigned int j=0; j<10; ++j)
          {
            double reg = 1e-8 * AWiAt.diagonal().sum() / double(m);
            DEB_line(2,
                     "Warning: LDLT factorization failed --> regularize AAt (could lead to reduced accuracy), reg=" << reg);
            for (size_t j = 0; j < AWiAt.rows(); ++j)
              AWiAt.coeffRef(j, j) += reg;

            ldlt.compute(AWiAt);
            if(ldlt.info() == Eigen::Success)
              break;
          }

        }
//        DEB_line(2, "done!" );
      }

      // ---------------------------
      // Projected-Preconditioned-CG
      bool step_reached_tr_bound = false;
      dx.setZero();
      r = g;

      // project r  --> q
      v  = ldlt.solve(_A*Wi.asDiagonal()*r);
      gz = r - _A.transpose()*v;
      q  = Wi.asDiagonal()*gz;

      p = -q;

      double rtq = r.dot(q);

      // check convergence
      double gzn = gz.norm();
      if(gzn < newton_tol)
      {
        constraint_violation = (_A*x-_b).norm();

        DEB_line(4,"converged" <<  ", f(x) = " << fx
                               << ", ||gz|| = " << gzn
                               << ", constraint_violation = " << constraint_violation
                               << ", " << "PCG_iters = " << n_pcg_iters );
        break;
      }

      // choose CG tolerance
      if(adaptive_tolerance_)
        eta = std::min(0.1, std::sqrt(gzn))*gzn;
      else
        eta = pcg_tol*gzn;

      for(int pcg_iter=0; pcg_iter < max_pcg_iters; ++pcg_iter)
      {
        // compute norm of projected residual (in original norm)
        double rpn = (W.asDiagonal()*q).norm();

        // debug
        if(0)
        {
          VectorD rr = H * dx + g;
          v = ldlt.solve(_A*Wi.asDiagonal()*rr);
          VectorD rr2 = (rr - _A.transpose()*v);

          std::cerr << "++++++++++++++++++++++++++" << std::endl
                    << "||P(Hdx+g)|| = " << rr2.norm() << std::endl
                    << "||qn||       = " << rpn << std::endl
                    << "sqrt(rtq)    = " << std::sqrt(rtq) << std::endl
                    << std::endl;
        }

        // stop if accuracy sufficient
        if(rpn < eta)
        {
          cg_converged = true;
          break;
        }

        // cache re-used quantities
        Hp = H*p;
        double pHp = p.dot(Hp);

        // stop if direction of negative curvature is found
        if(pHp <= 0.0)
        {
          //compute minimizer along y = dx + tau*p with ||y||=trr
          std::cerr << "----> observed negative curvature (case has not been tested yet)" << std::endl;

          W2dx = W2*dx;
          W2p  = W2*p;
          double a = W2p.dot(W2p);
          double b = 2.0*W2p.dot(W2dx);
          double c = W2dx.dot(W2dx)-trr*trr;

          double dd = b*b-4.0*a*c;
          if(dd < -1e-6)
            std::cerr << "Warning: could not find intersection of step with trust-region boundary: " << dd << std::endl;
          dd = std::sqrt(std::max(dd,0.0));

          double t0 = (-b+dd)/(2.0*a);
          double t1 = (-b-dd)/(2.0*a);

          // generate both candidate solutions
          W2dx = dx + t0*p;
          dx += t1*p;

          // check which one has lower value in quadratic model function
          if( W2dx.dot(g) + W2dx.transpose()*H*W2dx < dx.dot(g) + dx.transpose()*H*dx)
            dx.swap(W2dx);

          // debug output
          if(1)
          {
            W2dx = W2*dx;
            std::cerr << "---> reached trust-region boundary, dx.dot(dx)/r^2 = " << W2dx.dot(W2dx)/(trr*trr) << std::endl;
          }

          step_reached_tr_bound = true;
          break;
        }

        // optimal step length along p (minimizer of quadratic)
        double alpha = rtq/(p.dot(Hp));
        // update dx
        dx += alpha*p;

        // count number of pcg iters
        ++n_pcg_iters;

        // get dx in norm of preconditioner
        W2dx = W2*dx;

        // first iteration?
        if( iter == 0)
        {
          // initialize trust-region
          trr = std::sqrt(W2dx.dot(W2dx));
          break;
        }

        // left trust-region (defined w.r.t. to preconditioner, i.e. ||Wdx|| <= trr)?
        if( W2dx.dot(W2dx) >= trr*trr)
        {
          // shrink last step to match trust-region
          W2p = W2*p;
          double a = W2p.dot(W2p);
          double b = 2.0*W2p.dot(W2dx);
          double c = W2dx.dot(W2dx)-trr*trr;

          double dd = b*b-4.0*a*c;
          if(dd < -1e-6)
            std::cerr << "Warning: could not find intersection of step with trust-region boundary: " << dd << std::endl;
          dd = std::sqrt(std::max(dd,0.0));

          double t0 = (-b+dd)/(2.0*a);
          double t1 = (-b-dd)/(2.0*a);

          // sort
          if(t1 < t0)
            std::swap(t0,t1);

          dx += t1*p;

          // debug output
          if(0)
          {
            W2dx = W2*dx;
            std::cerr << "---> reached trust-region boundary, dx.dot(dx)/r^2 = " << W2dx.dot(W2dx)/(trr*trr) << std::endl;
          }

          step_reached_tr_bound = true;
          break;
        }

        r2 = r + alpha*Hp;

        // project r2  --> q2
        v = ldlt.solve(_A*Wi.asDiagonal()*r2);
        q2 = Wi.asDiagonal()*(r2 - _A.transpose()*v);


        double rtq2 = r2.dot(q2);
        double beta = rtq2/rtq;
        p = -q2 + beta*p;

        // swap vectors
        q.swap(q2);
        // r.swap(r2);
        // constraint refinement
        r = r2 - _A.transpose()*v;
        // update rtq
        rtq = rtq2;
      }

      // get new point
      xn = x + dx;
      double fxn = _problem->eval_f(xn.data());

      // evaluate accuracy of quadratic model
      double actual_reduction    = fx - fxn;
      double predicted_reduction = -g.dot(dx) - 0.5*dx.transpose()*H*dx;
      double observed_accuracy   = actual_reduction/predicted_reduction;

      // catch out-of-domain situations and numerical issues
      if(!std::isfinite(observed_accuracy))
        observed_accuracy = -HUGE_VAL;

      if(observed_accuracy < eta_shrink_radius_)
        trr *= shrink_radius_scale_;
      else if( observed_accuracy > eta_grow_radius_ && step_reached_tr_bound)
          trr = std::min(grow_radius_scale_ * trr, max_trust_region_radius_);

      // accept step?
      double step_size(0.0);
      if(observed_accuracy > eta_accept_step_)
      {
        fx = fxn;
        x.swap(xn);
        step_size = 1.0;
      }
      else
        if(try_shortened_step_)
        {
          // try to use shorter step to at least make some progress
          xn = x + shrink_radius_scale_ * dx;
          double fxn = _problem->eval_f(xn.data());

          // evaluate accuracy of quadratic model
          double actual_reduction = fx - fxn;
          double predicted_reduction = -g.dot(dx) - 0.5 * dx.transpose() * H * dx;
          double observed_accuracy = actual_reduction / predicted_reduction;

          // catch out-of-domain situations and numerical issues
          if (!std::isfinite(observed_accuracy))
            observed_accuracy = -HUGE_VAL;

          // accept step?
          if (observed_accuracy > eta_accept_step_)
          {
            fx = fxn;
            x.swap(xn);
            step_size = shrink_radius_scale_;
          }
        }

      if(trr < eps_trust_region_radius_)
      {
        constraint_violation = (_A*x-_b).norm();

//        DEB_line(4,"terminate due to vanishing trustregion-radius"
//                               <<  ", f(x) = " << std::setw(8) << fx << ", r = " << std::setw(8) << trr
//                               << ", observed accuracy=" << std::setw(8) << observed_accuracy
//                               << ", |reduced grad| = " << std::setw(8) << gzn
//                               << ", constraint_violation = " << std::setw(8) << constraint_violation
//                               << ", " << "PCG_iters = " << std::setw(8) << n_pcg_iters );

        std::cerr << "terminate due to vanishing trustregion-radius"
                <<  ", f(x) = " << std::setw(14) << fx << ", r = " << std::setw(8) << trr
                << ", observed accuracy=" << std::setw(8) << observed_accuracy
                << ", |reduced grad| = " << std::setw(14) << gzn
                << ", constraint_violation = " << std::setw(8) << constraint_violation
                << ", " << "PCG_iters = " << std::setw(8) << n_pcg_iters << std::endl;

        break;
      }

      // output iteration statistics
//      DEB_line(4,
//               "iter = " << iter << ", f(x) = " << std::setw(8) << fx << ", r = " << std::setw(8) << trr
//                         << ", observed accuracy=" << std::setw(8) << observed_accuracy
//                         << ", |reduced grad| = " << std::setw(8) << gzn
//                         << ", " << "PCG_tol = " << std::setw(8) << eta
//                         << ", " << "PCG_iters = " << std::setw(8) << n_pcg_iters
//                         << ", " << "PCG_converged = " << int(cg_converged));

      std::cerr << "iter = " << std::setw(4) << iter << ", f(x) = " << std::setw(14) << fx << ", r = " << std::setw(8) << trr
                         << ", observed accuracy=" << std::setw(8) << observed_accuracy
                         << ", |reduced grad| = " << std::setw(14) << gzn
                         << ", " << "PCG_tol = " << std::setw(8) << eta
                         << ", " << "PCG_iters = " << std::setw(6) << n_pcg_iters
                         << ", " << "PCG_converged = " << int(cg_converged)
                         << ", " << "TR bound = " << int(step_reached_tr_bound)
                         << ", " << "step size = " << step_size
                         << std::endl;
    }

    // store result
    _problem->store_result(x.data());

    // return success
    return converged_;
  }

  int solve(NProblemInterface* _problem, std::vector<LinearConstraint>& _constraints)
  {
    // convert constraints
    SMatrixD A;
    VectorD b;
    LinearConstraintConverter::nsolver_to_eigen(_constraints, A, b);

    return solve(_problem, A, b);
  }

  bool converged() { return converged_; }

private:

  double eps_;
  double eps_trust_region_radius_;
  int    max_iters_;
  int    max_pcg_iters_;
  double eta_shrink_radius_;
  double eta_grow_radius_;
  double eta_accept_step_;

  double shrink_radius_scale_;
  double grow_radius_scale_;
  double max_trust_region_radius_;

  // adaptively choose tolerance of CG optimization?
  bool adaptive_tolerance_;
  bool always_update_preconditioner_;
  bool try_shortened_step_;

  bool converged_;
};


//=============================================================================
} // namespace COMISO
//=============================================================================
#endif // COMISO_TRUNCATEDNEWTONPCG_HH defined
//=============================================================================

