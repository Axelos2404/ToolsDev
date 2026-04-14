//=============================================================================
//
//  CLASS AugmentedLagrangianProblem
//
//=============================================================================


#ifndef COMISO_AUGMENTEDLAGRANGIANPROBLEM_HH
#define COMISO_AUGMENTEDLAGRANGIANPROBLEM_HH


//== COMPILE-TIME PACKAGE REQUIREMENTS ========================================
#include <CoMISo/Config/config.hh>
#if COMISO_EIGEN3_AVAILABLE

//== INCLUDES =================================================================

#include <stdio.h>
#include <iostream>
#include <vector>

#include <CoMISo/Config/CoMISoDefines.hh>
#include <CoMISo/NSolver/NProblemInterface.hh>
#include <CoMISo/NSolver/NConstraintInterface.hh>

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace COMISO {

//== CLASS DEFINITION =========================================================

	      

/** \class AugmentedLagrangianProblem AugmentedLagrangianProblem.hh <CoMISo/NSolver/AugmentedLagrangianProblem.hh>

    Create Problem of the type ALM(x) = f(x) + sum_i nue_i*h_i(x) + mu * sum_i h_i(x)^2

    input:
    - objective function f(x)
    - vector of equality constraint functionc h_i(x)
    - quadratic penalty parameter mu
    - values of dual variables nue_i (one for each constraint)

    A more elaborate description follows.
*/
class COMISODLLEXPORT AugmentedLagrangianProblem : public NProblemInterface
{
public:

  using SVectorNC = NConstraintInterface::SVectorNC;
  using SMatrixNC = NConstraintInterface::SMatrixNC;

  using Triplet = Eigen::Triplet<double>;
  
  /// Default constructor
  AugmentedLagrangianProblem (NProblemInterface* _f, std::vector<NConstraintInterface*>& _h, const double _mu = 1.0, const double _nue = 0.0)
   : f_(_f), h_(_h), mu_(_mu), dual_var_(_h.size(), _nue)
  {
    for(size_t i=0; i<h_.size(); ++i)
      if(h_[i]->constraint_type() != NConstraintInterface::NC_EQUAL)
        std::cerr << "Error: AugmentedLagrangianProblem received a constraint which is not of type NC_EQUAL\n";

      // get starting point
      primal_var_.resize(_f->n_unknowns());
      f_->initial_x(primal_var_.data());
  }

  // access quadratic penalty parameter
  double& mu() { return mu_;}

  // access primal variables
  std::vector<double>& primal_variables() {return primal_var_;}

  // access dual variables
  std::vector<double>& dual_variables() {return dual_var_;}

  // problem definition
  virtual int    n_unknowns() { return f_->n_unknowns();}

  virtual void   initial_x(double* _x)
  {
    for(int i=0; i<this->n_unknowns(); ++i)
      _x[i] = primal_var_[i];
  }

  virtual double eval_f( const double* _x)
  {
    double f = f_->eval_f(_x);

    for(size_t i=0; i<h_.size(); ++i)
    {
      // evaluate constraint function
      double hx = h_[i]->eval_constraint(_x);
      // add linear part
      f += dual_var_[i]*hx;
      // add quadratic part
      f += 0.5*mu_*hx*hx;
    }

    return f;
  }

  virtual void   eval_gradient( const double* _x, double* _g)
  {
    f_->eval_gradient(_x, _g);

    for(size_t i=0; i<h_.size(); ++i)
    {
      h_[i]->eval_gradient(_x, gs_temp_);

      double w = dual_var_[i] + mu_*h_[i]->eval_constraint(_x);

      SVectorNC::InnerIterator it(gs_temp_);
      for(; it; ++it)
        _g[it.index()] += w*it.value();
    }
  }

  virtual void   eval_hessian ( const double* _x, SMatrixNP& _H)
  {
    f_->eval_hessian(_x, _H);

    // clear old data
    triplets_.clear();
    for (int k=0; k<_H.outerSize(); ++k)
      for (SMatrixNP::InnerIterator it(_H,k); it; ++it)
      {
        triplets_.push_back( Triplet(it.row(), it.col(), it.value()) );
      }

    // add hessian information from constraint terms
    for(size_t i=0; i<h_.size(); ++i)
    {
      h_[i]->eval_gradient(_x, gs_temp_);

      SVectorNC::InnerIterator it(gs_temp_);
      for(; it; ++it)
      {
        SVectorNC::InnerIterator it2(gs_temp_);
        for(; it2; ++it2)
          triplets_.push_back(Triplet(it.index(),it2.index(),mu_*it.value()*it2.value()));
      }

      if(!h_[i]->is_linear())
      {
        double w = dual_var_[i]+mu_*h_[i]->eval_constraint(_x);
        h_[i]->eval_hessian(_x, Hh_temp_);

        SMatrixNC::iterator H_it = Hh_temp_.begin();
        SMatrixNC::iterator H_end = Hh_temp_.end();

        for(; H_it != H_end; ++H_it)
          triplets_.push_back( Triplet(H_it.row(), H_it.col(), w*(*H_it)) );
      }
    }

    _H.setFromTriplets(triplets_.begin(), triplets_.end());
  }

  virtual void   store_result ( const double* _x )
  {
    for( int i=0; i<this->n_unknowns(); ++i)
      primal_var_[i] = _x[i];
  }

  // advanced properties
  virtual bool   constant_gradient() const
  {
    if(mu_> 0.0) return false;

    if(!f_->constant_gradient()) return false;

    for(size_t i=0; i<h_.size(); ++i)
      if(!h_[i]->is_linear()) return false;

    return true;
  }
  virtual bool   constant_hessian()  const
  {
    if(!f_->constant_hessian()) return false;

    if(mu_>0.0)
    {
      for(size_t i=0; i<h_.size(); ++i)
        if(!h_[i]->is_linear())
          return false;

        for(size_t i=0; i<h_.size(); ++i)
          if(!h_[i]->constant_hessian())
            return false;
     }

    return true;
  }

  virtual double max_feasible_step ( const double* _x, const double* _v)
  {
    return f_->max_feasible_step(_x,_v);
  }

  double gradient_norm()
  {
    g_temp_.resize(this->n_unknowns());
    this->eval_gradient(primal_var_.data(), g_temp_.data());

    return g_temp_.norm();
  }

  double constraint_violation()
  {
    double cv(0.0);

    for(size_t i=0; i<h_.size(); ++i)
      cv += std::abs( h_[i]->eval_constraint(primal_var_.data()));

    return cv;
  }

  double max_constraint_violation()
  {
    double cv(0.0);

    for(size_t i=0; i<h_.size(); ++i)
      cv = std::max(cv, std::abs( h_[i]->eval_constraint(primal_var_.data())));

    return cv;
  }

private:

  // objective function
  NProblemInterface* f_;
  // vector of constraints
  std::vector<NConstraintInterface*> h_;

  // quadratic penalty parameter
  double mu_;

  // primal variables
  std::vector<double> primal_var_;

  // dual variables
  std::vector<double> dual_var_;

  // temporary variables
  Eigen::VectorXd g_temp_;
  SVectorNC gs_temp_;
  SMatrixNC Hh_temp_;
  std::vector<Triplet> triplets_;
};


//=============================================================================
} // namespace COMISO
//=============================================================================
#endif // COMISO_EIGEN3_AVAILABLE
//=============================================================================
#endif // COMISO_AUGMENTEDLAGRANGIANPROBLEM_HH defined
//=============================================================================

