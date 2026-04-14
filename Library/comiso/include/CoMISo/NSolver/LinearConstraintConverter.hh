//=============================================================================
//
//  CLASS LinearConstraintConverter
//
//=============================================================================


#ifndef COMISO_LINEARCONSTRAINTCONVERTER_HH
#define COMISO_LINEARCONSTRAINTCONVERTER_HH


//== INCLUDES =================================================================

#include <CoMISo/Config/CoMISoDefines.hh>
#include "NConstraintInterface.hh"
#include "LinearConstraint.hh"

#include <Base/Code/Quality.hh>
LOW_CODE_QUALITY_SECTION_BEGIN
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
LOW_CODE_QUALITY_SECTION_END


//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace COMISO {

//== CLASS DEFINITION =========================================================



/** \class LinearConstraintConverter LinearConstraintConverter.hh <COMISO/.../LinearConstraintConverter.hh>

    Brief Description.

    A more elaborate description follows.
*/
class COMISODLLEXPORT LinearConstraintConverter
{
public:

  // sparse vector type
  typedef NConstraintInterface::SVectorNC SVectorNC;

  typedef Eigen::SparseMatrix<double,Eigen::ColMajor> SparseMatrixC;
  typedef Eigen::SparseMatrix<double,Eigen::RowMajor> SparseMatrixR;
  typedef Eigen::VectorXd                             VectorXd;

  LinearConstraintConverter(SparseMatrixR& _A, VectorXd& _b)
  : A_(_A), b_(_b), linear_constraints_initialized_(false)
  {}

  LinearConstraintConverter(SparseMatrixC& _A, VectorXd& _b)
  : A_(_A), b_(_b), linear_constraints_initialized_(false)
  {}

  std::vector<NConstraintInterface*>& constraints_nsolver()
  {
    initialize_linear_constraints();
    return constraint_pointers_;
  }

  // TODO
//  static void eigen_to_nsolver()
//  {
//
//  }

template <class SMatrixT, class VectorT>
static void nsolver_to_eigen( std::vector<LinearConstraint>& _constraints, SMatrixT& _A, VectorT& _b, int _n_variables = 0)
{
  if(_constraints.empty())
  {
    _A.resize(0,_n_variables);
    _b.resize(0);
    return;
  }

  int m = _constraints.size();
  int n = _constraints[0].n_unknowns();
  _A.resize(m,n);
  _b.resize(m);

  std::vector<Eigen::Triplet<double> > triplets;

  for(size_t i=0; i<_constraints.size(); ++i)
  {
    _b[i] = -_constraints[i].b();

    NConstraintInterface::SVectorNC::InnerIterator it(_constraints[i].coeffs());
    for(; it; ++it)
      triplets.push_back(Eigen::Triplet<double>(i, it.index(), it.value()));
  }

  _A.setFromTriplets(triplets.begin(), triplets.end());
}

template <class SMatrixT, class VectorT>
static void nsolver_to_eigen( std::vector<NConstraintInterface*>& _constraints, SMatrixT& _A, VectorT& _b, int _n_variables = 0)
{
  if(_constraints.empty())
  {
    _A.resize(0,_n_variables);
    _b.resize(0);
    return;
  }

  int m = _constraints.size();
  int n = _constraints[0]->n_unknowns();
  _A.resize(m,n);
  _b.resize(m);

  Eigen::VectorXd x(n);
  x.setZero();

  std::vector<Eigen::Triplet<double> > triplets;

  for(size_t i=0; i<_constraints.size(); ++i)
  {
    // get constant
    _b[i] = -_constraints[i]->eval_constraint(x.data());

    // get coefficients
    NConstraintInterface::SVectorNC g(n);
    _constraints[i]->eval_gradient(x.data(), g);

    NConstraintInterface::SVectorNC::InnerIterator it(g);
    for(; it; ++it)
      triplets.push_back(Eigen::Triplet<double>(i, it.index(), it.value()));
  }

  _A.setFromTriplets(triplets.begin(), triplets.end());
}

private:

  void initialize_linear_constraints()
  {
    if(!linear_constraints_initialized_)
    {
      // tag as done
      linear_constraints_initialized_ = true;

      int m = A_.rows();
      int n = A_.cols();

      linear_constraints_.clear();
      linear_constraints_.resize(m);

      constraint_pointers_.clear();
      constraint_pointers_.resize(m);

      for( int i=0; i<m; ++i)
      {
        // convert i-th constraint
        linear_constraints_.resize(n);
        linear_constraints_[i].coeffs() =  A_.row(i);
        linear_constraints_[i].b()      = -b_[i];
        // store pointer
        constraint_pointers_[i] = &(linear_constraints_[i]);
      }
    }
  }

private:
  SparseMatrixR A_;
  VectorXd      b_;

  bool                               linear_constraints_initialized_;
  std::vector<LinearConstraint>      linear_constraints_;
  std::vector<NConstraintInterface*> constraint_pointers_;
};


//=============================================================================
} // namespace COMISO
//=============================================================================
//=============================================================================
#endif // COMISO_LINEARCONSTRAINTCONVERTER_HH defined
//=============================================================================

