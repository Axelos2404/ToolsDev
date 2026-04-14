/*===========================================================================*\
 *                                                                           *
 *                        ExactConstraintProjection                          *
 *      Copyright (C) 2025 by Computer Graphics Group, University of Bern    *
 *                           http://cgg.unibe.ch                             *
 *                                                                           *
 *      Author: David Bommes                                                 *
 *                                                                           *
\*===========================================================================*/

#pragma once

#include <CoMISo/Config/config.hh>
#include <CoMISo/Config/CoMISoDefines.hh>
#include <CoMISo/Solver/Eigen_Tools.hh>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace COMISO {

class COMISODLLEXPORT ExactConstraintProjection
{
public:

  // Sparse vector/matrix types with integer scalars
  using DVectorInt  = Eigen::VectorXi;
  using SVectorInt  = Eigen::SparseVector<int>;

  using SMatrixIntC = COMISO_EIGEN::HalfSparseColMatrix<int>;
  using SMatrixIntR = COMISO_EIGEN::HalfSparseRowMatrix<int>;

  using PairII = std::pair<int,int>;
  using PairDD = std::pair<double,double>;

  // default constructor
  ExactConstraintProjection() {}

  // transform linear system _A*x=_b into integer-reduced row echelon form (IRREF)
  // Return value: true upon success, otherwise false
  // Note I : _A and _b are expected to have integer coefficients only
  template<class SMatrixEigen,class DVectorEigen>
  bool initialize(const SMatrixEigen& _A, const DVectorEigen& _b);
  // same as above but with specific type
  bool initialize(const SMatrixIntR& _A, const DVectorInt& _b);

  // modify _x such that _A*x=b is exactly satisfied without any numerical error
  // Return value: true upon success, otherwise false
  // Note: As described in the addendum of "Exact Constraint Satisfaction for Truly Seamless Parametrization", the projection might be impossible for inhomogenous systems with _b not equal to 0
  template<class DVectorT>
  bool project(DVectorT& _x);

  // access truncation delta
  const double& delta() const {return delta_;}
        double& delta()       {return delta_;}

  // access truncation margin
  const int& K_margin() const {return K_margin_;}
  int&       K_margin()       {return K_margin_;}

  // zero all mantissa bits in conflict with F_delta (while rounding to the closest possibility)
  double round_to_F_delta(const double _d) const;

  // check whether a double is in F_delta
  bool is_in_F_delta(const double _d) const;

  // check for errors
  void check_consistency();

private:

  // transform linear system A_IRREF_R_*x=b_IRREF_ into integer-reduced row echelon form (IRREF)
  // simultaneously transfrom A_IRREF_C_*x=b_IRREF_
  bool transform_to_IRREF();

  // assume pairs where p.first is an integer coefficient and p.second is a double coefficient
  double safe_dot(const std::vector<PairDD>& _dp) const;
  // same as above but constructed from sparse vector
  template<class DVectorT>
  double safe_dot(const SVectorInt& _v, const DVectorT& _w) const;

private:
  // constraint matrix in IRREF both in row and column storage
  SMatrixIntR A_IRREF_R_;
  SMatrixIntC A_IRREF_C_;
  // rhs of constraint system in IRREF
  DVectorInt b_IRREF_;

  // colum of pivot of row i is stored in pivot_[i]
  // -1 if row has no pivot ---> zero row
  std::vector<int> pivot_;

  // distinguish between free and dependent variables. dependent are those chosen as pivots.
  std::vector<bool> is_free_variable_;

  // largest required exponent
  int K_ = 0;
  int K_margin_ = 1;
  // largest number 2^K_
  double delta_ = 0.0;
  // first bit, which will always be zeroed
  double epsilon_ = 0.0;

  // prefer unit pivots (coefficients +1/-1) at the cost of potentially more fill-in
  // empirically fill-in reduction is more important and delivers lower truncation errors
  bool prioritize_unit_pivots_ = false;

  const bool enable_detailed_logging_ = false;
};

//=============================================================================
} // namespace COMISO
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(COMISO_EXACTCONSTRAINTPROJECTION_C)
#define COMISO_EXACTCONSTRAINTPROJECTION_TEMPLATES
#include "ExactConstraintProjection_impl.hh"
#endif
