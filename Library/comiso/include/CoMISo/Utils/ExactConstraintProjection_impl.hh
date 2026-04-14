#pragma once
/*===========================================================================*\
 *                                                                           *
 *                        ExactConstraintProjection                          *
 *      Copyright (C) 2025 by Computer Graphics Group, University of Bern    *
 *                           http://cgg.unibe.ch                             *
 *                                                                           *
 *      Author: David Bommes                                                 *
 *                                                                           *
\*===========================================================================*/

#define COMISO_EXACTCONSTRAINTPROJECTION_C

#include "ExactConstraintProjection.hh"
#include <CoMISo/Utils/StopWatch.hh>
#include <numeric>
#include <cassert>

namespace COMISO
{


template<class SMatrixEigen, class DVectorEigen>
bool
ExactConstraintProjection::
initialize(const SMatrixEigen &_A, const DVectorEigen &_b)
{
  DEB_enter_func
  COMISO::StopWatch sw;
  sw.start();

  assert(_A.rows() == _b.size());

  // init empty row matrix and col matrix
  A_IRREF_R_ = SMatrixIntR(_A.rows(), _A.cols());
  A_IRREF_C_ = SMatrixIntC(_A.rows(), _A.cols());

  for (int k = 0; k < _A.outerSize(); ++k)
    for (typename SMatrixEigen::InnerIterator it(_A, k); it; ++it)
    {
      // verify the requirement of integer coefficients
      const int val_int = static_cast<int>(it.value());
      assert(it.value() == val_int);

      // TODO: can be done faster? (probably needs assumption on row/col major input)
      A_IRREF_R_.coeffRef(it.row(), it.col()) = val_int;
      A_IRREF_C_.coeffRef(it.row(), it.col()) = val_int;
    }

  // init rhs
  b_IRREF_.resize(_b.size());
  for (int i = 0; i < _b.size(); ++i)
  {
    // check requirement of integer coefficients
    assert(_b[i] == static_cast<int>(_b[i]));
    b_IRREF_[i] = _b[i];
  }

  // perform transformation
  bool valid = transform_to_IRREF();

  DEB_line(2, "IRREF transformation took " << sw.stop() / 1000.0 << " seconds");
  return valid;
}


//-----------------------------------------------------------------------------


template<class DVectorT>
bool
ExactConstraintProjection::
project(DVectorT &_x)
{
  DEB_enter_func;
  COMISO::StopWatch sw; sw.start();

  bool valid = true;

  // 0. smallest K where we can find a non-subnormal epsilon
  K_ = std::numeric_limits<double>::min_exponent + std::numeric_limits<double>::digits;
  // 1. determine K=max_i ceil(log2(|x_i|) + 1 + K_margin_ and delta = 2^K
  double max_abs = 0.0;
  for (int i = 0; i < _x.size(); ++i) {
    K_ = std::max(K_, std::ilogb(_x[i]));
  }

  K_ += 1 + K_margin_;

  // delta = 2^K
  delta_ = std::ldexp(1., K_);
  epsilon_ = std::ldexp(delta_, -std::numeric_limits<double>::digits);

  // verify epsilon
  assert(!is_in_F_delta(epsilon_));
  assert(round_to_F_delta(    epsilon_)==0.0);
  assert(round_to_F_delta(2.0*epsilon_) >0.0);

  DEB_line(2, "delta   = " << delta_);
  DEB_line(2, "epsilon = " << epsilon_);

  // 2. truncate free variables (collect divisors)
  double max_abs_diff_free_variables = 0.0;
  int max_lcm = 1;
  std::vector<size_t> dependent_variables;
  for (size_t i = 0; i < is_free_variable_.size(); ++i)
    if (is_free_variable_[i])
    {
      int lcm_i = 1;
      // collect pivots
      for (SVectorInt::InnerIterator it_col(A_IRREF_C_.col(i)); it_col; ++it_col)
      {
        const int row_idx = it_col.index();
        const int col_idx = pivot_[it_col.index()];

        lcm_i = std::lcm(lcm_i, A_IRREF_C_.coeffRef(row_idx, col_idx));
      }

      max_lcm = std::max(max_lcm, lcm_i);

      double x_old = _x[i];

      _x[i] = round_to_F_delta(_x[i] / lcm_i) * lcm_i;

      // collect statistics
      max_abs_diff_free_variables = std::max(max_abs_diff_free_variables, std::abs(x_old - _x[i]));

      assert(is_in_F_delta(_x[i]));
    }
    else
      dependent_variables.push_back(i);

  // 4. compute dependent variables (use safe_dot)
  double max_abs_diff_dependent_variables = 0.0;
  for (const auto pivot_i: dependent_variables)
  {
    assert(A_IRREF_C_.col(pivot_i).nonZeros() == 1); // a dependent variable has a single 1 in its column
    auto col_it = SVectorInt::InnerIterator(A_IRREF_C_.col(pivot_i));
    int row_idx = col_it.index();
    int C_pivot = col_it.value();

    // setup coefficients for safe_dot(a,b)
    std::vector<PairDD> dp;
    for (SVectorInt::InnerIterator row_it(A_IRREF_R_.row(row_idx)); row_it; ++row_it)
      if (row_it.index() != static_cast<Eigen::Index>(pivot_i))
      {
        assert((_x[row_it.index()] / C_pivot) * C_pivot == _x[row_it.index()]); // assume divisibility
        const double a = static_cast<double>(row_it.value());
        const double b = _x[row_it.index()] / C_pivot;
        dp.emplace_back(PairDD(a, b));
      }

    if(enable_detailed_logging_)
      DEB_line(2, "*** projection process variable " << pivot_i << " with #dp coefficients = " << dp.size());

    double b_div = b_IRREF_[row_idx] / C_pivot;

    // check divisibility of rhs
    if (static_cast<int>(b_div * C_pivot) != b_IRREF_[row_idx])
    {
      DEB_line(2, "Warning: rhs value " << b_IRREF_[row_idx] << " is not exactly divisible by row pivot value " << C_pivot);
      valid = false;
    }

    double x_old = _x[pivot_i];
    _x[pivot_i] = b_div - safe_dot(dp);
    max_abs_diff_dependent_variables = std::max(max_abs_diff_dependent_variables, std::abs(x_old - _x[pivot_i]));

    assert(is_in_F_delta(_x[pivot_i]));
  }

  // output statistics on max/avg change
  DEB_line(2, "max_diff_free_variables      = " << max_abs_diff_free_variables);
  DEB_line(2, "max_diff_dependent_variables = " << max_abs_diff_dependent_variables);
  DEB_line(2, "max_lcm                      = " << max_lcm);

  DEB_line(2, "project took " << sw.stop()/1000.0 << " seconds");

#if DEB_ON
  // verify result
  double max_abs_deviation = 0.0;
  for(int i=0; i<A_IRREF_C_.rows(); ++i)
  {
    double deviation = b_IRREF_[i]-safe_dot(A_IRREF_R_.row(i), _x);
    max_abs_deviation = std::max(max_abs_deviation, std::abs(deviation));
  }
  DEB_line(2, "result verification max_abs_deviation = " << max_abs_deviation);
#endif


  return valid;
}


//-----------------------------------------------------------------------------


template<class DVectorT>
double
ExactConstraintProjection::
safe_dot(const SVectorInt& _v, const DVectorT& _w) const
{
  std::vector<PairDD> dp;
  for (SVectorInt::InnerIterator row_it(_v); row_it; ++row_it)
    dp.emplace_back(PairDD(row_it.value(), _w[row_it.index()]));

  return safe_dot(dp);
}

} // namespace COMISO
