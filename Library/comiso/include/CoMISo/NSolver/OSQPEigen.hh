#pragma once

#include <CoMISo/Config/config.hh>
#if COMISO_OSQP_AVAILABLE

#include <CoMISo/Config/CoMISoDefines.hh>
#include <CoMISo/Utils/CoMISoError.hh>

#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Sparse>
#include <osqp.h>

#include <Base/Debug/DebUtils.hh>
#include <Base/Debug/DebOut.hh>
#include <Base/Debug/DebTime.hh>


namespace COMISO
{

#if COMISO_OSQP_NEW_API
    using c_int = OSQPInt;
    using c_float = OSQPFloat;
    struct OSQPData {
        ~OSQPData() {
        }
        std::unique_ptr<OSQPCscMatrix> P;
        const OSQPFloat*     q = nullptr;
        std::unique_ptr<OSQPCscMatrix> A;
        const OSQPFloat*     l = nullptr;
        const OSQPFloat*     u = nullptr;
        OSQPInt              m;
        OSQPInt              n;
    };
#endif

//== CLASS DEFINITION =========================================================

/** \class OSQP with EIGEN datastructures OSQPEigen.hh

    Solver for quadratic problem with linear equality and linear inequality
   constraints based on OSQP.
*/
class COMISODLLEXPORT OSQPEigen
{
public:

  OSQPEigen()
  {
    osqp_set_default_settings(&settings_);
    settings_.alpha = 1.0; // this value works better than the default
    settings_.max_iter = 10000;
#if COMISO_OSQP_NEW_API
    settings_.warm_starting = 1;
    settings_.polishing = 1;
#else
    settings_.warm_start = true;
    settings_.polish = 1;
#endif
    settings_.polish_refine_iter = 5;
    settings_.eps_abs = 1e-5;      // absolute convergence tolerance
    settings_.eps_rel = 1e-5;      // relative convergence tolerance
    settings_.eps_prim_inf = 1e-6; // primal infeasibility tolerance
    settings_.eps_dual_inf = 1.;   // dual infeasibility tolerance
    settings_.verbose = 0;   // verbosity
    // settings.linsys_solver = MKL_PARDISO_SOLVER;

    data_.n = 0;
    data_.m = 0;
    data_.P = nullptr;
    data_.A = nullptr;
    data_.q = nullptr;
    data_.l = nullptr;
    data_.u = nullptr;

    work_ = nullptr;
  }

  ~OSQPEigen()
  {
    osqp_cleanup(work_);
#if !COMISO_OSQP_NEW_API
    // c_free is the OSQP-provided free() wrapper:
    c_free(data_.P);
    c_free(data_.A);
#endif
  }

  template<class SMatrixT, class SMatrixT2>
  int setup(const SMatrixT& _P, const Eigen::VectorXd _q, const SMatrixT2& _A, const Eigen::VectorXd& _lower_bnd, const Eigen::VectorXd& _upper_bnd)
  {
    DEB_enter_func;
    // geta data of upper triangular part
    std::vector<Eigen::Triplet<double> > trip;
    for (int k=0; k<_P.outerSize(); ++k)
      for (typename SMatrixT::InnerIterator it(_P,k); it; ++it)
        if(it.row()<=it.col())
          trip.push_back( Eigen::Triplet<double>(it.row(),it.col(),it.value()));

    P_upper_.resize(_P.rows(),_P.cols());
    P_upper_.setFromTriplets(trip.begin(), trip.end());
    P_upper_.makeCompressed();

    std::cerr << "#nonzeros = " << P_upper_.nonZeros() << std::endl;

    q_.resize(_q.size());
    for(long i=0; i<_q.size(); ++i)
      q_[i] = _q[i];

    A_ = _A;
    A_.makeCompressed();

    data_.n = P_upper_.cols(); // number of variables n
    data_.m = A_.rows();       // number of constraints m

    lower_bnd_.resize(_lower_bnd.size());
    upper_bnd_.resize(_upper_bnd.size());
    for(long i=0; i<_lower_bnd.size(); ++i)
    {
      lower_bnd_[i] = _lower_bnd[i];
      upper_bnd_[i] = _upper_bnd[i];
    }

    osqp_cleanup(work_); work_ = nullptr;

#if !COMISO_OSQP_NEW_API
    if(data_.P != nullptr)
      c_free(data_.P);
    if(data_.A != nullptr)
      c_free(data_.A);
#endif

    data_.P = create_osqp_csc(P_upper_, P_v, P_i, P_c);
    data_.A = create_osqp_csc(A_, A_v, A_i, A_c);
    data_.q = q_.data(); // dense array for linear part of cost function (size n)
    data_.l = lower_bnd_.data(); // dense array for lower bound (size m)
    data_.u = upper_bnd_.data(); // dense array for upper bound (size m)

#if COMISO_OSQP_NEW_API
    auto exitflag = osqp_setup(&work_,
            data_.P.get(),
            data_.q, data_.A.get(), data_.l, data_.u, data_.m, data_.n,
            &settings_); // Setup workspace
#else
    auto exitflag = osqp_setup(&work_, &data_, &settings_); // Setup workspace
#endif
//    DEB_error_if( (exitflag != 0), ("OSQP Setup failed with exit flag " << int(exitflag)) );
//    COMISO_THROW_if(exitflag != 0, QP_INITIALIZATION_FAILED);
    return int(exitflag);
  }

  int solve()
  {
    if(work_ == nullptr)
    {
      std::cerr << "Warning: OSQPEigen::solve was called before OSQPEigen::setup ---> abort" << std::endl;
      return -1;
    }

    // Solve Problem
    std::cerr << "SOLVE WITH OSQP" << std::endl;
    auto exitflag = osqp_solve(work_);
//    DEB_error_if( (exitflag != 0), ("OSQP Setup failed with exit flag " << int(exitflag)) );
//    COMISO_THROW_if(exitflag != 0, QP_OPTIMIZATION_FAILED);
    std::cerr << "OSQP exit flag = " << int(exitflag) << std::endl;
    std::cerr << "OSQP status_val = " << work_->info->status_val << std::endl;
#if 0
    for(int i=0; i<100; ++i)
      std::cerr << "x[" << i << "] = " << get_x()[i] << std::endl;
#endif
    return int(exitflag);
  }

  double* get_x() const
  {
     return work_->solution->x;
  }

  double* get_y() const
  {
     return work_->solution->y;
  }

  void get_x( double* _x) const
  {
    const auto &x = get_x();
    for(unsigned int i=0; i<q_.size(); ++i)
      _x[i] = x[i];
  }

  void get_y( double* _y) const
  {
    const auto &y = get_x();
    for(unsigned int i=0; i<A_.rows(); ++i)
      _y[i] = y[i];
  }

  double objective_value() const { return work_->info->obj_val; }
  long long status() const { return work_->info->status_val; }

  template<class SMatrixT>
  int update_objective(const SMatrixT& _P, const Eigen::VectorXd& _q)
  {
    for(long i=0; i<_q.size(); ++i)
      q_[i] = _q[i];
#if COMISO_OSQP_NEW_API
    osqp_update_data_vec(work_, q_.data(), nullptr, nullptr);
#else
    osqp_update_lin_cost(work_, q_.data()); // dense array for linear part of cost function (size n)
#endif

    // geta data of upper triangular part
    std::vector<Eigen::Triplet<double> > trip;
    for (int k=0; k<_P.outerSize(); ++k)
      for (typename SMatrixT::InnerIterator it(_P,k); it; ++it)
        if(it.row()<=it.col())
          trip.push_back( Eigen::Triplet<double>(it.row(),it.col(),it.value()));

    P_upper_.resize(_P.rows(),_P.cols());
    P_upper_.setFromTriplets(trip.begin(), trip.end());
    P_upper_.makeCompressed();

    std::cerr << "#nonzeros = " << P_upper_.nonZeros() << std::endl;

    for(c_int i=0; i<P_upper_.nonZeros(); ++i)
      P_v[i] = P_upper_.valuePtr()[i];

#if COMISO_OSQP_NEW_API
    return osqp_update_data_mat(work_, P_v.data(), nullptr, 0,
            nullptr, nullptr, 0);
#else
    return osqp_update_P(work_, P_v.data(), nullptr, 0); // update all values
#endif
  }

  int update_bounds(const Eigen::VectorXd& _lower_bnd, const Eigen::VectorXd& _upper_bnd)
  {
    for(long i=0; i<_lower_bnd.size(); ++i)
    {
      lower_bnd_[i] = _lower_bnd[i];
      upper_bnd_[i] = _upper_bnd[i];
    }

#if COMISO_OSQP_NEW_API
    return osqp_update_data_vec(work_, nullptr, lower_bnd_.data(), upper_bnd_.data());
#else
    return osqp_update_bounds(work_, lower_bnd_.data(), upper_bnd_.data());
#endif
  }

#if COMISO_OSQP_NEW_API
    std::unique_ptr<OSQPCscMatrix>
#else
  csc* 
#endif
      
      create_osqp_csc(const Eigen::SparseMatrix<double,Eigen::ColMajor>& _A, std::vector<c_float>& _A_v, std::vector<c_int>& _A_i, std::vector<c_int>& _A_c)
  {
    c_int    nnz  = _A.nonZeros(); // number of non zeros

    _A_v.resize(nnz);
    _A_i.resize(nnz);
    _A_c.resize(_A.outerSize()+1);

    for(c_int i=0; i<nnz; ++i)
    {
      _A_v[i] = _A.valuePtr()[i];
      _A_i[i] = _A.innerIndexPtr()[i];
    }
    for(c_int i=0; i<_A.outerSize()+1; ++i)
      _A_c[i] =  _A.outerIndexPtr()[i];

#if COMISO_OSQP_NEW_API
    auto m = std::make_unique<OSQPCscMatrix>();
    m->m = _A.rows();
    m->n = _A.cols();
    m->p = _A_c.data();
    m->i = _A_i.data();
    m->x = _A_v.data();
    m->nzmax = nnz;
    m->nz = -1; // csc
    return m;
#else
    return csc_matrix(_A.rows(), _A.cols(), nnz, _A_v.data(), _A_i.data(), _A_c.data());
#endif
  }

  OSQPSettings& settings() {return settings_;}

private:
  OSQPSettings   settings_;
  OSQPData       data_;
#if COMISO_OSQP_NEW_API
  ::OSQPSolver     *work_ = nullptr;
#else
  OSQPWorkspace* work_ = nullptr;
#endif

  // QP data
  Eigen::SparseMatrix<double,Eigen::ColMajor> P_upper_;
  Eigen::SparseMatrix<double,Eigen::ColMajor> A_;
  std::vector<c_float>                        q_;
  std::vector<c_float>                        lower_bnd_;
  std::vector<c_float>                        upper_bnd_;
  std::vector<c_float>                        P_v;
  std::vector<c_int>                          P_i;
  std::vector<c_int>                          P_c;
  std::vector<c_float>                        A_v;
  std::vector<c_int>                          A_i;
  std::vector<c_int>                          A_c;
};

//=============================================================================
} // namespace COMISO

//=============================================================================
#endif // COMISO_OSQP_AVAILABLE
//=============================================================================
