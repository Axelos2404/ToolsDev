//=============================================================================
//
//  CLASS NewtonSolver
//
//=============================================================================


#ifndef COMISO_NEWTONSOLVER_HH
#define COMISO_NEWTONSOLVER_HH

//== COMPILE-TIME PACKAGE REQUIREMENTS ========================================
#include <CoMISo/Config/config.hh>

//== INCLUDES =================================================================

#include <CoMISo/Config/CoMISoDefines.hh>
#include <CoMISo/Utils/StopWatch.hh>
#include <CoMISo/NSolver/NProblemInterface.hh>
#include <CoMISo/NSolver/NProblemGmmInterface.hh>
#include <CoMISo/NSolver/LinearConstraint.hh>
#include <CoMISo/NSolver/LinearConstraintConverter.hh>

#if COMISO_OSQP_AVAILABLE
  #include <CoMISo/NSolver/OSQPEigen.hh>
#endif

//#include <Base/Debug/DebTime.hh>

#if COMISO_SUITESPARSE_UMFPACK_AVAILABLE
#  include <Eigen/UmfPackSupport>
#endif

#if COMISO_SUITESPARSE_CHOLMOD_AVAILABLE
#  include <Eigen/CholmodSupport>
#endif

#if COMISO_SUITESPARSE_SPQR_AVAILABLE && EIGEN_VERSION_AT_LEAST(3,4,90)
#  include <Eigen/SPQRSupport>
#endif

// ToDo: why is Metis not working yet?
//#if COMISO_METIS_AVAILABLE
//  #include <Eigen/MetisSupport>
//#endif

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace COMISO {

//== CLASS DEFINITION =========================================================

	      

/** \class NewtonSolver NewtonSolver.hh <COMISO/.../NewtonSolver.hh>

    Brief Description.
  
    A more elaborate description follows.
*/
class COMISODLLEXPORT NewtonSolver
{
public:

  /// Supported linear solvers for KKT system
  // Warning: LS_EigenCG is only suitable for unconstrained problems
  enum LinearSolver {LS_EigenLU, LS_Umfpack, LS_SPQR, LS_MUMPS, LS_EigenCG, LS_EigenBiCGSTAB, LS_OSQP_DIRECT, LS_OSQP_MKL, LS_OSQP_CG};

  typedef Eigen::VectorXd             VectorD;
  typedef Eigen::SparseMatrix<double> SMatrixD;
  typedef Eigen::Triplet<double>      Triplet;

  /// Default constructor
  NewtonSolver(const double _eps = 1e-6, const double _eps_line_search = 1e-9,
      const int _max_iters = 200, const double _alpha_ls = 0.2,
      const double _beta_ls = 0.6)
      : eps_(_eps), eps_ls_(_eps_line_search), max_iters_(_max_iters),
        alpha_ls_(_alpha_ls), beta_ls_(_beta_ls), max_feasible_step_safety_factor_(0.5),
        rho_merit_(0.5), mu_merit_(0.0),
        prescribed_constraint_regularization_absolute_(0.0), prescribed_hessian_regularization_relative_(0.0),
        use_trust_region_regularization_(true),
        solver_type_(LS_EigenLU), constant_hessian_structure_(false)
  {
#if COMISO_SUITESPARSE_UMFPACK_AVAILABLE
    solver_type_ = LS_Umfpack;
#endif
  }

  // solve without linear constraints
  int solve(NProblemInterface* _problem)
  {
    SMatrixD A(0,_problem->n_unknowns());
    VectorD b(VectorD::Index(0));
    return solve(_problem, A, b);
  }

  // solve with linear constraints and feasible starting point
  int solve(NProblemInterface* _problem, const SMatrixD& _A, const VectorD& _b);

  int solve(NProblemInterface* _problem, std::vector<LinearConstraint>& _constraints)
  {
    // convert constraints
    SMatrixD A;
    VectorD b;
    LinearConstraintConverter::nsolver_to_eigen(_constraints, A, b);
    return solve(_problem, A, b);
  }

  // solve with linear constraints, where the starting point doesn't need to be feasible
  // it is recommended to provide linearly independent constraints but it is not necessary
  // by default the algorithm uses a trust-region regularization based on line-search step length
  // at the moment this function is experimental but it will probably replace the solve(...) function in the near future
  int solve_infeasible_start(NProblemInterface* _problem, const SMatrixD& _A, const VectorD& _b);

  int solve_infeasible_start(NProblemInterface* _problem, std::vector<LinearConstraint>& _constraints)
  {
    // convert constraints
    SMatrixD A;
    VectorD b;
    LinearConstraintConverter::nsolver_to_eigen(_constraints, A, b);
    return solve_infeasible_start(_problem, A, b);
  }

  int solve_infeasible_start(NProblemInterface* _problem, std::vector<NConstraintInterface*>& _constraints)
  {
    // convert constraints
    SMatrixD A;
    VectorD b;
    LinearConstraintConverter::nsolver_to_eigen(_constraints, A, b);
    return solve_infeasible_start(_problem, A, b);
  }


  // select internal linear solver
  void set_linearsolver(LinearSolver _st)
  {
    solver_type_ = _st;
  }

  void set_prescribed_constraint_regularization_absolute(const double _pcr)
  {
    prescribed_constraint_regularization_absolute_ = _pcr;
  }

  void set_prescribed_hessian_regularization_relative(const double _phr)
  {
    prescribed_hessian_regularization_relative_ = _phr;
  }

  void use_trust_region_regularization( const bool _enable)
  {
    use_trust_region_regularization_ = _enable;
  }

  bool converged() { return converged_; }

  int iters() { return iters_; }

  double& max_feasible_step_safety_factor() { return max_feasible_step_safety_factor_;}

  std::string name_of( const LinearSolver _ls)
  {
    switch(_ls)
    {
      case LS_EigenLU: return "EigenLU";
      case LS_Umfpack: return "Umfpack";
      case LS_SPQR:    return "SPQR";
      case LS_MUMPS:   return "MUMPS";
      case LS_EigenCG: return "EigenCG";
      case LS_EigenBiCGSTAB: return "EigenBiCGSTAB";
      case LS_OSQP_DIRECT: return "OSQP_DIRECT";
      case LS_OSQP_MKL: return "OSQP_MKL";
      case LS_OSQP_CG: return "OSQP_CG";
      default: return "UNKNOWN";
    }
  }

protected:

  bool factorize(NProblemInterface* _problem, const SMatrixD& _A,
    const VectorD& _b, const VectorD& _x, double& _regularize_hessian,
    double& _regularize_constraints, const bool _first_factorization);

  // novel varient of above for infeasible solver, where Hessian is provided
  bool factorize(const SMatrixD& _H, const VectorD& _g, const SMatrixD& _A, const VectorD& _b,
                 const double _regularize_hessian_relative, const double _regularize_constraints_absolute, const bool _update_pattern);

  double backtracking_line_search(NProblemInterface* _problem, VectorD& _x, 
    VectorD& _g, VectorD& _dx, double& _newton_decrement, double& _fx, 
    const double _t_start = 1.0);

  double backtracking_line_search_infeasible(NProblemInterface* _problem, const SMatrixD& _A, const VectorD& _b,
            VectorD& _x, VectorD& _nue, VectorD& _g, VectorD& _dz, double& _fx, double& _res_primal2, double& _res_dual2,
            const double _t_start = 1.0, const int _max_ls_iters = 20);

  double backtracking_line_search_infeasible_merit_l1(NProblemInterface* _problem, const SMatrixD& _H,
                                                      const SMatrixD& _A, const VectorD& _b,
                                                      const VectorD& _x, const VectorD& _g, VectorD& _dz, double& _fx,
                                                      const double _t_start=1.0, const int _max_ls_iters=20);



  void analyze_pattern(SMatrixD& _KKT);

  bool numerical_factorization(SMatrixD& _KKT);

  void solve_kkt_system(const VectorD& _rhs, VectorD& _dx);

  // Warning: this function is not working properly yet because Eigen:LLT sometimes succeeds for matrices that are not positive definite!!!
  // return scalar s for which (_A + s*Id) is positive definite
  double determine_spd_regularization(const SMatrixD& _A, const double _r0 = 1e-6, const double _rscale = 10.0, const int _max_iters = 10);

#if COMISO_GMM_AVAILABLE
  // deprecated function!
  // solve
  int solve(NProblemGmmInterface* _problem);

  // deprecated function!
  // solve specifying parameters
  int solve(NProblemGmmInterface* _problem, int _max_iter, double _eps)
  {
    max_iters_ = _max_iter;
    eps_ = _eps;
    return solve(_problem);
  }
#endif // COMISO_GMM_AVAILABLE

  // deprecated function!
  bool& constant_hessian_structure() { return constant_hessian_structure_; }

#if COMISO_OSQP_AVAILABLE
  // osqp helpers
  int setup_osqp(const SMatrixD& _H, const VectorD& _g, const SMatrixD& _A, const VectorD& _b);
  int update_osqp(const SMatrixD& _H, const VectorD& _g, const VectorD& _b);
  void solve_kkt_system_osqp(VectorD& _dx);
#endif

protected:
  double* P(std::vector<double>& _v)
  {
    if( !_v.empty())
      return ((double*)&_v[0]);
    else
      return 0;
  }

private:

  double eps_;
  double eps_ls_;
  int    max_iters_;
  double alpha_ls_;
  double beta_ls_;

  double max_feasible_step_safety_factor_;
  
  double rho_merit_;
  double mu_merit_;

  double prescribed_constraint_regularization_absolute_;
  double prescribed_hessian_regularization_relative_;

  bool use_trust_region_regularization_;

  VectorD x_ls_;
  VectorD nue_ls_;
  VectorD g_ls_;

  // permanent storage for triplets to avoid re-allocation
  std::vector< Triplet > trips_;

  LinearSolver solver_type_;

  // cache KKT Matrix
  SMatrixD KKT_;

  // Sparse LU decomposition
  Eigen::SparseLU<SMatrixD> lu_solver_;

  // CG solver
  Eigen::ConjugateGradient<SMatrixD, Eigen::Lower|Eigen::Upper> cg_solver_;

  // BiCGSTAB solver
  Eigen::BiCGSTAB<SMatrixD> bicgstab_solver_;

#if COMISO_SUITESPARSE_UMFPACK_AVAILABLE
  Eigen::UmfPackLU<SMatrixD> umfpack_solver_;
#endif
#if COMISO_SUITESPARSE_SPQR_AVAILABLE && COMISO_SUITESPARSE_CHOLMOD_AVAILABLE && EIGEN_VERSION_AT_LEAST(3,4,90)
  Eigen::SPQR<SMatrixD>      spqr_solver_;
#endif

#if COMISO_OSQP_AVAILABLE
  OSQPEigen osqp_eigen_;
#endif

  // deprecated
  bool   constant_hessian_structure_;

  bool converged_;

  int iters_;
};


//=============================================================================
} // namespace COMISO
//=============================================================================
#endif // COMISO_NEWTONSOLVER_HH defined
//=============================================================================

