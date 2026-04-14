//=============================================================================
//
//  CLASS FINITEELEMENTHESSIANPROJECTION
//
//=============================================================================


#ifndef COMISO_FINITEELEMENTHESSIANPROJECTION_HH
#define COMISO_FINITEELEMENTHESSIANPROJECTION_HH

//== INCLUDES =================================================================

#include <CoMISo/Config/CoMISoDefines.hh>

#include<Eigen/Dense>
#include<Eigen/Sparse>
#include<cassert>


//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace COMISO { 

//== CLASS DEFINITION =========================================================

	      
/** \class FINITEELEMENTHESSIANPROJECTION

    Implements forward mode algorithmic differentiation (first and second order) for FiniteElements via TinyAD
  
    A more elaborate description follows.
*/

template<class ElementT>
class FiniteElementHessianProjection : public ElementT
{
public:

  // import dimensions
  const static int NV = ElementT::NV;
  const static int NC = ElementT::NC;

  using VecI = Eigen::Matrix<size_t,NV,1>;
  using VecV = Eigen::Matrix<double,NV,1>;
  using VecC = Eigen::Matrix<double,NC,1>;
  using Triplet = Eigen::Triplet<double>;

  inline double eval_f(const VecV& _x, const VecC& _c) const
  {
    return ElementT::eval_f(_x,_c);
  }

  inline void   eval_gradient(const VecV& _x, const VecC& _c, VecV& _g) const
  {
    return ElementT::eval_gradient(_x,_c,_g);
  }

  inline void   eval_hessian (const VecV& _x, const VecC& _c, std::vector<Triplet>& _triplets) const
  {
    ElementT::eval_hessian(_x,_c,_triplets);

    // convert to dense matrix
    Eigen::Matrix<double,NV,NV> H;
    H.resize(_x.size(),_x.size());
    H.setZero();
    for(auto t : _triplets)
      H(t.row(), t.col()) += t.value();

    // make s.p.d. if necessary
    if(!weakly_diagonal_dominant(H))
    {
      project_to_positive_semidefinite(H);

      _triplets.clear();
      for (Eigen::Index i = 0; i < H.rows(); ++i)
        for (Eigen::Index j = 0; j < H.cols(); ++j)
          _triplets.push_back(Triplet(i, j, H(i, j)));
    }
  }

  inline double max_feasible_step(const VecV& _x, const VecV& _v, const VecC& _c)
  {
    return ElementT::max_feasible_step(_x, _v, _c);
  }

  template <class MatT>
  inline bool weakly_diagonal_dominant(const MatT& _A) const
  {
    // only works for square matrices
    assert(_A.cols() == _A.rows());

    for(unsigned int i=0; i<_A.rows(); ++i)
    {
      double asum(0.0);
      for(unsigned int j=0; j<_A.cols(); ++j)
        asum += std::abs(_A(i,j));

      double d = _A(i,i);

      if( 2*d < asum)
        return false;
    }

    return true;
  }

  template <class MatT>
  inline void project_to_positive_semidefinite( MatT& _A ) const
  {
    typename Eigen::SelfAdjointEigenSolver<MatT> es(_A);

    typename Eigen::SelfAdjointEigenSolver<MatT>::RealVectorType ev = es.eigenvalues();
    for (unsigned int i = 0; i < ev.size(); ++i)
    {
      ev[i] = std::max(ev[i], 0.0);
    }

    _A = es.eigenvectors() * ev.asDiagonal() * es.eigenvectors().transpose();
  }
};


template<class ElementT>
class FiniteElementHessianProjectionIdentity : public ElementT
{
public:

  // import dimensions
  const static int NV = ElementT::NV;
  const static int NC = ElementT::NC;

  using VecI = Eigen::Matrix<size_t,NV,1>;
  using VecV = Eigen::Matrix<double,NV,1>;
  using VecC = Eigen::Matrix<double,NC,1>;
  using Triplet = Eigen::Triplet<double>;

  inline double eval_f(const VecV& _x, const VecC& _c) const
  {
    return ElementT::eval_f(_x,_c);
  }

  inline void   eval_gradient(const VecV& _x, const VecC& _c, VecV& _g) const
  {
    return ElementT::eval_gradient(_x,_c,_g);
  }

  inline void   eval_hessian (const VecV& _x, const VecC& _c, std::vector<Triplet>& _triplets) const
  {
    ElementT::eval_hessian(_x,_c,_triplets);

    // convert to dense matrix
    Eigen::Matrix<double,NV,NV> H;
    H.resize(_x.size(),_x.size());
    H.setZero();
    for(auto t : _triplets)
      H(t.row(), t.col()) += t.value();

    // make s.p.d. if necessary
    if(!weakly_diagonal_dominant(H))
    {
      project_to_positive_semidefinite(H);

      _triplets.clear();
      for (Eigen::Index i = 0; i < H.rows(); ++i)
        for (Eigen::Index j = 0; j < H.cols(); ++j)
          _triplets.push_back(Triplet(i, j, H(i, j)));
    }
  }

  inline double max_feasible_step(const VecV& _x, const VecV& _v, const VecC& _c)
  {
    return ElementT::max_feasible_step(_x, _v, _c);
  }

  template <class MatT>
  inline bool weakly_diagonal_dominant(const MatT& _A) const
  {
    // only works for square matrices
    assert(_A.cols() == _A.rows());

    for(unsigned int i=0; i<_A.rows(); ++i)
    {
      double asum(0.0);
      for(unsigned int j=0; j<_A.cols(); ++j)
        asum += std::abs(_A(i,j));

      double d = _A(i,i);

      if( 2*d < asum)
        return false;
    }

    return true;
  }

  template <class MatT>
  inline void project_to_positive_semidefinite( MatT& _A ) const
  {
    typename Eigen::SelfAdjointEigenSolver<MatT> es(_A);

    typename Eigen::SelfAdjointEigenSolver<MatT>::RealVectorType ev = es.eigenvalues();
    double min_eval = std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < ev.size(); ++i)
      if(ev[i] < min_eval)
        min_eval = ev[i];

    // shif all eigenvalues by -min_eval
    if(min_eval < 0.0)
      _A -= min_eval*MatT::Identity();
  }
};

//=============================================================================
} // namespace COMISO
//=============================================================================
#endif // COMISO_FINITEELEMENTHESSIANPROJECTION_HH defined
//=============================================================================

