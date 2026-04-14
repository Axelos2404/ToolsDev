//=============================================================================
//
//  CLASS FINITEELEMENTTINYAD
//
//=============================================================================


#ifndef COMISO_FINITEELEMENTTINYAD_HH
#define COMISO_FINITEELEMENTTINYAD_HH

//== COMPILE-TIME PACKAGE REQUIREMENTS ========================================
#include <CoMISo/Config/config.hh>
#if COMISO_TINYAD_AVAILABLE

//== INCLUDES =================================================================

#include <CoMISo/Config/CoMISoDefines.hh>

#include<Eigen/Dense>
#include<Eigen/Sparse>

#include <TinyAD/Scalar.hh>


//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace COMISO { 

//== CLASS DEFINITION =========================================================

	      
/** \class FiniteElementTinyAD

    Implements forward mode algorithmic differentiation (first and second order) for FiniteElements via TinyAD
  
    A more elaborate description follows.
*/

template<class ElementT>
class FiniteElementTinyAD : public ElementT
{
public:

  // import dimensions
  const static int NV = ElementT::NV;
  const static int NC = ElementT::NC;

  using VecI = Eigen::Matrix<size_t,NV,1>;
  using VecV = Eigen::Matrix<double,NV,1>;
  using VecC = Eigen::Matrix<double,NC,1>;
  using Triplet = Eigen::Triplet<double>;

  using TADG = TinyAD::Scalar<NV,double,false>;
  using TADH = TinyAD::Scalar<NV,double,true>;

  inline double eval_f(const VecV& _x, const VecC& _c) const
  {
    return ElementT::eval_f(_x,_c);
  }

  inline void   eval_gradient(const VecV& _x, const VecC& _c, VecV& _g) const
  {
    auto x_ad = TADG::make_active(_x);

    auto f = ElementT::eval_f(x_ad,_c);

    _g = f.grad;
  }

  inline void   eval_hessian (const VecV& _x, const VecC& _c, std::vector<Triplet>& _triplets) const
  {
    auto x_ad = TADH::make_active(_x);

    auto f = ElementT::eval_f(x_ad,_c);

//    auto H = f.Hess.template selfadjointView<Eigen::Lower>();
    auto H = f.Hess;
    _triplets.clear();

    for(unsigned int i=0; i<H.rows(); ++i)
      for(unsigned int j=0; j<H.cols(); ++j)
        _triplets.push_back(Triplet(i,j,H(i,j)));
  }

  inline double max_feasible_step(const VecV& _x, const VecV& _v, const VecC& _c)
  {
    return ElementT::max_feasible_step(_x, _v, _c);
  }
};


//=============================================================================
} // namespace COMISO
//=============================================================================
#endif // COMISO_TINYAD_AVAILABLE
//=============================================================================
#endif // COMISO_FINITEELEMENTTINYAD_HH defined
//=============================================================================

