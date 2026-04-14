//=============================================================================
//
//                               OpenFlipper
//        Copyright (C) 2008 by Computer Graphics Group, RWTH Aachen
//                           www.openflipper.org
//
//-----------------------------------------------------------------------------
//
//                                License
//
//  OpenFlipper is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  OpenFlipper is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with OpenFlipper.  If not, see <http://www.gnu.org/licenses/>.
//
//-----------------------------------------------------------------------------
//
//   $Author: David Bommes $
//   $Date: 2021-06-21 $
//
//=============================================================================


//=============================================================================
//
//  CLASS Polynomials
//
//=============================================================================

#ifndef COMISO_POLYNOMIALS_HH_INCLUDED
#define COMISO_POLYNOMIALS_HH_INCLUDED
//=============================================================================

#include <Eigen/Dense>
#include <Base/Debug/DebOut.hh>
#include <cassert>


namespace COMISO
{

template<int DIMENSION>
class Monomial
{
public:
  using Coefficients  = Eigen::Matrix<double,DIMENSION+1,1>;
  using Index = Eigen::Index;

  // --------- non-static members

  Monomial() {}
  Monomial(const Coefficients& _c) : c_(_c) {}

  inline double eval(const double _t) const
  {
    Index n = DIMENSION;
    double val = c_[n];
    // horner scheme
    for(Index i=1; i<c_.innerSize(); ++i)
      val = val*_t+coeff(n-i);

    return val;
  }

  inline       double& coeff(Index _i)       { return c_[_i];}
  inline const double& coeff(Index _i) const { return c_[_i];}

  Coefficients& coeffs() {return c_;}

  inline Index degree() const
  {
    return DIMENSION;
  }

  // --------- static members
  static double eval(const Coefficients& _c, const double _t)
  {
    Index n = DIMENSION;
    double val = _c[n];
    // horner scheme
    for(Index i=1; i<_c.innerSize(); ++i)
      val = val*_t+_c(n-i);

    return val;
  }

private:
  Coefficients c_;
};


template<int DIMENSION>
class BezierCurve
{
public:
  using Coefficients  = Eigen::Matrix<double,DIMENSION+1,1>;
  using Index = Eigen::Index;

  // --------- non-static members

  BezierCurve() {}
  BezierCurve(const Coefficients& _c) : c_(_c) {}
  BezierCurve(const Monomial<DIMENSION>& _m, const double _s_start, const double _s_end)
  {
    convert_monomial_to_bezier(_m, _s_start, _s_end, c_);
  }

  inline       double& coeff(Index _i)       { return c_[_i];}
  inline const double& coeff(Index _i) const { return c_[_i];}

  Coefficients& coeffs() {return c_;}

  inline Index degree() const
  {
    return DIMENSION;
  }


  inline double eval(const double _t) const
  {
    Index  n = DIMENSION;
    int    n_choose_i = 1;

    double s    = 1.0-_t;
    double fact = 1.0;
    double val  = c_[0]*s;

    // horner scheme
    for(Index i=1; i<n; ++i)
    {
      fact = fact*_t;
      n_choose_i=n_choose_i*(n-i+1)/i;  // always int!
      val=(val + fact*n_choose_i*c_(i))*s;
    }
    val = val + fact*_t*c_(n);

    return val;
  }

  inline double eval_from_basis(const double _t) const
  {
    double val = 0.0;
    for(Index i=0; i<c_.innerSize(); ++i)
      val += c_[i]*bernstein_polynomial(DIMENSION,i,_t);
    return val;
  }

  void subdivide(const double _t, BezierCurve& _b0, BezierCurve& _b1)
  {
    // de Casteljau tableau (only use half of matrix)
    Eigen::Matrix<double, DIMENSION+1,DIMENSION+1> B;

    double t1 = 1.0-_t;

    // initialize i=0
    B.row(0) = c_;

    for(Index i=1; i<=DIMENSION; ++i)
      for(Index j=0; j<=DIMENSION-i; ++j)
        B(i,j) = t1*B(i-1,j)+_t*B(i-1,j+1);

    for(Index i=0; i<=DIMENSION; ++i)
    {
      _b0.coeff(i) = B(i, 0);
      _b1.coeff(i) = B(DIMENSION-i,i);
    }
  }

  bool first_root_in_interval(double& _t, const double _eps=1e-6)
  {
    if(!first_root_of_control_polygon(_t))
      return false;
    else
    {
      if( _eps > 1.0)
        return true;
      else
      {
        // accuracy not sufficient ---> subdivide
        double t_sub = 0.5;
        if(_t < 0.2)
          t_sub = 2.0*_t;
        else if(_t > 0.8)
          t_sub = 2.0*_t-1.0;

        // truncate to warrant progress
        if(t_sub < 0.05) t_sub = 0.05;
        if(t_sub > 0.95) t_sub = 0.95;

        BezierCurve b0, b1;
        subdivide(t_sub, b0, b1);

        if (b0.first_root_in_interval(_t, _eps / t_sub))
        {
          _t *= t_sub; // re-parametrize result
          return true;
        }
        else if (b1.first_root_in_interval(_t, _eps / (1.0 - t_sub)))
        {
          _t = t_sub + _t * (1.0 - t_sub);
          return true;
        }
        else
          return false;
      }
    }
  }


  // compute roots within parameter domain [0,1]
  void roots_in_interval(std::vector<double>& _roots, const double _eps=1e-6)
  {
    _roots.clear();
    double t=0.0;
    if(!first_root_of_control_polygon(t))
      return;
    else
    {
      if( _eps > 1.0)
      {
        // return root
        _roots.emplace_back(t);
        return;
      }
      else
      {
        // accuracy not sufficient ---> subdivide
        double t_sub = 0.5;
        if(t < 0.2)
          t_sub = 2.0*t;
        else if(t > 0.8)
          t_sub = 2.0*t-1.0;

        // truncate to warrant progress
        if(t_sub < 0.05) t_sub = 0.05;
        if(t_sub > 0.95) t_sub = 0.95;

        BezierCurve b0, b1;
        subdivide(t_sub, b0, b1);

        std::vector<double> roots0;
        b0.roots_in_interval(roots0, _eps / t_sub);
        for(const auto& r : roots0)
        {
          // re-parametrize result
          double rp = r * t_sub;
          // add to output if sufficiently different from previous one
          if(_roots.empty() || std::abs(_roots.front()-rp) > _eps)
            _roots.emplace_back(rp);
        }

        std::vector<double> roots1;
        b1.roots_in_interval(roots1, _eps / (1.0 - t_sub));
        for(const auto& r : roots1)
        {
          // re-parametrize result
          double rp = t_sub + r * (1.0 - t_sub);
          // add to output if sufficiently different from previous one
          if(_roots.empty() || std::abs(_roots.front()-rp) > _eps)
            _roots.emplace_back(rp);
        }
      }
    }
  }



  bool first_root_of_control_polygon(double& _t_root)
  {
    DEB_enter_func;
    for(Index i=1; i<=DIMENSION; ++i)
    {
      // check segment [(i-1)/n, i/n]
      // opposite signs or zero?
      double p = c_[i-1]*c_[i];

      // catch numerical problems
      if(!std::isfinite(p))
      {
        DEB_warning(2, "Warning: first_root_of_control_polygon observed numerical issues --- c_[i-1]*c_[i] = " << p);
        return false;
      }

      if( p <= 0.0)
      {
        double t = -c_[i-1]/(c_[i]-c_[i-1]);
        if(!std::isfinite(t)) // catch numerically degenerate cases
          t = 0.5;
        if(t >= 0.0 && t <= 1.0)
        {
          _t_root = 1.0 / double(DIMENSION) * (double(i - 1) + t);
          return true;
        }
      }
    }
    return false;
  }

  // --------- static members
  static double eval(const Coefficients& _c, const double _t)
  {
    Index n = DIMENSION;
    int n_choose_i = 1;

    double s = 1.0 - _t;
    double fact = 1.0;
    double val = _c[0] * s;

    // horner scheme
    for (Index i = 1; i < n; ++i) {
      fact = fact * _t;
      n_choose_i = n_choose_i * (n - i + 1) / i;  // always int!
      val = (val + fact * n_choose_i * _c(i)) * s;
    }
    val = val + fact * _t * _c(n);

    return val;
  }

  static void convert_monomial_to_bezier(const Monomial<DIMENSION>& _m, const double _t_start, const double _t_end, Coefficients& _b)
  {
    // solve interpolation system
    Eigen::Matrix<double, DIMENSION+1,DIMENSION+1> A;
    Eigen::Matrix<double, DIMENSION+1,1>           rhs;

    for(Index i=0; i<DIMENSION+1; ++i)
    {
      double s = double(i)/double(DIMENSION);
      double t = (1.0-s)*_t_start + s*_t_end;

      rhs(i) = _m.eval(t);
      for(Index j=0; j<DIMENSION+1; ++j)
        A(i,j) = bernstein_polynomial(DIMENSION, j, s);
    }

    Eigen::Matrix<double, DIMENSION+1,DIMENSION+1> Ai = A.inverse();  //ToDo inverse should be cached!!!

    _b = Ai*rhs;
  }

  static double bernstein_polynomial(const int _n, const int _i, const double _t)
  {
    assert(_n > 0 && _i >=0 && _i <= _n);
    return binomial(_n, _i)*std::pow(_t,_i)*std::pow(1.0-_t,_n-_i);
  }

  static double binomial(const int _n, const int _k)
  {
    if(_k == 0 || _k == _n)
      return 1.0;

    int a=1, b=1;
    for(int i=1; i<=_k; ++i)
      a *= i;
    for(int i=_n; i>_n-_k; --i)
      b *= i;

    return double(b)/double(a);
  }

private:
  Coefficients c_;
};




class Polynomials
{
public:

  using Vec4d = Eigen::Matrix<double,4,1>;


  // robustly find first roots of f(x) = _a0 + _a1*x + _a2*x^2 + _a3*x^2 in interval [_t_start,_t_end]
  // return true if a root was found in the interval, or false if there is no root in this interval
  // the estimated root is returned as _t with accuracy |_t-t^*| < _eps
  template<int DIMENSION>
  static bool first_root_in_interval(const Monomial<DIMENSION>& _m, const double _t_start, const double _t_end, double& _t, const double _eps=1e-6)
  {
    BezierCurve<DIMENSION> b(_m, _t_start, _t_end);
    if( b.first_root_in_interval( _t, _eps))
    {
      // transform parameter
      _t = (1.0-_t)*_t_start + _t*_t_end;
      return true;
    }
    else
      return false;
  }

  // robustly find roots of f(x) = _a0 + _a1*x + _a2*x^2 + _a3*x^2 + ... in interval [_t_start,_t_end]
  // the estimated roots are computed with accuracy |_t-t^*| < _eps
  // roots t1, t2 with |t1-t2| <= 2*_eps might not be distinguished

  template<int DIMENSION>
  static void roots_in_interval(const Monomial<DIMENSION>& _m, const double _t_start, const double _t_end, std::vector<double>& _roots, const double _eps=1e-6)
  {
    BezierCurve<DIMENSION> b(_m, _t_start, _t_end);
    std::vector<double> b_roots;
    b.roots_in_interval( b_roots, _eps);

    // clear old data and compute result
    _roots.clear();
    for(const auto& r : b_roots)
      // re-parametrize roots
      _roots.emplace_back((1.0-r)*_t_start + r*_t_end);
  }

  // robustly find **positive** roots of f(x) = _a x^2 + _b x + _c
  // return {-1, 0, 1, 2} which corresponds to number of roots, or -1 which means infinitely many roots for a=b=c=0
  // the potential roots are returned as _x0 and _x1 with _x0 < _x1
  static int positive_roots_of_quadratic(const double _a, const double _b, const double _c, double& _x0, double& _x1, const double _eps=1e-12)
  {
    // get all roots
    int n = roots_of_quadratic(_a, _b, _c, _x0, _x1, _eps);

    // remove negative ones
    if (n == 2 && _x0 < 0.0) {
      std::swap(_x0, _x1);
      --n;
    }

    // remove negative ones
    if (n == 1 && _x0 < 0.0)
      --n;

    return n;
  }

    // robustly find roots of f(x) = _a x^2 + _b x + _c
  // return {-1, 0, 1, 2} which corresponds to number of roots, or -1 which means infinitely many roots for a=b=c=0
  // the potential roots are returned as _x0 and _x1 with _x0 < _x1
  static int roots_of_quadratic(const double _a, const double _b, const double _c, double& _x0, double& _x1, const double _eps=1e-12)
  {
    double abs_max = std::max(std::max(std::abs(_a), std::abs(_b)), std::abs(_c));

    // infinitely many roots?
    // a==b==c==0
    if(abs_max < _eps)
      return -1;
    else
    {
      double a = _a/abs_max;
      double b = _b/abs_max;
      double c = _c/abs_max;

      if( std::abs(a) < _eps)
        // a==0
        return roots_of_linear(_b, _c, _x0, _eps);
      else
      {
        // no constant part?
        if(std::abs(c) < _eps)
        {
          if(std::abs(b) < _eps)
          {
            // a!=0, b==0, c==0
            _x0 = 0.0;
            return 1;
          }
          else
            {
              // a!=0, b!=0, c==0
              _x0 = 0.0;
              _x1 = -_b / _a;

              if( _x0 > _x1)
                std::swap(_x0,_x1);

              return 2;
            }
        }
        else // a!=0 && c!=0
        {
          // calculate discriminant
          double d = _b*_b - 4.0*_a*_c;

          if( d < -_eps*_eps) // d<-eps
            return 0;
          else
            if( d<0.0) // -eps < d < 0
            {
              _x0 = -0.5*_b/_a;
              return 1;
            }
            else
            {
              double sd = std::sqrt(d);
              _x0 = 0.5*(-_b + sd)/_a;
              _x1 = 0.5*(-_b - sd)/_a;

              if(_x0 > _x1) std::swap(_x0,_x1);
              return 2;
            }
        }
      }
    }
  }

  // robustly find roots of f(x) = _a x + _b
  // return {-1, 0, 1} which corresponds to number of roots, or -1 which means infinitely many roots for a=b=0
  // the potential roots are returned as _x0 and _x1 with _x0 < _x1
  static int roots_of_linear(const double _a, const double _b, double& _x0, const double _eps=1e-12)
  {
    double abs_max = std::max(std::abs(_a), std::abs(_b));

    // infinitely many roots?
    if(abs_max < _eps)
      return -1;

    if(std::abs(_a/abs_max) < _eps)
      return 0;
    else
    {
      _x0 = -_b/_a;
      return 1;
    }
  }

//  static std::map<int,Eigen::MatrixXd> bezier_interpolation_inverse_;
};

};


#endif // COMISO_POLYNOMIALS_HH_INCLUDED
//=============================================================================
