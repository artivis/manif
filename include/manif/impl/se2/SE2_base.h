#ifndef _MANIF_MANIF_SE2_BASE_H_
#define _MANIF_MANIF_SE2_BASE_H_

#include "manif/impl/se2/SE2_properties.h"
#include "manif/impl/manifold_base.h"
#include "manif/impl/utils.h"

namespace manif
{

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Derived>
struct SE2Base : ManifoldBase<_Derived>
{
private:

  using Base = ManifoldBase<_Derived>;
  using Type = SE2Base<_Derived>;

public:

  MANIF_MANIFOLD_PROPERTIES

  MANIF_MANIFOLD_TYPEDEF

  /// @todo find a mechanism to fetch it from base
  /// just like the other typedefs
  using Translation = typename internal::traits<_Derived>::Translation;

  /// Manifold common API

  Transformation transform() const;
  Rotation rotation() const;
  Translation translation() const;

  SE2Base<_Derived>& setIdentity();

  Manifold inverse(OptJacobianRef J_minv_m = {}) const;
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  template <typename _DerivedOther>
  Manifold compose(const ManifoldBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  Vector act(const Vector &v,
             OptJacobianRef J_vout_m = {},
             OptJacobianRef J_vout_v = {}) const;

  using Base::coeffs;
  using Base::coeffs_nonconst;
  MANIF_INHERIT_MANIFOLD_AUTO_API
  MANIF_INHERIT_MANIFOLD_OPERATOR

  /// SE2 specific functions

  Scalar real() const;
  Scalar imag() const;
  Scalar angle() const;

  Scalar x() const;
  Scalar y() const;
};

template <typename _Derived>
typename SE2Base<_Derived>::Transformation
SE2Base<_Derived>::transform() const
{
  Transformation T(Transformation::Identity());
  T.template block<2,2>(0,0) = rotation();
  T(0,2) = x();
  T(1,2) = y();
  return T;
}

template <typename _Derived>
typename SE2Base<_Derived>::Rotation
SE2Base<_Derived>::rotation() const
{
  return (Rotation() << real(), -imag(),
                        imag(),  real() ).finished();
}

template <typename _Derived>
typename SE2Base<_Derived>::Translation
SE2Base<_Derived>::translation() const
{
  return Translation(x(), y());
}

template <typename _Derived>
SE2Base<_Derived>&
SE2Base<_Derived>::setIdentity()
{
  coeffs_nonconst().setZero();
  coeffs_nonconst()(2) = 1;
  return *this;
}

template <typename _Derived>
typename SE2Base<_Derived>::Manifold
SE2Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  using std::cos;
  using std::sin;

  if (J_minv_m)
  {
    Jacobian adj = Jacobian::Identity();
    adj.template topLeftCorner<2,2>() = rotation();
    adj(0,2) =  y();
    adj(1,2) = -x();
    (*J_minv_m) = -adj;
  }

  return Manifold(-x()*real() - y()*imag(),
                   x()*imag() - y()*real(),
                           -angle()        );
}

template <typename _Derived>
typename SE2Base<_Derived>::Tangent
SE2Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  using std::abs;
  using std::cos;
  using std::sin;

  const Scalar theta     = angle();
  const Scalar cos_theta = cos(theta);
  const Scalar sin_theta = sin(theta);
  const Scalar theta_sq  = theta * theta;

  Scalar A,  // sin_theta_by_theta
         B;  // one_minus_cos_theta_by_theta

  if (abs(theta) < Constants<Scalar>::eps)
  {
    // Taylor approximation
    A = Scalar(1) - Scalar(1. / 6.) * theta_sq;
    B = Scalar(.5) * theta - Scalar(1. / 24.) * theta * theta_sq;
  }
  else
  {
    // Euler
    A = sin_theta / theta;
    B = (Scalar(1) - cos_theta) / theta;
  }

  const Scalar den = Scalar(1) / (A*A + B*B);

  A *= den;
  B *= den;

  Tangent tan( A * x() + B * y(),
              -B * x() + A * y(),
               theta       );

  A /= den;
  B /= den;

  if (J_t_m)
  {
    // Jr^-1
    Jacobian rjac = Jacobian::Identity();
    rjac(0,0) =  A;
    rjac(0,1) =  B;
    rjac(1,0) = -B;
    rjac(1,1) =  A;

    rjac(0,2) = (-tan.y() + theta*tan.x() + tan.y()*cos_theta - tan.x()*sin_theta)/theta_sq;
    rjac(1,2) = ( tan.x() + theta*tan.y() - tan.x()*cos_theta - tan.y()*sin_theta)/theta_sq;

    (*J_t_m) = rjac.inverse();

//    Scalar theta_by_2 = theta / Scalar(2);

//    Jacobian rjacinv = Jacobian::Identity();
//    rjacinv(0,0) =  (theta*sin_theta)/(Scalar(2)-Scalar(2)*cos_theta);
//    rjacinv(0,1) = -theta_by_2;
//    rjacinv(1,0) =  theta_by_2;
//    rjacinv(1,1) =  rjacinv(0,0);

//    rjacinv(0,2) = theta * (theta_by_2*(x()*sin_theta + y()*cos_theta - y()) + cos_theta*x() - x()) / (cos_theta - Scalar(1));
//    rjacinv(1,2) = theta * (theta_by_2*(y()*sin_theta - x()*cos_theta + x()) + cos_theta*y() - y()) / (cos_theta - Scalar(1));

//    (*J_t_m) = rjacinv;
  }

  return tan;
}

template <typename _Derived>
template <typename _DerivedOther>
typename SE2Base<_Derived>::Manifold
SE2Base<_Derived>::compose(
    const ManifoldBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  static_assert(
    std::is_base_of<SE2Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from SE2Base !");

  const auto& m_se2 = static_cast<const SE2Base<_DerivedOther>&>(m);

  const Scalar lhs_real = real(); // cos(t)
  const Scalar lhs_imag = imag(); // sin(t)
  const Scalar rhs_real = m_se2.real();
  const Scalar rhs_imag = m_se2.imag();

  if (J_mc_ma)
  {
    Jacobian adj = Jacobian::Identity();
    adj.template topLeftCorner<2,2>() = m_se2.rotation();
    adj(0,2) =  m_se2.y();
    adj(1,2) = -m_se2.x();

    (*J_mc_ma) = adj.inverse();

//    std::cout << "J_mc_ma 1\n" << (*J_mc_ma) << "\n";

//    Jacobian& refJ_mc_ma = (*J_mc_ma);
//    refJ_mc_ma = Jacobian::Identity();
//    refJ_mc_ma.template topLeftCorner<2,2>() = m_se2.rotation().template transpose();
//    refJ_mc_ma(0,2) = -m_se2.real()*m_se2.x() - m_se2.imag()*m_se2.y();
//    refJ_mc_ma(1,2) =  m_se2.imag()*m_se2.x() - m_se2.real()*m_se2.y();

//    std::cout << "J_mc_ma 2\n" << (*J_mc_ma) << "\n";
  }

  if (J_mc_mb)
  {
    J_mc_mb->setIdentity();
  }

  return Manifold(
        lhs_real * m_se2.x() - lhs_imag * m_se2.y() + x(),
        lhs_imag * m_se2.x() + lhs_real * m_se2.y() + y(),
        lhs_real * rhs_real  - lhs_imag * rhs_imag,
        lhs_real * rhs_imag  + lhs_imag * rhs_real         );
}

template <typename _Derived>
typename SE2Base<_Derived>::Vector
SE2Base<_Derived>::act(const Vector &v,
                       OptJacobianRef J_vout_m,
                       OptJacobianRef J_vout_v) const
{
  if (J_vout_m)
  {
    (*J_vout_m) = rotation() * skew2(1) * v;
  }

  if (J_vout_v)
  {
    (*J_vout_v) = rotation();
  }

  return transform() * v;
}

/// SE2 specific function

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::real() const
{
  return coeffs()(2);
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::imag() const
{
  return coeffs()(3);
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::angle() const
{
  using std::atan2;
  return atan2(imag(), real());
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::y() const
{
  return coeffs().y();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE2_BASE_H_ */
