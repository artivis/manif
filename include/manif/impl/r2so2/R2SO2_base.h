#ifndef _MANIF_MANIF_R2SO2_BASE_H_
#define _MANIF_MANIF_R2SO2_BASE_H_

#include "manif/impl/R2SO2/R2SO2_properties.h"
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
struct R2SO2Base : ManifoldBase<_Derived>
{
private:

  using Base = ManifoldBase<_Derived>;
  using Type = R2SO2Base<_Derived>;

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

  R2SO2Base<_Derived>& setIdentity();

  Manifold inverse(OptJacobianRef J_minv_m = {}) const;
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  template <typename _DerivedOther>
  Manifold compose(const ManifoldBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  Vector act(const Vector &v,
             OptJacobianRef J_vout_m = {},
             OptJacobianRef J_vout_v = {}) const;

  Jacobian adj() const;

  using Base::coeffs;
  using Base::coeffs_nonconst;
  MANIF_INHERIT_MANIFOLD_AUTO_API
  MANIF_INHERIT_MANIFOLD_OPERATOR

  /// R2SO2 specific functions

  Scalar real() const;
  Scalar imag() const;
  Scalar angle() const;

  Scalar x() const;
  Scalar y() const;
};

template <typename _Derived>
typename R2SO2Base<_Derived>::Transformation
R2SO2Base<_Derived>::transform() const
{
  Transformation T(Transformation::Identity());
  T.template block<2,2>(0,0) = rotation();
  T(0,2) = x();
  T(1,2) = y();
  return T;
}

template <typename _Derived>
typename R2SO2Base<_Derived>::Rotation
R2SO2Base<_Derived>::rotation() const
{
  return (Rotation() << real(), -imag(),
                        imag(),  real() ).finished();
}

template <typename _Derived>
typename R2SO2Base<_Derived>::Translation
R2SO2Base<_Derived>::translation() const
{
  return Translation(x(), y());
}

template <typename _Derived>
R2SO2Base<_Derived>&
R2SO2Base<_Derived>::setIdentity()
{
  coeffs_nonconst().setZero();
  coeffs_nonconst()(2) = 1;
  return *this;
}

template <typename _Derived>
typename R2SO2Base<_Derived>::Manifold
R2SO2Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  using std::cos;
  using std::sin;

  if (J_minv_m)
  {
    MANIF_NOT_IMPLEMENTED_YET
    //(*J_minv_m) = -adj();
  }

  return Manifold(-x()*real() - y()*imag(),
                   x()*imag() - y()*real(),
                           -angle()        );
}

template <typename _Derived>
typename R2SO2Base<_Derived>::Tangent
R2SO2Base<_Derived>::lift(OptJacobianRef J_t_m) const
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

  if (J_t_m)
  {
    // Jr^-1
    (*J_t_m) = tan.rjac().inverse();
  }

  return tan;
}

template <typename _Derived>
template <typename _DerivedOther>
typename R2SO2Base<_Derived>::Manifold
R2SO2Base<_Derived>::compose(
    const ManifoldBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  static_assert(
    std::is_base_of<R2SO2Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from R2SO2Base !");

  const auto& m_R2SO2 = static_cast<const R2SO2Base<_DerivedOther>&>(m);

  const Scalar lhs_real = real(); // cos(t)
  const Scalar lhs_imag = imag(); // sin(t)
  const Scalar rhs_real = m_R2SO2.real();
  const Scalar rhs_imag = m_R2SO2.imag();

  if (J_mc_ma)
  {
    (*J_mc_ma) = m.adj().inverse();
  }

  if (J_mc_mb)
  {
    J_mc_mb->setIdentity();
  }

  return Manifold(
        lhs_real * m_R2SO2.x() - lhs_imag * m_R2SO2.y() + x(),
        lhs_imag * m_R2SO2.x() + lhs_real * m_R2SO2.y() + y(),
        lhs_real * rhs_real  - lhs_imag * rhs_imag,
        lhs_real * rhs_imag  + lhs_imag * rhs_real           );
}

template <typename _Derived>
typename R2SO2Base<_Derived>::Vector
R2SO2Base<_Derived>::act(const Vector &v,
                         OptJacobianRef J_vout_m,
                         OptJacobianRef J_vout_v) const
{
  if (J_vout_m)
  {
    (*J_vout_m) = rotation() * skew(1) * v;
  }

  if (J_vout_v)
  {
    (*J_vout_v) = rotation();
  }

  return transform() * v;
}

template <typename _Derived>
typename R2SO2Base<_Derived>::Jacobian
R2SO2Base<_Derived>::adj() const
{
  Jacobian Adj = Jacobian::Identity();
  Adj.template topLeftCorner<2,2>() = rotation();
  Adj(0,2) =  y();
  Adj(1,2) = -x();
  return Adj;
}

/// R2SO2 specific function

template <typename _Derived>
typename R2SO2Base<_Derived>::Scalar
R2SO2Base<_Derived>::real() const
{
  return coeffs()(2);
}

template <typename _Derived>
typename R2SO2Base<_Derived>::Scalar
R2SO2Base<_Derived>::imag() const
{
  return coeffs()(3);
}

template <typename _Derived>
typename R2SO2Base<_Derived>::Scalar
R2SO2Base<_Derived>::angle() const
{
  using std::atan2;
  return atan2(imag(), real());
}

template <typename _Derived>
typename R2SO2Base<_Derived>::Scalar
R2SO2Base<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename R2SO2Base<_Derived>::Scalar
R2SO2Base<_Derived>::y() const
{
  return coeffs().y();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_R2SO2_BASE_H_ */
