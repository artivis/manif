#ifndef _MANIF_MANIF_SE2_BASE_H_
#define _MANIF_MANIF_SE2_BASE_H_

#include "manif/impl/se2/SE2_properties.h"
#include "manif/impl/manifold_base.h"

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

  void identity();

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
  using std::sin;
  using std::cos;
  const Scalar theta = angle();
  return (Rotation() << cos(theta), -sin(theta),
                        sin(theta),  cos(theta)).finished();
}

template <typename _Derived>
typename SE2Base<_Derived>::Translation
SE2Base<_Derived>::translation() const
{
  return Translation(x(), y());
}

template <typename _Derived>
void SE2Base<_Derived>::identity()
{
  coeffs_nonconst().setZero();
  coeffs_nonconst()(2) = 1;
}

template <typename _Derived>
typename SE2Base<_Derived>::Manifold
SE2Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  using std::cos;
  using std::sin;

  const Scalar theta_inv = -angle();

  if (J_minv_m)
  {
    //  p  = T(1:2);
    //  th = T(3);
    //  R  = [cos(th)  -sin(th);  sin(th)  cos(th)];
    //  Ri = R';
    //  Ti = [-Ri*p; -th];
    //  u_x = [[0 -1] ; [1 0]];
    //  J_Ti_T = [-Ri Ri*u_x*p ; [0 0 -1]];

    Jacobian& J_minv_m_ref = *J_minv_m;

    J_minv_m_ref = -Jacobian::Identity();
    J_minv_m_ref.template block<2,2>(0,0) = -rotation().transpose();
    J_minv_m_ref(0,2) = x()*sin(theta_inv) - y()*cos(theta_inv);
    J_minv_m_ref(1,2) = x()*cos(theta_inv) + y()*sin(theta_inv);
  }

  return Manifold(-(x()*cos(theta_inv) - y()*sin(theta_inv)),
                  -(x()*sin(theta_inv) + y()*cos(theta_inv)),
                   theta_inv );
}

template <typename _Derived>
typename SE2Base<_Derived>::Tangent
SE2Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  using std::abs;
  using std::cos;
  using std::sin;

  const Scalar theta = angle();
  const Scalar theta_sq = theta * theta;

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
    A = sin(theta) / theta;
    B = (Scalar(1) - cos(theta) ) / theta;
  }

  if (J_t_m)
  {
    J_t_m->setIdentity();
    J_t_m->template block<2,2>(0,0) = rotation();

    Scalar d_sin_theta_by_theta;
    Scalar d_one_minus_cos_theta_by_theta;

    if (abs(theta) < Constants<Scalar>::eps)
    {
      d_sin_theta_by_theta = -theta / Scalar(3);
      d_one_minus_cos_theta_by_theta =
          Scalar(0.5) - theta_sq * Scalar(0.125);
    }
    else
    {
      const Scalar cos_theta = cos(theta);
      const Scalar sin_theta = sin(theta);

      d_sin_theta_by_theta =
          (theta * cos_theta - sin_theta) / theta_sq;

      d_one_minus_cos_theta_by_theta =
          (theta * sin_theta + cos_theta - Scalar(1)) / theta_sq;
    }

    (*J_t_m)(0,2) =  d_sin_theta_by_theta * x() +
                     d_one_minus_cos_theta_by_theta * y();

    (*J_t_m)(1,2) = -d_one_minus_cos_theta_by_theta * x() +
                     d_sin_theta_by_theta * y();
  }

  const Scalar den = Scalar(1) / (A*A + B*B);

  return Tangent( A * den * x() + B * den * y(),
                 -B * den * x() + A * den * y(),
                  theta );
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
    const Scalar theta_inv = -angle();

    J_mc_ma->setIdentity();
    (*J_mc_ma)(0,2) = m.coeffs().x()*sin(theta_inv) -
                      m.coeffs().y()*cos(theta_inv);
    (*J_mc_ma)(1,2) = m.coeffs().x()*cos(theta_inv) +
                      m.coeffs().y()*sin(theta_inv);
  }

  if (J_mc_mb)
  {
    J_mc_mb->setIdentity();
    J_mc_mb->template block<2,2>(0,0) = rotation();
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
    (*J_vout_m) = rotation() * skew(1) * v;
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
