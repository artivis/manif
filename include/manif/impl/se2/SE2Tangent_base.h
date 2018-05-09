#ifndef _MANIF_MANIF_SE2TANGENT_BASE_H_
#define _MANIF_MANIF_SE2TANGENT_BASE_H_

#include "manif/impl/se2/SE2_properties.h"
#include "manif/impl/tangent_base.h"

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Derived>
struct SE2TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SE2TangentBase<_Derived>;

public:

  MANIF_TANGENT_PROPERTIES
  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::data;
  using Base::coeffs;

  /// Tangent common API

  Manifold retract(OptJacobianRef J_m_t = {}) const;

  /// SE2Tangent specific API

  Scalar x() const;
  Scalar y() const;
  Scalar angle() const;
};

template <typename _Derived>
typename SE2TangentBase<_Derived>::Manifold
SE2TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  using std::abs;
  using std::cos;
  using std::sin;

  const Scalar theta = angle();
  const Scalar theta_sq = theta*theta;

  Scalar sin_theta_by_theta;
  Scalar one_minus_cos_theta_by_theta;

  if (J_m_t)
    J_m_t->setIdentity();

  if (abs(theta) < Constants<Scalar>::eps)
  {
    // Taylor approximation
    const Scalar theta_sq = theta * theta;
    sin_theta_by_theta = Scalar(1) - Scalar(1. / 6.) * theta_sq;
    one_minus_cos_theta_by_theta =
        Scalar(.5) * theta - Scalar(1. / 24.) * theta * theta_sq;

    if (J_m_t)
    {
      const Scalar d_sin_theta_by_theta = -theta / Scalar(3);

      const Scalar d_one_minus_cos_theta_by_theta =
                    Scalar(0.5) - theta_sq * Scalar(0.125);

      (*J_m_t)(0,2) = d_sin_theta_by_theta * x() -
                      d_one_minus_cos_theta_by_theta * y();

      (*J_m_t)(1,2) = d_one_minus_cos_theta_by_theta * x() +
                      d_sin_theta_by_theta * y();
    }
  }
  else
  {
    // Euler
    sin_theta_by_theta = sin(theta) / theta;
    one_minus_cos_theta_by_theta = (Scalar(1) - cos(theta)) / theta;

    if (J_m_t)
    {
      const Scalar cos_theta = cos(theta);
      const Scalar sin_theta = sin(theta);

      const Scalar d_sin_theta_by_theta =
          (theta * cos_theta - sin_theta) / theta_sq;

      const Scalar d_one_minus_cos_theta_by_theta =
          (theta * sin_theta + cos_theta - Scalar(1)) / theta_sq;

      (*J_m_t)(0,2) = d_sin_theta_by_theta * x() -
                      d_one_minus_cos_theta_by_theta * y();

      (*J_m_t)(1,2) = d_one_minus_cos_theta_by_theta * x() +
                      d_sin_theta_by_theta * y();
    }
  }

  return Manifold(
    sin_theta_by_theta * x() - one_minus_cos_theta_by_theta * y(),
    one_minus_cos_theta_by_theta * x() + sin_theta_by_theta * y(),
    angle() );
}

/// SE2Tangent specific API

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::angle() const
{
  return coeffs().z();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE2_BASE_H_ */
