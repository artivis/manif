#ifndef _MANIF_MANIF_SE2TANGENT_BASE_H_
#define _MANIF_MANIF_SE2TANGENT_BASE_H_

#include "manif/impl/so2/SE2_properties.h"
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

  /// Tangent common API

  Manifold retract() const;

  /// with Jacs

  void retract(Manifold& m, Jacobian& J_m_t) const;

  /// SE2Tangent specific API

  Scalar x() const;
  Scalar y() const;
  Scalar angle() const;
};

template <typename _Derived>
typename SE2TangentBase<_Derived>::Manifold
SE2TangentBase<_Derived>::retract() const
{
  using std::abs;
  using std::cos;
  using std::sin;

  const Scalar theta = angle();

  Scalar sin_theta_by_theta;
  Scalar one_minus_cos_theta_by_theta;

  if (abs(theta) < constants<Scalar>::eps)
  {
    // Taylor approximation
    const Scalar theta_sq = theta * theta;
    sin_theta_by_theta = Scalar(1) - Scalar(1. / 6.) * theta_sq;
    one_minus_cos_theta_by_theta =
        Scalar(.5) * theta - Scalar(1. / 24.) * theta * theta_sq;
  }
  else
  {
    // Euler
    sin_theta_by_theta = sin(theta) / theta;
    one_minus_cos_theta_by_theta = (Scalar(1) - cos(theta)) / theta;
  }

  return Manifold(
    sin_theta_by_theta * x() - one_minus_cos_theta_by_theta * y(),
    one_minus_cos_theta_by_theta * x() + sin_theta_by_theta * y(),
    angle() );
}

/// with Jacs

template <typename _Derived>
void SE2TangentBase<_Derived>::retract(
    Manifold& m, Jacobian& J_m_t) const
{
  using std::abs;
  using std::cos;
  using std::sin;

  m = retract();

  /// @todo check J

  const Scalar theta    = angle();
  const Scalar theta_sq = theta*theta;

  J_m_t.setIdentity();
  J_m_t.template block<2,2>(0,0) = rotation();

  Scalar d_sin_theta_by_theta;
  Scalar d_one_minus_cos_theta_by_theta;

  if (abs(theta) < constants<Scalar>::eps)
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

  J_m_t(0,2) = d_sin_theta_by_theta * x() -
               d_one_minus_cos_theta_by_theta * y();

  J_m_t(1,2) = d_one_minus_cos_theta_by_theta * x() +
               d_sin_theta_by_theta * y();
}

/// SE2Tangent specific API

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::x() const
{
  return data()->x();
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::y() const
{
  return data()->y();
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::angle() const
{
  return data()->z();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE2_BASE_H_ */
