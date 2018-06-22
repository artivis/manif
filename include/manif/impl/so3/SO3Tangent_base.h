#ifndef _MANIF_MANIF_SO3TANGENT_BASE_H_
#define _MANIF_MANIF_SO3TANGENT_BASE_H_

#include "manif/impl/so3/SO3_properties.h"
#include "manif/impl/tangent_base.h"

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Derived>
struct SO3TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SO3TangentBase<_Derived>;

public:

  MANIF_TANGENT_PROPERTIES

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  /// Tangent common API

  using Base::coeffs;

  LieType skew() const;

  Manifold retract(OptJacobianRef J_m_t = {}) const;

  Jacobian rjac() const;
  Jacobian ljac() const;

  Jacobian adj() const;

  /// SO3Tangent specific API

  Scalar x() const;
  Scalar y() const;
  Scalar z() const;
};

template <typename _Derived>
typename SO3TangentBase<_Derived>::Manifold
SO3TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  using std::sqrt;
  using std::cos;
  using std::sin;

  const DataType& theta_vec = coeffs();
  const Scalar theta_sq = theta_vec.squaredNorm();
  const Scalar theta    = sqrt(theta_sq);

  if (theta_sq > Constants<Scalar>::eps)
  {
    if (J_m_t)
    {
      Jacobian M1, M2;

      const LieType W = skew();

      M1.noalias() = (Scalar(1.0) - cos(theta)) / theta_sq * W;
      M2.noalias() = (theta - sin(theta)) / (theta_sq * theta) * (W * W);;

      *J_m_t = Jacobian::Identity() - M1 + M2;
    }

    return Manifold( Eigen::AngleAxis<Scalar>(theta, theta_vec.normalized()) );
  }
  else
  {
    if (J_m_t)
    {
      *J_m_t = Jacobian::Identity() - Scalar(0.5) * skew();
    }

    return Manifold(x()/Scalar(2), y()/Scalar(2), z()/Scalar(2), Scalar(1));
  }
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Jacobian
SO3TangentBase<_Derived>::rjac() const
{
//  Jacobian Jr;
//  retract(Jr);

//  return Jr;

  return ljac().transpose();
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Jacobian
SO3TangentBase<_Derived>::ljac() const
{
  using std::sqrt;
  using std::cos;
  using std::sin;

  const Scalar theta_sq = coeffs().squaredNorm();

  const LieType W = skew();

  // Small angle approximation
  if (theta_sq <= Constants<Scalar>::eps)
    return Jacobian::Identity() - Scalar(0.5) * W;

  const Scalar theta = sqrt(theta_sq); // rotation angle
  Jacobian M1, M2;
  M1.noalias() = (Scalar(1) - cos(theta)) / theta_sq * W;
  M2.noalias() = (theta - sin(theta)) / (theta_sq * theta) * (W * W);

  return Jacobian::Identity() + M1 + M2;
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Jacobian
SO3TangentBase<_Derived>::adj() const
{
  return skew();
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::LieType
SO3TangentBase<_Derived>::skew() const
{
  return (LieType() << Scalar(0)  , -coeffs()(2),  coeffs()(1),
                       coeffs()(2),  Scalar(0)  , -coeffs()(0),
                      -coeffs()(1),  coeffs()(0),  Scalar(0)    ).finished();
}

/// SO3Tangent specifics

template <typename _Derived>
typename SO3TangentBase<_Derived>::Scalar
SO3TangentBase<_Derived>::x() const
{
  return coeffs()(0);
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Scalar
SO3TangentBase<_Derived>::y() const
{
  return coeffs()(1);
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Scalar
SO3TangentBase<_Derived>::z() const
{
  return coeffs()(2);
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3TANGENT_BASE_H_ */
