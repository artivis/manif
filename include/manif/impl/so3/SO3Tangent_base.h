#ifndef _MANIF_MANIF_SO3TANGENT_BASE_H_
#define _MANIF_MANIF_SO3TANGENT_BASE_H_

#include "manif/impl/so3/SO3_properties.h"
#include "manif/impl/tangent_base.h"

#include <Eigen/Geometry>

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

  static constexpr int Dim = internal::ManifoldProperties<Type>::Dim;
  static constexpr int DoF = internal::ManifoldProperties<Type>::DoF;
  static constexpr int N   = internal::ManifoldProperties<Type>::N;

  using Scalar = typename Base::Scalar;

  using Manifold = typename Base::Manifold;
  using Tangent  = typename Base::Tangent;
  using Jacobian = typename Base::Jacobian;
  using DataType = typename Base::DataType;
  using LieType  = typename Base::LieType;

  using Base::coeffs;
  using Base::coeffs_nonconst;

  /// Tangent common API

  void zero();
  void random();
  Manifold retract() const;
  LieType skew() const;

  /// with Jacs

  void retract(Manifold& m, Jacobian& J_m_t) const;

  /// SO3Tangent specific API

  Scalar x() const;
  Scalar y() const;
  Scalar z() const;
};

template <typename _Derived>
void SO3TangentBase<_Derived>::zero()
{
  coeffs_nonconst().setZero();
}

template <typename _Derived>
void SO3TangentBase<_Derived>::random()
{
  coeffs_nonconst().setRandom();
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Manifold
SO3TangentBase<_Derived>::retract() const
{
  using std::sqrt;
  using std::cos;
  using std::sin;

  const Scalar angle = sqrt(coeffs().squaredNorm());

  if (angle > Constants<Scalar>::eps)
  {
    return Manifold( Eigen::AngleAxis<Scalar>(angle, coeffs().normalized()) );
  }
  else
  {
    return Manifold(x()/2, y()/2, z()/2, 1);
  }
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::LieType
SO3TangentBase<_Derived>::skew() const
{
  MANIF_NOT_IMPLEMENTED_YET
  return (LieType() <<      0     , -coeffs()(2),  coeffs()(1),
                       coeffs()(2),       0     , -coeffs()(0),
                      -coeffs()(1),  coeffs()(0),      0      ).finished();
}

/// with Jacs

template <typename _Derived>
void SO3TangentBase<_Derived>::retract(
    Manifold& m, Jacobian& J_m_t) const
{
  MANIF_NOT_IMPLEMENTED_YET
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
