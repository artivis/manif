#ifndef _MANIF_MANIF_SE3TANGENT_BASE_H_
#define _MANIF_MANIF_SE3TANGENT_BASE_H_

#include "manif/impl/se3/SE3_properties.h"
#include "manif/impl/tangent_base.h"

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Derived>
struct SE3TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SE3TangentBase<_Derived>;

public:

  MANIF_TANGENT_PROPERTIES
  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::data;
  using Base::coeffs;
  using Base::coeffs_nonconst;

  /// Tangent common API

  void zero();
  void random();

  Manifold retract(OptJacobianRef J_m_t = {}) const;

  /// SE3Tangent specific API

  Scalar x() const;
  Scalar y() const;
  Scalar z() const;

  //Scalar roll() const;
  //Scalar pitch() const;
  //Scalar yaw() const;
};

template <typename _Derived>
void SE3TangentBase<_Derived>::zero()
{
  coeffs_nonconst().setZero();
}

template <typename _Derived>
void SE3TangentBase<_Derived>::random()
{
  coeffs_nonconst().setRandom();
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::Manifold
SE3TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  MANIF_NOT_IMPLEMENTED_YET;

  if (J_m_t)
  {

  }

  return Manifold();
}

/// SE3Tangent specific API

template <typename _Derived>
typename SE3TangentBase<_Derived>::Scalar
SE3TangentBase<_Derived>::x() const
{
  return data()->x();
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::Scalar
SE3TangentBase<_Derived>::y() const
{
  return data()->y();
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::Scalar
SE3TangentBase<_Derived>::z() const
{
  return data()->z();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE3_BASE_H_ */
