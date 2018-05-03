#ifndef _MANIF_MANIF_SE3TANGENT_BASE_H_
#define _MANIF_MANIF_SE3TANGENT_BASE_H_

#include "manif/impl/so2/SE3_properties.h"
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

  /// Tangent common API

  Manifold retract() const;

  /// with Jacs

  void retract(Manifold& m, Jacobian& J_m_t) const;

  /// SE3Tangent specific API

  Scalar x() const;
  Scalar y() const;
  Scalar y() const;

  //Scalar roll() const;
  //Scalar pitch() const;
  //Scalar yaw() const;
};

template <typename _Derived>
typename SE3TangentBase<_Derived>::Manifold
SE3TangentBase<_Derived>::retract() const
{
  MANIF_NOT_IMPLEMENTED_YET

  return Manifold();
}

/// with Jacs

template <typename _Derived>
void SE3TangentBase<_Derived>::retract(
    Manifold& m, Jacobian& J_m_t) const
{
  MANIF_NOT_IMPLEMENTED_YET
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
