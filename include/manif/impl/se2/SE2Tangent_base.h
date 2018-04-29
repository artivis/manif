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
  using std::cos;
  using std::sin;

  const Scalar theta = angle();

  /// @todo ret

  return Manifold(x(), y(), cos(theta), sin(theta));
}

/// with Jacs

template <typename _Derived>
void SE2TangentBase<_Derived>::retract(
    Manifold& m, Jacobian& J_m_t) const
{
  m = retract();
  /// @todo J
  J_m_t.setIdentity();
  J_m_t.template block<2,2>(0,0) = m.rotation();
  J_m_t(0,2) = x();
  J_m_t(1,2) = y();
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
