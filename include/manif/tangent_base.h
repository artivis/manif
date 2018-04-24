#ifndef _MANIF_MANIF_TANGENT_BASE_H_
#define _MANIF_MANIF_TANGENT_BASE_H_

#include "manif/fwd.h"

#include "lspdlog/logging.h"

namespace manif
{

template <class _Derived>
struct TangentBase
{
  using Scalar   = typename internal::traits<_Derived>::Scalar;

  using Manifold = typename internal::traits<_Derived>::Manifold;
  using Tangent  = typename internal::traits<_Derived>::Tangent;

  static constexpr int Dim     = internal::traits<_Derived>::Dim;
  static constexpr int RepSize = internal::traits<_Derived>::RepSize;
  static constexpr int DoF     = internal::traits<_Derived>::DoF;

  using TangentDataType = typename internal::traits<_Derived>::TangentDataType;
  using JacobianType = typename internal::traits<_Derived>::JacobianType;

  /// @todo this is an implicit conversion operator,
  /// evaluate how bad it is to use it.
  operator Tangent&() { return tangent(); }
  operator const Tangent& () const { return tangent(); }

  TangentDataType* data();
  const TangentDataType* data() const;

  void zero();

  void random();

  Manifold retract() const;

  static Manifold Retract(const Tangent& t);

private:

  Tangent& tangent() { return *static_cast< Tangent* >(this); }
  const Tangent& tangent() const { return *static_cast< const Tangent* >(this); }
};

template <class _Derived>
typename TangentBase<_Derived>::TangentDataType*
TangentBase<_Derived>::data()
{
  return tangent().data();
}

template <class _Derived>
const typename TangentBase<_Derived>::TangentDataType*
TangentBase<_Derived>::data() const
{
  return tangent().data();
}

template <class _Derived>
void TangentBase<_Derived>::zero()
{
  MANIF_INFO("TangentBase zero");
  tangent().zero();
}

template <class _Derived>
void TangentBase<_Derived>::random()
{
  MANIF_INFO("TangentBase random");
  tangent().random();
}

template <class _Derived>
typename TangentBase<_Derived>::Manifold
TangentBase<_Derived>::retract() const
{
  MANIF_INFO("TangentBase retract");
  return tangent().retract();
}

template <class _Derived>
typename TangentBase<_Derived>::Manifold
TangentBase<_Derived>::Retract(const Tangent& t)
{
  MANIF_INFO("TangentBase Retract");
  return t.retract();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_TANGENT_BASE_H_ */
