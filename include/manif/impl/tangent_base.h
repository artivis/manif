#ifndef _MANIF_MANIF_TANGENT_BASE_H_
#define _MANIF_MANIF_TANGENT_BASE_H_

#include "manif/impl/fwd.h"

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

  using Jacobian = typename internal::traits<_Derived>::Jacobian;

  /// @todo this is an implicit conversion operator,
  /// evaluate how bad it is to use it.
  operator Tangent&() { return derived(); }
  operator const Tangent& () const { return derived(); }

  TangentDataType* data();
  const TangentDataType* data() const;

  /// Common Tangent API

  void zero();
  void random();
  Manifold retract() const;

  Manifold rplus(const Manifold& m) const;
  Manifold lplus(const Manifold& m) const;

  /**
   * @brief plus, calls lplus
   * @see lplus
   */
  Manifold plus(const Manifold& m) const;

  /// Some operators

  /**
   * @brief operator +, lplus
   * @param t
   * @return
   * @see lplus
   */
  Manifold operator +(const Manifold& t) const;

  /// with Jacs

  void retract(Manifold& m, Jacobian& J_m_t) const;

  /// static helpers

  static Tangent Zero();
  static Tangent Random();
  static Manifold Retract(const Tangent& t);

  static void Retract(const Tangent& t, Manifold& m, Jacobian& J_m_t);

private:

  Tangent& derived() { return *static_cast< Tangent* >(this); }
  const Tangent& derived() const { return *static_cast< const Tangent* >(this); }
};

template <class _Derived>
typename TangentBase<_Derived>::TangentDataType*
TangentBase<_Derived>::data()
{
  return derived().data();
}

template <class _Derived>
const typename TangentBase<_Derived>::TangentDataType*
TangentBase<_Derived>::data() const
{
  return derived().data();
}

template <class _Derived>
void TangentBase<_Derived>::zero()
{
  derived().zero();
}

template <class _Derived>
void TangentBase<_Derived>::random()
{
  derived().random();
}

template <class _Derived>
typename TangentBase<_Derived>::Manifold
TangentBase<_Derived>::retract() const
{
  return derived().retract();
}

template <class _Derived>
typename TangentBase<_Derived>::Manifold
TangentBase<_Derived>::rplus(const Manifold& m) const
{
  return m.rplus(derived());
}

template <class _Derived>
typename TangentBase<_Derived>::Manifold
TangentBase<_Derived>::lplus(const Manifold& m) const
{
  return m.lplus(derived());
}

template <class _Derived>
typename TangentBase<_Derived>::Manifold
TangentBase<_Derived>::plus(const Manifold& m) const
{
  return m.lplus(derived());
}

/// Some operators

/// Operators

template <typename _Derived>
typename TangentBase<_Derived>::Manifold
TangentBase<_Derived>::operator +(const Manifold& t) const
{
  return derived().lplus(t);
}

/// with Jacs

template <class _Derived>
void TangentBase<_Derived>::retract(Manifold& m,
                                    Jacobian& J_m_t) const
{
  derived().retract(m, J_m_t);
}

template <class _Derived>
typename TangentBase<_Derived>::Tangent
TangentBase<_Derived>::Zero()
{
  static const Tangent t(TangentDataType::Zero());
  return t;
}

template <class _Derived>
typename TangentBase<_Derived>::Tangent
TangentBase<_Derived>::Random()
{
  static const Tangent t(TangentDataType::Random());
  return t;
}

template <class _Derived>
typename TangentBase<_Derived>::Manifold
TangentBase<_Derived>::Retract(const Tangent& t)
{
  return t.retract();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_TANGENT_BASE_H_ */
