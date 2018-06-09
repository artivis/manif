#ifndef _MANIF_MANIF_TANGENT_BASE_H_
#define _MANIF_MANIF_TANGENT_BASE_H_

#include "manif/impl/macro.h"
#include "manif/impl/traits.h"
#include "manif/constants.h"

#include "lt/optional.hpp"
//#include "lspdlog/logging.h"

namespace manif
{

template <class _Derived>
struct TangentBase
{
  static constexpr int Dim     = internal::traits<_Derived>::Dim;
  static constexpr int RepSize = internal::traits<_Derived>::RepSize;
  static constexpr int DoF     = internal::traits<_Derived>::DoF;

  using Scalar   = typename internal::traits<_Derived>::Scalar;
  using Manifold = typename internal::traits<_Derived>::Manifold;
  using Tangent  = typename internal::traits<_Derived>::Tangent;
  using DataType = typename internal::traits<_Derived>::DataType;
  using Jacobian = typename internal::traits<_Derived>::Jacobian;
  using LieType  = typename internal::traits<_Derived>::LieType;

  using OptJacobianRef = tl::optional<Jacobian&>;

  template <typename T>
  using TangentTemplate =
  typename internal::traits<_Derived>::template TangentTemplate<T>;

  /// @todo this is an implicit conversion operator,
  /// evaluate how bad it is to use it.
  operator _Derived&() { return derived(); }
  operator const _Derived& () const { return derived(); }

public:

  DataType& coeffs();
  const DataType& coeffs() const;

  Scalar* data();
  const Scalar* data() const;

  template <class _NewScalar>
  TangentTemplate<_NewScalar> cast() const;

  /// Common Tangent API

  _Derived& setZero();
  _Derived& setRandom();

  LieType skew() const;

  Manifold retract(OptJacobianRef J_m_t =
                    OptJacobianRef{}) const;

  Manifold rplus(const Manifold& m) const;
  Manifold lplus(const Manifold& m) const;

  /**
   * @brief plus, calls lplus
   * @see lplus
   */
  Manifold plus(const Manifold& m) const;

  template <typename _DerivedOther>
  Tangent plus(const TangentBase<_DerivedOther>& t) const;

  template <typename _DerivedOther>
  Tangent minus(const TangentBase<_DerivedOther>& t) const;

  /// Some operators

  /**
   * @brief operator +, lplus
   * @param t
   * @return
   * @see lplus
   */
  Manifold operator +(const Manifold& m) const;

  template <typename _DerivedOther>
  Tangent operator +(const TangentBase<_DerivedOther>& t) const;

  template <typename _DerivedOther>
  Tangent operator -(const TangentBase<_DerivedOther>& t) const;

  /**
   * @brief operator =, assignment oprator
   * @param t
   * @return
   */
  _Derived& operator =(const TangentBase<_Derived>& t);

  template <typename _DerivedOther>
  _Derived& operator =(const TangentBase<_DerivedOther>& t);

  template <class _DerivedOther>
  friend typename TangentBase<_DerivedOther>::Tangent
  operator *(const typename TangentBase<_DerivedOther>::Jacobian& J,
             const TangentBase<_DerivedOther>& t);

  /// static helpers

  static Tangent Zero();
  static Tangent Random();

private:

  _Derived& derived()
  { return *static_cast< _Derived* >(this); }
  const _Derived& derived() const
  { return *static_cast< const _Derived* >(this); }
};

template <typename _Derived>
typename TangentBase<_Derived>::DataType&
TangentBase<_Derived>::coeffs()
{
  return derived().coeffs();
}

template <typename _Derived>
const typename TangentBase<_Derived>::DataType&
TangentBase<_Derived>::coeffs() const
{
  return derived().coeffs();
}

template <class _Derived>
typename TangentBase<_Derived>::Scalar*
TangentBase<_Derived>::data()
{
  return derived().coeffs().data();
}

template <class _Derived>
const typename TangentBase<_Derived>::Scalar*
TangentBase<_Derived>::data() const
{
  return derived().coeffs().data();
}

template <typename _Derived>
template <class _NewScalar>
typename TangentBase<_Derived>::template TangentTemplate<_NewScalar>
TangentBase<_Derived>::cast() const
{
  return TangentTemplate<_NewScalar>(coeffs().template cast<_NewScalar>());
}

template <class _Derived>
_Derived& TangentBase<_Derived>::setZero()
{
  coeffs().setZero();
  return derived();
}

template <class _Derived>
_Derived& TangentBase<_Derived>::setRandom()
{
  coeffs().setRandom();
  return derived();
}

template <class _Derived>
typename TangentBase<_Derived>::Manifold
TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return derived().retract(J_m_t);
}

template <class _Derived>
typename TangentBase<_Derived>::LieType
TangentBase<_Derived>::skew() const
{
  return derived().skew();
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

template <class _Derived>
template <typename _DerivedOther>
typename TangentBase<_Derived>::Tangent
TangentBase<_Derived>::plus(const TangentBase<_DerivedOther>& t) const
{
  return Tangent(coeffs()+t.coeffs().template cast<Scalar>());
}

template <class _Derived>
template <typename _DerivedOther>
typename TangentBase<_Derived>::Tangent
TangentBase<_Derived>::minus(const TangentBase<_DerivedOther>& t) const
{
  return Tangent(coeffs()-t.coeffs().template cast<Scalar>());
}

/// Operators

template <typename _Derived>
typename TangentBase<_Derived>::Manifold
TangentBase<_Derived>::operator +(const Manifold& m) const
{
  return m.lplus(derived());
}

template <typename _Derived>
template <typename _DerivedOther>
typename TangentBase<_Derived>::Tangent
TangentBase<_Derived>::operator +(const TangentBase<_DerivedOther>& t) const
{
  return plus(t);
}

template <typename _Derived>
template <typename _DerivedOther>
typename TangentBase<_Derived>::Tangent
TangentBase<_Derived>::operator -(const TangentBase<_DerivedOther>& t) const
{
  return minus(t);
}

template <typename _Derived>
_Derived&
TangentBase<_Derived>::operator =(
    const TangentBase<_Derived>& t)
{
  derived().coeffs() = t.coeffs();
  return derived();
}

template <typename _Derived>
template <typename _DerivedOther>
_Derived&
TangentBase<_Derived>::operator =(
    const TangentBase<_DerivedOther>& t)
{
  derived().coeffs() = t.coeffs();
  return derived();
}

template <class _Derived>
typename TangentBase<_Derived>::Tangent
TangentBase<_Derived>::Zero()
{
  static const Tangent t(DataType::Zero());
  return t;
}

template <class _Derived>
typename TangentBase<_Derived>::Tangent
TangentBase<_Derived>::Random()
{
  return Tangent().setRandom();
}

template <class _DerivedOther>
typename TangentBase<_DerivedOther>::Tangent
operator *(const typename TangentBase<_DerivedOther>::Jacobian& J,
           const TangentBase<_DerivedOther>& t)
{
  return typename TangentBase<_DerivedOther>::Tangent(
        typename TangentBase<_DerivedOther>::DataType(J*t.coeffs()));
}

/// Utils

template <typename _Stream, typename _Derived>
_Stream& operator << (
    _Stream& s,
    const manif::TangentBase<_Derived>& m)
{
  s << m.coeffs().transpose();
  return s;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_TANGENT_BASE_H_ */
