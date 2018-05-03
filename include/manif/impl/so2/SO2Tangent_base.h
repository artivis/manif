#ifndef _MANIF_MANIF_SO2TANGENT_BASE_H_
#define _MANIF_MANIF_SO2TANGENT_BASE_H_

#include "manif/impl/so2/SO2_properties.h"
#include "manif/impl/tangent_base.h"

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Derived>
struct SO2TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SO2TangentBase<_Derived>;

public:

  static constexpr int Dim = internal::ManifoldProperties<Type>::Dim;
  static constexpr int DoF = internal::ManifoldProperties<Type>::DoF;
  static constexpr int N   = internal::ManifoldProperties<Type>::N;

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::coeffs;

  /// Tangent common API

  Manifold retract() const;

  LieType skew() const;

  /// with Jacs

  void retract(Manifold& m, Jacobian& J_m_t) const;

  /// SO2Tangent specific API

  //const Scalar& angle() const;

  Scalar angle() const;
};

template <typename _Derived>
typename SO2TangentBase<_Derived>::Manifold
SO2TangentBase<_Derived>::retract() const
{
  using std::cos;
  using std::sin;
  return Manifold(cos(angle()), sin(angle()));
}

template <typename _Derived>
typename SO2TangentBase<_Derived>::LieType
SO2TangentBase<_Derived>::skew() const
{
  return (LieType() << Scalar(0), Scalar(-1),
                       Scalar(0), Scalar( 1)).finished();
}

/// with Jacs

template <typename _Derived>
void SO2TangentBase<_Derived>::retract(
    Manifold& m, Jacobian& J_m_t) const
{
  m = retract();
  J_m_t.setConstant(1);
}

/// SO2Tangent specific API

//template <typename _Derived>
//const typename SO2TangentBase<_Derived>::Scalar&
//SO2TangentBase<_Derived>::angle() const
//{
//  return coeffs().x();
//}

template <typename _Derived>
typename SO2TangentBase<_Derived>::Scalar
SO2TangentBase<_Derived>::angle() const
{
  return coeffs()(0);
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_BASE_H_ */
