#ifndef _MANIF_MANIF_SO2_BASE_H_
#define _MANIF_MANIF_SO2_BASE_H_

#include "manif/manifold_base.h"
#include "manif/tangent_base.h"

namespace manif
{

template <typename _Derived> struct SO2Base;
template <typename _Derived> struct SO2TangentBase;

template <>
template <typename _Derived>
struct ManifoldProperties<SO2Base<_Derived>>
{
  static constexpr int Dim = 2;
  static constexpr int DoF = 1;
};

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Derived>
struct SO2Base : ManifoldBase<_Derived>
{
  using Base = ManifoldBase<_Derived>;

  using Manifold = typename Base::Manifold;
  using Tangent  = typename Base::Tangent;

  static constexpr int Dim      = ManifoldProperties<SO2Base<_Derived>>::Dim;
  static constexpr int DoF      = ManifoldProperties<SO2Base<_Derived>>::DoF;
  static constexpr int RepSize  = 2;
};

template <typename _Derived>
struct SO2TangentBase : TangentBase<_Derived>
{
  using Base = TangentBase<_Derived>;

  using Manifold = typename Base::Manifold;
  using Tangent  = typename Base::Tangent;

  static constexpr int Dim      = ManifoldProperties<SO2Base<_Derived>>::Dim;
  static constexpr int DoF      = ManifoldProperties<SO2Base<_Derived>>::DoF;
  static constexpr int RepSize  = 2;
};

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_BASE_H_ */
