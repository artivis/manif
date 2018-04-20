#ifndef _MANIF_MANIF_SO2_BASE_H_
#define _MANIF_MANIF_SO2_BASE_H_

#include "manif/manifold_base.h"
#include "manif/tangent_base.h"

namespace manif
{

//////////////
///        ///
/// Traits ///
///        ///
//////////////

// Forward declaration for type traits specialization

template <template <typename _Scalar> class _Manifold, typename _Scalar>
struct SO2Base;

namespace internal
{

// Traits specialization

template <>
template <template <typename _Scalar> class _Manifold, typename _Scalar>
struct traits<SO2Base<_Manifold, _Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = _Manifold<Scalar>;

  using Base = SO2Base<_Manifold, Scalar>;

  static constexpr int Dim = Base::Dim;
  static constexpr int RepSize = Base::RepSize;
};

} /* namespace internal */

////////////////
///          ///
/// Manifold ///
///          ///
////////////////


template <template <typename _Scalar> class _Manifold, typename _Scalar>
struct SO2Base : ManifoldBase<SO2Base<_Manifold, _Scalar>>
{
  using Base = ManifoldBase<SO2Base<_Manifold, _Scalar>>;

  MANIFOLD_BASE_TYPEDEF

  static constexpr int Dim     = 2;
  static constexpr int RepSize = 2;
};

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_BASE_H_ */
