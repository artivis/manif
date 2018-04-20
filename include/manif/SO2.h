#ifndef _MANIF_MANIF_SO2_H_
#define _MANIF_MANIF_SO2_H_

#include "manif/manifold_base.h"
#include "manif/tangent_base.h"

#include <Eigen/Core>

#include <array>
#include <iostream>

namespace manif
{

// Forward declaration to specialize traits

//template <template <typename _Scalar> class _Tangent, typename _Scalar>
//struct SO2TangentBase;

//template <typename _Scalar>
//struct SO2Tangent;

template <template <typename _Scalar> class _Manifold, typename _Scalar>
struct SO2Base;

template <typename _Scalar>
struct SO2;

// Traits specialization

//template <>
//template <template <typename _Scalar> class _Tangent, typename _Scalar>
//struct TangentTraits<SO2TangentBase<_Tangent, _Scalar>>
//{
//  using Scalar = _Scalar;
//  using Tangent = _Tangent<Scalar>;

//  static constexpr int Dim = SO2TangentBase<_Tangent, Scalar>::Dim;
//  static constexpr int RepSize = SO2TangentBase<_Tangent, Scalar>::RepSize;

//  using Manifold = SO2<Scalar>;
//};

template <>
template <template <typename _Scalar> class _Manifold, typename _Scalar>
struct ManifoldTraits<SO2Base<_Manifold, _Scalar>>
{
  using Scalar = _Scalar;

  using Base = SO2Base<_Manifold, _Scalar>;

  using Manifold = _Manifold<Scalar>;

  static constexpr int Dim = SO2Base<_Manifold, Scalar>::Dim;
  static constexpr int RepSize = SO2Base<_Manifold, Scalar>::RepSize;

//  using Tangent = SO2Tangent<Scalar>;
};

//template <>
//template <typename _Scalar>
//struct ManifoldTraits<SO2<_Scalar>>
//{
//  using Scalar = _Scalar;
//  using Manifold = SO2<_Scalar>;


//};

///////////////
///         ///
/// Tangent ///
///         ///
///////////////
/*
template <template <typename _Scalar> class _Tangent, typename _Scalar>
struct SO2TangentBase : TangentBase<SO2TangentBase<_Tangent, _Scalar>>
{
  using Base = TangentBase<SO2TangentBase<_Tangent, _Scalar>>;

  using Manifold = typename Base::Manifold;
  using Tangent  = typename Base::Tangent;

  static constexpr int Dim     = 2;
  static constexpr int RepSize = 1;
};

template <typename _Scalar>
struct SO2Tangent : SO2TangentBase<SO2Tangent, _Scalar>
{
  using Scalar = _Scalar;
  using TangentBase = SO2TangentBase<SO2Tangent, _Scalar>;
  using Manifold = typename TangentBase::Manifold;
  using Tangent  = typename TangentBase::Tangent;

  using TangentBase::Dim;
  using TangentBase::RepSize;

  Manifold retract() const
  {
    std::cout << "SO2Tangent retract\n";
    return Manifold();
  }
};
*/


////////////////
///          ///
/// Manifold ///
///          ///
////////////////


template <template <typename _Scalar> class _Manifold, typename _Scalar>
struct SO2Base : ManifoldBase<SO2Base<_Manifold, _Scalar>>
{
  using Base = ManifoldBase<SO2Base<_Manifold, _Scalar>>;

  using Manifold = typename Base::Manifold;
  using Tangent = typename Base::Tangent;

  static constexpr int Dim     = 2;
  static constexpr int RepSize = 2;
};

template <template <typename _Scalar> class _Manifold, typename _Scalar>
using SO2Tangent = typename SO2Base<_Manifold, _Scalar>::Tangent;


//template <template <typename _Scalar> class _Manifold, typename _Scalar>
//typename SO2Tangent<_Manifold, _Scalar>::Manifold
//SO2Tangent<_Manifold, _Scalar>::retract() const
//{
//  return SO2Tangent<_Manifold, _Scalar>::Manifold();
//}

template <typename _Scalar>
struct SO2 : SO2Base<SO2, _Scalar>
{
  using Scalar = _Scalar;
  using ManifoldBase = SO2Base<SO2, Scalar>;
  using Manifold = typename ManifoldBase::Manifold;
  using Tangent  = typename ManifoldBase::Tangent;

  using ManifoldBase::Dim;
  using ManifoldBase::RepSize;

  using UnderlyingData = Eigen::Matrix<Scalar, RepSize, 1>;

  SO2()  = default;
  ~SO2() = default;

  SO2(const UnderlyingData& d)
    : data_(d) { }

  void identity()
  {
    data_.setIdentity();
  }

  static Manifold Identity()
  {
    static SO2 identity(UnderlyingData::Zero());
    return identity;
  }

  Tangent lift() const
  {
    std::cout << "SO2 lift\n";
    return Tangent();
  }

protected:

  UnderlyingData data_;
};

template <template <typename _Scalar> class _Manifold, typename _Scalar>
_Manifold<_Scalar>
SO2::Tangent::retract() const
{
  return SO2Tangent<_Manifold, _Scalar>::Manifold();
}

using SO2f = SO2<float>;
using SO2d = SO2<double>;

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_H_ */
