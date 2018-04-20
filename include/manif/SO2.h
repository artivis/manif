#ifndef _MANIF_MANIF_SO2_H_
#define _MANIF_MANIF_SO2_H_

#include "manif/SO2Base.h"

#include <Eigen/Core>

#include <iostream>

namespace manif
{

// Forward declare for type traits specialization

//template <typename _Scalar> struct SO2;

} /* namespace manif */

namespace Eigen
{
namespace internal
{

//template<template <typename _Scalar> class _Manifold, typename _Scalar>
//struct traits<manif::SO2Base<_Manifold, _Scalar> >
//{
//  using Dim = /*typename*/ manif::SO2Base<_Manifold, _Scalar>::Dim;
//  using ManifoldDataType = Eigen::Matrix<_Scalar, Dim, 1>;
//};

} /* namespace internal */
} /* namespace Eigen */

namespace manif
{

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Scalar>
struct SO2 : SO2Base<SO2, _Scalar>
{
  using ManifoldBase = SO2Base<SO2, _Scalar>;

  COMPLETE_MANIFOLD_TYPEDEF

  using ManifoldDataType = Eigen::Matrix<Scalar, RepSize, 1>;
  using TangentDataType  = Eigen::Matrix<Scalar, Dim, 1>;

  SO2()  = default;
  ~SO2() = default;

  SO2(const ManifoldDataType& d);

  void identity();

  void random();

  SO2 inverse() const;

  SO2 rplus(const SO2::Tangent& /*t*/) const;

  SO2 lplus(const SO2::Tangent& /*t*/) const;

  SO2 rminus(const SO2& /*m*/) const;

  SO2 lminus(const SO2& /*m*/) const;

  Tangent lift() const;

  SO2 compose(const SO2& /*m*/) const;

protected:

  ManifoldDataType data_;
};

EXTRA_MANIFOLD_TYPEDEF(SO2)

/// SO2 functions definitions

template <typename _Scalar>
SO2<_Scalar>::SO2(const ManifoldDataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
void SO2<_Scalar>::identity()
{
  std::cout << "SO2 identity\n";
  data_.setIdentity();
}

template <typename _Scalar>
void SO2<_Scalar>::random()
{
  std::cout << "SO2 random\n";
  data_.setRandom();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::inverse() const
{
  std::cout << "SO2 inverse\n";
  return SO2();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::rplus(const SO2::Tangent& /*t*/) const
{
  std::cout << "SO2 rplus\n";
  return Manifold();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::lplus(const SO2::Tangent& /*t*/) const
{
  std::cout << "SO2 lplus\n";
  return Manifold();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::rminus(const SO2& /*m*/) const
{
  std::cout << "SO2 rminus\n";
  return SO2();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::lminus(const SO2& /*m*/) const
{
  std::cout << "SO2 lminus\n";
  return SO2();
}

template <typename _Scalar>
typename SO2<_Scalar>::Tangent SO2<_Scalar>::lift() const
{
  std::cout << "SO2 lift\n";
  return Tangent();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::compose(const SO2& /*m*/) const
{
  std::cout << "SO2 compose\n";
  return SO2();
}

/// Tangent functions definitions

template <typename _Scalar>
void SO2<_Scalar>::Tangent::zero()
{
  std::cout << "SO2Tangent zero\n";
  return SO2<_Scalar>::Tangent();
}

template <typename _Scalar>
void SO2<_Scalar>::Tangent::random()
{
  std::cout << "SO2Tangent random\n";
  return SO2<_Scalar>::Tangent();
}

template <typename _Scalar>
typename SO2<_Scalar>::Manifold
SO2<_Scalar>::Tangent::retract() const
{
  std::cout << "SO2Tangent rectract\n";
  return SO2<_Scalar>();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_H_ */
