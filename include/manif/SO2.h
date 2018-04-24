#ifndef _MANIF_MANIF_SO2_H_
#define _MANIF_MANIF_SO2_H_

#include "manif/SO2_base.h"

#include <Eigen/Core>

#include <iostream>

namespace manif
{

// Forward declare for type traits specialization

template <typename _Scalar> struct SO2;
template <typename _Scalar> struct SO2Tangent;

namespace internal
{

// Traits specialization

template <>
template <typename _Scalar>
struct traits<SO2<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SO2<_Scalar>;
  using Tangent  = SO2Tangent<_Scalar>;

  using Base = SO2Base<SO2<_Scalar>>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int RepSize = 2;

  using ManifoldDataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using JacobianType = Eigen::Matrix<Scalar, RepSize, RepSize>;
  /*
  using JacobianType0    = Eigen::Matrix<Scalar, RepSize, RepSize>;
  using JacobianType1    = Eigen::Matrix<Scalar, RepSize, DoF>;
  using JacobianType2    = Eigen::Matrix<Scalar, DoF, RepSize>;
  */

  using Transformation = Eigen::Matrix<Scalar, Dim+1, Dim+1>;
};

template <typename _Scalar>
struct traits<SO2Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SO2<_Scalar>;
  using Tangent = SO2Tangent<_Scalar>;

  using Base = SO2Base<SO2<_Scalar>>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using TangentDataType  = Eigen::Matrix<Scalar, RepSize, 1>;

  using JacobianType = Eigen::Matrix<Scalar, 2, 2>;
  /*
  using JacobianType0    = Eigen::Matrix<Scalar, RepSize, RepSize>;
  using JacobianType1    = Eigen::Matrix<Scalar, RepSize, DoF>;
  using JacobianType2    = Eigen::Matrix<Scalar, DoF, RepSize>;
  */
};

} /* namespace internal */

} /* namespace manif */

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Scalar>
struct SO2Tangent : SO2TangentBase<SO2Tangent<_Scalar>>
{
  using Base = SO2TangentBase<SO2Tangent<_Scalar>>;
  using Manifold = typename Base::Manifold;
  using TangentDataType = typename Base::TangentDataType;

  using Base::zero;
  using Base::random;
  using Base::retract;

  SO2Tangent() = default;

  SO2Tangent(const double theta);
  SO2Tangent(const TangentDataType& theta);

  TangentDataType* data();
  const TangentDataType* data() const;

protected:

  TangentDataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SO2Tangent);

template <typename _Scalar>
SO2Tangent<_Scalar>::SO2Tangent(const double theta)
  : data_(theta)
{
  //
}

template <typename _Scalar>
SO2Tangent<_Scalar>::SO2Tangent(const TangentDataType& theta)
  : data_(theta)
{
  //
}

template <typename _Scalar>
typename SO2Tangent<_Scalar>::TangentDataType*
SO2Tangent<_Scalar>::data()
{
  return &data_;
}

template <typename _Scalar>
const typename SO2Tangent<_Scalar>::TangentDataType*
SO2Tangent<_Scalar>::data() const
{
  return &data_;
}

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Scalar>
struct SO2 : SO2Base<SO2<_Scalar>>
{
  using Base = SO2Base<SO2<_Scalar>>;

  using Tangent = typename Base::Tangent;

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  SO2()  = default;
  ~SO2() = default;

  SO2(const ManifoldDataType& d);

  ManifoldDataType* data();
  const ManifoldDataType* data() const;

  using Base::identity;
  using Base::random;
  using Base::inverse;
  using Base::rplus;
  using Base::lplus;
  using Base::lift;
  using Base::compose;

  SO2 rminus(const SO2& /*m*/) const;

  SO2 lminus(const SO2& /*m*/) const;

protected:

  ManifoldDataType data_;
};

MANIF_EXTRA_MANIFOLD_TYPEDEF(SO2)

/// SO2 functions definitions

template <typename _Scalar>
SO2<_Scalar>::SO2(const ManifoldDataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
typename SO2<_Scalar>::ManifoldDataType*
SO2<_Scalar>::data()
{
  return &data_;
}

template <typename _Scalar>
const typename SO2<_Scalar>::ManifoldDataType*
SO2<_Scalar>::data() const
{
  return &data_;
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::rminus(const SO2& /*m*/) const
{
  MANIF_INFO("SO2 rminus");
  return SO2();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::lminus(const SO2& /*m*/) const
{
  MANIF_INFO("SO2 lminus");
  return SO2();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_H_ */
