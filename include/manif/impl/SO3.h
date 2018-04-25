#ifndef _MANIF_MANIF_SO3_H_
#define _MANIF_MANIF_SO3_H_

#include "manif/impl/SO3_base.h"

#include <Eigen/Core>

namespace manif
{

// Forward declare for type traits specialization

template <typename _Scalar> struct SO3;
template <typename _Scalar> struct SO3Tangent;

namespace internal
{

// Traits specialization

template <>
template <typename _Scalar>
struct traits<SO3<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SO3<_Scalar>;
  using Tangent  = SO3Tangent<_Scalar>;

  using Base = SO3Base<SO3<_Scalar>>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int N       = ManifoldProperties<Base>::N;
  static constexpr int RepSize = 4;

  using ManifoldDataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using JacobianMtoM = Eigen::Matrix<Scalar, RepSize, RepSize>;
  using JacobianMtoT = Eigen::Matrix<Scalar, DoF, RepSize>;

  using Transformation = Eigen::Matrix<Scalar, N, N>;

  using Rotation = Eigen::Matrix<Scalar, Dim, Dim>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif
{

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Scalar>
struct SO3 : SO3Base<SO3<_Scalar>>
{
private:

  using Base = SO3Base<SO3<_Scalar>>;
  using Type = SO3<_Scalar>;

public:

  using Tangent = typename Base::Tangent;

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  SO3()  = default;
  ~SO3() = default;

  SO3(const ManifoldDataType& d);

  const ManifoldDataType* data() const;

  MANIF_INHERIT_MANIFOLD_API

protected:

  friend class ManifoldBase<SO3<Scalar>>;
  ManifoldDataType* data();

  ManifoldDataType data_;
};

MANIF_EXTRA_MANIFOLD_TYPEDEF(SO3)

/// SO3 functions definitions

template <typename _Scalar>
SO3<_Scalar>::SO3(const ManifoldDataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
typename SO3<_Scalar>::ManifoldDataType*
SO3<_Scalar>::data()
{
  return &data_;
}

template <typename _Scalar>
const typename SO3<_Scalar>::ManifoldDataType*
SO3<_Scalar>::data() const
{
  return &data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3_H_ */
