#ifndef _MANIF_MANIF_SO3TANGENT_H_
#define _MANIF_MANIF_SO3TANGENT_H_

#include "manif/impl/so3/SO3Tangent_base.h"

#include <Eigen/Core>

namespace manif
{
namespace internal
{

// Traits specialization

template <typename _Scalar>
struct traits<SO3Tangent<_Scalar>>
{
  template <typename T>
  using TangentTemplate = SO3Tangent<T>;

  using Scalar = _Scalar;

  using Manifold = SO3<_Scalar>;
  using Tangent  = SO3Tangent<_Scalar>;

  using Base = SO3TangentBase<_Scalar>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;
  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieType  = Eigen::Matrix<Scalar, 3, 3>;
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
struct SO3Tangent : SO3TangentBase<SO3Tangent<_Scalar>>
{
private:

  using Base = SO3TangentBase<SO3Tangent<_Scalar>>;
  using Type = SO3Tangent<_Scalar>;

public:

  MANIF_TANGENT_TYPEDEF

  SO3Tangent() = default;

  SO3Tangent(const DataType& vec);

  /// Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  MANIF_INHERIT_TANGENT_API;
  MANIF_INHERIT_TANGENT_OPERATOR;

  /// SO3Tangent specific API

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SO3Tangent);

template <typename _Scalar>
SO3Tangent<_Scalar>::SO3Tangent(const DataType& vec)
  : data_(vec)
{
  //
}

template <typename _Scalar>
typename SO3Tangent<_Scalar>::DataType&
SO3Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename SO3Tangent<_Scalar>::DataType&
SO3Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3TANGENT_H_ */
