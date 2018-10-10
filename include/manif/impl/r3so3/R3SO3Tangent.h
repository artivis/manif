#ifndef _MANIF_MANIF_R3SO3TANGENT_H_
#define _MANIF_MANIF_R3SO3TANGENT_H_

#include "manif/impl/R3SO3/R3SO3Tangent_base.h"

#include <Eigen/Core>

namespace manif
{
namespace internal
{

// Traits specialization

template <typename _Scalar>
struct traits<R3SO3Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = R3SO3<_Scalar>;
  using Tangent  = R3SO3Tangent<_Scalar>;

  using Base = R3SO3TangentBase<_Scalar>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;
  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg   = Eigen::Matrix<Scalar, 3, 3>;
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
struct R3SO3Tangent : R3SO3TangentBase<R3SO3Tangent<_Scalar>>
{
private:

  using Base = R3SO3TangentBase<R3SO3Tangent<_Scalar>>;
  using Type = R3SO3Tangent<_Scalar>;

public:

  MANIF_TANGENT_TYPEDEF

  R3SO3Tangent() = default;

  R3SO3Tangent(const DataType& v);

  /// Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  /// R3SO3Tangent specific API

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(R3SO3Tangent);

template <typename _Scalar>
R3SO3Tangent<_Scalar>::R3SO3Tangent(const DataType& theta)
  : data_(theta)
{
  //
}

template <typename _Scalar>
typename R3SO3Tangent<_Scalar>::DataType&
R3SO3Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename R3SO3Tangent<_Scalar>::DataType&
R3SO3Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_R3SO3TANGENT_H_ */
