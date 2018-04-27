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
  using Scalar = _Scalar;

  using Manifold = SO3<_Scalar>;
  using Tangent  = SO3Tangent<_Scalar>;

  /// @todo is it useful ???
  using Base = SO3TangentBase<_Scalar>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using TangentDataType  = Eigen::Matrix<Scalar, RepSize, 1>;

  using JacobianTtoT = Eigen::Matrix<Scalar, DoF, DoF>;
  using JacobianTtoM = Eigen::Matrix<Scalar, RepSize, DoF>;
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

  using Scalar = typename Base::Scalar;
  using Manifold = typename Base::Manifold;
  using TangentDataType = typename Base::TangentDataType;

  SO3Tangent() = default;

  SO3Tangent(const TangentDataType& theta);

  /// Tangent common API

  TangentDataType* data();
  const TangentDataType* data() const;

  MANIF_INHERIT_TANGENT_API

  /// SO3Tangent specific API

protected:

  TangentDataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SO3Tangent);

template <typename _Scalar>
SO3Tangent<_Scalar>::SO3Tangent(const TangentDataType& theta)
  : data_(theta)
{
  //
}

template <typename _Scalar>
typename SO3Tangent<_Scalar>::TangentDataType*
SO3Tangent<_Scalar>::data()
{
  return &data_;
}

template <typename _Scalar>
const typename SO3Tangent<_Scalar>::TangentDataType*
SO3Tangent<_Scalar>::data() const
{
  return &data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3TANGENT_H_ */
