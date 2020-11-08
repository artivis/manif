#ifndef _MANIF_MANIF_SE3TANGENT_H_
#define _MANIF_MANIF_SE3TANGENT_H_

#include "manif/impl/se3/SE3Tangent_base.h"

namespace manif {
namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<SE3Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = SE3<_Scalar>;
  using Tangent  = SE3Tangent<_Scalar>;

  using Base = SE3TangentBase<Tangent>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg   = Eigen::Matrix<Scalar, 4, 4>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif {

//
// Tangent
//

/**
 * @brief Represents an element of tangent space of SE3.
 */
template <typename _Scalar>
struct SE3Tangent : SE3TangentBase<SE3Tangent<_Scalar>>
{
private:

  using Base = SE3TangentBase<SE3Tangent<_Scalar>>;
  using Type = SE3Tangent<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  SE3Tangent()  = default;
  ~SE3Tangent() = default;

  MANIF_COPY_CONSTRUCTOR(SE3Tangent)
  MANIF_MOVE_CONSTRUCTOR(SE3Tangent)

  // Copy constructor given base
  template <typename _DerivedOther>
  SE3Tangent(const TangentBase<_DerivedOther>& o);

  MANIF_TANGENT_ASSIGN_OP(SE3Tangent)

  // Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // SE3Tangent specific API

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SE3Tangent);

template <typename _Scalar>
template <typename _DerivedOther>
SE3Tangent<_Scalar>::SE3Tangent(
    const TangentBase<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar>
typename SE3Tangent<_Scalar>::DataType&
SE3Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename SE3Tangent<_Scalar>::DataType&
SE3Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE3TANGENT_H_ */
