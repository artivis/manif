#ifndef _MANIF_MANIF_SE_2_3TANGENT_H_
#define _MANIF_MANIF_SE_2_3TANGENT_H_

#include "manif/impl/se_2_3/SE_2_3Tangent_base.h"

namespace manif {
namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<SE_2_3Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = SE_2_3<_Scalar>;
  using Tangent  = SE_2_3Tangent<_Scalar>;

  using Base = SE_2_3TangentBase<Tangent>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg   = Eigen::Matrix<Scalar, 5, 5>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif {

//
// Tangent
//

/**
 * @brief Represents an element of tangent space of SE_2_3.
 */
template <typename _Scalar>
struct SE_2_3Tangent : SE_2_3TangentBase<SE_2_3Tangent<_Scalar>>
{
private:

  using Base = SE_2_3TangentBase<SE_2_3Tangent<_Scalar>>;
  using Type = SE_2_3Tangent<_Scalar>;

protected:

    using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  SE_2_3Tangent()  = default;
  ~SE_2_3Tangent() = default;

  MANIF_COPY_CONSTRUCTOR(SE_2_3Tangent)
  MANIF_MOVE_CONSTRUCTOR(SE_2_3Tangent)

  template <typename _DerivedOther>
  SE_2_3Tangent(const TangentBase<_DerivedOther>& o);

  MANIF_TANGENT_ASSIGN_OP(SE_2_3Tangent)

  // Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // SE_2_3Tangent specific API

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SE_2_3Tangent);

template <typename _Scalar>
template <typename _DerivedOther>
SE_2_3Tangent<_Scalar>::SE_2_3Tangent(
    const TangentBase<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar>
typename SE_2_3Tangent<_Scalar>::DataType&
SE_2_3Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename SE_2_3Tangent<_Scalar>::DataType&
SE_2_3Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE_2_3TANGENT_H_ */
