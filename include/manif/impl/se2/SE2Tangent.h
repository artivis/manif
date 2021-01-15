#ifndef _MANIF_MANIF_SE2TANGENT_H_
#define _MANIF_MANIF_SE2TANGENT_H_

#include "manif/impl/se2/SE2Tangent_base.h"

namespace manif {
namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<SE2Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = SE2<_Scalar>;
  using Tangent  = SE2Tangent<_Scalar>;

  using Base = SE2TangentBase<Tangent>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, DoF, 1>;

  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg   = Eigen::Matrix<Scalar, 3, 3>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif {

//
// Tangent
//

/**
 * @brief Represent an element of the tangent space of SE2.
 */
template <typename _Scalar>
struct SE2Tangent : SE2TangentBase<SE2Tangent<_Scalar>>
{
private:

  using Base = SE2TangentBase<SE2Tangent<_Scalar>>;
  using Type = SE2Tangent<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  SE2Tangent()  = default;
  ~SE2Tangent() = default;

  MANIF_COPY_CONSTRUCTOR(SE2Tangent)
  MANIF_MOVE_CONSTRUCTOR(SE2Tangent)

  // Copy constructor given base
  template <typename _DerivedOther>
  SE2Tangent(const TangentBase<_DerivedOther>& o);

  MANIF_TANGENT_ASSIGN_OP(SE2Tangent)

  SE2Tangent(const Scalar x, const Scalar y, const Scalar theta);

  // Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // SE2Tangent specific API

  using Base::angle;

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SE2Tangent);

template <typename _Scalar>
template <typename _DerivedOther>
SE2Tangent<_Scalar>::SE2Tangent(const TangentBase<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar>
SE2Tangent<_Scalar>::SE2Tangent(const Scalar x,
                                const Scalar y,
                                const Scalar theta)
  : SE2Tangent(DataType(x, y, theta))
{
  //
}

template <typename _Scalar>
typename SE2Tangent<_Scalar>::DataType&
SE2Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename SE2Tangent<_Scalar>::DataType&
SE2Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE2TANGENT_H_ */
