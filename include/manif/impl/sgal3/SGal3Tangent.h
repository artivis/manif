#ifndef _MANIF_MANIF_SGAL3TANGENT_H_
#define _MANIF_MANIF_SGAL3TANGENT_H_

#include "manif/impl/sgal3/SGal3Tangent_base.h"

namespace manif {
namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<SGal3Tangent<_Scalar>> {
  using Scalar = _Scalar;

  using LieGroup = SGal3<_Scalar>;
  using Tangent  = SGal3Tangent<_Scalar>;

  using Base = SGal3TangentBase<Tangent>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg   = Eigen::Matrix<Scalar, 5, 5>;
};

} // namespace internal
} // namespace manif

namespace manif {

//
// Tangent
//

/**
 * @brief Represents an element of tangent space of SGal3.
 */
template <typename _Scalar>
struct SGal3Tangent : SGal3TangentBase<SGal3Tangent<_Scalar>> {
private:

  using Base = SGal3TangentBase<SGal3Tangent<_Scalar>>;
  using Type = SGal3Tangent<_Scalar>;

protected:

    using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  SGal3Tangent()  = default;
  ~SGal3Tangent() = default;

  MANIF_COPY_CONSTRUCTOR(SGal3Tangent)
  MANIF_MOVE_CONSTRUCTOR(SGal3Tangent)

  template <typename _DerivedOther>
  SGal3Tangent(const TangentBase<_DerivedOther>& o);

  MANIF_TANGENT_ASSIGN_OP(SGal3Tangent)

  // Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // SGal3Tangent specific API

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SGal3Tangent);

template <typename _Scalar>
template <typename _DerivedOther>
SGal3Tangent<_Scalar>::SGal3Tangent(const TangentBase<_DerivedOther>& o)
  : data_(o.coeffs()) {
  //
}

template <typename _Scalar>
typename SGal3Tangent<_Scalar>::DataType&
SGal3Tangent<_Scalar>::coeffs() {
  return data_;
}

template <typename _Scalar>
const typename SGal3Tangent<_Scalar>::DataType&
SGal3Tangent<_Scalar>::coeffs() const {
  return data_;
}

} // namespace manif

#endif // _MANIF_MANIF_SGAL3TANGENT_H_
