#ifndef _MANIF_MANIF_RxSO3TANGENT_H_
#define _MANIF_MANIF_RxSO3TANGENT_H_

#include "manif/impl/rxso3/RxSO3Tangent_base.h"

namespace manif {
namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<RxSO3Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = RxSO3<_Scalar>;
  using Tangent  = RxSO3Tangent<_Scalar>;

  using Base = RxSO3TangentBase<Tangent>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg   = Eigen::Matrix<Scalar, 3, 3>;
};

} // namespace internal
} // namespace manif

namespace manif {

//
// Tangent
//

/**
 * @brief Represents an element of tangent space of RxSO3.
 */
template <typename _Scalar>
struct RxSO3Tangent : RxSO3TangentBase<RxSO3Tangent<_Scalar>>
{
private:

  using Base = RxSO3TangentBase<RxSO3Tangent<_Scalar>>;
  using Type = RxSO3Tangent<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  RxSO3Tangent()  = default;
  ~RxSO3Tangent() = default;

  MANIF_COPY_CONSTRUCTOR(RxSO3Tangent)
  MANIF_MOVE_CONSTRUCTOR(RxSO3Tangent)

  // Copy constructor given base
  template <typename _DerivedOther>
  RxSO3Tangent(const TangentBase<_DerivedOther>& o);

  MANIF_TANGENT_ASSIGN_OP(RxSO3Tangent)

  // Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // RxSO3Tangent specific API

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(RxSO3Tangent);

template <typename _Scalar>
template <typename _DerivedOther>
RxSO3Tangent<_Scalar>::RxSO3Tangent(const TangentBase<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar>
typename RxSO3Tangent<_Scalar>::DataType&
RxSO3Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename RxSO3Tangent<_Scalar>::DataType&
RxSO3Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} // namespace manif

#endif // _MANIF_MANIF_RxSO3TANGENT_H_
