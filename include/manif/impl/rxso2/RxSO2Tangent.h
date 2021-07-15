#ifndef _MANIF_MANIF_RxSO2TANGENT_H_
#define _MANIF_MANIF_RxSO2TANGENT_H_

#include "manif/impl/rxso2/RxSO2Tangent_base.h"

namespace manif {
namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<RxSO2Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = RxSO2<_Scalar>;
  using Tangent  = RxSO2Tangent<_Scalar>;

  using Base = RxSO2TangentBase<Tangent>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg   = Eigen::Matrix<Scalar, 2, 2>;
};

} // namespace internal
} // namespace manif

namespace manif {

//
// Tangent
//

/**
 * @brief Represents an element of tangent space of RxSO2.
 */
template <typename _Scalar>
struct RxSO2Tangent : RxSO2TangentBase<RxSO2Tangent<_Scalar>>
{
private:

  using Base = RxSO2TangentBase<RxSO2Tangent<_Scalar>>;
  using Type = RxSO2Tangent<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  RxSO2Tangent()  = default;
  ~RxSO2Tangent() = default;

  MANIF_COPY_CONSTRUCTOR(RxSO2Tangent)
  MANIF_MOVE_CONSTRUCTOR(RxSO2Tangent)

  // Copy constructor given base
  template <typename _DerivedOther>
  RxSO2Tangent(const TangentBase<_DerivedOther>& o);

  MANIF_TANGENT_ASSIGN_OP(RxSO2Tangent)

  //! @brief Constructor given angle (rad.) and sigma.
  RxSO2Tangent(const Scalar theta, const Scalar sigma);

  // Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // RxSO2Tangent specific API

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(RxSO2Tangent);

template <typename _Scalar>
template <typename _DerivedOther>
RxSO2Tangent<_Scalar>::RxSO2Tangent(const TangentBase<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar>
RxSO2Tangent<_Scalar>::RxSO2Tangent(const Scalar theta, const Scalar sigma)
  : data_(theta, sigma)
{
  //
}

template <typename _Scalar>
typename RxSO2Tangent<_Scalar>::DataType&
RxSO2Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename RxSO2Tangent<_Scalar>::DataType&
RxSO2Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} // namespace manif

#endif // _MANIF_MANIF_RxSO2TANGENT_H_
