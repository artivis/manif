#ifndef _MANIF_MANIF_DHUTANGENT_H_
#define _MANIF_MANIF_DHUTANGENT_H_

#include "manif/impl/dhu/DHuTangent_base.h"

namespace manif {
namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<DHuTangent<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = DHu<_Scalar>;
  using Tangent  = DHuTangent<_Scalar>;

  using Base = DHuTangentBase<Tangent>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg   = Eigen::Matrix<Scalar, 4, 4>;
};

} // namespace internal
} // namespace manif

namespace manif {

//
// Tangent
//

/**
 * @brief Represents an element of tangent space of DHu.
 */
template <typename _Scalar>
struct DHuTangent : DHuTangentBase<DHuTangent<_Scalar>>
{
private:

  using Base = DHuTangentBase<DHuTangent<_Scalar>>;
  using Type = DHuTangent<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  DHuTangent()  = default;
  ~DHuTangent() = default;

  MANIF_COPY_CONSTRUCTOR(DHuTangent)

  // Copy constructor given base
  template <typename _DerivedOther>
  DHuTangent(const TangentBase<_DerivedOther>& o);

  MANIF_TANGENT_ASSIGN_OP(DHuTangent)

  // Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // DHuTangent specific API

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(DHuTangent);

template <typename _Scalar>
template <typename _DerivedOther>
DHuTangent<_Scalar>::DHuTangent(
    const TangentBase<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar>
typename DHuTangent<_Scalar>::DataType&
DHuTangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename DHuTangent<_Scalar>::DataType&
DHuTangent<_Scalar>::coeffs() const
{
  return data_;
}

} // namespace manif

#endif //_MANIF_MANIF_DHUTANGENT_H_
