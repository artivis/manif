#ifndef _MANIF_MANIF_RNTANGENT_H_
#define _MANIF_MANIF_RNTANGENT_H_

#include "manif/impl/rn/RnTangent_base.h"

namespace manif {
namespace internal {

//! Traits specialization
template <typename _Scalar, unsigned int _N>
struct traits<RnTangent<_Scalar, _N>>
{
  using Scalar = _Scalar;

  using LieGroup = Rn<_Scalar, _N>;
  using Tangent  = RnTangent<_Scalar, _N>;

  using Base = RnTangentBase<Tangent>;

  static constexpr int Dim     = _N;
  static constexpr int DoF     = _N;
  static constexpr int RepSize = _N;

  using DataType = Eigen::Matrix<Scalar, DoF, 1>;

  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg   = Eigen::Matrix<Scalar, DoF+1, DoF+1>;
};

} // namespace internal
} // namespace manif

namespace manif {

//
// Tangent
//

/**
 * @brief Represents an element of tangent space of Rn.
 */
template <typename _Scalar, unsigned int _N>
struct RnTangent : RnTangentBase<RnTangent<_Scalar, _N>>
{
private:

  static_assert(_N > 0, "N must be greater than 0 !");

  using Base = RnTangentBase<RnTangent<_Scalar, _N>>;
  using Type = RnTangent<_Scalar, _N>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  RnTangent()  = default;
  ~RnTangent() = default;

  MANIF_COPY_CONSTRUCTOR(RnTangent)
  MANIF_MOVE_CONSTRUCTOR(RnTangent)

  // Copy constructor given base
  template <typename _DerivedOther>
  RnTangent(const TangentBase<_DerivedOther>& o);

  MANIF_TANGENT_ASSIGN_OP(RnTangent)

  // Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

protected:

  DataType data_;
};

template <typename _Scalar> using R1Tangent = RnTangent<_Scalar, 1>;
template <typename _Scalar> using R2Tangent = RnTangent<_Scalar, 2>;
template <typename _Scalar> using R3Tangent = RnTangent<_Scalar, 3>;
template <typename _Scalar> using R4Tangent = RnTangent<_Scalar, 4>;
template <typename _Scalar> using R5Tangent = RnTangent<_Scalar, 5>;
template <typename _Scalar> using R6Tangent = RnTangent<_Scalar, 6>;
template <typename _Scalar> using R7Tangent = RnTangent<_Scalar, 7>;
template <typename _Scalar> using R8Tangent = RnTangent<_Scalar, 8>;
template <typename _Scalar> using R9Tangent = RnTangent<_Scalar, 9>;

MANIF_EXTRA_GROUP_TYPEDEF(R1Tangent)
MANIF_EXTRA_GROUP_TYPEDEF(R2Tangent)
MANIF_EXTRA_GROUP_TYPEDEF(R3Tangent)
MANIF_EXTRA_GROUP_TYPEDEF(R4Tangent)
MANIF_EXTRA_GROUP_TYPEDEF(R5Tangent)
MANIF_EXTRA_GROUP_TYPEDEF(R6Tangent)
MANIF_EXTRA_GROUP_TYPEDEF(R7Tangent)
MANIF_EXTRA_GROUP_TYPEDEF(R8Tangent)
MANIF_EXTRA_GROUP_TYPEDEF(R9Tangent)

template <typename _Scalar, unsigned int _N>
template <typename _DerivedOther>
RnTangent<_Scalar, _N>::RnTangent(const TangentBase<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar, unsigned int _N>
typename RnTangent<_Scalar, _N>::DataType&
RnTangent<_Scalar, _N>::coeffs()
{
  return data_;
}

template <typename _Scalar, unsigned int _N>
const typename RnTangent<_Scalar, _N>::DataType&
RnTangent<_Scalar, _N>::coeffs() const
{
  return data_;
}

} // namespace manif

#endif // _MANIF_MANIF_RNTANGENT_H_
