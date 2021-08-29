#ifndef _MANIF_MANIF_SIM2TANGENT_H_
#define _MANIF_MANIF_SIM2TANGENT_H_

#include "manif/impl/sim2/Sim2Tangent_base.h"

namespace manif {
namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<Sim2Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = Sim2<_Scalar>;
  using Tangent  = Sim2Tangent<_Scalar>;

  using Base = Sim2TangentBase<Tangent>;

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
 * @brief Represents an element of tangent space of Sim2.
 */
template <typename _Scalar>
struct Sim2Tangent : Sim2TangentBase<Sim2Tangent<_Scalar>>
{
private:

  using Base = Sim2TangentBase<Sim2Tangent<_Scalar>>;
  using Type = Sim2Tangent<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Sim2Tangent()  = default;
  ~Sim2Tangent() = default;

  MANIF_COPY_CONSTRUCTOR(Sim2Tangent)
  MANIF_MOVE_CONSTRUCTOR(Sim2Tangent)

  // Copy constructor given base
  template <typename _DerivedOther>
  Sim2Tangent(const TangentBase<_DerivedOther>& o);

  MANIF_TANGENT_ASSIGN_OP(Sim2Tangent)

  // Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // Sim2Tangent specific API

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(Sim2Tangent);

template <typename _Scalar>
template <typename _DerivedOther>
Sim2Tangent<_Scalar>::Sim2Tangent(const TangentBase<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar>
typename Sim2Tangent<_Scalar>::DataType& Sim2Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename Sim2Tangent<_Scalar>::DataType& Sim2Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} // namespace manif

#endif // _MANIF_MANIF_SIM2TANGENT_H_
