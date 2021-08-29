#ifndef _MANIF_MANIF_SIM3TANGENT_H_
#define _MANIF_MANIF_SIM3TANGENT_H_

#include "manif/impl/sim3/Sim3Tangent_base.h"

namespace manif {
namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<Sim3Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = Sim3<_Scalar>;
  using Tangent  = Sim3Tangent<_Scalar>;

  using Base = Sim3TangentBase<Tangent>;

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
 * @brief Represents an element of tangent space of Sim3.
 */
template <typename _Scalar>
struct Sim3Tangent : Sim3TangentBase<Sim3Tangent<_Scalar>>
{
private:

  using Base = Sim3TangentBase<Sim3Tangent<_Scalar>>;
  using Type = Sim3Tangent<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Sim3Tangent()  = default;
  ~Sim3Tangent() = default;

  MANIF_COPY_CONSTRUCTOR(Sim3Tangent)
  MANIF_MOVE_CONSTRUCTOR(Sim3Tangent)

  // Copy constructor given base
  template <typename _DerivedOther>
  Sim3Tangent(const TangentBase<_DerivedOther>& o);

  MANIF_TANGENT_ASSIGN_OP(Sim3Tangent)

  // Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // Sim3Tangent specific API

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(Sim3Tangent);

template <typename _Scalar>
template <typename _DerivedOther>
Sim3Tangent<_Scalar>::Sim3Tangent(
    const TangentBase<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar>
typename Sim3Tangent<_Scalar>::DataType&
Sim3Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename Sim3Tangent<_Scalar>::DataType&
Sim3Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} // namespace manif

#endif // _MANIF_MANIF_SIM3TANGENT_H_
