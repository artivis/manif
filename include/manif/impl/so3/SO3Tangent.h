#ifndef _MANIF_MANIF_SO3TANGENT_H_
#define _MANIF_MANIF_SO3TANGENT_H_

namespace manif {
namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<SO3Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = SO3<_Scalar>;
  using Tangent  = SO3Tangent<_Scalar>;

  using Base = SO3TangentBase<Tangent>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

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
 * @brief Represents an element of tangent space of SO3.
 */
template <typename _Scalar>
struct SO3Tangent : SO3TangentBase<SO3Tangent<_Scalar>>
{
private:

  using Base = SO3TangentBase<SO3Tangent<_Scalar>>;
  using Type = SO3Tangent<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF

  SO3Tangent()  = default;
  ~SO3Tangent() = default;

  // Copy constructor
  MANIF_COPY_CONSTRUCTOR(SO3Tangent)
  template <typename _DerivedOther>
  SO3Tangent(const TangentBase<_DerivedOther>& o);

  MANIF_TANGENT_API
  using Base::data;

  MANIF_COEFFS_FUNCTIONS

  MANIF_TANGENT_ASSIGN_OP(SO3Tangent)
  MANIF_TANGENT_OPERATOR

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SO3Tangent);

template <typename _Scalar>
template <typename _DerivedOther>
SO3Tangent<_Scalar>::SO3Tangent(const TangentBase<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3TANGENT_H_ */
