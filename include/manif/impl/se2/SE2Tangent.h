#ifndef _MANIF_MANIF_SE2TANGENT_H_
#define _MANIF_MANIF_SE2TANGENT_H_

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

  SE2Tangent()  = default;
  ~SE2Tangent() = default;

  // Copy constructor given base
  MANIF_COPY_CONSTRUCTOR(SE2Tangent)
  template <typename _DerivedOther>
  SE2Tangent(const TangentBase<_DerivedOther>& o);

  SE2Tangent(const Scalar x, const Scalar y, const Scalar theta);

  MANIF_TANGENT_API
  using Base::data;

  MANIF_COEFFS_FUNCTIONS

  MANIF_TANGENT_ASSIGN_OP(SE2Tangent)
  MANIF_TANGENT_OPERATOR

  // SE2Tangent specific API

  using Base::angle;
  using Base::x;
  using Base::y;

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

} /* namespace manif */

#endif /* _MANIF_MANIF_SE2TANGENT_H_ */
