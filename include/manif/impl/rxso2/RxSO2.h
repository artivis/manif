#ifndef _MANIF_MANIF_RxSO2_H_
#define _MANIF_MANIF_RxSO2_H_

#include "manif/impl/rxso2/RxSO2_base.h"

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct RxSO2;
template <typename _Scalar> struct RxSO2Tangent;

namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<RxSO2<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = RxSO2<_Scalar>;
  using Tangent  = RxSO2Tangent<_Scalar>;

  using Base = RxSO2Base<RxSO2<_Scalar>>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = 3;

  using DataType       = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, 3, 3>;
  using Rotation       = Eigen::Matrix<Scalar, Dim, Dim>;
  using Vector         = Eigen::Matrix<Scalar, Dim, 1>;
};

} // namespace internal
} // namespace manif

namespace manif {

//
// LieGroup
//

/**
 * @brief Represents an element of RxSO2.
 */
template <typename _Scalar>
struct RxSO2 : RxSO2Base<RxSO2<_Scalar>>
{
private:

  using Base = RxSO2Base<RxSO2<_Scalar>>;
  using Type = RxSO2<_Scalar>;

  using Scale = typename Base::Scale;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;
  using Base::normalize;

  RxSO2()  = default;
  ~RxSO2() = default;

  MANIF_COPY_CONSTRUCTOR(RxSO2)
  MANIF_MOVE_CONSTRUCTOR(RxSO2)

  // Copy constructor given base
  template <typename _DerivedOther>
  RxSO2(const LieGroupBase<_DerivedOther>& o);

  MANIF_GROUP_ASSIGN_OP(RxSO2)

  /**
   * @brief Constructor given the real and imaginary part
   * of a unit complex number representing the angle.
   * @param[in] real The real of a unitary complex number.
   * @param[in] imag The imaginary of a unitary complex number.
   * @param[in] scale The scale.
   * @throws manif::invalid_argument on un-normalized complex number.
   */
  RxSO2(const Scalar real, const Scalar imag, const Scalar scale);

  //! @brief Constructor given an angle (rad.) and scale
  RxSO2(const Scalar theta, const Scalar scale);

  DataType& coeffs();
  const DataType& coeffs() const;

protected:

  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(RxSO2)

template <typename _Scalar>
template <typename _DerivedOther>
RxSO2<_Scalar>::RxSO2(const LieGroupBase<_DerivedOther>& o)
  : RxSO2(o.coeffs())
{
  //
}

template <typename _Scalar>
RxSO2<_Scalar>::RxSO2(const Scalar real, const Scalar imag, const Scalar scale)
  : RxSO2(DataType(real, imag, scale))
{
  //
}

template <typename _Scalar>
RxSO2<_Scalar>::RxSO2(const Scalar theta, const Scalar scale)
  : RxSO2(cos(theta), sin(theta), scale)
{
  using std::cos;
  using std::sin;
}

template <typename _Scalar>
typename RxSO2<_Scalar>::DataType&
RxSO2<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename RxSO2<_Scalar>::DataType&
RxSO2<_Scalar>::coeffs() const
{
  return data_;
}

} // namespace manif

#endif // _MANIF_MANIF_RxSO2_H_
