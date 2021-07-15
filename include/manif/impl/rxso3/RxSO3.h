#ifndef _MANIF_MANIF_RxSO3_H_
#define _MANIF_MANIF_RxSO3_H_

#include "manif/impl/rxso3/RxSO3_base.h"

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct RxSO3;
template <typename _Scalar> struct RxSO3Tangent;

namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<RxSO3<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = RxSO3<_Scalar>;
  using Tangent  = RxSO3Tangent<_Scalar>;

  using Base = RxSO3Base<RxSO3<_Scalar>>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = 5;

  using DataType       = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, 4, 4>;
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
 * @brief Represents an element of RxSO3.
 */
template <typename _Scalar>
struct RxSO3 : RxSO3Base<RxSO3<_Scalar>>
{
private:

  using Base = RxSO3Base<RxSO3<_Scalar>>;
  using Type = RxSO3<_Scalar>;

  using QuaternionDataType = typename Base::QuaternionDataType;
  using Scale = typename Base::Scale;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;
  using Base::quat;
  using Base::normalize;

  RxSO3()  = default;
  ~RxSO3() = default;

  MANIF_COPY_CONSTRUCTOR(RxSO3)
  MANIF_MOVE_CONSTRUCTOR(RxSO3)

  // Copy constructor given base
  template <typename _DerivedOther>
  RxSO3(const LieGroupBase<_DerivedOther>& o);

  MANIF_GROUP_ASSIGN_OP(RxSO3)

  /**
   * @brief Constructor given a scalar and a unit quaternion.
   * @param[in] scale A scalar representing the scale.
   * @param[in] q A unit quaternion.
   * @throws manif::invalid_argument on un-normalized quaternion.
   */
  RxSO3(const QuaternionDataType& q, const Scale scale);

  /**
   * @brief Constructor given the quaternion's coefficients.
   * @param[in] scale The scale.
   * @param[in] x The x-components of a unit quaternion.
   * @param[in] y The x-components of a unit quaternion.
   * @param[in] z The x-components of a unit quaternion.
   * @param[in] w The x-components of a unit quaternion.
   * @throws manif::invalid_argument on un-normalized quaternion.
   */
  RxSO3(const Scalar x, const Scalar y, const Scalar z, const Scalar w, const Scale scale);

  /**
   * @brief Constructor given a scale and an angle axis.
   */
  RxSO3(const Eigen::AngleAxis<Scalar>& angle_axis, const Scale scale);

  /**
   * @brief Constructor given a scale and Euler angles.
   */
  RxSO3(const Scalar roll, const Scalar pitch, const Scalar yaw, const Scale scale);

  DataType& coeffs();
  const DataType& coeffs() const;

protected:

  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(RxSO3)

template <typename _Scalar>
template <typename _DerivedOther>
RxSO3<_Scalar>::RxSO3(const LieGroupBase<_DerivedOther>& o)
  : RxSO3(o.coeffs())
{
  //
}

template <typename _Scalar>
RxSO3<_Scalar>::RxSO3(const QuaternionDataType& q, const Scale scale)
  : RxSO3((DataType() << q.coeffs(), scale).finished())
{
  //
}

template <typename _Scalar>
RxSO3<_Scalar>::RxSO3(
  const Scalar x, const Scalar y, const Scalar z, const Scalar w, const Scale scale
)
  : RxSO3((DataType() << x, y, z, w, scale).finished())
{
  //
}

template <typename _Scalar>
RxSO3<_Scalar>::RxSO3(const Eigen::AngleAxis<Scalar>& angle_axis, const Scale scale)
  : RxSO3(QuaternionDataType(angle_axis), scale)
{

}

template <typename _Scalar>
RxSO3<_Scalar>::RxSO3(
  const Scalar roll, const Scalar pitch, const Scalar yaw, const Scale scale
)
  : RxSO3(
    Eigen::AngleAxis<Scalar>(yaw,   Eigen::Matrix<Scalar, 3, 1>::UnitZ()) *
    Eigen::AngleAxis<Scalar>(pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()) *
    Eigen::AngleAxis<Scalar>(roll,  Eigen::Matrix<Scalar, 3, 1>::UnitX()),
    scale
  )
{
  //
}

template <typename _Scalar>
typename RxSO3<_Scalar>::DataType&
RxSO3<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename RxSO3<_Scalar>::DataType&
RxSO3<_Scalar>::coeffs() const
{
  return data_;
}

} // namespace manif

#endif // _MANIF_MANIF_RxSO3_H_
