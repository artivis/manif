#ifndef _MANIF_MANIF_DHU_H_
#define _MANIF_MANIF_DHU_H_

#include "manif/impl/dhu/DHu_base.h"

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct DHu;
template <typename _Scalar> struct DHuTangent;

namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<DHu<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = DHu<_Scalar>;
  using Tangent  = DHuTangent<_Scalar>;

  using Base = DHuBase<DHu<_Scalar>>;

  static constexpr int Dim = LieGroupProperties<Base>::Dim;
  static constexpr int DoF = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = 8;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, 4, 4>;
  using Rotation       = Eigen::Matrix<Scalar, Dim, Dim>;
  using Translation    = Eigen::Matrix<Scalar, Dim, 1>;
  using Vector         = Eigen::Matrix<Scalar, Dim, 1>;
};

} // namespace internal
} // namespace manif

namespace manif {

//
// LieGroup
//

/**
 * @brief Represent an element of DHu.
 */
template <typename _Scalar>
struct DHu : DHuBase<DHu<_Scalar>>
{
private:

  using Base = DHuBase<DHu<_Scalar>>;
  using Type = DHu<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_COMPLETE_GROUP_TYPEDEF
  using Translation = typename Base::Translation;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using Real = Quaternion;
  using Dual = Quaternion;

  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;
  using Base::normalize;

  DHu()  = default;
  ~DHu() = default;

  MANIF_COPY_CONSTRUCTOR(DHu)

  template <typename _DerivedOther>
  DHu(const LieGroupBase<_DerivedOther>& o);

  MANIF_GROUP_ASSIGN_OP(DHu)

  /**
   * @brief Constructor given a translation and a unit quaternion.
   * @param[in] t A translation vector.
   * @param[in] q A unit quaternion.
   * @throws manif::invalid_argument on un-normalized complex number.
   */
  DHu(const Real& real, const Dual& dual);

  /**
   * @brief Constructor given a translation and a unit quaternion.
   * @param[in] t A translation vector.
   * @param[in] q A unit quaternion.
   * @throws manif::invalid_argument on un-normalized complex number.
   */
  // DHu(const Translation& t,
  //     const Eigen::Quaternion<Scalar>& q);

  /**
   * @brief Constructor given a translation and an angle axis.
   * @param[in] t A translation vector.
   * @param[in] angle_axis An angle-axis.
   */
  // DHu(const Translation& t,
  //     const Eigen::AngleAxis<Scalar>& angle_axis);

  /**
   * @brief Constructor given a translation and SO3 element.
   * @param[in] t A translation vector.
   * @param[in] SO3 An element of SO3.
   */
  // DHu(const Translation& t,
  //     const SO3<Scalar>& SO3);

  /**
   * @brief Constructor given translation components and
   * roll-pitch-yaw angles.
   * @param[in] x The x component of the translation.
   * @param[in] y The y component of the translation.
   * @param[in] z The z component of the translation.
   * @param[in] roll The roll angle.
   * @param[in] pitch The pitch angle.
   * @param[in] yaw The yaw angle.
   */
  // DHu(const Scalar x, const Scalar y, const Scalar z,
      // const Scalar roll, const Scalar pitch, const Scalar yaw);

  /**
   * @brief Constructor from a 3D Eigen::Isometry<Scalar>
   * @param[in] h an isometry object from Eigen
   *
   * Isometry is a typedef from Eigen::Transform, in which the linear part is assumed a rotation matrix.
   * This is used to speed up certain methods of Transform, especially inverse().
   */
  DHu(const Eigen::Transform<_Scalar,3,Eigen::Isometry>& h);

  // LieGroup common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // DHu specific API

protected:

  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(DHu)

template <typename _Scalar>
template <typename _DerivedOther>
DHu<_Scalar>::DHu(const LieGroupBase<_DerivedOther>& o)
  : DHu(o.coeffs())
{
  //
}

template <typename _Scalar>
DHu<_Scalar>::DHu(const Real& real, const Dual& dual)
  : DHu((DataType() << real.coeffs(), dual.coeffs() ).finished())
{
  //
}

// template <typename _Scalar>
// DHu<_Scalar>::DHu(const Translation& t, const Eigen::Quaternion<Scalar>& q)
//   : DHu((DataType() << t, q.coeffs() ).finished())
// {
//   //
// }

// template <typename _Scalar>
// DHu<_Scalar>::DHu(const Translation& t, const Eigen::AngleAxis<Scalar>& a)
//   : DHu(t, Quaternion(a))
// {
//   //
// }

// template <typename _Scalar>
// DHu<_Scalar>::DHu(const Scalar x, const Scalar y, const Scalar z,
//                   const Scalar roll, const Scalar pitch, const Scalar yaw)
//   : DHu(Translation(x,y,z), Eigen::Quaternion<Scalar>(
//       Eigen::AngleAxis<Scalar>(yaw,   Eigen::Matrix<Scalar, 3, 1>::UnitZ()) *
//       Eigen::AngleAxis<Scalar>(pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()) *
//       Eigen::AngleAxis<Scalar>(roll,  Eigen::Matrix<Scalar, 3, 1>::UnitX())  ))
// {
//   //
// }

// template <typename _Scalar>
// DHu<_Scalar>::DHu(const Translation& t, const SO3<Scalar>& so3)
//   : DHu(t, so3.quat())
// {
//   //
// }

// template <typename _Scalar>
// DHu<_Scalar>::DHu(const Eigen::Transform<_Scalar,3,Eigen::Isometry>& h)
//   : DHu(h.translation(), Eigen::Quaternion<_Scalar>(h.rotation()))
// {
//   //
// }

template <typename _Scalar>
typename DHu<_Scalar>::DataType&
DHu<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename DHu<_Scalar>::DataType&
DHu<_Scalar>::coeffs() const
{
  return data_;
}

} // namespace manif

#endif // _MANIF_MANIF_DHU_H_
