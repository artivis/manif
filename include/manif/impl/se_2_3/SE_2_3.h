#ifndef _MANIF_MANIF_SE_2_3_H_
#define _MANIF_MANIF_SE_2_3_H_

#include "manif/impl/se_2_3/SE_2_3_base.h"

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct SE_2_3;
template <typename _Scalar> struct SE_2_3Tangent;

namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<SE_2_3<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = SE_2_3<_Scalar>;
  using Tangent  = SE_2_3Tangent<_Scalar>;

  using Base = SE_2_3Base<SE_2_3<_Scalar>>;

  static constexpr int Dim = LieGroupProperties<Base>::Dim;
  static constexpr int DoF = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = 10;

  /// @todo would be nice to concat vec3 + quaternion + vec3
  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Rotation       = Eigen::Matrix<Scalar, Dim, Dim>;
  using Translation    = Eigen::Matrix<Scalar, Dim, 1>;
  using LinearVelocity = Eigen::Matrix<Scalar, Dim, 1>;
  using Vector         = Eigen::Matrix<Scalar, Dim, 1>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif {

//
// LieGroup
//

/**
 * @brief Represent an element of SE_2_3.
 */
template <typename _Scalar>
struct SE_2_3 : SE_2_3Base<SE_2_3<_Scalar>>
{
private:

  using Base = SE_2_3Base<SE_2_3<_Scalar>>;
  using Type = SE_2_3<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_COMPLETE_GROUP_TYPEDEF
  using Translation = typename Base::Translation;
  using Quaternion  = Eigen::Quaternion<Scalar>;
  using LinearVelocity = typename Base::LinearVelocity;

  MANIF_INHERIT_GROUP_API
  using Base::rotation;
  using Base::normalize;

  SE_2_3()  = default;
  ~SE_2_3() = default;

  MANIF_COPY_CONSTRUCTOR(SE_2_3)
  MANIF_MOVE_CONSTRUCTOR(SE_2_3)

  template <typename _DerivedOther>
  SE_2_3(const LieGroupBase<_DerivedOther>& o);

  MANIF_GROUP_ASSIGN_OP(SE_2_3)

  /**
   * @brief Constructor given a translation, a unit quaternion and a linear velocity.
   * @param[in] t A translation vector.
   * @param[in] q A unit quaternion.
   * @param[in] v A linear velocity vector.
   * @throws manif::invalid_argument on un-normalized complex number.
   */
  SE_2_3(const Translation& t,
         const Eigen::Quaternion<Scalar>& q,
         const LinearVelocity& v);

  /**
   * @brief Constructor given a translation, an angle axis and a linear velocity.
   * @param[in] t A translation vector.
   * @param[in] angle_axis An angle-axis.
   * @param[in] v A linear velocity vector.
   */
  SE_2_3(const Translation& t,
         const Eigen::AngleAxis<Scalar>& angle_axis,
         const LinearVelocity& v);

  /**
   * @brief Constructor given a translation, SO3 element and a linear velocity.
   * @param[in] t A translation vector.
   * @param[in] SO3 An element of SO3.
   * @param[in] v A linear velocity vector.
   */
  SE_2_3(const Translation& t,
         const SO3<Scalar>& SO3,
         const LinearVelocity& v);

  /**
   * @brief Constructor given translation components,
   * roll-pitch-yaw angles and linear velocity components
   * @param[in] x The x component of the translation.
   * @param[in] y The y component of the translation.
   * @param[in] z The z component of the translation.
   * @param[in] roll The roll angle.
   * @param[in] pitch The pitch angle.
   * @param[in] yaw The yaw angle.
   * @param[in] vx The x component of the linear velocity.
   * @param[in] vy The y component of the linear velocity.
   * @param[in] vz The z component of the linear velocity.
   */
  SE_2_3(const Scalar x, const Scalar y, const Scalar z,
         const Scalar roll, const Scalar pitch, const Scalar yaw,
         const Scalar vx, const Scalar vy, const Scalar vz);

  /**
   * @brief Constructor from a 3D Eigen::Isometry<Scalar> relevant to SE(3) and a linear velocity
   * @param[in] h a isometry object from Eigen defined for SE(3)
   * @param[in] v a linear velocity vector.
   * @note overall, this should be a double direct spatial isometry,
   */
  SE_2_3(const Eigen::Transform<_Scalar,3,Eigen::Isometry>& h, const LinearVelocity& v);

  // LieGroup common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // SE_2_3 specific API

protected:

  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(SE_2_3)

template <typename _Scalar>
template <typename _DerivedOther>
SE_2_3<_Scalar>::SE_2_3(
    const LieGroupBase<_DerivedOther>& o)
  : SE_2_3(o.coeffs())
{
  //
}

template <typename _Scalar>
SE_2_3<_Scalar>::SE_2_3(const Translation& t,
                  const Eigen::Quaternion<Scalar>& q,
                  const LinearVelocity& v)
  : SE_2_3((DataType() << t, q.coeffs(), v ).finished())
{
  //
}

template <typename _Scalar>
SE_2_3<_Scalar>::SE_2_3(const Translation& t,
                  const Eigen::AngleAxis<Scalar>& a,
                  const LinearVelocity& v)
  : SE_2_3(t, Quaternion(a), v)
{
  //
}

template <typename _Scalar>
SE_2_3<_Scalar>::SE_2_3(const Scalar x, const Scalar y, const Scalar z,
                  const Scalar roll, const Scalar pitch, const Scalar yaw,
                  const Scalar vx, const Scalar vy, const Scalar vz)
  : SE_2_3(Translation(x,y,z), Eigen::Quaternion<Scalar>(
          Eigen::AngleAxis<Scalar>(yaw,   Eigen::Matrix<Scalar, 3, 1>::UnitZ()) *
          Eigen::AngleAxis<Scalar>(pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()) *
          Eigen::AngleAxis<Scalar>(roll,  Eigen::Matrix<Scalar, 3, 1>::UnitX())  ), LinearVelocity(vx, vy, vz))
{
  //
}

template <typename _Scalar>
SE_2_3<_Scalar>::SE_2_3(const Translation& t,
                  const SO3<Scalar>& so3,
                  const LinearVelocity& v)
  : SE_2_3(t, so3.quat(), v)
{
  //
}

template <typename _Scalar>
SE_2_3<_Scalar>::SE_2_3(const Eigen::Transform<_Scalar,3,Eigen::Isometry>& h, const LinearVelocity& v)
  : SE_2_3(h.translation(), Eigen::Quaternion<_Scalar>(h.rotation()), v)
{
  //
}


template <typename _Scalar>
typename SE_2_3<_Scalar>::DataType&
SE_2_3<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename SE_2_3<_Scalar>::DataType&
SE_2_3<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE_2_3_H_ */
