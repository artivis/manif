#ifndef _MANIF_MANIF_SE3_H_
#define _MANIF_MANIF_SE3_H_

#include "manif/impl/se3/SE3_base.h"

#include <Eigen/Core>

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct SE3;
template <typename _Scalar> struct SE3Tangent;

namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<SE3<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = SE3<_Scalar>;
  using Tangent  = SE3Tangent<_Scalar>;

  using Base = SE3Base<SE3<_Scalar>>;

  static constexpr int Dim = LieGroupProperties<Base>::Dim;
  static constexpr int DoF = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = 7;

  /// @todo would be nice to concat vec3 + quaternion
  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, 4, 4>;
  using Rotation       = Eigen::Matrix<Scalar, Dim, Dim>;
  using Translation    = Eigen::Matrix<Scalar, Dim, 1>;
  using Vector         = Eigen::Matrix<Scalar, Dim, 1>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif {

//
// LieGroup
//

/**
 * @brief Represent an element of SE3.
 */
template <typename _Scalar>
struct SE3 : SE3Base<SE3<_Scalar>>
{
private:

  using Base = SE3Base<SE3<_Scalar>>;
  using Type = SE3<_Scalar>;

public:

  // Needed this underlying vector is size 7
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MANIF_COMPLETE_GROUP_TYPEDEF
  using Translation = typename Base::Translation;
  using Quaternion = Eigen::Quaternion<Scalar>;

  MANIF_INHERIT_GROUP_API

  using Base::normalize;

  SE3()  = default;
  ~SE3() = default;

  // Copy constructor given base
  SE3(const Base& o);

  template <typename _DerivedOther>
  SE3(const SE3Base<_DerivedOther>& o);

  template <typename _DerivedOther>
  SE3(const LieGroupBase<_DerivedOther>& o);

  // Copy constructor given Eigen
  template <typename _EigenDerived>
  SE3(const Eigen::MatrixBase<_EigenDerived>& data);

  /**
   * @brief Constructor given a translation and a unit quaternion.
   * @param[in] t A translation vector.
   * @param[in] q A unit quaternion.
   * @throws manif::invalid_argument on un-normalized complex number.
   */
  SE3(const Translation& t,
      const Eigen::Quaternion<Scalar>& q);

  /**
   * @brief Constructor given a translation and an angle axis.
   * @param[in] t A translation vector.
   * @param[in] angle_axis An angle-axis.
   */
  SE3(const Translation& t,
      const Eigen::AngleAxis<Scalar>& angle_axis);

  /**
   * @brief Constructor given a translation and SO3 element.
   * @param[in] t A translation vector.
   * @param[in] SO3 An element of SO3.
   */
  SE3(const Translation& t,
      const SO3<Scalar>& SO3);

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
  SE3(const Scalar x, const Scalar y, const Scalar z,
      const Scalar roll, const Scalar pitch, const Scalar yaw);

  /**
   * @brief Constructor from a 3D Eigen::Isometry<Scalar>
   * @param[in] h an isometry object from Eigen
   *
   * Isometry is a typedef from Eigen::Transform, in which the linear part is assumed a rotation matrix.
   * This is used to speed up certain methods of Transform, especially inverse().
   */
  SE3(const Eigen::Transform<_Scalar,3,Eigen::Isometry>& h);

  // LieGroup common API

  const DataType& coeffs() const;

  // SE3 specific API

protected:

  friend struct LieGroupBase<SE3<Scalar>>;
  DataType& coeffs_nonconst();

  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(SE3)

template <typename _Scalar>
template <typename _EigenDerived>
SE3<_Scalar>::SE3(const Eigen::MatrixBase<_EigenDerived>& data)
  : data_(data)
{
  using std::abs;
  MANIF_CHECK(abs(data_.template tail<4>().norm()-Scalar(1)) <
              Constants<Scalar>::eps_s,
              "SE3 constructor argument not normalized !",
              invalid_argument);
}

template <typename _Scalar>
SE3<_Scalar>::SE3(const Base& o)
  : SE3(o.coeffs())
{
  //
}

template <typename _Scalar>
template <typename _DerivedOther>
SE3<_Scalar>::SE3(
    const SE3Base<_DerivedOther>& o)
  : SE3(o.coeffs())
{
  //
}

template <typename _Scalar>
template <typename _DerivedOther>
SE3<_Scalar>::SE3(
    const LieGroupBase<_DerivedOther>& o)
  : SE3(o.coeffs())
{
  //
}

template <typename _Scalar>
SE3<_Scalar>::SE3(const Translation& t,
                  const Eigen::Quaternion<Scalar>& q)
  : SE3((DataType() << t, q.coeffs() ).finished())
{
  //
}

template <typename _Scalar>
SE3<_Scalar>::SE3(const Translation& t,
                  const Eigen::AngleAxis<Scalar>& a)
  : SE3(t, Quaternion(a))
{
  //
}

template <typename _Scalar>
SE3<_Scalar>::SE3(const Scalar x, const Scalar y, const Scalar z,
                  const Scalar roll, const Scalar pitch, const Scalar yaw)
  : SE3(Translation(x,y,z), Eigen::Quaternion<Scalar>(
          Eigen::AngleAxis<Scalar>(yaw,   Eigen::Matrix<Scalar, 3, 1>::UnitZ()) *
          Eigen::AngleAxis<Scalar>(pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()) *
          Eigen::AngleAxis<Scalar>(roll,  Eigen::Matrix<Scalar, 3, 1>::UnitX())  ))
{
  //
}

template <typename _Scalar>
SE3<_Scalar>::SE3(const Translation& t,
                  const SO3<Scalar>& so3)
  : SE3(t, so3.quat())
{
  //
}

template <typename _Scalar>
SE3<_Scalar>::SE3(const Eigen::Transform<_Scalar,3,Eigen::Isometry>& h)
  : SE3(h.translation(), Eigen::Quaternion<_Scalar>(h.rotation()))
{
  //
}


template <typename _Scalar>
typename SE3<_Scalar>::DataType&
SE3<_Scalar>::coeffs_nonconst()
{
  return data_;
}

template <typename _Scalar>
const typename SE3<_Scalar>::DataType&
SE3<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE3_H_ */
