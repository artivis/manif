#ifndef _MANIF_MANIF_SO3_BASE_H_
#define _MANIF_MANIF_SO3_BASE_H_

#include "manif/impl/so3/SO3_properties.h"
#include "manif/impl/lie_group_base.h"
#include "manif/impl/utils.h"

namespace manif {

////////////////
///          ///
/// LieGroup ///
///          ///
////////////////

/**
 * @brief The base class of the SO3 group.
 * @note See Appendix B of the paper.
 */
template <typename _Derived>
struct SO3Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = SO3Base<_Derived>;

public:

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using QuaternionDataType = Eigen::Quaternion<Scalar>;

  // LieGroup common API

  /**
   * @brief Get the inverse of this.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   * @note q^-1 = q*. See Eq. (140).
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the SO3 corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The SO3 tangent of this.
   * @note This is the log() map in vector form.
   * @note See Eq. (133) & Eq. (144).
   * @see SO3Tangent.
   */
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief Composition of this and another SO3 element.
   * @param[in] m Another SO3 element.
   * @param[out] -optional- J_mc_ma Jacobian of the composition wrt this.
   * @param[out] -optional- J_mc_mb Jacobian of the composition wrt m.
   * @return The composition of 'this . m'.
   * @note Quaternion product.
   * @note See Eqs. (141,142).
   */
  template <typename _DerivedOther>
  LieGroup compose(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  /**
   * @brief Rotation action on a 3-vector.
   * @param  v A 2-vector.
   * @param[out] -optional- J_vout_m The Jacobian of the new object wrt this.
   * @param[out] -optional- J_vout_v The Jacobian of the new object wrt input object.
   * @return The rotated 3-vector.
   * @note See Eq (136), Eqs. (150,151)
   */
  template <typename _EigenDerived>
  Eigen::Matrix<Scalar, 3, 1>
  act(const Eigen::MatrixBase<_EigenDerived> &v,
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_m = {},
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_v = {}) const;

  /**
   * @brief Get the adjoint of SO3 at this.
   * @note See Eq. (139).
   */
  Jacobian adj() const;

  // SO3 specific functions

  /**
   * @brief Get the transformation matrix (3D isometry).
   * @note T = | R 0 |
   *           | 0 1 |
   */
  Transformation transform() const;

  //! @brief Get a rotation matrix.
  Rotation rotation() const;

  //! @brief Get the x component of the quaternion.
  Scalar x() const;
  //! @brief Get the y component of the quaternion.
  Scalar y() const;
  //! @brief Get the z component of the quaternion.
  Scalar z() const;
  //! @brief Get the w component of the quaternion.
  Scalar w() const;

  //! @brief Get quaternion.
  QuaternionDataType quat() const;

protected:

  using Base::coeffs_nonconst;

  void normalize();
};

template <typename _Derived>
typename SO3Base<_Derived>::Transformation
SO3Base<_Derived>::transform() const
{
  Transformation T = Transformation::Identity();
  T.template block<3,3>(0,0) = rotation();
  return T;
}

template <typename _Derived>
typename SO3Base<_Derived>::Rotation
SO3Base<_Derived>::rotation() const
{
  return quat().matrix();
}

template <typename _Derived>
typename SO3Base<_Derived>::LieGroup
SO3Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  if (J_minv_m)
  {
    *J_minv_m = -rotation();
  }

  /// @todo, conjugate doc :
  /// equal to the multiplicative inverse if
  /// the quaternion is normalized
  return LieGroup(quat().conjugate());
}

template <typename _Derived>
typename SO3Base<_Derived>::Tangent
SO3Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  using std::sqrt;
  using std::atan2;

  Tangent tan;
  Scalar lift_coeff;

  const Scalar sin_angle_squared = coeffs().template head<3>().squaredNorm();
  if (sin_angle_squared > Constants<Scalar>::eps)
  {
    const Scalar sin_angle = sqrt(sin_angle_squared);
    const Scalar cos_angle = w();

    /** @note If (cos_angle < 0) then angle >= pi/2 ,
     *  means : angle for angle_axis vector >= pi (== 2*angle)
     *   |-> results in correct rotation but not a normalized angle_axis vector
     *
     * In that case we observe that 2 * angle ~ 2 * angle - 2 * pi,
     * which is equivalent saying
     *
     * angle - pi = atan(sin(angle - pi), cos(angle - pi))
     *            = atan(-sin(angle), -cos(angle))
     */
    const Scalar two_angle = Scalar(2.0) * ((cos_angle < Scalar(0.0)) ?
                                 atan2(-sin_angle, -cos_angle) :
                                 atan2( sin_angle,  cos_angle));

    lift_coeff = two_angle / sin_angle;
  }
  else
  {
    // small-angle approximation
    lift_coeff = Scalar(2.0);
  }

  tan = Tangent(coeffs().template head<3>() * lift_coeff);

//  using std::atan2;
//  Scalar n = coeffs().template head<3>().norm();
//  Scalar angle(0);
//  typename Tangent::DataType axis(1,0,0);
//  if (n<Constants<Scalar>::eps)
//    n = coeffs().template head<3>().stableNorm();
//  if (n > Scalar(0))
//  {
//    angle = Scalar(2)*atan2(n, w());
//    axis  = coeffs().template head<3>() / n;
//  }

//  tan = Tangent(axis*angle);

  if (J_t_m)
  {
    Scalar theta2 = tan.coeffs().squaredNorm();
    typename Tangent::LieAlg W = tan.hat();
    if (theta2 <= Constants<Scalar>::eps)
      J_t_m->noalias() = Jacobian::Identity() + Scalar(0.5) * W; // Small angle approximation
    else
    {
      Scalar theta = sqrt(theta2);  // rotation angle
      Jacobian M;
      M.noalias() = (Scalar(1) / theta2 - (Scalar(1) + cos(theta)) / (Scalar(2) * theta * sin(theta))) * (W * W);
      J_t_m->noalias() = Jacobian::Identity() + Scalar(0.5) * W + M; //is this really more optimized?
    }
  }

  return tan;
}

template <typename _Derived>
template <typename _DerivedOther>
typename SO3Base<_Derived>::LieGroup
SO3Base<_Derived>::compose(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  static_assert(
    std::is_base_of<SO3Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from S03Base !");

  const auto& m_SO3 = static_cast<const SO3Base<_DerivedOther>&>(m);

  if (J_mc_ma)
  {
    *J_mc_ma = m_SO3.rotation().transpose();
  }

  if (J_mc_mb)
    J_mc_mb->setIdentity();

  return LieGroup(quat() * m_SO3.quat());
}

template <typename _Derived>
template <typename _EigenDerived>
Eigen::Matrix<typename SO3Base<_Derived>::Scalar, 3, 1>
SO3Base<_Derived>::act(const Eigen::MatrixBase<_EigenDerived> &v,
                       tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_m,
                       tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_v) const
{
  assert_vector_dim(v, 3);
  const Rotation R(rotation());

  if (J_vout_m)
  {
    (*J_vout_m) = -R * skew(v);
  }

  if (J_vout_v)
  {
    (*J_vout_v) = R;
  }

  return R * v;
}

template <typename _Derived>
typename SO3Base<_Derived>::Jacobian
SO3Base<_Derived>::adj() const
{
  return rotation();
}

/// SO3 specific

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::z() const
{
  return coeffs().z();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::w() const
{
  return coeffs().w();
}

template <typename _Derived>
typename SO3Base<_Derived>::QuaternionDataType
SO3Base<_Derived>::quat() const
{
  return QuaternionDataType(coeffs());
}

template <typename _Derived>
void SO3Base<_Derived>::normalize()
{
  coeffs().normalize();
}

namespace internal {

template <typename Derived>
struct GeneratorEvaluator<SO3Base<Derived>>
{
  static typename SO3Base<Derived>::Basis
  run(const int i)
  {
    MANIF_CHECK(i>=0 && i<SO3Base<Derived>::DoF,
                "Index i must be in [0,2]!");

    using Basis  = typename SO3Base<Derived>::Basis;
    using Scalar = typename SO3Base<Derived>::Scalar;

    switch (i)
    {
      case 0:
      {
        static Basis E0(
              (Basis() << Scalar(0), Scalar(0), Scalar( 0),
                          Scalar(0), Scalar(0), Scalar(-1),
                          Scalar(0), Scalar(1), Scalar( 0) ).finished());
        return E0;
      }
      case 1:
      {
        static Basis E1(
              (Basis() << Scalar( 0), Scalar(0), Scalar(1),
                          Scalar( 0), Scalar(0), Scalar(0),
                          Scalar(-1), Scalar(0), Scalar(0) ).finished());
        return E1;
      }
      case 2:
      {
        static Basis E2(
              (Basis() << Scalar(0), Scalar(-1), Scalar(0),
                          Scalar(1), Scalar( 0), Scalar(0),
                          Scalar(0), Scalar( 0), Scalar(0) ).finished());
        return E2;
      }
      default:
        MANIF_THROW("Index i must be in [0,2]!");
        break;
    }

    return Basis{};
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SO3_BASE_H_ */
