#ifndef _MANIF_MANIF_SE_2_3_BASE_H_
#define _MANIF_MANIF_SE_2_3_BASE_H_

#include "manif/impl/se_2_3/SE_2_3_properties.h"
#include "manif/impl/lie_group_base.h"
#include "manif/impl/so3/SO3_map.h"
#include "manif/impl/se3/SE3_map.h"

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the SE_2_3 group.
 * @note See Appendix A2 in the paper "The Invariant Extended Kalman filter as a stable
observer".
 * However, note that the serialization used in that paper is different from that defined below
 * The paper uses a SE_2_3 definition as,
 *  X = |R v p|
 *      |  1  |
 *      |    1|
 * with a vector space serialization as (w, a, v)
 * Instead, here we define the SE_2_3 to be,
 *  X = |R p v|
 *      |  1  |
 *      |    1|
 * with a vector space serialization as (v, w, a)
 */
template <typename _Derived>
struct SE_2_3Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = SE_2_3Base<_Derived>;

public:

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Translation    = typename internal::traits<_Derived>::Translation;
  using LinearVelocity = typename internal::traits<_Derived>::LinearVelocity;
  using Isometry       = Eigen::Matrix<Scalar, 5, 5>; /**< Double direct spatial isometry*/
  using QuaternionDataType = Eigen::Quaternion<Scalar>;

  // LieGroup common API

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SE_2_3Base)

public:

  MANIF_GROUP_ML_ASSIGN_OP(SE_2_3Base)

  /**
   * @brief Get the inverse.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the SE_2_3 corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The SE_2_3 tangent of this.
   * @note This is the log() map in vector form.
   * @see SE_2_3Tangent.
   */
  Tangent log(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref log instead.
   */
  MANIF_DEPRECATED
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief Composition of this and another SE_2_3 element.
   * @param[in] m Another SE_2_3 element.
   * @param[out] -optional- J_mc_ma Jacobian of the composition wrt this.
   * @param[out] -optional- J_mc_mb Jacobian of the composition wrt m.
   * @return The composition of 'this . m'.
   */
  template <typename _DerivedOther>
  LieGroup compose(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  /**
   * @brief Get the action of the underlying SE(3) element on a 3d point
   * @note this method by default returns a rigid motion action on 3d points and
   * does not take into account the embedded linear velocity of total SE_2(3) element
   * @param[in]  v A 3D point.
   * @param[out] -optional- J_vout_m The Jacobian of the new object wrt this.
   * @param[out] -optional- J_vout_v The Jacobian of the new object wrt input object.
   */
  template <typename _EigenDerived>
  Eigen::Matrix<Scalar, 3, 1>
  act(const Eigen::MatrixBase<_EigenDerived> &v,
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 9>>> J_vout_m = {},
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_v = {}) const;

  /**
   * @brief Get the adjoint matrix of SE_2_3 at this.
   */
  Jacobian adj() const;

  // SE_2_3 specific functions

  /**
   * Get the isometry object (double direct isometry).
   * @note T = | R t v|
   *           |   1  |
   *           |     1|
   */
  Isometry isometry() const;

  /**
   * @brief Get the rotational part of this as a rotation matrix.
   */
  Rotation rotation() const;

  /**
   * @brief Get the rotational part of this as a quaternion.
   */
  QuaternionDataType quat() const;

  /**
   * @brief Get the translational part in vector form.
   */
  Translation translation() const;

  /**
   * @brief Get the x component of the translational part.
   */
  Scalar x() const;

  /**
   * @brief Get the y component of translational part.
   */
  Scalar y() const;

  /**
   * @brief Get the z component of translational part.
   */
  Scalar z() const;

  /**
   * @brief Get the linear velocity part in vector form.
   */
  LinearVelocity linearVelocity() const;

  /**
   * @brief Get the x component of the linear velocity part.
   */
  Scalar vx() const;

  /**
   * @brief Get the y component of linear velocity part.
   */
  Scalar vy() const;

  /**
   * @brief Get the z component of linear velocity part.
   */
  Scalar vz() const;

  /**
   * @brief Normalize the underlying quaternion.
   */
  void normalize();

public: /// @todo make protected

  Eigen::Map<const SO3<Scalar>> asSO3() const
  {
    return Eigen::Map<const SO3<Scalar>>(coeffs().data()+3);
  }

  Eigen::Map<SO3<Scalar>> asSO3()
  {
    return Eigen::Map<SO3<Scalar>>(coeffs().data()+3);
  }
};

template <typename _Derived>
typename SE_2_3Base<_Derived>::Isometry
SE_2_3Base<_Derived>::isometry() const
{
  Eigen::Matrix<Scalar, 5, 5> T = Eigen::Matrix<Scalar, 5, 5>::Identity();
  T.template topLeftCorner<3,3>() = rotation();
  T.template block<3, 1>(0, 3) = translation();
  T.template topRightCorner<3,1>() = linearVelocity();
  return T;
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::Rotation
SE_2_3Base<_Derived>::rotation() const
{
  return asSO3().rotation();
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::QuaternionDataType
SE_2_3Base<_Derived>::quat() const
{
  return asSO3().quat();
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::Translation
SE_2_3Base<_Derived>::translation() const
{
  return coeffs().template head<3>();
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::LinearVelocity
SE_2_3Base<_Derived>::linearVelocity() const
{
  return coeffs().template tail<3>();
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::LieGroup
SE_2_3Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  if (J_minv_m)
  {
    (*J_minv_m) = -adj();
  }

  const SO3<Scalar> so3inv = asSO3().inverse();

  return LieGroup(-so3inv.act(translation()),
                   so3inv,
                  -so3inv.act(linearVelocity()));
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::Tangent
SE_2_3Base<_Derived>::log(OptJacobianRef J_t_m) const
{
  const SO3Tangent<Scalar> so3tan = asSO3().log();

  Tangent tan((typename Tangent::DataType() <<
               so3tan.ljacinv()*translation(),
               so3tan.coeffs(),
               so3tan.ljacinv()*linearVelocity()).finished());

  if (J_t_m)
  {
    // Jr^-1
    (*J_t_m) = tan.rjacinv();
  }

  return tan;
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::Tangent
SE_2_3Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename SE_2_3Base<_Derived>::LieGroup
SE_2_3Base<_Derived>::compose(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  static_assert(
    std::is_base_of<SE_2_3Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from SE_2_3Base !");

  const auto& m_se_2_3 = static_cast<const SE_2_3Base<_DerivedOther>&>(m);

  if (J_mc_ma)
  {
    (*J_mc_ma) = m.inverse().adj();
  }

  if (J_mc_mb)
  {
    J_mc_mb->setIdentity();
  }

  return LieGroup(rotation()*m_se_2_3.translation() + translation(),
                  asSO3().compose(m_se_2_3.asSO3()).quat(),
                  rotation()*m_se_2_3.linearVelocity() + linearVelocity());
}

template <typename _Derived>
template <typename _EigenDerived>
Eigen::Matrix<typename SE_2_3Base<_Derived>::Scalar, 3, 1>
SE_2_3Base<_Derived>::act(const Eigen::MatrixBase<_EigenDerived> &v,
                          tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 9>>> J_vout_m,
                          tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_v) const
{
  assert_vector_dim(v, 3);

  const Rotation R(rotation());

  if (J_vout_m)
  {
    J_vout_m->template topLeftCorner<3,3>()  =  R;
    J_vout_m->template block<3,3>(0, 3) = -R * skew(v);
    J_vout_m->template topRightCorner<3,3>().setZero();
  }

  if (J_vout_v)
  {
    (*J_vout_v) = R;
  }

  return translation() + R * v;
}


template <typename _Derived>
typename SE_2_3Base<_Derived>::Jacobian
SE_2_3Base<_Derived>::adj() const
{
  ///
  /// this is
  ///       Ad(g) = | R T.R 0|
  ///               | 0  R  0|
  ///               | 0 V.R R|
  ///
  /// considering vee(log(g)) = (v;w;a)
  /// with T = [t]_x
  /// with V = [v]_x

  Jacobian Adj = Jacobian::Zero();
  Adj.template topLeftCorner<3,3>() = rotation();
  Adj.template bottomRightCorner<3,3>() =
      Adj.template topLeftCorner<3,3>();
  Adj.template block<3,3>(3,3) =
      Adj.template topLeftCorner<3,3>();

  Adj.template block<3,3>(0, 3) =
    skew(translation()) * Adj.template topLeftCorner<3,3>();
  Adj.template block<3,3>(6, 3) =
    skew(linearVelocity()) * Adj.template topLeftCorner<3,3>();

  return Adj;
}

// SE_2_3 specific function

template <typename _Derived>
typename SE_2_3Base<_Derived>::Scalar
SE_2_3Base<_Derived>::x() const
{
  return coeffs()(0);
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::Scalar
SE_2_3Base<_Derived>::y() const
{
  return coeffs()(1);
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::Scalar
SE_2_3Base<_Derived>::z() const
{
  return coeffs()(2);
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::Scalar
SE_2_3Base<_Derived>::vx() const
{
  return coeffs()(7);
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::Scalar
SE_2_3Base<_Derived>::vy() const
{
  return coeffs()(8);
}

template <typename _Derived>
typename SE_2_3Base<_Derived>::Scalar
SE_2_3Base<_Derived>::vz() const
{
  return coeffs()(9);
}

template <typename _Derived>
void SE_2_3Base<_Derived>::normalize()
{
  coeffs().template segment<4>(3).normalize();
}

namespace internal {

//! @brief Random specialization for SE_2_3Base objects.
template <typename Derived>
struct RandomEvaluatorImpl<SE_2_3Base<Derived>>
{
  template <typename T>
  static void run(T& m)
  {
    using Scalar      = typename SE_2_3Base<Derived>::Scalar;
    using Translation = typename SE_2_3Base<Derived>::Translation;
    using LinearVelocity = typename SE_2_3Base<Derived>::LinearVelocity;
    using LieGroup    = typename SE_2_3Base<Derived>::LieGroup;

    m = LieGroup(Translation::Random(), randQuat<Scalar>(), LinearVelocity::Random());
  }
};

//! @brief Assignment assert specialization for SE2Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<SE_2_3Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)
  {
    using std::abs;
    MANIF_ASSERT(
      abs(data.template segment<4>(3).norm()-typename SE_2_3Base<Derived>::Scalar(1)) <
      Constants<typename SE_2_3Base<Derived>::Scalar>::eps_s,
      "SE_2_3 assigned data not normalized !",
      manif::invalid_argument
    );
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SE_2_3_BASE_H_ */
