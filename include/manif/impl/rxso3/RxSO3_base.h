#ifndef _MANIF_MANIF_RxSO3_BASE_H_
#define _MANIF_MANIF_RxSO3_BASE_H_

#include "manif/impl/rxso3/RxSO3_properties.h"
#include "manif/impl/lie_group_base.h"
#include "manif/impl/so3/SO3_map.h"

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the RxSO3 group.
 */
template <typename _Derived>
struct RxSO3Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = RxSO3Base<_Derived>;

public:

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Scale          = Scalar;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using QuaternionDataType = Eigen::Quaternion<Scalar>;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(RxSO3Base)

public:

  MANIF_GROUP_ML_ASSIGN_OP(RxSO3Base)

  // LieGroup common API

  /**
   * @brief Get the inverse of this.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the RxSO3 corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The RxSO3 tangent of this.
   * @see RxSO3Tangent.
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
   * @brief Composition of this and another RxSO3 element.
   * @param[in] m Another RxSO3 element.
   * @param[out] -optional- J_mc_ma Jacobian of the composition wrt this.
   * @param[out] -optional- J_mc_mb Jacobian of the composition wrt m.
   * @return The composition of 'this . m'.
   */
  template <typename _DerivedOther>
  LieGroup compose(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  /**
   * @brief Action on a point.
   * @param  v A point.
   * @param[out] -optional- J_vout_m The Jacobian of the new object wrt this.
   * @param[out] -optional- J_vout_v The Jacobian of the new object wrt input object.
   * @return The transformed point.
   */
  template <typename _EigenDerived>
  Eigen::Matrix<Scalar, 3, 1>
  act(const Eigen::MatrixBase<_EigenDerived> &v,
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m = {},
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v = {}) const;

  /**
   * @brief Get the adjoint of RxSO3 at this.
   */
  Jacobian adj() const;

  // RxSO3 specific functions

  /**
   * @brief Get the transformation matrix (3D isometry).
   * @note T = | s.R 0 |
   *           |  0  1 |
   */
  Transformation transform() const;

  //! @brief Get a rotation matrix.
  Rotation rotation() const;

  //! @brief Get the scale.
  Scale scale() const;

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

  /**
   * @brief Normalize the underlying quaternion.
   */
  void normalize();

  /**
   * @brief Set the rotational as a quaternion.
   * @param quaternion a unitary quaternion
   */
  void quat(const QuaternionDataType& quaternion);

  /**
   * @brief Set the rotational as a quaternion.
   * @param quaternion an Eigen::Vector representing a unitary quaternion
   */
  template <typename _EigenDerived>
  void quat(const Eigen::MatrixBase<_EigenDerived>& quaternion);

protected:

  Eigen::Map<SO3<Scalar>> asSO3();
  Eigen::Map<const SO3<Scalar>> asSO3() const;
};

template <typename _Derived>
typename RxSO3Base<_Derived>::Transformation
RxSO3Base<_Derived>::transform() const
{
  Transformation T = Transformation::Identity();
  T.template topLeftCorner<3, 3>() = scale() * rotation();
  return T;
}

template <typename _Derived>
typename RxSO3Base<_Derived>::Rotation
RxSO3Base<_Derived>::rotation() const
{
  return quat().matrix();
}

template <typename _Derived>
typename RxSO3Base<_Derived>::Scale
RxSO3Base<_Derived>::scale() const
{
  return coeffs()(4);
}

template <typename _Derived>
typename RxSO3Base<_Derived>::LieGroup
RxSO3Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  if (J_minv_m) {
    *J_minv_m = -adj();
  }

  Scale inv_s = scale() > Constants<Scalar>::eps?
    Scalar(1) / scale() : Scalar(2) - scale(); // Taylor order 2

  return LieGroup(quat().conjugate(), inv_s);
}

template <typename _Derived>
typename RxSO3Base<_Derived>::Tangent
RxSO3Base<_Derived>::log(OptJacobianRef J_t_m) const
{
  using std::log;

  if (J_t_m) {
    J_t_m->setIdentity();
    // J_t_m->template bottomRows<1>().template head<2>().setZero();
    // J_t_m->template leftCols<1>().template head<2>().setZero();
    // J_t_m->template bottomRightCorner<1,1>().setIdentity();

    return Tangent((
      typename Tangent::DataType() <<
        asSO3().log(J_t_m->template topLeftCorner<3, 3>()).coeffs(),
        log(scale())
    ).finished());
  }

  return Tangent((
    typename Tangent::DataType() << asSO3().log().coeffs(), log(scale())
  ).finished());
}

template <typename _Derived>
typename RxSO3Base<_Derived>::Tangent
RxSO3Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename RxSO3Base<_Derived>::LieGroup
RxSO3Base<_Derived>::compose(
  const LieGroupBase<_DerivedOther>& m,
  OptJacobianRef J_mc_ma,
  OptJacobianRef J_mc_mb
) const {
  using std::abs;

  static_assert(
    std::is_base_of<RxSO3Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from RxS03Base !");

  const auto& m_RxSO3 = static_cast<const RxSO3Base<_DerivedOther>&>(m);

  if (J_mc_ma) {
    *J_mc_ma = m.inverse().adj();
  }

  if (J_mc_mb) {
    J_mc_mb->setIdentity();
  }

  QuaternionDataType ret_q = quat() * m_RxSO3.quat();

  const Scalar ret_sqnorm = ret_q.squaredNorm();

  if (abs(ret_sqnorm-Scalar(1)) > Constants<Scalar>::eps) {
    ret_q.coeffs() *= approxSqrtInv(ret_sqnorm);
  }

  return LieGroup(ret_q, scale() * m_RxSO3.scale());
}

template <typename _Derived>
template <typename _EigenDerived>
Eigen::Matrix<typename RxSO3Base<_Derived>::Scalar, 3, 1>
RxSO3Base<_Derived>::act(
  const Eigen::MatrixBase<_EigenDerived> &v,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v
) const {
  assert_vector_dim(v, 3);
  const Rotation sR = scale() * rotation();

  const Vector vout = sR * v;

  if (J_vout_m) {
    J_vout_m->template topLeftCorner<3, 3>().noalias() = -sR * skew(v);
    J_vout_m->template rightCols<1>() = vout;
  }

  if (J_vout_v) {
    *J_vout_v = sR;
  }

  return vout;
}

template <typename _Derived>
typename RxSO3Base<_Derived>::Jacobian
RxSO3Base<_Derived>::adj() const
{
  Jacobian adj;
  adj.template topLeftCorner<3, 3>() = rotation();
  adj.template bottomRows<1>().template head<3>().setZero();
  adj.template rightCols<1>().template head<3>().setZero();
  adj(3, 3) = Scalar(1);
  return adj;
}

// RxSO3 specific

template <typename _Derived>
typename RxSO3Base<_Derived>::Scalar
RxSO3Base<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename RxSO3Base<_Derived>::Scalar
RxSO3Base<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename RxSO3Base<_Derived>::Scalar
RxSO3Base<_Derived>::z() const
{
  return coeffs().z();
}

template <typename _Derived>
typename RxSO3Base<_Derived>::Scalar
RxSO3Base<_Derived>::w() const
{
  return coeffs().w();
}

template <typename _Derived>
typename RxSO3Base<_Derived>::QuaternionDataType
RxSO3Base<_Derived>::quat() const
{
  return QuaternionDataType(coeffs().template head<4>());
}

template <typename _Derived>
void RxSO3Base<_Derived>::normalize()
{
  coeffs().template head<4>().normalize();
}

template <typename _Derived>
void RxSO3Base<_Derived>::quat(const QuaternionDataType& quaternion)
{
  quat(quaternion.coeffs());
}

template <typename _Derived>
template <typename _EigenDerived>
void RxSO3Base<_Derived>::quat(const Eigen::MatrixBase<_EigenDerived>& quaternion)
{
  using std::abs;
  assert_vector_dim(quaternion, 4);
  MANIF_ASSERT(abs(quaternion.norm()-Scalar(1)) <
               Constants<Scalar>::eps,
               "The quaternion is not normalized !",
               invalid_argument);

  coeffs().template head<4>() = quaternion;
}

template <typename _Derived>
Eigen::Map<const SO3<typename RxSO3Base<_Derived>::Scalar>>
RxSO3Base<_Derived>::asSO3() const
{
  return Eigen::Map<const SO3<Scalar>>(coeffs().data());
}

template <typename _Derived>
Eigen::Map<SO3<typename RxSO3Base<_Derived>::Scalar>>
RxSO3Base<_Derived>::asSO3()
{
  return Eigen::Map<SO3<Scalar>>(coeffs().data());
}

namespace internal {

//! @brief Random specialization for SO3Base objects.
template <typename Derived>
struct RandomEvaluatorImpl<RxSO3Base<Derived>>
{
  template <typename T>
  static void run(T& m)
  {
    using Scalar = typename RxSO3Base<Derived>::Scalar;

    m.coeffs()(4) = Eigen::internal::random<Scalar>(0, 10);
    m.quat(randQuat<Scalar>());
  }
};

//! @brief Assignment assert specialization for SE2Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<RxSO3Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)
  {
    using Scalar = typename RxSO3Base<Derived>::Scalar;
    using std::abs;
    MANIF_ASSERT(
      abs(data.template head<4>().norm()-Scalar(1)) < Constants<Scalar>::eps,
      "RxSO3 assigned data (quat) not normalized !",
      manif::invalid_argument
    );
    MANIF_ASSERT(
      data(4) > Scalar(0),
      "RxSO3 assigned data (scale) can not be negative nor zero!",
      manif::invalid_argument
    );
    MANIF_UNUSED_VARIABLE(data);
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_RxSO3_BASE_H_
