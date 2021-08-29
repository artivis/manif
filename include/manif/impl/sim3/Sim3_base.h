#ifndef _MANIF_MANIF_SIM3_BASE_H_
#define _MANIF_MANIF_SIM3_BASE_H_

#include "manif/impl/sim3/Sim3_properties.h"
#include "manif/impl/lie_group_base.h"
#include "manif/impl/rxso3/RxSO3_map.h"

#include <iostream>

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the Sim3 group.
 */
template <typename _Derived>
struct Sim3Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = Sim3Base<_Derived>;

public:

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Translation    = typename internal::traits<_Derived>::Translation;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using Isometry       = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
  using Scale          = Scalar;
  using QuaternionDataType = Eigen::Quaternion<Scalar>;

  // LieGroup common API

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(Sim3Base)

public:

  MANIF_GROUP_ML_ASSIGN_OP(Sim3Base)

  /**
   * @brief Get the inverse.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the Sim3 corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The Sim3 tangent of this.
   * @note This is the log() map in vector form.
   * @see Sim3Tangent.
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
   * @brief Composition of this and another Sim3 element.
   * @param[in] m Another Sim3 element.
   * @param[out] -optional- J_mc_ma Jacobian of the composition wrt this.
   * @param[out] -optional- J_mc_mb Jacobian of the composition wrt m.
   * @return The composition of 'this . m'.
   */
  template <typename _DerivedOther>
  LieGroup compose(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma = {},
    OptJacobianRef J_mc_mb = {}
  ) const;

  /**
   * @brief Similarity action on a 3D point.
   * @param v A 3D point.
   * @param[out] -optional- J_vout_m The Jacobian of the new object wrt this.
   * @param[out] -optional- J_vout_v The Jacobian of the new object wrt input object.
   * @return The transformed 3D point.
   */
  template <typename _EigenDerived>
  Eigen::Matrix<Scalar, 3, 1>
  act(
    const Eigen::MatrixBase<_EigenDerived> &v,
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m = {},
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v = {}
  ) const;

  /**
   * @brief Get the adjoint matrix of Sim3 at this.
   */
  Jacobian adj() const;

  // Sim3 specific functions

  /**
   * Get the transformation matrix (3D isometry).
   * @note T = | sR t |
   *           | 0  1 |
   */
  Transformation transform() const;

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
   * @brief Get the z component of translational part.
   */
  Scale scale() const;

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

  /**
   * @brief Set the rotational as a so3 object.
   * @param so3 a manif::SO3 object
   */
  // void quat(const SO3<Scalar>& so3);

  /**
   * @brief Set the translation of the Sim3 object
   * @param translation, 3d-vector representing the translation
   */
  void translation(const Translation& translation);

  /**
   * @brief Set the translation of the Sim3 object
   * @param translation, 3d-vector representing the translation
   */
  void scale(const Scale scale);

// protected:

  Eigen::Map<RxSO3<Scalar>> asRxSO3();
  Eigen::Map<const RxSO3<Scalar>> asRxSO3() const;
};

template <typename _Derived>
typename Sim3Base<_Derived>::Transformation
Sim3Base<_Derived>::transform() const {
  Transformation T = Transformation::Identity();
  T.template topLeftCorner<3,3>()  = scale() * rotation();
  T.template topRightCorner<3,1>() = translation();
  return T;
}

template <typename _Derived>
typename Sim3Base<_Derived>::Rotation
Sim3Base<_Derived>::rotation() const {
  return asRxSO3().rotation();
}

template <typename _Derived>
typename Sim3Base<_Derived>::QuaternionDataType
Sim3Base<_Derived>::quat() const {
  return asRxSO3().quat();
}

template <typename _Derived>
typename Sim3Base<_Derived>::Translation
Sim3Base<_Derived>::translation() const {
  return coeffs().template head<3>();
}

template <typename _Derived>
void Sim3Base<_Derived>::quat(const QuaternionDataType& quaternion) {
  quat(quaternion.coeffs());
}

template <typename _Derived>
template <typename _EigenDerived>
void Sim3Base<_Derived>::quat(const Eigen::MatrixBase<_EigenDerived>& quaternion) {
  using std::abs;
  assert_vector_dim(quaternion, 4);
  MANIF_ASSERT(abs(quaternion.norm()-Scalar(1)) <
               Constants<Scalar>::eps,
               "The quaternion is not normalized !",
               invalid_argument);

  asRxSO3().coeffs() = quaternion;
}

// template <typename _Derived>
// void Sim3Base<_Derived>::quat(const SO3<Scalar>& so3)
// {
//   quat(so3.coeffs());
// }

template <typename _Derived>
void Sim3Base<_Derived>::translation(const Translation& translation) {
  coeffs().template head<3>() = translation;
}

template <typename _Derived>
void Sim3Base<_Derived>::scale(const Scale scale) {
  MANIF_ASSERT(
    scale > Scalar(0),
    "Sim3::scale assigned data (scale) can not be negative nor zero!",
    manif::invalid_argument
  );
  coeffs()(7) = scale;
}

template <typename _Derived>
typename Sim3Base<_Derived>::LieGroup
Sim3Base<_Derived>::inverse(OptJacobianRef J_minv_m) const {
  if (J_minv_m) {
    (*J_minv_m) = -adj();
  }

  const RxSO3<Scalar> RxSO3inv = asRxSO3().inverse();

  return LieGroup(-RxSO3inv.act(translation()), RxSO3inv);
}

template <typename _Derived>
typename Sim3Base<_Derived>::Tangent
Sim3Base<_Derived>::log(OptJacobianRef J_t_m) const {
  const RxSO3Tangent<Scalar> RxSO3tan = asRxSO3().log();

  Tangent tan((
    typename Tangent::DataType() <<
    Tangent::computeW(RxSO3tan.ang(), RxSO3tan.s()).inverse()*translation(),
    RxSO3tan.coeffs()
  ).finished());

  // Tangent tan(
  //   Tangent::computeW(RxSO3tan.ang(), RxSO3tan.s()).inverse()*translation(),
  //   RxSO3tan
  // );

  if (J_t_m) {
    *J_t_m = tan.rjacinv();
  }

  return tan;
}

template <typename _Derived>
typename Sim3Base<_Derived>::Tangent
Sim3Base<_Derived>::lift(OptJacobianRef J_t_m) const {
  return log(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename Sim3Base<_Derived>::LieGroup
Sim3Base<_Derived>::compose(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb
) const {
  static_assert(
    std::is_base_of<Sim3Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from Sim3Base !");

  const auto& m_sim3 = static_cast<const Sim3Base<_DerivedOther>&>(m);

  if (J_mc_ma) {
    (*J_mc_ma) = m.inverse().adj();
  }

  if (J_mc_mb) {
    J_mc_mb->setIdentity();
  }

  const Eigen::Map<const RxSO3<Scalar>> RxSO3 = asRxSO3();

  return LieGroup(
    translation() + RxSO3.act(m_sim3.translation()),
    RxSO3.compose(m_sim3.asRxSO3())
  );
}

template <typename _Derived>
template <typename _EigenDerived>
Eigen::Matrix<typename Sim3Base<_Derived>::Scalar, 3, 1>
Sim3Base<_Derived>::act(
  const Eigen::MatrixBase<_EigenDerived> &v,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v
) const {
  assert_vector_dim(v, 3);
  const Rotation sR = scale() * rotation();

  if (J_vout_m) {
    J_vout_m->template topLeftCorner<3,3>()  =  sR;
    J_vout_m->template topRightCorner<3,3>() = -sR * skew(v);
  }

  if (J_vout_v) {
    (*J_vout_v) = sR;
  }

  return translation() + sR * v;
}

template <typename _Derived>
typename Sim3Base<_Derived>::Jacobian
Sim3Base<_Derived>::adj() const {
  /// this is
  ///       Ad(g) = | sR T.R -t |
  ///               | 0   R   0 |
  ///               | 0   0   1 |
  ///
  /// considering vee(log(g)) = (v;w;s)

  Jacobian Adj;
  Adj.template block<3,3>(3, 3) = rotation();

  Adj.template topLeftCorner<3, 3>() = scale() * Adj.template block<3,3>(3, 3);
  Adj.template block<3, 3>(0, 3).noalias() =
    skew(translation()) * Adj.template block<3,3>(3, 3);
  Adj.template topRightCorner<3, 1>() = -translation();

  Adj.template bottomLeftCorner<4,3>().setZero();
  Adj.template block<3,1>(3, 6).setZero();

  Adj.template block<1,3>(6, 3).setZero();
  Adj(6, 6) = Scalar(1);

  return Adj;

  /// this is
  ///       Ad(g) = | sR sT.R -st |
  ///               | 0   R    0  |
  ///               | 0   0    1  |
  ///
  /// considering vee(log(g)) = (v;w;s)

  // Jacobian Adj;
  // Adj.template block<3,3>(3, 3) = rotation();

  // Adj.template topLeftCorner<3, 3>() = scale() * Adj.template block<3,3>(3, 3);
  // Adj.template block<3, 3>(0, 3).noalias() =
  //   scale() * skew(translation()) * Adj.template block<3,3>(3, 3);
  // Adj.template topRightCorner<3, 1>() = -scale() * translation();

  // Adj.template bottomLeftCorner<4,3>().setZero();
  // Adj.template block<3,1>(3, 6).setZero();

  // Adj.template block<1,3>(6, 3).setZero();
  // Adj(6, 6) = Scalar(1);

  // return Adj;
}

// Sim3 specific function

template <typename _Derived>
typename Sim3Base<_Derived>::Scalar
Sim3Base<_Derived>::x() const {
  return coeffs().x();
}

template <typename _Derived>
typename Sim3Base<_Derived>::Scalar
Sim3Base<_Derived>::y() const {
  return coeffs().y();
}

template <typename _Derived>
typename Sim3Base<_Derived>::Scalar
Sim3Base<_Derived>::z() const {
  return coeffs().z();
}

template <typename _Derived>
typename Sim3Base<_Derived>::Scale
Sim3Base<_Derived>::scale() const {
  return coeffs()(7);
}

template <typename _Derived>
void Sim3Base<_Derived>::normalize() {
  coeffs().template segment<4>(3).normalize();
}

template <typename _Derived>
Eigen::Map<const RxSO3<typename Sim3Base<_Derived>::Scalar>>
Sim3Base<_Derived>::asRxSO3() const {
  return Eigen::Map<const RxSO3<Scalar>>(coeffs().data()+3);
}

template <typename _Derived>
Eigen::Map<RxSO3<typename Sim3Base<_Derived>::Scalar>>
Sim3Base<_Derived>::asRxSO3() {
  return Eigen::Map<RxSO3<Scalar>>(coeffs().data()+3);
}

namespace internal {

//! @brief Random specialization for Sim3Base objects.
template <typename Derived>
struct RandomEvaluatorImpl<Sim3Base<Derived>>
{
  template <typename T>
  static void run(T& m)   {
    using Scalar      = typename Sim3Base<Derived>::Scalar;
    using Translation = typename Sim3Base<Derived>::Translation;
    using LieGroup    = typename Sim3Base<Derived>::LieGroup;

    m = LieGroup(
      Translation::Random(),
      randQuat<Scalar>(),
      Eigen::internal::random<Scalar>(0, 10)
    );
  }
};

//! @brief Assignment assert specialization for SE2Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<Sim3Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)   {
    using Scalar = typename Sim3Base<Derived>::Scalar;
    using std::abs;
    MANIF_ASSERT(
      abs(data.template segment<4>(3).norm()-Scalar(1)) <
      Constants<Scalar>::eps,
      "Sim3 assigned data (quat) not normalized !",
      manif::invalid_argument
    );
    MANIF_ASSERT(
      data(7) > Scalar(0),
      "Sim3 assigned data (scale) can not be negative nor zero!",
      manif::invalid_argument
    );
    MANIF_UNUSED_VARIABLE(data);
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SIM3_BASE_H_
