#ifndef _MANIF_MANIF_SGAL3_BASE_H_
#define _MANIF_MANIF_SGAL3_BASE_H_

#include "manif/impl/sgal3/SGal3_properties.h"
#include "manif/impl/lie_group_base.h"
#include "manif/impl/so3/SO3_map.h"
#include "manif/impl/se3/SE3_map.h"

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the SGal3 group.
 * @note See "All About the Galilean Group SGal(3)" J. Kelly.
 * https://arxiv.org/abs/2312.07555
 *
 */
template <typename _Derived>
struct SGal3Base : LieGroupBase<_Derived> {
private:

  using Base = LieGroupBase<_Derived>;
  using Type = SGal3Base<_Derived>;

public:

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Translation    = typename internal::traits<_Derived>::Translation;
  using LinearVelocity = typename internal::traits<_Derived>::LinearVelocity;
  using Time           = Scalar;
  using Transformation = Eigen::Matrix<Scalar, 5, 5>;
  using Isometry       = Eigen::Matrix<Scalar, 5, 5>; /**< Double direct spatial isometry*/
  using QuaternionDataType = Eigen::Quaternion<Scalar>;

  // LieGroup common API

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SGal3Base)

public:

  MANIF_GROUP_ML_ASSIGN_OP(SGal3Base)

  /**
   * @brief Get the inverse.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the SGal3 corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The SGal3 tangent of this.
   * @note This is the log() map in vector form.
   * @see SGal3Tangent.
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
   * @brief Composition of this and another SGal3 element.
   * @param[in] m Another SGal3 element.
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
   * @brief Get the action
   * @param[in] v A 3D point.
   * @param[out] -optional- J_vout_m The Jacobian of the new object wrt this.
   * @param[out] -optional- J_vout_v The Jacobian of the new object wrt input object.
   */
  template <typename _EigenDerived>
  Eigen::Matrix<Scalar, 3, 1>
  act(
    const Eigen::MatrixBase<_EigenDerived> &v,
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 10>>> J_vout_m = {},
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_v = {}
  ) const;

  /**
   * @brief Get the adjoint matrix of SGal3 at this.
   */
  Jacobian adj() const;

  // SGal3 specific functions

  /**
   * Get the isometry object (double direct isometry).
   * @note T = | R v t|
   *           |   1 s|
   *           |     1|
   */
  Transformation transform() const;

  /**
   * Get the isometry object (double direct isometry).
   * @note T = | R v t|
   *           |   1 s|
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
   * @brief Get the time.
   */
  Scalar t() const;

  /**
   * @brief Normalize the underlying quaternion.
   */
  void normalize();

public: /// @todo make protected

  Eigen::Map<const SO3<Scalar>> asSO3() const {
    return Eigen::Map<const SO3<Scalar>>(coeffs().data()+3);
  }

  Eigen::Map<SO3<Scalar>> asSO3() {
    return Eigen::Map<SO3<Scalar>>(coeffs().data()+3);
  }
};

template <typename _Derived>
typename SGal3Base<_Derived>::Transformation
SGal3Base<_Derived>::transform() const {
  Eigen::Matrix<Scalar, 5, 5> T;
  T.template topLeftCorner<3, 3>() = rotation();
  T.template block<3, 1>(0, 3) = linearVelocity();
  T.template topRightCorner<3, 1>() = translation();
  T.template bottomLeftCorner<2, 3>().setZero();
  T.template bottomRightCorner<2, 2>().setIdentity();
  T(3, 4) = t();
  return T;
}

template <typename _Derived>
typename SGal3Base<_Derived>::Isometry
SGal3Base<_Derived>::isometry() const {
  return Isometry(transform());
}

template <typename _Derived>
typename SGal3Base<_Derived>::Rotation
SGal3Base<_Derived>::rotation() const {
  return asSO3().rotation();
}

template <typename _Derived>
typename SGal3Base<_Derived>::QuaternionDataType
SGal3Base<_Derived>::quat() const {
  return asSO3().quat();
}

template <typename _Derived>
typename SGal3Base<_Derived>::Translation
SGal3Base<_Derived>::translation() const {
  return coeffs().template head<3>();
}

template <typename _Derived>
typename SGal3Base<_Derived>::LinearVelocity
SGal3Base<_Derived>::linearVelocity() const {
  return coeffs().template segment<3>(7);
}

template <typename _Derived>
typename SGal3Base<_Derived>::LieGroup
SGal3Base<_Derived>::inverse(OptJacobianRef J_minv_m) const {
  if (J_minv_m) {
    (*J_minv_m) = -adj();
  }

  const SO3<Scalar> so3inv = asSO3().inverse();

  return LieGroup(
    -so3inv.act((translation()-t()*linearVelocity())),
     so3inv,
    -so3inv.act(linearVelocity()),
    -t()
  );
}

template <typename _Derived>
typename SGal3Base<_Derived>::Tangent
SGal3Base<_Derived>::log(OptJacobianRef J_t_m) const {
  using I = typename Eigen::DiagonalMatrix<Scalar, 3>;

  using std::sqrt;
  using std::cos;
  using std::sin;

  const SO3Tangent<Scalar> so3tan = asSO3().log();

  const Scalar theta_sq = so3tan.coeffs().squaredNorm();
  const Scalar theta = sqrt(theta_sq); // rotation angle

  // small angle approx.
  // @todo move 'fillE' to separate utils file and use it here.
  Eigen::Matrix<Scalar, 3, 3> E = I(Scalar(0.5), Scalar(0.5), Scalar(0.5)).toDenseMatrix();
  if (theta_sq > Constants<Scalar>::eps) {
    const Scalar A = (theta - sin(theta)) / theta_sq / theta;
    const Scalar B = (theta_sq + Scalar(2) * cos(theta) - Scalar(2)) / (Scalar(2) * theta_sq * theta_sq);

    const typename SO3Tangent<Scalar>::LieAlg W = so3tan.hat();
    E.noalias() += (A * W + B * W * W);
  }

  const LinearVelocity nu = so3tan.ljacinv() * linearVelocity();

  Tangent tan(
    (
      typename Tangent::DataType() <<
        so3tan.ljacinv() * (translation() - E * (t() * nu)), nu, so3tan.coeffs(), t()
    ).finished()
  );

  if (J_t_m) {
    // Jr^-1
    (*J_t_m) = tan.rjacinv();
  }

  return tan;
}

template <typename _Derived>
typename SGal3Base<_Derived>::Tangent
SGal3Base<_Derived>::lift(OptJacobianRef J_t_m) const {
  return log(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename SGal3Base<_Derived>::LieGroup
SGal3Base<_Derived>::compose(
  const LieGroupBase<_DerivedOther>& m,
  OptJacobianRef J_mc_ma,
  OptJacobianRef J_mc_mb
) const {
  static_assert(
    std::is_base_of<SGal3Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from SGal3Base !"
  );

  const auto& m_sgal3 = static_cast<const SGal3Base<_DerivedOther>&>(m);

  if (J_mc_ma) {
    (*J_mc_ma) = m.inverse().adj();
  }

  if (J_mc_mb) {
    J_mc_mb->setIdentity();
  }

  return LieGroup(
    rotation() * m_sgal3.translation() + m_sgal3.t() * linearVelocity() + translation(),
    asSO3() * m_sgal3.asSO3(),
    rotation() * m_sgal3.linearVelocity() + linearVelocity(),
    t() + m_sgal3.t()
  );
}

template <typename _Derived>
template <typename _EigenDerived>
Eigen::Matrix<typename SGal3Base<_Derived>::Scalar, 3, 1>
SGal3Base<_Derived>::act(
  const Eigen::MatrixBase<_EigenDerived> &v,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 10>>> J_vout_m,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_v
) const {
  assert_vector_dim(v, 3);

  const Rotation R(rotation());

  if (J_vout_m) {
    J_vout_m->template topLeftCorner<3, 3>() = R;
    J_vout_m->template block<3, 3>(0, 3).setZero();
    J_vout_m->template block<3, 3>(0, 6).noalias() = -R * skew(v);
    J_vout_m->template topRightCorner<3, 1>() = linearVelocity();
  }

  if (J_vout_v) {
    (*J_vout_v) = R;
  }

  return translation() + R * v;
}


template <typename _Derived>
typename SGal3Base<_Derived>::Jacobian
SGal3Base<_Derived>::adj() const {
  ///
  /// this is
  ///       Ad(g) = | R -R.tau [(t-v.tau)]x.R v|
  ///               | 0   R        [v]x.R     0|
  ///               | 0   0          R        0|
  ///               | 0   0          0        1|
  ///
  /// considering vee(log(g)) = (rho;v;w;iota)

  Jacobian Adj;
  Adj.template topLeftCorner<3, 3>() = rotation();
  Adj.template block<3, 3>(0, 3).noalias() = -t() * Adj.template topLeftCorner<3, 3>();
  Adj.template block<3, 3>(0, 6).noalias() =
    skew(translation() - t() * linearVelocity()) * Adj.template topLeftCorner<3, 3>();
  Adj.template topRightCorner<3, 1>() = linearVelocity();

  Adj.template block<3, 3>(3, 3).noalias() = Adj.template topLeftCorner<3, 3>();
  Adj.template block<3, 3>(3, 6).noalias() =
    skew(linearVelocity()) * Adj.template topLeftCorner<3, 3>();

  Adj.template block<3,  3>(6, 6).noalias() = Adj.template topLeftCorner<3, 3>();

  Adj.template bottomLeftCorner<7, 3>().setZero();
  Adj.template block<4, 3>(6, 3).setZero();
  Adj.template block<1, 3>(9, 6).setZero();
  Adj.template block<6, 1>(3, 9).setZero();

  Adj(9, 9) = Scalar(1);

  return Adj;
}

// SGal3 specific function

template <typename _Derived>
typename SGal3Base<_Derived>::Scalar
SGal3Base<_Derived>::x() const {
  return coeffs()(0);
}

template <typename _Derived>
typename SGal3Base<_Derived>::Scalar
SGal3Base<_Derived>::y() const {
  return coeffs()(1);
}

template <typename _Derived>
typename SGal3Base<_Derived>::Scalar
SGal3Base<_Derived>::z() const {
  return coeffs()(2);
}

template <typename _Derived>
typename SGal3Base<_Derived>::Scalar
SGal3Base<_Derived>::vx() const {
  return coeffs()(7);
}

template <typename _Derived>
typename SGal3Base<_Derived>::Scalar
SGal3Base<_Derived>::vy() const {
  return coeffs()(8);
}

template <typename _Derived>
typename SGal3Base<_Derived>::Scalar
SGal3Base<_Derived>::vz() const {
  return coeffs()(9);
}

template <typename _Derived>
typename SGal3Base<_Derived>::Scalar
SGal3Base<_Derived>::t() const {
  return coeffs()(10);
}

template <typename _Derived>
void SGal3Base<_Derived>::normalize() {
  coeffs().template segment<4>(3).normalize();
}

namespace internal {

//! @brief Random specialization for SGal3Base objects.
template <typename Derived>
struct RandomEvaluatorImpl<SGal3Base<Derived>> {
  template <typename T>
  static void run(T& m) {
    using Scalar = typename SGal3Base<Derived>::Scalar;
    using LieGroup = typename SGal3Base<Derived>::LieGroup;

    typename LieGroup::DataType data = LieGroup::DataType::Random();
    data.template segment<4>(3) = randQuat<Scalar>().coeffs();

    m = LieGroup(data);
  }
};

//! @brief Assignment assert specialization for SGal3Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<SGal3Base<Derived>> {
  template <typename T>
  static void run_impl(const T& data) {
    using std::abs;
    MANIF_ASSERT(
      abs(data.template segment<4>(3).norm()-typename SGal3Base<Derived>::Scalar(1)) <
      Constants<typename SGal3Base<Derived>::Scalar>::eps,
      "SGal3 assigned data not normalized !",
      manif::invalid_argument
    );
    MANIF_UNUSED_VARIABLE(data);
  }
};

//! @brief Cast specialization for SGal3Base objects.
template <typename Derived, typename NewScalar>
struct CastEvaluatorImpl<SGal3Base<Derived>, NewScalar> {
  template <typename T>
  static auto run(const T& o) -> typename Derived::template LieGroupTemplate<NewScalar> {
    return typename Derived::template LieGroupTemplate<NewScalar>(
      o.translation().template cast<NewScalar>(),
      o.quat().template cast<NewScalar>().normalized(),
      o.linearVelocity().template cast<NewScalar>(),
      NewScalar(o.t())
    );
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SGAL3_BASE_H_
