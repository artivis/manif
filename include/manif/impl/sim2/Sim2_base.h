#ifndef _MANIF_MANIF_SIM2_BASE_H_
#define _MANIF_MANIF_SIM2_BASE_H_

#include "manif/impl/sim2/Sim2_properties.h"
#include "manif/impl/lie_group_base.h"
#include "manif/impl/rxso2/RxSO2_map.h"

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the Sim2 group.
 */
template <typename _Derived>
struct Sim2Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = Sim2Base<_Derived>;

public:

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Translation    = typename internal::traits<_Derived>::Translation;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using Scale          = Scalar;

  // LieGroup common API

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(Sim2Base)

public:

  MANIF_GROUP_ML_ASSIGN_OP(Sim2Base)

  /**
   * @brief Get the inverse.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the Sim2 corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The Sim2 tangent of this.
   * @note This is the log() map in vector form.
   * @see Sim2Tangent.
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
   * @brief Composition of this and another Sim2 element.
   * @param[in] m Another Sim2 element.
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
  Eigen::Matrix<Scalar, 2, 1>
  act(
    const Eigen::MatrixBase<_EigenDerived> &v,
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m = {},
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v = {}
  ) const;

  /**
   * @brief Get the adjoint matrix of Sim2 at this.
   */
  Jacobian adj() const;

  // Sim2 specific functions

  /**
   * Get the transformation matrix.
   * @note T = | sR t |
   *           | 0  1 |
   */
  Transformation transform() const;

  /**
   * @brief Get the rotational part of this as a rotation matrix.
   */
  Rotation rotation() const;

  /**
   * @brief Get the real part of the underlying complex number representing
   * the rotational part.
   */
  Scalar real() const;

  /**
   * @brief Get the imaginary part of the underlying complex number representing
   * the rotational part.
   */
  Scalar imag() const;

  /**
   * @brief Get the rotational angle.
   */
  Scalar angle() const;

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
   * @brief Get the scale.
   */
  Scale scale() const;

  /**
   * @brief Normalize the underlying quaternion.
   */
  void normalize();

  /**
   * @brief Set the translation of the Sim2 object
   * @param translation, 2d-vector representing the translation.
   */
  void translation(const Translation& translation);

  /**
   * @brief Set the translation of the Sim2 object
   * @param scale, scalar representing the scale.
   */
  void scale(const Scale scale);

// protected:

  Eigen::Map<RxSO2<Scalar>> asRxSO2();
  Eigen::Map<const RxSO2<Scalar>> asRxSO2() const;
};

template <typename _Derived>
typename Sim2Base<_Derived>::Transformation
Sim2Base<_Derived>::transform() const {
  Transformation T = Transformation::Identity();
  T.template topLeftCorner<2,2>()  = scale() * rotation();
  T.template topRightCorner<2,1>() = translation();
  return T;
}

template <typename _Derived>
typename Sim2Base<_Derived>::Rotation
Sim2Base<_Derived>::rotation() const {
  return asRxSO2().rotation();
}

template <typename _Derived>
typename Sim2Base<_Derived>::Scalar
Sim2Base<_Derived>::angle() const {
  return asRxSO2().angle();
}

template <typename _Derived>
typename Sim2Base<_Derived>::Translation
Sim2Base<_Derived>::translation() const {
  return coeffs().template head<2>();
}

template <typename _Derived>
void Sim2Base<_Derived>::translation(const Translation& translation) {
  coeffs().template head<2>() = translation;
}

template <typename _Derived>
void Sim2Base<_Derived>::scale(const Scale scale) {
  MANIF_ASSERT(
    scale > Scalar(0),
    "Sim2::scale assigned data (scale) can not be negative nor zero!",
    manif::invalid_argument
  );
  coeffs()(4) = scale;
}

template <typename _Derived>
typename Sim2Base<_Derived>::LieGroup
Sim2Base<_Derived>::inverse(OptJacobianRef J_minv_m) const {
  if (J_minv_m) {
    (*J_minv_m) = -adj();
  }

  const RxSO2<Scalar> RxSO2inv = asRxSO2().inverse();

  return LieGroup(-RxSO2inv.act(translation()), RxSO2inv);
}

template <typename _Derived>
typename Sim2Base<_Derived>::Tangent
Sim2Base<_Derived>::log(OptJacobianRef J_t_m) const {
  const RxSO2Tangent<Scalar> RxSO2tan = asRxSO2().log();

  Tangent tan((
    typename Tangent::DataType() <<
    Tangent::computeW(RxSO2tan.angle(), RxSO2tan.sigma()).inverse()*translation(),
    RxSO2tan.coeffs()
  ).finished());

  // Tangent tan(
  //   Tangent::computeW(RxSO2tan.angle(), RxSO2tan.sigma()).inverse()*translation(),
  //   RxSO2tan
  // );

  if (J_t_m) {
    *J_t_m = tan.rjacinv();
  }

  return tan;
}

template <typename _Derived>
typename Sim2Base<_Derived>::Tangent
Sim2Base<_Derived>::lift(OptJacobianRef J_t_m) const {
  return log(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename Sim2Base<_Derived>::LieGroup
Sim2Base<_Derived>::compose(
  const LieGroupBase<_DerivedOther>& m,
  OptJacobianRef J_mc_ma,
  OptJacobianRef J_mc_mb
) const {
  static_assert(
    std::is_base_of<Sim2Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from Sim2Base !");

  const auto& m_sim2 = static_cast<const Sim2Base<_DerivedOther>&>(m);

  if (J_mc_ma) {
    (*J_mc_ma) = m.inverse().adj();
  }

  if (J_mc_mb) {
    J_mc_mb->setIdentity();
  }

  const Eigen::Map<const RxSO2<Scalar>> RxSO2 = asRxSO2();

  return LieGroup(
    translation() + RxSO2.act(m_sim2.translation()),
    RxSO2.compose(m_sim2.asRxSO2())
  );
}

template <typename _Derived>
template <typename _EigenDerived>
Eigen::Matrix<typename Sim2Base<_Derived>::Scalar, 2, 1>
Sim2Base<_Derived>::act(
  const Eigen::MatrixBase<_EigenDerived> &v,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v
) const {
  assert_vector_dim(v, 2);
  const Rotation sR = scale() * rotation();

  if (J_vout_m) {
    J_vout_m->template topLeftCorner<2,2>()  = sR;
    J_vout_m->template topRightCorner<2,1>() = sR * (skew(Scalar(1)) * v);
    // J_vout_m->template topRightCorner<2,1>() = -sR * (skew(Scalar(1)) * v);
  }

  if (J_vout_v) {
    (*J_vout_v) = sR;
  }

  return translation() + sR * v;
}

template <typename _Derived>
typename Sim2Base<_Derived>::Jacobian
Sim2Base<_Derived>::adj() const {
  /// this is
  ///       Ad(g) = | sR  [T]x -t |
  ///               | 0     1   0 |
  ///               | 0     0   1 |
  ///
  /// considering vee(log(g)) = (v;w;s)

  Jacobian Adj;

  Adj.template topLeftCorner<2, 2>() = scale() * rotation();
  Adj.template block<2, 1>(0, 2).noalias() = skew(Scalar(1)) * translation();
  Adj.template topRightCorner<2, 1>() = -translation();

  Adj.template bottomLeftCorner<2,2>().setZero();
  Adj.template bottomRightCorner<2,2>().setIdentity();

  return Adj;
}

// Sim2 specific function

template <typename _Derived>
typename Sim2Base<_Derived>::Scalar
Sim2Base<_Derived>::x() const {
  return coeffs().x();
}

template <typename _Derived>
typename Sim2Base<_Derived>::Scalar
Sim2Base<_Derived>::y() const {
  return coeffs().y();
}

template <typename _Derived>
typename Sim2Base<_Derived>::Scalar
Sim2Base<_Derived>::real() const {
  return coeffs().z();
}

template <typename _Derived>
typename Sim2Base<_Derived>::Scalar
Sim2Base<_Derived>::imag() const {
  return coeffs().w();
}

template <typename _Derived>
typename Sim2Base<_Derived>::Scale
Sim2Base<_Derived>::scale() const {
  return coeffs()(4);
}

template <typename _Derived>
void Sim2Base<_Derived>::normalize() {
  coeffs().template segment<2>(2).normalize();
}

template <typename _Derived>
Eigen::Map<const RxSO2<typename Sim2Base<_Derived>::Scalar>>
Sim2Base<_Derived>::asRxSO2() const {
  return Eigen::Map<const RxSO2<Scalar>>(coeffs().data()+2);
}

template <typename _Derived>
Eigen::Map<RxSO2<typename Sim2Base<_Derived>::Scalar>>
Sim2Base<_Derived>::asRxSO2() {
  return Eigen::Map<RxSO2<Scalar>>(coeffs().data()+2);
}

namespace internal {

//! @brief Random specialization for Sim2Base objects.
template <typename Derived>
struct RandomEvaluatorImpl<Sim2Base<Derived>>
{
  template <typename T>
  static void run(T& m) {
    using Tangent = typename LieGroupBase<Derived>::Tangent;
    m = Tangent::Random().exp();
  }
};

//! @brief Assignment assert specialization for SE2Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<Sim2Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)   {
    using Scalar = typename Sim2Base<Derived>::Scalar;
    using std::abs;
    MANIF_ASSERT(
      abs(data.template segment<2>(2).norm()-Scalar(1)) <
      Constants<Scalar>::eps,
      "Sim2 assigned data (quat) not normalized !",
      manif::invalid_argument
    );
    MANIF_ASSERT(
      data(4) > Scalar(0),
      "Sim2 assigned data (scale) can not be negative nor zero!",
      manif::invalid_argument
    );
    MANIF_UNUSED_VARIABLE(data);
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SIM2_BASE_H_
