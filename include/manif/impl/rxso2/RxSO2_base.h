#ifndef _MANIF_MANIF_RxSO2_BASE_H_
#define _MANIF_MANIF_RxSO2_BASE_H_

#include "manif/impl/rxso2/RxSO2_properties.h"
#include "manif/impl/lie_group_base.h"

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the RxSO2 group.
 */
template <typename _Derived>
struct RxSO2Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = RxSO2Base<_Derived>;

public:

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Scale          = Scalar;
  using Transformation = typename internal::traits<_Derived>::Transformation;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(RxSO2Base)

public:

  MANIF_GROUP_ML_ASSIGN_OP(RxSO2Base)

  // LieGroup common API

  /**
   * @brief Get the inverse of this.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the RxSO2 corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The RxSO2 tangent of this.
   * @see RxSO2Tangent.
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
   * @brief Composition of this and another RxSO2 element.
   * @param[in] m Another RxSO2 element.
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
  Eigen::Matrix<Scalar, 2, 1>
  act(const Eigen::MatrixBase<_EigenDerived> &v,
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m = {},
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v = {}) const;

  /**
   * @brief Get the adjoint of RxSO2 at this.
   */
  Jacobian adj() const;

  // RxSO2 specific functions

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

  /**
   * @brief Get the real part of the underlying complex number.
   */
  Scalar real() const;

  /**
   * @brief Get the imaginary part of the underlying complex number.
   */
  Scalar imag() const;

  /**
   * @brief Get the angle (rad.).
   */
  Scalar angle() const;

  /**
   * @brief Normalize the underlying quaternion.
   */
  void normalize();
};

template <typename _Derived>
typename RxSO2Base<_Derived>::Transformation
RxSO2Base<_Derived>::transform() const
{
  Transformation T = Transformation::Identity();
  T.template topLeftCorner<2, 2>() = scale() * rotation();
  return T;
}

template <typename _Derived>
typename RxSO2Base<_Derived>::Rotation
RxSO2Base<_Derived>::rotation() const
{
  return (Rotation() << real(), -imag(),
                        imag(),  real()).finished();
}

template <typename _Derived>
typename RxSO2Base<_Derived>::LieGroup
RxSO2Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  if (J_minv_m) {
    *J_minv_m = -adj();
  }

  Scale inv_s = scale() > Constants<Scalar>::eps?
    Scalar(1)/scale() : [&](){
      Scalar x_1 = scale()-Scalar(1);
      Scalar x_12 = x_1*x_1;
      return Scalar(1) - x_1 + x_12 - x_12*x_1 + x_12*x_12;
    }();

  return LieGroup(real(), -imag(), inv_s);
}

template <typename _Derived>
typename RxSO2Base<_Derived>::Tangent
RxSO2Base<_Derived>::log(OptJacobianRef J_t_m) const
{
  using std::log;

  if (J_t_m) {
    J_t_m->setIdentity();
  }

  return Tangent(angle(), log(scale()));
}

template <typename _Derived>
typename RxSO2Base<_Derived>::Tangent
RxSO2Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename RxSO2Base<_Derived>::LieGroup
RxSO2Base<_Derived>::compose(
  const LieGroupBase<_DerivedOther>& m,
  OptJacobianRef J_mc_ma,
  OptJacobianRef J_mc_mb
) const {
  using std::abs;

  static_assert(
    std::is_base_of<RxSO2Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from RxSO2Base !");

  const auto& m_RxSO2 = static_cast<const RxSO2Base<_DerivedOther>&>(m);

  if (J_mc_ma) {
    J_mc_ma->setIdentity();
  }

  if (J_mc_mb) {
    J_mc_mb->setIdentity();
  }

  Scalar ret_real = real() * m_RxSO2.real() - imag() * m_RxSO2.imag();
  Scalar ret_imag = real() * m_RxSO2.imag() + imag() * m_RxSO2.real();

  const Scalar ret_sqnorm = ret_real*ret_real+ret_imag*ret_imag;

  if (abs(ret_sqnorm-Scalar(1)) > Constants<Scalar>::eps)
  {
    const Scalar scale = approxSqrtInv(ret_sqnorm);
    ret_real *= scale;
    ret_imag *= scale;
  }

  return LieGroup(ret_real, ret_imag, scale() * m_RxSO2.scale());
}

template <typename _Derived>
template <typename _EigenDerived>
Eigen::Matrix<typename RxSO2Base<_Derived>::Scalar, 2, 1>
RxSO2Base<_Derived>::act(
  const Eigen::MatrixBase<_EigenDerived> &v,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v
) const {
  assert_vector_dim(v, 2);
  const Rotation sR = scale() * rotation();

  const Vector vout = sR * v;

  if (J_vout_m) {
    J_vout_m->template leftCols<1>().noalias() = sR * skew(Scalar(1)) * v;
    J_vout_m->template rightCols<1>() = vout;
  }

  if (J_vout_v) {
    *J_vout_v = sR;
  }

  return vout;
}

template <typename _Derived>
typename RxSO2Base<_Derived>::Jacobian
RxSO2Base<_Derived>::adj() const
{
  return Jacobian::Identity();
}

// RxSO2 specific

template <typename _Derived>
typename RxSO2Base<_Derived>::Scalar
RxSO2Base<_Derived>::real() const
{
  return coeffs().x();
}

template <typename _Derived>
typename RxSO2Base<_Derived>::Scalar
RxSO2Base<_Derived>::imag() const
{
  return coeffs().y();
}

template <typename _Derived>
typename RxSO2Base<_Derived>::Scalar
RxSO2Base<_Derived>::angle() const
{
  using std::atan2;
  return atan2(imag(), real());
}

template <typename _Derived>
void RxSO2Base<_Derived>::normalize()
{
  coeffs().template head<2>().normalize();
}

template <typename _Derived>
typename RxSO2Base<_Derived>::Scale
RxSO2Base<_Derived>::scale() const
{
  return coeffs().z();
}

namespace internal {

//! @brief Random specialization for RxSO2Base objects.
template <typename Derived>
struct RandomEvaluatorImpl<RxSO2Base<Derived>>
{
  template <typename T>
  static void run(T& m)
  {
    using Scalar = typename RxSO2Base<Derived>::Scalar;
    using std::cos;
    using std::sin;

    const Scalar t = Eigen::internal::random<Scalar>(-MANIF_PI, MANIF_PI);
    m.coeffs()(0) = cos(t);
    m.coeffs()(1) = sin(t);
    m.coeffs()(2) = Eigen::internal::random<Scalar>(0, 2);
  }
};

//! @brief Assignment assert specialization for RxSO2Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<RxSO2Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)
  {
    using Scalar = typename RxSO2Base<Derived>::Scalar;
    using std::abs;
    MANIF_ASSERT(
      abs(data.template head<2>().norm()-Scalar(1)) < Constants<Scalar>::eps,
      "RxSO2 assigned data (rotational part) not normalized!",
      manif::invalid_argument
    );
    MANIF_ASSERT(
      data(2) > Scalar(0),
      "RxSO2 assigned data (scale) can not be negative nor zero!",
      manif::invalid_argument
    );
    MANIF_UNUSED_VARIABLE(data);
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_RxSO2_BASE_H_
