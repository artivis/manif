#ifndef _MANIF_MANIF_SO2_BASE_H_
#define _MANIF_MANIF_SO2_BASE_H_

#include "manif/impl/so2/SO2_properties.h"
#include "manif/impl/lie_group_base.h"

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the SO2 group.
 * @note See Appendix A of the paper.
 */
template <typename _Derived>
struct SO2Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = SO2Base<_Derived>;

public:

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Transformation = typename internal::traits<_Derived>::Transformation;

  // LieGroup common API

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SO2Base)

public:

  MANIF_GROUP_ML_ASSIGN_OP(SO2Base)

  /**
   * @brief Get the inverse of this.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   * @note z^-1 = z*
   * @note See Eqs. (118,124).
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the SO2 corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The SO2 tangent of this.
   * @note This is the log() map in vector form.
   * @note See Eq. (115) & Eqs. (79,126).
   * @see SO2Tangent.
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
   * @brief Composition of this and another SO2 element.
   * @param[in] m Another SO2 element.
   * @param[out] -optional- J_mc_ma Jacobian of the composition wrt this.
   * @param[out] -optional- J_mc_mb Jacobian of the composition wrt m.
   * @return The composition of 'this . m'.
   * @note z_c = z_a z_b.
   * @note See Eq. (125).
   */
  template <typename _DerivedOther>
  LieGroup compose(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  /**
   * @brief Rotation action on a 2-vector.
   * @param  v A 2-vector.
   * @param[out] -optional- J_vout_m The Jacobian of the new object wrt this.
   * @param[out] -optional- J_vout_v The Jacobian of the new object wrt input object.
   * @return The rotated 2-vector.
   * @note See Eqs. (129, 130).
   */
  template <typename _EigenDerived>
  Eigen::Matrix<Scalar, 2, 1>
  act(const Eigen::MatrixBase<_EigenDerived> &v,
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 1>>> J_vout_m = {},
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 2>>> J_vout_v = {}) const;

  /**
   * @brief Get the ajoint matrix of SO2 at this.
   * @note See Eqs. (123).
   */
  Jacobian adj() const;

  // SO2 specific functions

  /**
   * @brief Get the transformation matrix (2D isometry).
   * @note T = | R 0 |
   *           | 0 1 |
   */
  Transformation transform() const;

  /**
   * @brief Get the rotation matrix R.
   */
  Rotation rotation() const;

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
   * @brief Normalize the underlying complex number.
   */
  void normalize();

// protected:

  /// @todo given a Eigen::Map<const SO2>
  /// coeffs()->x() return a reference to
  /// temporary ...

//  Scalar& real();
//  Scalar& imag();
};

template <typename _Derived>
typename SO2Base<_Derived>::Transformation
SO2Base<_Derived>::transform() const
{
  Transformation T(Transformation::Identity());
  T.template block<2,2>(0,0) = rotation();
  return T;
}

template <typename _Derived>
typename SO2Base<_Derived>::Rotation
SO2Base<_Derived>::rotation() const
{
  using std::sin;
  using std::cos;
  const Scalar theta = angle();
  return (Rotation() << cos(theta), -sin(theta),
                        sin(theta),  cos(theta)).finished();
}

template <typename _Derived>
typename SO2Base<_Derived>::LieGroup
SO2Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  if (J_minv_m)
    J_minv_m->setConstant(Scalar(-1));

  return LieGroup(real(), -imag());
}

template <typename _Derived>
typename SO2Base<_Derived>::Tangent
SO2Base<_Derived>::log(OptJacobianRef J_t_m) const
{
  if (J_t_m)
    J_t_m->setConstant(Scalar(1));

  return Tangent(angle());
}

template <typename _Derived>
typename SO2Base<_Derived>::Tangent
SO2Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename SO2Base<_Derived>::LieGroup
SO2Base<_Derived>::compose(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  using std::abs;

  static_assert(
    std::is_base_of<SO2Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from SE2Base !");

  if (J_mc_ma)
    J_mc_ma->setConstant(Scalar(1));

  if (J_mc_mb)
    J_mc_mb->setConstant(Scalar(1));

  const auto& m_so2 = static_cast<const SO2Base<_DerivedOther>&>(m);

  Scalar ret_real = real() * m_so2.real() - imag() * m_so2.imag();
  Scalar ret_imag = real() * m_so2.imag() + imag() * m_so2.real();

  const Scalar ret_sqnorm = ret_real*ret_real+ret_imag*ret_imag;

  if (abs(ret_sqnorm-Scalar(1)) > Constants<Scalar>::eps_s)
  {
    const Scalar scale = approxSqrtInv(ret_sqnorm);
    ret_real *= scale;
    ret_imag *= scale;
  }

  return LieGroup(ret_real, ret_imag);
}

template <typename _Derived>
template <typename _EigenDerived>
Eigen::Matrix<typename SO2Base<_Derived>::Scalar, 2, 1>
SO2Base<_Derived>::act(const Eigen::MatrixBase<_EigenDerived> &v,
                       tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 1>>> J_vout_m,
                       tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 2>>> J_vout_v) const
{
  assert_vector_dim(v, 2);
  const Rotation R(rotation());

  if (J_vout_m)
  {
    (*J_vout_m) = R * skew(Scalar(1)) * v;
  }

  if (J_vout_v)
  {
    (*J_vout_v) = R;
  }

  return R * v;
}

template <typename _Derived>
typename SO2Base<_Derived>::Jacobian
SO2Base<_Derived>::adj() const
{
  static const Jacobian adj = Jacobian::Constant(Scalar(1));
  return adj;
}

// SO2 specific function

template <typename _Derived>
/*const*/ typename SO2Base<_Derived>::Scalar/*&*/
SO2Base<_Derived>::real() const
{
  return coeffs().x();
}

template <typename _Derived>
/*const*/ typename SO2Base<_Derived>::Scalar/*&*/
SO2Base<_Derived>::imag() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SO2Base<_Derived>::Scalar
SO2Base<_Derived>::angle() const
{
  using std::atan2;
  return atan2(imag(), real());
}

//template <typename _Derived>
//typename SO2Base<_Derived>::Scalar&
//SO2Base<_Derived>::real()
//{
//  return coeffs.x();
//}

//template <typename _Derived>
//typename SO2Base<_Derived>::Scalar&
//SO2Base<_Derived>::imag()
//{
//  return coeffs.y();
//}

template <typename _Derived>
void SO2Base<_Derived>::normalize()
{
  coeffs().normalize();
}

namespace internal {

//! @brief Random specialization for SO2Base objects.
template <typename Derived>
struct RandomEvaluatorImpl<SO2Base<Derived>>
{
  template <typename T>
  static void run(T& m)
  {
    using Tangent = typename LieGroupBase<Derived>::Tangent;
    m = Tangent::Random().exp();
  }
};

//! @brief Assignment assert specialization for SO2Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<SO2Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)
  {
    using std::abs;
    MANIF_ASSERT(
      abs(data.norm()-typename SO2Base<Derived>::Scalar(1)) <
      Constants<typename SO2Base<Derived>::Scalar>::eps_s,
      "SO2 assigned data not normalized !",
      invalid_argument
    );
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_BASE_H_ */
