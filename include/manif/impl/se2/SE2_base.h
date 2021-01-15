#ifndef _MANIF_MANIF_SE2_BASE_H_
#define _MANIF_MANIF_SE2_BASE_H_

#include "manif/impl/se2/SE2_properties.h"
#include "manif/impl/lie_group_base.h"
#include "manif/impl/utils.h"

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the SE2 group.
 * @note See Appendix C of the paper.
 */
template <typename _Derived>
struct SE2Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = SE2Base<_Derived>;

public:

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Translation    = typename internal::traits<_Derived>::Translation;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using Isometry       = Eigen::Transform<Scalar, 2, Eigen::Isometry>;

  // LieGroup common API

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SE2Base)

public:

  MANIF_GROUP_ML_ASSIGN_OP(SE2Base)

  /**
   * @brief Get the inverse of this.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   * @note See Eqs. (154, 160).
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the SE2 corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The SE2 tangent of this.
   * @note This is the log() map in vector form.
   * @note See Eqs. (157, 158).
   * @see SE2Tangent.
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
   * @brief Composition of this and another SE2 element.
   * @param[in] m Another SE2 element.
   * @param[out] -optional- J_mc_ma Jacobian of the composition wrt this.
   * @param[out] -optional- J_mc_mb Jacobian of the composition wrt m.
   * @return The composition of 'this . m'.
   * @note See Eq. (155) & Eqs. (161,162).
   */
  template <typename _DerivedOther>
  LieGroup compose(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  /**
   * @brief Rigid motion action on a 2D point.
   * @param  v A 2D point.
   * @param[out] -optional- J_vout_m The Jacobian of the new object wrt this.
   * @param[out] -optional- J_vout_v The Jacobian of the new object wrt input object.
   * @return The transformed 2D point.
   * @note See Eq. (165) & Eqs. (166,167).
   */
  template <typename _EigenDerived>
  Eigen::Matrix<Scalar, 2, 1>
  act(const Eigen::MatrixBase<_EigenDerived> &v,
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 3>>> J_vout_m = {},
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 2>>> J_vout_v = {}) const;

  /**
   * @brief Get the adjoint matrix of SE2 at this.
   * @note See Eq. (159).
   */
  Jacobian adj() const;

  // SE2 specific functions

  /**
   * @brief Get the transformation matrix (2D isometry).
   * @note T = | R t |
   *           | 0 1 |
   */
  Transformation transform() const;

  /**
   * Get the isometry object (Eigen 2D isometry).
   * @note T = | R t |
   *           | 0 1 |
   */
  Isometry isometry() const;

  //! @brief Get the rotational part of this as a rotation matrix.
  Rotation rotation() const;

  //! @brief Get the translational part of this as a vector.
  Translation translation() const;

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
   * @brief Get the angle (rad.) of the rotational part.
   */
  Scalar angle() const;

  /**
   * @brief Get the x component of the translational part.
   */
  Scalar x() const;

  /**
   * @brief Get the y component of the translational part.
   */
  Scalar y() const;

  /**
   * @brief Normalize the underlying complex number.
   */
  void normalize();
};

template <typename _Derived>
typename SE2Base<_Derived>::Transformation
SE2Base<_Derived>::transform() const
{
  Transformation T(Transformation::Identity());
  T.template block<2,2>(0,0) = rotation();
  T(0,2) = x();
  T(1,2) = y();
  return T;
}

template <typename _Derived>
typename SE2Base<_Derived>::Isometry
SE2Base<_Derived>::isometry() const
{
  return Isometry(transform());
}

template <typename _Derived>
typename SE2Base<_Derived>::Rotation
SE2Base<_Derived>::rotation() const
{
  return (Rotation() << real(), -imag(),
                        imag(),  real() ).finished();
}

template <typename _Derived>
typename SE2Base<_Derived>::Translation
SE2Base<_Derived>::translation() const
{
  return Translation(x(), y());
}

template <typename _Derived>
typename SE2Base<_Derived>::LieGroup
SE2Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  using std::cos;
  using std::sin;

  if (J_minv_m)
  {
    (*J_minv_m) = -adj();
  }

  return LieGroup(-x()*real() - y()*imag(),
                   x()*imag() - y()*real(),
                           -angle()        );
}

template <typename _Derived>
typename SE2Base<_Derived>::Tangent
SE2Base<_Derived>::log(OptJacobianRef J_t_m) const
{
  using std::abs;
  using std::cos;
  using std::sin;

  const Scalar theta     = angle();
  const Scalar cos_theta = coeffs()[2];
  const Scalar sin_theta = coeffs()[3];
  const Scalar theta_sq  = theta * theta;

  Scalar A,  // sin_theta_by_theta
         B;  // one_minus_cos_theta_by_theta

  if (abs(theta) < Constants<Scalar>::eps)
  {
    // Taylor approximation
    A = Scalar(1) - Scalar(1. / 6.) * theta_sq;
    B = Scalar(.5) * theta - Scalar(1. / 24.) * theta * theta_sq;
  }
  else
  {
    // Euler
    A = sin_theta / theta;
    B = (Scalar(1) - cos_theta) / theta;
  }

  const Scalar den = Scalar(1) / (A*A + B*B);

  A *= den;
  B *= den;

  Tangent tan( A * x() + B * y(),
              -B * x() + A * y(),
                     theta       );

  if (J_t_m)
  {
    // Jr^-1
    (*J_t_m) = tan.rjacinv();
  }

  return tan;
}

template <typename _Derived>
typename SE2Base<_Derived>::Tangent
SE2Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename SE2Base<_Derived>::LieGroup
SE2Base<_Derived>::compose(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  using std::abs;

  static_assert(
    std::is_base_of<SE2Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from SE2Base !");

  if (J_mc_ma)
  {
    (*J_mc_ma) = m.inverse().adj();
  }

  if (J_mc_mb)
  {
    J_mc_mb->setIdentity();
  }

  const auto& m_se2 = static_cast<const SE2Base<_DerivedOther>&>(m);

  const Scalar lhs_real = real(); // cos(t)
  const Scalar lhs_imag = imag(); // sin(t)
  const Scalar rhs_real = m_se2.real();
  const Scalar rhs_imag = m_se2.imag();

  Scalar ret_real = lhs_real * rhs_real - lhs_imag * rhs_imag;
  Scalar ret_imag = lhs_real * rhs_imag + lhs_imag * rhs_real;

  const Scalar ret_sqnorm = ret_real*ret_real+ret_imag*ret_imag;

  if (abs(ret_sqnorm-Scalar(1)) > Constants<Scalar>::eps_s)
  {
    const Scalar scale = approxSqrtInv(ret_sqnorm);
    ret_real *= scale;
    ret_imag *= scale;
  }

  return LieGroup(lhs_real * m_se2.x() - lhs_imag * m_se2.y() + x(),
                  lhs_imag * m_se2.x() + lhs_real * m_se2.y() + y(),
                  ret_real, ret_imag                                );
}

template <typename _Derived>
template <typename _EigenDerived>
Eigen::Matrix<typename SE2Base<_Derived>::Scalar, 2, 1>
SE2Base<_Derived>::act(const Eigen::MatrixBase<_EigenDerived> &v,
                       tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 3>>> J_vout_m,
                       tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 2>>> J_vout_v) const
{
  assert_vector_dim(v, 2);
  const Rotation R(rotation());

  if (J_vout_m)
  {
    J_vout_m->template topLeftCorner<2,2>()  = R;
    J_vout_m->template topRightCorner<2,1>() = R * (skew(Scalar(1)) * v);
  }

  if (J_vout_v)
  {
    (*J_vout_v) = R;
  }

  return translation() + R * v;
}

template <typename _Derived>
typename SE2Base<_Derived>::Jacobian
SE2Base<_Derived>::adj() const
{
  Jacobian Adj = Jacobian::Identity();
  Adj.template topLeftCorner<2,2>() = rotation();
  Adj(0,2) =  y();
  Adj(1,2) = -x();
  return Adj;
}

// SE2 specific function

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::real() const
{
  return coeffs()(2);
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::imag() const
{
  return coeffs()(3);
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::angle() const
{
  using std::atan2;
  return atan2(imag(), real());
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
void SE2Base<_Derived>::normalize()
{
  coeffs().template tail<2>().normalize();
}

namespace internal {

//! @brief Random specialization for SE2Base objects
template <typename Derived>
struct RandomEvaluatorImpl<SE2Base<Derived>>
{
  template <typename T>
  static void run(T& m)
  {
    using Tangent = typename LieGroupBase<Derived>::Tangent;
    m = Tangent::Random().exp();
  }
};

//! @brief Assignment assert specialization for SE2Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<SE2Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)
  {
    using std::abs;
    MANIF_ASSERT(
      abs(data.template tail<2>().norm()-typename SE2Base<Derived>::Scalar(1)) <
      Constants<typename SE2Base<Derived>::Scalar>::eps_s,
      "SE2 assigned data not normalized !",
      invalid_argument
    );
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SE2_BASE_H_ */
