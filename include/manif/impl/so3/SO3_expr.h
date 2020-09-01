#ifndef _MANIF_MANIF_SO3_EXPR_H_
#define _MANIF_MANIF_SO3_EXPR_H_

namespace manif {
namespace internal {

//! @brief Random specialization for SO3Base objects.
template <typename Derived>
struct RandomEvaluatorImpl<SO3Base<Derived>>
{
  template <typename T>
  static void run(T& m)
  {
    // @note:
    // Quaternion::UnitRandom is not available in Eigen 3.3-beta1
    // which is the default version in Ubuntu 16.04
    // So we copy its implementation here.

    using std::sqrt;
    using std::sin;
    using std::cos;

    using Scalar     = typename SO3Base<Derived>::Scalar;
    using Quaternion = typename SO3Base<Derived>::QuaternionDataType;
    using LieGroup   = typename SO3Base<Derived>::LieGroup;

    const Scalar u1 = Eigen::internal::random<Scalar>(0, 1),
                 u2 = Eigen::internal::random<Scalar>(0, 2*EIGEN_PI),
                 u3 = Eigen::internal::random<Scalar>(0, 2*EIGEN_PI);
    const Scalar a = sqrt(1. - u1),
                 b = sqrt(u1);
    m = LieGroup(Quaternion(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3)));

    // m = Derived(Quaternion::UnitRandom());
  }
};

//! @brief Assignment assert specialization for SO3Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<SO3Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)
  {
    using std::abs;
    using Scalar = typename SO3Base<Derived>::Scalar;
    MANIF_CHECK(abs(data.norm()-Scalar(1)) < Constants<Scalar>::eps_s,
                "SO3 assigned data not normalized !",
                invalid_argument);
  }
};

/**
 * @brief Get the inverse of this.
 * @note q^-1 = q*. See Eq. (140).
 */
template <typename _Derived>
struct ExprEvaluator<InverseExpr<SO3Base<_Derived>>>
{
  using Ret = ReturnType<InverseExpr<SO3Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J)
  {
    if (J)
    {
      *J = -m.rotation();
    }

    /// @todo, conjugate doc :
    /// equal to the multiplicative inverse if
    /// the quaternion is normalized
    return Ret(m.quat().conjugate());
  }
};

/**
 * @brief Composition of this and another SO3 element.
 * @note Quaternion product.
 * @note See Eqs. (141,142).
 */
template <typename _Derived, typename _DerivedOther>
struct ExprEvaluator<ComposeExpr<SO3Base<_Derived>, SO3Base<_DerivedOther>>>
{
  using Ret = ReturnType<ComposeExpr<SO3Base<_Derived>, SO3Base<_DerivedOther>>>;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 typename Lhs::OptJacobianRef& J_mc_ma,
                 typename Rhs::OptJacobianRef& J_mc_mb)
  {
    using Scalar = typename Lhs::Scalar;
    using QuaternionDataType = typename Lhs::QuaternionDataType;
    using std::abs;

    static_assert(
      std::is_base_of<SO3Base<_DerivedOther>, Rhs>::value,
      "Argument does not inherit from S03Base !");

    // const auto& m_SO3 = static_cast<const SO3Base<_DerivedOther>&>(m);

    if (J_mc_ma)
    {
      *J_mc_ma = rhs.rotation().transpose();
    }

    if (J_mc_mb)
    {
      J_mc_mb->setIdentity();
    }

    QuaternionDataType ret_q = lhs.quat() * rhs.quat();

    const Scalar ret_sqnorm = ret_q.squaredNorm();

    if (abs(ret_sqnorm-Scalar(1)) > Constants<Scalar>::eps_s)
    {
      const Scalar scale = approxSqrtInv(ret_sqnorm);
      ret_q.coeffs() *= scale;
    }

    return Ret(ret_q);
  }
};

/**
 * @brief Rotation action on a 3-vector.
 * @note See Eq (136), Eqs. (150,151)
 */
template <typename _Derived, typename _DerivedOther>
struct ExprEvaluator<ActExpr<SO3Base<_Derived>, _DerivedOther>>
{
  using Scalar = typename _Derived::Scalar;
  using OptJacobianLhsRef = tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>>;
  using OptJacobianRhsRef = tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>>;

  using Ret = ReturnType<ActExpr<SO3Base<_Derived>, _DerivedOther>>;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 OptJacobianLhsRef& J_ret_lhs,
                 OptJacobianRhsRef& J_ret_rhs)
  {
    using Rotation = typename _Derived::Rotation;

    assert_vector_dim(rhs, 3);
    const Rotation R(lhs.rotation());

    if (J_ret_lhs)
    {
      (*J_ret_lhs) = -R * skew(rhs);
    }

    if (J_ret_rhs)
    {
      (*J_ret_rhs) = R;
    }

    return R * rhs;
  }
};

/**
 * @brief Get the adjoint of SO3 at this.
 * @note See Eq. (139).
 */
template <typename _Derived>
struct ExprEvaluator<AdjExpr<SO3Base<_Derived>>>
{
  using Ret = ReturnType<AdjExpr<SO3Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    return m.rotation();
  }
};

/**
 * @brief Get the SO3 corresponding Lie algebra element in vector form.
 * @note This is the log() map in vector form.
 * @note See Eq. (133) & Eq. (144).
 * @see SO3Tangent.
 */
template <typename _Derived>
struct ExprEvaluator<LogExpr<SO3Base<_Derived>>>
{
  using Ret = ReturnType<LogExpr<SO3Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J)
  {
    using Scalar = typename T::Scalar;
    using Tangent = typename T::Tangent;
    using LieAlg = typename Tangent::LieAlg;
    using Jacobian = typename T::Jacobian;

    using std::sqrt;
    using std::atan2;

    Tangent tan;
    Scalar log_coeff;

    const Scalar sin_angle_squared = m.coeffs().template head<3>().squaredNorm();
    if (sin_angle_squared > Constants<Scalar>::eps)
    {
      const Scalar sin_angle = sqrt(sin_angle_squared);
      const Scalar cos_angle = m.w();

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

      log_coeff = two_angle / sin_angle;
    }
    else
    {
      // small-angle approximation
      log_coeff = Scalar(2.0);
    }

    tan = Tangent(m.coeffs().template head<3>() * log_coeff);

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

    if (J)
    {
      Scalar theta2 = tan.coeffs().squaredNorm();
      LieAlg W = tan.hat();
      if (theta2 <= Constants<Scalar>::eps)
        J->noalias() = Jacobian::Identity() + Scalar(0.5) * W; // Small angle approximation
      else
      {
        Scalar theta = sqrt(theta2);  // rotation angle
        Jacobian M;
        M.noalias() = (Scalar(1) / theta2 - (Scalar(1) + cos(theta)) / (Scalar(2) * theta * sin(theta))) * (W * W);
        J->noalias() = Jacobian::Identity() + Scalar(0.5) * W + M; // is this really more optimized?
      }
    }

    return tan;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SO3_EXPR_H_
