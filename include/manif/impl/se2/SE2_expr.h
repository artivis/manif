#ifndef _MANIF_MANIF_SE2_EXPR_H_
#define _MANIF_MANIF_SE2_EXPR_H_

namespace manif {
namespace internal {

//! @brief Assignment assert specialization for SE2Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<SE2Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)
  {
    using std::abs;
    using Scalar = typename SE2Base<Derived>::Scalar;
    MANIF_ASSERT(abs(data.template tail<2>().norm()-Scalar(1)) <
                 Constants<Scalar>::eps_s,
                 "SE2 assigned data not normalized !",
                 invalid_argument);
  }
};

/**
 * @brief Get the inverse of an SE2 object.
 * @note See Eqs. (154, 160).
 */
template <typename _Derived>
struct ExprEvaluator<InverseExpr<SE2Base<_Derived>>>
{
  using Ret = ReturnType<InverseExpr<SE2Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J)
  {
    using std::cos;
    using std::sin;

    if (J)
    {
      (*J) = -m.adj();
    }

    return Ret(-m.x()*m.real() - m.y()*m.imag(),
                m.x()*m.imag() - m.y()*m.real(),
                            -m.angle()          );
  }
};

/**
 * @brief Composition of two SE2 elements.
 * @note See Eq. (155) & Eqs. (161,162).
 */
template <typename _Derived, typename _DerivedOther>
struct ExprEvaluator<ComposeExpr<SE2Base<_Derived>, SE2Base<_DerivedOther>>>
{
  using Ret = ReturnType<ComposeExpr<SE2Base<_Derived>, SE2Base<_DerivedOther>>>;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 typename Lhs::OptJacobianRef& J_mc_ma,
                 typename Rhs::OptJacobianRef& J_mc_mb)
  {
    using std::abs;
    using Scalar = typename Lhs::Scalar;

    static_assert(
      std::is_base_of<SE2Base<_DerivedOther>, Rhs>::value,
      "Argument does not inherit from SE2Base !");

    if (J_mc_ma)
    {
      (*J_mc_ma) = rhs.inverse().adj();
    }

    if (J_mc_mb)
    {
      J_mc_mb->setIdentity();
    }

    const auto& lhs_se2 = static_cast<const SE2Base<_Derived>&>(lhs);
    const auto& rhs_se2 = static_cast<const SE2Base<_DerivedOther>&>(rhs);

    const Scalar lhs_real = lhs_se2.real(); // cos(t)
    const Scalar lhs_imag = lhs_se2.imag(); // sin(t)
    const Scalar rhs_real = rhs_se2.real();
    const Scalar rhs_imag = rhs_se2.imag();

    Scalar ret_real = lhs_real * rhs_real - lhs_imag * rhs_imag;
    Scalar ret_imag = lhs_real * rhs_imag + lhs_imag * rhs_real;

    const Scalar ret_sqnorm = ret_real*ret_real+ret_imag*ret_imag;

    if (abs(ret_sqnorm-Scalar(1)) > Constants<Scalar>::eps_s)
    {
      const Scalar scale = approxSqrtInv(ret_sqnorm);
      ret_real *= scale;
      ret_imag *= scale;
    }

    return Ret(
      lhs_real * rhs_se2.x() - lhs_imag * rhs_se2.y() + lhs_se2.x(),
      lhs_imag * rhs_se2.x() + lhs_real * rhs_se2.y() + lhs_se2.y(),
      ret_real, ret_imag
    );
  }
};

/**
 * @brief Rigid motion (SE2) action on a 2D point.
 * @note See Eq. (165) & Eqs. (166,167).
 */
template <typename _Derived, typename _DerivedOther>
struct ExprEvaluator<ActExpr<SE2Base<_Derived>, _DerivedOther>>
{
  using Scalar = typename _Derived::Scalar;
  using OptJacobianLhsRef = tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 3>>>;
  using OptJacobianRhsRef = tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 2>>>;

  using Ret = ReturnType<ActExpr<SE2Base<_Derived>, _DerivedOther>>;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 OptJacobianLhsRef& J_ret_lhs,
                 OptJacobianRhsRef& J_ret_rhs)
  {
    using Rotation = typename _Derived::Rotation;

    assert_vector_dim(rhs, 2);
    const Rotation R(lhs.rotation());

    if (J_ret_lhs)
    {
      J_ret_lhs->template topLeftCorner<2,2>()  = R;
      J_ret_lhs->template topRightCorner<2,1>() = R * (skew(Scalar(1)) * rhs);
    }

    if (J_ret_rhs)
    {
      (*J_ret_rhs) = R;
    }

    return lhs.translation() + R * rhs;
  }
};

/**
 * @brief Get the adjoint matrix of an SE2 object.
 * @note See Eq. (159).
 */
template <typename _Derived>
struct ExprEvaluator<AdjExpr<SE2Base<_Derived>>>
{
  using Ret = ReturnType<AdjExpr<SE2Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    Ret Adj = Ret::Identity();
    Adj.template topLeftCorner<2,2>() = m.rotation();
    Adj(0,2) =  m.y();
    Adj(1,2) = -m.x();
    return Adj;
  }
};

/**
 * @brief Get the SE2 corresponding Lie algebra element in vector form.
 * @note This is the log() map in vector form.
 * @note See Eqs. (157, 158).
 */
template <typename _Derived>
struct ExprEvaluator<LogExpr<SE2Base<_Derived>>>
{
  using Ret = ReturnType<LogExpr<SE2Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J)
  {
    using Scalar = typename T::Scalar;

    using std::abs;
    using std::cos;
    using std::sin;
    using std::atan2;

    const Scalar cos_theta = m.coeffs()[2];
    const Scalar sin_theta = m.coeffs()[3];
    const Scalar theta     = atan2(sin_theta, cos_theta);
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

    Ret tan( A * m.x() + B * m.y(),
            -B * m.x() + A * m.y(),
                     theta         );

    if (J)
    {
      (*J) = tan.rjacinv();
    }

    return tan;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SE2_EXPR_H_
