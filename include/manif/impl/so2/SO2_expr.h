#ifndef _MANIF_MANIF_SO2_EXPR_H_
#define _MANIF_MANIF_SO2_EXPR_H_

namespace manif {
namespace internal {

//! @brief Assignment assert specialization for SO2Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<SO2Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)
  {
    using std::abs;
    using Scalar = typename SO2Base<Derived>::Scalar;
    MANIF_CHECK(abs(data.norm()-Scalar(1)) < Constants<Scalar>::eps_s,
                "SO2 assigned data not normalized !",
                invalid_argument);
  }
};

/**
 * @brief Get the inverse of this.
 * @note z^-1 = z*
 * @note See Eqs. (118,124).
 */
template <typename _Derived>
struct ExprEvaluator<InverseExpr<SO2Base<_Derived>>>
{
  using Ret = ReturnType<InverseExpr<SO2Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J)
  {
    using Scalar = typename traits<_Derived>::Scalar;

    if (J)
      J->setConstant(Scalar(-1));

    return Ret(m.real(), -m.imag());
  }
};

/**
 * @brief Composition of this and another SO2 element.
 * @note z_c = z_a z_b.
 * @note See Eq. (125).
 */
template <typename _Derived, typename _DerivedOther>
struct ExprEvaluator<ComposeExpr<SO2Base<_Derived>, SO2Base<_DerivedOther>>>
{
  using Ret = ReturnType<ComposeExpr<SO2Base<_Derived>, SO2Base<_DerivedOther>>>;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 typename Lhs::OptJacobianRef& J_mc_ma,
                 typename Rhs::OptJacobianRef& J_mc_mb)
  {
    using std::abs;
    using Scalar = typename traits<_Derived>::Scalar;

    static_assert(
      std::is_base_of<SO2Base<_DerivedOther>, Rhs>::value,
      "Argument does not inherit from SO2Base !");

    if (J_mc_ma)
      J_mc_ma->setConstant(Scalar(1));

    if (J_mc_mb)
      J_mc_mb->setConstant(Scalar(1));

    // const auto& rhs = static_cast<const SO2Base<_DerivedOther>&>(m);

    Scalar ret_real = lhs.real() * rhs.real() - lhs.imag() * rhs.imag();
    Scalar ret_imag = lhs.real() * rhs.imag() + lhs.imag() * rhs.real();

    const Scalar ret_sqnorm = ret_real*ret_real+ret_imag*ret_imag;

    if (abs(ret_sqnorm-Scalar(1)) > Constants<Scalar>::eps_s)
    {
      const Scalar scale = approxSqrtInv(ret_sqnorm);
      ret_real *= scale;
      ret_imag *= scale;
    }

    return Ret(ret_real, ret_imag);
  }
};

/**
 * @brief Rotation action on a 2-vector.
 * @note See Eqs. (129, 130).
 */
template <typename _Derived, typename _DerivedOther>
struct ExprEvaluator<ActExpr<SO2Base<_Derived>, _DerivedOther>>
{
  using Scalar = typename _Derived::Scalar;
  using OptJacobianLhsRef = tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 1>>>;
  using OptJacobianRhsRef = tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 2, 2>>>;

  using Ret = ReturnType<ActExpr<SO2Base<_Derived>, _DerivedOther>>;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 OptJacobianLhsRef& J_ret_lhs,
                 OptJacobianRhsRef& J_ret_rhs)
  {
    using Scalar = typename Lhs::Scalar;
    using Rotation = typename Lhs::Rotation;

    assert_vector_dim(rhs, 2);

    const Rotation R(lhs.rotation());

    if (J_ret_lhs)
    {
      (*J_ret_lhs) = R * skew(Scalar(1)) * rhs;
    }

    if (J_ret_rhs)
    {
      (*J_ret_rhs) = R;
    }

    return R * rhs;
  }
};

/**
 * @brief Get the ajoint matrix of SO2 at this.
 * @note See Eqs. (123).
 */
template <typename _Derived>
struct ExprEvaluator<AdjExpr<SO2Base<_Derived>>>
{
  using Ret = ReturnType<AdjExpr<SO2Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename SO2Base<_Derived>::Jacobian;
    using Scalar = typename SO2Base<_Derived>::Scalar;

    static const Ret adj = Jacobian::Constant(Scalar(1));
    return adj;
  }
};

/**
 * @brief Get the SO2 corresponding Lie algebra element in vector form.
 * @note This is the log() map in vector form.
 * @note See Eq. (115) & Eqs. (79,126).
 * @see SO2Tangent.
 */
template <typename _Derived>
struct ExprEvaluator<LogExpr<SO2Base<_Derived>>>
{
  using Ret = ReturnType<LogExpr<SO2Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J)
  {
    using Scalar = typename traits<_Derived>::Scalar;

    if (J)
      J->setConstant(Scalar(1));

    return Ret(m.angle());
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SO2_EXPR_H_
