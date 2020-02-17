#ifndef _MANIF_MANIF_RN_EXPR_H_
#define _MANIF_MANIF_RN_EXPR_H_

namespace manif {
namespace internal {

  /**
   * @brief Get the inverse of this.
   * @note r^-1 = -r
   * @note See Appendix E and Eq. (189).
   */
template <typename _Derived>
struct ExprEvaluator<InverseExpr<RnBase<_Derived>>>
{
  using Ret = ReturnType<InverseExpr<RnBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J)
  {
    using Scalar = typename T::Scalar;

    if (J)
      J->setIdentity() *= Scalar(-1);

    return -m.coeffs();
  }
};

/**
 * @brief Composition of two Rn elements.
 * @note See Eq. (190).
 */
template <typename _Derived, typename _DerivedOther>
struct ExprEvaluator<ComposeExpr<RnBase<_Derived>, RnBase<_DerivedOther>>>
{
  using Ret = ReturnType<ComposeExpr<RnBase<_Derived>, RnBase<_DerivedOther>>>;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 typename Lhs::OptJacobianRef& J_mc_ma,
                 typename Rhs::OptJacobianRef& J_mc_mb)
  {
    using std::abs;

    static_assert(
      std::is_base_of<RnBase<_DerivedOther>, typename Rhs::LieGroup>::value,
      "Argument does not inherit from RnBase !");

    static_assert(Lhs::Dim==Rhs::Dim, "Dimension mismatch !");

    if (J_mc_ma)
      J_mc_ma->setIdentity();

    if (J_mc_mb)
      J_mc_mb->setIdentity();

    return lhs.coeffs() + rhs.coeffs();
  }
};

/**
 * @brief Translation action on a 2-vector.
 */
template <typename _Derived, typename _DerivedOther>
struct ExprEvaluator<ActExpr<RnBase<_Derived>, _DerivedOther>>
{
  using Scalar = typename _Derived::Scalar;
  using OptJacobianLhsRef = tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, _Derived::Dim, _Derived::DoF>>>;
  using OptJacobianRhsRef = tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, _Derived::Dim, _Derived::Dim>>>;

  using Ret = ReturnType<ActExpr<RnBase<_Derived>, _DerivedOther>>;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 OptJacobianLhsRef& J_ret_lhs,
                 OptJacobianRhsRef& J_ret_rhs)
  {
    static_assert_vector_dim(rhs, Lhs::Dim);

    if (J_ret_lhs)
    {
      J_ret_lhs->setIdentity();
    }

    if (J_ret_rhs)
    {
      J_ret_rhs->setIdentity();
    }

    return lhs.coeffs() + rhs;
  }
};

/**
 * @brief Get the ajoint matrix of Rn at this.
 * @note See Eqs. (188).
 */
template <typename _Derived>
struct ExprEvaluator<AdjExpr<RnBase<_Derived>>>
{
  using Ret = ReturnType<AdjExpr<RnBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename RnBase<_Derived>::Jacobian;
    static const Ret adj = Jacobian::Identity();
    return adj;
  }
};

/**
 * @brief Get the Rn corresponding Lie algebra element in vector form.
 * @note This is the log() map in vector form.
 * @note See Appendix E.
 * @see RnTangent.
 */
template <typename _Derived>
struct ExprEvaluator<LogExpr<RnBase<_Derived>>>
{
  using Ret = ReturnType<LogExpr<RnBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J)
  {
    if (J)
      J->setIdentity();

    return m.coeffs();
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_RN_EXPR_H_
