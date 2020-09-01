#ifndef _MANIF_MANIF_RNTANGENT_EXPR_H_
#define _MANIF_MANIF_RNTANGENT_EXPR_H_

namespace manif {
namespace internal {

/**
 * @brief Generator specialization for RnTangentBase objects.
 */
template <typename Derived>
struct GeneratorEvaluator<RnTangentBase<Derived>>
{
  static typename RnTangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    MANIF_CHECK(i<RnTangentBase<Derived>::DoF,
                "Index i must less than DoF!",
                invalid_argument);

    using LieAlg = typename RnTangentBase<Derived>::LieAlg;

    LieAlg Ei = LieAlg::Constant(0);

    Ei(i, RnTangentBase<Derived>::DoF) = 1;

    return Ei;
  }
};

/**
 * @brief Hat operator of Rn.
 * @note See Appendix E.
 */
template <typename _Derived>
struct ExprEvaluator<HatExpr<RnTangentBase<_Derived>>>
{
  using Ret = ReturnType<HatExpr<RnTangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using LieAlg = typename traits<_Derived>::LieAlg;

    LieAlg hat = LieAlg::Constant(0);
    hat.template topRightCorner<RnTangentBase<_Derived>::Dim, 1>() = m.coeffs();
    return hat;
  }
};

/**
 * @brief Get the Rn element.
 * @note This is the exp() map with the argument in vector form.
 * @note See Eqs. (184) and Eq. (191).
 */
template <typename _Derived>
struct ExprEvaluator<ExpExpr<RnTangentBase<_Derived>>>
{
  using Ret = ReturnType<ExpExpr<RnTangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J_m_t)
  {
    if (J_m_t)
    {
      J_m_t->setIdentity();
    }

    return m.coeffs();
  }
};

/**
 * @brief Get the right Jacobian of Rn.
 * @note See Eq. (191).
 */
template <typename _Derived>
struct ExprEvaluator<RjacExpr<RnTangentBase<_Derived>>>
{
  using Ret = ReturnType<RjacExpr<RnTangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename RnTangentBase<_Derived>::Jacobian;
    static const Ret Jr = Jacobian::Identity();
    return Jr;
  }
};

/**
 * @brief Get the inverse of the right Jacobian of Rn.
 * @note See Eq. (191).
 */
template <typename _Derived>
struct ExprEvaluator<RjacinvExpr<RnTangentBase<_Derived>>>
{
  using Ret = ReturnType<RjacinvExpr<RnTangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    return m.rjac();
  }
};

/**
 * @brief Get the left Jacobian of Rn.
 * @note See Eq. (191).
 */
template <typename _Derived>
struct ExprEvaluator<LjacExpr<RnTangentBase<_Derived>>>
{
  using Ret = ReturnType<LjacExpr<RnTangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    return m.rjac();
  }
};

/**
 * @brief Get the inverse of the right Jacobian of Rn.
 * @note See Eq. (191).
 */
template <typename _Derived>
struct ExprEvaluator<LjacinvExpr<RnTangentBase<_Derived>>>
{
  using Ret = ReturnType<LjacinvExpr<RnTangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    return m.ljac();
  }
};

/**
 * @brief smallAdj
 */
template <typename _Derived>
struct ExprEvaluator<SmallAdjExpr<RnTangentBase<_Derived>>>
{
  using Ret = ReturnType<SmallAdjExpr<RnTangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename RnTangentBase<_Derived>::Jacobian;
    static const Ret smallAdj = Jacobian::Constant(0);
    return smallAdj;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_RNTANGENT_EXPR_H_
