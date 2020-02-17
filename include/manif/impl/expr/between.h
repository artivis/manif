#ifndef _MANIF_MANIF_IMPL_EXPR_BETWEEN_H_
#define _MANIF_MANIF_IMPL_EXPR_BETWEEN_H_

namespace manif {

MANIF_DECLARE_BINARY_EXPR(Between);

namespace internal {

// Default expression return type

template <template <typename _LhsLieGroupDerived> class _LhsLieGroupDerivedBase,
          typename _RhsLieGroupDerived,
          typename _LhsLieGroupDerived>
struct ReturnTypeHelper<BetweenExpr<_LhsLieGroupDerivedBase<_LhsLieGroupDerived>, _RhsLieGroupDerived>>
{
  using type = typename internal::traits<_LhsLieGroupDerived>::LieGroup;
};

// Default expression evaluation

template <typename _LhsLieGroupDerived, typename _RhsLieGroupDerived>
struct ExprEvaluator<BetweenExpr<_LhsLieGroupDerived, _RhsLieGroupDerived>>
{
  using Expr = BetweenExpr<_LhsLieGroupDerived, _RhsLieGroupDerived>;
  using Ret = ReturnType<Expr>;

  using OptJacobianLhsRef = typename _LhsLieGroupDerived::OptJacobianRef;
  using OptJacobianRhsRef = typename _RhsLieGroupDerived::OptJacobianRef;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 OptJacobianLhsRef& J_ret_lhs,
                 OptJacobianRhsRef& J_ret_rhs)
  {
    const Ret ret = lhs.inverse().compose(rhs);

    if (J_ret_lhs)
    {
      *J_ret_lhs = -(ret.inverse().adj());
    }

    if (J_ret_rhs)
    {
      J_ret_rhs->setIdentity();
    }

    return ret;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_BETWEEN_H_
