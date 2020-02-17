#ifndef _MANIF_MANIF_IMPL_EXPR_RMINUS_H_
#define _MANIF_MANIF_IMPL_EXPR_RMINUS_H_

namespace manif {

MANIF_DECLARE_BINARY_EXPR(Rminus);

namespace internal {

// Default expression return type

template <template <typename _LhsLieGroupDerived> class _LhsLieGroupDerivedBase,
          typename _RhsLieGroupDerived,
          typename _LhsLieGroupDerived>
struct ReturnTypeHelper<RminusExpr<_LhsLieGroupDerivedBase<_LhsLieGroupDerived>, _RhsLieGroupDerived>>
{
  using type = typename internal::traits<_LhsLieGroupDerived>::Tangent;
};

// Default expression evaluation

template <typename _LhsLieGroupDerived, typename _RhsLieGroupDerived>
struct ExprEvaluator<RminusExpr<_LhsLieGroupDerived, _RhsLieGroupDerived>>
{
  using Ret = ReturnType<RminusExpr<_LhsLieGroupDerived, _RhsLieGroupDerived>>;

  using OptJacobianLhsRef = typename _LhsLieGroupDerived::OptJacobianRef;
  using OptJacobianRhsRef = typename _RhsLieGroupDerived::OptJacobianRef;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 OptJacobianLhsRef& J_ret_rhs,
                 OptJacobianRhsRef& J_ret_lhs)
  {
    const Ret ret = rhs.inverse().compose(lhs).log();

    if (J_ret_rhs)
    {
      (*J_ret_rhs) = ret.rjacinv();
    }
    if (J_ret_lhs)
    {
      (*J_ret_lhs) = -(-ret).rjacinv();
    }

    return ret;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_RMINUS_H_
