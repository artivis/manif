#ifndef _MANIF_MANIF_IMPL_EXPR_LPLUS_H_
#define _MANIF_MANIF_IMPL_EXPR_LPLUS_H_

namespace manif {

MANIF_DECLARE_BINARY_EXPR(Lplus);

namespace internal {

// Default expression return type

template <template <typename _LhsLieGroupDerived> class _LhsLieGroupDerivedBase,
          typename _RhsTangentDerived,
          typename _LhsLieGroupDerived>
struct ReturnTypeHelper<LplusExpr<_LhsLieGroupDerivedBase<_LhsLieGroupDerived>, _RhsTangentDerived>>
{
  using type = typename internal::traits<_LhsLieGroupDerived>::LieGroup;
};

// Default expression evaluation

template <typename _LhsLieGroupDerived, typename _RhsTangentDerived>
struct ExprEvaluator<LplusExpr<_LhsLieGroupDerived, _RhsTangentDerived>>
{
  using Ret = ReturnType<LplusExpr<_LhsLieGroupDerived, _RhsTangentDerived>>;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 typename Lhs::OptJacobianRef& J_ret_lhs,
                 typename Rhs::OptJacobianRef& J_ret_rhs)
  {
    if (J_ret_lhs)
    {
      J_ret_lhs->setIdentity();
    }

    if (J_ret_rhs)
    {
      J_ret_rhs->noalias() = lhs.inverse().adj() * rhs.rjac();
    }

    return rhs.exp().compose(lhs);
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_LPLUS_H_
