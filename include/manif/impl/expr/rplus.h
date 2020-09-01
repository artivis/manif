#ifndef _MANIF_MANIF_IMPL_EXPR_RPLUS_H_
#define _MANIF_MANIF_IMPL_EXPR_RPLUS_H_

namespace manif {

MANIF_DECLARE_BINARY_EXPR(Rplus);

namespace internal {

// Default expression return type

template <template <typename _LhsLieGroupDerived> class _LhsLieGroupDerivedBase,
          typename _RhsTangentDerived,
          typename _LhsLieGroupDerived>
struct ReturnTypeHelper<RplusExpr<_LhsLieGroupDerivedBase<_LhsLieGroupDerived>, _RhsTangentDerived>>
{
  using type = typename internal::traits<_LhsLieGroupDerived>::LieGroup;
};

// Default expression evaluation

template <typename _LhsLieGroupDerived, typename _RhsTangentDerived>
struct ExprEvaluator<RplusExpr<_LhsLieGroupDerived, _RhsTangentDerived>>
{
  using Ret = ReturnType<RplusExpr<_LhsLieGroupDerived, _RhsTangentDerived>>;

  using OptJacobianLhsRef = typename _LhsLieGroupDerived::OptJacobianRef;
  using OptJacobianRhsRef = typename _RhsTangentDerived::OptJacobianRef;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 OptJacobianLhsRef& J_ret_lhs,
                 OptJacobianRhsRef& J_ret_rhs)
  {
    if (J_ret_rhs)
    {
      (*J_ret_rhs) = rhs.rjac();
    }

    return lhs.compose(rhs.exp(), J_ret_lhs);
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_RPLUS_H_
