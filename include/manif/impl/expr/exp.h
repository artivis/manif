#ifndef _MANIF_MANIF_IMPL_EXPR_EXP_H_
#define _MANIF_MANIF_IMPL_EXPR_EXP_H_

namespace manif {

MANIF_DECLARE_UNARY_JAC_EXPR(Exp);

namespace internal {

// Default expression return type

template <template <typename _LieGroupDerived> class _LieGroupDerivedBase, typename _LieGroupDerived>
struct ReturnTypeHelper<ExpExpr<_LieGroupDerivedBase<_LieGroupDerived>>>
{
  using type = typename internal::traits<_LieGroupDerived>::LieGroup;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_EXP_H_
