#ifndef _MANIF_MANIF_IMPL_EXPR_ADJ_H_
#define _MANIF_MANIF_IMPL_EXPR_ADJ_H_

namespace manif {

MANIF_DECLARE_UNARY_EXPR(Adj);

namespace internal {

// Default expression return type

template <template <typename _LieGroupDerived> class _LieGroupDerivedBase, typename _LieGroupDerived>
struct ReturnTypeHelper<AdjExpr<_LieGroupDerivedBase<_LieGroupDerived>>>
{
  using type = typename internal::traits<_LieGroupDerived>::Jacobian;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_ADJ_H_
