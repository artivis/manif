#ifndef _MANIF_MANIF_IMPL_EXPR_HAT_H_
#define _MANIF_MANIF_IMPL_EXPR_HAT_H_

namespace manif {

MANIF_DECLARE_UNARY_EXPR(Hat);

namespace internal {

// Default expression return type

template <template <typename _LieGroupDerived> class _LieGroupDerivedBase, typename _LieGroupDerived>
struct ReturnTypeHelper<HatExpr<_LieGroupDerivedBase<_LieGroupDerived>>>
{
  using type = typename internal::traits<_LieGroupDerived>::LieAlg;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_HAT_H_
