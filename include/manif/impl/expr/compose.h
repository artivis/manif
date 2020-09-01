#ifndef _MANIF_MANIF_IMPL_EXPR_COMPOSE_H_
#define _MANIF_MANIF_IMPL_EXPR_COMPOSE_H_

namespace manif {

MANIF_DECLARE_BINARY_EXPR(Compose);

namespace internal {

// Default expression return type

template <template <typename _LhsLieGroupDerived> class _LhsLieGroupDerivedBase,
          typename _RhsLieGroupDerived,
          typename _LhsLieGroupDerived>
struct ReturnTypeHelper<ComposeExpr<_LhsLieGroupDerivedBase<_LhsLieGroupDerived>, _RhsLieGroupDerived>>
{
  using type = typename internal::traits<_LhsLieGroupDerived>::LieGroup;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_COMPOSE_H_
