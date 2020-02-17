#ifndef _MANIF_MANIF_IMPL_EXPR_LOG_H_
#define _MANIF_MANIF_IMPL_EXPR_LOG_H_

namespace manif {

MANIF_DECLARE_UNARY_JAC_EXPR(Log);

namespace internal {

// Default expression return type

template <template <typename _LieGroupDerived> class _LieGroupDerivedBase, typename _LieGroupDerived>
struct ReturnTypeHelper<LogExpr<_LieGroupDerivedBase<_LieGroupDerived>>>
{
  using type = typename internal::traits<_LieGroupDerived>::Tangent;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_LOG_H_
