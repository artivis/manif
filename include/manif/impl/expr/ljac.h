#ifndef _MANIF_MANIF_IMPL_EXPR_LJAC_H_
#define _MANIF_MANIF_IMPL_EXPR_LJAC_H_

namespace manif {

MANIF_DECLARE_UNARY_EXPR(Ljac);

namespace internal {

// Default expression return type

template <template <typename _TangentDerived> class _TangentDerivedBase, typename _TangentDerived>
struct ReturnTypeHelper<LjacExpr<_TangentDerivedBase<_TangentDerived>>>
{
  using type = typename internal::traits<_TangentDerived>::Jacobian;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_LJAC_H_
