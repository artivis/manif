#ifndef _MANIF_MANIF_IMPL_EXPR_RJAC_H_
#define _MANIF_MANIF_IMPL_EXPR_RJAC_H_

namespace manif {

MANIF_DECLARE_UNARY_EXPR(Rjac);

namespace internal {

// Default expression return type

template <template <typename _TangentDerived> class _TangentDerivedBase, typename _TangentDerived>
struct ReturnTypeHelper<RjacExpr<_TangentDerivedBase<_TangentDerived>>>
{
  using type = typename internal::traits<_TangentDerived>::Jacobian;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_RJAC_H_
