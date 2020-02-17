#ifndef _MANIF_MANIF_IMPL_EXPR_RJACINV_H_
#define _MANIF_MANIF_IMPL_EXPR_RJACINV_H_

namespace manif {

MANIF_DECLARE_UNARY_EXPR(Rjacinv);

namespace internal {

// Default expression return type

template <template <typename _TangentDerived> class _TangentDerivedBase, typename _TangentDerived>
struct ReturnTypeHelper<RjacinvExpr<_TangentDerivedBase<_TangentDerived>>>
{
  using type = typename internal::traits<_TangentDerived>::Jacobian;
};

// Default expression evaluation

template <typename _TangentDerived>
struct ExprEvaluator<RjacinvExpr<_TangentDerived>>
{
  using Ret = ReturnType<RjacinvExpr<_TangentDerived>>;

  template <typename Lhs>
  static Ret run(const Lhs& lhs)
  {
    return lhs.rjac().inverse();
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_RJACINV_H_
