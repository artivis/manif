#ifndef _MANIF_MANIF_IMPL_EXPR_LJACINV_H_
#define _MANIF_MANIF_IMPL_EXPR_LJACINV_H_

namespace manif {

MANIF_DECLARE_UNARY_EXPR(Ljacinv);

namespace internal {

// Default expression return type

template <template <typename _TangentDerived> class _TangentDerivedBase, typename _TangentDerived>
struct ReturnTypeHelper<LjacinvExpr<_TangentDerivedBase<_TangentDerived>>>
{
  using type = typename internal::traits<_TangentDerived>::Jacobian;
};

// Default expression evaluation

template <typename _TangentDerived>
struct ExprEvaluator<LjacinvExpr<_TangentDerived>>
{
  using Ret = ReturnType<LjacinvExpr<_TangentDerived>>;

  template <typename Lhs>
  static Ret run(const Lhs& lhs)
  {
    return lhs.ljac().inverse();
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_LJACINV_H_
