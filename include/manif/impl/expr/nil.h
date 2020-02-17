#ifndef _MANIF_MANIF_IMPL_EXPR_NIL_H_
#define _MANIF_MANIF_IMPL_EXPR_NIL_H_

namespace manif {

MANIF_DECLARE_UNARY_EXPR(Nil);

namespace internal {

// Default expression return type

template <template <typename _LieGroupDerived> class _LieGroupDerivedBase, typename _LieGroupDerived>
struct ReturnTypeHelper<NilExpr<_LieGroupDerivedBase<_LieGroupDerived>>>
{
  using type = _LieGroupDerived;

  // This return type should be the object itself
  // but first we have to handle reference (&, &&)
  // using type = _LieGroupDerivedBase<_LieGroupDerived>;
};

// Default expression evaluation

template <template <typename _LieGroupDerived> class _LieGroupDerivedBase, typename _LieGroupDerived>
struct ExprEvaluator<NilExpr<_LieGroupDerivedBase<_LieGroupDerived>>>
{
  using Ret = ReturnType<NilExpr<_LieGroupDerivedBase<_LieGroupDerived>>>;

  template <typename T>
  static Ret run(T& m)
  {
    return m;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_NIL_H_
