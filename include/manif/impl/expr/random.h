#ifndef _MANIF_MANIF_IMPL_EXPR_RANDOM_H_
#define _MANIF_MANIF_IMPL_EXPR_RANDOM_H_

namespace manif {

MANIF_DECLARE_UNARY_EXPR(Random);

namespace internal {

// Default expression return type

template <template <typename _LieGroupDerived> class _LieGroupDerivedBase, typename _LieGroupDerived>
struct ReturnTypeHelper<RandomExpr<_LieGroupDerivedBase<_LieGroupDerived>>>
{
  using type = typename internal::traits<_LieGroupDerived>::LieGroup&;
};

// Default expression evaluation

template <template <typename _LieGroupDerived> class _LieGroupDerivedBase, typename _LieGroupDerived>
struct ExprEvaluator<RandomExpr<_LieGroupDerivedBase<_LieGroupDerived>>>
{
  using Ret = ReturnType<RandomExpr<_LieGroupDerivedBase<_LieGroupDerived>>>;

  template <typename T>
  static Ret run(T& m)
  {
    using Tangent = typename internal::traits<_LieGroupDerived>::Tangent;
    m = Tangent::Random().exp();
    return m;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_RANDOM_H_
