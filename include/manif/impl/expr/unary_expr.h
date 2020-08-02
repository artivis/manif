#ifndef _MANIF_MANIF_IMPL_UNARY_EXPR_H_
#define _MANIF_MANIF_IMPL_UNARY_EXPR_H_

namespace manif {
namespace internal {

/** Base class for all unary expressions */
template <template <typename _LieGroupDerived> class _ExprDerived,
          typename _LieGroupDerived>
struct UnaryExpr
  : ExprBase<_ExprDerived<_LieGroupDerived>>
{
  using LieGroupDerived = _LieGroupDerived;
  using Derived = _ExprDerived<LieGroupDerived>;
  using Base = ExprBase<Derived>;

  using Ret = ReturnType<Derived>;

  using Base::derived;
  using Base::eval;

  UnaryExpr(const LieGroupDerived& lhsptr)
    : lhsptr_(lhsptr) { }

protected:

  friend Base;
  Ret evalImpl() const
  {
    return internal::ExprEvaluator<Derived>::run(lhsptr_);
  }

  const LieGroupDerived& lhsptr_;
};

}  // namespace internal
}  // namespace manif

#define MANIF_DECLARE_UNARY_EXPR(EXPR)                                            \
template <typename _LieGroupDerived>                                              \
struct EXPR##Expr                                                                 \
  : manif::internal::UnaryExpr<::manif::EXPR##Expr, _LieGroupDerived> {           \
  using Base = manif::internal::UnaryExpr<::manif::EXPR##Expr, _LieGroupDerived>; \
  using LieGroup = typename _LieGroupDerived::LieGroup;                           \
  using Base::Base; using Base::eval;                                             \
};

#endif // _MANIF_MANIF_IMPL_UNARY_EXPR_H_
