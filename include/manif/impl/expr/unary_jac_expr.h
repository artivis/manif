#ifndef _MANIF_MANIF_IMPL_UNARY_JAC_EXPR_H_
#define _MANIF_MANIF_IMPL_UNARY_JAC_EXPR_H_

namespace manif {
namespace internal {

/** Base class for all unary expressions */
template <template <typename _LieGroupDerived> class _ExprDerived,
          typename _LieGroupDerived>
struct UnaryJacExpr
  : ExprBase<_ExprDerived<_LieGroupDerived>>
{
  using LieGroupDerived = _LieGroupDerived;
  using Derived = _ExprDerived<LieGroupDerived>;
  using Base = ExprBase<Derived>;

  using OptJacobianRef = typename LieGroupDerived::OptJacobianRef;

  using ExprReturnType = ReturnType<Derived>;

  using Base::derived;
  using Base::eval;

  UnaryJacExpr(const LieGroupDerived& lhsptr,
               OptJacobianRef& J_ret_lhs = {})
    : lhsptr_(lhsptr)
    , J_ret_lhs_(J_ret_lhs) { }

protected:

  friend Base;
  ExprReturnType evalImpl() const
  {
    return internal::ExprEvaluator<Derived>::run(lhsptr_, J_ret_lhs_);
  }

  const LieGroupDerived& lhsptr_;
  mutable OptJacobianRef J_ret_lhs_;
};

}  // namespace internal
}  // namespace manif

#define MANIF_DECLARE_UNARY_JAC_EXPR(EXPR)                                  \
template <typename _LieGroupDerived>                                        \
struct EXPR##Expr                                                           \
  : manif::internal::UnaryJacExpr<EXPR##Expr, _LieGroupDerived> {           \
  using Base = manif::internal::UnaryJacExpr<EXPR##Expr, _LieGroupDerived>; \
  using LieGroup = typename _LieGroupDerived::LieGroup;                     \
  using Base::Base; using Base::eval;                                       \
};

#endif  // _MANIF_MANIF_IMPL_UNARY_JAC_EXPR_H_
