#ifndef _MANIF_MANIF_IMPL_EXPR_BINARY_H_
#define _MANIF_MANIF_IMPL_EXPR_BINARY_H_

// #include "manif/impl/expr/expr_base.h"

namespace manif {
namespace internal {

/** Base class for all expressions */
template <template <typename _Lhs, typename _Rhs> class _ExprDerived,
          typename _Lhs, typename _Rhs>
struct ExprBinary
  : ExprBase<_ExprDerived<_Lhs, _Rhs>>
{
  using ExprDerived = _ExprDerived<_Lhs, _Rhs>;
  using Base = ExprBase<ExprDerived>;

  using LhsOptJacobianRef = typename _Lhs::OptJacobianRef;
  using RhsOptJacobianRef = typename _Rhs::OptJacobianRef;

  using Ret = ReturnType<ExprDerived>;

  using Base::derived;
  using Base::eval;

  ExprBinary(const _Lhs& lhsptr,
             const _Rhs& rhsptr,
             LhsOptJacobianRef& J_ret_lhs = {},
             RhsOptJacobianRef& J_ret_rhs = {})
    : lhsptr_(lhsptr)
    , rhsptr_(rhsptr)
    , J_ret_lhs_(J_ret_lhs)
    , J_ret_rhs_(J_ret_rhs) { }

protected:

  friend Base;
  Ret evalImpl() const
  {
    return internal::ExprEvaluator<ExprDerived>::run(
      lhsptr_, rhsptr_, J_ret_lhs_, J_ret_rhs_);
  }

  const _Lhs& lhsptr_;
  const _Rhs& rhsptr_;
  mutable LhsOptJacobianRef J_ret_lhs_;
  mutable RhsOptJacobianRef J_ret_rhs_;
};

}  // namespace internal
}  // namespace manif

#define MANIF_DECLARE_BINARY_EXPR(EXPR)                   \
template <typename LhsLieGroupDerived,                    \
          typename RhsLieGroupDerived>                    \
struct EXPR##Expr                                         \
  : manif::internal::ExprBinary<EXPR##Expr,               \
                         LhsLieGroupDerived,              \
                         RhsLieGroupDerived> {            \
  using Base = manif::internal::ExprBinary<EXPR##Expr,    \
                                    LhsLieGroupDerived,   \
                                    RhsLieGroupDerived>;  \
  using Base::Base; using Base::eval;                     \
};

#endif  // _MANIF_MANIF_IMPL_EXPR_BINARY_H_
