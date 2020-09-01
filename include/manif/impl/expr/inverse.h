#ifndef _MANIF_MANIF_IMPL_EXPR_INVERSE_H_
#define _MANIF_MANIF_IMPL_EXPR_INVERSE_H_

namespace manif {

MANIF_DECLARE_UNARY_JAC_EXPR(Inverse);

// template <typename _LieGroupDerived>
// struct InverseExpr
//   : ExprBase<InverseExpr<_LieGroupDerived>>
// {
//   using LieGroupDerived = remove_cr_t<_LieGroupDerived>;
//
//   using Storage = storage_t<_LieGroupDerived>;
//
//   using Base = ExprBase<InverseExpr<_LieGroupDerived>>;
//
//   using OptJacobianRef = typename LieGroupDerived::OptJacobianRef;
//
//   using Ret = ReturnType<Derived>;
//
//   using Base::derived;
//   using Base::eval;
//
//   InverseExpr(_LieGroupDerived lhsptr,
//               OptJacobianRef& J_ret_lhs = {})
//     : lhsptr_(lhsptr)
//     , J_ret_lhs_(J_ret_lhs) { }
//
// protected:
//
//   friend Base;
//   Ret evalImpl() const
//   {
//     return internal::ExprEvaluator<Derived>::run(lhsptr_, J_ret_lhs_);
//   }
//
//   Storage lhsptr_;
//   mutable OptJacobianRef J_ret_lhs_;
// };

namespace internal {

// Default expression return type

template <template <typename _LieGroupDerived> class _LieGroupDerivedBase, typename _LieGroupDerived>
struct ReturnTypeHelper<InverseExpr<_LieGroupDerivedBase<_LieGroupDerived>>>
{
  using type = typename internal::traits<_LieGroupDerived>::LieGroup;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_INVERSE_H_
