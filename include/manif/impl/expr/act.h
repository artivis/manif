#ifndef _MANIF_MANIF_IMPL_EXPR_ACT_H_
#define _MANIF_MANIF_IMPL_EXPR_ACT_H_

namespace manif {

// MANIF_DECLARE_BINARY_EXPR(Act);

template <typename _Lhs, typename _Rhs>
struct ActExpr
  : internal::ExprBase<ActExpr<_Lhs, _Rhs>>
{
  using ExprDerived = ActExpr<_Lhs, _Rhs>;
  using Base = internal::ExprBase<ExprDerived>;

  using LieGroup = typename _Lhs::LieGroup;
  using Scalar = typename internal::traits<LieGroup>::Scalar;

  using LhsOptJacobianRef = ::tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, internal::traits<LieGroup>::Dim, internal::traits<LieGroup>::DoF>>>;
  using RhsOptJacobianRef = ::tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, internal::traits<LieGroup>::Dim, internal::traits<LieGroup>::Dim>>>;

  using Ret = ReturnType<ExprDerived>;

  using Base::derived;
  using Base::eval;

  ActExpr(const _Lhs& lhsptr,
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

namespace internal {

template <template <typename _Derived> class _LieGroupBase,
          typename _Derived, typename _DerivedOther>
struct ReturnTypeHelper<ActExpr<_LieGroupBase<_Derived>, _DerivedOther>>
{
  using type = typename traits<_Derived>::Vector;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_ACT_H_
