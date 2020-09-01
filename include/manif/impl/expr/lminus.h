#ifndef _MANIF_MANIF_IMPL_EXPR_LMINUS_H_
#define _MANIF_MANIF_IMPL_EXPR_LMINUS_H_

namespace manif {

MANIF_DECLARE_BINARY_EXPR(Lminus);

namespace internal {

// Default expression return type

template <template <typename _LhsLieGroupDerived> class _LhsLieGroupDerivedBase,
          typename _RhsLieGroupDerived,
          typename _LhsLieGroupDerived>
struct ReturnTypeHelper<LminusExpr<_LhsLieGroupDerivedBase<_LhsLieGroupDerived>, _RhsLieGroupDerived>>
{
  using type = typename internal::traits<_LhsLieGroupDerived>::Tangent;
};

// Default expression evaluation

template <typename _LhsLieGroupDerived, typename _RhsLieGroupDerived>
struct ExprEvaluator<LminusExpr<_LhsLieGroupDerived, _RhsLieGroupDerived>>
{
  using Ret = ReturnType<LminusExpr<_LhsLieGroupDerived, _RhsLieGroupDerived>>;

  using OptJacobianLhsRef = typename _LhsLieGroupDerived::OptJacobianRef;
  using OptJacobianRhsRef = typename _RhsLieGroupDerived::OptJacobianRef;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 OptJacobianLhsRef& J_ret_rhs,
                 OptJacobianRhsRef& J_ret_lhs)
  {

    using Jacobian = typename Lhs::Jacobian;

    const Ret ret = lhs.compose(rhs.inverse()).log();

    if (J_ret_rhs || J_ret_lhs)
    {
      const Jacobian J = ret.rjacinv() * rhs.adj();

      if (J_ret_rhs)
      {
        (*J_ret_rhs) =  J;
      }
      if (J_ret_lhs)
      {
        (*J_ret_lhs) = -J;
      }
    }

    return ret;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_EXPR_LMINUS_H_
