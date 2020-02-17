#ifndef _MANIF_MANIF_IMPL_EXPR_BASE_H_
#define _MANIF_MANIF_IMPL_EXPR_BASE_H_

// #include "manif/impl/core/crtp.h"
// #include "manif/impl/lie_group_base.h"

namespace manif {
namespace internal {

template <typename _Expr>
struct ReturnTypeHelper;

}

template <typename _Expr>
using ReturnType = typename internal::ReturnTypeHelper<_Expr>::type;

}

namespace manif {
namespace internal {

/** Base class for all expressions */
template <typename _ExprDerived>
struct ExprBase
  // : LieGroupBase<_ExprDerived>
  : crtp<_ExprDerived>
{
  // using Base = LieGroupBase<_ExprDerived>;
  using Base = crtp<_ExprDerived>;

  using Base::derived;

  // explicit operator typename _ExprDerived::LieGroup() const
  // { return derived().evalImpl(); }

  ReturnType<_ExprDerived> eval() const &
  {
    return derived().evalImpl();
  }

  // ReturnType<_ExprDerived> eval() &
  // ReturnType<_ExprDerived> eval() &&
};

}  // namespace internal
}  // namespace manif

#endif  // _MANIF_MANIF_IMPL_EXPR_BASE_H_
