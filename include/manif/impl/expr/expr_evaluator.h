#ifndef _MANIF_MANIF_IMPL_EXPR_EVALUATOR_H_
#define _MANIF_MANIF_IMPL_EXPR_EVALUATOR_H_

// #include "manif/impl/traits.h"

namespace manif {
namespace internal {

/** Base class for all expressions */
template <typename... Expr>
struct ExprEvaluator
{
  template <typename T, typename J>
  static typename T::LieGroup run(const T&, J&)
  {
    /// @todo print actual Derived type
    static_assert(internal::constexpr_false<Expr...>(),
                  "ExprEvaluator not overloaded for Derived type!");
                  return T::LieGroup();
  }

  template <typename Lhs, typename Rhs, typename J0, typename J1>
  static typename Lhs::LieGroup run(const Lhs&, const Rhs&, J0&, J1&)
  {
    /// @todo print actual Derived type
    static_assert(internal::constexpr_false<Expr...>(),
                  "ExprEvaluator not overloaded for Derived type!");
                  return Lhs::LieGroup();
  }
};

}  // namespace internal
}  // namespace manif

#endif  // _MANIF_MANIF_IMPL_EXPR_EVALUATOR_H_
