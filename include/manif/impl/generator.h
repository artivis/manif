#ifndef _MANIF_MANIF_GENERATOR_H_
#define _MANIF_MANIF_GENERATOR_H_

namespace manif {
namespace internal {

template <typename Derived>
struct GeneratorEvaluator
{
  static typename Derived::Basis
  run(const int)
  {
    /// @todo print actual Derived type
    static_assert(constexpr_false<Derived>(),
                  "GeneratorEvaluator not overloaded for Derived type!");
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_GENERATOR_H_ */
