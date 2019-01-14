#ifndef _MANIF_MANIF_GENERATOR_H_
#define _MANIF_MANIF_GENERATOR_H_

namespace manif {
namespace internal {

template <typename Derived>
struct GeneratorEvaluator
{
  static typename Derived::LieAlg
  run(const int)
  {
    /// @todo print actual Derived type
    static_assert(constexpr_false<Derived>(),
                  "GeneratorEvaluator not overloaded for Derived type!");
  }
};

template <typename Derived>
struct WEvaluator
{
  static typename Derived::Jacobian
  run()
  {
    using Jacobian = typename Derived::Jacobian;

    auto computeW = []()
    {
      Jacobian W = Jacobian::Zero();

      for (int r=0; r<Derived::DoF; ++r)
        for (int c=0; c<Derived::DoF; ++c)
          W(r,c) = (Derived::Generator(r)*Derived::Generator(c).transpose()).trace();

      return W;
    };

    const static Jacobian W = computeW();

    return W;
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_GENERATOR_H_ */
