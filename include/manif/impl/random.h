#ifndef _MANIF_MANIF_IMPL_RANDOM_H_
#define _MANIF_MANIF_IMPL_RANDOM_H_

namespace manif {
namespace internal {

template <typename Derived>
struct RandomEvaluatorImpl
{
  template <typename T>
  static void run(T&)
  {
    /// @todo print actual Derived type
    static_assert(constexpr_false<Derived>(),
                  "RandomEvaluator not overloaded for Derived type!");
  }
};

template <typename Derived>
struct RandomEvaluator : RandomEvaluatorImpl<Derived>
{
  using Base = RandomEvaluatorImpl<Derived>;

  RandomEvaluator(Derived& xptr) : xptr_(xptr) {}

  void run()
  {
    Base::run(xptr_);
  }

protected:

  Derived& xptr_;
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_IMPL_RANDOM_H_ */
