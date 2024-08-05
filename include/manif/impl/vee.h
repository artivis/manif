#ifndef _MANIF_MANIF_IMPL_VEE_H_
#define _MANIF_MANIF_IMPL_VEE_H_

namespace manif {
namespace internal {

template <typename Derived>
struct VeeEvaluatorImpl {
  template <typename TL, typename TR>
  static void run(TL& t, const TR&) {
    static_assert(
      constexpr_false<Derived>(), "VeeEvaluator not overloaded for Derived type!"
    );
    // t.setRandom();
  }
};

template <typename Derived>
struct VeeEvaluator : VeeEvaluatorImpl<Derived> {
  using Base = VeeEvaluatorImpl<Derived>;

  VeeEvaluator(Derived& xptr) : xptr_(xptr) {}

  template <typename T>
  void run(const T& t) {
    Base::run(xptr_, t);
  }

protected:

  Derived& xptr_;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_VEE_H_
