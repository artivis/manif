#ifndef _MANIF_MANIF_IMPL_BRACKET_H_
#define _MANIF_MANIF_IMPL_BRACKET_H_

namespace manif {
namespace internal {

template <typename Derived>
struct BracketEvaluatorImpl {
  template <typename TL, typename TR>
  static typename Derived::Tangent run(const TL& a, const TR& b) {
    return a.smallAdj() * b;
  }
};

template <typename Derived, typename DerivedOther>
struct BracketEvaluator : BracketEvaluatorImpl<Derived> {
  using Base = BracketEvaluatorImpl<Derived>;

  BracketEvaluator(const Derived& xptr, const DerivedOther& xptr_o)
  : xptr_(xptr), xptr_o_(xptr_o) {}

  typename Derived::Tangent run() {
    return Base::run(xptr_, xptr_o_);
  }

protected:

  const Derived& xptr_;
  const DerivedOther& xptr_o_;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_BRACKET_H_
