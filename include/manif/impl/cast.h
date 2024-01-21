#ifndef _MANIF_MANIF_IMPL_CAST_H_
#define _MANIF_MANIF_IMPL_CAST_H_

namespace manif {
namespace internal {

template <typename Derived, typename NewScalar>
struct CastEvaluatorImpl {
  template <typename T>
  static auto run(const T& o) -> typename T::template LieGroupTemplate<NewScalar> {
    return typename T::template LieGroupTemplate<NewScalar>(
      o.coeffs().template cast<NewScalar>()
    );
  }
};

template <typename Derived, typename NewScalar>
struct CastEvaluator : CastEvaluatorImpl<Derived, NewScalar> {
  using Base = CastEvaluatorImpl<Derived, NewScalar>;

  CastEvaluator(const Derived& xptr) : xptr_(xptr) {}

  auto run() const -> typename Derived::template LieGroupTemplate<NewScalar> {
    return Base::run(xptr_);
  }

protected:

  const Derived& xptr_;
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_CAST_H_
