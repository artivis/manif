#ifndef _MANIF_MANIF_IMPL_RANDOM_H_
#define _MANIF_MANIF_IMPL_RANDOM_H_

namespace manif {

template <class Derived> struct LieGroupBase;
template <class Derived> struct TangentBase;

namespace internal {

template <typename Derived>
struct RandomEvaluatorImpl
{
  template <typename T>
  static void run(TangentBase<T>& t)
  {
    t.coeffs().setRandom();
  }

  template <typename T>
  static void run(LieGroupBase<T>& t)
  {
    using Tangent = typename LieGroupBase<T>::Tangent;
    t = Tangent::Random().exp();
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
