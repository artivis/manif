#ifndef _MANIF_MANIF_IMPL_ASSIGNMENT_ASSERT_H_
#define _MANIF_MANIF_IMPL_ASSIGNMENT_ASSERT_H_

namespace manif {
namespace internal {

template <typename Derived>
struct AssignmentEvaluatorImpl
{
  template <typename T> static void run_impl(const T&) { }
};

template <typename Derived>
struct AssignmentEvaluator : AssignmentEvaluatorImpl<Derived>
{
  using Base = AssignmentEvaluatorImpl<Derived>;

  template <typename T> void run(T&& t) { Base::run_impl(std::forward<T>(t)); }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_ASSIGNMENT_ASSERT_H_
