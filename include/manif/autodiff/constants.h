#ifndef _MANIF_MANIF_AUTODIFF_CONSTANTS_H_
#define _MANIF_MANIF_AUTODIFF_CONSTANTS_H_

namespace {
  // size_t without includes.
  using size_type = decltype(alignof(char));
}

namespace autodiff {
namespace detail {
  template <typename T, typename G> struct Dual;
  template <size_type N, typename T> class Real;
} // namespace detail
} // namespace autodiff

namespace manif {

/// @brief Specialize Constants traits for autodiff::Dual type
template <typename Scalar, typename G>
struct Constants<autodiff::detail::Dual<Scalar, G>> {
  static const autodiff::detail::Dual<Scalar, G> eps;
};

template <typename Scalar, typename G>
const autodiff::detail::Dual<Scalar, G>
Constants<autodiff::detail::Dual<Scalar, G>>::eps =
  autodiff::detail::Dual<Scalar, G>(Constants<Scalar>::eps);

/// @brief Specialize Constants traits for autodiff::Real type
template <size_type N, typename T>
struct Constants<autodiff::detail::Real<N, T>> {
  static const autodiff::detail::Real<N, T> eps;
};

template <size_type N, typename T>
const autodiff::detail::Real<N, T>
Constants<autodiff::detail::Real<N, T>>::eps =
  autodiff::detail::Real<N, T>(Constants<T>::eps);

} // namespace manif

#endif // _MANIF_MANIF_AUTODIFF_CONSTANTS_H_
