#ifndef _MANIF_MANIF_AUTODIFF_CONSTANTS_H_
#define _MANIF_MANIF_AUTODIFF_CONSTANTS_H_

namespace autodiff {
namespace detail {
  template <typename T, typename G> struct Dual;
} // namespace detail
} // namespace autodiff

namespace manif {

/// @brief Specialize Constants traits for the float-based autodiff::dual type
template <typename G>
struct Constants<autodiff::detail::Dual<float, G>> {
  static const autodiff::detail::Dual<float, G> eps;
};

template <typename G>
const autodiff::detail::Dual<float, G>
Constants<autodiff::detail::Dual<float, G>>::eps = autodiff::detail::Dual<float, G>(1e-6);

/// @brief Specialize Constants traits for the double-based autodiff::dual type
template <typename G>
struct Constants<autodiff::detail::Dual<double, G>> {
  static const autodiff::detail::Dual<double, G> eps;
};

template <typename G>
const autodiff::detail::Dual<double, G>
Constants<autodiff::detail::Dual<double, G>>::eps = autodiff::detail::Dual<double, G>(1e-14);

} // namespace manif

#endif // _MANIF_MANIF_AUTODIFF_CONSTANTS_H_
