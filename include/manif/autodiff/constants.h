#ifndef _MANIF_MANIF_AUTODIFF_CONSTANTS_H_
#define _MANIF_MANIF_AUTODIFF_CONSTANTS_H_

namespace autodiff {
namespace detail {
  template <typename T, typename G> struct Dual;
} // namespace detail
} // namespace autodiff

namespace manif {

/// @brief Specialize Constants traits for the float-based autodiff::dual type
template <typename Scalar, typename G>
struct Constants<autodiff::detail::Dual<Scalar, G>> {
  static const autodiff::detail::Dual<Scalar, G> eps;
};

template <typename Scalar, typename G>
const autodiff::detail::Dual<Scalar, G>
Constants<autodiff::detail::Dual<Scalar, G>>::eps =
  autodiff::detail::Dual<Scalar, G>(Constants<Scalar>::eps);

} // namespace manif

#endif // _MANIF_MANIF_AUTODIFF_CONSTANTS_H_
