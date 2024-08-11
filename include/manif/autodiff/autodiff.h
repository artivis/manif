#ifndef _MANIF_MANIF_AUTODIFF_AUTODIFF_H_
#define _MANIF_MANIF_AUTODIFF_AUTODIFF_H_

#include "manif/autodiff/constants.h"
#include "manif/autodiff/local_parameterization.h"

namespace manif::internal {
// @note: Unfortunately HigherOrderDual is a non-deducible context
// template <size_t N, typename T>
// struct is_ad<autodiff::HigherOrderDual<N, T>> : std::integral_constant<bool, true> { };

using dual0thf = autodiff::HigherOrderDual<0, float>;
using dual1stf = autodiff::HigherOrderDual<1, float>;
using dual2ndf = autodiff::HigherOrderDual<2, float>;
using dual3rdf = autodiff::HigherOrderDual<3, float>;
using dual4thf = autodiff::HigherOrderDual<4, float>;

template <> struct is_ad<autodiff::dual0th> : std::integral_constant<bool, true> { };
template <> struct is_ad<autodiff::dual1st> : std::integral_constant<bool, true> { };
template <> struct is_ad<autodiff::dual2nd> : std::integral_constant<bool, true> { };
template <> struct is_ad<autodiff::dual3rd> : std::integral_constant<bool, true> { };
template <> struct is_ad<autodiff::dual4th> : std::integral_constant<bool, true> { };

template <> struct is_ad<dual0thf> : std::integral_constant<bool, true> { };
template <> struct is_ad<dual1stf> : std::integral_constant<bool, true> { };
template <> struct is_ad<dual2ndf> : std::integral_constant<bool, true> { };
template <> struct is_ad<dual3rdf> : std::integral_constant<bool, true> { };
template <> struct is_ad<dual4thf> : std::integral_constant<bool, true> { };

template <size_t N, typename T>
struct is_ad<autodiff::Real<N, T>> : std::integral_constant<bool, true> { };
} // namespace manif

namespace autodiff::detail {
/// @brief VectorTraits specialization for Derived of LieGroupBase
template <typename T>
struct VectorTraits<T, EnableIf<manif::internal::is_manif_group<T>::value>> {
  using ValueType = typename T::Scalar;

  template <typename NewValueType>
  using ReplaceValueType = typename T::template LieGroupTemplate<NewValueType>;
};

/// @brief VectorTraits specialization for Derived of TangentBase
template <typename T>
struct VectorTraits<T, EnableIf<manif::internal::is_manif_tangent<T>::value>> {
  using ValueType = typename T::Scalar;

  template <typename NewValueType>
  using ReplaceValueType = typename T::template TangentTemplate<NewValueType>;
};
} // namespace autodiff::detail

#endif // _MANIF_MANIF_AUTODIFF_AUTODIFF_H_