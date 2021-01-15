#ifndef _MANIF_MANIF_RN_PROPERTIES_H_
#define _MANIF_MANIF_RN_PROPERTIES_H_

#include "manif/impl/traits.h"

namespace manif {

// Forward declaration
template <typename _Derived> struct RnBase;
template <typename _Derived> struct RnTangentBase;

namespace internal {

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<RnBase<_Derived>>
{
  static constexpr int Dim = traits<_Derived>::Dim; /// @brief Space dimension
  static constexpr int DoF = traits<_Derived>::Dim; /// @brief Degrees of freedom
};

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<RnTangentBase<_Derived>>
{
  static constexpr int Dim = traits<_Derived>::Dim; /// @brief Space dimension
  static constexpr int DoF = traits<_Derived>::Dim; /// @brief Degrees of freedom
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_RN_PROPERTIES_H_
