#ifndef _MANIF_MANIF_SGAL3_PROPERTIES_H_
#define _MANIF_MANIF_SGAL3_PROPERTIES_H_

#include "manif/impl/traits.h"

namespace manif {

// Forward declaration
template <typename _Derived> struct SGal3Base;
template <typename _Derived> struct SGal3TangentBase;

namespace internal {

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<SGal3Base<_Derived>> {
  static constexpr int Dim = 3; /// @brief Space dimension
  static constexpr int DoF = 10; /// @brief Degrees of freedom
};

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<SGal3TangentBase<_Derived>> {
  static constexpr int Dim = 3; /// @brief Space dimension
  static constexpr int DoF = 10; /// @brief Degrees of freedom
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SGAL3_PROPERTIES_H_
