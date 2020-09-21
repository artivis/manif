#ifndef _MANIF_MANIF_DHU_PROPERTIES_H_
#define _MANIF_MANIF_DHU_PROPERTIES_H_

#include "manif/impl/traits.h"

namespace manif {

// Forward declaration
template <typename _Derived> struct DHuBase;
template <typename _Derived> struct DHuTangentBase;

namespace internal {

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<DHuBase<_Derived>>
{
  static constexpr int Dim = 3; /// @brief Space dimension
  static constexpr int DoF = 6; /// @brief Degrees of freedom
};

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<DHuTangentBase<_Derived>>
{
  static constexpr int Dim = 3; /// @brief Space dimension
  static constexpr int DoF = 6; /// @brief Degrees of freedom
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_DHU_PROPERTIES_H_
