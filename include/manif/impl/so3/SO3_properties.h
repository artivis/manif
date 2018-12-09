#ifndef _MANIF_MANIF_SO3_PROPERTIES_H_
#define _MANIF_MANIF_SO3_PROPERTIES_H_

#include "manif/impl/traits.h"

namespace manif {

// Forward declaration
template <typename _Derived> struct SO3Base;
template <typename _Derived> struct SO3TangentBase;

namespace internal {

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<SO3Base<_Derived>>
{
  static constexpr int Dim = 3; /// @brief Space dimension
  static constexpr int DoF = 3; /// @brief Degrees of freedom
};

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<SO3TangentBase<_Derived>>
{
  static constexpr int Dim = 3; /// @brief Space dimension
  static constexpr int DoF = 3; /// @brief Degrees of freedom
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SO3_PROPERTIES_H_ */
