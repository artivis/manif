#ifndef _MANIF_MANIF_SO2_PROPERTIES_H_
#define _MANIF_MANIF_SO2_PROPERTIES_H_

#include "manif/impl/traits.h"

namespace manif {

// Forward declaration
template <typename _Derived> struct SO2Base;
template <typename _Derived> struct SO2TangentBase;

namespace internal {

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<SO2Base<_Derived>>
{
  static constexpr int Dim = 2; /// @brief Space dimension
  static constexpr int DoF = 1; /// @brief Degrees of freedom
};

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<SO2TangentBase<_Derived>>
{
  static constexpr int Dim = 2; /// @brief Space dimension
  static constexpr int DoF = 1; /// @brief Degrees of freedom
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_PROPERTIES_H_ */
