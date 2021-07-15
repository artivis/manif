#ifndef _MANIF_MANIF_RxSO2_PROPERTIES_H_
#define _MANIF_MANIF_RxSO2_PROPERTIES_H_

#include "manif/impl/traits.h"

namespace manif {

// Forward declaration
template <typename _Derived> struct RxSO2Base;
template <typename _Derived> struct RxSO2TangentBase;

namespace internal {

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<RxSO2Base<_Derived>>
{
  static constexpr int Dim = 2; /// @brief Space dimension
  static constexpr int DoF = 2; /// @brief Degrees of freedom
};

//! traits specialization
template <typename _Derived>
struct LieGroupProperties<RxSO2TangentBase<_Derived>>
{
  static constexpr int Dim = 2; /// @brief Space dimension
  static constexpr int DoF = 2; /// @brief Degrees of freedom
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_RxSO2_PROPERTIES_H_
