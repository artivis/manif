#ifndef _MANIF_MANIF_R2SO2_PROPERTIES_H_
#define _MANIF_MANIF_R2SO2_PROPERTIES_H_

#include "manif/impl/traits.h"

namespace manif
{

// Forward declaration
template <typename _Derived> struct R2SO2Base;
template <typename _Derived> struct R2SO2TangentBase;

namespace internal
{

// traits specialization

template <typename _Derived>
struct ManifoldProperties<R2SO2Base<_Derived>>
{
  static constexpr int Dim = 2; /// @brief Space dimension
  static constexpr int DoF = 3; /// @brief Degrees of freedom
  static constexpr int N   = 3; /// @brief Dimension of transformation matrix
};

template <typename _Derived>
struct ManifoldProperties<R2SO2TangentBase<_Derived>>
{
  static constexpr int Dim = 2; /// @brief Space dimension
  static constexpr int DoF = 3; /// @brief Degrees of freedom
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_R2SO2_PROPERTIES_H_ */
