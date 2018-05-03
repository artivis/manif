#ifndef _MANIF_MANIF_SE3_PROPERTIES_H_
#define _MANIF_MANIF_SE3_PROPERTIES_H_

#include "manif/impl/traits.h"

namespace manif
{

// Forward declaration
template <typename _Derived> struct SE3Base;
template <typename _Derived> struct SE3TangentBase;

namespace internal
{

// traits specialization

template <>
template <typename _Derived>
struct ManifoldProperties<SE3Base<_Derived>>
{
  static constexpr int Dim = 3; /// @brief Space dimension
  static constexpr int DoF = 6; /// @brief Degrees of freedom
  static constexpr int N   = 3; /// @brief Dimension of transformation matrix
};

template <>
template <typename _Derived>
struct ManifoldProperties<SE3TangentBase<_Derived>>
{
  static constexpr int Dim = 3; /// @brief Space dimension
  static constexpr int DoF = 6; /// @brief Degrees of freedom
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SE3_PROPERTIES_H_ */
