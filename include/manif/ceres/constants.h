#ifndef _MANIF_MANIF_CERES_CONSTANTS_H_
#define _MANIF_MANIF_CERES_CONSTANTS_H_

#include "manif/constants.h"

#include <ceres/jet.h>

namespace manif {

/// @brief Specialize Constants traits
/// for the ceres::Jet type
template <typename _Scalar, int N>
struct Constants<ceres::Jet<_Scalar, N>>
{
  static const ceres::Jet<_Scalar, N> eps;
  static const ceres::Jet<_Scalar, N> eps_s;
};

template <typename _Scalar, int N>
const ceres::Jet<_Scalar, N>
Constants<ceres::Jet<_Scalar, N>>::eps = ceres::Jet<_Scalar, N>(1e-10);

template <typename _Scalar, int N>
const ceres::Jet<_Scalar, N>
Constants<ceres::Jet<_Scalar, N>>::eps_s = ceres::Jet<_Scalar, N>(1e-15);

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_CONSTANTS_H_ */
