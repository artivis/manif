#ifndef _MANIF_MANIF_CERES_CONSTANTS_H_
#define _MANIF_MANIF_CERES_CONSTANTS_H_

#include "manif/constants.h"

#include <ceres/jet.h>

namespace manif {

/// @brief Specialize Constant traits
/// for the ceres::Jet type
template <typename _Scalar, int N>
struct Constants<ceres::Jet<_Scalar, N>>
{
  static const ceres::Jet<_Scalar, N> eps;
  static const ceres::Jet<_Scalar, N> eps_s;
//  static const Scalar eps_sqrt;

//  static const Scalar to_rad; // pi / 180
//  static const Scalar to_deg; // 180 / pi
};

template <typename _Scalar, int N>
const ceres::Jet<_Scalar, N>
Constants<ceres::Jet<_Scalar, N>>::eps = ceres::Jet<_Scalar, N>(1e-10);

template <typename _Scalar, int N>
const ceres::Jet<_Scalar, N>
Constants<ceres::Jet<_Scalar, N>>::eps_s = ceres::Jet<_Scalar, N>(1e-15);

//template <typename _Scalar, int N>
//const ceres::Jet<_Scalar, N>
//Constants<ceres::Jet<_Scalar, N>>::eps_sqrt = Constants<_Scalar>::eps_sqrt;

//template <typename _Scalar, int N>
//const ceres::Jet<_Scalar, N>
//Constants<ceres::Jet<_Scalar, N>>::to_deg = Constants<_Scalar>::to_deg;

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_CONSTANTS_H_ */
